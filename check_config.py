"""Check dependencies, MuJoCo assets and policy interfaces without a viewer."""

import argparse
import importlib
from pathlib import Path
import sys

import config


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("stand", "parkour", "all"), default="all")
    parser.add_argument("--render", action="store_true", help="Also render a depth frame (requires OpenGL).")
    args = parser.parse_args()
    errors = []

    for name in ("numpy", "mujoco", "onnxruntime", "cv2", "glfw"):
        try:
            module = importlib.import_module(name)
            print(f"[OK] {name}: {module.__version__}")
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    if errors:
        for error in errors:
            print(f"[FAIL] {error}")
        return 1

    import mujoco
    import numpy as np
    from onnx_inference import OnnxPolicyInference
    from sim2sim import DepthImagePipeline

    try:
        names = config.UNITREE_G1_29DOF_SDK_JOINT_NAMES
        require(len(names) == len(set(names)) == 29, "Expected 29 unique SDK joints")
        require(len(config.POLICY_JOINT_NAMES) == 29 and set(config.POLICY_JOINT_NAMES) == set(names), "Policy joint mapping mismatch")
        for name in ("STIFFNESS", "DAMPING", "DEFAULT_JOINT_POS", "ACTION_SCALES", "TORQUE_LIMITS", "JOINT_SIGNS"):
            values = np.asarray(getattr(config, name))
            require(values.shape == (29,) and np.isfinite(values).all(), f"Invalid {name}")
        require(config.SIM_DT > 0 and isinstance(config.DECIMATION, int) and config.DECIMATION > 0, "Invalid simulation timing")
        require(config.OBS_DIMS == [3, 3, 3, 29, 29, 29] and config.HISTORY_LENGTH > 0, "Invalid observation history")

        model = mujoco.MjModel.from_xml_path(config.MJCF_FILE)
        model.opt.timestep = config.SIM_DT
        require(model.nu == 29 and model.nq == 36, "Expected G1 29DoF model with a free base")
        actuator_joints = [model.joint(int(joint_id)).name for joint_id in model.actuator_trnid[:, 0]]
        require(actuator_joints == names, "Actuator order differs from SDK joint order")
        for offset, sensor_type in ((0, mujoco.mjtSensor.mjSENS_JOINTPOS), (29, mujoco.mjtSensor.mjSENS_JOINTVEL)):
            for index, name in enumerate(names):
                sensor = model.sensor(offset + index)
                require(int(sensor.type[0]) == sensor_type and int(sensor.adr[0]) == offset + index, "Joint sensor layout mismatch")
                require(model.joint(int(sensor.objid[0])).name == name, "Joint sensor order mismatch")
        model.camera(config.DEPTH_CAMERA_NAME)
        imu_name = "secondary_imu_quat" if config.USE_SECONDARY_IMU else "imu_quat"
        imu = model.sensor(imu_name)
        require(int(imu.dim[0]) == 4, "IMU quaternion must contain 4 values")
        gyro = model.sensor("secondary_imu_gyro" if config.USE_SECONDARY_IMU else "imu_gyro")
        require(int(gyro.dim[0]) == 3 and int(gyro.adr[0]) == int(imu.adr[0]) + 4, "IMU gyro layout mismatch")
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        for _ in range(config.DECIMATION):
            mujoco.mj_step(model, data)
        require(np.isfinite(data.qpos).all(), "Non-finite simulation state")
        print(f"[OK] Scene, meshes, 29 joints, sensors, camera; control frequency: {1 / (config.SIM_DT * config.DECIMATION):g} Hz")

        if args.render:
            with mujoco.Renderer(model, *config.DEPTH_IMAGE_SHAPE) as renderer:
                renderer.enable_depth_rendering()
                renderer.update_scene(data, camera=config.DEPTH_CAMERA_NAME)
                depth = renderer.render()
                require(depth.shape == config.DEPTH_IMAGE_SHAPE and np.isfinite(depth).all(), "Invalid depth frame")
                print(f"[OK] Depth rendering: {depth.shape}, range {depth.min():.3f}..{depth.max():.3f} m")
    except Exception as exc:
        errors.append(f"Scene/configuration: {exc}")

    tasks = ("stand", "parkour") if args.task == "all" else (args.task,)
    for task in tasks:
        actor_path = getattr(config, f"{task.upper()}_POLICY_FILE")
        encoder_path = getattr(config, f"{task.upper()}_DEPTH_ENCODER_FILE")
        missing = [path for path in (actor_path, encoder_path) if not Path(path).is_file()]
        if missing:
            errors.extend(f"Missing ONNX: {path}" for path in missing)
            continue
        try:
            pipeline = DepthImagePipeline(shape=config.RESIZED_DEPTH_IMAGE_SHAPE, crop_region=config.OBS_DEPTH_IMAGE_CROP_REGION)
            pipeline.append(np.full(config.DEPTH_IMAGE_SHAPE, 1.0, dtype=np.float32))
            encoder = OnnxPolicyInference(encoder_path)
            features = encoder({"input": pipeline.get_depth_obs()}).ravel()
            require(np.isfinite(features).all(), "Non-finite depth features")
            obs = np.concatenate((np.zeros(sum(config.OBS_DIMS) * config.HISTORY_LENGTH, dtype=np.float32), features))[None, :]
            actor = OnnxPolicyInference(actor_path)
            action = actor({"input": obs}).ravel()
            require(action.shape == (29,) and np.isfinite(action).all(), "Expected 29 finite actions")
            print(f"[OK] {task}: depth {pipeline.get_depth_obs().shape} -> observation {obs.shape} -> action {action.shape}")
        except Exception as exc:
            errors.append(f"{task} ONNX interface: {exc}")

    for error in errors:
        print(f"[FAIL] {error}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
