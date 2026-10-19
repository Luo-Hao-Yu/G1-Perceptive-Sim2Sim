"""Named-joint 23DoF sim2sim; SDK motor indices are never MuJoCo indices."""
import hashlib
import math
import time

import mujoco
import numpy as np

from g1_23dof import config as cfg
from mujoco_env import MujocoEnv
from onnx_inference import OnnxPolicyInference
from sim2sim import DepthImagePipeline, RingBuffer, quat_apply_inverse


def validate_model(model):
    if hashlib.sha256(cfg.URDF_FILE.read_bytes()).hexdigest() != cfg.METADATA["urdf_sha256"]:
        raise ValueError("URDF hash differs from training metadata")
    if (model.nq, model.nv, model.nu) != (30, 29, 23):
        raise ValueError(f"Expected nq/nv/nu=30/29/23; got {model.nq}/{model.nv}/{model.nu}")
    active = {model.joint(i).name for i in range(model.njnt)
              if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE}
    for names in (cfg.ACTION_JOINT_NAMES, cfg.OBSERVATION_JOINT_NAMES):
        if len(names) != 23 or len(set(names)) != 23 or set(names) != active:
            raise ValueError("Policy/model joint names disagree")
    for name in ("OBS_POSITION_OFFSET", "ACTION_OFFSET", "ACTION_SCALES", "STIFFNESS", "DAMPING", "TORQUE_LIMITS"):
        value = np.asarray(getattr(cfg, name))
        if value.shape != (23,) or not np.isfinite(value).all():
            raise ValueError(f"Invalid {name}")
    if not math.isclose(cfg.SIM_DT * cfg.DECIMATION, cfg.POLICY_DT):
        raise ValueError("Simulation decimation does not match policy_dt_s")
    for name, dim in zip(cfg.OBS_NAMES, cfg.OBS_DIMS):
        if cfg.METADATA["observation_segments"][name] != [dim * cfg.HISTORY_LENGTH]:
            raise ValueError(f"Unsupported history format: {name}")
    h, w = cfg.RESIZED_DEPTH_IMAGE_SHAPE
    top, bottom, left, right = cfg.OBS_DEPTH_IMAGE_CROP_REGION
    if cfg.DEPTH_INPUT_SHAPE != (1, cfg.HISTORY_LENGTH, h-top-bottom, w-left-right):
        raise ValueError("Depth preprocessing shape disagrees with metadata")
    model.body(cfg.ROOT_BODY)
    model.camera(cfg.DEPTH_CAMERA_NAME)
    for name, limit in zip(cfg.ACTION_JOINT_NAMES, cfg.TORQUE_LIMITS):
        motor = model.actuator(name)
        if int(model.actuator_trnid[motor.id, 0]) != model.joint(name).id:
            raise ValueError(f"Motor transmission mismatch: {name}")
        if not np.allclose(model.actuator_ctrlrange[motor.id], [-limit, limit]):
            raise ValueError(f"Motor effort limit mismatch: {name}")


def _interface(network, expected_input, expected_output):
    if len(network.input_infos) != 1 or len(network.output_infos) != 1:
        raise ValueError(f"Expected a single input/output: {network.onnx_file}")
    name, (shape, dtype) = next(iter(network.input_infos.items()))
    output_shape, output_dtype = next(iter(network.output_infos.values()))
    for actual, expected in ((shape, expected_input), (output_shape, expected_output)):
        if len(actual) != len(expected) or any(isinstance(a, int) and a != e for a, e in zip(actual, expected)):
            raise ValueError(f"Unexpected ONNX shape {actual}, expected {expected}: {network.onnx_file}")
    if dtype != np.float32 or output_dtype != np.float32:
        raise ValueError("Expected float32 ONNX input and output")
    return name


class Policy:
    def __init__(self):
        self.actor = OnnxPolicyInference(str(cfg.ACTOR_FILE), settings=cfg)
        self.encoder = OnnxPolicyInference(str(cfg.DEPTH_ENCODER_FILE), settings=cfg)
        self.actor_input = _interface(self.actor, (1, cfg.ACTOR_INPUT_DIM), (1, 23))
        self.encoder_input = _interface(self.encoder, cfg.DEPTH_INPUT_SHAPE, (1, cfg.DEPTH_FEATURE_DIM))

    def __call__(self, history, depth):
        features = self.encoder({self.encoder_input: depth})
        if features.shape != (1, cfg.DEPTH_FEATURE_DIM) or not np.isfinite(features).all():
            raise ValueError("Depth encoder returned invalid features")
        obs = np.concatenate([*history, features.ravel()]).astype(np.float32)[None]
        action = self.actor({self.actor_input: obs})
        if action.shape != (1, 23) or not np.isfinite(action).all():
            raise ValueError("Actor must return 23 finite actions")
        return action[0]


class Simulation:
    def __init__(self, *, scene="flat", preview=False, headless=False, control="keyboard", gamepad_id=None):
        # Validate the export before starting native graphics threads.
        self.policy = None if preview else Policy()
        self.preview = preview
        self.commands = np.zeros(3, dtype=np.float32)
        self.last_action = np.zeros(23, dtype=np.float32)
        self._reset_requested = False
        self.gamepad = None
        if control == "gamepad":
            from gamepad_control import GamepadController
            self.gamepad = GamepadController(gamepad_id, settings=cfg)
        scene_file = cfg.HERE / f"scene_{scene}.xml"
        validate_model(mujoco.MjModel.from_xml_path(str(scene_file)))
        self.env = MujocoEnv(
            str(scene_file), sim_dt=cfg.SIM_DT, decimation=cfg.DECIMATION,
            depth_camera_name=cfg.DEPTH_CAMERA_NAME, depth_image_shape=cfg.DEPTH_IMAGE_SHAPE,
            use_secondray_imu=False, viewer_enabled=not headless, depth_enabled=not preview,
            viewer_lookat=cfg.VIEWER_LOOKAT, viewer_distance=cfg.VIEWER_DISTANCE,
            viewer_azimuth=cfg.VIEWER_AZIMUTH, viewer_elevation=cfg.VIEWER_ELEVATION,
            key_callback=self.key_callback,
        )
        model = self.env.model
        self.obs_qpos = np.array([model.jnt_qposadr[model.joint(n).id] for n in cfg.OBSERVATION_JOINT_NAMES])
        self.obs_qvel = np.array([model.jnt_dofadr[model.joint(n).id] for n in cfg.OBSERVATION_JOINT_NAMES])
        self.action_qpos = np.array([model.jnt_qposadr[model.joint(n).id] for n in cfg.ACTION_JOINT_NAMES])
        self.action_qvel = np.array([model.jnt_dofadr[model.joint(n).id] for n in cfg.ACTION_JOINT_NAMES])
        self.action_motors = np.array([model.actuator(n).id for n in cfg.ACTION_JOINT_NAMES])
        self.root_id = model.body(cfg.ROOT_BODY).id
        self.offset = np.array(cfg.ACTION_OFFSET)
        self.scale = np.array(cfg.ACTION_SCALES)
        self.kp, self.kd, self.effort = map(np.array, (cfg.STIFFNESS, cfg.DAMPING, cfg.TORQUE_LIMITS))
        self.history = [RingBuffer((dim,), cfg.HISTORY_LENGTH) for dim in cfg.OBS_DIMS]
        self.depth = DepthImagePipeline(
            shape=cfg.RESIZED_DEPTH_IMAGE_SHAPE, length=cfg.DEPTH_HISTORY_LENGTH,
            crop_region=cfg.OBS_DEPTH_IMAGE_CROP_REGION, near_far_clip=cfg.DEPTH_NEAR_FAR_CLIP,
            nb_frames=cfg.DEPTH_INPUT_SHAPE[1], delay_ranges=cfg.DEPTH_DELAY_RANGE,
        )
        self.reset()

    def reset(self):
        data = self.env.model_data
        mujoco.mj_resetData(self.env.model, data)
        data.qpos[self.action_qpos] = self.offset
        self.env.forward()
        self.commands[:] = 0
        self.last_action[:] = 0
        self._reset_requested = False
        if self.gamepad:
            self.gamepad.stop()
        self.depth.reset()
        # Same startup convention as the existing runner: valid stationary
        # proprioception history, zero depth history gradually filled by images.
        for buffer, value in zip(self.history, self.proprioception()):
            buffer.reset()
            for _ in range(cfg.HISTORY_LENGTH):
                buffer.append(value)

    def proprioception(self):
        data = self.env.model_data
        # Root body is pelvis per metadata. No torso IMU or waist sign flip.
        angular_velocity = data.sensor("imu_gyro").data.astype(np.float32) * cfg.ANGULAR_VELOCITY_SCALE
        gravity = quat_apply_inverse(data.xquat[self.root_id], np.array([0., 0., -1.])).astype(np.float32)
        pos = (data.qpos[self.obs_qpos] - cfg.OBS_POSITION_OFFSET).astype(np.float32)
        vel = (data.qvel[self.obs_qvel] * cfg.JOINT_VELOCITY_SCALE).astype(np.float32)
        return angular_velocity, gravity, self.commands.copy(), pos, vel, self.last_action.copy()

    def key_callback(self, code):
        # Callback runs on viewer thread; reset on the simulation thread.
        if code == 82:  # R
            self._reset_requested = True
            return
        if code in (32, 325):  # space, numpad 5
            self.commands[:] = 0
            if self.gamepad:
                self.gamepad.stop()
        elif self.gamepad is None:
            if code in (265, 328):
                self.commands[0] += cfg.COMMAND_STEP[0]
            elif code in (264, 322):
                self.commands[0] -= cfg.COMMAND_STEP[0]
            elif code in (263, 324):
                self.commands[2] += cfg.COMMAND_STEP[2]
            elif code in (262, 326):
                self.commands[2] -= cfg.COMMAND_STEP[2]
        for i, name in enumerate(("lin_vel_x", "lin_vel_y", "ang_vel_z")):
            self.commands[i] = np.clip(self.commands[i], *cfg.COMMAND_RANGES[name])

    def step(self):
        if self._reset_requested:
            self.reset()
        if self.gamepad:
            command = self.gamepad.poll()
            if command.reset:
                self.reset()
            else:
                self.commands[:] = command.forward, 0, command.yaw
        if not self.preview:
            for buffer, value in zip(self.history, self.proprioception()):
                buffer.append(value)
            self.depth.append(self.env.render_depth_image())
            self.last_action[:] = self.policy([b.get_history().ravel() for b in self.history], self.depth.get_depth_obs())
        target = self.offset + self.scale * self.last_action
        data = self.env.model_data
        for _ in range(cfg.DECIMATION):
            tau = self.kp * (target - data.qpos[self.action_qpos]) - self.kd * data.qvel[self.action_qvel]
            data.ctrl[self.action_motors] = np.clip(tau, -self.effort, self.effort)
            self.env.physical_step()
            for warning in (mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQACC):
                if data.warning[warning].number:
                    raise FloatingPointError(f"MuJoCo reported {warning.name}; simulation is unstable")
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise FloatingPointError("Non-finite MuJoCo state")
        if self.env.viewer:
            self.env.viewer.sync()

    def run(self, seconds=None, profile=False):
        print(f"G1 23DoF | {'PD preview' if self.preview else 'actor 752 -> 23, depth 8x18x32 -> 128'} | 50 Hz control")
        print("Keyboard: up/down speed, left/right yaw, space stop, R reset. Xbox: left stick forward, right stick yaw, B stop, Y reset.")
        print("Depth preprocessing uses reference defaults; metadata does not specify camera FOV, clipping, or frame sampling.")
        count = 0
        steps = None if seconds is None else math.ceil(seconds / cfg.POLICY_DT)
        report_start = time.perf_counter()
        times = []
        min_height = float(self.env.model_data.qpos[2])
        max_tilt = 0.
        try:
            while (steps is None or count < steps) and (self.env.viewer is None or self.env.viewer.is_running()):
                start = time.perf_counter()
                self.step()
                elapsed = time.perf_counter() - start
                count += 1
                data = self.env.model_data
                min_height = min(min_height, float(data.qpos[2]))
                max_tilt = max(max_tilt, math.degrees(math.acos(np.clip(data.xmat[self.root_id].reshape(3, 3)[2, 2], -1, 1))))
                times.append(elapsed * 1000)
                if self.env.viewer is not None:
                    time.sleep(max(0, cfg.POLICY_DT - elapsed))
                wall = time.perf_counter() - report_start
                if profile and wall >= 2:
                    print(f"Profile: control={len(times)/wall:.1f} Hz, work={np.mean(times):.2f} ms, pelvis_z={data.qpos[2]:.3f} m, max_tilt={max_tilt:.1f} deg", flush=True)
                    times.clear()
                    report_start = time.perf_counter()
        finally:
            print(f"Finished {count * cfg.POLICY_DT:.2f} simulated seconds; pelvis={self.env.model_data.qpos[:3].round(3)}, min_z={min_height:.3f} m, max_tilt={max_tilt:.1f} deg")
            self.env.close()
