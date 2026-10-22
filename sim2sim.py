import time
import numpy as np

from collections import deque
from typing import Tuple, List, Literal

import os
import sys
import cv2

try:
    import glfw
except Exception:
    glfw = None

sys.path.append(os.path.dirname(__file__))

from mujoco_env import MujocoEnv
from config import *
from onnx_inference import OnnxPolicyInference as Inference
from heading_control import HeadingHold, heading_from_matrix

def key(name: str, fallback: int) -> int:
    if glfw is None:
        return fallback
    return getattr(glfw, f"KEY_{name}", fallback)

def quat_apply_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:

    shape = vec.shape

    quat = quat.reshape(-1, 4)
    vec = vec.reshape(-1, 3)

    xyz = quat[:, 1:]
    t = np.cross(xyz, vec, axis=-1) * 2
    return (vec - quat[:, 0:1] * t + np.cross(xyz, t, axis=-1)).reshape(shape)

class RingBuffer:
    def __init__(self, shape: Tuple[int, int], length: int = 8):
        self.shape = shape
        self.length = length
        self.que = deque(maxlen=self.length)
        self.reset()

    def reset(self):
        self.que.clear()
        for _ in range(self.length):
            self.que.append(np.zeros(self.shape, dtype=np.float32))

    def append(self, data: np.ndarray):
        if self.shape != data.shape:
            raise ValueError(f"Data size mismatch expected {self.shape} while get {data.shape}")
        self.que.append(data.copy())

    def get_history(self):
        return np.array(self.que).copy()

class DepthImagePipeline:
    def __init__(
        self,
        shape: Tuple[int, int] = (36, 64),
        length: int = 60,
        crop_region: Tuple[int, int, int, int] = tuple([0, 0, 0, 0]),
        near_far_clip: Tuple[float, float] = tuple([0, 2.5]),
        nb_frames: int = 8,
        delay_ranges: Tuple[int, int] = (0, 1),
    ):
        self.shape = shape
        self.length = length
        self.que = deque(maxlen=length)
        self.crop_region = crop_region
        self.near_clip, self.far_clip = near_far_clip
        self.nb_frames = nb_frames
        self.delay_ranges = delay_ranges

        down_sample_factor = int(self.length / 60 * 5)
        self.indices = np.linspace(-1 - down_sample_factor * (self.nb_frames - 1), -1, self.nb_frames).astype(np.int32)
        self.delay = 0
        self.reset()

    def reset(self):
        self.que.clear()
        h, w = self.shape
        y1, y2, x1, x2 = self.crop_region
        dh, dw = h - y1 - y2, w - x1 - x2
        self._history_initialized = False
        for _ in range(self.length):
            self.que.append(np.zeros((dh, dw), dtype=np.float32))
        self.resample_delay()

    def show_image(self, image: np.ndarray):
        if image.dtype == np.float32:
            image = (image * 255).astype(np.uint8)
        cv2.imshow("current_depth_image", image)
        cv2.waitKey(1)

    def show_normalize_image(self, image: np.ndarray):
        if self.far_clip <= self.near_clip:
            raise ValueError("far_clip must be greater than near_clip")

        vis_image = image.astype(np.float32, copy=True)
        vis_image = np.nan_to_num(
            vis_image,
            nan=self.far_clip,
            posinf=self.far_clip,
            neginf=self.near_clip,
        )
        np.clip(
            vis_image,
            a_min=self.near_clip,
            a_max=self.far_clip,
            out=vis_image,
        )
        vis_image = (vis_image - self.near_clip) / (self.far_clip - self.near_clip)
        vis_image = (vis_image * 255).astype(np.uint8)

        cv2.imshow("normalized_depth_image", vis_image)
        cv2.waitKey(1)

    def append(self, image: np.ndarray):

        h, w = self.shape

        image = cv2.resize(image, (w,h), interpolation=cv2.INTER_NEAREST)

        y1, y2, x1, x2 = self.crop_region

        image = image[y1 : h - y2, x1 : w - x2]

        mask = (image < 0.2).astype(np.uint8)

        image = cv2.inpaint(image, mask, 3, cv2.INPAINT_NS)

        image = cv2.GaussianBlur(image, (3, 3), sigmaX=1, sigmaY=1)

        np.clip(image, a_min=self.near_clip, a_max=self.far_clip, out=image)

        image = (image - self.near_clip) / (self.far_clip - self.near_clip)

        image = image.astype(np.float32)
        if not self._history_initialized:
            # Match Isaac's AsyncCircularBuffer first push after startup/reset.
            # Zero-filled history represents fictitious near-depth frames to the policy.
            self.que.clear()
            self.que.extend(image.copy() for _ in range(self.length))
            self._history_initialized = True
        else:
            self.que.append(image)

        self.resample_delay()

    def resample_delay(self):

        min_delay, max_delay = self.delay_ranges
        if max_delay <= min_delay:
            self.delay = min_delay
        else:
            self.delay = np.random.randint(min_delay, max_delay + 1)

    def get_depth_obs(self):
        obs = []
        for index in self.indices:

            delayed_index = index - self.delay
            obs.append(self.que[delayed_index])
        obs = np.array(obs, dtype=np.float32)[None, ...]
        return obs

class Sim2simInstance:
    def __init__(self, sim2sim_type: Literal["stand", "parkour"], control: Literal["keyboard", "gamepad"] = "keyboard", gamepad_id: int | None = None):
        if control not in ("keyboard", "gamepad"):
            raise ValueError(f"Unsupported control mode: {control}")
        if control == "gamepad" and sim2sim_type != "parkour":
            raise ValueError("Gamepad velocity control is only available for the parkour task")
        self.sim2sim_type = sim2sim_type
        self.control_mode = control
        self.gamepad = None
        self.heading_hold = None
        self._reset_requested = False
        if control == "gamepad":
            from gamepad_control import GamepadController
            self.gamepad = GamepadController(gamepad_id)
        self.velocity_commands = self._default_velocity_commands()

        policy_files = (
            (STAND_POLICY_FILE, STAND_DEPTH_ENCODER_FILE)
            if sim2sim_type == "stand"
            else (PARKOUR_POLICY_FILE, PARKOUR_DEPTH_ENCODER_FILE)
        )
        missing_files = [path for path in policy_files if not os.path.isfile(path)]
        if missing_files:
            raise FileNotFoundError(
                "Missing ONNX policies. Place the matching actor and depth encoder "
                "under policy/ or update config.py:\n" + "\n".join(missing_files)
            )

        print(f"Scene XML: {os.path.realpath(MJCF_FILE)}", flush=True)
        self.sim_env = MujocoEnv(
            mjcf_file=MJCF_FILE,
            sim_dt=SIM_DT,
            decimation=DECIMATION,
            depth_camera_name=DEPTH_CAMERA_NAME,
            depth_image_shape=DEPTH_IMAGE_SHAPE,
            use_secondray_imu=USE_SECONDARY_IMU,
            viewer_lookat=VIEWER_LOOKAT,
            viewer_distance=VIEWER_DISTANCE,
            viewer_azimuth=VIEWER_AZIMUTH,
            viewer_elevation=VIEWER_ELEVATION,
            key_callback=self._key_callback,
        )

        policy_joint_name = POLICY_JOINT_NAMES
        robot_joint_name = UNITREE_G1_29DOF_SDK_JOINT_NAMES

        self.robot_to_policy = self.compute_joint_indices(robot_joint_name, policy_joint_name)
        self.policy_to_robot = self.compute_joint_indices(policy_joint_name, robot_joint_name)

        self.obs_ring_buffer: List[RingBuffer] = []
        history_length = HISTORY_LENGTH
        obs_dims = OBS_DIMS
        for dim in obs_dims:
            self.obs_ring_buffer.append(RingBuffer(shape=(dim,), length=history_length))

        self.resized_depth_image_shape = RESIZED_DEPTH_IMAGE_SHAPE
        y1, y2, x1, x2 = OBS_DEPTH_IMAGE_CROP_REGION
        self.obs_depth_image_shape = (
            self.resized_depth_image_shape[0] - y1 - y2,
            self.resized_depth_image_shape[1] - x1 - x2,
        )
        self.depth_pipeline = DepthImagePipeline(
            shape=self.resized_depth_image_shape,
            crop_region=OBS_DEPTH_IMAGE_CROP_REGION,
        )

        self.action_scale = np.array(ACTION_SCALES, dtype=np.float32)[self.policy_to_robot]
        self.default_joint_pos = np.array(DEFAULT_JOINT_POS, dtype=np.float32)[self.policy_to_robot]
        self.joint_signs = np.array(JOINT_SIGNS, dtype=np.float32)[self.policy_to_robot]
        self.stiffness = np.array(STIFFNESS)[self.policy_to_robot]
        self.damping = np.array(DAMPING)[self.policy_to_robot]
        self.torque_limit = np.array(TORQUE_LIMITS)[self.policy_to_robot]

        self.last_action = np.zeros((29,), dtype=np.float32)
        self.effective_velocity_commands = np.zeros(3, dtype=np.float32)
        if sim2sim_type == "parkour" and HEADING_HOLD_ENABLED:
            self._heading_body_id = self.sim_env.model.body(HEADING_HOLD_BODY).id
            self.heading_hold = HeadingHold(HEADING_HOLD_GAIN, HEADING_HOLD_MAX_YAW_RATE, COMMAND_RANGES["ang_vel_z"])

        if self.sim2sim_type == "stand":
            self.actor = Inference(STAND_POLICY_FILE)
            self.depth_encoder = Inference(STAND_DEPTH_ENCODER_FILE)
        else:
            self.actor = Inference(PARKOUR_POLICY_FILE)
            self.depth_encoder = Inference(PARKOUR_DEPTH_ENCODER_FILE)
            self._print_keyboard_help()
        self.reset()

    def _default_velocity_commands(self) -> np.ndarray:
        return np.array((0.0, 0.0, 0.0), dtype=np.float32)

    def _print_keyboard_help(self):
        if self.sim2sim_type != "parkour":
            return
        if self.gamepad is not None:
            print("Gamepad: left stick up = forward, right stick left/right = yaw; B = stop, Y = reset. Release sticks to stop. Center sticks after connection/stop/reset.")
            print("Keyboard in gamepad mode: Space/KP5 stop, R reset. Use --control keyboard for arrow-key velocity control.")
        else:
            print("Keyboard: Up/Down or KP8/KP2 vx, Left/Right or KP4/KP6 yaw, Space/KP5 stop, R reset.")
        if self.heading_hold is not None:
            print("Heading hold: zero manual yaw holds the reset/last turning direction; manual yaw input takes priority.")
        print(f"Initial velocity command: {self.velocity_commands.tolist()}")

    def reset(self):
        self.velocity_commands[:] = 0.0
        if self.gamepad is not None:
            self.gamepad.stop()
        self.last_action[:] = 0.0
        for buffer in self.obs_ring_buffer:
            buffer.reset()
        self.depth_pipeline.reset()
        initial_joint_pos = self.default_joint_pos * self.joint_signs
        self.sim_env.reset(initial_joint_pos)
        self._reset_requested = False
        if self.heading_hold is not None:
            self.heading_hold.reset(self._heading())
        # Give the policy valid stationary observations from its first step.
        # In particular, gravity history should not be eight zero vectors.
        for buffer, observation in zip(self.obs_ring_buffer, self._proprioception()):
            for _ in range(buffer.length):
                buffer.append(observation)
        message = "Reset simulation state."
        if self.heading_hold is not None:
            message += f" Heading target: {np.degrees(self.heading_hold.target):+.2f} deg."
        print(message)

    def _key_callback(self, keycode: int):
        if keycode == key("R", 82):
            # GLFW invokes this on the viewer thread. Apply the reset on the
            # control thread so the model is not reset in the middle of a step.
            self._reset_requested = True
            return

        if self.sim2sim_type != "parkour":
            return

        if self.gamepad is not None:
            if keycode in (key("SPACE", 32), key("KP_5", KEY_KP_5)):
                self.gamepad.stop()
                self.velocity_commands[:] = 0.0
                print("Gamepad commands stopped; center sticks to resume.")
            return

        vx_step, _, yaw_step = COMMAND_STEP
        handled = True
        if keycode in (key("UP", 265), key("KP_8", KEY_KP_8)):
            self.velocity_commands[0] += vx_step
        elif keycode in (key("DOWN", 264), key("KP_2", KEY_KP_2)):
            self.velocity_commands[0] -= vx_step
        elif keycode in (key("LEFT", 263), key("KP_4", KEY_KP_4)):
            self.velocity_commands[2] += yaw_step
        elif keycode in (key("RIGHT", 262), key("KP_6", KEY_KP_6)):
            self.velocity_commands[2] -= yaw_step
        elif keycode in (key("SPACE", 32), key("KP_5", KEY_KP_5)):
            self.velocity_commands[:] = 0.0
        else:
            handled = False

        if not handled:
            return

        self.velocity_commands[0] = np.clip(self.velocity_commands[0], *COMMAND_RANGES["lin_vel_x"])
        self.velocity_commands[1] = np.clip(self.velocity_commands[1], *COMMAND_RANGES["lin_vel_y"])
        self.velocity_commands[2] = np.clip(self.velocity_commands[2], *COMMAND_RANGES["ang_vel_z"])
        print(
            "Command "
            f"vx={self.velocity_commands[0]: .2f}, "
            f"vy={self.velocity_commands[1]: .2f}, "
            f"yaw={self.velocity_commands[2]: .2f}"
        )

    def _heading(self) -> float:
        matrix = self.sim_env.model_data.xmat[self._heading_body_id].reshape(3, 3)
        return heading_from_matrix(matrix)

    def _proprioception(self):

        imu_offset = self.sim_env.imu_offset
        sensordata = self.sim_env.model_data.sensordata.astype(np.float32)

        base_angle_vel = sensordata[imu_offset + 4 : imu_offset + 7] * 0.25

        root_quat = sensordata[imu_offset : imu_offset + 4]
        projected_gravity = quat_apply_inverse(root_quat, np.array([0, 0, -1], dtype=np.float32))

        if self.sim2sim_type == "stand":
            velocity_commands = np.array((0, 0.0, 0.0), dtype=np.float32)
        else:
            velocity_commands = self.velocity_commands.copy()
            if self.heading_hold is not None:
                velocity_commands[2] = self.heading_hold.update(float(velocity_commands[2]), self._heading())
        self.effective_velocity_commands[:] = velocity_commands

        joint_pos_rel = sensordata[:29] * self.joint_signs - self.default_joint_pos

        joint_pos_rel = joint_pos_rel[self.robot_to_policy]

        joint_vel_rel = sensordata[29:58] * self.joint_signs * 0.05
        joint_vel_rel = joint_vel_rel[self.robot_to_policy]
        return (base_angle_vel, projected_gravity, velocity_commands, joint_pos_rel, joint_vel_rel, self.last_action)

    def update_observation(self):
        for buffer, observation in zip(self.obs_ring_buffer, self._proprioception()):
            buffer.append(observation)

        if self.sim2sim_type == "stand":
            image = np.zeros(self.obs_depth_image_shape, dtype=np.float32)
        else:
            image = self.sim_env.depth_render.render()

        self.depth_pipeline.append(image)

    def get_history_obs(self):

        history_obs = []

        for each in self.obs_ring_buffer:
            history_obs.append(each.get_history().ravel())

        history_depth_image = self.depth_pipeline.get_depth_obs()

        depth_image_feat = self.depth_encoder({"input": history_depth_image})
        history_obs.append(depth_image_feat.ravel())
        return np.concatenate(history_obs)

    def compute_joint_indices(self, joint_a: List[str], joint_b: List[str]) -> np.ndarray:

        if joint_b is None:
            return np.arange(len(joint_a), dtype=np.uint32)
        if joint_a is None:
            return np.arange(len(joint_b), dtype=np.uint32)
        indices = []
        for joint in joint_b:
            indices.append(joint_a.index(joint))
        return np.array(indices, dtype=np.uint32)

    def keep_running(self, profile: bool = False):
        decimation = DECIMATION
        eposide_length = 0
        profile_start = time.perf_counter()
        timings = []
        while self.sim_env.viewer.is_running():
            time_start = time.perf_counter()

            if self._reset_requested:
                self.reset()
            if self.gamepad is not None:
                command = self.gamepad.poll()
                if command.reset:
                    self.reset()
                else:
                    self.velocity_commands[:] = (command.forward, 0.0, command.yaw)
            self.update_observation()
            observation_end = time.perf_counter()
            history_obs = self.get_history_obs()[None, ...]
            encoder_end = time.perf_counter()

            raw_action = self.actor({"input": history_obs}).ravel()
            actor_end = time.perf_counter()
            self.last_action = raw_action.copy()

            action = raw_action[self.policy_to_robot] * self.action_scale + self.default_joint_pos
            action = action * self.joint_signs
            for _ in range(decimation):

                tau = (
                    self.stiffness * (action - self.sim_env.model_data.sensordata[:29])
                    - self.damping * self.sim_env.model_data.sensordata[29:58]
                )
                tau = np.clip(tau, a_min=-self.torque_limit, a_max=self.torque_limit)
                self.sim_env.model_data.ctrl[:] = tau
                self.sim_env.physical_step()
            physics_end = time.perf_counter()
            self.sim_env.viewer.sync()
            sync_end = time.perf_counter()
            eposide_length += 1

            if profile:
                depth_ms = self.sim_env.depth_render.last_render_ms if self.sim2sim_type == "parkour" else 0.0
                timings.append((
                    depth_ms,
                    (observation_end - time_start) * 1000 - depth_ms,
                    (encoder_end - observation_end) * 1000,
                    (actor_end - encoder_end) * 1000,
                    (physics_end - actor_end) * 1000,
                    (sync_end - physics_end) * 1000,
                ))

            time_to_sleep = SIM_DT * DECIMATION - (time.perf_counter() - time_start)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

            if profile:
                elapsed = time.perf_counter() - profile_start
                if elapsed >= 2.0:
                    avg = np.mean(timings, axis=0)
                    print(
                        f"Profile: control={len(timings) / elapsed:.1f} Hz "
                        f"(target={1 / (SIM_DT * DECIMATION):g}), "
                        f"depth={avg[0]:.1f} ms, preprocessing={avg[1]:.1f} ms, "
                        f"encoder={avg[2]:.1f} ms, actor={avg[3]:.1f} ms, "
                        f"physics={avg[4]:.1f} ms, viewer_sync={avg[5]:.1f} ms"
                        + (f", heading={np.degrees(self._heading()):+.2f} deg, "
                           f"target={np.degrees(self.heading_hold.target):+.2f} deg, "
                           f"policy_yaw={self.effective_velocity_commands[2]:+.3f} rad/s"
                           if self.heading_hold is not None else ""),
                        flush=True,
                    )
                    timings.clear()
                    profile_start = time.perf_counter()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str, choices=["parkour", "stand"], default="parkour")
    parser.add_argument("--profile", action="store_true", help="Print control rate and stage timings every two seconds")
    parser.add_argument("--control", choices=("keyboard", "gamepad"), default="keyboard", help="Velocity command source (default: keyboard)")
    parser.add_argument("--gamepad-id", type=int, choices=range(16), help="GLFW gamepad ID; default is the first mapped gamepad")
    args = parser.parse_args()
    if args.control == "gamepad" and args.task != "parkour":
        parser.error("--control gamepad requires --task parkour")
    if args.gamepad_id is not None and args.control != "gamepad":
        parser.error("--gamepad-id requires --control gamepad")

    print(f"Python executable: {sys.executable}")
    instance = Sim2simInstance(args.task, control=args.control, gamepad_id=args.gamepad_id)
    print(f"Actor ONNX provider: {instance.actor.provider}")
    print(f"Depth encoder ONNX provider: {instance.depth_encoder.provider}")
    print(f"ONNX threading: intra_op={ONNX_INTRA_OP_THREADS}, inter_op={ONNX_INTER_OP_THREADS}, spinning={ONNX_ALLOW_SPINNING}")
    instance.keep_running(profile=args.profile)
