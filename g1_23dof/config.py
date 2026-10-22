"""23DoF export settings; all joint arrays are in the metadata's named order."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
POLICY_DIR = HERE / "policy" / "exported"
METADATA_FILE = POLICY_DIR / "policy_metadata.json"
METADATA = json.loads(METADATA_FILE.read_text(encoding="utf-8"))
ACTOR_FILE = POLICY_DIR / "actor.onnx"
DEPTH_ENCODER_FILE = POLICY_DIR / "0-depth_encoder.onnx"
URDF_FILE = HERE / "g1_23dof.urdf"
ROOT_BODY = METADATA["root_body"]
OBSERVATION_JOINT_NAMES = METADATA["observation_joint_names"]
ACTION_JOINT_NAMES = METADATA["action_joint_names"]
NUM_ACTIONS = len(ACTION_JOINT_NAMES)
OBS_POSITION_OFFSET = METADATA["observation_joint_pos_offset_rad"]
ACTION_OFFSET = METADATA["action_offset_rad"]
ACTION_SCALES = METADATA["action_scale"]
STIFFNESS = METADATA["stiffness_nm_per_rad"]
DAMPING = METADATA["damping_nm_s_per_rad"]
TORQUE_LIMITS = METADATA["effort_limits_nm"]
ANGULAR_VELOCITY_SCALE = METADATA["observation_base_ang_vel_scale"]
JOINT_VELOCITY_SCALE = METADATA["observation_joint_vel_scale"]
POLICY_DT = METADATA["policy_dt_s"]
SIM_DT = 0.005
DECIMATION = round(POLICY_DT / SIM_DT)
# Same rotor inertia used by this project's reference MuJoCo model; the URDF
# and policy export do not specify actuator armature. Confirm with training.
JOINT_ARMATURE = 0.01
HISTORY_LENGTH = METADATA["observation_segments"]["base_ang_vel"][0] // 3
OBS_NAMES = ("base_ang_vel", "projected_gravity", "velocity_commands", "joint_pos", "joint_vel", "actions")
OBS_DIMS = (3, 3, 3, NUM_ACTIONS, NUM_ACTIONS, NUM_ACTIONS)
DEPTH_INPUT_SHAPE = tuple(METADATA["depth_encoder_input_shape"])
DEPTH_FEATURE_DIM = METADATA["depth_encoder_output_shape"][-1]
ACTOR_INPUT_DIM = sum(OBS_DIMS) * HISTORY_LENGTH + DEPTH_FEATURE_DIM

# Below are simulation/reference defaults, not specified by the export metadata.
SPAWN_HEIGHT = 0.793
DEPTH_CAMERA_NAME = "depth_camera"
DEPTH_FOVY = 58.29
DEPTH_IMAGE_SHAPE = (270, 480)
RESIZED_DEPTH_IMAGE_SHAPE = (36, 64)
OBS_DEPTH_IMAGE_CROP_REGION = (18, 0, 16, 16)
DEPTH_NEAR_FAR_CLIP = (0.0, 2.5)
DEPTH_HISTORY_LENGTH = 60
DEPTH_DELAY_RANGE = (0, 1)
ONNX_INTRA_OP_THREADS = 1
ONNX_INTER_OP_THREADS = 1
ONNX_ALLOW_SPINNING = False
VIEWER_LOOKAT = (2.0, 0.0, 0.5)
VIEWER_DISTANCE = 8.0
VIEWER_AZIMUTH = 0.0
VIEWER_ELEVATION = -35.0
COMMAND_STEP = (0.2, 0.0, 0.5)
COMMAND_RANGES = {"lin_vel_x": (0.0, 0.9), "lin_vel_y": (0.0, 0.0), "ang_vel_z": (-1.0, 1.0)}
# External world-frame feedback, as required by the training command generator.
# off: raw commands; heading: fixed yaw; path: heading plus lateral position.
COMMAND_FEEDBACK_MODE = "path"
HEADING_HOLD_GAIN = 2.0
HEADING_HOLD_MAX_YAW_RATE = 1.0
PATH_LOOKAHEAD = 1.0  # m: larger values make lateral recovery more gradual.
PATH_MAX_HEADING_OFFSET = 0.5235987755982988  # 30 degrees relative to the line.
GAMEPAD_DEADZONE = 0.15
GAMEPAD_FORWARD_AXIS = 1
GAMEPAD_FORWARD_SIGN = -1.0
GAMEPAD_YAW_AXIS = 2
GAMEPAD_YAW_SIGN = -1.0
GAMEPAD_STOP_BUTTON = 1
GAMEPAD_RESET_BUTTON = 3
