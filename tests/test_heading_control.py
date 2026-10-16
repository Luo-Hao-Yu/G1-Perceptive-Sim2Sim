import math
import unittest

import mujoco
import numpy as np

import config
from heading_control import HeadingHold, heading_from_matrix
from mujoco_env import MujocoEnv
from sim2sim import DepthImagePipeline, RingBuffer, Sim2simInstance


class HeadingControlTests(unittest.TestCase):
    def controller(self, limits=(-1.0, 1.0)):
        return HeadingHold(2.0, 0.5, limits)

    def test_turns_back_towards_reference_and_limits_correction(self):
        controller = self.controller()
        controller.reset(0)
        self.assertAlmostEqual(controller.update(0, 0.1), -0.2)
        self.assertAlmostEqual(controller.update(0, -0.1), 0.2)
        self.assertAlmostEqual(controller.update(0, 1.0), -0.5)
        controller = self.controller((-0.2, 0.3))
        self.assertAlmostEqual(controller.update(0, 1), -0.2)
        self.assertAlmostEqual(controller.update(0, -1), 0.3)

    def test_uses_shortest_turn_across_angle_wrap(self):
        controller = self.controller()
        controller.reset(math.radians(-179))
        self.assertAlmostEqual(controller.update(0, math.radians(179)), math.radians(4))

    def test_manual_turn_takes_priority_and_release_holds_new_direction(self):
        controller = self.controller()
        self.assertAlmostEqual(controller.update(0.8, 1.2), 0.8)
        self.assertAlmostEqual(controller.target, 1.2)
        self.assertAlmostEqual(controller.update(0, 1.22), -0.04)
        controller.reset(-0.2)
        self.assertAlmostEqual(controller.update(0, -0.2), 0)

    def test_heading_uses_forward_axis_in_world_coordinates(self):
        yaw = math.radians(37)
        matrix = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                           [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        self.assertAlmostEqual(heading_from_matrix(matrix), yaw)


class ResetObservationTests(unittest.TestCase):
    def setUp(self):
        env = MujocoEnv.__new__(MujocoEnv)
        env.model = mujoco.MjModel.from_xml_path(config.MJCF_FILE)
        env.model_data = mujoco.MjData(env.model)
        env.imu_offset = int(env.model.sensor('secondary_imu_quat').adr[0])
        instance = Sim2simInstance.__new__(Sim2simInstance)
        instance.sim_env = env
        instance.sim2sim_type = 'parkour'
        instance.gamepad = None
        instance._heading_body_id = env.model.body(config.HEADING_HOLD_BODY).id
        instance.heading_hold = HeadingHold(2, 0.5, (-1, 1))
        instance.velocity_commands = np.zeros(3, dtype=np.float32)
        instance.effective_velocity_commands = np.zeros(3, dtype=np.float32)
        instance.last_action = np.zeros(29, dtype=np.float32)
        instance.robot_to_policy = instance.compute_joint_indices(config.UNITREE_G1_29DOF_SDK_JOINT_NAMES, config.POLICY_JOINT_NAMES)
        indices = instance.compute_joint_indices(config.POLICY_JOINT_NAMES, config.UNITREE_G1_29DOF_SDK_JOINT_NAMES)
        instance.default_joint_pos = np.array(config.DEFAULT_JOINT_POS, dtype=np.float32)[indices]
        instance.joint_signs = np.array(config.JOINT_SIGNS, dtype=np.float32)[indices]
        instance.obs_ring_buffer = [RingBuffer((dim,), config.HISTORY_LENGTH) for dim in config.OBS_DIMS]
        instance.depth_pipeline = DepthImagePipeline(shape=config.RESIZED_DEPTH_IMAGE_SHAPE, crop_region=config.OBS_DEPTH_IMAGE_CROP_REGION)
        self.instance = instance

    def test_reset_restores_heading_and_fills_valid_history(self):
        instance = self.instance
        instance.sim_env.model_data.qpos[3:7] = [math.cos(0.3), 0, 0, math.sin(0.3)]
        instance.sim_env.model_data.qvel[:] = 2
        instance.velocity_commands[:] = [0.4, 0, 0.5]
        instance.last_action[:] = 1
        instance.heading_hold.reset(0.6)
        instance.reset()
        np.testing.assert_allclose(instance.sim_env.model_data.qpos[3:7], [1, 0, 0, 0])
        np.testing.assert_allclose(instance.sim_env.model_data.qvel, 0)
        self.assertAlmostEqual(instance.heading_hold.target, 0)
        np.testing.assert_allclose(instance.velocity_commands, 0)
        np.testing.assert_allclose(instance.effective_velocity_commands, 0)
        np.testing.assert_allclose(instance.obs_ring_buffer[1].get_history(), np.tile([0, 0, -1], (config.HISTORY_LENGTH, 1)))
        np.testing.assert_allclose(instance.obs_ring_buffer[3].get_history(), 0, atol=1e-7)
        np.testing.assert_allclose(instance.obs_ring_buffer[5].get_history(), 0)

    def test_keyboard_reset_is_queued_for_control_thread(self):
        instance = self.instance
        instance.reset()
        instance.sim_env.model_data.qpos[0] = 0.4
        instance._key_callback(82)
        self.assertTrue(instance._reset_requested)
        self.assertEqual(instance.sim_env.model_data.qpos[0], 0.4)
        instance.reset()
        self.assertFalse(instance._reset_requested)
        self.assertEqual(instance.sim_env.model_data.qpos[0], 0)


if __name__ == '__main__':
    unittest.main()
