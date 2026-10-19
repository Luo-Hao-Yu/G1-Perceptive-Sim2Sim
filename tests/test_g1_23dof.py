"""Physical model provenance and policy/MuJoCo order boundary checks."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

import config as root_config
from g1_23dof import config as cfg
from g1_23dof.check_config import check_urdf
from g1_23dof.runtime import Policy, Simulation, validate_model


class G1ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(cfg.HERE / "scene_flat.xml"))

    def test_urdf_and_training_metadata_match_compiled_model(self):
        validate_model(self.model)
        check_urdf(self.model)
        self.assertAlmostEqual(self.model.body("torso_link").mass[0], 8.562)
        for side in ("left", "right"):
            for axis in ("pitch", "roll"):
                j = self.model.joint(f"{side}_ankle_{axis}_joint")
                np.testing.assert_array_equal(self.model.jnt_actfrcrange[j.id], [-35, 35])

    def test_independent_scenes_and_plank_gaps(self):
        scene = cfg.HERE / "scene_rough.xml"
        model = mujoco.MjModel.from_xml_path(str(scene))
        validate_model(model)
        root = ET.parse(scene).getroot()
        self.assertEqual(root.find("include").get("file"), "g1_23dof.xml")
        planks = [g for g in root.findall("worldbody/geom") if g.get("name", "").startswith("wooden_planks_board_")]
        self.assertEqual(len(planks), 16)
        for a, b in zip(planks, planks[1:]):
            edge_a = float(a.get("pos").split()[0]) + float(a.get("size").split()[0])
            edge_b = float(b.get("pos").split()[0]) - float(b.get("size").split()[0])
            self.assertGreaterEqual(edge_b-edge_a, .10-1e-4)
            self.assertLessEqual(edge_b-edge_a, .15+1e-4)

    def test_reference_29dof_configuration_is_not_replaced(self):
        self.assertEqual(len(root_config.POLICY_JOINT_NAMES), 29)
        self.assertEqual(cfg.NUM_ACTIONS, 23)
        self.assertEqual(cfg.ROOT_BODY, "pelvis")


class G1MappingTests(unittest.TestCase):
    def setUp(self):
        self.sim = Simulation(preview=True, headless=True)
        self.addCleanup(self.sim.env.close)

    def test_named_joint_observation_order_and_scales(self):
        s = self.sim
        data = s.env.model_data
        for i in range(s.env.model.njnt):
            if s.env.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
                name = s.env.model.joint(i).name
                index = cfg.OBSERVATION_JOINT_NAMES.index(name)
                data.qpos[s.env.model.jnt_qposadr[i]] = cfg.OBS_POSITION_OFFSET[index] + .001 * (index+1)
                data.qvel[s.env.model.jnt_dofadr[i]] = index + 1
        s.env.forward()
        obs = s.proprioception()
        np.testing.assert_allclose(obs[3], .001*np.arange(1,24), atol=1e-7)
        np.testing.assert_allclose(obs[4], cfg.JOINT_VELOCITY_SCALE*np.arange(1,24), atol=1e-7)

    def test_pd_action_goes_to_named_motor_without_sdk_holes_or_sign_flip(self):
        s = self.sim
        # Compute expected torques from pre-step named positions and compare the
        # actual first step's applied controls; extra physics steps are disabled.
        s.last_action[:] = np.linspace(-.1, .1, 23)
        expected = np.array(cfg.STIFFNESS) * np.array(cfg.ACTION_SCALES) * s.last_action
        from unittest.mock import patch
        with patch.object(s.env, "physical_step"):
            s.step()
        for name, tau in zip(cfg.ACTION_JOINT_NAMES, expected):
            self.assertAlmostEqual(s.env.model_data.ctrl[s.env.model.actuator(name).id], tau, places=6)

    def test_reset_restores_base_history_and_actions(self):
        s = self.sim
        s.env.model_data.qpos[:3] = [1, 2, 3]
        s.last_action[:] = 1
        s.commands[:] = 1
        s.reset()
        np.testing.assert_allclose(s.env.model_data.qpos[:7], [0,0,cfg.SPAWN_HEIGHT,1,0,0,0])
        np.testing.assert_allclose(s.proprioception()[3], 0, atol=1e-7)
        self.assertFalse(s.last_action.any())
        self.assertFalse(s.commands.any())
        gravity = s.history[1].get_history()
        np.testing.assert_allclose(gravity, np.tile([0,0,-1], (8,1)))
        self.assertEqual(s.depth.get_depth_obs().shape, (1,8,18,32))


@unittest.skipUnless(all(Path(p).is_file() for p in (cfg.ACTOR_FILE,cfg.DEPTH_ENCODER_FILE)), "Local 23DoF export required")
class G1PolicyTests(unittest.TestCase):
    def test_real_policy_accepts_752_observations_and_returns_23_actions(self):
        policy = Policy()
        rng = np.random.default_rng(23)
        action = policy([rng.normal(size=d*8).astype(np.float32) for d in cfg.OBS_DIMS],
                        rng.uniform(size=cfg.DEPTH_INPUT_SHAPE).astype(np.float32))
        self.assertEqual(action.shape, (23,))
        self.assertTrue(np.isfinite(action).all())


if __name__ == "__main__":
    unittest.main()
