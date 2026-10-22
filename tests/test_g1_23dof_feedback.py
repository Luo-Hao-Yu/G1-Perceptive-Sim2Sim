import math
import unittest
from unittest.mock import Mock, patch

import numpy as np

from g1_23dof.command_control import PlanarCommandFeedback
from g1_23dof.runtime import Simulation
from gamepad_control import GamepadCommand


def controller(mode="path"):
    return PlanarCommandFeedback(mode,2.,1.,(-1.,1.),1.,math.radians(30))


class CommandFeedbackTests(unittest.TestCase):
    def test_heading_feedback_corrects_yaw_with_shortest_angle(self):
        c=controller("heading")
        c.reset((0,0),0)
        self.assertAlmostEqual(c.update(.4,0,(0,.2),.1),-.2)
        self.assertAlmostEqual(c.target_heading,0)
        c.reset((0,0),math.radians(-179))
        self.assertAlmostEqual(c.update(.4,0,(0,0),math.radians(179)),math.radians(4))

    def test_path_feedback_steers_back_to_line_and_obeys_limits(self):
        c=controller()
        self.assertAlmostEqual(c.update(.4,0,(2,.2),0),2*math.atan2(-.2,1))
        self.assertAlmostEqual(c.cross_track_error,.2)
        self.assertEqual(c.update(.4,0,(2,5),0),-1.)
        self.assertAlmostEqual(c.target_heading,-math.radians(30))

    def test_cross_track_is_in_line_coordinates_after_rotation(self):
        c=controller()
        c.reset((1,2),math.pi/2)
        value=c.update(.4,0,(.8,4),math.pi/2)
        self.assertAlmostEqual(c.cross_track_error,.2)
        self.assertLess(value,0)

    def test_manual_turn_release_captures_actual_new_heading_and_position(self):
        c=controller()
        self.assertAlmostEqual(c.update(.4,.7,(1,.3),.4),.7)
        self.assertAlmostEqual(c.update(.4,0,(1.1,.4),.45),0)
        self.assertEqual(c.origin,(1.1,.4))
        self.assertAlmostEqual(c.path_heading,.45)
        self.assertAlmostEqual(c.target_heading,.45)
        self.assertLess(c.update(.4,0,(1.1,.4),.50),0)

    def test_stop_clears_previous_path_and_does_not_recover_old_offset(self):
        c=controller()
        c.update(.4,0,(2,.2),.05)
        self.assertEqual(c.update(0,0,(2.1,.25),.08),0)
        self.assertEqual(c.origin,(2.1,.25))
        self.assertAlmostEqual(c.target_heading,.08)
        self.assertEqual(c.update(.4,0,(2.1,.25),.08),0)

    def test_off_mode_preserves_manual_command(self):
        c=controller("off")
        self.assertEqual(c.update(.4,.3,(10,10),1),.3)
        self.assertEqual(c.update(.4,0,(10,10),1),0)


class RuntimeFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.s=Simulation(preview=True,headless=True,feedback="path")
        self.addCleanup(self.s.env.close)

    def test_corrected_command_reaches_proprioception_without_changing_manual_input(self):
        s=self.s
        s.env.model_data.qpos[1]=.2
        s.env.forward()
        s.commands[:]=(.4,0,0)
        s.update_velocity_command()
        np.testing.assert_array_equal(s.commands,[np.float32(.4),0,0])
        self.assertLess(s.effective_commands[2],0)
        np.testing.assert_array_equal(s.proprioception()[2],s.effective_commands)

    def test_stop_and_reset_clear_references_and_effective_history(self):
        s=self.s
        s.env.model_data.qpos[0:2]=[1,.2];s.env.forward()
        s.commands[:]=(.4,0,.3)
        s.update_velocity_command()
        s.key_callback(32)
        self.assertTrue(s._stop_requested)
        s.stop()
        self.assertFalse(s._stop_requested)
        self.assertFalse(s.effective_commands.any())
        self.assertEqual(s.feedback.origin,(1,.2))
        s.reset()
        self.assertEqual(s.feedback.origin,(0,0))
        self.assertEqual(s.feedback.target_heading,0)
        self.assertFalse(s.effective_commands.any())
        self.assertFalse(s.history[2].get_history().any())

    def test_gamepad_stop_and_reset_are_applied_on_control_thread(self):
        s=self.s
        s.gamepad=Mock()
        s.env.model_data.qpos[:2]=[1,.2];s.env.forward()
        s.commands[:]=(.4,0,0)
        s.gamepad.poll.return_value=GamepadCommand(stop=True)
        with patch.object(s.env,"physical_step"):
            s.step()
        self.assertFalse(s.commands.any())
        self.assertEqual(s.feedback.origin,(1,.2))
        s.gamepad.poll.return_value=GamepadCommand(reset=True)
        with patch.object(s.env,"physical_step"):
            s.step()
        self.assertEqual(s.feedback.origin,(0,0))
        self.assertEqual(s.env.model_data.qpos[0],0)

    def test_keyboard_reset_is_queued_and_leaves_no_stale_yaw_reference(self):
        s=self.s
        s.env.model_data.qpos[:2]=[1,.2];s.env.forward()
        s.feedback.reset((1,.2),.4)
        s.key_callback(82)
        self.assertTrue(s._reset_requested)
        self.assertEqual(s.env.model_data.qpos[0],1)
        with patch.object(s.env,"physical_step"):
            s.step()
        self.assertFalse(s._reset_requested)
        self.assertEqual(s.feedback.target_heading,0)
        self.assertFalse(s.effective_commands.any())


if __name__=="__main__":
    unittest.main()
