"""Control transitions can be checked without an attached gamepad or display."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

import config
from gamepad_control import GamepadCommand, GamepadController, axis_with_deadzone


class FakeGLFW:
    def __init__(self):
        self.states = {0: self.state()}
        self.unmapped = set()

    @staticmethod
    def state(forward=0.0, yaw=0.0, stop=False, reset=False):
        axes = [0.0] * 6
        axes[1] = forward
        axes[2] = yaw
        buttons = [0] * 15
        buttons[1] = int(stop)
        buttons[3] = int(reset)
        return SimpleNamespace(axes=axes, buttons=buttons)

    def joystick_present(self, joystick_id):
        return joystick_id in self.states

    def joystick_is_gamepad(self, joystick_id):
        return joystick_id in self.states and joystick_id not in self.unmapped

    def get_gamepad_state(self, joystick_id):
        return self.states.get(joystick_id)

    def get_gamepad_name(self, joystick_id):
        return b"Xbox test controller"


class GamepadControlTests(unittest.TestCase):
    def setUp(self):
        self.backend = FakeGLFW()
        self.logs = []
        self.controller = GamepadController(backend=self.backend, logger=self.logs.append)

    def move(self, **kwargs):
        self.backend.states[0] = self.backend.state(**kwargs)
        return self.controller.poll()

    def test_deadzone_and_continuous_scaling(self):
        for value in (-0.15, -0.1, 0, 0.1, 0.15):
            self.assertEqual(axis_with_deadzone(value, 0.15), 0)
        self.assertAlmostEqual(axis_with_deadzone(0.575, 0.15), 0.5)
        self.assertAlmostEqual(axis_with_deadzone(-0.575, 0.15), -0.5)
        self.assertEqual(axis_with_deadzone(2, 0.15), 1)
        self.assertEqual(axis_with_deadzone(-2, 0.15), -1)

    def test_start_with_displaced_stick_requires_neutral(self):
        self.assertEqual(self.move(forward=-1), GamepadCommand())
        self.assertEqual(self.move(forward=-1), GamepadCommand())
        self.assertEqual(self.move(), GamepadCommand())
        self.assertAlmostEqual(self.move(forward=-1).forward, 0.9)

    def test_direction_limits_and_release(self):
        self.controller.poll()
        self.assertEqual(self.move(forward=-1, yaw=-1), GamepadCommand(0.9, 1.0))
        self.assertEqual(self.move(forward=1, yaw=1), GamepadCommand(0, -1.0))
        command = self.move(forward=-0.575, yaw=0.575)
        self.assertAlmostEqual(command.forward, 0.45)
        self.assertAlmostEqual(command.yaw, -0.5)
        self.assertEqual(self.move(), GamepadCommand())

    def test_unused_left_horizontal_axis_does_not_command_lateral_motion(self):
        self.controller.poll()
        self.backend.states[0].axes[0] = 1
        self.assertEqual(self.controller.poll(), GamepadCommand())

    def test_stop_requires_stick_center_after_button_release(self):
        self.controller.poll()
        self.assertGreater(self.move(forward=-1).forward, 0)
        self.assertEqual(self.move(forward=-1, stop=True), GamepadCommand())
        self.assertEqual(self.move(forward=-1), GamepadCommand())
        self.assertEqual(self.move(), GamepadCommand())
        self.assertGreater(self.move(forward=-1).forward, 0)

    def test_keyboard_stop_uses_same_neutral_gate(self):
        self.controller.poll()
        self.move(forward=-1)
        self.controller.stop()
        self.assertEqual(self.move(forward=-1), GamepadCommand())
        self.move()
        self.assertGreater(self.move(forward=-1).forward, 0)

    def test_reset_is_an_edge_and_prevents_movement_while_held(self):
        self.controller.poll()
        self.assertEqual(self.move(forward=-1, reset=True), GamepadCommand(reset=True))
        self.assertEqual(self.move(reset=True), GamepadCommand())
        self.assertEqual(self.move(forward=-1, reset=True), GamepadCommand())
        self.assertEqual(self.move(forward=-1), GamepadCommand())
        self.move()
        self.assertGreater(self.move(forward=-1).forward, 0)
        self.assertTrue(self.move(reset=True).reset)

    def test_disconnect_zeros_commands_and_reconnect_requires_neutral(self):
        with patch('gamepad_control.time.monotonic', return_value=10):
            self.controller.poll()
            self.move(forward=-1)
            self.backend.states.clear()
            self.assertEqual(self.controller.poll(), GamepadCommand())
            self.assertIsNone(self.controller.joystick_id)
            self.backend.states[0] = self.backend.state(forward=-1)
            self.assertEqual(self.controller.poll(), GamepadCommand())
        with patch('gamepad_control.time.monotonic', return_value=12):
            self.assertEqual(self.controller.poll(), GamepadCommand())
            self.move()
            self.assertGreater(self.move(forward=-1).forward, 0)

    def test_only_selected_mapped_device_is_used(self):
        self.backend.states[7] = self.backend.state()
        controller = GamepadController(7, backend=self.backend, logger=self.logs.append)
        controller.poll()
        self.assertEqual(controller.joystick_id, 7)
        self.backend.states[0] = self.backend.state(forward=-1)
        self.assertEqual(controller.poll(), GamepadCommand())
        self.backend.unmapped.add(7)
        controller = GamepadController(7, backend=self.backend, logger=self.logs.append)
        self.assertEqual(controller.poll(), GamepadCommand())
        self.assertIsNone(controller.joystick_id)

    def test_invalid_axis_data_stops_until_neutral(self):
        self.controller.poll()
        self.assertEqual(self.move(forward=float('nan')), GamepadCommand())
        self.assertEqual(self.move(forward=-1), GamepadCommand())
        self.move()
        self.assertGreater(self.move(forward=-1).forward, 0)

    def test_absent_controller_waits_without_repeated_logging(self):
        self.backend.states.clear()
        with patch('gamepad_control.time.monotonic', return_value=10):
            for _ in range(20):
                self.assertEqual(self.controller.poll(), GamepadCommand())
        self.assertEqual(len(self.logs), 1)

    def test_invalid_configuration_fails_before_viewer_launch(self):
        for deadzone in (-0.1, 1, float('nan')):
            with self.subTest(deadzone=deadzone), patch.object(config, 'GAMEPAD_DEADZONE', deadzone):
                with self.assertRaises(ValueError):
                    GamepadController(backend=self.backend)
        for joystick_id in (-1, 16):
            with self.assertRaises(ValueError):
                GamepadController(joystick_id, backend=self.backend)


if __name__ == '__main__':
    unittest.main()
