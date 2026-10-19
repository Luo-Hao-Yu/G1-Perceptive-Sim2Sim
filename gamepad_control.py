"""Xbox/XInput velocity commands using GLFW's standard gamepad mapping.

Run this module to list devices, or pass --monitor to check live controls.
The simulator already initializes GLFW through its viewer; this controller
does not initialize, terminate, or poll the viewer's window events.
"""

import argparse
from dataclasses import dataclass
import math
import time

import glfw

import config


@dataclass(frozen=True)
class GamepadCommand:
    forward: float = 0.0
    yaw: float = 0.0
    reset: bool = False


def axis_with_deadzone(value: float, deadzone: float) -> float:
    """Remove drift and rescale the remaining stick travel to full range."""
    if not math.isfinite(value):
        raise ValueError("Non-finite gamepad axis")
    value = max(-1.0, min(1.0, value))
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def device_name(backend, joystick_id: int) -> str:
    name = backend.get_gamepad_name(joystick_id) or backend.get_joystick_name(joystick_id)
    return name.decode(errors="replace") if isinstance(name, bytes) else str(name)


class GamepadController:
    def __init__(self, joystick_id: int | None = None, *, backend=None, logger=print, settings=None):
        self.settings = config if settings is None else settings
        self.backend = glfw if backend is None else backend
        self.log = logger
        if joystick_id is not None and not 0 <= joystick_id <= 15:
            raise ValueError("Gamepad ID must be in 0..15")
        if not math.isfinite(self.settings.GAMEPAD_DEADZONE) or not 0 <= self.settings.GAMEPAD_DEADZONE < 1:
            raise ValueError("GAMEPAD_DEADZONE must be in [0, 1)")
        for name in ("GAMEPAD_FORWARD_AXIS", "GAMEPAD_YAW_AXIS"):
            value = getattr(self.settings, name)
            if not isinstance(value, int) or not 0 <= value < 6:
                raise ValueError(f"{name} must be an axis index in 0..5")
        for name in ("GAMEPAD_STOP_BUTTON", "GAMEPAD_RESET_BUTTON"):
            value = getattr(self.settings, name)
            if not isinstance(value, int) or not 0 <= value < 15:
                raise ValueError(f"{name} must be a button index in 0..14")
        for name in ("GAMEPAD_FORWARD_SIGN", "GAMEPAD_YAW_SIGN"):
            if getattr(self.settings, name) not in (-1, 1):
                raise ValueError(f"{name} must be -1 or 1")
        for name in ("lin_vel_x", "ang_vel_z"):
            low, high = self.settings.COMMAND_RANGES[name]
            if not (math.isfinite(low) and math.isfinite(high) and low <= 0 <= high):
                raise ValueError(f"{name} range must be finite and include zero")
        self.preferred_id = joystick_id
        self.joystick_id = None
        self._armed = False
        self._reset_down = False
        self._next_scan = 0.0
        self._status = None

    def _report(self, message: str) -> None:
        if message != self._status:
            self.log(message)
            self._status = message

    def stop(self) -> None:
        """Require neutral sticks before movement can resume."""
        self._armed = False

    def _connect(self) -> None:
        now = time.monotonic()
        if now < self._next_scan:
            return
        self._next_scan = now + 1.0
        candidates = range(16) if self.preferred_id is None else (self.preferred_id,)
        for joystick_id in candidates:
            if self.backend.joystick_present(joystick_id) and self.backend.joystick_is_gamepad(joystick_id):
                self.joystick_id = joystick_id
                self._reset_down = False
                self.stop()
                self._report(f"[Gamepad] Connected #{joystick_id}: {device_name(self.backend, joystick_id)}; center sticks to enable movement.")
                return
        self._report("[Gamepad] No mapped gamepad available; commands are zero. Connect an Xbox/XInput controller; waiting for connection.")

    @staticmethod
    def _velocity(value: float, limits) -> float:
        low, high = limits
        return value * (high if value >= 0 else -low)

    def poll(self) -> GamepadCommand:
        if self.joystick_id is None:
            self._connect()
        if self.joystick_id is None:
            return GamepadCommand()

        state = self.backend.get_gamepad_state(self.joystick_id)
        if state is None:
            self.joystick_id = None
            self._reset_down = False
            self.stop()
            self._next_scan = time.monotonic() + 1.0
            self._report("[Gamepad] Disconnected; commands are zero. Waiting for reconnection.")
            return GamepadCommand()

        try:
            forward = axis_with_deadzone(float(state.axes[self.settings.GAMEPAD_FORWARD_AXIS]), self.settings.GAMEPAD_DEADZONE) * self.settings.GAMEPAD_FORWARD_SIGN
            yaw = axis_with_deadzone(float(state.axes[self.settings.GAMEPAD_YAW_AXIS]), self.settings.GAMEPAD_DEADZONE) * self.settings.GAMEPAD_YAW_SIGN
            stop_down = bool(state.buttons[self.settings.GAMEPAD_STOP_BUTTON])
            reset_down = bool(state.buttons[self.settings.GAMEPAD_RESET_BUTTON])
        except (IndexError, TypeError, ValueError):
            self.stop()
            self._report("[Gamepad] Invalid input; commands are zero. Check the gamepad mapping.")
            return GamepadCommand()

        reset = reset_down and not self._reset_down
        self._reset_down = reset_down
        if stop_down or reset_down:
            self.stop()
            return GamepadCommand(reset=reset)
        if not self._armed:
            if forward == 0.0 and yaw == 0.0:
                self._armed = True
            return GamepadCommand()
        return GamepadCommand(
            forward=self._velocity(forward, self.settings.COMMAND_RANGES["lin_vel_x"]),
            yaw=self._velocity(yaw, self.settings.COMMAND_RANGES["ang_vel_z"]),
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--monitor", action="store_true", help="Continuously print velocity commands without starting MuJoCo")
    parser.add_argument("--gamepad-id", type=int, choices=range(16), help="Select a device ID; default is the first mapped gamepad")
    args = parser.parse_args()
    if not glfw.init():
        print("[FAIL] GLFW initialization failed. Run from a terminal with desktop/display access.")
        return 1
    try:
        found = False
        for joystick_id in range(16):
            if glfw.joystick_present(joystick_id):
                mapped = bool(glfw.joystick_is_gamepad(joystick_id))
                print(f"#{joystick_id}: {device_name(glfw, joystick_id)}; standard mapping: {mapped}")
                found |= mapped
        if not args.monitor:
            if not found:
                print("No mapped gamepads detected. Connect the controller, then run this command again.")
            return 0 if found else 1
        controller = GamepadController(args.gamepad_id)
        print("Left stick up: forward; right stick left/right: yaw; B: stop; Y: reset; Ctrl+C: quit.")
        while True:
            glfw.poll_events()
            command = controller.poll()
            print(f"forward={command.forward:+.3f} m/s, yaw={command.yaw:+.3f} rad/s, reset={command.reset}", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        return 0
    finally:
        glfw.terminate()


if __name__ == "__main__":
    raise SystemExit(main())
