"""World-position and heading feedback around the policy's velocity command."""
import math

from heading_control import HeadingHold


class PlanarCommandFeedback:
    def __init__(self, mode, gain, max_yaw_rate, yaw_limits, lookahead, max_heading_offset):
        if mode not in ("off", "heading", "path"):
            raise ValueError("Command feedback mode must be off, heading or path")
        if not math.isfinite(lookahead) or lookahead <= 0:
            raise ValueError("PATH_LOOKAHEAD must be finite and positive")
        if not math.isfinite(max_heading_offset) or not 0 < max_heading_offset < math.pi/2:
            raise ValueError("PATH_MAX_HEADING_OFFSET must be in (0, pi/2)")
        self.mode = mode
        self.hold = HeadingHold(gain, max_yaw_rate, yaw_limits)
        self.lookahead = lookahead
        self.max_heading_offset = max_heading_offset
        self.reset((0, 0), 0)

    def reset(self, position, heading):
        self.origin = (float(position[0]), float(position[1]))
        self.path_heading = float(heading)
        self.hold.reset(float(heading))
        self.cross_track_error = 0.
        self.heading_error = 0.
        self._turning = False
        self._moving = False

    @property
    def target_heading(self):
        return self.hold.target

    def update(self, forward, manual_yaw, position, heading):
        if not all(math.isfinite(float(v)) for v in (forward, manual_yaw, position[0], position[1], heading)):
            raise ValueError("Command feedback requires finite state and commands")
        if self.mode == "off":
            return manual_yaw
        moving = abs(forward) > 1e-6
        if abs(manual_yaw) > 1e-6:
            # Manual yaw has priority. On release, capture the actual heading
            # after the last turning step, and start a new line at that point.
            self.reset(position, heading)
            self._turning = True
            self._moving = moving
            return self.hold.update(manual_yaw, heading)
        if self._turning or (self._moving and not moving):
            self.reset(position, heading)
        self._moving = moving
        dx, dy = float(position[0])-self.origin[0], float(position[1])-self.origin[1]
        self.cross_track_error = -math.sin(self.path_heading)*dx + math.cos(self.path_heading)*dy
        if self.mode == "path" and moving:
            # A lookahead point on the reference line provides a gentle heading
            # offset to compensate lateral velocity error without commanding vy.
            offset = math.atan2(-self.cross_track_error, self.lookahead)
            offset = max(-self.max_heading_offset, min(self.max_heading_offset, offset))
            self.hold.target = self.path_heading + offset
        self.heading_error = math.atan2(math.sin(self.hold.target-heading), math.cos(self.hold.target-heading))
        return self.hold.update(0., heading)
