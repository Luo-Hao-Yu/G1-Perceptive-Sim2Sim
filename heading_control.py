"""Heading feedback for a policy that accepts angular velocity commands."""

import math


def heading_from_matrix(matrix) -> float:
    """World heading of the body's local +X axis, in radians."""
    return math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))


class HeadingHold:
    def __init__(self, gain: float, max_yaw_rate: float, yaw_limits):
        if not math.isfinite(gain) or gain <= 0:
            raise ValueError("HEADING_HOLD_GAIN must be finite and positive")
        if not math.isfinite(max_yaw_rate) or max_yaw_rate <= 0:
            raise ValueError("HEADING_HOLD_MAX_YAW_RATE must be finite and positive")
        low, high = yaw_limits
        if not (math.isfinite(low) and math.isfinite(high) and low < 0 < high):
            raise ValueError("Heading hold requires yaw limits spanning zero")
        self.gain = gain
        self.low = max(low, -max_yaw_rate)
        self.high = min(high, max_yaw_rate)
        self.yaw_limits = (low, high)
        self.target = 0.0

    def reset(self, heading: float) -> None:
        self.target = heading

    def update(self, requested_yaw: float, heading: float) -> float:
        if abs(requested_yaw) > 1e-6:
            # Manual turns have priority. Hold this direction on stick release,
            # rather than pulling the robot back to its original heading.
            self.target = heading
            return max(self.yaw_limits[0], min(self.yaw_limits[1], requested_yaw))
        error = math.atan2(math.sin(self.target - heading), math.cos(self.target - heading))
        return max(self.low, min(self.high, self.gain * error))
