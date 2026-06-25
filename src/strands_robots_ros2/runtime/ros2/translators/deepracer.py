"""DeepRacer Twist -> ServoCtrlMsg translator.

The bicycle-model conversion for an Ackermann-steered car of wheelbase
``L``, given a body-frame Twist ``(linear_x, angular_z)``:

    delta    = atan2(L * angular_z, linear_x)
    throttle = linear_x / max_linear_x

Special case at ``v ≈ 0``: a stock Ackermann car cannot generate yaw
from rest (the steering relation is singular at v=0). We publish zero
throttle and zero steering, leaving the LLM to re-plan with a forward
velocity if it really wants to rotate the heading.

Outputs a ``deepracer_interfaces_pkg/msg/ServoCtrlMsg`` whose ``angle``
field is normalized to ``[-1, 1]`` (steering / delta_max) and whose
``throttle`` field is normalized to ``[-1, 1]``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

# Below this absolute linear velocity we treat the command as 'rest'
# and emit zero. Avoids atan2 blowing up on tiny denominators and
# matches physical reality (no yaw at rest on stock Ackermann).
_REST_VELOCITY_EPS = 1e-3


class TwistToServoCtrl:
    """Bicycle-model Twist → ServoCtrlMsg converter for AWS DeepRacer."""

    src_type = "geometry_msgs/msg/Twist"
    dst_type = "deepracer_interfaces_pkg/msg/ServoCtrlMsg"

    def convert(self, action: Mapping[str, Any], robot_params: Mapping[str, Any]) -> Any:
        """Convert a normalized Twist action dict to a ServoCtrlMsg."""
        v = float(action["linear_x"])
        omega = float(action["angular_z"])
        L = float(robot_params["wheelbase_m"])
        safety = robot_params["safety_limits"]
        v_max = float(safety["max_linear_x"])
        delta_max = float(safety["max_steering_rad"])

        if abs(v) < _REST_VELOCITY_EPS:
            delta_norm = 0.0
            throttle = 0.0
        else:
            delta = math.atan2(L * omega, v)
            delta = max(-delta_max, min(delta_max, delta))
            delta_norm = delta / delta_max
            throttle = max(-1.0, min(1.0, v / v_max))

        # Lazy import — the message type is only available when rclpy is
        # importable (real ROS2 install OR test mock).
        from deepracer_interfaces_pkg.msg import ServoCtrlMsg

        msg = ServoCtrlMsg()
        msg.angle = float(delta_norm)
        msg.throttle = float(throttle)
        return msg
