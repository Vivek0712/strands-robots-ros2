"""``twist_to_servo`` — Ackermann bicycle-model conversion.

Same math as the original DeepRacer translator, lifted into the
generic HTTP runtime so any Ackermann robot whose endpoint expects
normalized servo commands (``angle, throttle`` ∈ [-1, 1]) can use it
via JSON-only config.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.translators.base import PayloadTranslator

# Below this absolute linear velocity we treat the command as 'rest'
# and emit zero. Avoids atan2 blowing up on tiny denominators and
# matches physical reality (no yaw at rest on stock Ackermann).
_REST_VELOCITY_EPS = 1e-3


def twist_to_servo_norm(
    *,
    linear_x: float,
    angular_z: float,
    wheelbase_m: float,
    max_linear_x: float,
    max_steering_rad: float,
) -> tuple[float, float]:
    """Convert a body-frame Twist to (angle_norm, throttle_norm) in [-1, 1]."""
    v = float(linear_x)
    omega = float(angular_z)

    if abs(v) < _REST_VELOCITY_EPS:
        return (0.0, 0.0)

    delta = math.atan2(float(wheelbase_m) * omega, v)
    delta = max(-float(max_steering_rad), min(float(max_steering_rad), delta))
    delta_norm = delta / float(max_steering_rad)
    throttle = max(-1.0, min(1.0, v / float(max_linear_x)))
    return (float(delta_norm), float(throttle))


class TwistToServo(PayloadTranslator):
    """Bicycle-model translator for Ackermann HTTP robots."""

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> TwistToServo:
        return cls()

    def convert(
        self,
        action: Mapping[str, Any],
        params: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        limits = params.get("safety_limits", {}) or {}
        wheelbase = float(params.get("wheelbase_m", 0.164))
        v_max = float(limits.get("max_linear_x", 1.5))
        delta_max = float(limits.get("max_steering_rad", 0.5236))
        angle, throttle = twist_to_servo_norm(
            linear_x=float(action.get("linear_x", 0.0)),
            angular_z=float(action.get("angular_z", 0.0)),
            wheelbase_m=wheelbase,
            max_linear_x=v_max,
            max_steering_rad=delta_max,
        )
        return {"angle": angle, "throttle": throttle}

    def safe_state(self) -> Mapping[str, Any]:
        return {"angle": 0.0, "throttle": 0.0}

    def action_schema(self, params: Mapping[str, Any]) -> Mapping[str, Any]:
        limits = params.get("safety_limits", {}) or {}
        max_lin = float(limits.get("max_linear_x", float("inf")))
        max_ang = float(limits.get("max_angular_z", float("inf")))
        schema: dict[str, Any] = {
            "linear_x": {"min": -max_lin, "max": max_lin, "unit": "m/s"},
            "angular_z": {"min": -max_ang, "max": max_ang, "unit": "rad/s"},
        }
        wb = params.get("wheelbase_m")
        if wb is not None:
            schema["wheelbase"] = {"value": float(wb), "unit": "m"}
            delta_max_p = limits.get("max_steering_rad")
            if delta_max_p:
                R_min = float(wb) / math.tan(float(delta_max_p))
                schema["min_turning_radius"] = {"value": R_min, "unit": "m"}
        return schema
