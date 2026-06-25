"""``twist_passthrough`` — forwards ``{linear_x, angular_z}`` as-is.

For HTTP-controlled differential / omni / un-translated robots whose
endpoint already accepts the canonical Twist shape.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.translators.base import PayloadTranslator


class TwistPassthrough(PayloadTranslator):
    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> TwistPassthrough:
        return cls()

    def convert(
        self,
        action: Mapping[str, Any],
        params: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        return {
            "linear_x": float(action.get("linear_x", 0.0)),
            "angular_z": float(action.get("angular_z", 0.0)),
        }

    def safe_state(self) -> Mapping[str, Any]:
        return {"linear_x": 0.0, "angular_z": 0.0}

    def action_schema(self, params: Mapping[str, Any]) -> Mapping[str, Any]:
        limits = params.get("safety_limits", {}) or {}
        max_lin = float(limits.get("max_linear_x", float("inf")))
        max_ang = float(limits.get("max_angular_z", float("inf")))
        return {
            "linear_x": {"min": -max_lin, "max": max_lin, "unit": "m/s"},
            "angular_z": {"min": -max_ang, "max": max_ang, "unit": "rad/s"},
        }
