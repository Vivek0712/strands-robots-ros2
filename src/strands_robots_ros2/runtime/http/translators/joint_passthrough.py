"""``joint_passthrough`` — joint-name keyed action → JSON.

Two modes:

- Default (no ``joint_order``): emits a flat dict of ``{joint: value}``.
- With ``joint_order``: emits ``{"positions": [v1, v2, ...]}`` in the
  declared joint order. Missing joints zero-padded.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.translators.base import PayloadTranslator


class JointPassthrough(PayloadTranslator):
    def __init__(self, *, joint_order: list[str] | None = None) -> None:
        self.joint_order: list[str] | None = list(joint_order) if joint_order else None

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> JointPassthrough:
        order = config.get("joint_order")
        return cls(joint_order=list(order) if order else None)

    def convert(
        self,
        action: Mapping[str, Any],
        params: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        if self.joint_order:
            return {"positions": [float(action.get(j, 0.0)) for j in self.joint_order]}
        return {k: float(v) for k, v in action.items() if isinstance(v, (int, float))}

    def safe_state(self) -> Mapping[str, Any]:
        if self.joint_order:
            return {"positions": [0.0] * len(self.joint_order)}
        return {}

    def action_schema(self, params: Mapping[str, Any]) -> Mapping[str, Any]:
        if self.joint_order:
            return {
                "positions": {
                    "shape": [len(self.joint_order)],
                    "joints": list(self.joint_order),
                }
            }
        return {}
