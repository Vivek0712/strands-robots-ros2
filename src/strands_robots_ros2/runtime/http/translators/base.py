"""``PayloadTranslator`` ABC — pluggable action → JSON-body conversion."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from typing import Any


class PayloadTranslator(ABC):
    """Convert a canonical action dict into an HTTP request body."""

    @classmethod
    @abstractmethod
    def from_config(cls, config: Mapping[str, Any]) -> PayloadTranslator:
        """Build from the registry's ``translator_config`` block (may be empty)."""

    @abstractmethod
    def convert(
        self,
        action: Mapping[str, Any],
        params: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """Canonical action → body.

        Args:
            action: Canonical action dict (``{linear_x, angular_z, ...}``,
                joint-name keyed, etc.).
            params: Top-level fields from the registry (``wheelbase_m``,
                ``safety_limits``, etc.) — what ``Ros2Runtime._translator_params``
                already passes today.

        Returns:
            JSON-serializable dict the HTTP runtime POSTs as request body.
        """

    def safe_state(self) -> Mapping[str, Any]:
        """Body for ``stop()`` and the trailing-zero on ``disconnect()``.

        Default: empty dict (the runtime falls back to publishing zero
        of whatever the previous action was). Override for translators
        that need an explicit safe-state shape.
        """
        return {}

    def action_schema(self, params: Mapping[str, Any]) -> Mapping[str, Any]:
        """Schema reflected to the ``Policy`` / agent layer.

        Default: empty. Translators that know their action's bounds
        (Ackermann's wheelbase / min_turning_radius, joint limits, …)
        override this so ``RuntimeAdapter.action_schema`` can surface
        them.
        """
        return {}
