"""No-op auth strategy — for HTTP robots inside a trusted LAN."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.auth.base import AuthStrategy


class NoAuth(AuthStrategy):
    """No auth. ``prepare`` is a no-op."""

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> NoAuth:
        return cls()

    def prepare(self, session: Any, host: str) -> None:
        return None
