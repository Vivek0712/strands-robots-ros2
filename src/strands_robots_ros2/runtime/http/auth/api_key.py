"""``api_key`` auth — single header (default ``X-API-Key``)."""

from __future__ import annotations

import os
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.auth.base import AuthStrategy


class ApiKeyAuth(AuthStrategy):
    """Sets a single header on the session for every request."""

    def __init__(
        self,
        *,
        key: str = "",
        key_env: str = "",
        header_name: str = "X-API-Key",
    ) -> None:
        self._key_literal = key
        self._key_env = key_env
        self.header_name = header_name

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> ApiKeyAuth:
        return cls(
            key=str(config.get("key", "") or ""),
            key_env=str(config.get("key_env", "") or ""),
            header_name=str(config.get("header_name", "X-API-Key")),
        )

    def _resolve_key(self) -> str:
        if self._key_literal:
            return self._key_literal
        if self._key_env:
            return os.environ.get(self._key_env, "")
        return ""

    def prepare(self, session: Any, host: str) -> None:
        key = self._resolve_key()
        if not key:
            raise ValueError(f"ApiKeyAuth: empty key (set 'key' in config or env var {self._key_env!r})")
        session.headers[self.header_name] = key
