"""``bearer_token`` auth — set ``Authorization: Bearer <token>`` header
on every session request."""

from __future__ import annotations

import os
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.auth.base import AuthStrategy


class BearerTokenAuth(AuthStrategy):
    """Adds an ``Authorization`` header with a configurable scheme."""

    def __init__(
        self,
        *,
        token: str = "",
        token_env: str = "",
        header_name: str = "Authorization",
        scheme: str = "Bearer",
    ) -> None:
        self._token_literal = token
        self._token_env = token_env
        self.header_name = header_name
        self.scheme = scheme

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> BearerTokenAuth:
        return cls(
            token=str(config.get("token", "") or ""),
            token_env=str(config.get("token_env", "") or ""),
            header_name=str(config.get("header_name", "Authorization")),
            scheme=str(config.get("scheme", "Bearer")),
        )

    def _resolve_token(self) -> str:
        if self._token_literal:
            return self._token_literal
        if self._token_env:
            return os.environ.get(self._token_env, "")
        return ""

    def prepare(self, session: Any, host: str) -> None:
        token = self._resolve_token()
        if not token:
            raise ValueError(f"BearerTokenAuth: empty token (set 'token' in config or env var {self._token_env!r})")
        prefix = f"{self.scheme} " if self.scheme else ""
        session.headers[self.header_name] = f"{prefix}{token}"
