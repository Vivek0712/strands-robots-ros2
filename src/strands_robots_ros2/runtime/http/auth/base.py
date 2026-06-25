"""``AuthStrategy`` ABC — pluggable HTTP auth for ``HttpRuntime``.

Owns 'how do I prove identity to this robot's HTTP API?'. Called once
at ``HttpRuntime.connect`` before the connect_sequence runs. Free to
attach cookies, set default headers on the session, or wrap the
session's request method to add per-request auth (signing, refresh).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from typing import Any


class AuthStrategy(ABC):
    """Pluggable auth strategy for the HTTP runtime."""

    @classmethod
    @abstractmethod
    def from_config(cls, config: Mapping[str, Any]) -> AuthStrategy:
        """Build a strategy from the ``http.auth`` registry block."""

    @abstractmethod
    def prepare(self, session: Any, host: str) -> None:
        """Apply auth to ``session`` before the connect_sequence runs.

        Implementations should attach cookies, set default headers, or
        configure session behavior. Called exactly once per
        ``HttpRuntime.connect`` cycle.
        """

    def refresh(self, session: Any, host: str) -> None:  # noqa: B027 — intentional optional override
        """Optional: refresh credentials. Default no-op.

        Strategies with finite-TTL tokens (bearer with refresh, OAuth2)
        override this. The base ``HttpRuntime`` does not call refresh
        on a schedule today; v1.1 may add a per-request hook.
        """
        return None
