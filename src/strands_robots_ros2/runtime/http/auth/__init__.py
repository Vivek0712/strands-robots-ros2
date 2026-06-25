"""Pluggable HTTP auth strategies.

Each strategy implements ``AuthStrategy.prepare(session, host)`` —
called once at ``HttpRuntime.connect`` to attach cookies, headers, or
session-level config before any other request.

Built-in names (use directly in ``http.auth.strategy``):

- ``none``             → no auth
- ``cookie_password``  → POST password to login endpoint, expect a session cookie
- ``bearer_token``     → ``Authorization: Bearer <token>`` header
- ``api_key``          → custom header (default ``X-API-Key``)

Custom strategies use ``module.path:ClassName`` form.
"""

from __future__ import annotations

import importlib
import inspect
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.auth.api_key import ApiKeyAuth
from strands_robots_ros2.runtime.http.auth.base import AuthStrategy
from strands_robots_ros2.runtime.http.auth.bearer_token import BearerTokenAuth
from strands_robots_ros2.runtime.http.auth.cookie_password import CookiePasswordAuth
from strands_robots_ros2.runtime.http.auth.none import NoAuth

_BUILTIN_AUTH: dict[str, type[AuthStrategy]] = {
    "none": NoAuth,
    "cookie_password": CookiePasswordAuth,
    "bearer_token": BearerTokenAuth,
    "api_key": ApiKeyAuth,
}


def resolve_auth_strategy(config: Mapping[str, Any]) -> AuthStrategy:
    """Build an ``AuthStrategy`` from the ``http.auth`` registry block.

    Resolution rules:
        1. ``strategy`` matches a built-in name → use it.
        2. ``strategy`` is ``module.path:ClassName`` → ``importlib`` it.
        3. Otherwise → ``ValueError`` listing the built-ins.
    """
    name = config.get("strategy", "none") if config else "none"
    if name in _BUILTIN_AUTH:
        return _BUILTIN_AUTH[name].from_config(config or {})

    if ":" in name:
        module_path, class_name = name.split(":", 1)
        module = importlib.import_module(module_path)
        cls = getattr(module, class_name)
        if not inspect.isclass(cls) or not issubclass(cls, AuthStrategy):
            cls_name = getattr(cls, "__name__", repr(cls))
            raise ValueError(f"Auth strategy {name!r} is not a subclass of AuthStrategy (got {cls_name})")
        return cls.from_config(config or {})

    raise ValueError(
        f"Unknown auth strategy {name!r}. Built-ins: {sorted(_BUILTIN_AUTH)}. "
        "For custom strategies use 'module.path:ClassName'."
    )


__all__ = [
    "AuthStrategy",
    "ApiKeyAuth",
    "BearerTokenAuth",
    "CookiePasswordAuth",
    "NoAuth",
    "resolve_auth_strategy",
]
