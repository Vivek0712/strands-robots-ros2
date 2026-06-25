"""``cookie_password`` auth — POST a form password to a login endpoint,
expect a session cookie back, attach to all subsequent requests.

DeepRacer uses this: ``POST /login`` with ``{"password": "<chassis>"}``
returns a ``deepracer_token`` cookie that's checked via
``hmac.compare_digest`` on every request.

Optional CSRF support — set ``csrf: true`` in the registry config to
GET the login page first, scrape the Flask-WTF ``csrf_token`` from the
form, and POST it back alongside the password. Required by DeepRacer's
webserver_pkg (which uses Flask-WTF's ``CSRFProtect``).
"""

from __future__ import annotations

import logging
import os
import re
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.auth.base import AuthStrategy

logger = logging.getLogger(__name__)

# Standard Flask-WTF token-extraction patterns. Order matters — first match wins.
_CSRF_PATTERNS = (
    # Hidden form input with name first, value second (Flask-WTF default render).
    re.compile(r'name=["\']csrf_token["\'][^>]*value=["\']([^"\']+)["\']'),
    # Same input but with value first, name second (Jinja templating quirks).
    re.compile(r'value=["\']([^"\']+)["\'][^>]*name=["\']csrf_token["\']'),
    # `<meta name="csrf-token" content="...">` style (less common in Flask-WTF).
    re.compile(r'<meta\s+name=["\']csrf-token["\']\s+content=["\']([^"\']+)["\']'),
)


def _extract_csrf_token(html: str, custom_pattern: str | None = None) -> str | None:
    """Find a Flask-WTF / meta-tag CSRF token in an HTML response."""
    patterns: tuple[re.Pattern, ...]
    if custom_pattern:
        patterns = (re.compile(custom_pattern),)
    else:
        patterns = _CSRF_PATTERNS
    for pat in patterns:
        m = pat.search(html)
        if m:
            return m.group(1)
    return None


class CookiePasswordAuth(AuthStrategy):
    """Login with a form password, expect a session cookie."""

    def __init__(
        self,
        *,
        endpoint: str = "/login",
        method: str = "POST",
        field_name: str = "password",
        password: str = "",
        password_env: str = "",
        cookie_name: str = "session",
        expected_status: tuple[int, ...] = (200, 302),
        timeout_s: float = 5.0,
        csrf: bool = False,
        csrf_endpoint: str | None = None,
        csrf_field_name: str = "csrf_token",
        csrf_header_name: str | None = None,
        csrf_pattern: str | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.method = method
        self.field_name = field_name
        self._password_literal = password
        self._password_env = password_env
        self.cookie_name = cookie_name
        self.expected_status = expected_status
        self.timeout_s = timeout_s
        self.csrf = csrf
        self.csrf_endpoint = csrf_endpoint
        self.csrf_field_name = csrf_field_name
        self.csrf_header_name = csrf_header_name
        self.csrf_pattern = csrf_pattern

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> CookiePasswordAuth:
        return cls(
            endpoint=str(config.get("endpoint", "/login")),
            method=str(config.get("method", "POST")),
            field_name=str(config.get("field_name", "password")),
            password=str(config.get("password", "") or ""),
            password_env=str(config.get("password_env", "") or ""),
            cookie_name=str(config.get("cookie_name", "session")),
            expected_status=tuple(config.get("expected_status", [200, 302])),
            timeout_s=float(config.get("timeout_s", 5.0)),
            csrf=bool(config.get("csrf", False)),
            csrf_endpoint=config.get("csrf_endpoint") or None,
            csrf_field_name=str(config.get("csrf_field_name", "csrf_token")),
            csrf_header_name=config.get("csrf_header_name") or None,
            csrf_pattern=config.get("csrf_pattern") or None,
        )

    def _resolve_password(self) -> str:
        if self._password_literal:
            return self._password_literal
        if self._password_env:
            env_val = os.environ.get(self._password_env, "")
            if env_val:
                return env_val
        return ""

    def prepare(self, session: Any, host: str) -> None:
        password = self._resolve_password()
        if not password:
            raise ValueError(
                f"CookiePasswordAuth: empty password (set 'password' in config or env var {self._password_env!r})"
            )

        resp = session.request(
            self.method,
            f"{host}{self.endpoint}",
            data={self.field_name: password},
            timeout=self.timeout_s,
            allow_redirects=False,
        )
        if resp.status_code not in self.expected_status:
            raise RuntimeError(
                f"CookiePasswordAuth login failed: status={resp.status_code}, expected one of {self.expected_status}"
            )
        if self.cookie_name not in session.cookies:
            raise RuntimeError(
                f"CookiePasswordAuth login: expected cookie {self.cookie_name!r} was not set on the session"
            )
