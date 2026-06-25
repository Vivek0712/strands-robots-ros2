"""``HttpRuntime`` — generic, registry-driven HTTP adapter.

Reads its entire behavior from a ``robots.json`` ``http`` block:
auth strategy, connect/disconnect sequences, command endpoint +
translator, and observation endpoints with per-endpoint TTL caching.

Adding a new HTTP-controlled robot is a JSON entry; no Python code
change unless the robot needs a custom ``AuthStrategy`` or
``PayloadTranslator``.

All ``requests`` / ``urllib3`` imports are lazy — importing this
module does NOT pull in HTTP libs; the imports happen on first use
inside :meth:`connect`.
"""

from __future__ import annotations

import logging
import os
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.base import RuntimeAdapter
from strands_robots_ros2.runtime.http.auth import resolve_auth_strategy
from strands_robots_ros2.runtime.http.freshness import ObservationCache, fetch_endpoint
from strands_robots_ros2.runtime.http.translators import resolve_translator

logger = logging.getLogger(__name__)


class HttpRuntime(RuntimeAdapter):
    """RuntimeAdapter for any robot exposing an HTTP/JSON control API."""

    def __init__(
        self,
        *,
        robot_id: str,
        http_block: Mapping[str, Any],
        world_id: str | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.world_id = world_id
        self._block: Mapping[str, Any] = dict(http_block or {})

        # Resolve transport config with env-var fallback.
        # ``host_env`` may be a string or a list of names — first non-empty wins.
        # If the resolved value is a bare host/IP (no scheme), prepend https://
        # so registries can keep ``DEEPRACER_IP=192.168.0.8`` style env vars.
        host = self._block.get("host") or self._resolve_env(self._block.get("host_env"))
        self._host: str = self._normalize_host(host)

        verify_ssl = self._block.get("verify_ssl")
        env_verify = self._resolve_env(self._block.get("verify_ssl_env"))
        if env_verify:
            verify_ssl = env_verify.strip().lower() in {"1", "true", "yes"}
        self._verify_ssl: bool = bool(verify_ssl) if verify_ssl is not None else True
        self._timeout: float = float(self._block.get("request_timeout_s", 5.0))
        self._default_headers: dict[str, str] = dict(self._block.get("default_headers") or {})

        # Resolve auth + translator at construction time so config errors
        # surface fast (not at first send_action 30 minutes into a run).
        self._auth = resolve_auth_strategy(self._block.get("auth", {}) or {})

        cmd_block = self._block.get("command_endpoint", {}) or {}
        self._command_url = cmd_block.get("url", "")
        self._command_method = str(cmd_block.get("method", "POST"))
        self._command_extra: dict[str, Any] = dict(cmd_block.get("extra_body") or {})
        self._translator = resolve_translator(
            cmd_block.get("translator"),
            cmd_block.get("translator_config", {}),
        )

        self._connect_seq: list[Mapping[str, Any]] = list(self._block.get("connect_sequence", []) or [])
        self._disconnect_seq: list[Mapping[str, Any]] = list(self._block.get("disconnect_sequence", []) or [])
        self._observation_endpoints: list[Mapping[str, Any]] = list(self._block.get("observation_endpoints", []) or [])

        self._session: Any = None
        self._observation_cache = ObservationCache()
        self._connected: bool = False

    @staticmethod
    def _resolve_env(env_name: Any) -> str:
        """Read an env var. Accepts a single name or a list — first non-empty wins."""
        if not env_name:
            return ""
        if isinstance(env_name, str):
            return os.environ.get(env_name, "")
        if isinstance(env_name, (list, tuple)):
            for n in env_name:
                if isinstance(n, str):
                    val = os.environ.get(n, "")
                    if val:
                        return val
        return ""

    @staticmethod
    def _normalize_host(host: Any) -> str:
        """Strip trailing slash; prepend ``https://`` if the value looks like a
        bare host or IP without a scheme. Empty input → empty string."""
        if not host or not isinstance(host, str):
            return ""
        host = host.strip().rstrip("/")
        if not host:
            return ""
        if "://" in host:
            return host
        return f"https://{host}"

    # ----- Lifecycle -----

    def connect(self) -> None:
        if self._connected:
            return
        if not self._host:
            raise ValueError(
                f"HttpRuntime[{self.robot_id}]: 'host' is required "
                "(set http.host, http.host_env, or pass host=... at construction)"
            )
        if not self._command_url:
            raise ValueError(f"HttpRuntime[{self.robot_id}]: command_endpoint.url is required")

        # Lazy import — keeps the runtime loadable on hosts without requests.
        import requests  # noqa: PLC0415
        import urllib3  # noqa: PLC0415

        if not self._verify_ssl:
            urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

        self._session = requests.Session()
        self._session.verify = self._verify_ssl
        if self._default_headers:
            self._session.headers.update(self._default_headers)

        # 1. Auth.
        try:
            self._auth.prepare(self._session, self._host)
        except Exception:
            self._session.close()
            self._session = None
            raise

        # 2. Connect-time sequence (e.g. set drive mode, enable motors).
        try:
            for entry in self._connect_seq:
                self._run_sequence_entry(entry)
        except Exception:
            self._session.close()
            self._session = None
            raise

        self._connected = True
        logger.info(
            "HttpRuntime[%s] connected to %s (verify_ssl=%s)",
            self.robot_id,
            self._host,
            self._verify_ssl,
        )

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            # Trailing safe state on the command endpoint, then disconnect_sequence.
            safe = self._translator.safe_state()
            if safe:
                self._post_command(dict(safe))
            for entry in self._disconnect_seq:
                self._run_sequence_entry(entry)
        except Exception as exc:  # noqa: BLE001 — best-effort teardown
            logger.warning("HttpRuntime[%s] disconnect cleanup raised: %s", self.robot_id, exc)
        finally:
            if self._session is not None:
                self._session.close()
            self._session = None
            self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def stop(self) -> None:
        """Publish a safe-state body via the command endpoint. Idempotent."""
        if self._session is None:
            return
        try:
            safe = self._translator.safe_state()
            if safe:
                self._post_command(dict(safe))
        except Exception as exc:  # noqa: BLE001
            logger.warning("HttpRuntime[%s] stop raised: %s", self.robot_id, exc)

    # ----- I/O -----

    def get_observation(self) -> Mapping[str, Any]:
        if self._session is None:
            return {}

        for entry in self._observation_endpoints:
            name = entry.get("name")
            if not isinstance(name, str):
                continue
            poll_hz = float(entry.get("poll_hz", 1.0))
            ttl = (1.0 / poll_hz) if poll_hz > 0 else float("inf")
            if self._observation_cache.is_fresh(name, ttl):
                continue
            value = fetch_endpoint(self._session, self._host, entry, self._timeout)
            if value is not None:
                self._observation_cache.set(name, value)

        return self._observation_cache.all_values()

    def send_action(self, action: Mapping[str, Any]) -> None:
        if self._session is None:
            raise RuntimeError(f"HttpRuntime[{self.robot_id}] is not connected — call connect() first")

        body = dict(self._translator.convert(action, self._translator_params()))
        # Static fields from the registry (e.g. DeepRacer's max_speed) merge AFTER
        # the translator output so they're stable across actions.
        body.update(self._command_extra)
        self._post_command(body)

    # ----- Introspection -----

    def safety_limits(self) -> Mapping[str, Any]:
        return dict(self._block.get("safety_limits", {}) or {})

    def command_rate_hz(self) -> float:
        return float(self._block.get("command_rate_hz", 20.0))

    def capabilities(self) -> set[str]:
        caps = self._block.get("capabilities", []) or []
        return set(caps)

    def observation_schema(self) -> Mapping[str, Any]:
        schema: dict[str, Any] = {}
        for entry in self._observation_endpoints:
            name = entry.get("name")
            if not isinstance(name, str):
                continue
            schema[name] = {
                "url": entry.get("url"),
                "decode": entry.get("decode", "json"),
                "poll_hz": float(entry.get("poll_hz", 1.0)),
            }
        return schema

    def action_schema(self) -> Mapping[str, Any]:
        return self._translator.action_schema(self._translator_params())

    # ----- Helpers -----

    def _translator_params(self) -> dict[str, Any]:
        return {
            "wheelbase_m": self._block.get("wheelbase_m"),
            "wheel_separation_m": self._block.get("wheel_separation_m"),
            "wheel_radius_m": self._block.get("wheel_radius_m"),
            "drive_type": self._block.get("drive_type"),
            "safety_limits": dict(self._block.get("safety_limits", {}) or {}),
        }

    def _post_command(self, body: Mapping[str, Any]) -> None:
        url = f"{self._host}{self._command_url}"
        resp = self._session.request(self._command_method, url, json=dict(body), timeout=self._timeout)
        if resp.status_code >= 400:
            raise RuntimeError(
                f"HttpRuntime[{self.robot_id}] {self._command_url} failed: "
                f"status={resp.status_code}, body={resp.text[:200]!r}"
            )

    def _run_sequence_entry(self, entry: Mapping[str, Any]) -> None:
        method = str(entry.get("method", "POST"))
        path = entry.get("url")
        if not isinstance(path, str) or not path:
            raise ValueError(f"HttpRuntime[{self.robot_id}]: sequence entry missing 'url': {entry!r}")
        url = f"{self._host}{path}"
        kwargs: dict[str, Any] = {"timeout": self._timeout}
        if "json" in entry:
            kwargs["json"] = entry["json"]
        if "data" in entry:
            kwargs["data"] = entry["data"]

        resp = self._session.request(method, url, **kwargs)
        expected = entry.get("expected_status")
        if expected:
            ok = resp.status_code in tuple(expected)
        else:
            ok = resp.status_code < 400
        if not ok:
            raise RuntimeError(
                f"HttpRuntime[{self.robot_id}] sequence entry "
                f"{entry.get('name', path)!r} failed: "
                f"status={resp.status_code}, body={resp.text[:200]!r}"
            )
