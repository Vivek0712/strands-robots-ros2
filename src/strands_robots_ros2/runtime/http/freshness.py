"""On-demand observation polling with per-endpoint TTL cache.

The HTTP runtime is pull-style — there's no DDS spin thread feeding
``_latest_messages``. Instead, ``get_observation()`` fetches each
declared ``observation_endpoints[]`` entry, capped by ``poll_hz``:
within ``1/poll_hz`` seconds the cached value is reused.

V1 is single-threaded and on-demand. V1.1 may add a background poller
pool; the cache shape stays the same.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Mapping
from typing import Any

logger = logging.getLogger(__name__)


class ObservationCache:
    """Per-endpoint TTL cache keyed by ``name``."""

    def __init__(self) -> None:
        self._values: dict[str, Any] = {}
        self._fetched_at: dict[str, float] = {}

    def get(self, name: str) -> Any:
        return self._values.get(name)

    def set(self, name: str, value: Any) -> None:
        self._values[name] = value
        self._fetched_at[name] = time.monotonic()

    def is_fresh(self, name: str, ttl_s: float) -> bool:
        ts = self._fetched_at.get(name)
        if ts is None:
            return False
        return (time.monotonic() - ts) < ttl_s

    def all_values(self) -> dict[str, Any]:
        return dict(self._values)


def fetch_endpoint(
    session: Any,
    host: str,
    endpoint: Mapping[str, Any],
    timeout_s: float,
) -> Any:
    """Run one observation HTTP request and decode per the registry hint.

    Returns ``None`` on transient failure (logged at warning) so a single
    bad endpoint doesn't take down ``get_observation()``.
    """
    method = str(endpoint.get("method", "GET"))
    url = f"{host}{endpoint['url']}"
    decode = str(endpoint.get("decode", "json")).lower()

    try:
        resp = session.request(method, url, timeout=timeout_s)
    except Exception as exc:  # noqa: BLE001 — observation polling must not crash the loop
        logger.warning("observation fetch %s failed: %s", url, exc)
        return None

    if resp.status_code >= 400:
        logger.warning("observation fetch %s returned status=%s", url, resp.status_code)
        return None

    if decode == "binary":
        return resp.content
    if decode == "image":
        try:
            from io import BytesIO  # noqa: PLC0415

            from PIL import Image  # noqa: PLC0415

            return Image.open(BytesIO(resp.content)).convert("RGB")
        except Exception as exc:  # noqa: BLE001
            logger.warning("observation image decode failed for %s: %s", url, exc)
            return None

    # Default: JSON.
    try:
        body = resp.json()
    except Exception as exc:  # noqa: BLE001
        logger.warning("observation JSON decode failed for %s: %s", url, exc)
        return None

    json_path = endpoint.get("json_path")
    if json_path:
        body = _apply_jsonpath(body, json_path)
    return body


def _apply_jsonpath(body: Any, expr: str) -> Any:
    """Apply a JSONPath expression. Lazy import — soft dep."""
    try:
        from jsonpath_ng import parse  # type: ignore[import-not-found]  # noqa: PLC0415
    except ImportError:
        logger.warning(
            "jsonpath_ng not installed — install strands-robots[http] for json_path support; "
            "returning full body for expr=%r",
            expr,
        )
        return body

    try:
        matches = [m.value for m in parse(expr).find(body)]
    except Exception as exc:  # noqa: BLE001
        logger.warning("json_path %r failed against body: %s", expr, exc)
        return body

    if not matches:
        return None
    if len(matches) == 1:
        return matches[0]
    return matches
