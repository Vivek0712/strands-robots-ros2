"""Slim AWS DeepRacer console client — vendored + reduced.

Adapted from the community ``aws_deepracer_control_v2`` package
(https://github.com/jacobcantwell/aws_deepracer_control_v2). Stripped to:

- CSRF + login flow (GET ``/`` for the meta token, POST ``/login``).
- Drive-mode / start-stop / manual_drive (PUT JSON to ``/api/...``).
- Battery + USB telemetry (GET ``/api/...``).
- Video stream URL (raw response object; consumer streams).

Stripped: model upload (requires ``requests_toolbelt`` multipart),
get/set calibration, auto-drive throttle config, model load/list. Add
back when needed; the ``RuntimeAdapter`` wrapper doesn't need them today.

CSRF parsing uses stdlib ``html.parser`` instead of ``bs4`` so we don't
add an extra dep just to find a single ``<meta>`` tag.
"""

from __future__ import annotations

import json
import logging
from html.parser import HTMLParser
from typing import Any

logger = logging.getLogger(__name__)


class DeepRacerV2Error(RuntimeError):
    """Raised when a DeepRacer console request fails."""


class _CsrfMetaParser(HTMLParser):
    """Find ``<meta name="csrf-token" content="...">`` in a Flask page."""

    def __init__(self) -> None:
        super().__init__()
        self.token: str | None = None

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag != "meta":
            return
        d = {k: v for k, v in attrs if v is not None}
        if d.get("name") == "csrf-token":
            self.token = d.get("content")


def _extract_csrf_meta(html_text: str) -> str | None:
    """Return the value of ``<meta name="csrf-token">`` or ``None``."""
    parser = _CsrfMetaParser()
    parser.feed(html_text)
    return parser.token


_DEFAULT_USER_AGENT = (
    "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/76.0.3809.100 Safari/537.36"
)


class DeepRacerV2Client:
    """Console client. Mirrors the community wrapper's call shape so a
    drop-in replacement is plausible if anyone's already integrated it."""

    def __init__(
        self,
        password: str,
        ip: str = "127.0.0.1",
        scheme: str = "https",
        verify_ssl: bool = False,
        timeout_s: float = 10.0,
    ) -> None:
        # Lazy import — keeps this module loadable on hosts without requests.
        import requests  # noqa: PLC0415
        import urllib3  # noqa: PLC0415

        if not verify_ssl:
            urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

        self.password = password
        self.ip = ip
        self.url = f"{scheme}://{ip}".rstrip("/") + "/"
        self.verify_ssl = verify_ssl
        self.timeout_s = float(timeout_s)

        self.session = requests.Session()
        self.session.verify = verify_ssl
        self.csrf_token: str | None = None
        self.headers: dict[str, str] | None = None

    # ----- Lifecycle -----

    def login(self) -> None:
        """GET ``/`` to grab the CSRF meta token, then POST ``/login`` with
        password. Subsequent calls reuse the session cookies + headers."""
        if self.csrf_token:
            return

        import requests  # noqa: PLC0415

        try:
            resp = self.session.get(self.url, verify=self.verify_ssl, timeout=self.timeout_s)
        except requests.exceptions.ConnectionError as exc:
            raise DeepRacerV2Error(f"Cannot reach DeepRacer at {self.url} — {exc.__class__.__name__}: {exc}") from exc

        token = _extract_csrf_meta(resp.text)
        if not token:
            raise DeepRacerV2Error(
                f"CSRF meta tag not found at {self.url}. "
                "DeepRacer firmware variant? Page returned "
                f"status={resp.status_code}, body[:200]={resp.text[:200]!r}"
            )
        self.csrf_token = token

        # Initial login headers — POST /login uses x-www-form-urlencoded.
        # We send the CSRF token via BOTH the header (X-CSRFToken) and the
        # form field (csrf_token) — Flask-WTF default check order is
        # form > header, and some firmware variants disable header lookup.
        # Belt-and-suspenders here costs us nothing.
        self.headers = {
            "X-CSRFToken": self.csrf_token,
            "user-agent": _DEFAULT_USER_AGENT,
            "referer": self.url + "login",
            "origin": self.url.rstrip("/"),
        }
        post = self.session.post(
            self.url + "login",
            data={"password": self.password, "csrf_token": self.csrf_token},
            headers=self.headers,
            verify=self.verify_ssl,
            timeout=self.timeout_s,
            allow_redirects=False,
        )
        if post.status_code not in (200, 302):
            raise DeepRacerV2Error(
                f"Login failed: status={post.status_code}, body[:200]={post.text[:200]!r}"
            )

        # On real login success, webserver_pkg sets the ``deepracer_token``
        # cookie via ``response.set_cookie(...)``. Status 200 alone is not
        # enough — Flask returns 200 on a re-rendered login page too. If
        # the auth cookie is missing, the password / CSRF was rejected.
        if "deepracer_token" not in self.session.cookies:
            cookies_seen = {c.name: c.value[:8] + "..." for c in self.session.cookies}
            raise DeepRacerV2Error(
                "Login response was 200 but the 'deepracer_token' cookie was not set. "
                "Either the chassis password is wrong, or the firmware uses a different "
                f"login mechanism. Cookies returned: {cookies_seen}. "
                f"Login body[:200]={post.text[:200]!r}"
            )

        # AJAX-style headers for subsequent JSON API calls.
        self.headers = {
            "X-CSRFToken": self.csrf_token,
            "user-agent": _DEFAULT_USER_AGENT,
            "referer": self.url + "home",
            "origin": self.url.rstrip("/"),
            "accept-encoding": "gzip, deflate, br",
            "content-type": "application/json;charset=UTF-8",
            "accept": "*/*",
            "sec-fetch-mode": "cors",
            "sec-fetch-site": "same-origin",
            "accept-language": "en-US,en;q=0.9",
            "x-requested-with": "XMLHttpRequest",
        }

    def close(self) -> None:
        try:
            self.session.close()
        except Exception:  # noqa: BLE001 — best-effort
            pass

    # ----- Manual control -----

    def set_manual_mode(self) -> Any:
        self.stop_car()
        return self._put("api/drive_mode", {"drive_mode": "manual"})

    def set_autonomous_mode(self) -> Any:
        self.stop_car()
        return self._put("api/drive_mode", {"drive_mode": "auto"})

    def start_car(self) -> Any:
        return self._put("api/start_stop", {"start_stop": "start"})

    def stop_car(self) -> Any:
        return self._put("api/start_stop", {"start_stop": "stop"})

    def move(self, steering_angle: float, throttle: float, max_speed: float) -> Any:
        """Send one ``ServoCtrlMsg``-shaped command via /api/manual_drive."""
        return self._put(
            "api/manual_drive",
            {
                "angle": float(steering_angle),
                "throttle": float(throttle),
                "max_speed": float(max_speed),
            },
        )

    # ----- Telemetry -----

    def get_battery_level(self) -> Any:
        return self._get("api/get_battery_level")

    def get_is_usb_connected(self) -> Any:
        return self._get("api/is_usb_connected")

    def get_raw_video_stream(self, width: int = 480, height: int = 360) -> Any:
        """Return a streaming Response for the MJPEG topic. Caller iterates
        ``response.iter_content(...)`` and decodes the MJPEG framing.

        Used by M5 (vision-LLM) — not consumed by ``Robot.move``.
        """
        self.login()
        video_url = self.url + f"route?topic=/display_mjpeg&width={width}&height={height}"
        return self.session.get(
            video_url,
            headers=self.headers,
            stream=True,
            verify=self.verify_ssl,
            timeout=self.timeout_s,
        )

    # ----- Helpers -----

    def _get(self, path: str, check_status: bool = True) -> Any:
        self.login()
        resp = self.session.get(
            self.url + path,
            headers=self.headers,
            verify=self.verify_ssl,
            timeout=self.timeout_s,
        )
        if check_status and resp.status_code != 200:
            raise DeepRacerV2Error(f"GET {path} failed: status={resp.status_code}, body[:200]={resp.text[:200]!r}")
        try:
            return json.loads(resp.text)
        except json.JSONDecodeError as exc:
            raise DeepRacerV2Error(f"GET {path} returned non-JSON body[:200]={resp.text[:200]!r}") from exc

    def _put(self, path: str, data: Any, check_success: bool = True) -> Any:
        self.login()
        resp = self.session.put(
            self.url + path,
            json=data,
            headers=self.headers,
            verify=self.verify_ssl,
            timeout=self.timeout_s,
        )
        if check_success:
            ok_status = resp.status_code == 200
            ok_body = '"success": true' in resp.text or '"success":true' in resp.text
            if not (ok_status and ok_body):
                raise DeepRacerV2Error(f"PUT {path} failed: status={resp.status_code}, body[:200]={resp.text[:200]!r}")
        try:
            return json.loads(resp.text)
        except json.JSONDecodeError:
            return {}
