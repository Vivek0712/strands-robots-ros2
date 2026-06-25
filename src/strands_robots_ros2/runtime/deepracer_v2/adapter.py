"""DeepRacerV2Runtime — RuntimeAdapter wrapping the vendored DeepRacer client.

Composition split:

- The ``DeepRacerV2Client`` handles transport (CSRF, auth, JSON PUTs,
  session cookies, AJAX headers) — battle-tested by the community.
- This adapter applies the framework-side discipline that the wrapper
  doesn't: bicycle-model Twist→servo conversion, ``safety_limits``
  clamping, ``capabilities`` introspection, ``observation_schema``,
  trailing zero on stop/disconnect.

``Robot.move(linear_x, angular_z, duration)`` therefore behaves
identically across all our adapters (LeRobot, ROS2, HTTP, V2). The only
thing that changes is the wire format underneath.
"""

from __future__ import annotations

import logging
import os
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.base import RuntimeAdapter
from strands_robots_ros2.runtime.deepracer_v2.client import DeepRacerV2Client
from strands_robots_ros2.runtime.http.translators.twist_to_servo import twist_to_servo_norm

logger = logging.getLogger(__name__)


class DeepRacerV2Runtime(RuntimeAdapter):
    """RuntimeAdapter for AWS DeepRacer via the community console wrapper."""

    def __init__(
        self,
        *,
        robot_id: str,
        block: Mapping[str, Any],
        world_id: str | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.world_id = world_id
        self._block: Mapping[str, Any] = dict(block or {})

        # Resolve host (IP). Accept full URL or bare IP; strip scheme.
        host_env = self._block.get("host_env") or self._block.get("ip_env")
        host = self._block.get("host") or self._block.get("ip") or self._resolve_env(host_env)
        self._host = self._normalize_to_ip(host or "")

        # Resolve password (env or literal).
        password = self._block.get("password") or self._resolve_env(
            self._block.get("password_env", "DEEPRACER_PASSWORD")
        )
        self._password = password or ""

        self._verify_ssl: bool = bool(self._block.get("verify_ssl", False))
        self._timeout = float(self._block.get("request_timeout_s", 10.0))
        self._max_speed = float(self._block.get("max_speed", 1.0))

        self._client: DeepRacerV2Client | None = None
        self._connected: bool = False

    @staticmethod
    def _resolve_env(env_name: Any) -> str:
        if not env_name:
            return ""
        if isinstance(env_name, str):
            return os.environ.get(env_name, "")
        if isinstance(env_name, (list, tuple)):
            for n in env_name:
                if isinstance(n, str) and (val := os.environ.get(n, "")):
                    return val
        return ""

    @staticmethod
    def _normalize_to_ip(value: str) -> str:
        """Accept bare IP or full URL; return just the host:port for the client."""
        if not value:
            return ""
        v = value.strip().rstrip("/")
        if "://" in v:
            v = v.split("://", 1)[1]
        return v

    # ----- Lifecycle -----

    def connect(self) -> None:
        if self._connected:
            return
        if not self._host:
            raise ValueError(
                f"DeepRacerV2Runtime[{self.robot_id}]: host required "
                "(set 'host'/'ip' in registry, or DEEPRACER_HOST/DEEPRACER_IP env)"
            )
        if not self._password:
            raise ValueError(
                f"DeepRacerV2Runtime[{self.robot_id}]: password required "
                "(DEEPRACER_PASSWORD env or 'password' in registry)"
            )

        self._client = DeepRacerV2Client(
            password=self._password,
            ip=self._host,
            verify_ssl=self._verify_ssl,
            timeout_s=self._timeout,
        )
        # The wrapper's login does GET / → parse CSRF → POST /login.
        # set_manual_mode + start_car together replace our drive_mode +
        # start_stop sequence.
        self._client.login()
        self._client.set_manual_mode()
        self._client.start_car()
        self._connected = True
        logger.info(
            "DeepRacerV2Runtime[%s] connected to %s (verify_ssl=%s)",
            self.robot_id,
            self._host,
            self._verify_ssl,
        )

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            # Trailing zero, then stop the car. Idempotent.
            if self._client is not None:
                self._client.move(steering_angle=0.0, throttle=0.0, max_speed=self._max_speed)
                self._client.stop_car()
        except Exception as exc:  # noqa: BLE001 — best-effort teardown
            logger.warning("DeepRacerV2Runtime[%s] disconnect cleanup raised: %s", self.robot_id, exc)
        finally:
            if self._client is not None:
                self._client.close()
            self._client = None
            self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def stop(self) -> None:
        """Publish a zero command. Idempotent."""
        if self._client is None:
            return
        try:
            self._client.move(steering_angle=0.0, throttle=0.0, max_speed=self._max_speed)
        except Exception as exc:  # noqa: BLE001
            logger.warning("DeepRacerV2Runtime[%s] stop raised: %s", self.robot_id, exc)

    # ----- I/O -----

    def get_observation(self) -> Mapping[str, Any]:
        # Battery + USB are cheap polls; expose them as M1 telemetry.
        if self._client is None:
            return {}
        obs: dict[str, Any] = {}
        try:
            obs["battery_level"] = self._client.get_battery_level()
        except Exception as exc:  # noqa: BLE001 — telemetry must not crash the loop
            logger.debug("battery_level fetch failed: %s", exc)
        try:
            obs["usb_connected"] = self._client.get_is_usb_connected()
        except Exception as exc:  # noqa: BLE001
            logger.debug("is_usb_connected fetch failed: %s", exc)
        return obs

    def send_action(self, action: Mapping[str, Any]) -> None:
        if self._client is None:
            raise RuntimeError(f"DeepRacerV2Runtime[{self.robot_id}] is not connected — call connect() first")

        # Accept either canonical Twist or pre-normalized servo shape.
        if "angle" in action and "throttle" in action:
            angle = float(action["angle"])
            throttle = float(action["throttle"])
        else:
            limits = self._block.get("safety_limits", {}) or {}
            wheelbase = float(self._block.get("wheelbase_m", 0.164))
            v_max = float(limits.get("max_linear_x", 1.5))
            delta_max = float(limits.get("max_steering_rad", 0.5236))
            lin = float(action.get("linear_x", 0.0))
            ang = float(action.get("angular_z", 0.0))
            # Clamp Twist before bicycle model to keep safety_limits authoritative.
            max_lin = float(limits.get("max_linear_x", float("inf")))
            max_ang = float(limits.get("max_angular_z", float("inf")))
            lin = max(-max_lin, min(max_lin, lin))
            ang = max(-max_ang, min(max_ang, ang))
            angle, throttle = twist_to_servo_norm(
                linear_x=lin,
                angular_z=ang,
                wheelbase_m=wheelbase,
                max_linear_x=v_max,
                max_steering_rad=delta_max,
            )

        self._client.move(steering_angle=angle, throttle=throttle, max_speed=self._max_speed)

    # ----- Introspection -----

    def safety_limits(self) -> Mapping[str, Any]:
        return dict(self._block.get("safety_limits", {}) or {})

    def command_rate_hz(self) -> float:
        return float(self._block.get("command_rate_hz", 20.0))

    def capabilities(self) -> set[str]:
        caps = self._block.get("capabilities", []) or []
        return set(caps)

    def observation_schema(self) -> Mapping[str, Any]:
        return {
            "battery_level": {"type": "float", "units": "percent"},
            "usb_connected": {"type": "bool"},
        }

    def action_schema(self) -> Mapping[str, Any]:
        limits = self._block.get("safety_limits", {}) or {}
        max_lin = float(limits.get("max_linear_x", float("inf")))
        max_ang = float(limits.get("max_angular_z", float("inf")))
        schema: dict[str, Any] = {
            "linear_x": {"min": -max_lin, "max": max_lin, "unit": "m/s"},
            "angular_z": {"min": -max_ang, "max": max_ang, "unit": "rad/s"},
        }
        wb = self._block.get("wheelbase_m")
        if wb is not None:
            schema["wheelbase"] = {"value": float(wb), "unit": "m"}
        return schema
