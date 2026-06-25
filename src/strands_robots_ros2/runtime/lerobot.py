"""LeRobotRuntime — RuntimeAdapter implementation backed by ``lerobot``.

Wraps a ``lerobot.robots.robot.Robot`` instance. All ``lerobot``
imports are lazy so module load does NOT require ``lerobot`` to be
installed; only paths that actually instantiate from a string or
``RobotConfig`` pull in the dependency.

Construction modes:

- ``robot`` is an already-built ``lerobot.robots.robot.Robot``: stored
  directly. No lerobot factory call.
- ``robot`` is a ``lerobot.robots.config.RobotConfig``: resolved via
  ``lerobot.robots.utils.make_robot_from_config``.
- ``robot`` is a string (e.g. ``"so101_follower"``): a minimal
  per-robot config is constructed from a hardcoded mapping, then
  resolved as above.

This is the LeRobot-only path extracted from the historical
``robot.py``. Behavior is preserved bit-for-bit for existing users; the
runtime adapter shape is the only addition.
"""

from __future__ import annotations

import importlib
import logging
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.base import RuntimeAdapter

logger = logging.getLogger(__name__)


# Mapping from robot-type string to (module_path, config_class_name).
# This is the same mapping that today's robot.py keeps inside
# ``_create_minimal_config``; it lives here so registry-driven creation
# can pick the right config class without code changes elsewhere.
_LEROBOT_CONFIG_MAPPING: dict[str, tuple[str, str]] = {
    "so101_follower": ("lerobot.robots.so101_follower", "SO101FollowerConfig"),
    "so100_follower": ("lerobot.robots.so100_follower", "SO100FollowerConfig"),
    "bi_so100_follower": ("lerobot.robots.bi_so100_follower", "BiSO100FollowerConfig"),
    "viperx": ("lerobot.robots.viperx", "ViperXConfig"),
    "koch_follower": ("lerobot.robots.koch_follower", "KochFollowerConfig"),
}


class LeRobotRuntime(RuntimeAdapter):
    """RuntimeAdapter backed by a ``lerobot`` Robot instance."""

    def __init__(
        self,
        *,
        robot_id: str,
        robot: Any,
        cameras: dict[str, dict[str, Any]] | None = None,
        command_rate_hz: float = 50.0,
        safety_limits: dict[str, Any] | None = None,
        observation_schema: Mapping[str, Any] | None = None,
        action_schema: Mapping[str, Any] | None = None,
        **kwargs: Any,
    ) -> None:
        self.robot_id = robot_id
        self.world_id = None  # always None for hardware-backed runtimes

        self._command_rate_hz = float(command_rate_hz)
        self._safety_limits: dict[str, Any] = {
            "max_duration_per_command": 5.0,
        }
        if safety_limits:
            self._safety_limits.update(safety_limits)

        self._observation_schema: Mapping[str, Any] = observation_schema or {}
        self._action_schema: Mapping[str, Any] = action_schema or {}

        self._robot = self._resolve_robot(robot, cameras, **kwargs)

    # ----- Robot resolution -----

    def _resolve_robot(
        self,
        robot: Any,
        cameras: dict[str, dict[str, Any]] | None,
        **kwargs: Any,
    ) -> Any:
        """Coerce the ``robot`` argument into a ``lerobot.robots.robot.Robot`` instance.

        Discrimination is duck-typed to avoid importing ``lerobot`` on the
        instance-passthrough path:

        * pre-built instance (has ``connect`` + ``send_action`` + ``get_observation``):
          stored directly. No lerobot import.
        * string: minimal config built from the registry mapping, then
          ``make_robot_from_config``. Imports lerobot.
        * everything else: assumed to be a ``lerobot.robots.config.RobotConfig``
          and handed to ``make_robot_from_config``. Imports lerobot.
        """
        if self._looks_like_lerobot_instance(robot):
            return robot

        if isinstance(robot, str):
            from lerobot.robots.utils import make_robot_from_config

            config = self._build_minimal_config(robot, cameras, **kwargs)
            return make_robot_from_config(config)

        # Treat as a RobotConfig — but reject obvious junk first so users
        # get a clear error instead of an opaque lerobot crash.
        if not self._looks_like_lerobot_config(robot):
            raise ValueError(
                f"Unsupported robot type: {type(robot)}. "
                "Expected a lerobot Robot instance, RobotConfig, or robot-type string."
            )

        from lerobot.robots.utils import make_robot_from_config

        return make_robot_from_config(robot)

    @staticmethod
    def _looks_like_lerobot_instance(obj: Any) -> bool:
        return all(
            hasattr(obj, attr) for attr in ("connect", "disconnect", "send_action", "get_observation", "is_connected")
        )

    @staticmethod
    def _looks_like_lerobot_config(obj: Any) -> bool:
        # RobotConfig instances at minimum carry an ``id`` attribute and a
        # configuration shape — they don't expose I/O methods.
        return hasattr(obj, "id") and not hasattr(obj, "send_action")

    def _build_minimal_config(
        self,
        robot_type: str,
        cameras: dict[str, dict[str, Any]] | None,
        **kwargs: Any,
    ) -> Any:
        """Construct a minimal lerobot ``RobotConfig`` for ``robot_type``."""
        from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

        camera_configs: dict[str, Any] = {}
        if cameras:
            for name, cfg in cameras.items():
                kind = cfg.get("type", "opencv")
                if kind != "opencv":
                    raise ValueError(f"Unsupported camera type: {kind}")
                camera_configs[name] = OpenCVCameraConfig(
                    index_or_path=cfg["index_or_path"],
                    fps=cfg.get("fps", 30),
                    width=cfg.get("width", 640),
                    height=cfg.get("height", 480),
                    rotation=cfg.get("rotation", 0),
                    color_mode=cfg.get("color_mode", "rgb"),
                )

        if robot_type not in _LEROBOT_CONFIG_MAPPING:
            raise ValueError(
                f"Unsupported robot type: {robot_type}. Supported: {sorted(_LEROBOT_CONFIG_MAPPING.keys())}"
            )

        module_name, class_name = _LEROBOT_CONFIG_MAPPING[robot_type]
        module = importlib.import_module(module_name)
        ConfigClass = getattr(module, class_name)

        config_data: dict[str, Any] = {
            "id": self.robot_id,
            "cameras": camera_configs,
        }
        if "port" in kwargs:
            config_data["port"] = kwargs["port"]
        for key in ("calibration_dir", "mock", "use_degrees"):
            if key in kwargs:
                config_data[key] = kwargs[key]

        try:
            return ConfigClass(**config_data)
        except Exception as exc:  # noqa: BLE001 — surface library errors verbatim
            raise ValueError(
                f"Failed to create {class_name} for robot type {robot_type!r}: {exc}. Config: {config_data}"
            ) from exc

    # ----- Lifecycle -----

    def connect(self) -> None:
        """Connect to the underlying lerobot device. Idempotent — already-
        connected devices do not raise."""
        if self._robot.is_connected:
            logger.info("%s already connected", self._robot)
            return

        # Lerobot's typed exception is preferred when available, but we
        # also fall back to message-sniffing so the runtime works in
        # environments where ``lerobot.utils.errors`` is not importable
        # (e.g. CI with a pre-built mock instance).
        try:
            from lerobot.utils.errors import DeviceAlreadyConnectedError as _DAC  # type: ignore[import-not-found]

            already_connected_types: tuple[type[Exception], ...] = (_DAC,)
        except ImportError:
            already_connected_types = ()

        try:
            self._robot.connect(False)
        except already_connected_types:
            logger.info("%s was already connected", self._robot)
        except Exception as exc:  # noqa: BLE001 — fall back to message-sniffing
            msg = str(exc).lower()
            if "already connected" not in msg:
                raise

        if not self._robot.is_connected:
            raise RuntimeError(f"Failed to connect to {self._robot}")

        if hasattr(self._robot, "is_calibrated") and not self._robot.is_calibrated:
            raise RuntimeError(f"Robot {self._robot} is not calibrated. Run `lerobot-calibrate` first.")

    def disconnect(self) -> None:
        if hasattr(self._robot, "disconnect"):
            self._robot.disconnect()

    def is_connected(self) -> bool:
        return bool(getattr(self._robot, "is_connected", False))

    def stop(self) -> None:
        """Bring the robot to a safe state. For an arm this is a no-op
        at the runtime layer — the agent's ``stop_task`` handles the
        cancel signal, and disconnect / cleanup happen in lifecycle."""
        # A future enhancement could send a "hold position" action; for
        # now stopping mid-action is handled by the task layer.
        return None

    # ----- I/O -----

    def get_observation(self) -> Mapping[str, Any]:
        return self._robot.get_observation()

    def send_action(self, action: Mapping[str, Any]) -> None:
        self._robot.send_action(action)

    # ----- Introspection -----

    def observation_schema(self) -> Mapping[str, Any]:
        return self._observation_schema

    def action_schema(self) -> Mapping[str, Any]:
        return self._action_schema

    def safety_limits(self) -> Mapping[str, Any]:
        return dict(self._safety_limits)

    def command_rate_hz(self) -> float:
        return self._command_rate_hz

    def capabilities(self) -> set[str]:
        return {"vla_execute"}
