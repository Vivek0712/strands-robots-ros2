"""Runtime factory — registry entry → ``RuntimeAdapter`` instance.

Inspects the registry entry's ``runtime`` field and constructs the
matching adapter:

- ``runtime: lerobot`` (or absent) → :class:`LeRobotRuntime`.
- ``runtime: ros2`` → :class:`Ros2Runtime`.
- ``runtime: http`` → :class:`HttpRuntime`
  (any robot exposing an HTTP/JSON control API; AWS DeepRacer is the
  first concrete consumer).

Unknown values raise ``ValueError``.

For block-driven runtimes (``ros2``, ``http``), the factory merges the
top-level fields into the nested sub-block before constructing the
adapter, so adapters consume a single flat block. This keeps adapter
APIs minimal while letting the registry use the layered shape humans
actually edit.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.base import RuntimeAdapter

# Keys that live at the top level of a registry entry but logically
# belong inside the runtime sub-block.
_RUNTIME_TOP_LEVEL_FIELDS = (
    "drive_type",
    "wheelbase_m",
    "wheel_separation_m",
    "wheel_radius_m",
    "safety_limits",
    "command_rate_hz",
    "capabilities",
)


def build_runtime(
    *,
    robot_id: str,
    registry_entry: Mapping[str, Any],
    robot: Any | None = None,
    cameras: Mapping[str, Any] | None = None,
    world_id: str | None = None,
    **kwargs: Any,
) -> RuntimeAdapter:
    """Construct the right ``RuntimeAdapter`` for ``registry_entry``.

    Args:
        robot_id: Identifier for this live connection (typically the
            ``tool_name`` from the agent layer).
        registry_entry: The ``robots.json`` entry for this robot.
        robot: For LeRobot entries, the robot argument forwarded to
            :class:`LeRobotRuntime` (instance, ``RobotConfig``, or
            type string). Ignored for ROS2 / HTTP entries.
        cameras: For LeRobot entries, the camera config dict.
        world_id: For sim-hosted runtimes, the opaque world identifier.
            ``None`` for hardware.
        **kwargs: Forwarded to the adapter constructor. For HTTP, may
            include ``host``, ``password``, ``token``, ``api_key``,
            ``verify_ssl``, ``max_speed``, ``request_timeout_s``.

    Returns:
        A constructed ``RuntimeAdapter`` ready for ``connect()``.

    Raises:
        ValueError: If ``runtime`` is unrecognized.
    """
    runtime_kind = registry_entry.get("runtime", "lerobot")

    if runtime_kind == "lerobot":
        from strands_robots_ros2.runtime.lerobot import LeRobotRuntime

        return LeRobotRuntime(
            robot_id=robot_id,
            robot=robot,
            cameras=dict(cameras) if cameras else None,
            **kwargs,
        )

    if runtime_kind == "ros2":
        from strands_robots_ros2.runtime.ros2.adapter import Ros2Runtime

        merged = _merge_block(registry_entry, "ros2", _RUNTIME_TOP_LEVEL_FIELDS)
        return Ros2Runtime(
            robot_id=robot_id,
            ros2_block=merged,
            world_id=world_id,
        )

    if runtime_kind == "http":
        from strands_robots_ros2.runtime.http import HttpRuntime

        merged = _merge_block(registry_entry, "http", _RUNTIME_TOP_LEVEL_FIELDS)
        merged = _apply_http_overrides(merged, kwargs)
        return HttpRuntime(
            robot_id=robot_id,
            http_block=merged,
            world_id=world_id,
        )

    if runtime_kind == "deepracer_v2":
        from strands_robots_ros2.runtime.deepracer_v2 import DeepRacerV2Runtime

        merged = _merge_block(registry_entry, "deepracer_v2", _RUNTIME_TOP_LEVEL_FIELDS)
        for k in ("host", "ip", "password", "verify_ssl", "max_speed", "request_timeout_s"):
            if k in kwargs and kwargs[k] is not None:
                merged[k] = kwargs[k]
        return DeepRacerV2Runtime(
            robot_id=robot_id,
            block=merged,
            world_id=world_id,
        )

    if runtime_kind == "rosbridge":
        from strands_robots_ros2.runtime.rosbridge import RosbridgeRuntime

        merged = _merge_block(registry_entry, "rosbridge", _RUNTIME_TOP_LEVEL_FIELDS)
        for k in ("host", "port"):
            if k in kwargs and kwargs[k] is not None:
                merged[k] = kwargs[k]
        return RosbridgeRuntime(
            robot_id=robot_id,
            rosbridge_block=merged,
            world_id=world_id,
        )

    raise ValueError(
        f"Unknown runtime {runtime_kind!r} for robot {robot_id!r}. "
        "Supported: 'lerobot', 'ros2', 'http', 'deepracer_v2', 'rosbridge'."
    )


def _merge_block(
    entry: Mapping[str, Any],
    sub_key: str,
    top_level_fields: tuple[str, ...],
) -> dict[str, Any]:
    """Build a flat block by merging selected top-level fields with the
    named sub-block. Sub-block values win on conflict."""
    nested = dict(entry.get(sub_key, {}) or {})
    merged: dict[str, Any] = {}
    for field in top_level_fields:
        if field in entry:
            merged[field] = entry[field]
    merged.update(nested)
    return merged


def _apply_http_overrides(block: dict[str, Any], kwargs: Mapping[str, Any]) -> dict[str, Any]:
    """Merge ``Robot(**kwargs)`` overrides into the right slots of the
    HTTP block. Kwargs win over registry values; registry wins over env.

    Recognized kwargs: ``host``, ``verify_ssl``, ``request_timeout_s``,
    ``password``, ``token``, ``api_key``, ``max_speed``.
    """
    out = dict(block)

    if "host" in kwargs and kwargs["host"] is not None:
        out["host"] = kwargs["host"]
    if "verify_ssl" in kwargs and kwargs["verify_ssl"] is not None:
        out["verify_ssl"] = kwargs["verify_ssl"]
    if "request_timeout_s" in kwargs and kwargs["request_timeout_s"] is not None:
        out["request_timeout_s"] = kwargs["request_timeout_s"]

    # Auth overrides — slot into the auth sub-block keyed by strategy convention.
    auth = dict(out.get("auth", {}) or {})
    if "password" in kwargs and kwargs["password"] is not None:
        auth["password"] = kwargs["password"]
    if "token" in kwargs and kwargs["token"] is not None:
        auth["token"] = kwargs["token"]
    if "api_key" in kwargs and kwargs["api_key"] is not None:
        auth["key"] = kwargs["api_key"]
    if auth:
        out["auth"] = auth

    # Translator-specific override (DeepRacer's max_speed lives in extra_body).
    if "max_speed" in kwargs and kwargs["max_speed"] is not None:
        cmd = dict(out.get("command_endpoint", {}) or {})
        extra = dict(cmd.get("extra_body", {}) or {})
        extra["max_speed"] = kwargs["max_speed"]
        cmd["extra_body"] = extra
        out["command_endpoint"] = cmd

    return out
