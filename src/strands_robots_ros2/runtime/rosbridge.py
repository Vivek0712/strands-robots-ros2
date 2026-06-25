# Copyright (c) 2026 Vivek Raja. Original component.
# Licensed under PolyForm Noncommercial License 1.0.0 (see LICENSE-NONCOMMERCIAL.md).
# Commercial use requires the author's written permission. Attribution required (see NOTICE).
"""RosbridgeRuntime — drive a robot over a ``rosbridge`` WebSocket via ``roslibpy``.

Lets the framework reach robots the DDS-based :class:`Ros2Runtime` can't —
notably **ROS1** robots (e.g. the NASA Curiosity Gazebo sim, which is ROS1
Noetic) — without rclpy. The runtime connects to a ``rosbridge_server``
WebSocket, publishes a velocity ``Twist`` to the registry-declared command
topic, and caches subscribed observation topics. Adding a rosbridge robot is
a JSON registry entry; no Python change.

The same flat ``{linear_x, angular_z}`` action contract as the other mobile
runtimes is honored, so ``Robot.move(...)`` drives a rosbridge robot exactly
like a ROS2 one.

``roslibpy`` is imported lazily inside :meth:`connect` — importing this module
does not require it (install via the ``rosbridge`` extra).
"""

from __future__ import annotations

import logging
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.base import RuntimeAdapter

logger = logging.getLogger(__name__)


class RosbridgeRuntime(RuntimeAdapter):
    """RuntimeAdapter backed by a rosbridge WebSocket (roslibpy client)."""

    def __init__(
        self,
        *,
        robot_id: str,
        rosbridge_block: Mapping[str, Any],
        world_id: str | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.world_id = world_id
        self._block: Mapping[str, Any] = rosbridge_block

        self._ros: Any = None
        self._cmd_topic: Any = None
        self._cmd_msg_type: str = "geometry_msgs/Twist"
        self._subs: dict[str, Any] = {}
        self._latest_messages: dict[str, Any] = {}
        self._connected: bool = False

    # ----- Lifecycle -----

    def connect(self) -> None:
        if self._connected:
            return

        import roslibpy

        host = str(self._block.get("host", "localhost"))
        port = int(self._block.get("port", 9090))
        self._ros = roslibpy.Ros(host=host, port=port)
        self._ros.run()

        cmd = self._block.get("command_topic")
        if not cmd or "topic" not in cmd:
            raise ValueError(f"rosbridge block for {self.robot_id!r} declares no command_topic.topic")
        self._cmd_msg_type = str(cmd.get("type", "geometry_msgs/Twist"))
        self._cmd_topic = roslibpy.Topic(self._ros, cmd["topic"], self._cmd_msg_type)
        self._cmd_topic.advertise()

        for entry in self._block.get("observation_topics", []) or []:
            name = entry["name"]
            topic = roslibpy.Topic(self._ros, entry["topic"], entry["type"])

            def _cb(msg: Any, _name: str = name) -> None:
                self._latest_messages[_name] = msg

            topic.subscribe(_cb)
            self._subs[name] = topic

        self._connected = True
        logger.info("RosbridgeRuntime[%s] connected (ws://%s:%d)", self.robot_id, host, port)

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            if self._cmd_topic is not None:
                self._cmd_topic.unadvertise()
            for topic in self._subs.values():
                topic.unsubscribe()
            if self._ros is not None:
                self._ros.terminate()
        except Exception as exc:  # noqa: BLE001
            logger.warning("RosbridgeRuntime[%s] teardown raised: %s", self.robot_id, exc)
        self._ros = None
        self._cmd_topic = None
        self._subs = {}
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def stop(self) -> None:
        """Publish a safe-state zero command. Idempotent."""
        if self._cmd_topic is None:
            return
        self._cmd_topic.publish(self._build_twist({"linear_x": 0.0, "angular_z": 0.0}))

    # ----- I/O -----

    def get_observation(self) -> Mapping[str, Any]:
        return dict(self._latest_messages)

    def send_action(self, action: Mapping[str, Any]) -> None:
        if self._cmd_topic is None:
            raise RuntimeError(f"RosbridgeRuntime[{self.robot_id}] is not connected — call connect() first")
        clamped = self._clamp_action(action)
        self._cmd_topic.publish(self._build_twist(clamped))

    # ----- Introspection -----

    def safety_limits(self) -> Mapping[str, Any]:
        return dict(self._block.get("safety_limits", {}))

    def command_rate_hz(self) -> float:
        return float(self._block.get("command_rate_hz", 10.0))

    def capabilities(self) -> set[str]:
        return set(self._block.get("capabilities", []) or [])

    def observation_schema(self) -> Mapping[str, Any]:
        return {
            entry["name"]: {"type": entry["type"], "topic": entry["topic"]}
            for entry in self._block.get("observation_topics", []) or []
        }

    def action_schema(self) -> Mapping[str, Any]:
        limits = self._block.get("safety_limits", {}) or {}
        max_lin = float(limits.get("max_linear_x", float("inf")))
        max_ang = float(limits.get("max_angular_z", float("inf")))
        return {
            "linear_x": {"min": -max_lin, "max": max_lin, "unit": "m/s"},
            "angular_z": {"min": -max_ang, "max": max_ang, "unit": "rad/s"},
        }

    # ----- Action plumbing -----

    def _clamp_action(self, action: Mapping[str, Any]) -> dict[str, Any]:
        limits = self._block.get("safety_limits", {}) or {}
        lin = float(action.get("linear_x", 0.0))
        ang = float(action.get("angular_z", 0.0))
        max_lin = float(limits.get("max_linear_x", float("inf")))
        max_ang = float(limits.get("max_angular_z", float("inf")))
        return {
            "linear_x": max(-max_lin, min(max_lin, lin)),
            "angular_z": max(-max_ang, min(max_ang, ang)),
        }

    def _build_twist(self, action: Mapping[str, Any]) -> Any:
        """Build a rosbridge JSON ``geometry_msgs/Twist`` message."""
        import roslibpy

        return roslibpy.Message(
            {
                "linear": {"x": float(action["linear_x"]), "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(action["angular_z"])},
            }
        )
