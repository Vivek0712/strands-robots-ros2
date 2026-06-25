"""Ros2Runtime — generic, registry-driven ROS2 adapter.

Reads its topic surface from the registry's ``ros2`` block. Adding a
new ROS2 robot is a JSON entry; no Python code change. The same
adapter is reused for physical hardware (LAN-connected DeepRacer /
TurtleBot) and for sim-hosted ROS2 robots (TurtleBot in Gazebo via
``ros_gz_bridge``-exposed topics).

All ``rclpy`` imports are lazy. Importing this module does NOT pull
in ``rclpy``; the imports happen on first use inside :meth:`connect`
and the helper methods it calls.
"""

from __future__ import annotations

import importlib
import logging
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.base import RuntimeAdapter

logger = logging.getLogger(__name__)


class Ros2Runtime(RuntimeAdapter):
    """RuntimeAdapter backed by a generic ROS2 pub/sub surface."""

    def __init__(
        self,
        *,
        robot_id: str,
        ros2_block: Mapping[str, Any],
        world_id: str | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.world_id = world_id
        self._ros2_block: Mapping[str, Any] = ros2_block

        self._node: Any = None
        self._command_pub: Any = None
        self._command_msg_type: Any = None
        self._chosen_interface: str | None = None
        self._translator: Any = None
        self._subs: dict[str, Any] = {}
        self._latest_messages: dict[str, Any] = {}
        self._connected: bool = False

        # Test hook: pre-pin the interface choice without going through DDS
        # discovery. Set to "preferred" or "fallback" before calling
        # connect(). When None, the runtime defaults to "preferred".
        self._force_interface_choice: str | None = None

    # ----- Lifecycle -----

    def connect(self) -> None:
        if self._connected:
            return

        from rclpy.node import Node

        from strands_robots_ros2.runtime.ros2.lifecycle import Ros2Lifecycle

        lifecycle = Ros2Lifecycle.get_instance()
        namespace = str(self._ros2_block.get("namespace", "") or "")
        node_name = f"strands_{self.robot_id}".replace("-", "_")
        self._node = Node(node_name, namespace=namespace, context=lifecycle.context)
        lifecycle.add_node(self._node)

        self._setup_command_publisher()
        self._setup_subscriptions()
        self._maybe_call_enable_state()

        self._connected = True
        logger.info(
            "Ros2Runtime[%s] connected (interface=%s, namespace=%r)",
            self.robot_id,
            self._chosen_interface,
            namespace,
        )

    def disconnect(self) -> None:
        if not self._connected:
            return

        from strands_robots_ros2.runtime.ros2.lifecycle import Ros2Lifecycle

        lifecycle = Ros2Lifecycle.get_instance()
        if self._node is not None:
            try:
                lifecycle.remove_node(self._node)
                self._node.destroy_node()
            except Exception as exc:  # noqa: BLE001
                logger.warning("Node teardown raised: %s", exc)

        self._node = None
        self._command_pub = None
        self._subs = {}
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def stop(self) -> None:
        """Publish a safe-state zero command. Idempotent."""
        if self._command_pub is None:
            return
        zero = {"linear_x": 0.0, "angular_z": 0.0}
        msg = self._build_outbound_msg(zero)
        self._command_pub.publish(msg)

    # ----- I/O -----

    def get_observation(self) -> Mapping[str, Any]:
        return dict(self._latest_messages)

    def send_action(self, action: Mapping[str, Any]) -> None:
        if self._command_pub is None:
            raise RuntimeError(f"Ros2Runtime[{self.robot_id}] is not connected — call connect() first")
        clamped = self._clamp_action(action)
        msg = self._build_outbound_msg(clamped)
        self._command_pub.publish(msg)

    # ----- Introspection -----

    def safety_limits(self) -> Mapping[str, Any]:
        return dict(self._ros2_block.get("safety_limits", {}))

    def command_rate_hz(self) -> float:
        return float(self._ros2_block.get("command_rate_hz", 20.0))

    def capabilities(self) -> set[str]:
        caps = self._ros2_block.get("capabilities", []) or []
        return set(caps)

    def observation_schema(self) -> Mapping[str, Any]:
        return {
            entry["name"]: {"type": entry["type"], "topic": entry["topic"]}
            for entry in self._ros2_block.get("observation_topics", []) or []
        }

    def action_schema(self) -> Mapping[str, Any]:
        limits = self._ros2_block.get("safety_limits", {}) or {}
        max_lin = float(limits.get("max_linear_x", float("inf")))
        max_ang = float(limits.get("max_angular_z", float("inf")))
        schema: dict[str, Any] = {
            "linear_x": {"min": -max_lin, "max": max_lin, "unit": "m/s"},
            "angular_z": {"min": -max_ang, "max": max_ang, "unit": "rad/s"},
        }
        if self._ros2_block.get("drive_type") == "ackermann":
            wb = self._ros2_block.get("wheelbase_m")
            if wb is not None:
                schema["wheelbase"] = {"value": float(wb), "unit": "m"}
            delta_max = limits.get("max_steering_rad")
            if delta_max and wb:
                import math

                R_min = float(wb) / math.tan(float(delta_max))
                schema["min_turning_radius"] = {"value": R_min, "unit": "m"}
        return schema

    # ----- Setup helpers -----

    def _setup_command_publisher(self) -> None:
        interfaces = self._ros2_block.get("command_interfaces") or []
        if not interfaces:
            raise ValueError(f"ros2 block for {self.robot_id!r} declares no command_interfaces")
        # Phase 1 supports a single command interface (the velocity field).
        cmd_iface = interfaces[0]
        choice = self._force_interface_choice or self._probe_interface(cmd_iface)
        chosen = cmd_iface.get(choice)
        if chosen is None:
            # Fallback to preferred when the requested choice isn't declared.
            choice = "preferred"
            chosen = cmd_iface[choice]

        self._chosen_interface = choice
        msg_type = self._import_msg_type(chosen["type"])
        self._command_msg_type = msg_type

        qos = self._make_qos_profile(chosen.get("qos", "default"))
        self._command_pub = self._node.create_publisher(msg_type, chosen["topic"], qos)

        translator_path = chosen.get("translator")
        self._translator = self._load_translator(translator_path) if translator_path else None

    def _probe_interface(self, cmd_iface: Mapping[str, Any]) -> str:
        """Pick between ``preferred`` and ``fallback`` interfaces.

        Phase 1 always returns ``preferred`` unless the registry omits it.
        Real DDS probing (count subscribers on the preferred topic with
        a configurable wait window) is a v1.1 enhancement; it slots in
        here without changing the call site.
        """
        if cmd_iface.get("preferred") is None and cmd_iface.get("fallback") is not None:
            return "fallback"
        return "preferred"

    def _setup_subscriptions(self) -> None:
        for entry in self._ros2_block.get("observation_topics", []) or []:
            name = entry["name"]
            msg_type = self._import_msg_type(entry["type"])
            qos = self._make_qos_profile(entry.get("qos", "default"))

            def _callback(msg: Any, _name: str = name) -> None:
                self._latest_messages[_name] = msg

            sub = self._node.create_subscription(msg_type, entry["topic"], _callback, qos)
            self._subs[name] = sub

    def _maybe_call_enable_state(self) -> None:
        """Call the registry-declared init services so the device accepts
        external commands.

        For DeepRacer the sequence the AWS web UI does on its "Manual
        drive" tab is:

            1. ``/ctrl_pkg/vehicle_state``  (ActiveStateSrv) — switch to
               manual mode (state=1).
            2. ``/ctrl_pkg/enable_state``   (EnableStateSrv) — activate
               the now-selected mode.

        Both are optional in the registry — adapters that don't need
        them simply omit the ``vehicle_state_service`` /
        ``enable_state_service`` fields.
        """
        # 1. vehicle_state — switch the device into the right mode.
        self._call_init_service(
            srv_name=self._ros2_block.get("vehicle_state_service"),
            srv_type_str=self._ros2_block.get("vehicle_state_service_type"),
            request_kwargs={"state": int(self._ros2_block.get("vehicle_state_target_mode", 1))},
        )

        # 2. enable_state — activate it.
        self._call_init_service(
            srv_name=self._ros2_block.get("enable_state_service"),
            srv_type_str=self._ros2_block.get("enable_state_service_type"),
            request_kwargs={"is_active": True},
        )

    def _call_init_service(
        self,
        srv_name: str | None,
        srv_type_str: str | None,
        request_kwargs: dict[str, Any],
    ) -> None:
        """Best-effort one-shot service call used at connect-time."""
        if not srv_name:
            return

        try:
            srv_type: Any = self._import_msg_type(srv_type_str) if srv_type_str else type("_PlaceholderSrv", (), {})
        except Exception as exc:  # noqa: BLE001
            logger.warning("Unable to import service type %r: %s", srv_type_str, exc)
            srv_type = type("_PlaceholderSrv", (), {})

        client = self._node.create_client(srv_type, srv_name)
        try:
            client.wait_for_service(timeout_sec=2.0)
        except Exception as exc:  # noqa: BLE001
            logger.warning("wait_for_service %r raised: %s", srv_name, exc)

        # Build the request from the real Request class when available;
        # fall back to a duck-typed object for tests / placeholder types.
        request_cls = getattr(srv_type, "Request", None)
        if request_cls is not None:
            try:
                request = request_cls()
            except Exception:  # noqa: BLE001
                request = type("_Req", (), {})()
        else:
            request = type("_Req", (), {})()
        for k, v in request_kwargs.items():
            try:
                setattr(request, k, v)
            except Exception:  # noqa: BLE001 — placeholder request may be frozen
                pass

        try:
            client.call(request)
        except Exception as exc:  # noqa: BLE001
            logger.warning("service call %r raised: %s", srv_name, exc)

    # ----- Action plumbing -----

    def _clamp_action(self, action: Mapping[str, Any]) -> dict[str, Any]:
        limits = self._ros2_block.get("safety_limits", {}) or {}
        lin = float(action.get("linear_x", 0.0))
        ang = float(action.get("angular_z", 0.0))
        max_lin = float(limits.get("max_linear_x", float("inf")))
        max_ang = float(limits.get("max_angular_z", float("inf")))
        return {
            "linear_x": max(-max_lin, min(max_lin, lin)),
            "angular_z": max(-max_ang, min(max_ang, ang)),
        }

    def _build_outbound_msg(self, action: Mapping[str, Any]) -> Any:
        if self._translator is not None:
            return self._translator.convert(action, self._translator_params())
        return self._build_twist(action)

    def _translator_params(self) -> dict[str, Any]:
        return {
            "wheelbase_m": self._ros2_block.get("wheelbase_m"),
            "safety_limits": dict(self._ros2_block.get("safety_limits", {})),
        }

    def _build_twist(self, action: Mapping[str, Any]) -> Any:
        msg = self._command_msg_type()
        # Twist has linear (Vector3) and angular (Vector3); TwistStamped
        # nests a Twist under .twist. We support both shapes.
        target = msg
        if hasattr(msg, "twist"):
            target = msg.twist
        target.linear.x = float(action["linear_x"])
        target.angular.z = float(action["angular_z"])
        return msg

    # ----- Lazy imports -----

    @staticmethod
    def _import_msg_type(type_str: str) -> Any:
        """Resolve a canonical ROS2 type string ``pkg/msg/MsgType`` (or
        ``pkg/srv/SrvType``) to the actual class via ``importlib``."""
        parts = type_str.split("/")
        if len(parts) != 3:
            raise ValueError(f"Invalid ROS2 type identifier {type_str!r}; expected 'pkg/msg/Type' or 'pkg/srv/Type'")
        pkg, kind, name = parts
        module = importlib.import_module(f"{pkg}.{kind}")
        return getattr(module, name)

    @staticmethod
    def _load_translator(path: str) -> Any:
        """Resolve ``module.path:ClassName`` into an instantiated translator."""
        if ":" not in path:
            raise ValueError(f"Translator path {path!r} must be 'module.path:ClassName'")
        module_path, class_name = path.split(":", 1)
        module = importlib.import_module(module_path)
        cls = getattr(module, class_name)
        return cls()

    @staticmethod
    def _make_qos_profile(name: str) -> Any:
        """Return a ``QoSProfile`` for the named profile.

        Recognised profile names:

        - ``"default"``      — reliable, volatile, keep_last(10)
        - ``"command"``      — reliable, volatile, keep_last(1)
          (publishers like cmd_vel that only care about the latest setpoint)
        - ``"sensor"``       — best_effort, volatile, keep_last(5)
          (matches ``rmw_qos_profile_sensor_data`` upstream)
        - ``"tf_static"``    — reliable, transient_local, keep_last(100)
          (matches ``/tf_static`` publishers; required for late joiners)

        Unknown names fall back to ``"default"`` rather than raising — a
        registry typo logs a warning at first publish, not a crash.
        """
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

        profiles = {
            "default": dict(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
            "command": dict(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
            "sensor": dict(
                depth=5,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
            "tf_static": dict(
                depth=100,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        }
        kwargs = profiles.get(name) or profiles["default"]
        return QoSProfile(**kwargs)
