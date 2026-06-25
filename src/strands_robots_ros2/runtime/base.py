"""RuntimeAdapter ABC — the live execution surface for one robot.

One adapter per live robot connection. ROS2 hardware, ROS2 in Gazebo,
and LeRobot all implement this. Bound to a single ``robot_id``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from typing import Any


class RuntimeAdapter(ABC):
    """One adapter per live robot connection. ROS2 hardware, ROS2 in Gazebo,
    and LeRobot all implement this. Bound to a single robot_id."""

    robot_id: str
    world_id: str | None  # None for hardware; set for sim-hosted runtimes

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def is_connected(self) -> bool: ...

    # --- Introspection (mandatory in v1) ---
    @abstractmethod
    def observation_schema(self) -> Mapping[str, Any]:
        """Per-field shapes and dtypes; e.g. {'image_front': {'shape':[H,W,3],'dtype':'uint8'},
        'joint_pos': {'shape':[N],'dtype':'float32'}}."""

    @abstractmethod
    def action_schema(self) -> Mapping[str, Any]:
        """Structured dict-of-fields with constraints. Examples:
        differential: {'linear_x':{'min':-0.5,'max':0.5,'unit':'m/s'},
                       'angular_z':{'min':-1.5,'max':1.5,'unit':'rad/s'}}
        ackermann:    {'linear_x':{...}, 'angular_z':{...},
                       'wheelbase':{'value':0.16,'unit':'m'},
                       'min_turning_radius':{'value':0.28,'unit':'m'}}
        humanoid_28j: {'joint_pos':{'shape':[28],'min':[...],'max':[...]},
                       'gripper_l':{...}, 'gripper_r':{...}}."""

    @abstractmethod
    def safety_limits(self) -> Mapping[str, Any]:
        """Hard caps enforced at the adapter boundary. Includes max_linear_x, max_angular_z,
        max_steering, max_duration_per_command, command_watchdog_timeout."""

    @abstractmethod
    def command_rate_hz(self) -> float:
        """Native command publish rate hint; consumers (move loop, PolicyRunner) cap to this."""

    # --- I/O ---
    @abstractmethod
    def get_observation(self) -> Mapping[str, Any]: ...

    @abstractmethod
    def send_action(self, action: Mapping[str, Any]) -> None:
        """Validates against action_schema and safety_limits, applies any required translation
        (e.g. Twist -> ServoCtrlMsg), then publishes."""

    @abstractmethod
    def stop(self) -> None:
        """Idempotent. Publishes a zero command immediately. Cancels any in-flight
        duration-aware move loop."""

    # --- Optional capabilities ---
    def capabilities(self) -> set[str]:
        """Free-form tags consumed by AgentTools and policies, e.g.
        {'twist','tf','laserscan','rgb','navigation','slam'}. Default empty."""
        return set()
