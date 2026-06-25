"""Runtime adapters — the live execution surface for a robot.

A ``RuntimeAdapter`` owns one robot connection. Implementations include
``LeRobotRuntime`` (serial-port LeRobot hardware) and ``Ros2Runtime``
(any ROS2 robot, physical or sim-hosted via ``ros_gz_bridge``-exposed
topics). The same ``Ros2Runtime`` is reused for hardware and sim-hosted
ROS2 robots — only the medium differs, the adapter does not.

The ``Robot(AgentTool)`` class composes one ``RuntimeAdapter`` instance,
chosen at construction time by ``build_runtime`` based on the registry
entry's ``runtime`` field.

Usage::

    from strands_robots_ros2.runtime import RuntimeAdapter, build_runtime

    adapter = build_runtime(robot_id="dr1", registry_entry={...})
    adapter.connect()
    obs = adapter.get_observation()
    adapter.send_action({"linear_x": 0.5, "angular_z": 0.0})
    adapter.disconnect()
"""

from strands_robots_ros2.runtime.base import RuntimeAdapter
from strands_robots_ros2.runtime.factory import build_runtime

__all__ = ["RuntimeAdapter", "build_runtime"]
