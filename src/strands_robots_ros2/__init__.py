"""strands-robots-ros2 — ROS2 / HTTP / rosbridge mobile-robot runtimes for Strands.

Extension package for upstream ``strands-robots``. It adds the ``RuntimeAdapter``
layer (``ros2``, ``http``, ``rosbridge``, ``deepracer_v2``) plus mobile registry
entries, so a Strands agent can drive **mobile** robots — which upstream
``strands-robots`` (LeRobot manipulation + sim) does not cover.

Flagship: drive the real NASA Curiosity rover (ROS1, over rosbridge) with

    from strands_robots_ros2 import Robot
    rover = Robot("curiosity", robot="curiosity")   # -> RosbridgeRuntime
    rover.move(linear_x=1.5, angular_z=0.2, duration=4.0)

``Robot`` is loaded lazily so runtime/registry use does not require
``strands-agents`` to be importable.
"""

from strands_robots_ros2.registry import get_robot
from strands_robots_ros2.runtime import RuntimeAdapter, build_runtime

__all__ = ["Robot", "RuntimeAdapter", "build_runtime", "get_robot"]
__version__ = "0.1.0"


def __getattr__(name: str):  # PEP 562 lazy attribute
    if name == "Robot":
        from strands_robots_ros2.robot import Robot

        return Robot
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
