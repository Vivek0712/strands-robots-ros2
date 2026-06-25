"""ROS2 runtime adapter — generic, registry-driven.

All ``rclpy`` imports inside this package are lazy. Importing
``strands_robots_ros2.runtime.ros2`` does NOT import rclpy; the imports
happen on first use inside ``Ros2Runtime.connect()`` and friends.
"""
