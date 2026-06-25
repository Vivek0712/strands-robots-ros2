# Copyright (c) 2026 Vivek Raja. Original component.
# Licensed under PolyForm Noncommercial License 1.0.0 (see LICENSE-NONCOMMERCIAL.md).
# Commercial use requires the author's written permission. Attribution required (see NOTICE).
"""Nav2 navigation skills for ROS2 rovers — autonomous "navigate to (x, y)" + pose.

These are Strands ``@tool``s wrapping ``nav2_simple_commander`` so an agent can
send **autonomous goals** to a Nav2 + SLAM stack (path planning, obstacle
avoidance, live mapping) — beyond the open-loop ``Robot.move`` primitive. Hand
them to an agent alongside (or instead of) a ``Robot``:

    from strands import Agent
    from strands_robots_ros2.skills.nav2 import navigate_to, get_pose, cancel_navigation

    Agent(tools=[navigate_to, get_pose, cancel_navigation])(
        "Navigate to (3.0, 1.5), then report your pose."
    )

``rclpy`` and ``nav2_simple_commander`` come from the ROS2 environment (install
the ``ros2`` extra / source your ROS2 setup). All imports are lazy.
"""

from __future__ import annotations

import math
import time

from strands import tool

MAP_FRAME = "map"
BASE_FRAME = "base_footprint"  # the frame slam_toolbox / Nav2 track by default

_state: dict = {"nav": None, "node": None, "tf": None}


def _ensure_rclpy():
    import rclpy

    if not rclpy.ok():
        rclpy.init()
    return rclpy


def _navigator():
    if _state["nav"] is None:
        rclpy = _ensure_rclpy()
        from nav2_simple_commander.robot_navigator import BasicNavigator
        from rclpy.parameter import Parameter

        nav = BasicNavigator()
        nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        # Wait for the action server only — NOT waitUntilNav2Active(), whose
        # localizer probe hangs against slam_toolbox in mapping mode.
        if not nav.nav_to_pose_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("Nav2 /navigate_to_pose action server not available after 30s")
        _state["nav"] = nav
    return _state["nav"]


def _tf_node():
    if _state["node"] is None:
        rclpy = _ensure_rclpy()
        import threading

        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener

        node = Node("strands_nav2_pose")
        buf = Buffer()
        TransformListener(buf, node)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        _state["node"], _state["tf"] = node, buf
    return _state["tf"]


@tool
def navigate_to(x: float, y: float, yaw: float = 0.0) -> str:
    """Autonomously navigate to a goal pose via Nav2 (path planning + obstacle avoidance). Blocks until done.

    Args:
        x: goal X in meters, map frame.
        y: goal Y in meters, map frame.
        yaw: goal heading in radians, map frame.
    """
    from geometry_msgs.msg import PoseStamped
    from nav2_simple_commander.robot_navigator import TaskResult

    nav = _navigator()
    goal = PoseStamped()
    goal.header.frame_id = MAP_FRAME
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.z = math.sin(float(yaw) / 2.0)
    goal.pose.orientation.w = math.cos(float(yaw) / 2.0)

    nav.goToPose(goal)
    while not nav.isTaskComplete():
        time.sleep(0.2)

    return {
        TaskResult.SUCCEEDED: f"Arrived at ({x}, {y}).",
        TaskResult.CANCELED: f"Navigation to ({x}, {y}) was canceled.",
        TaskResult.FAILED: f"Navigation to ({x}, {y}) failed.",
    }.get(nav.getResult(), "Navigation finished with an unknown result.")


@tool
def cancel_navigation() -> str:
    """Cancel the current Nav2 goal immediately."""
    if _state["nav"] is None:
        return "No active navigation to cancel."
    _state["nav"].cancelTask()
    return "Navigation canceled."


@tool
def get_pose() -> str:
    """Return the rover's current pose (x, y, yaw) in the map frame (from the SLAM/TF tree)."""
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

    buf = _tf_node()
    try:
        tf = buf.lookup_transform(MAP_FRAME, BASE_FRAME, Time(), timeout=Duration(seconds=2.0))
    except (LookupException, ConnectivityException, ExtrapolationException) as exc:
        return f"Pose unavailable (TF {MAP_FRAME}->{BASE_FRAME}): {exc}"
    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return f"Pose: x={t.x:.2f} y={t.y:.2f} yaw={yaw:.2f} rad (map frame)."
