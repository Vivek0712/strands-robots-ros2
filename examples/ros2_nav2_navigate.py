# Copyright (c) 2026 Vivek Raja. Original component.
# Licensed under PolyForm Noncommercial License 1.0.0 (see LICENSE-NONCOMMERCIAL.md).
# Commercial use requires the author's written permission. Attribution required (see NOTICE).
"""Autonomous natural-language navigation on a ROS2 rover — Nav2 + live SLAM.

    NL  ->  Strands Agent (Amazon Nova)  ->  navigate_to(x, y)
        ->  Nav2 (path planning + obstacle avoidance) + slam_toolbox (live map)
        ->  ROS2 rover in Gazebo

Prereqs (a ROS2 rover with a lidar): bring up the rover sim, slam_toolbox, and
Nav2, e.g. a TurtleBot4-class rover in Gazebo. Then run this in the ROS2 env
(rclpy + nav2_simple_commander sourced).

    ROBOT=turtlebot4 PROMPT="Navigate to (3,1.5), then report pose" python ros2_nav2_navigate.py
"""

import os

from strands import Agent
from strands.models.bedrock import BedrockModel

from strands_robots_ros2 import Robot
from strands_robots_ros2.skills.nav2 import cancel_navigation, get_pose, navigate_to


def main() -> None:
    robot = Robot(os.environ.get("ROBOT", "turtlebot4"), robot=os.environ.get("ROBOT", "turtlebot4"))
    agent = Agent(
        model=BedrockModel(model_id=os.environ.get("MODEL", "us.amazon.nova-lite-v1:0")),
        tools=[robot, navigate_to, get_pose, cancel_navigation],
    )
    prompt = os.environ.get(
        "PROMPT",
        "Report your pose, then autonomously navigate to (3.0, 1.5) avoiding obstacles, then report your pose again.",
    )
    print(agent(prompt))


if __name__ == "__main__":
    main()
