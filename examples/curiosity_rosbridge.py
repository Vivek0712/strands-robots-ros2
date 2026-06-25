# Copyright (c) 2026 Vivek Raja. Original component.
# Licensed under PolyForm Noncommercial License 1.0.0 (see LICENSE-NONCOMMERCIAL.md).
# Commercial use requires the author's written permission. Attribution required (see NOTICE).
"""Drive the real NASA Curiosity rover with natural language — via strands-robots-ros2.

Pipeline:
    NL  ->  Strands Agent  ->  strands_robots_ros2.Robot  ->  RosbridgeRuntime
        ->  rosbridge (WebSocket)  ->  Curiosity (ROS1 Gazebo)

The package provides the runtime + the ``curiosity`` registry entry. The Curiosity
**simulation** (ROS1 Noetic + Gazebo + rosbridge_server) runs separately — see
``examples/README.md``. This script only needs to reach the rosbridge WebSocket
(default ``ws://localhost:9090``; override with ``ROS_HOST`` / ``ROS_PORT``).

    pip install "strands-robots-ros2[rosbridge]"
    ROS_HOST=localhost \
      PROMPT="Drive forward 5 seconds, then turn left, then stop." \
      python examples/curiosity_rosbridge.py
"""

import os

from strands import Agent
from strands.models.bedrock import BedrockModel

from strands_robots_ros2 import Robot


def main() -> None:
    # registry "curiosity" -> runtime: rosbridge. host/port override the entry's
    # defaults (handy when rosbridge is on another host / docker service name).
    rover = Robot(
        "curiosity",
        robot="curiosity",
        host=os.environ.get("ROS_HOST", "localhost"),
        port=int(os.environ.get("ROS_PORT", "9090")),
    )

    agent = Agent(
        model=BedrockModel(model_id=os.environ.get("MODEL", "us.amazon.nova-lite-v1:0")),
        tools=[rover],
    )

    prompt = os.environ.get(
        "PROMPT",
        "Drive the Curiosity rover forward at 1.5 m/s for 5 seconds, then stop. Report what you did.",
    )
    print(agent(prompt))


if __name__ == "__main__":
    main()
