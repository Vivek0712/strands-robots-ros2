# Examples

## `curiosity_rosbridge.py` — NL-drive the real NASA Curiosity rover

Drives the authentic Curiosity rover (rocker-bogie) on Mars terrain, in natural
language, **through this package's framework**:

```
NL → Strands Agent → strands_robots_ros2.Robot → RosbridgeRuntime → rosbridge → Curiosity
```

What lives where:
- **In this package:** `RosbridgeRuntime` (generic) + the `curiosity` registry entry + this example.
- **External (NOT in this package):** the Curiosity *simulation* — ROS1 Noetic + Gazebo +
  `rosbridge_server`. It's a third-party, ROS1, unlicensed asset, so it is not vendored here.

### 1. Stand up the Curiosity sim (external, one-time)

Use the [`mark-gl/curiosity_mars_rover_ws`](https://github.com/mark-gl/curiosity_mars_rover_ws)
sim in a ROS1 Noetic + Gazebo Classic environment. Build the driving subset and add rosbridge:

```bash
# in a ROS1 Noetic container/host:
catkin_make -DCATKIN_WHITELIST_PACKAGES="ackermann_drive_controller;curiosity_mars_rover_description;curiosity_mars_rover_control;curiosity_mars_rover_gazebo"
roslaunch curiosity_mars_rover_gazebo main_mars_terrain.launch rviz:=false   # DISPLAY=:99 if headless
apt-get install -y ros-noetic-rosbridge-suite
roslaunch rosbridge_server rosbridge_websocket.launch                        # WebSocket on :9090
```

> A ready-made Docker harness for exactly this lives in the `strands-robots-curiosity`
> workspace (Noetic + Gazebo + noVNC + rosbridge + a py3.12 agent container).

### 2. Run the agent (this package)

```bash
pip install "strands-robots-ros2[rosbridge]"
# AWS creds for Bedrock (or edit the model in the script)
ROS_HOST=localhost \
  PROMPT="Drive forward 5 seconds, then turn left, then stop." \
  python curiosity_rosbridge.py
```

Set `ROS_HOST` to the rosbridge host (e.g. a docker service name `curiosity` when the
agent runs in the same compose network). The rover drives; watch it in the sim's GUI.

### Generalizes
Swap `robot="curiosity"` for any mobile entry in `registry/robots.json`
(`turtlebot4`, `aws_deepracer`). Adding another rosbridge/ROS2/HTTP robot is a
registry-JSON change — no new code.
