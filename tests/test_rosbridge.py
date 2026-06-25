"""Unit tests for RosbridgeRuntime + factory dispatch + the curiosity registry entry.

``roslibpy`` is faked via sys.modules so the tests need no rosbridge server.
"""

from __future__ import annotations

import sys
import types

import pytest

from strands_robots_ros2.runtime.factory import build_runtime
from strands_robots_ros2.runtime.rosbridge import RosbridgeRuntime

CURIOSITY_BLOCK = {
    "host": "curiosity",
    "port": 9090,
    "command_topic": {"topic": "/curiosity_mars_rover/ackermann_drive_controller/cmd_vel", "type": "geometry_msgs/Twist"},
    "observation_topics": [{"name": "odom", "topic": "/curiosity_mars_rover/odom", "type": "nav_msgs/Odometry"}],
    "safety_limits": {"max_linear_x": 2.0, "max_angular_z": 1.0},
    "command_rate_hz": 10.0,
    "capabilities": ["velocity_control", "odom"],
}


@pytest.fixture
def fake_roslibpy(monkeypatch):
    mod = types.ModuleType("roslibpy")
    published: list = []

    class Ros:
        def __init__(self, host, port):
            self.host, self.port, self.running = host, port, False

        def run(self):
            self.running = True

        def terminate(self):
            self.running = False

    class Topic:
        def __init__(self, ros, name, mtype):
            self.ros, self.name, self.mtype = ros, name, mtype
            self.advertised = False
            self.cb = None

        def advertise(self):
            self.advertised = True

        def unadvertise(self):
            self.advertised = False

        def subscribe(self, cb):
            self.cb = cb

        def unsubscribe(self):
            self.cb = None

        def publish(self, msg):
            published.append((self.name, msg.data))

    class Message:
        def __init__(self, data):
            self.data = data

    mod.Ros, mod.Topic, mod.Message = Ros, Topic, Message
    mod._published = published
    monkeypatch.setitem(sys.modules, "roslibpy", mod)
    return mod


def test_connect_advertises_command_and_subscribes_observations(fake_roslibpy):
    rt = RosbridgeRuntime(robot_id="c", rosbridge_block=CURIOSITY_BLOCK)
    rt.connect()
    assert rt.is_connected()
    assert rt._cmd_topic.advertised
    assert "odom" in rt._subs
    rt.disconnect()
    assert not rt.is_connected()


def test_send_action_clamps_and_publishes_twist(fake_roslibpy):
    rt = RosbridgeRuntime(robot_id="c", rosbridge_block=CURIOSITY_BLOCK)
    rt.connect()
    rt.send_action({"linear_x": 5.0, "angular_z": -3.0})  # both over the safety limits
    name, data = fake_roslibpy._published[-1]
    assert name.endswith("/cmd_vel")
    assert data["linear"]["x"] == 2.0  # clamped to max_linear_x
    assert data["angular"]["z"] == -1.0  # clamped to -max_angular_z


def test_stop_publishes_zero(fake_roslibpy):
    rt = RosbridgeRuntime(robot_id="c", rosbridge_block=CURIOSITY_BLOCK)
    rt.connect()
    rt.stop()
    _, data = fake_roslibpy._published[-1]
    assert data["linear"]["x"] == 0.0 and data["angular"]["z"] == 0.0


def test_send_action_before_connect_raises(fake_roslibpy):
    rt = RosbridgeRuntime(robot_id="c", rosbridge_block=CURIOSITY_BLOCK)
    with pytest.raises(RuntimeError):
        rt.send_action({"linear_x": 1.0, "angular_z": 0.0})


def test_observation_cache_updates_from_subscription(fake_roslibpy):
    rt = RosbridgeRuntime(robot_id="c", rosbridge_block=CURIOSITY_BLOCK)
    rt.connect()
    rt._subs["odom"].cb({"pose": {"pose": {"position": {"x": 1.5}}}})
    assert rt.get_observation()["odom"]["pose"]["pose"]["position"]["x"] == 1.5


def test_schemas_and_introspection(fake_roslibpy):
    rt = RosbridgeRuntime(robot_id="c", rosbridge_block=CURIOSITY_BLOCK)
    assert rt.action_schema()["linear_x"]["max"] == 2.0
    assert rt.action_schema()["angular_z"]["min"] == -1.0
    assert rt.observation_schema()["odom"]["topic"] == "/curiosity_mars_rover/odom"
    assert rt.command_rate_hz() == 10.0
    assert "odom" in rt.capabilities()


def test_factory_dispatches_rosbridge(fake_roslibpy):
    entry = {
        "runtime": "rosbridge",
        "drive_type": "ackermann",
        "command_rate_hz": 10.0,
        "safety_limits": {"max_linear_x": 2.0, "max_angular_z": 1.0},
        "capabilities": ["velocity_control"],
        "rosbridge": CURIOSITY_BLOCK,
    }
    rt = build_runtime(robot_id="c", registry_entry=entry)
    assert isinstance(rt, RosbridgeRuntime)
    # top-level safety_limits merged into the block
    assert rt.safety_limits()["max_linear_x"] == 2.0


def test_curiosity_registry_entry_resolves():
    from strands_robots_ros2.registry.robots import get_robot

    entry = get_robot("curiosity")
    assert entry is not None
    assert entry["runtime"] == "rosbridge"
    assert entry["rosbridge"]["command_topic"]["topic"].endswith("/cmd_vel")
    # aliases resolve to the same entry
    assert get_robot("msl") is not None
    assert get_robot("curiosity_rover") is not None
