"""Translator plugins for ROS2 robots whose command surface differs from
the canonical action shape (e.g. Twist on /cmd_vel).

A translator implements ``convert(action, robot_params) -> Any`` and
declares ``src_type`` / ``dst_type`` strings. The Ros2 adapter resolves
the translator class via ``importlib.import_module`` against the
registry's ``translator: "module:Class"`` field.
"""
