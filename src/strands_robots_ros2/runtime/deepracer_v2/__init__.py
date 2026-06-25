"""DeepRacer V2 runtime — wraps the community AWS DeepRacer console client.

The community ``aws_deepracer_control_v2`` package solves auth/CSRF/firmware
quirks against the AWS DeepRacer console API. We vendor a slim version of
its ``Client`` class (CSRF + manual_drive + drive_mode + start_stop +
battery + USB + video stream — strips multipart model upload to avoid
``requests_toolbelt``/``bs4`` deps) and wrap it inside our
``RuntimeAdapter`` so:

- Auth / CSRF / API quirks are handled by the (battle-tested) wrapper.
- Bicycle-model conversion (Twist → angle/throttle) lives in our adapter.
- ``Robot.move`` duration-loop + safety clamp + trailing zero apply uniformly.
- The agent-tool surface (``Robot.move`` / ``Robot.execute``) is the same
  as for any other robot; only the registry's ``runtime`` field changes.

Source the Client class adapts:
    https://github.com/jacobcantwell/aws_deepracer_control_v2
"""

from strands_robots_ros2.runtime.deepracer_v2.adapter import DeepRacerV2Runtime
from strands_robots_ros2.runtime.deepracer_v2.client import DeepRacerV2Client, DeepRacerV2Error

__all__ = ["DeepRacerV2Runtime", "DeepRacerV2Client", "DeepRacerV2Error"]
