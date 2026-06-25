"""Generic, registry-driven HTTP runtime adapter.

Any robot exposing an HTTP/JSON control API becomes a ``robots.json``
entry plus (optionally) a small ``AuthStrategy`` and a small
``PayloadTranslator`` plug-in. ``DeepRacerHttpRuntime`` is the first
concrete consumer — it collapses to a JSON entry once we add the
``runtime: "http"`` factory branch.

Usage from the registry::

    "<robot>": {
      "runtime": "http",
      "http": {
        "host": "https://1.2.3.4",
        "auth": {"strategy": "cookie_password", ...},
        "command_endpoint": {"url": "/api/manual_drive", "translator": "twist_to_servo"},
        ...
      }
    }
"""

from strands_robots_ros2.runtime.http.adapter import HttpRuntime

__all__ = ["HttpRuntime"]
