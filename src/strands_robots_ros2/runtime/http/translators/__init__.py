"""Pluggable HTTP payload translators.

Each translator owns 'what JSON does this robot accept for its control
endpoint?'. Receives a canonical action dict (e.g.
``{linear_x, angular_z}`` for mobile, joint-name keyed for arms) and
emits the body the endpoint expects.

Built-in names (use directly in ``http.command_endpoint.translator``):

- ``twist_passthrough``  → forwards ``{linear_x, angular_z}`` as-is
- ``twist_to_servo``     → bicycle-model conversion → ``{angle, throttle}`` (Ackermann)
- ``joint_passthrough``  → joint-name dict → JSON dict (or array via ``joint_order``)

Custom translators use ``module.path:ClassName`` form.
"""

from __future__ import annotations

import importlib
import inspect
from collections.abc import Mapping
from typing import Any

from strands_robots_ros2.runtime.http.translators.base import PayloadTranslator
from strands_robots_ros2.runtime.http.translators.joint_passthrough import JointPassthrough
from strands_robots_ros2.runtime.http.translators.twist_passthrough import TwistPassthrough
from strands_robots_ros2.runtime.http.translators.twist_to_servo import TwistToServo

_BUILTIN_TRANSLATORS: dict[str, type[PayloadTranslator]] = {
    "twist_passthrough": TwistPassthrough,
    "twist_to_servo": TwistToServo,
    "joint_passthrough": JointPassthrough,
}


def resolve_translator(
    name: str | None,
    config: Mapping[str, Any] | None = None,
) -> PayloadTranslator:
    """Build a ``PayloadTranslator`` from the registry's
    ``command_endpoint.translator`` field + optional ``translator_config``.

    Resolution rules:
        1. Built-in name → exact match.
        2. ``module.path:ClassName`` form → ``importlib``.
        3. Otherwise ``ValueError``.
    """
    if not name:
        raise ValueError("command_endpoint.translator is required")

    if name in _BUILTIN_TRANSLATORS:
        return _BUILTIN_TRANSLATORS[name].from_config(dict(config or {}))

    if ":" in name:
        module_path, class_name = name.split(":", 1)
        module = importlib.import_module(module_path)
        cls = getattr(module, class_name)
        if not inspect.isclass(cls) or not issubclass(cls, PayloadTranslator):
            cls_name = getattr(cls, "__name__", repr(cls))
            raise ValueError(f"Translator {name!r} is not a subclass of PayloadTranslator (got {cls_name})")
        return cls.from_config(dict(config or {}))

    raise ValueError(
        f"Unknown translator {name!r}. Built-ins: {sorted(_BUILTIN_TRANSLATORS)}. "
        "For custom translators use 'module.path:ClassName'."
    )


__all__ = [
    "JointPassthrough",
    "PayloadTranslator",
    "TwistPassthrough",
    "TwistToServo",
    "resolve_translator",
]
