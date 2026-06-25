#!/usr/bin/env python3
"""Universal Robot Control with RuntimeAdapter composition.

The ``Robot(AgentTool)`` class is the agent-facing entry point. It
composes one ``RuntimeAdapter`` (LeRobot or ROS2), an optional
``Policy`` instance for VLA execution, and an async task manager for
non-blocking ``execute(...)`` / cooperative ``stop()``.

Backward compatibility:

* ``from strands_robots import Robot`` resolves to this class — unchanged.
* ``Robot(tool_name="alice", robot="so101_follower", cameras={...},
  port="/dev/ttyACM0", data_config="...")`` continues to behave as
  before; the LeRobot path is preserved bit-for-bit.
* The ``execute / start / status / stop`` tool actions still work the
  same way for existing GR00T / lerobot_local Policy users.

What's new:

* The class no longer talks to ``lerobot`` directly. All hardware I/O
  goes through ``self._runtime``, picked by ``runtime.factory.build_runtime``
  based on the registry entry's ``runtime`` field.
* The ``tool_spec`` advertises a fixed action surface that includes
  primitive velocity control (``move``) alongside the legacy VLA
  actions, so the LLM can drive a ROS2 mobile robot like AWS
  DeepRacer with no skill abstraction in between.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections.abc import AsyncGenerator
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
from typing import Any, cast

from strands.tools.tools import AgentTool
from strands.types._events import ToolResultEvent
from strands.types.tools import ToolResult, ToolSpec, ToolUse

from strands_robots_ros2.registry import get_robot
from strands_robots_ros2.runtime import RuntimeAdapter, build_runtime

logger = logging.getLogger(__name__)


class TaskStatus(Enum):
    """Robot task execution status."""

    IDLE = "idle"
    CONNECTING = "connecting"
    RUNNING = "running"
    COMPLETED = "completed"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass
class RobotTaskState:
    """Robot task execution state."""

    status: TaskStatus = TaskStatus.IDLE
    instruction: str = ""
    start_time: float = 0.0
    duration: float = 0.0
    step_count: int = 0
    error_message: str = ""
    task_future: Future | None = None


class Robot(AgentTool):
    """Universal robot control: composes a RuntimeAdapter, exposes a
    fixed agent-tool action surface, and runs the 50 Hz async control
    loop for VLA execution."""

    def __init__(
        self,
        tool_name: str,
        robot: Any,
        cameras: dict[str, dict[str, Any]] | None = None,
        action_horizon: int = 8,
        data_config: str | Any | None = None,
        control_frequency: float = 50.0,
        **kwargs: Any,
    ) -> None:
        """Initialize Robot with async capabilities.

        Args:
            tool_name: Name for this robot tool (also used as ``robot_id``
                for the runtime adapter).
            robot: A LeRobot ``Robot`` instance, ``RobotConfig``, robot
                type string, or registry name (e.g. ``"deepracer"``).
            cameras: LeRobot camera config dict.
            action_horizon: Actions consumed per VLA inference call.
            data_config: GR00T data config name (forwarded to the policy).
            control_frequency: VLA control loop frequency in Hz.
            **kwargs: Forwarded to the runtime constructor (e.g. ``port``,
                ``calibration_dir`` for LeRobot).
        """
        super().__init__()

        self.tool_name_str = tool_name
        self.action_horizon = action_horizon
        self.data_config = data_config
        self.control_frequency = control_frequency
        self.action_sleep_time = 1.0 / control_frequency

        # Task execution state.
        self._task_state = RobotTaskState()
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"{tool_name}_executor")
        self._shutdown_event = threading.Event()
        self._move_stop_event = threading.Event()

        # Resolve the registry entry so the factory knows which runtime
        # to build. For pre-built instances / configs (existing LeRobot
        # use), we skip the lookup and default to lerobot.
        if isinstance(robot, str):
            registry_entry = get_robot(robot) or {"runtime": "lerobot"}
        else:
            registry_entry = {"runtime": "lerobot"}

        self._runtime: RuntimeAdapter = build_runtime(
            robot_id=tool_name,
            registry_entry=registry_entry,
            robot=robot,
            cameras=cameras,
            **kwargs,
        )

        runtime_kind = registry_entry.get("runtime", "lerobot")
        logger.info("Robot %s initialized (runtime=%s)", tool_name, runtime_kind)

    # ------------------------------------------------------------------
    # Runtime-backed I/O helpers (replace the old ``self.robot`` calls).
    # ------------------------------------------------------------------

    @property
    def runtime(self) -> RuntimeAdapter:
        """The composed RuntimeAdapter — exposed read-only for advanced
        callers (e.g. integration tests). Most agent-tool flows go
        through ``stream`` / ``move`` / ``execute`` instead."""
        return self._runtime

    async def _connect_runtime(self) -> tuple[bool, str]:
        """Connect the runtime adapter. Idempotent."""
        try:
            await asyncio.to_thread(self._runtime.connect)
            return True, ""
        except Exception as exc:  # noqa: BLE001 — surface to caller
            logger.error("Runtime connect failed: %s", exc)
            return False, str(exc)

    # ------------------------------------------------------------------
    # Move action — primitive duration-aware velocity control.
    # ------------------------------------------------------------------

    def move(
        self,
        linear_x: float,
        angular_z: float,
        duration: float,
        rate_hz: float = 20.0,
        fast_mode: bool = False,
    ) -> dict[str, Any]:
        """Publish a Twist at ``rate_hz`` for ``duration`` seconds, then
        publish a final zero Twist on exit (normal, cancel, or exception).

        Args:
            linear_x: Body-frame forward velocity (m/s). Clamped to
                ``safety_limits.max_linear_x`` by the runtime adapter.
            angular_z: Body-frame yaw rate (rad/s). Clamped to
                ``safety_limits.max_angular_z``.
            duration: Wall-clock seconds to hold the command. Capped by
                ``safety_limits.max_duration_per_command``; out-of-range
                values are rejected (not clamped).
            rate_hz: Publish rate. Capped to ``runtime.command_rate_hz()``.
            fast_mode: If True, skip ``time.sleep`` between publishes
                (used by tests to keep things fast).

        Returns:
            Standard ``{status, content[]}`` dict.
        """
        limits = self._runtime.safety_limits()
        max_dur = float(limits.get("max_duration_per_command", float("inf")))
        if duration > max_dur:
            return {
                "status": "error",
                "content": [
                    {
                        "text": (
                            f"Move rejected: duration={duration}s exceeds "
                            f"max_duration_per_command={max_dur}s for {self.tool_name_str}"
                        )
                    }
                ],
            }

        # Cap publish rate to the runtime's native cadence.
        try:
            native_hz = float(self._runtime.command_rate_hz())
        except Exception:  # noqa: BLE001 — runtime may not implement
            native_hz = rate_hz
        rate_hz = min(rate_hz, native_hz) if native_hz > 0 else rate_hz

        if not self._runtime.is_connected():
            try:
                self._runtime.connect()
            except Exception as exc:  # noqa: BLE001
                return {
                    "status": "error",
                    "content": [{"text": f"Move failed to connect: {exc}"}],
                }

        self._move_stop_event.clear()
        action: dict[str, Any] = {"linear_x": float(linear_x), "angular_z": float(angular_z)}
        sleep_s = 1.0 / rate_hz if rate_hz > 0 else 0.0
        end_time = time.time() + duration
        executed = 0.0
        reason = "completed"

        try:
            start = time.time()
            while time.time() < end_time:
                if self._move_stop_event.is_set() or self._shutdown_event.is_set():
                    reason = "cancelled"
                    break
                self._runtime.send_action(action)
                if not fast_mode:
                    time.sleep(sleep_s)
            executed = time.time() - start
        except Exception as exc:  # noqa: BLE001 — best-effort even on raise
            reason = f"error: {exc}"
            executed = time.time() - start
        finally:
            # Always publish a zero Twist on the way out — the controller
            # watchdog assumes commands keep coming, and a stale non-zero
            # would keep wheels turning if we don't.
            try:
                self._runtime.send_action({"linear_x": 0.0, "angular_z": 0.0})
            except Exception as exc:  # noqa: BLE001
                logger.warning("Trailing zero publish failed: %s", exc)

        return {
            "status": "success" if reason == "completed" else ("success" if reason == "cancelled" else "error"),
            "content": [
                {
                    "text": (
                        f"Move on {self.tool_name_str}: linear_x={linear_x:.3f}, "
                        f"angular_z={angular_z:.3f}, duration={duration:.2f}s, "
                        f"rate_hz={rate_hz:.0f}, reason={reason}, "
                        f"executed_duration={executed:.2f}s"
                    )
                }
            ],
        }

    # ------------------------------------------------------------------
    # VLA execute path — the 50 Hz control loop. Backward-compat with
    # the existing GR00T / lerobot_local Policy flows.
    # ------------------------------------------------------------------

    async def _get_policy(
        self,
        policy_port: int | None = None,
        policy_host: str = "localhost",
        policy_provider: str = "groot",
    ) -> Any:
        from strands_robots.policies import create_policy

        if not policy_port and policy_provider == "groot":
            raise ValueError("policy_port is required for the groot policy provider")

        policy_config: dict[str, Any] = {"host": policy_host}
        if policy_port:
            policy_config["port"] = policy_port
        if self.data_config:
            policy_config["data_config"] = self.data_config
        return create_policy(policy_provider, **policy_config)

    async def _initialize_policy(self, policy: Any) -> bool:
        """Filter camera keys from a sample observation so the policy
        knows the joint-state ordering. Falls back gracefully when the
        runtime doesn't expose camera config (e.g. ROS2 mobile)."""
        try:
            test_obs = await asyncio.to_thread(self._runtime.get_observation)
            camera_keys: list[str] = []
            underlying = getattr(self._runtime, "_robot", None)
            if underlying is not None:
                cfg = getattr(underlying, "config", None)
                cams = getattr(cfg, "cameras", None) if cfg is not None else None
                if cams is not None:
                    camera_keys = list(cams.keys())
            robot_state_keys = [k for k in test_obs.keys() if k not in camera_keys]
            policy.set_robot_state_keys(robot_state_keys)
            return True
        except Exception as exc:  # noqa: BLE001
            logger.error("Policy initialization failed: %s", exc)
            return False

    async def _execute_task_async(
        self,
        instruction: str,
        policy_port: int | None = None,
        policy_host: str = "localhost",
        policy_provider: str = "groot",
        duration: float = 30.0,
    ) -> None:
        """Run the VLA control loop in the background executor."""
        try:
            self._task_state.status = TaskStatus.CONNECTING
            self._task_state.instruction = instruction
            self._task_state.start_time = time.time()
            self._task_state.step_count = 0
            self._task_state.error_message = ""

            connected, connect_error = await self._connect_runtime()
            if not connected:
                self._task_state.status = TaskStatus.ERROR
                self._task_state.error_message = connect_error or (
                    f"Failed to connect runtime for {self.tool_name_str}"
                )
                return

            policy = await self._get_policy(policy_port, policy_host, policy_provider)
            if not await self._initialize_policy(policy):
                self._task_state.status = TaskStatus.ERROR
                self._task_state.error_message = "Failed to initialize policy"
                return

            logger.info("Starting task '%s' on %s", instruction, self.tool_name_str)

            self._task_state.status = TaskStatus.RUNNING
            start_time = time.time()
            while (
                time.time() - start_time < duration
                and self._task_state.status == TaskStatus.RUNNING
                and not self._shutdown_event.is_set()
            ):
                observation = await asyncio.to_thread(self._runtime.get_observation)
                robot_actions = await policy.get_actions(observation, instruction)
                for action_dict in robot_actions[: self.action_horizon]:
                    if self._task_state.status != TaskStatus.RUNNING:
                        break
                    await asyncio.to_thread(self._runtime.send_action, action_dict)
                    self._task_state.step_count += 1
                    await asyncio.sleep(self.action_sleep_time)

            elapsed = time.time() - start_time
            self._task_state.duration = elapsed
            if self._task_state.status == TaskStatus.RUNNING:
                self._task_state.status = TaskStatus.COMPLETED
                logger.info(
                    "Task completed: '%s' in %.1fs (%d steps)",
                    instruction,
                    elapsed,
                    self._task_state.step_count,
                )
        except Exception as exc:  # noqa: BLE001
            logger.error("Task execution failed: %s", exc)
            self._task_state.status = TaskStatus.ERROR
            self._task_state.error_message = str(exc)

    def _execute_task_sync(
        self,
        instruction: str,
        policy_port: int | None = None,
        policy_host: str = "localhost",
        policy_provider: str = "groot",
        duration: float = 30.0,
    ) -> dict[str, Any]:
        """Run the async task in a fresh event loop on the background thread."""

        async def _runner() -> None:
            await self._execute_task_async(instruction, policy_port, policy_host, policy_provider, duration)

        try:
            asyncio.get_running_loop()
            import concurrent.futures

            with concurrent.futures.ThreadPoolExecutor() as exec_:
                future = exec_.submit(lambda: asyncio.run(_runner()))
                future.result()
        except RuntimeError:
            asyncio.run(_runner())

        return {
            "status": "success" if self._task_state.status == TaskStatus.COMPLETED else "error",
            "content": [
                {
                    "text": (
                        f"Task: '{instruction}' - {self._task_state.status.value}\n"
                        f"Robot: {self.tool_name_str}\n"
                        f"Policy: {policy_provider} on {policy_host}:{policy_port}\n"
                        f"Duration: {self._task_state.duration:.1f}s\n"
                        f"Steps: {self._task_state.step_count}"
                        + (f"\nError: {self._task_state.error_message}" if self._task_state.error_message else "")
                    )
                }
            ],
        }

    def start_task(
        self,
        instruction: str,
        policy_port: int | None = None,
        policy_host: str = "localhost",
        policy_provider: str = "groot",
        duration: float = 30.0,
    ) -> dict[str, Any]:
        if self._task_state.status == TaskStatus.RUNNING:
            return {
                "status": "error",
                "content": [{"text": f"Task already running: {self._task_state.instruction}"}],
            }
        self._task_state.task_future = self._executor.submit(
            self._execute_task_sync,
            instruction,
            policy_port,
            policy_host,
            policy_provider,
            duration,
        )
        return {
            "status": "success",
            "content": [
                {
                    "text": (
                        f"Task started: '{instruction}'\n"
                        f"Robot: {self.tool_name_str}\n"
                        "Use action='status' to check progress\n"
                        "Use action='stop' to interrupt"
                    )
                }
            ],
        }

    def get_task_status(self) -> dict[str, Any]:
        if self._task_state.status == TaskStatus.RUNNING:
            self._task_state.duration = time.time() - self._task_state.start_time

        text = f"Robot Status: {self._task_state.status.value.upper()}\n"
        if self._task_state.instruction:
            text += f"Task: {self._task_state.instruction}\n"
        if self._task_state.status == TaskStatus.RUNNING:
            text += f"Duration: {self._task_state.duration:.1f}s\n"
            text += f"Steps: {self._task_state.step_count}\n"
        elif self._task_state.status in (
            TaskStatus.COMPLETED,
            TaskStatus.STOPPED,
            TaskStatus.ERROR,
        ):
            text += f"Total Duration: {self._task_state.duration:.1f}s\n"
            text += f"Total Steps: {self._task_state.step_count}\n"
        if self._task_state.error_message:
            text += f"Error: {self._task_state.error_message}\n"

        return {"status": "success", "content": [{"text": text}]}

    def stop_task(self) -> dict[str, Any]:
        # Always signal the move loop too — the user expects ``stop`` to halt
        # any motion, regardless of which path is in flight.
        self._move_stop_event.set()

        if self._task_state.status != TaskStatus.RUNNING:
            return {
                "status": "success",
                "content": [{"text": f"No task running to stop (current: {self._task_state.status.value})"}],
            }
        self._task_state.status = TaskStatus.STOPPED
        if self._task_state.task_future is not None:
            self._task_state.task_future.cancel()
        logger.info("Task stopped: %s", self._task_state.instruction)

        return {
            "status": "success",
            "content": [
                {
                    "text": (
                        f"Task stopped: '{self._task_state.instruction}'\n"
                        f"Duration: {self._task_state.duration:.1f}s\n"
                        f"Steps completed: {self._task_state.step_count}"
                    )
                }
            ],
        }

    # ------------------------------------------------------------------
    # AgentTool surface.
    # ------------------------------------------------------------------

    @property
    def tool_name(self) -> str:
        return self.tool_name_str

    @property
    def tool_type(self) -> str:
        return "robot"

    @property
    def tool_spec(self) -> ToolSpec:
        """Stable action surface across every robot.

        Capability differences are surfaced in the description text and
        in structured-error responses (``available_actions``) — never in
        a shape-shifting schema. Agents plan against one contract."""
        caps = sorted(self._runtime.capabilities()) if self._runtime is not None else []
        cap_text = ", ".join(caps) if caps else "(none)"
        return {
            "name": self.tool_name_str,
            "description": (
                f"Universal robot control for {self.tool_name_str}. "
                f"Capabilities: {cap_text}. "
                "Actions: execute (blocking VLA), start (async VLA), status, stop, "
                "move (primitive Twist publish for duration). "
                "execute / start require instruction; "
                "move requires linear_x, angular_z, duration."
            ),
            "inputSchema": {
                "json": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "description": ("Action to perform: execute, start, status, stop, move."),
                            "enum": ["execute", "start", "status", "stop", "move"],
                            "default": "execute",
                        },
                        "instruction": {
                            "type": "string",
                            "description": "Natural-language instruction (for execute/start).",
                        },
                        "policy_port": {
                            "type": "integer",
                            "description": "Policy service port (for execute/start with groot).",
                        },
                        "policy_host": {
                            "type": "string",
                            "description": "Policy service host (default: localhost).",
                            "default": "localhost",
                        },
                        "policy_provider": {
                            "type": "string",
                            "description": "Policy provider (groot, lerobot_local, mock, etc.).",
                            "default": "groot",
                        },
                        "duration": {
                            "type": "number",
                            "description": (
                                "For execute/start: max wall-clock seconds. For move: how long to hold the command."
                            ),
                            "default": 30.0,
                        },
                        "linear_x": {
                            "type": "number",
                            "description": "Body-frame forward velocity m/s (move action).",
                        },
                        "angular_z": {
                            "type": "number",
                            "description": "Body-frame yaw rate rad/s (move action).",
                        },
                        "rate_hz": {
                            "type": "number",
                            "description": "Publish rate for move loop (default 20).",
                            "default": 20.0,
                        },
                    },
                    "required": ["action"],
                }
            },
        }

    @staticmethod
    def _make_tool_result(tool_use_id: str, result: dict[str, Any]) -> ToolResult:
        return cast(ToolResult, {"toolUseId": tool_use_id, **result})

    async def stream(
        self, tool_use: ToolUse, invocation_state: dict[str, Any], **kwargs: Any
    ) -> AsyncGenerator[ToolResultEvent, None]:
        try:
            tool_use_id = tool_use.get("toolUseId", "")
            input_data = tool_use.get("input", {})
            action = input_data.get("action", "execute")

            if action == "execute":
                instruction = input_data.get("instruction", "")
                policy_port = input_data.get("policy_port")
                policy_host = input_data.get("policy_host", "localhost")
                policy_provider = input_data.get("policy_provider", "groot")
                duration = input_data.get("duration", 30.0)
                if not instruction:
                    yield ToolResultEvent(
                        self._make_tool_result(
                            tool_use_id,
                            {
                                "status": "error",
                                "content": [{"text": "Instruction is required for execute action."}],
                            },
                        )
                    )
                    return
                result = self._execute_task_sync(instruction, policy_port, policy_host, policy_provider, duration)
                yield ToolResultEvent(self._make_tool_result(tool_use_id, result))

            elif action == "start":
                instruction = input_data.get("instruction", "")
                policy_port = input_data.get("policy_port")
                policy_host = input_data.get("policy_host", "localhost")
                policy_provider = input_data.get("policy_provider", "groot")
                duration = input_data.get("duration", 30.0)
                if not instruction:
                    yield ToolResultEvent(
                        self._make_tool_result(
                            tool_use_id,
                            {
                                "status": "error",
                                "content": [{"text": "Instruction is required for start action."}],
                            },
                        )
                    )
                    return
                start_result = self.start_task(instruction, policy_port, policy_host, policy_provider, duration)
                yield ToolResultEvent(self._make_tool_result(tool_use_id, start_result))

            elif action == "status":
                yield ToolResultEvent(self._make_tool_result(tool_use_id, self.get_task_status()))

            elif action == "stop":
                yield ToolResultEvent(self._make_tool_result(tool_use_id, self.stop_task()))

            elif action == "move":
                if "linear_x" not in input_data or "angular_z" not in input_data:
                    yield ToolResultEvent(
                        self._make_tool_result(
                            tool_use_id,
                            {
                                "status": "error",
                                "content": [{"text": "linear_x and angular_z are required for move."}],
                            },
                        )
                    )
                    return
                duration = float(input_data.get("duration", 1.0))
                rate_hz = float(input_data.get("rate_hz", 20.0))
                move_result = self.move(
                    linear_x=float(input_data["linear_x"]),
                    angular_z=float(input_data["angular_z"]),
                    duration=duration,
                    rate_hz=rate_hz,
                )
                yield ToolResultEvent(self._make_tool_result(tool_use_id, move_result))

            else:
                yield ToolResultEvent(
                    self._make_tool_result(
                        tool_use_id,
                        {
                            "status": "error",
                            "content": [
                                {
                                    "text": (
                                        f"Unknown action: {action}. Valid actions: execute, start, status, stop, move."
                                    )
                                }
                            ],
                        },
                    )
                )
        except Exception as exc:  # noqa: BLE001
            logger.error("%s error: %s", self.tool_name_str, exc)
            yield ToolResultEvent(
                self._make_tool_result(
                    tool_use_id,
                    {
                        "status": "error",
                        "content": [{"text": f"{self.tool_name_str} error: {exc}"}],
                    },
                )
            )

    # ------------------------------------------------------------------
    # Lifecycle / cleanup.
    # ------------------------------------------------------------------

    def cleanup(self) -> None:
        try:
            self._shutdown_event.set()
            self._move_stop_event.set()
            if self._task_state.status == TaskStatus.RUNNING:
                self.stop_task()
            self._executor.shutdown(wait=True)
        except Exception as exc:  # noqa: BLE001
            logger.error("Cleanup error for %s: %s", self.tool_name_str, exc)

    def __del__(self) -> None:
        try:
            self.cleanup()
        except Exception:  # noqa: BLE001
            pass

    async def get_status(self) -> dict[str, Any]:
        try:
            is_connected = self._runtime.is_connected()
            caps = sorted(self._runtime.capabilities())
            data: dict[str, Any] = {
                "robot_name": self.tool_name_str,
                "data_config": self.data_config,
                "is_connected": is_connected,
                "capabilities": caps,
                "task_status": self._task_state.status.value,
                "current_instruction": self._task_state.instruction,
                "task_duration": self._task_state.duration,
                "task_steps": self._task_state.step_count,
            }
            if self._task_state.error_message:
                data["task_error"] = self._task_state.error_message
            return data
        except Exception as exc:  # noqa: BLE001
            logger.error("Error getting status for %s: %s", self.tool_name_str, exc)
            return {
                "robot_name": self.tool_name_str,
                "error": str(exc),
                "is_connected": False,
                "task_status": "error",
            }

    async def stop(self) -> None:
        """Async wrapper used by AgentTool lifecycle — stops any active
        task and disconnects the runtime."""
        try:
            self._move_stop_event.set()
            if self._task_state.status == TaskStatus.RUNNING:
                self.stop_task()
            await asyncio.to_thread(self._runtime.disconnect)
            self.cleanup()
        except Exception as exc:  # noqa: BLE001
            logger.error("Error stopping robot: %s", exc)
