"""Process-wide rclpy lifecycle singleton.

Owns one ``rclpy.context.Context`` and one ``MultiThreadedExecutor``
shared across every ROS2 component (``Ros2Runtime`` plus future
``nav2_tool`` / ``slam_tool``). Every component attaches its node here
via :meth:`Ros2Lifecycle.add_node`; nothing creates a second context.

Why explicit context: rclpy's global default context has a known
wait-set collision when multiple threads call ``rclpy.spin*()`` against
it (``ros2/rclpy#1009``). By owning our own ``Context``, all our nodes
share one wait set and one executor, and we never collide with an
application that calls ``rclpy.init()`` itself.

Shutdown order (enforced by :meth:`shutdown`):

    1. ``executor.shutdown(timeout)`` — stops scheduling, waits for
       in-flight callbacks.
    2. For each node: ``executor.remove_node(node)`` then
       ``node.destroy_node()`` to free DDS readers/writers.
    3. ``context.try_shutdown()`` — tear down the rclpy context.
    4. ``spin_thread.join(timeout)`` — drain the spin thread.

The spin thread is ``daemon=False`` so an early process exit can't kill
it mid-callback; production code can register ``shutdown`` with
``atexit`` to drain it cleanly. Tests call :meth:`_reset_for_test`
between cases.
"""

from __future__ import annotations

import logging
import threading
from typing import Any

logger = logging.getLogger(__name__)

_DEFAULT_SHUTDOWN_TIMEOUT_SEC = 2.0


class Ros2Lifecycle:
    """Singleton holding the rclpy Context, executor, and spin thread."""

    _instance: Ros2Lifecycle | None = None
    _lock = threading.Lock()

    def __init__(self) -> None:
        # Lazy imports — paying for rclpy only when the lifecycle is
        # actually built. The LeRobot-only path never reaches this.
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor

        self._context = Context()
        # init() with no signal handler args — we are library code; the
        # application owns SIGINT/SIGTERM. The rclpy mock accepts the
        # same signature.
        self._context.init()

        self._executor = MultiThreadedExecutor(num_threads=4, context=self._context)
        self._shutdown_called = False
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="strands-rclpy-spin",
            daemon=False,
        )
        self._spin_thread.start()

    @classmethod
    def get_instance(cls) -> Ros2Lifecycle:
        """Return the process-wide singleton, constructing it if needed.

        If a previous instance was shut down (e.g. by an explicit
        :meth:`shutdown` or :meth:`_reset_for_test`), a fresh instance
        is constructed so the caller always receives an active lifecycle.
        """
        with cls._lock:
            if cls._instance is None or cls._instance._shutdown_called:
                cls._instance = cls()
            return cls._instance

    @property
    def context(self) -> Any:
        return self._context

    @property
    def executor(self) -> Any:
        return self._executor

    def is_active(self) -> bool:
        """True if the lifecycle's context is still ok and we haven't
        shut down yet."""
        return (not self._shutdown_called) and bool(self._context.ok())

    def add_node(self, node: Any) -> None:
        """Register a node with the shared executor."""
        self._executor.add_node(node)

    def remove_node(self, node: Any) -> None:
        """Deregister a node from the shared executor (does NOT destroy it)."""
        self._executor.remove_node(node)

    def shutdown(self, timeout_sec: float = _DEFAULT_SHUTDOWN_TIMEOUT_SEC) -> None:
        """Tear down in the documented order. Idempotent."""
        if self._shutdown_called:
            return
        self._shutdown_called = True

        # 1. Stop scheduling, wait for in-flight callbacks.
        try:
            self._executor.shutdown(timeout_sec=timeout_sec)
        except Exception as exc:  # noqa: BLE001 — defensive
            logger.warning("Executor shutdown raised: %s", exc)

        # 2. Remove and destroy every node currently attached.
        for node in list(self._executor.get_nodes()):
            try:
                self._executor.remove_node(node)
                node.destroy_node()
            except Exception as exc:  # noqa: BLE001 — defensive
                logger.warning("Node teardown raised: %s", exc)

        # 3. Tear down the rclpy context.
        try:
            self._context.try_shutdown()
        except Exception as exc:  # noqa: BLE001 — defensive
            logger.warning("Context try_shutdown raised: %s", exc)

        # 4. Drain the spin thread.
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=timeout_sec)

    @classmethod
    def _reset_for_test(cls) -> None:
        """Test-only: shut down the active singleton and clear the slot."""
        with cls._lock:
            if cls._instance is not None:
                try:
                    cls._instance.shutdown()
                except Exception:  # noqa: BLE001 — best-effort cleanup
                    pass
                cls._instance = None
