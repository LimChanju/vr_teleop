"""Fail-closed arm command watchdog used by the ALVR upstream overlay."""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Any

import numpy as np


def _timeout_from_environment() -> float:
    raw = os.environ.get("ALVR_ARM_COMMAND_TIMEOUT", "0.25")
    try:
        value = float(raw)
    except ValueError as error:
        raise ValueError("ALVR_ARM_COMMAND_TIMEOUT must be a number") from error
    if not 0.05 <= value <= 2.0:
        raise ValueError("ALVR_ARM_COMMAND_TIMEOUT must be between 0.05 and 2.0 seconds")
    return value


class ArmCommandSafety:
    """Latch measured-position hold if the teleoperation loop stops updating."""

    def __init__(
        self,
        controller: Any,
        motion_ready: Any = None,
        *,
        timeout: float | None = None,
        start_thread: bool = True,
    ) -> None:
        self.controller = controller
        self.motion_ready = motion_ready
        self.timeout = _timeout_from_environment() if timeout is None else float(timeout)
        if not 0.05 <= self.timeout <= 2.0:
            raise ValueError("arm command timeout must be between 0.05 and 2.0 seconds")
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._armed = False
        self._timeout_latched = False
        self._closed = False
        self._hold_pending = True
        self._last_command_at = time.monotonic()
        self._thread: threading.Thread | None = None

        # Upstream controllers start with an all-zero target.  Hold the actual
        # measured pose before the operator starts tracking instead.
        initial_q = np.asarray(self.controller.get_current_dual_arm_q(), dtype=float).copy()
        if initial_q.ndim != 1 or initial_q.size == 0 or not np.all(np.isfinite(initial_q)):
            raise ValueError("initial measured arm joints must be a non-empty finite 1-D array")
        self._expected_shape = initial_q.shape
        self._last_measured_q = initial_q.copy()
        with self._lock:
            self._set_motion_ready_locked(False)
            self._hold_locked(initial_q)
        if start_thread:
            self._thread = threading.Thread(
                target=self._watchdog_loop,
                name="alvr-arm-command-watchdog",
                daemon=True,
            )
            self._thread.start()

    @property
    def timeout_latched(self) -> bool:
        with self._lock:
            return self._timeout_latched

    def can_command(self) -> bool:
        with self._lock:
            return not self._closed and not self._timeout_latched

    def observe_measured(self, measured_q: Any) -> None:
        q = np.asarray(measured_q, dtype=float).copy()
        if q.shape != self._expected_shape or not np.all(np.isfinite(q)):
            raise ValueError(f"measured arm joints must be shaped {self._expected_shape} and finite")
        with self._lock:
            self._last_measured_q = q

    def command(self, q_target: Any, tauff_target: Any) -> bool:
        q = np.asarray(q_target, dtype=float).copy()
        tau = np.asarray(tauff_target, dtype=float).copy()
        if (
            q.shape != self._expected_shape
            or tau.shape != self._expected_shape
            or not np.all(np.isfinite(q))
            or not np.all(np.isfinite(tau))
        ):
            with self._lock:
                self._armed = False
                self._timeout_latched = True
                self._hold_pending = True
                self._set_motion_ready_locked(False)
                self._hold_locked()
            raise ValueError(
                f"arm command must contain finite q/tau arrays shaped {self._expected_shape}"
            )
        with self._lock:
            if self._closed or self._timeout_latched:
                self._set_motion_ready_locked(False)
                self._hold_pending = True
                self._hold_locked()
                return False
            try:
                self.controller.ctrl_dual_arm(q, tau)
            except Exception:
                self._armed = False
                self._timeout_latched = True
                self._hold_pending = True
                self._set_motion_ready_locked(False)
                try:
                    self._hold_locked()
                except Exception:
                    pass
                raise
            self._last_command_at = time.monotonic()
            self._armed = True
            self._set_motion_ready_locked(True)
            return True

    def hold(self, measured_q: Any = None, *, released: bool) -> None:
        q = None if measured_q is None else np.asarray(measured_q, dtype=float).copy()
        with self._lock:
            self._armed = False
            self._hold_pending = True
            if released:
                self._timeout_latched = False
            self._set_motion_ready_locked(False)
            self._hold_locked(q)

    def _set_motion_ready_locked(self, value: bool) -> None:
        if self.motion_ready is None:
            return
        lock = self.motion_ready.get_lock()
        with lock:
            self.motion_ready.value = bool(value)

    def _hold_locked(self, measured_q: np.ndarray | None = None) -> None:
        if measured_q is None:
            try:
                candidate = np.asarray(
                    self.controller.get_current_dual_arm_q(), dtype=float
                ).copy()
            except Exception:
                candidate = self._last_measured_q.copy()
        else:
            candidate = np.asarray(measured_q, dtype=float).copy()
        if candidate.shape == self._expected_shape and np.all(np.isfinite(candidate)):
            self._last_measured_q = candidate
        q = self._last_measured_q.copy()
        try:
            self.controller.ctrl_dual_arm(q, np.zeros_like(q))
        except Exception:
            self._hold_pending = True
            raise
        self._hold_pending = False

    def _watchdog_step(self, now: float | None = None) -> bool:
        current = time.monotonic() if now is None else now
        timed_out = False
        with self._lock:
            if self._closed:
                return False
            if self._armed and current - self._last_command_at > self.timeout:
                self._armed = False
                self._timeout_latched = True
                self._hold_pending = True
                timed_out = True
            if not self._hold_pending:
                return False
            self._set_motion_ready_locked(False)
            self._hold_locked()
        if timed_out:
            print(
                "[ALVR] Arm command heartbeat timed out; release both grips before re-arming.",
                file=sys.stderr,
            )
        return timed_out

    def _watchdog_loop(self) -> None:
        interval = min(0.02, self.timeout / 4.0)
        while not self._stop.wait(interval):
            try:
                self._watchdog_step()
            except Exception as error:
                print(f"[ALVR] Arm safety watchdog failed to apply hold: {error}", file=sys.stderr)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._armed = False
            self._timeout_latched = True
            self._hold_pending = True
            self._set_motion_ready_locked(False)
            self._hold_locked()


__all__ = ["ArmCommandSafety"]
