"""
utils/timing.py
Simple fixed-rate loop helper for MicroPython on ESP32.

Uses time.ticks_ms() for monotonic timing (handles wrap-around).
Reports actual loop period and jitter — essential for later odometry,
PID, and path following.

Usage (example 20 ms control loop):

    from utils.timing import RateTimer
    import time
    from robot import drive

    timer = RateTimer(period_ms=20)
    while True:
        dt = timer.wait()          # blocks until next tick, returns actual dt (ms)
        # read sensors / encoders
        # compute commands
        drive.set_side_speeds(...)
        if timer.jitter > 5:
            print("High jitter:", timer.jitter)
"""

import time


class RateTimer:
    """
    Fixed period timer.

    period_ms: target loop time in milliseconds (e.g. 20 for 50 Hz)
    """

    def __init__(self, period_ms: int = 20):
        self.period_ms = period_ms
        self._next_tick = time.ticks_add(time.ticks_ms(), period_ms)
        self.last_dt = 0
        self.jitter = 0
        self.iteration = 0

    def wait(self) -> int:
        """
        Block until the next scheduled tick.
        Returns the *actual* elapsed time since previous call (ms).
        Updates .jitter (absolute deviation from target).
        """
        now = time.ticks_ms()
        # Sleep until (or past) the target
        sleep_ms = time.ticks_diff(self._next_tick, now)
        if sleep_ms > 0:
            time.sleep_ms(sleep_ms)

        actual_now = time.ticks_ms()
        self.last_dt = time.ticks_diff(actual_now, self._next_tick) + self.period_ms
        self.jitter = abs(self.last_dt - self.period_ms)

        # Schedule next
        self._next_tick = time.ticks_add(self._next_tick, self.period_ms)
        self.iteration += 1

        return self.last_dt

    def reset(self):
        """Restart timing from now."""
        self._next_tick = time.ticks_add(time.ticks_ms(), self.period_ms)
        self.last_dt = 0
        self.jitter = 0
        self.iteration = 0


def busy_wait_ms(ms: int):
    """Crude busy-wait (only for very short debug delays)."""
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < ms:
        pass