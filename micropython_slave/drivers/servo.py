"""
drivers/servo.py

Simple, reliable Servo driver for ESP32 MicroPython using LEDC/PWM.

Designed for SG90, MG996R, and similar hobby servos (50 Hz, 0.5–2.5 ms pulse).

Features:
- Angle in degrees (0–180) or microseconds (500–2500)
- Smooth move with speed limit (deg/s)
- Detach (stop PWM) for power saving when not moving
- Works on any output-capable GPIO (not 34/35/36/39)

Usage:
    from drivers.servo import Servo
    s = Servo(pin=25, min_us=500, max_us=2500, freq=50)
    s.move_to(90, speed=60)     # degrees, deg/s
    s.detach()
"""

from machine import Pin, PWM
import time


class Servo:
    """
    One servo channel.

    Default calibration matches most SG90-style servos:
      500 µs  = 0°
      1500 µs = 90°
      2500 µs = 180°
    """

    def __init__(self, pin: int, min_us: int = 500, max_us: int = 2500,
                 freq: int = 50, name: str = ""):
        self.pin = Pin(pin, Pin.OUT)
        self.pwm = PWM(self.pin, freq=freq)
        self.min_us = min_us
        self.max_us = max_us
        self.freq = freq
        self.name = name or f"servo_{pin}"

        # Current state
        self._current_angle = 90.0
        self._last_us = 1500
        self._attached = True

        # Set initial position to center
        self._set_us(1500)

    def _us_to_duty(self, us: int) -> int:
        """Convert pulse width in µs to PWM duty (0-65535 for 16-bit)."""
        period_us = 1_000_000 // self.freq
        duty = int((us / period_us) * 65535)
        return max(0, min(65535, duty))

    def _set_us(self, us: int):
        """Low-level: set exact pulse width."""
        if not self._attached:
            self.attach()
        duty = self._us_to_duty(us)
        self.pwm.duty_u16(duty)
        self._last_us = us

    def attach(self):
        """Re-enable PWM output."""
        if not self._attached:
            self.pwm = PWM(self.pin, freq=self.freq)
            self._attached = True

    def detach(self):
        """Stop PWM (servo will hold last position but draw less power)."""
        try:
            self.pwm.deinit()
        except:
            pass
        self._attached = False

    # ------------------------------------------------------------------
    # High level API
    # ------------------------------------------------------------------

    def move_to(self, angle: float, speed: float = 0):
        """
        Move to target angle in degrees (0-180).

        If speed > 0, performs a blocking move at limited speed (deg/s).
        """
        target = max(0.0, min(180.0, float(angle)))

        if speed <= 0:
            # Instant
            us = self.min_us + int((target / 180.0) * (self.max_us - self.min_us))
            self._set_us(us)
            self._current_angle = target
            return

        # Smooth move
        delta = target - self._current_angle
        if abs(delta) < 0.5:
            return

        duration = abs(delta) / speed          # seconds
        steps = max(5, int(duration * 50))     # ~20 ms steps
        step_size = delta / steps

        for i in range(steps):
            self._current_angle += step_size
            us = self.min_us + int((self._current_angle / 180.0) * (self.max_us - self.min_us))
            self._set_us(us)
            time.sleep_ms(20)

        # Final exact position
        self._current_angle = target
        us = self.min_us + int((target / 180.0) * (self.max_us - self.min_us))
        self._set_us(us)

    def set_angle(self, angle: float):
        """Alias for instant move."""
        self.move_to(angle, speed=0)

    @property
    def angle(self) -> float:
        return self._current_angle

    def set_us(self, us: int):
        """Direct microsecond control (for fine calibration)."""
        us = max(self.min_us, min(self.max_us, us))
        self._set_us(us)
        # Update angle approximation
        self._current_angle = ((us - self.min_us) / (self.max_us - self.min_us)) * 180.0

    def deinit(self):
        self.detach()


# Convenience factory for multiple servos
def create_servos(pin_list: list[int], **kwargs) -> list[Servo]:
    """Create a list of Servo objects from pin numbers."""
    return [Servo(pin=p, name=f"servo_{i}", **kwargs) for i, p in enumerate(pin_list)]