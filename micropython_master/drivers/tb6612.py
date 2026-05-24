"""
tb6612.py
Minimal, debuggable driver for the CODBOT 4-channel TB6612FNG board
(only channels 1 and 4 used for the front encoder motors).

Follows the exact direction + PWM logic from the official CODBOT Arduino
examples (Two_Encoder_Motor_PID_Control.ino and similar) that the user
referenced as the manufacturing source of truth.

- One TB6612Channel per motor (left=CH1, right=CH4)
- Speed API: -100.0 ... +100.0 (percent of full PWM)
- Explicit brake() / coast() for safety
- No STBY handling here (see DESIGN.md + config: if your board requires
  STBY1/STBY2 driven high, jumper them to 3V3 or add the two spare pins
  (16/21) to config and drive them in a board-level init).

Current baseline (from user's verified pin map):
  Left  (CH1): IN1=26, IN2=27, PWM=25
  Right (CH4): IN1=32, IN2=4,  PWM=33

PWM frequency chosen to match common 520-size encoder motor practice (~1 kHz).
"""

from machine import Pin, PWM
import time


class TB6612Channel:
    """
    Single motor channel on the TB6612.

    Usage (REPL example):
        from drivers.tb6612 import TB6612Channel
        import config
        m = TB6612Channel(config.MOTOR_LEFT_IN1, config.MOTOR_LEFT_IN2,
                          config.MOTOR_LEFT_PWM, name="left")
        m.set(60)      # forward 60%
        time.sleep(1)
        m.brake()
        m.deinit()
    """

    def __init__(self, in1_pin, in2_pin, pwm_pin, freq=1000, name="tb6612", invert: bool = False):
        self.name = name
        self.invert = invert

        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)

        # 10-bit duty (0-1023) is the MicroPython ESP32 default and matches
        # most Arduino TB6612 sketches for these motors.
        self.pwm = PWM(Pin(pwm_pin), freq=freq, duty=0)

        self._last_speed = 0.0
        self._freq = freq

        # Start safe
        self.coast()

    def _apply(self, fwd: bool, rev: bool, duty: int):
        """Low-level: set direction bits + PWM duty (0-1023)."""
        self.in1.value(1 if fwd else 0)
        self.in2.value(1 if rev else 0)
        self.pwm.duty(duty)

    def set(self, speed: float):
        """
        Set motor speed.

        :param speed: -100.0 (full reverse) ... +100.0 (full forward)
                      0.0 = coast (freewheel)
        The sign is flipped internally if self.invert is True (see config.MOTOR_*_INVERT).
        """
        speed = max(-100.0, min(100.0, float(speed)))
        if self.invert:
            speed = -speed
        self._last_speed = speed   # store the *commanded* (pre-invert) value for logging

        abs_speed = abs(speed)
        duty = int(abs_speed * 10.23)   # 0-100 → 0-1023

        if speed > 0:
            # Forward: IN1=H, IN2=L
            self._apply(fwd=True, rev=False, duty=duty)
        elif speed < 0:
            # Reverse: IN1=L, IN2=H
            self._apply(fwd=False, rev=True, duty=duty)
        else:
            self.coast()

    def brake(self):
        """Short brake: IN1=H, IN2=H (motor terminals shorted)."""
        self._last_speed = 0.0
        self._apply(fwd=True, rev=True, duty=0)   # duty ignored by hardware in brake

    def coast(self):
        """Coast / freewheel stop: IN1=L, IN2=L."""
        self._last_speed = 0.0
        self._apply(fwd=False, rev=False, duty=0)

    def get_last_speed(self) -> float:
        return self._last_speed

    def deinit(self):
        """Release PWM and pins for clean REPL reload / power-down."""
        try:
            self.coast()
            self.pwm.deinit()
        except Exception:
            pass

    def __repr__(self):
        return f"<TB6612Channel {self.name} speed={self._last_speed:.1f}>"


# Convenience factory helpers (optional, keeps call sites short)
def create_left():
    import config
    return TB6612Channel(
        config.MOTOR_LEFT_IN1,
        config.MOTOR_LEFT_IN2,
        config.MOTOR_LEFT_PWM,
        name="left",
        invert=config.MOTOR_LEFT_INVERT
    )


def create_right():
    import config
    return TB6612Channel(
        config.MOTOR_RIGHT_IN1,
        config.MOTOR_RIGHT_IN2,
        config.MOTOR_RIGHT_PWM,
        name="right",
        invert=config.MOTOR_RIGHT_INVERT
    )