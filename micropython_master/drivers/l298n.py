"""
l298n.py
Ultra-minimal driver for the rear DC motors on the L298N dual H-bridge.

Per the current verified design (DESIGN.md + config.py):
- ENA and ENB are tied HIGH in hardware (always enabled, full voltage).
- The ESP32 only controls direction via IN1/IN2.
- No PWM speed control on rear (front TB6612 sets the pace; rear just
  pushes in the same direction = "same command per side").

This matches the GPIO-saving simplification chosen after WROOM pin reality check.

Pins (from user's locked map):
  Rear Left : IN1=13, IN2=14
  Rear Right: IN1=22, IN2=23
"""

from machine import Pin


class L298NChannel:
    """
    Single rear motor channel (direction only).

    Usage:
        from drivers.l298n import L298NChannel
        import config
        r = L298NChannel(config.MOTOR_REAR_LEFT_IN1,
                         config.MOTOR_REAR_LEFT_IN2,
                         name="rear_left")
        r.set_direction(True)   # forward (same as front left)
        r.set_direction(False)  # reverse
        r.stop()
        r.deinit()
    """

    def __init__(self, in1_pin, in2_pin, name="l298n", invert: bool = False):
        self.name = name
        self.invert = invert
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self._last_dir = 0   # 1=fwd, -1=rev, 0=stop  (reported to caller, pre-invert)
        self.stop()

    def set_direction(self, fwd):
        """
        :param fwd: True / >0 / positive  -> forward for this side
                    False / <0 / negative -> reverse for this side
                    0 / None                -> stop (coast)
        The meaning of 'forward' is flipped if self.invert is True
        (see config MOTOR_*_INVERT after Phase 1 hardware test).
        """
        if fwd is None or fwd == 0:
            self.stop()
            return

        # Normalize to boolean "wants positive direction"
        wants_positive = bool(fwd) and (fwd > 0 if isinstance(fwd, (int, float)) else True)

        if self.invert:
            wants_positive = not wants_positive

        if wants_positive:
            self.in1.value(1)
            self.in2.value(0)
            self._last_dir = 1
        else:
            self.in1.value(0)
            self.in2.value(1)
            self._last_dir = -1

    def stop(self):
        """Coast stop (both low). Since ENA tied high this is the only stop mode."""
        self.in1.value(0)
        self.in2.value(0)
        self._last_dir = 0

    def get_last_dir(self):
        """Returns 1 (fwd), -1 (rev), or 0 (stop)."""
        return self._last_dir

    def deinit(self):
        self.stop()

    def __repr__(self):
        d = {1: "FWD", -1: "REV", 0: "STOP"}.get(self._last_dir, "?")
        return f"<L298NChannel {self.name} {d}>"


# Convenience factories
def create_rear_left():
    import config
    return L298NChannel(config.MOTOR_REAR_LEFT_IN1,
                        config.MOTOR_REAR_LEFT_IN2,
                        name="rear_left",
                        invert=config.MOTOR_LEFT_INVERT)


def create_rear_right():
    import config
    return L298NChannel(config.MOTOR_REAR_RIGHT_IN1,
                        config.MOTOR_REAR_RIGHT_IN2,
                        name="rear_right",
                        invert=config.MOTOR_RIGHT_INVERT)