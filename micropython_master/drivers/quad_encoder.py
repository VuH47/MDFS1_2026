"""
quad_encoder.py

A clean, interrupt-driven quadrature encoder decoder for MicroPython (ESP32).

Designed to be easy to debug and tune. We accept that it will not be as fast
as hardware PCNT, but it is much easier to understand and modify.

Features:
- 4x decoding using the standard 16-entry transition lookup table
- Direction inversion support
- Atomic delta read (IRQ-safe) for speed calculation
- Total position
- Warns when used on ESP32 input-only pins (34/35/36/39) which have no
  internal pull resistors and therefore require external pull-ups.
"""

from machine import Pin, disable_irq, enable_irq
import micropython

# Allocate emergency exception buffer so we can see IRQ errors
micropython.alloc_emergency_exception_buf(100)

# ESP32 input-only pins (no internal pull-up / pull-down available)
_INPUT_ONLY_PINS = (34, 35, 36, 39)

# Standard 4x quadrature transition table.
# Index = (last_a << 3) | (last_b << 2) | (a << 1) | b
# Value = delta count (-1, 0, +1). 0 = no change or invalid (both edges).
_QUAD_TABLE = (
    0,  -1,  1,  0,
    1,   0,  0, -1,
   -1,   0,  0,  1,
    0,   1, -1,  0,
)


class QuadEncoder:
    def __init__(self, pin_a, pin_b, invert=False, name="encoder"):
        """
        :param pin_a: GPIO number for encoder channel A
        :param pin_b: GPIO number for encoder channel B
        :param invert: If True, reverses the direction sign
        :param name: Name used for debug prints

        NOTE: If pin_a or pin_b is one of 34/35/36/39, you MUST add an
        external pull-up resistor (4.7k-10k to 3V3) in hardware. Those pins
        have no internal pull resistors on the ESP32.
        """
        self.name = name
        self._invert = invert

        for p in (pin_a, pin_b):
            if p in _INPUT_ONLY_PINS:
                print("[quad_encoder:{}] WARNING: GPIO {} is input-only on "
                      "ESP32. External pull-up required.".format(name, p))

        # Use PULL_UP only on pins that actually support it.
        pull_a = None if pin_a in _INPUT_ONLY_PINS else Pin.PULL_UP
        pull_b = None if pin_b in _INPUT_ONLY_PINS else Pin.PULL_UP

        self._pin_a = Pin(pin_a, Pin.IN, pull_a)
        self._pin_b = Pin(pin_b, Pin.IN, pull_b)

        self._position = 0
        self._last_a = self._pin_a.value()
        self._last_b = self._pin_b.value()

        self._pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                        handler=self._irq_handler)
        self._pin_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                        handler=self._irq_handler)

    def _irq_handler(self, pin):
        """
        Lightweight IRQ handler using a 16-entry transition table.
        Runs in interrupt context - keep it short, no allocations.
        """
        a = self._pin_a.value()
        b = self._pin_b.value()

        idx = (self._last_a << 3) | (self._last_b << 2) | (a << 1) | b
        delta = _QUAD_TABLE[idx]

        if delta:
            if self._invert:
                delta = -delta
            self._position += delta

        self._last_a = a
        self._last_b = b

    def get_delta(self):
        """
        Returns the number of pulses since the last call to get_delta()
        and resets the counter. IRQ-safe (atomic read+clear).

        This is the recommended method for speed calculation.
        """
        state = disable_irq()
        pos = self._position
        self._position = 0
        enable_irq(state)
        return pos

    def get_position(self):
        """Returns the current accumulated position (never resets automatically)."""
        return self._position

    def reset(self):
        """Resets the internal position counter to zero (IRQ-safe)."""
        state = disable_irq()
        self._position = 0
        enable_irq(state)

    def set_invert(self, invert: bool):
        """Change direction inversion at runtime."""
        self._invert = invert

    def deinit(self):
        """Detach IRQ handlers. Call this before re-creating the encoder."""
        try:
            self._pin_a.irq(handler=None)
            self._pin_b.irq(handler=None)
        except Exception:
            pass

    def __repr__(self):
        return "<QuadEncoder {} pos={}>".format(self.name, self._position)
