"""
slave_main.py
Main application for the **slave peripheral board**.

Responsibilities:
- Drive 5 servos according to commands received over ESP-NOW from master
- Continuously monitor 4 push buttons / bumpers (2 front, 2 rear)
- Send button state changes + periodic status back to master
- Be highly responsive ("responding task")

Run on the slave ESP32 after flashing:
    mpremote cp slave_main.py :main.py
    (and all required modules: slave_config.py, drivers/servo.py, common/espnow_comm.py)

The board will boot into this logic (rename to main.py on the slave filesystem).
"""

import time
import struct
from machine import Pin
import network

import slave_config as cfg
from drivers.servo import Servo
from common.espnow_comm import ESPNowLink   # we will enhance this module

# -----------------------------------------------------------------------------
# Hardware objects
# -----------------------------------------------------------------------------
servos = []
buttons = {}
esp = None

# Current button state (bitmask)
button_state = 0
last_sent_button_state = 0
last_button_send_ms = 0

# -----------------------------------------------------------------------------
# Button handling (simple polled + debounce)
# -----------------------------------------------------------------------------
def _read_buttons_raw() -> int:
    mask = 0
    for name, pin in cfg.BUTTON_PINS.items():
        # Buttons are active-low (pressed = 0)
        if not pin.value():
            bit = {
                "front_left":  cfg.BTN_FRONT_LEFT,
                "front_right": cfg.BTN_FRONT_RIGHT,
                "rear_left":   cfg.BTN_REAR_LEFT,
                "rear_right":  cfg.BTN_REAR_RIGHT,
            }[name]
            mask |= bit
    return mask


def _update_buttons():
    global button_state
    raw = _read_buttons_raw()

    # Very simple debounce: only change if stable for a few reads
    # (for production you can use IRQ + timer debounce)
    stable = raw
    for _ in range(3):
        time.sleep_ms(2)
        if _read_buttons_raw() != stable:
            return   # unstable, ignore this sample

    if stable != button_state:
        button_state = stable
        if cfg.DEBUG:
            pressed = []
            if button_state & cfg.BTN_FRONT_LEFT:  pressed.append("FL")
            if button_state & cfg.BTN_FRONT_RIGHT: pressed.append("FR")
            if button_state & cfg.BTN_REAR_LEFT:   pressed.append("RL")
            if button_state & cfg.BTN_REAR_RIGHT:  pressed.append("RR")
            print("[slave] Buttons pressed:", pressed or "none")


def _send_button_status(force: bool = False):
    global last_sent_button_state, last_button_send_ms
    now = time.ticks_ms()

    changed = (button_state != last_sent_button_state)
    periodic = (time.ticks_diff(now, last_button_send_ms) > 200)  # 5 Hz status

    if not (changed or periodic or force):
        return

    if esp and esp.esp:
        # Packet: type (1B) + button_mask (1B) + timestamp_low (2B)
        pkt = struct.pack("<BBH", cfg.PKT_BUTTON_STAT, button_state, now & 0xFFFF)
        try:
            esp.esp.send(cfg.MASTER_MAC, pkt)
        except Exception as e:
            if cfg.DEBUG:
                print("[slave] send status failed:", e)

    last_sent_button_state = button_state
    last_button_send_ms = now


# -----------------------------------------------------------------------------
# Servo command handler
# -----------------------------------------------------------------------------
def _handle_servo_cmd(data: bytes):
    """Expect 5 x uint16 (0-180) after the type byte."""
    if len(data) < 1 + 5 * 2:
        print("[slave] bad servo cmd len")
        return

    angles = struct.unpack("<5H", data[1:11])
    if cfg.DEBUG:
        print("[slave] Servo cmd:", angles)

    for i, ang in enumerate(angles):
        if i < len(servos):
            # Clamp and move (instant for responsiveness, or add speed later)
            ang = max(0, min(180, ang))
            servos[i].set_angle(ang)


# -----------------------------------------------------------------------------
# ESP-NOW receive callback / polling
# -----------------------------------------------------------------------------
def _on_recv():
    """Called periodically to process incoming ESP-NOW packets."""
    if not esp or not esp.esp:
        return

    try:
        host, msg = esp.esp.recv(0)   # non-blocking
        if msg and len(msg) >= 1:
            pkt_type = msg[0]
            if pkt_type == cfg.PKT_SERVO_CMD:
                _handle_servo_cmd(msg)
            elif pkt_type == cfg.PKT_HEARTBEAT:
                # optional: respond with heartbeat
                pass
    except Exception as e:
        if cfg.DEBUG:
            print("[slave] recv error:", e)


# -----------------------------------------------------------------------------
# Main setup
# -----------------------------------------------------------------------------
def setup():
    global servos, buttons, esp

    print("=" * 60)
    print(" ROBOT SLAVE PERIPHERAL  |  5 Servos + 4 Buttons")
    print("=" * 60)

    # --- Servos ---
    print("[slave] Initializing 5 servos...")
    for i, pin in enumerate(cfg.SERVO_PINS):
        s = Servo(pin=pin,
                  min_us=cfg.SERVO_MIN_US,
                  max_us=cfg.SERVO_MAX_US,
                  freq=cfg.SERVO_FREQ,
                  name=cfg.SERVO_NAMES[i])
        servos.append(s)
        # Start at safe center position
        s.set_angle(90)
    print("  Servos ready at 90°")

    # --- Buttons ---
    print("[slave] Initializing 4 buttons...")
    for name, pin_num in cfg.BUTTON_PINS.items():
        p = Pin(pin_num, Pin.IN, Pin.PULL_UP)
        buttons[name] = p
    print("  Buttons ready (active low)")

    # --- ESP-NOW ---
    print("[slave] Starting ESP-NOW (slave role)...")
    if cfg.MASTER_MAC is None:
        print("  !!! WARNING: MASTER_MAC is None in slave_config.py")
        print("  Set it to the master's MAC address after first boot.")

    esp = ESPNowLink(peer_mac=cfg.MASTER_MAC, channel=cfg.ESP_NOW_CHANNEL)

    # Make sure we can receive from master even if add_peer was only one way
    if esp.esp and cfg.MASTER_MAC:
        try:
            esp.esp.add_peer(cfg.MASTER_MAC)
        except:
            pass

    print("[slave] Setup complete. Entering main loop...")
    return esp


# -----------------------------------------------------------------------------
# Main loop (responding task)
# -----------------------------------------------------------------------------
def loop():
    global esp
    last_status = time.ticks_ms()

    while True:
        now = time.ticks_ms()

        # 1. Read and debounce buttons
        _update_buttons()

        # 2. Send status if changed or periodically
        _send_button_status()

        # 3. Process incoming servo commands
        _on_recv()

        # 4. Very light heartbeat every 1 s
        if time.ticks_diff(now, last_status) > 1000:
            last_status = now
            if esp and esp.esp and cfg.MASTER_MAC:
                try:
                    pkt = struct.pack("<B", cfg.PKT_HEARTBEAT)
                    esp.esp.send(cfg.MASTER_MAC, pkt)
                except:
                    pass

        # 5. Yield — keep loop responsive (~50-100 Hz is fine)
        time.sleep_ms(10)


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        esp_link = setup()
        loop()
    except KeyboardInterrupt:
        print("\n[slave] Stopped by user")
        for s in servos:
            try:
                s.detach()
            except:
                pass
    except Exception as e:
        print("[slave] FATAL:", e)
        import sys
        sys.print_exception(e)