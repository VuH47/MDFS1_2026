"""
common/espnow_comm.py
ESP-NOW communication layer — now used for both master ↔ slave.

Master typically sends:
  - Drive commands (speed L/R)
  - Servo commands (5 angles)

Slave responds with:
  - Button / bumper states (front + rear)
  - Status / heartbeat

The module is safe to import on both boards.
"""

import struct
from micropython import const

# Example packet format (will be refined)
# '<ffH' = float left, float right, uint16 status
PACKET_FMT = '<ffH'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# Status bits (example)
STATUS_MOVING   = const(1 << 0)
STATUS_FAULT    = const(1 << 1)
STATUS_LOW_BAT  = const(1 << 2)


class ESPNowLink:
    """
    Thin wrapper around espnow (once imported on a board that has it).

    On MicroPython for ESP32 the module is usually available as:
        import espnow
    """

    def __init__(self, peer_mac: bytes | None = None, channel: int = 1):
        self.peer = peer_mac
        self.channel = channel
        self.esp = None
        self._init_espnow()

    def _init_espnow(self):
        try:
            import espnow
            import network
            wlan = network.WLAN(network.STA_IF)
            wlan.active(True)
            # ESP-NOW requires station interface active
            self.esp = espnow.ESPNow()
            self.esp.active(True)
            if self.peer:
                self.esp.add_peer(self.peer)
            print("[espnow] initialized")
        except Exception as e:
            print("[espnow] init failed (probably single-board dev):", e)
            self.esp = None

    def send(self, left: float, right: float, status: int = 0):
        """Send a speed command packet."""
        if self.esp is None:
            return False
        pkt = struct.pack(PACKET_FMT, left, right, status)
        try:
            return self.esp.send(self.peer, pkt)
        except Exception as e:
            print("[espnow] send error:", e)
            return False

    def recv(self, timeout_ms: int = 10):
        """Non-blocking or short timeout receive. Returns (left, right, status) or None."""
        if self.esp is None:
            return None
        try:
            host, msg = self.esp.recv(timeout_ms)
            if msg and len(msg) == PACKET_SIZE:
                left, right, status = struct.unpack(PACKET_FMT, msg)
                return left, right, status
        except Exception:
            pass
        return None

    def close(self):
        if self.esp:
            self.esp.active(False)

    # ------------------------------------------------------------------
    # Convenience helpers for the new servo + button protocol
    # ------------------------------------------------------------------
    def send_servo_cmd(self, angles: list[int]):
        """
        Master → Slave: set 5 servo angles (0-180).
        Packet: [PKT_SERVO_CMD (1B)] + 5 x uint16
        """
        if self.esp is None or not self.peer:
            return False
        try:
            import slave_config as sc   # only on slave or when present
            pkt_type = sc.PKT_SERVO_CMD
        except:
            pkt_type = 0x10

        data = struct.pack("<B5H", pkt_type, * [max(0, min(180, a)) for a in angles])
        return self.esp.send(self.peer, data)

    def send_button_status(self, mask: int, timestamp: int = 0):
        """Slave → Master button report."""
        if self.esp is None or not self.peer:
            return False
        try:
            import slave_config as sc
            pkt_type = sc.PKT_BUTTON_STAT
        except:
            pkt_type = 0x20

        pkt = struct.pack("<BBH", pkt_type, mask & 0xFF, timestamp & 0xFFFF)
        return self.esp.send(self.peer, pkt)