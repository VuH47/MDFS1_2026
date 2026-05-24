"""
boot.py
This runs automatically on every boot, BEFORE main.py.

Purpose (temporary for development):
- Connect to your WiFi so you can use WebREPL wirelessly.
- This lets you control and run scripts on the robot from your laptop
  without being limited by USB cable length (very useful for calibration).

After calibration is done, we can make radio startup conditional
(only enable during teaching mode, as planned in DESIGN.md).
"""

import time
import network
import webrepl
import config

# ====================== USER CONFIGURATION ======================
# <<< EDIT THESE TWO LINES WITH YOUR ACTUAL WiFi CREDENTIALS >>>
WIFI_SSID = "Optus_B628_2D98"
WIFI_PASSWORD = "R2m24E5mdbM"
# ================================================================

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    
    # Robust reset of WiFi state (helps with "Wifi Internal State Error")
    wlan.active(False)
    time.sleep(1)
    wlan.active(True)
    time.sleep(1)

    if not wlan.isconnected():
        print("[boot] Connecting to WiFi:", WIFI_SSID)
        try:
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        except OSError as e:
            print("[boot] wlan.connect() raised OSError:", e)
            return None

        # Wait up to 15 seconds for connection
        timeout = 15
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            time.sleep(1)
            timeout -= 1
        print()

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print("[boot] WiFi connected! IP address:", ip)
        return ip
    else:
        # Print diagnostic status code
        status = wlan.status()
        status_meaning = {
            1000: "No connection (yet)",
            1010: "Connection lost",
            202:  "Wrong password",
            201:  "No AP found / SSID not visible",
            203:  "Authentication failed",
            204:  "Association failed",
        }.get(status, f"Unknown status code: {status}")

        print("[boot] Failed to connect to WiFi.")
        print(f"[boot] Status code: {status} → {status_meaning}")
        print("[boot] Common fixes:")
        print("  - Double-check SSID and password (case sensitive)")
        print("  - Make sure the network is 2.4 GHz (not 5 GHz only)")
        print("  - Power cycle the ESP32 completely (unplug USB for 10 seconds)")
        return None


# Connect to WiFi
ip_address = connect_wifi()

# Start WebREPL (auto-configured with a default password for development)
try:
    import os

    webrepl_configured = False
    try:
        files = os.listdir()
        if 'webrepl_cfg.py' in files or '_webrepl_cfg.py' in files:
            webrepl_configured = True
    except:
        pass

    if not webrepl_configured:
        # Create a simple webrepl config with default password "1234"
        with open('_webrepl_cfg.py', 'w') as f:
            f.write("PASS = '1234'\n")
        print("[boot] WebREPL auto-configured with password: 1234")
        webrepl_configured = True

    webrepl.start()
    if ip_address:
        print("[boot] WebREPL started successfully.")
        print("[boot] Open in browser: http://{}:8266/".format(ip_address))
        print("[boot] Password: 1234")
    else:
        print("[boot] WebREPL started (no WiFi IP).")

except Exception as e:
    print("[boot] Could not start WebREPL:", e)

print("[boot] boot.py finished. Starting main.py...")