"""
boot.py for the SLAVE board

Same WiFi + WebREPL setup as master for easy development.
After flashing, you can connect via WebREPL to inspect buttons/servos live.
"""

import time
import network
import webrepl
import slave_config as cfg

WIFI_SSID = "Optus_B628_2D98"          # <-- change to your network
WIFI_PASSWORD = "R2m24E5mdbM"

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)
    time.sleep(0.5)
    wlan.active(True)
    time.sleep(0.5)

    if not wlan.isconnected():
        print("[slave-boot] Connecting to WiFi:", WIFI_SSID)
        try:
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        except OSError as e:
            print("[slave-boot] connect error:", e)
            return None

        for _ in range(12):
            if wlan.isconnected():
                break
            print(".", end="")
            time.sleep(1)
        print()

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print("[slave-boot] WiFi connected. IP:", ip)
        return ip
    else:
        print("[slave-boot] WiFi connection failed")
        return None


ip = connect_wifi()

# WebREPL for wireless development (password 1234)
try:
    webrepl.start()
    if ip:
        print("[slave-boot] WebREPL ready: http://{}:8266/  (pw: 1234)".format(ip))
except Exception as e:
    print("[slave-boot] WebREPL error:", e)

print("[slave-boot] boot finished. Will run main.py / slave_main.py ...")