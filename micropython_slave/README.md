# micropython_slave

Firmware for the second ESP32-WROOM board (the "peripheral slave").

## Responsibilities
- Drive 5 hobby servos
- Read 4 bumper / push buttons (2 front + 2 rear)
- Respond instantly to ESP-NOW commands from the master board
- Report button states back to the master for safety logic

## Quick Flash (after first setup)

```bash
cd /mnt/d/PROJECTS/21_05_PROJECT/micropython_slave

mpremote connect <SLAVE_PORT> cp boot.py :boot.py
mpremote connect <SLAVE_PORT> cp slave_main.py :main.py
mpremote connect <SLAVE_PORT> cp slave_config.py :slave_config.py
mpremote connect <SLAVE_PORT> cp -r common :
mpremote connect <SLAVE_PORT> cp -r drivers :
```

See [FLASHING_SLAVE.md](./FLASHING_SLAVE.md) for the full first-time procedure and important power notes for servos.

## Key Files
- `slave_config.py` — pin map + packet definitions
- `slave_main.py` — the actual "responding task" application (rename to `main.py` on target)
- `drivers/servo.py` — clean 0-180° servo driver with speed limiting
- `common/espnow_comm.py` — shared protocol helpers

## Development Tips
- Use WebREPL (enabled in `boot.py`) for live debugging of buttons and servos.
- Keep the slave powered with a good 5V supply when testing multiple servos under load.
- Set `MASTER_MAC` in `slave_config.py` after you see the master's MAC on first boot.

This board is deliberately kept simple and highly responsive so the master can treat it as a fast "servo + bumper" peripheral.