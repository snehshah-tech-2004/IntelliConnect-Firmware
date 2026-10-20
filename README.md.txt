# Sigma Controls ESP32 Modbus Data Logger

ESP32 firmware for an industrial wireless data logger. It reads **RS-485 Modbus RTU** devices (multi-channel temperature scanners and PID controllers), saves readings to an **SD card** as CSV, and serves a **Wi-Fi web dashboard** with live tables, graphs, history and file download. Firmware can be updated over the air from **GitHub Releases**.

> Built for Sigma Controls. Proprietary: all rights reserved. Do not copy or redistribute without written permission.

**Firmware version:** 1.1.1 (build 3)

---

## Features

### Data acquisition
- Modbus RTU master over RS-485 (UART2)
- Up to **16 slave devices**, each enabled and configured independently
- Two logger types:
  - **TempScanner**: reads 8, 16 or 24 channels per slave, saves 1 to 24 of them
  - **PID**: reads one PV register and one SV register per slave
- Configurable serial settings: baud 1200 to 115200, parity (None/Even/Odd), 1 or 2 stop bits, 7 or 8 data bits, Modbus timeout 100 to 10000 ms
- Division factor and display unit for scaling raw register values
- One offline slave never stops the others: working slaves keep logging and the UI reports which slave ID failed

### Storage
- CSV files on SD card, one file per slave per day:
  `DD-MM-YYYY_<LoggerType>_ID<slaveID>.csv`
- Buffered batch writes to reduce SD wear
- A per-day user action log (start, stop, settings used)
- Settings saved to `config.json` on the SD card
- **Auto-resume** after a power cut, unless the user pressed Stop

### Web dashboard
- Works on a phone or PC browser, with light and dark themes
- Live monitor: last 10 readings per slave, with charts
- Full history: paginated, 200 rows per page
- Calendar-based file browser, multi-file download, and in-browser ZIP (no library needed)
- Status LEDs for power, COMM/Modbus and SD card
- Firmware and update panel

### Networking
- **SoftAP is always on** at `192.168.4.1` as a recovery and setup access point, with a captive-portal DNS so phones stay connected
- Station mode joins your home or office Wi-Fi (set from the dashboard)
- After the first successful join, the device locks to a fixed IP `x.x.x.200` on that network
- mDNS name: `http://sigma.local`
- Charts use Chart.js from a CDN, so they appear on Station Wi-Fi with internet; on SoftAP the tables still work

### Time
- NTP sync (India Standard Time, UTC+5:30) when internet is available
- Software clock fallback; last good time is saved in NVS and restored at boot
- The browser can set the time **once** if the device has none; after that, phones cannot change it (prevents timestamp collisions)

### OTA updates
- Reads a `version.json` manifest from GitHub and compares the build number
- Checks 45 s after Station Wi-Fi connects, then every 12 hours, or manually from the dashboard
- Never updates while logging
- Only accepts HTTPS firmware URLs hosted on GitHub
- Rollback protection: a new firmware is marked valid after 45 s of stable running

---

## Hardware

| Part | Notes |
|---|---|
| ESP32 DevKit (`esp32doit-devkit-v1`) | Main controller |
| RS-485 transceiver module | Must have **automatic direction control**; the firmware does not drive a DE/RE pin |
| microSD card module (SPI) | Default SPI pins, CS on GPIO 4 |
| Red LED, blue LED, buzzer | Status indication |

### Pin map

| Function | GPIO |
|---|---|
| RS-485 RX (UART2) | 16 |
| RS-485 TX (UART2) | 17 |
| SD card CS | 4 |
| Red LED (COMM/Modbus error) | 25 |
| Blue LED (SD error / SoftAP only) | 26 |
| Buzzer | 27 |

SD SPI uses the ESP32 default VSPI pins (SCK 18, MISO 19, MOSI 23).

---

## Build and flash

Requires [PlatformIO](https://platformio.org/) (VS Code extension or CLI).

1. Clone this repository.
2. Create your private credentials file from the template:
   ```
   copy include\secrets.h.example include\secrets.h
   ```
3. Edit `include/secrets.h` and set:
   - `AP_SSID_VALUE` and `AP_PASSWORD_VALUE`: the SoftAP name and password
   - `STA_SSID_DEFAULT_VALUE` and `STA_PASS_DEFAULT_VALUE`: default home Wi-Fi (can be left empty and set later from the dashboard)

   `include/secrets.h` is listed in `.gitignore` and must never be committed.
4. Check `upload_port` in `platformio.ini` (currently `COM6`) and change it to your port.
5. Build and upload:
   ```
   pio run -t upload
   pio device monitor
   ```

The project uses a custom partition table (`min_spiffs.csv`) so two firmware slots fit for OTA and rollback. `platformio.ini` also sets `erase_flash` on upload, which wipes saved Wi-Fi settings.

Libraries (installed automatically): `ModbusMaster`, `ArduinoJson` ^7.2.2, `WiFi`.

---

## First use

1. Insert a FAT-formatted SD card and power on.
2. Connect your phone to the SoftAP and open `http://192.168.4.1` (the repo includes `sigma_qr.png` and `QR_Code_Gen.py` for making a QR code to that page).
3. In **WiFi Setup**, enter your home or office 2.4 GHz network and save. The dashboard then shows your daily link (`http://x.x.x.200`).
4. Choose the logger type (it locks after the first start), set serial settings, enable your slaves, and tap **Start Logging**.

---

## OTA release process

1. Increase `FIRMWARE_VERSION` and `FIRMWARE_BUILD` in `src/main.cpp`.
2. Build and publish the `.bin` as a GitHub Release asset.
3. Update `version.json` in the OTA manifest repo (path set by `OTA_VERSION_URL` in `main.cpp`):
   ```json
   {
     "version": "1.1.2",
     "build": 4,
     "url": "https://github.com/<owner>/<repo>/releases/download/v1.1.2/firmware.bin"
   }
   ```
4. Devices update when the remote `build` is higher than their own.

---

## Project layout

```
src/main.cpp            Firmware: Modbus, SD logging, web server, Wi-Fi, NTP, OTA
include/secrets.h.example   Template for private credentials
min_spiffs.csv          Partition table
platformio.ini          PlatformIO build configuration
QR_Code_Gen.py          Generates the QR code for the setup page
sigma_qr.png            QR code image
```

---

## Security notes

- The dashboard has **no login** in this firmware. Anyone on the same network, or on the SoftAP, can open it. Set a strong SoftAP password in `secrets.h`.
- OTA downloads use HTTPS but do not verify the server certificate. Adding certificate pinning is a recommended improvement.
- Wi-Fi credentials saved from the dashboard are stored in device flash (NVS) and in `config.json` on the SD card.