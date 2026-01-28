# FabLab Internship Projects (Code Repository)

This repository contains separate projects I worked on during my internship at **Singapore Polytechnic FabLab (T1442)**.
Each project is stored in its own folder with its own code and purpose.

---

## 1) PivotLab (CDIO Project)
**Folder:** `CDIO_PivotLab/`  
**Platform:** ESP32 (Arduino IDE, C++)  
**Purpose:** Learning tool for secondary school students to understand **pivot, leverage, and moments**.

### Key features
- OLED UI (menu, info page, icons, custom fonts)
- Rotary encoder navigation + push button logic
- Servo motor control (smooth stepping)
- AS5600 magnetic encoder feedback  
  - I2C for precise angle  
  - Analog OUT for less precise balance detection
- Auto-balance + hold logic (deadband / hysteresis / filtering)
- Saved settings using ESP32 **Preferences (NVS)**

---

## 2) Mood Lamp
**Platform:** MicroPython  
**Purpose:** Wi-Fi + touch + web server control for NeoPixels and servos (interactive light installation).

### Key features
- Connects to Wi-Fi and hosts a web server
- Touch control:
  - Tap = change mode
  - Hold = turn off
- Lighting modes:
  - Static colour cycling
  - Rainbow animation
- Smooth servo movement with randomised behaviour
- Web endpoints:
  - `/on`, `/off`
  - `/rainbow?state=on`
  - `/brightness?value=###`
  - `/color1?...`, `/color2?...`, `/color3?...`


---
## 3) Android App (Mood Lamp Controller)  
**Platform:** Android Studio (Android)
**Purpose:** Mobile app to control the mood lamp (e.g., power, colours, modes, brightness).

### Key features
- Simple UI for turning the lamp on/off
- Colour selection and brightness control
- Mode switching (e.g., static / rainbow, if supported by firmware)
- Sends commands to the device over the configured connection method (e.g., Wi-Fi HTTP endpoints / BLE, depending on your setup)

---


## 4) OLED Image → Header Converter
**Platform:** Python  
**Purpose:** Convert images into `.h` bitmap arrays for OLED rendering.

### Used for
- Icons
- Frames / simple animations
- Custom display assets

---



