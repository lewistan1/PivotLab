# PivotLab
<img width="765" height="635" alt="image" src="https://github.com/user-attachments/assets/71b5fca6-1ab1-4f1c-b7a0-b4935a9525dd" />


PivotLab features multiple operating modes, real-time angle feedback, closed-loop servo control, and an OLED user interface.

Designed for **educational demonstrations**, **interactive gameplay**, and **automatic balance control** using an **AS5600 magnetic encoder**.

---

## ✨ Key Features

- Closed-loop **servo position control** using AS5600 feedback  
- OLED **menu-driven UI** (SSD1306 128×64)  
- Rotary encoder navigation with push-button input  
- Multiple operating modes:
  - Normal Mode
  - Infinite Mode
  - Education Mode
  - Auto-Balance Mode
- Persistent settings storage using **ESP32 Preferences (NVS)**  
- Smooth servo motion with:
  - Backlash compensation
  - Direction-change pauses
  - Speed ramping & slow zones
- Educational question system:
  - Numeric questions
  - Word-based questions
  - True / False logic  

---

## 🔌 Hardware Overview

| Component | Quantity |
|---------|----------|
| ESP32 | 1 |
| Servo Motor | 1 |
| AS5600 Magnetic Encoder | 2 |
| OLED Display (SSD1306 128×64) | 1 |
| Rotary Encoder (with button) | 1 |
| External Power Supply | 1 |

---

## 📍 Pin Configuration

| Function | ESP32 Pin |
|--------|-----------|
| Servo PWM | GPIO 4 |
| Encoder CLK | GPIO 25 |
| Encoder DT | GPIO 33 |
| Encoder Button | GPIO 32 |
| AS5600 Analog OUT | GPIO 34 |
| OLED SDA | GPIO 21 |
| OLED SCL | GPIO 22 |
| AS5600 SDA | GPIO 21 |
| AS5600 SCL | GPIO 22 |

---

## 🧭 Operating Modes

### Normal Mode
- Balance the lever to the target angle  
- Servo moves to a new random position after each success  
- Complete **10 successful balances** to finish  

---

### Infinite Mode
- Endless balancing gameplay  
- Difficulty increases dynamically  
- Game ends if balancing takes too long  

---

### Education Mode
- Lever must be balanced before answering  
- Question types include:
  - Force calculation
  - Moment calculation
  - Direction / stability interpretation
  - True / False questions  
- Immediate feedback provided  

---

### Auto-Balance Mode
- Fully automatic servo control  
- Continuously minimizes angle error  
- Adaptive speed control:
  - Fast when far from balance
  - Slow near balance
  - Super-slow fine tuning  
- Displays live:
  - Angle
  - Error
  - Servo position
  - Control state  

---

## 🖥 User Interface

- Rotate encoder to navigate menus  
- Press encoder to confirm selections  
- OLED displays:
  - Mode selection
  - Real-time angle feedback
  - Game statistics
  - Education questions & answers
  - Auto-balance state information  

---

## 🧠 Control Strategy Overview

- AS5600 **ADC output** used for smooth motion tracking  
- AS5600 **I²C read** used for precision holding  
- Dual-sensor strategy:
  - **ADC** → motion & detection
  - **I²C** → holding & accuracy  

Includes:
- Exponential Moving Average (EMA) filtering  
- Trimmed-mean ADC sampling  
- Deadband logic  
- Hysteresis-based hold control  

---

## 💾 Persistent Settings

Stored using **ESP32 Preferences (NVS)**:
- Servo target angles  
- Auto-balance center angle  
- Calibration parameters  

Settings persist across power cycles.

---

## 🚀 Setup Instructions

1. Install required Arduino libraries  
2. Wire hardware according to pin configuration  
3. Flash firmware to the ESP32  
4. Power the system using a **stable external supply**  
5. Navigate menus using the rotary encoder  



## 👤 Author

**Lewis Tan**  
Singapore Polytechnic – EEE / FabLab  
Educational Mechatronics & Interactive Systems Project
