# PivotLab
<img width="765" height="635" alt="PivotLab system overview" src="https://github.com/user-attachments/assets/71b5fca6-1ab1-4f1c-b7a0-b4935a9525dd" />

**PivotLab** is an ESP32-based interactive lever-balancing system featuring multiple operating modes, real-time angle feedback, closed-loop servo control, and an OLED-based user interface.

The system is designed for **educational demonstrations**, **interactive gameplay**, and **automatic balance control**, utilising **AS5600 magnetic encoders** for precise angular feedback.

---

## ✨ Key Features

- Closed-loop **servo position control** using AS5600 feedback  
- OLED **menu-driven user interface** (SSD1306 128×64)  
- Rotary encoder navigation with push-button selection  
- Multiple operating modes:
  - Normal Mode
  - Infinite Mode
  - Education Mode
  - Auto-Balance Mode
- Persistent settings storage using **ESP32 Preferences (NVS)**  
- Smooth servo motion with:
  - Backlash compensation
  - Direction-change pauses
  - Speed ramping and slow zones
- Integrated educational question system:
  - Numeric questions
  - Word-based interpretation questions
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
| Rotary Encoder CLK | GPIO 25 |
| Rotary Encoder DT | GPIO 33 |
| Rotary Encoder Button | GPIO 32 |
| AS5600 Analog OUT | GPIO 34 |
| OLED SDA | GPIO 21 |
| OLED SCL | GPIO 22 |
| AS5600 SDA | GPIO 21 |
| AS5600 SCL | GPIO 22 |

---

## 🧭 Operating Modes

### Normal Mode
- Users balance the lever to a target angle  
- Upon success, the servo moves to a new random position  
- Complete **10 successful balances** to finish the mode  

---

### Infinite Mode
- Continuous balancing gameplay with no fixed endpoint  
- Difficulty increases dynamically over time  
- Mode ends if the lever remains unbalanced for too long  

---

### Education Mode
- Lever must first be balanced before answering questions  
- Question types include:
  - Force calculation
  - Moment calculation
  - Direction and stability interpretation
  - True / False conceptual questions  
- Immediate feedback is provided after each response  

---

### Auto-Balance Mode
- Fully automatic closed-loop balance control  
- Servo continuously minimises angular error  
- Adaptive speed control:
  - Fast movement when far from balance
  - Slower correction near equilibrium
  - Super-slow fine tuning for precision holding  
- Live display of:
  - Current angle
  - Error value
  - Servo position
  - Control state  

---

## 🖥 User Interface

- Rotate the encoder to navigate menus  
- Press the encoder to confirm selections  
- OLED displays:
  - Mode selection screens
  - Real-time angle feedback
  - Game progress and statistics
  - Education questions and answers
  - Auto-balance diagnostic information  

---

## 🧠 Control Strategy Overview

PivotLab employs a **dual-sensor feedback strategy** using the AS5600 magnetic encoder, where different outputs are used for different control purposes.

### ADC Output – Balance Detection
- The **analog (ADC) output** of the AS5600 is used for:
  - Continuous lever angle monitoring
  - Balance detection and threshold checking
  - Fast response during user interaction
- Advantages:
  - Low-latency readings
  - Smooth signal behaviour after filtering
  - Ideal for detecting when the lever enters or leaves the balanced zone

### I²C Output – Servo Angle Detection
- The **digital I²C angle readout** of the AS5600 is used for:
  - Precise servo angle measurement
  - Closed-loop servo position control
  - Stable reference during auto-balance operation
- Advantages:
  - Higher angular resolution
  - Reduced noise and drift compared to ADC
  - Accurate feedback for fine servo adjustments

### Combined Control Strategy
By separating responsibilities between ADC and I²C measurements:
- ADC provides **fast and stable balance detection**
- I²C provides **high-precision servo angle feedback**

This hybrid approach improves both **responsiveness** and **control stability**, while reducing oscillation and jitter.

### Additional Control Techniques
- Exponential Moving Average (EMA) filtering  
- Trimmed-mean ADC sampling  
- Deadband logic to prevent servo chatter  
- Hysteresis-based hold control for stable equilibrium  

---

## 💾 Persistent Settings

System parameters are stored using **ESP32 Preferences (NVS)**, including:
- Servo target angles  
- Auto-balance centre angle  
- Calibration and tuning parameters  

All settings persist across power cycles.

---

## 🖋 Custom Font Rendering

The OLED interface uses a **custom bitmap font** instead of default library fonts to improve readability and layout control.

- Designed specifically for **128×64 SSD1306 displays**
- Fixed-width characters enable:
  - Clean menu alignment
  - Accurate text centring
  - Consistent numeric spacing
- Optimised for:
  - Educational clarity
  - High-contrast visibility
  - Reduced redraw flicker

This provides full control over typography, which is critical for small-display educational systems.

---

## 🎞 Animated Start-Up Menu

On power-up, the system displays an **animated splash screen** rendered frame-by-frame on the OLED.

- Bitmap frames stored directly in firmware memory  
- Timing-controlled playback for smooth animation  
- No external storage (e.g. SD card) required  

The animation:
- Indicates successful system boot  
- Engages users before interaction begins  
- Creates a polished, product-like first impression  

After the animation completes, the system automatically transitions to the **main mode selection menu**.

---

## 🚀 Setup Instructions

1. Install the required Arduino libraries  
2. Wire all hardware according to the pin configuration  
3. Flash the firmware to the ESP32  
4. Power the system using a **stable external supply**  
5. Navigate the interface using the rotary encoder  

---

## 👤 Author

**Lewis Tan**  
Singapore Polytechnic – EEE / FabLab  
Educational Mechatronics & Interactive Systems Project
