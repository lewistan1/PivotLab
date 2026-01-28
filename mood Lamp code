import network, socket, time
from machine import Pin, PWM
import neopixel
import math
import random

# ===== WiFi Setup =====
SSID = "FABLAB-IOT"
PASSWORD = "FABLABiot2025"
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

print("Connecting to Wi-Fi...")
for _ in range(20):
    if wlan.isconnected():
        break
    time.sleep(1)
if wlan.isconnected():
    print("Connected:", wlan.ifconfig())
else:
    print("Failed to connect")
    raise SystemExit

# ===== Hardware Setup =====
NUM_PIXELS = 24

# Light & Servo 1
PIN_NP1 = 5
np1 = neopixel.NeoPixel(Pin(PIN_NP1), NUM_PIXELS)
SERVO_PIN1 = 1
servo1 = PWM(Pin(SERVO_PIN1))
servo1.freq(50)

# Light & Servo 2
PIN_NP2 = 6
np2 = neopixel.NeoPixel(Pin(PIN_NP2), NUM_PIXELS)
SERVO_PIN2 = 2
servo2 = PWM(Pin(SERVO_PIN2))
servo2.freq(50)

# Light 3 (no servo)
PIN_NP3 = 7
np3 = neopixel.NeoPixel(Pin(PIN_NP3), NUM_PIXELS)

# Touch pin
touch = Pin(0, Pin.IN)

# ===== Settings =====
brightness = 0.7
color_index = 0
mode = None
colors = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255),
    (255, 255, 0), (0, 255, 255), (255, 0, 255), (255, 255, 255)
]

press_start = None
hold_time = 2  # seconds to hold for OFF

# Rainbow fade vars
rainbow_index = 0
fade_step = 0
fade_steps = 100
fade_delay = 0.05
last_fade_time = time.ticks_ms()
servo_rainbow_phase1 = 0.0
servo_rainbow_phase2 = 0.0

# Smooth servo movement
target_angle_servo1 = 0
current_angle_servo1 = 0
target_angle_servo2 = 0
current_angle_servo2 = 0


servo_min_angle1 = 60
servo_max_angle1 = 150
servo_min_angle2 = 60
servo_max_angle2 = 160

# Also add random direction factors
servo_dir1 = 1
servo_dir2 = -1
servo_pause_until1 = 0
servo_pause_until2 = 0



# ===== Helper Functions =====
def set_servo_angle(servo, angle):
    """Set the target angle for smooth movement"""
    global target_angle_servo1, target_angle_servo2
    angle = max(0, min(170, angle))
    if servo == servo1:
        target_angle_servo1 = angle
    elif servo == servo2:
        target_angle_servo2 = angle

def update_servos():
    """Gradually move servos toward target angles with smooth slowdown near target"""
    global current_angle_servo1, current_angle_servo2
    step_max = 1.5  # maximum degrees per loop
    step_min = 0.2  # minimum step so it doesn't freeze

    # --- Servo 1 ---
    diff1 = target_angle_servo1 - current_angle_servo1
    if abs(diff1) > 0:
        step1 = max(step_min, min(step_max, abs(diff1) * 0.05))  # smaller when close
        if diff1 > 0:
            current_angle_servo1 += step1
        else:
            current_angle_servo1 -= step1
        # Prevent overshoot
        if abs(target_angle_servo1 - current_angle_servo1) < 0.5:
            current_angle_servo1 = target_angle_servo1

    # --- Servo 2 ---
    diff2 = target_angle_servo2 - current_angle_servo2
    if abs(diff2) > 0:
        step2 = max(step_min, min(step_max, abs(diff2) * 0.05))
        if diff2 > 0:
            current_angle_servo2 += step2
        else:
            current_angle_servo2 -= step2
        if abs(target_angle_servo2 - current_angle_servo2) < 0.5:
            current_angle_servo2 = target_angle_servo2

    # --- Apply PWM ---
    duty1 = int(1638 + (current_angle_servo1 / 180) * (8191 - 1638))
    servo1.duty_u16(duty1)
    duty2 = int(1638 + (current_angle_servo2 / 180) * (8191 - 1638))
    servo2.duty_u16(duty2)

def color_to_angle_servo1(r, g, b):
    """Servo 1 follows red & brightness more"""
    brightness_level = (0.5 * r + 0.3 * g + 0.2 * b) / 255
    return int(brightness_level * 160)

def color_to_angle_servo2(r, g, b):
    """Servo 2 follows blue & green more"""
    brightness_level = (0.2 * r + 0.5 * g + 0.3 * b) / 255
    return int(brightness_level * 160)


def set_color(np, servo, r, g, b):
    """Update all pixels and set servo angle differently for each servo"""
    lr = int(r * brightness)
    lg = int(g * brightness)
    lb = int(b * brightness)
    
    for i in range(NUM_PIXELS):
        np[i] = (lr, lg, lb)
    np.write()

    if mode == "STATIC":
        if servo == servo1:
            target_angle = color_to_angle_servo1(r, g, b)
            set_servo_angle(servo1, target_angle)
        elif servo == servo2:
            target_angle = color_to_angle_servo2(r, g, b)
            set_servo_angle(servo2, target_angle)


def rainbow_cycle_step():
    global rainbow_index, fade_step, last_fade_time
    global servo_rainbow_phase1, servo_rainbow_phase2
    global servo_dir1, servo_dir2
    global servo_min_angle1, servo_max_angle1, servo_min_angle2, servo_max_angle2
    global servo_pause_until1, servo_pause_until2  # new pause timers

    now = time.ticks_ms()

    # --- Fade / color update ---
    if time.ticks_diff(now, last_fade_time) > fade_delay * 1000:
        c1 = colors[rainbow_index % len(colors)]
        c2 = colors[(rainbow_index + 1) % len(colors)]
        r = c1[0] + (c2[0]-c1[0])*fade_step//fade_steps
        g = c1[1] + (c2[1]-c1[1])*fade_step//fade_steps
        b = c1[2] + (c2[2]-c1[2])*fade_step//fade_steps

        set_color(np1, None, r, g, b)
        set_color(np2, None, r, g, b)
        set_color(np3, None, r, g, b)

        fade_step += 1
        if fade_step > fade_steps:
            fade_step = 0
            rainbow_index = (rainbow_index + 1) % len(colors)
        last_fade_time = now

    # --- Servo direction / random change ---
    if random.random() < 0.005:  # rarely flip direction
        servo_dir1 *= -1
        servo_min_angle1 = random.randint(20, 60)
        servo_max_angle1 = random.randint(100, 160)
        servo_pause_until1 = now + random.randint(300, 800)  # pause 0.8–1.5s

    if random.random() < 0.005:
        servo_dir2 *= -1
        servo_min_angle2 = random.randint(20, 60)
        servo_max_angle2 = random.randint(100, 160)
        servo_pause_until2 = now + random.randint(300, 800)  # pause 0.8–1.5s

    # --- Servo movement with pause ---
    if now > servo_pause_until1:  # only move if not in pause
        servo_rainbow_phase1 += servo_dir1 * (0.02 + random.uniform(0, 0.001))
        angle1 = int((math.sin(servo_rainbow_phase1) + 1) / 2 * (servo_max_angle1 - servo_min_angle1) + servo_min_angle1)
        set_servo_angle(servo1, angle1)

    if now > servo_pause_until2:
        servo_rainbow_phase2 += servo_dir2 * (0.03 + random.uniform(0, 0.001))
        angle2 = int((math.sin(servo_rainbow_phase2) + 1) / 2 * (servo_max_angle2 - servo_min_angle2) + servo_min_angle2)
        set_servo_angle(servo2, angle2)



def touch_next_mode():
    global color_index, mode
    if mode is None:
        mode = "STATIC"
        color_index = 0
        r, g, b = colors[color_index]
        set_color(np1, servo1, r, g, b)
        set_color(np2, servo2, r, g, b)
        set_color(np3, None, r, g, b)
        return

    if mode == "STATIC":
        color_index += 1
        if color_index >= len(colors):
            mode = "RAINBOW"
            color_index = 0
        else:
            r, g, b = colors[color_index]
            set_color(np1, servo1, r, g, b)
            set_color(np2, servo2, r, g, b)
            set_color(np3, None, r, g, b)
    elif mode == "RAINBOW":
        mode = "STATIC"
        color_index = 0
        r, g, b = colors[color_index]
        set_color(np1, servo1, r, g, b)
        set_color(np2, servo2, r, g, b)
        set_color(np3, None, r, g, b)

def touch_turn_off():
    global mode
    mode = None

    # Turn off all lights
    set_color(np1, None, 0, 0, 0)
    set_color(np2, None, 0, 0, 0)
    set_color(np3, None, 0, 0, 0)
    set_servo_angle(servo1, 0)
    set_servo_angle(servo2, 0)
    # Move servos to resting position once
    # Then stop the signal to prevent vibration
    servo_off(servo1)
    servo_off(servo2)

    # Wait until touch is released (avoid infinite loop)

    
    
def servo_off(servo):
    servo.duty_u16(0)  # stop signal (servo will relax)


# ===== Initialize =====
mode = None
set_color(np1, None, 0, 0, 0)
set_color(np2, None, 0, 0, 0)
set_color(np3, None, 0, 0, 0)
set_servo_angle(servo1, 0)
set_servo_angle(servo2, 0)
servo_off(servo1)
servo_off(servo2)


# ===== Web Server =====
addr = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
s.setblocking(False)
print("🌐 Listening on", addr)

# ===== Main Loop =====
while True:
    now = time.time()

    # --- Touch handling ---
    state = touch.value()
    if state == 1:
        if press_start is None:
            press_start = now
        elif now - press_start >= hold_time:
            touch_turn_off()
    else:
        if press_start is not None:
            duration = now - press_start
            if duration >= hold_time:
                touch_turn_off()
            else:
                touch_next_mode()
            press_start = None

    # --- Rainbow fade and servo targets ---
    if mode == "RAINBOW":
        rainbow_cycle_step()

    # --- Gradually move servos ---
    update_servos()

    # --- Web server ---
    try:
        cl, addr = s.accept()
        request = cl.recv(1024).decode()
        response = "OK"

        # --- Web Commands ---
        if "/on" in request:
            mode = "STATIC"
            color_index = 0
            r, g, b = colors[color_index]
            set_color(np1, servo1, r, g, b)
            set_color(np2, servo2, r, g, b)
            set_color(np3, None, r, g, b)
        elif "/off" in request:
            touch_turn_off()
        elif "/rainbow?state=on" in request:
            mode = "RAINBOW"
            rainbow_index = 0
            fade_step = 0
            servo_rainbow_phase1 = 0
            servo_rainbow_phase2 = 0
        elif "/brightness?value=" in request:
            try:
                val = int(request.split("/brightness?value=")[1].split(" ")[0])
                brightness = max(0.0, min(val/255, 1.0))
                if mode == "STATIC":
                    r, g, b = colors[color_index]
                    set_color(np1, servo1, r, g, b)
                    set_color(np2, servo2, r, g, b)
                    set_color(np3, None, r, g, b)
            except:
                pass
        # --- Color commands ---
        for n, servo, np in [(1, servo1, np1), (2, servo2, np2), (3, None, np3)]:
            key = f"/color{n}?"
            if key in request:
                try:
                    params = request.split(key)[1].split(" ")[0]
                    r = int(params.split("r=")[1].split("&")[0])
                    g = int(params.split("g=")[1].split("&")[0])
                    b = int(params.split("b=")[1])
                    mode = "STATIC"
                    set_color(np, servo, r, g, b)
                except:
                    pass

        cl.send("HTTP/1.0 200 OK\r\nContent-type: text/plain\r\n\r\n" + response)
        cl.close()
    except OSError:
        pass

    time.sleep(0.01)





