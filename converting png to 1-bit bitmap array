from PIL import Image
import os
import re

# === CONFIG ===
input_folder = "frames"        
output_folder = "output_h"     
bitmap_width = 128
bitmap_height = 64
frames_h_file = os.path.join(output_folder, "frames.h")  

os.makedirs(output_folder, exist_ok=True)
print("Script started...")

frame_names = []

# Convert each image to a bitmap header
for filename in sorted(os.listdir(input_folder)):
    if filename.lower().endswith((".png", ".jpg", ".bmp")):
        path = os.path.join(input_folder, filename)
        img = Image.open(path).convert("1")  # convert to black & white
        img = img.resize((bitmap_width, bitmap_height))

        bits = []
        for y in range(bitmap_height):
            byte = 0
            for x in range(bitmap_width):
                # Correct OLED mapping: 1 = white, 0 = black
                pixel = 1 if img.getpixel((x, y)) == 255 else 0
                byte = (byte << 1) | pixel
                if (x % 8) == 7:
                    bits.append(byte)
                    byte = 0

        # Sanitize array name
        array_name = re.sub(r'[^a-zA-Z0-9_]', '_', os.path.splitext(filename)[0])
        frame_names.append(array_name)
        output_path = os.path.join(output_folder, f"{array_name}.h")

        # Write individual .h file
        with open(output_path, "w") as f:
            f.write(f"const unsigned char {array_name}[] PROGMEM = {{\n")
            for i, b in enumerate(bits):
                if i % 12 == 0:
                    f.write("\n  ")
                f.write(f"0x{b:02X}, ")
            f.write("\n};\n")

        print(f"Converted: {filename} -> {output_path}")

# Generate frames.h
with open(frames_h_file, "w") as f:
    f.write("#ifndef FRAMES_H\n#define FRAMES_H\n\n")
    f.write("#include <Arduino.h>\n\n")

    # Include all frame headers
    for name in frame_names:
        f.write(f'#include "{name}.h"\n')
    f.write("\n")

    # Create frames[] array in PROGMEM
    f.write("const unsigned char* frames[] PROGMEM = {\n  ")
    for i, name in enumerate(frame_names):
        f.write(name)
        if i < len(frame_names) - 1:
            f.write(", ")
        if (i + 1) % 5 == 0:
            f.write("\n  ")
    f.write("\n};\n\n")

    # Define frame count
    f.write(f"const int FRAME_COUNT = sizeof(frames) / sizeof(frames[0]);\n\n")
    f.write("#endif\n")

print(f"frames.h generated at {frames_h_file}")
