import serial
import re
import time
import matplotlib.pyplot as plt

# === Config ===
SERIAL_PORT = "/dev/ttyACM2"  # or "COM3" on Windows
BAUD_RATE = 115200

# === Initialize plot ===
plt.ion()
fig, ax = plt.subplots()
angle_line, = ax.plot([], [], 'ro-')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_title("Sound Direction")
ax.set_xlabel("X")
ax.set_ylabel("Y")

def angle_to_vector(angle_deg):
    rad = angle_deg * 3.14159 / 180
    return [0, 0], [0.8 * math.cos(rad), 0.8 * math.sin(rad)]

# === Main ===
with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
    print("Connected to", SERIAL_PORT)
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            print("Received:", line)
            if not line.startswith("TDOA:"): continue

            # Parse values
            match = re.match(r"TDOA: ([\d\.\-e]+) ([\d\.\-e]+) ([\d\.\-e]+) ([\d\.\-e]+) ANGLE: ([\d\.\-e]+)", line)
            if not match:
                continue

            ts = [float(match.group(i)) for i in range(1, 5)]
            angle = float(match.group(5))

            print(f"Timestamps: {ts}, Angle: {angle:.2f}°")

            # Update plot
            import math
            x0, y0 = angle_to_vector(angle)
            angle_line.set_xdata([0, x0[1]])
            angle_line.set_ydata([0, y0[1]])
            fig.canvas.draw()
            fig.canvas.flush_events()

        except KeyboardInterrupt:
            break
