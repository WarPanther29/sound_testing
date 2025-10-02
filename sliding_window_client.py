
import time
import sys
import json
from smbus2 import SMBus, i2c_msg
import paho.mqtt.client as mqtt
import socket
import math
from collections import deque

# MQTT
MQTT_BROKER = "192.168.178.56"
MQTT_PORT = 1883
# MQTT_TOPIC is set dynamically from hostname in main()

# e-puck2
I2C_CHANNEL = 12
LEGACY_I2C_CHANNEL = 4
ROB_ADDR = 0x1F
ACTUATORS_SIZE = 20
SENSORS_SIZE = 47

# Sliding window config
WINDOW_SIZE = 10           # number of frames used for RMS
INTER_FRAME_SLEEP = 0.0    # seconds between frames (set >0 to throttle)

try:
    bus = SMBus(I2C_CHANNEL)
except Exception:
    try:
        bus = SMBus(LEGACY_I2C_CHANNEL)
    except Exception:
        print("Cannot open I2C device")
        sys.exit(1)

actuators_data = bytearray([0] * ACTUATORS_SIZE)
sensors_data = bytearray([0] * SENSORS_SIZE)


def update_robot():
    global sensors_data
    try:
        write = i2c_msg.write(ROB_ADDR, actuators_data)
        read = i2c_msg.read(ROB_ADDR, SENSORS_SIZE)
        bus.i2c_rdwr(write, read)
        sensors_data = list(read)
        return True
    except Exception:
        return False


def verify_checksum():
    checksum = 0
    for i in range(SENSORS_SIZE - 1):
        checksum ^= sensors_data[i]
    return checksum == sensors_data[SENSORS_SIZE - 1]


def read_mic_values():
    # mics are little-endian 16-bit, 4 channels starting at 32
    mic = [0] * 4
    for i in range(4):
        low = sensors_data[32 + i * 2]
        high = sensors_data[32 + i * 2 + 1]
        mic[i] = high * 256 + low
    return mic


def read_one_frame():
    """Read one valid mic frame from the robot (blocking until checksum passes)."""
    while True:
        if update_robot() and verify_checksum():
            return read_mic_values()
        else:
            # avoid tight loop on repeated failures
            time.sleep(0.001)


def main():
    client = mqtt.Client()
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    hostname = socket.gethostname()
    number = ''.join(filter(str.isdigit, hostname[len('pi-puck'):]))
    MQTT_TOPIC = f"robots/{number}/mics"

    # Sliding window of recent frames and running sum of squares per channel
    window = deque(maxlen=WINDOW_SIZE)
    sumsqs = [0.0, 0.0, 0.0, 0.0]

    # Prime the window with initial frames
    while len(window) < WINDOW_SIZE:
        frame = read_one_frame()
        window.append(frame)
        for ch in range(4):
            v = float(frame[ch])
            sumsqs[ch] += v * v

    # Publish immediately with the initial window
    def publish_current_rms():
        count = len(window)
        if count == 0:
            rms_vals = [0.0, 0.0, 0.0, 0.0]
        else:
            rms_vals = [math.sqrt(s / count) for s in sumsqs]
        payload = json.dumps({f"m{i}": rms_vals[i] for i in range(4)})
        result = client.publish(MQTT_TOPIC, payload)
        if result[0] == 0:
            print("RMS mic data (sliding window) published.")
        else:
            print("Failed to publish RMS mic data (sliding window).")

    publish_current_rms()

    # Now continuously update: read one new frame, update window/sumsqs, publish
    while True:
        # Identify frame that will be evicted (if any)
        old_frame = window[0] if len(window) == WINDOW_SIZE else None

        new_frame = read_one_frame()

        # Push new frame into the window (this may evict the oldest)
        window.append(new_frame)

        # Update sumsqs: add new, subtract old (if evicted)
        for ch in range(4):
            vn = float(new_frame[ch])
            sumsqs[ch] += vn * vn
            if old_frame is not None:
                vo = float(old_frame[ch])
                sumsqs[ch] -= vo * vo

        publish_current_rms()

        if INTER_FRAME_SLEEP > 0:
            time.sleep(INTER_FRAME_SLEEP)

    # not reached
    # client.disconnect()


if __name__ == "__main__":
    main()