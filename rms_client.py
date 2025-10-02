
import time
import sys
import json
from smbus2 import SMBus, i2c_msg
import paho.mqtt.client as mqtt
import socket
import math

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

# Sampling
SAMPLES_PER_BATCH = 10        # number of frames to collect per publish
INTER_SAMPLE_SLEEP = 0.0      # seconds between frames; set >0.0 if you need pacing

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


def collect_batch(n=SAMPLES_PER_BATCH, inter_sleep=INTER_SAMPLE_SLEEP):
    """Collect n successive frames of 4-channel mic data."""
    frames = []
    for _ in range(n):
        frame = read_one_frame()
        frames.append(frame)
        if inter_sleep > 0:
            time.sleep(inter_sleep)
    return frames


def rms_per_channel(frames):
    """Compute RMS for each of 4 channels across the collected frames.

    frames: list of [m0, m1, m2, m3] values (length == SAMPLES_PER_BATCH)
    returns: list [rms0, rms1, rms2, rms3] (floats)
    """
    # Accumulate sum of squares per channel
    sumsqs = [0.0, 0.0, 0.0, 0.0]
    count = len(frames)
    if count == 0:
        return [0.0, 0.0, 0.0, 0.0]

    for frame in frames:
        for ch in range(4):
            v = float(frame[ch])
            sumsqs[ch] += v * v

    return [math.sqrt(s / count) for s in sumsqs]


def main():
    client = mqtt.Client()
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    hostname = socket.gethostname()
    number = ''.join(filter(str.isdigit, hostname[len('pi-puck'):]))
    MQTT_TOPIC = f"robots/{number}/mics"

    while True:
        frames = collect_batch()
        rms_vals = rms_per_channel(frames)

        # Keep the MQTT "signature" (keys) the same: m0..m3; values now are RMS per mic
        payload = json.dumps({f"m{i}": rms_vals[i] for i in range(4)})

        result = client.publish(MQTT_TOPIC, payload)
        if result[0] == 0:
            print("RMS mic data published successfully.")
        else:
            print("Failed to publish RMS mic data.")

    # not reached
    # client.disconnect()


if __name__ == "__main__":
    main()