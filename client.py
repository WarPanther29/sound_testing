import time
import sys
import json
from smbus2 import SMBus, i2c_msg
import paho.mqtt.client as mqtt

# MQTT
MQTT_BROKER = "192.168.178.56"
MQTT_PORT = 1883
MQTT_TOPIC = "microphones/values"

# e-puck2
I2C_CHANNEL = 12
LEGACY_I2C_CHANNEL = 4
ROB_ADDR = 0x1F
ACTUATORS_SIZE = 20
SENSORS_SIZE = 47

# Initialize I2C
try:
    bus = SMBus(I2C_CHANNEL)
except:
    try:
        bus = SMBus(LEGACY_I2C_CHANNEL)
    except:
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
    except:
        return False

# Verify checksum
def verify_checksum():
    checksum = 0
    for i in range(SENSORS_SIZE - 1):
        checksum ^= sensors_data[i]
    return checksum == sensors_data[SENSORS_SIZE - 1]

# Read microphone data
def read_mic_values():
    mic = [0] * 4
    for i in range(4):
        mic[i] = sensors_data[32 + i*2 + 1] * 256 + sensors_data[32 + i*2]
    return mic

# Collect 100 samples per mic channel
def collect_mic_samples(sample_count=400):
    mic_data = [[] for _ in range(4)]

    print("Collecting mic samples...")
    start = time.time()
    while len(mic_data[0]) < sample_count:
        #start = time.time()

        if update_robot() and verify_checksum():
            mic = read_mic_values()
            for i in range(4):
                mic_data[i].append(mic[i])
        else:
            print("Invalid read, skipping.")

        # 
        #elapsed = time.time() - start
        #if elapsed < 0.01:
        #    time.sleep(0.01 - elapsed)
    print("Mic samples collected in {:.2f} seconds.".format(time.time() - start))
    return mic_data

# Main MQTT publisher
def main():
    client = mqtt.Client()
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    
    for _ in range(10):
        mic_data = collect_mic_samples()
        payload = json.dumps({
            "mic_0": mic_data[0],
            "mic_1": mic_data[1],
            "mic_2": mic_data[2],
            "mic_3": mic_data[3]
        })

        result = client.publish(MQTT_TOPIC, payload)
        if result[0] == 0:
            print("Mic data published successfully.")
        else:
            print("Failed to publish mic data.")

    client.disconnect()

if __name__ == "__main__":
    main()
