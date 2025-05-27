from smbus2 import SMBus, i2c_msg
import sys
import time

I2C_CHANNEL = 12
LEGACY_I2C_CHANNEL = 4
ROB_ADDR = 0x1F
ACTUATORS_SIZE = 20  # Data + checksum
SENSORS_SIZE = 47    # Data + checksum

actuators_data = bytearray([0] * ACTUATORS_SIZE)
sensors_data = bytearray([0] * SENSORS_SIZE)
mic = [0 for _ in range(4)]

def update_robot():
    global sensors_data

    # Update checksum for actuator data (even if it's all zero)
    checksum = 0
    for i in range(ACTUATORS_SIZE - 1):
        checksum ^= actuators_data[i]
    actuators_data[-1] = checksum

    try:
        write = i2c_msg.write(ROB_ADDR, actuators_data)
        read = i2c_msg.read(ROB_ADDR, SENSORS_SIZE)
        bus.i2c_rdwr(write, read)
        sensors_data = list(read)
        return True
    except Exception as e:
        print(f"I2C communication error: {e}")
        return False

def verify_checksum():
    checksum = 0
    for i in range(SENSORS_SIZE - 1):
        checksum ^= sensors_data[i]
    return checksum == sensors_data[SENSORS_SIZE - 1]

def read_mic_values():
    for i in range(4):
        mic[i] = sensors_data[32 + i*2 + 1] * 256 + sensors_data[32 + i*2]
    return mic

# Try I2C bus
try:
    bus = SMBus(I2C_CHANNEL)
except:
    try:
        bus = SMBus(LEGACY_I2C_CHANNEL)
    except:
        print("Cannot open I2C device")
        sys.exit(1)

# Continuous read loop
while True:
    start = time.time()

    if update_robot() and verify_checksum():
        mic_vals = read_mic_values()
        print("Mic values: {:4d}, {:4d}, {:4d}, {:4d}".format(*mic_vals))
    else:
        print("Checksum error or communication failure")

    # Run at ~20 Hz
    elapsed = time.time() - start
    if elapsed < 0.05:
        time.sleep(0.05 - elapsed)
