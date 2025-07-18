import numpy as np
from smbus2 import SMBus, i2c_msg
import sys
import time
import matplotlib.pyplot as plt
from scipy import fftpack
import os

I2C_CHANNEL = 12
LEGACY_I2C_CHANNEL = 4
ROB_ADDR = 0x1F
ACTUATORS_SIZE = 20  # Data + checksum
SENSORS_SIZE = 47    # Data + checksum

actuators_data = bytearray([0] * ACTUATORS_SIZE)
sensors_data = bytearray([0] * SENSORS_SIZE)
mic = [0 for _ in range(4)]
mic_0 = []
mic_1 = []
mic_2 = []
mic_3 = []


def update_robot():
    global sensors_data

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


try:
    bus = SMBus(I2C_CHANNEL)
except:
    try:
        bus = SMBus(LEGACY_I2C_CHANNEL)
    except:
        print("Cannot open I2C device")
        sys.exit(1)

# Create output folder if not existing
os.makedirs("plots", exist_ok=True)
plot_counter = 0  # You can increment this with each loop to save multiple files

def calculate_lag(x, y):
    corr = np.correlate(x - np.mean(x), y - np.mean(y), mode='full')
    lag = np.argmax(corr) - (len(x) - 1)
    return lag, corr


SOUND_SPEED = 343.0  # Speed of sound in m/s
while True:
    mic_0.clear()
    mic_1.clear()
    mic_2.clear()
    mic_3.clear()
    
    start = time.time()
    for _ in range(10):
        if update_robot() and verify_checksum():
            mic_vals = read_mic_values()
            mic_0.append(mic_vals[0])
            mic_1.append(mic_vals[1])
            mic_2.append(mic_vals[2])
            mic_3.append(mic_vals[3])
        else:
            print("Checksum error or communication failure")
    end = time.time()
    
    sample_rate = 10 / (end - start)
    t = np.arange(10) / sample_rate

    # FFT to find dominant frequency
    #fft_result = fft(mic_0)
    #freqs = fftfreq(len(mic_0), d=1/sample_rate)
    #idx = np.argmax(np.abs(fft_result[:len(freqs)//2]))
    #dominant_freq = freqs[idx]

    fft_result = fftpack.fft(mic_0)
    freqs = fftpack.fftfreq(len(mic_0), d=1/sample_rate)
    positive_freqs = freqs[:len(freqs)//2]
    positive_fft = fft_result[:len(freqs)//2]
    idx = np.argmax(np.abs(positive_fft))
    dominant_freq = positive_freqs[idx]

    # Cross-correlation lag calculation
    lag01, corr01 = calculate_lag(np.array(mic_0), np.array(mic_1))
    lag02, corr02 = calculate_lag(np.array(mic_0), np.array(mic_2))

    dt_01 = lag01 / sample_rate
    dt_02 = lag02 / sample_rate
    dx = dt_01 * SOUND_SPEED
    dy = dt_02 * SOUND_SPEED

    angle_rad = np.arctan2(dy, dx)
    angle_deg = np.degrees(angle_rad) % 360

    # Plot
    fig, axs = plt.subplots(2, 1, figsize=(10, 6))

    axs[0].plot(t, mic_0, label='Mic 0')
    axs[0].plot(t, mic_1, label='Mic 1')
    axs[0].plot(t, mic_2, label='Mic 2')
    axs[0].plot(t, mic_3, label='Mic 3')
    axs[0].set_title("Microphone Values")
    axs[0].set_xlabel("Time [s]")
    axs[0].set_ylabel("Amplitude")
    axs[0].legend()

    axs[1].bar(['Lag01', 'Lag02', 'Angle (deg)', 'Freq (Hz)'],
               [lag01, lag02, angle_deg, dominant_freq])
    axs[1].set_title("Lags, Angle, and Dominant Frequency")

    plt.tight_layout()
    plot_filename = f"plots/snapshot_{plot_counter:03d}.svg"
    plt.savefig(plot_filename, format='svg')
    plt.close(fig)

    print(f"Saved plot: {plot_filename}")
    plot_counter += 1  