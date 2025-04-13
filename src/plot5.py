import serial
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use('TkAgg')

# ============ CONFIG SERIAL ============
SERIAL_PORT = 'COM6'
BAUD_RATE = 115200
TIMEOUT = 1

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    print(f"[INFO] Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"[ERROR] Could not open port {SERIAL_PORT}: {e}")
    exit(1)

# ============ BUFFER CONFIG ============
BUFFER_SIZE = 128
FFT_BINS = BUFFER_SIZE

time_data = deque(maxlen=BUFFER_SIZE)
adc_data = deque(maxlen=BUFFER_SIZE)
fft_vals = deque([0]*FFT_BINS, maxlen=FFT_BINS)
fft_freqs = deque([i for i in range(FFT_BINS)], maxlen=FFT_BINS)

# ============ PLOT SETUP ============
plt.ion()
fig = plt.figure(figsize=(12, 8))
fig.suptitle('ESP32 Live Data Visualization')

# --- ADC Time Domain Plot (Top) ---
ax_adc = plt.subplot2grid((2, 2), (0, 0), colspan=2)
line_adc, = ax_adc.plot([], [], label="ADC")
ax_adc.set_ylabel("ADC Value")
# ax_adc.set_xlabel("Sample Index")
ax_adc.set_ylim(-10, 10)
ax_adc.grid()

# --- General Information Display (Bottom Left) ---
ax_info = plt.subplot2grid((2, 2), (1, 0))
ax_info.axis('off')  # Hide axes
text_avg = ax_info.text(0.5, 0.7, '', fontsize=14, ha='center', va='center')
text_freq = ax_info.text(0.5, 0.5, '', fontsize=12, ha='center', va='center')
text_time = ax_info.text(0.5, 0.3, '', fontsize=10, ha='center', va='center')
ax_info.set_title("Sampling and Comunication Information")

# --- FFT Frequency Domain Plot (Bottom Right) ---
ax_fft = plt.subplot2grid((2, 2), (1, 1))
bars_fft = ax_fft.bar(fft_freqs, fft_vals, width=1)
ax_fft.set_ylabel("Magnitude")
ax_fft.set_xlabel("Frequency (Hz)")
ax_fft.set_ylim(0, 100)
ax_fft.set_xlim(0, 10)
ax_fft.grid()

# ============ PARSE SERIAL LINE ============
def parse_line(line):
    try:
        if line.startswith("DATA:"):
            parts = line.replace("DATA:", "").strip().split(',')
            if len(parts) == 2:
                t = int(parts[0]) / 1e6  # μs to seconds (optional)
                val = int(parts[1])
                time_data.append(t)
                adc_data.append(val)

        elif line.startswith("FFT:"):
            parts = line.replace("FFT:", "").strip().split(':')
            if len(parts) == 2:
                freq = float(parts[0])
                mag = float(parts[1])
                fft_freqs.append(freq)
                fft_vals.append(mag)

        elif line.startswith("MQTT:"):
            avg = float(line.replace("MQTT:", "").strip())
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            text_avg.set_text(f"5sec Window Average: {avg:.2f}")
            text_time.set_text(f"Last Updated: {timestamp}")
            fig.canvas.draw_idle()

        elif line.startswith("SAMPLE_FREQ:"):
            freq = float(line.replace("SAMPLE_FREQ:", "").strip())
            text_freq.set_text(f"Sampling Frequency: {freq:.2f} Hz")
            fig.canvas.draw_idle()

    except Exception as e:
        print(f"[ERROR] parse_line failed with line: {line} → {e}")

# ============ LOOP ============
try:
    while True:
        while ser.in_waiting:
            raw = ser.readline()
            line = raw.decode('utf-8', errors='ignore').strip()
            if line:
                parse_line(line)

        # === Update ADC plot ===
        if adc_data:
            line_adc.set_data(range(len(adc_data)), adc_data)
            ax_adc.set_xlim(0, len(adc_data))
            ax_adc.set_ylim(min(adc_data) - 10, max(adc_data) + 10)

        # === Update FFT bar plot ===
        if fft_freqs and fft_vals:
            bins = min(len(bars_fft), len(fft_vals))
            for bar, freq, mag in zip(bars_fft[:bins], list(fft_freqs)[-bins:], list(fft_vals)[-bins:]):
                bar.set_x(freq)
                bar.set_height(mag)
            ax_fft.set_xlim(min(fft_freqs), max(fft_freqs))
            ax_fft.set_ylim(0, max(fft_vals) + 10)

        plt.pause(0.001)

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
    plt.ioff()
    plt.close()