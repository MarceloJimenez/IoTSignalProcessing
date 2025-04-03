import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import fft

# ==== CONFIGURA ESTO =====
SERIAL_PORT = 'COM6'     # Cambia esto según tu sistema (por ejemplo, '/dev/ttyUSB0' en Linux/Mac)
BAUD_RATE = 115200
NUM_SAMPLES = 128        # Debe coincidir con TOTAL_SAMPLES en el ESP32

# =========================

# Abrir puerto serie
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
print(f"Leyendo {NUM_SAMPLES} muestras desde {SERIAL_PORT}...")

timestamps = []
adc_values = []

while len(adc_values) < NUM_SAMPLES:
    print(f" {len(adc_values)} muestras.")
    line = ser.readline().decode().strip()
    print(line)
    if "," in line:
        try:
            t, val = line.split(",")
            # print(f"t: {t}, val: {val}")
            print(type(t), type(val))
            timestamps.append(int(t))
            adc_values.append(int(val))
            print(len(adc_values))
        except ValueError as e:
            print(f"Error al procesar la línea: {line}. Error: {e}")
print(f"Leídas {len(adc_values)} muestras.")
ser.close()

# Convertir a arrays de numpy
timestamps = np.array(timestamps)
adc_values = np.array(adc_values)

# Convertir a tiempo en segundos
time_sec = (timestamps - timestamps[0]) / 1e6

# === PLOTEO DE LA SEÑAL EN EL TIEMPO ===
plt.figure()
plt.plot(time_sec, adc_values, marker='o')
plt.title("Señal ADC en el tiempo")
plt.xlabel("Tiempo (s)")
plt.ylabel("ADC Value")
plt.grid(True)
plt.tight_layout()

# === FFT ===
n = len(adc_values)
dt = np.mean(np.diff(time_sec))  # Paso de tiempo medio
fs = 1.0 / dt                    # Frecuencia de muestreo estimada
print(f"Frecuencia de muestreo: {fs:.2f} Hz")
print('Parametros de la FFT:')
print(f"  n: {n}")
print(f"  dt: {dt:.6f} s")
print(f"  fs: {fs:.2f} Hz")
print(f"  Frecuencia de Nyquist: {fs/2:.2f} Hz")
print(adc_values.min(), adc_values.max())
yf = fft(adc_values)
xf = np.fft.fftfreq(n, d=dt)

# Solo parte positiva del espectro
idx = np.where(xf >= 0)
xf = xf[idx]
yf = 2.0/n * np.abs(yf[idx])

plt.figure()
plt.plot(xf, yf)
plt.title("Espectro FFT")
plt.xlabel("Frecuencia (Hz)")
plt.ylabel("Magnitud")
plt.grid(True)
plt.tight_layout()

plt.show()
