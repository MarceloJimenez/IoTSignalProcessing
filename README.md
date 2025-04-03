# 📡 IoT Adaptive Sampling System

**Author:** Marcelo Jiménez
**Platform:** ESP32 + FreeRTOS  
**Course:** IoT Performance Evaluation  
**Repository:** https://github.com/MarceloJimenez/IoTSignalProcessing

---

## 🧠 Project Overview

This project implements an **IoT system** in a ESP32 board (WROOM-32 Devkit)that:
- Samples a real analog signal 
- Analyzes it via FFT to detect the highest frequency,
- Adapts the sampling rate accordingly (Nyquist criterion),
- Computes the average value over a 5-second window,
- Sends this aggregate to:
  - A **local edge server** using MQTT over WiFi,
  - The **cloud** via **LoRaWAN + TTN** (NOT IMPLEMENTES YET)
- Evaluates energy savings, transmission volume, and system latency.

---

## ⚙️ System Architecture

```
  [DAC Signal Generator] -> [ADC Sampler - ESP32 (FreeRTOS)] -> [FFT Analysis]
                                                          ↘︎
                                                         [Adaptive Sampling]
                                                          ↘︎
                                                      [Aggregator (5s)]
                                                    ↙︎              ↘︎
                  [MQTT + WiFi to Edge Server]   [LoRaWAN + TTN to Cloud]
```

---

## 📡 Input Signal

The input is a synthetic signal:


Generated using DAC on ESP32 (see `Sampling1.ino`).

---

## 🚀 How to Run

### ✅ Requirements
- ESP32 board (WROOM-32 Devkit)
- Python + `matplotlib`, `numpy`, `scipy`, `pyserial`
- Arduino + `WiFi.h` ,`PubSubClient.h`,`arduinoFFT.h`


### ✅ Setup
0. Flash `Max_Freq.ino` to measure the maximun sampling frequency of ESP32: **20757.91 Hz**

1. Flash `Sampling1.ino` to ESP32 (signal generator + ADC)\
2. Open the serial port to see the messages of the implementation. 
3. Execute `plot2.py` to see the received data and the FFT. (Check first if the code in the line 95/97 of `Sampling1.ino` are not comented )

---

## 🔐 Measurements of the performance

- **Energy consumption**

Here its a diagram of the qualitative energy consumption of the system: 
![EnergyDiagram](ruta/a/la/imagen.png)

- **Volume of data**

The maximun sampling frequency of the system is aproximately 20.000 Hz. The frequency that we are using its 120 Hz

So we have a reduction factor of approximately 166.67 of the volume of data procesed by the IoT system.


---

## 📁 Folder Structure

```
├── Sampling1.ino            # DAC + ADC signal
├── Max_Freq.ino             # Max sampling test
├── README.md
└── doc/                     # Energy measurements, signal plots, etc.
```

---

## 📣 Notes

This project was developed as part of the IoT Performance Evaluation course.  
All work (code and documentation) is individual and original, following the academic integrity policy.

