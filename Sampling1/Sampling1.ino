/*
 * ESP32 Dual-Core Signal Processing System
 * - Core 0: DAC Signal Generation + Data Processing
 * - Core 1: High-Precision ADC Sampling
 */

#include <WiFi.h>               // WiFi library for ESP32
#include <PubSubClient.h>       // MQTT client library
#include <arduinoFFT.h>         // FFT processing library

// ======================
// CONFIGURATION CONSTANTS
// ======================
#define FFT_SAMPLE_SIZE 128                         // FFT window size
#define QUEUE_LENGTH (FFT_SAMPLE_SIZE * 4)          // Size of FreeRTOS queues
#define ADC_PIN 34                                  // GPIO34 used for ADC input
#define DAC_PIN 25                                  // GPIO25 used for DAC output

const int amplitude = 100;                          // Amplitude of sine wave
const int offset = 128;                             // DC offset for sine wave
const float frequency = 40.0;                       // Base frequency for DAC signal
const int dacUpdateRate = 1000;                     // DAC update rate in Hz
volatile float sampleFrequency = 20000.0;           // Initial ADC sampling frequency

// ======================
// RTOS GLOBAL VARIABLES
// ======================
QueueHandle_t sampleQueue;                          // Queue for FFT samples
QueueHandle_t aggQueue;                             // Queue for averaging
TaskHandle_t ADC_TaskHandle = NULL;                 // Handle for ADC task
TaskHandle_t Process_TaskHandle = NULL;             // Handle for processing task

// Struct for ADC data and its timing
typedef struct {
    int adc_value;
    unsigned long delta_time;
} ADCData_t;

// ======================
// MQTT CONFIGURATION
// ======================
volatile bool mqtt_connected = false;               // MQTT connection status flag

const char* ssid = "iPhone de Edgar";             // WiFi SSID
const char* password = "01234567";      // WiFi password

const char* mqtt_server = "broker.hivemq.com";         // MQTT broker IP
const int mqtt_port = 1883;                         // MQTT port
const char* mqtt_topic = "iot/aggregate";           // MQTT topic for publishing

WiFiClient espClient;                               // WiFi client for MQTT
PubSubClient client(espClient);                     // MQTT client object

// ======================
// DAC SIGNAL GENERATOR (Core 0)
// ======================
void TaskDACWrite(void *pvParameters) {
    float phase = 0;
    const float phaseIncrement = 2 * PI * frequency / dacUpdateRate;
    unsigned long lastUpdate = micros();

    while (1) {
        unsigned long now = micros();
        if (now - lastUpdate >= (1000000UL / dacUpdateRate)) {
            int sineValue = (int)(amplitude * sin(phase) + offset); // Generate sine wave value
            dacWrite(DAC_PIN, sineValue);                            // Output to DAC
            phase += phaseIncrement;
            if (phase >= 2 * PI) phase -= 2 * PI;                    // Keep phase within 0 to 2π
            lastUpdate = now;
        }
        vTaskDelay(1);  // Yield to other tasks
    }
}

// ======================
// ADC SAMPLING TASK (Core 1)
// ======================
void TaskADCRead(void *parameter) {
    ADCData_t adc_data;
    unsigned long t_prev = micros();  // Initial timestamp

    while (1) {
        unsigned long samplePeriod = 1000000UL / sampleFrequency;
        while ((micros() - t_prev) < samplePeriod) {
            vTaskDelay(1);  // Wait for correct sampling period
        }

        unsigned long t_now = micros();  // Capture time
        adc_data.adc_value = analogRead(ADC_PIN);  // Read ADC value
        adc_data.delta_time = t_now - t_prev;      // Time since last sample

        // Send sample to both queues (non-blocking)
        xQueueSend(sampleQueue, &adc_data, 0);
        xQueueSend(aggQueue, &adc_data, 0);

        //To use plot2.py uncomment this:
        Serial.print(micros());
        Serial.print(",");
        Serial.println(adc_data.adc_value);

        t_prev = t_now;  // Update timestamp
    }
}

// ======================
// DATA PROCESSING TASK (Core 0)
// ======================
void TaskProcess(void *pvParameters) {
    ADCData_t data;
    float vReal[FFT_SAMPLE_SIZE];
    float vImag[FFT_SAMPLE_SIZE];
    float freqs[FFT_SAMPLE_SIZE];

    while (true) {
        float sum = 0.0;

        // Receive FFT_SAMPLE_SIZE samples
        for (int i = 0; i < FFT_SAMPLE_SIZE; i++) {
            if (xQueueReceive(sampleQueue, &data, portMAX_DELAY) == pdTRUE) {
                freqs[i] = 1e6 / data.delta_time;  // Calculate sampling frequency from delta time
                vReal[i] = data.adc_value;         // Real part of FFT input
                vImag[i] = 0.0;                    // Imaginary part initialized to 0
                sum += freqs[i];                   // Accumulate sampling frequencies
            } else {
                Serial.println("Process Error: Queue broken!");
                break;
            }
        }

        float mean_sampling_freq = sum / FFT_SAMPLE_SIZE;

        // Track ADC min and max values
        int min_adc = 1024, max_adc = 0;
        for (int i = 0; i < FFT_SAMPLE_SIZE; i++) {
            if (vReal[i] < min_adc) min_adc = vReal[i];
            if (vReal[i] > max_adc) max_adc = vReal[i];
        }

        ArduinoFFT<float> FFT(vReal, vImag, FFT_SAMPLE_SIZE, mean_sampling_freq, false);
        FFT.windowing(vReal, FFT_SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Apply window
        FFT.compute(vReal, vImag, FFT_SAMPLE_SIZE, FFT_FORWARD);                 // Compute FFT

        for (int i = 0; i < 20; i++) vReal[i] = 0.0; // Zero out low-frequency bins

        FFT.complexToMagnitude(vReal, vImag, FFT_SAMPLE_SIZE); // Convert to magnitude

        float peakFrequency = FFT.majorPeak(vReal, FFT_SAMPLE_SIZE, mean_sampling_freq);

        float binWidth = mean_sampling_freq / FFT_SAMPLE_SIZE;
        float maxFreq = 0.0;
        int N = FFT_SAMPLE_SIZE / 2;

        float sum2 = 0.0;
        for (int i = 1; i < N; i++) sum2 += vReal[i];
        float mean = sum2 / (N - 1);

        float sq_diff = 0.0;
        for (int i = 1; i < N; i++) {
            float diff = vReal[i] - mean;
            sq_diff += diff * diff;
        }
        float stddev = sqrt(sq_diff / (N - 1));

        float threshold = mean + 2 * stddev;

        for (int i = 1; i < N; i++) {
            if (vReal[i] > threshold) {
                maxFreq = i * binWidth;
            }
        }

        if (maxFreq > 1.0 && maxFreq <= 10000.0) {
            sampleFrequency = 3 * maxFreq;  // Update sample rate to 3x detected frequency
            Serial.printf("[FTT] sampleFrequency set to %.2f Hz (max freq: %.2f Hz)\n",
                          sampleFrequency, maxFreq);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1000 milliseconds
    }
}

// ======================
// AGGREGATION + MQTT PUBLISH TASK (Core 0)
// ======================
void TaskAggregation(void *param) {
    unsigned long start_time = micros();
    unsigned long elapsed_time = 0;
    long sum = 0;
    int count = 0;
    ADCData_t data;

    while (1) {
        if (xQueueReceive(aggQueue, &data, portMAX_DELAY) == pdTRUE) {
            sum += data.adc_value;
            count++;
            elapsed_time = micros() - start_time;

            if (elapsed_time >= 5000000UL) {  // Every 5 seconds
                float average = (count > 0) ? (float)sum / count : 0.0;
                Serial.printf("[Agg] Average over 5s: %.2f (%d samples)\n", average, count);

                if (mqtt_connected) {
                    char payload[32];
                    snprintf(payload, sizeof(payload), "%.2f", average);

                    if (client.publish(mqtt_topic, payload)) {
                        Serial.printf("[MQTT] Sent average: %.2f\n", average);
                    } else {
                        Serial.println("[MQTT] Failed to send data!");
                    }
                } else {
                    Serial.println("[MQTT] Not connected, can't send data.");
                }

                // Reset counters
                start_time = micros();
                sum = 0;
                count = 0;
            }
        }
    }
}

// ======================
// INITIALIZATION
// ======================
void setup() {
    Serial.begin(115200);
    delay(1000);

    connectToWiFi();                       // Connect to WiFi
    client.setServer(mqtt_server, mqtt_port);
    connectToMQTT();                       // Connect to MQTT broker
    mqtt_connected = client.connected();   // Set connection flag

    dacWrite(DAC_PIN, offset);             // Initialize DAC output
    analogReadResolution(8);               // 8-bit ADC resolution
    analogSetAttenuation(ADC_11db);        // ADC attenuation setting

    sampleQueue = xQueueCreate(QUEUE_LENGTH, sizeof(ADCData_t));
    if (sampleQueue == NULL) {
        Serial.println("FATAL: Queue creation failed!");
        while (1);
    }

    aggQueue = xQueueCreate(QUEUE_LENGTH, sizeof(ADCData_t));
    if (aggQueue == NULL) {
        Serial.println("FATAL: Queue creation failed!");
        while (1);
    }

    // Create tasks and assign them to specific cores
    xTaskCreatePinnedToCore(TaskDACWrite, "DAC_Gen", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskADCRead, "ADC_Sample", 4096, NULL, 2, &ADC_TaskHandle, 1);
    xTaskCreatePinnedToCore(TaskProcess, "Data_Process", 8192, NULL, 1, &Process_TaskHandle, 0);
    xTaskCreatePinnedToCore(TaskAggregation, "RollingAverage", 4096, NULL, 1, NULL, 0);

    Serial.println("System Initialized: Tasks Running");
}

// ======================
// MAIN LOOP (Unused)
// ======================
void loop() {
    vTaskDelete(NULL);  // Prevent accidental use
    if (!client.connected()) {
        connectToMQTT();                     // Reconnect if MQTT is disconnected
        mqtt_connected = client.connected();
    }
    client.loop();                           // Maintain MQTT connection
    delay(10);                               // Short delay
}


// ======================
// WIFI CONNECTION HELPER
// ======================
void connectToWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 2) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi after 2 attempts.");
    }
}

// ======================
// MQTT CONNECTION HELPER
// ======================
void connectToMQTT() {
    int attempts = 0;
    while (!client.connected() && attempts < 2) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected.");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
            attempts++;
        }
    }

    if (!client.connected()) {
        Serial.println("Failed to connect to MQTT after 2 attempts.");
    }
}

