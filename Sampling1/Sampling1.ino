/*
 * ESP32 Dual-Core Signal Processing System
 * - Core 0: DAC Signal Generation + Data Processing
 * - Core 1: High-Precision ADC Sampling
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <arduinoFFT.h>

// ======================
// CONFIGURATION CONSTANTS
// ======================
#define FFT_SAMPLE_SIZE 128
#define QUEUE_LENGTH (FFT_SAMPLE_SIZE * 4)  // Larger queue for continuous  buffering
#define ADC_PIN              34  // GPIO34 = ADC1_CH6
#define DAC_PIN              25  // GPIO25 = DAC1

const int amplitude         = 100;
const int offset            = 128;
const float frequency       = 40.0;
const int dacUpdateRate     = 1000;
volatile float sampleFrequency = 20000.0;  


// ======================
// RTOS GLOBAL VARIABLES
// ======================
QueueHandle_t sampleQueue;
QueueHandle_t aggQueue;
TaskHandle_t ADC_TaskHandle     = NULL;
TaskHandle_t Process_TaskHandle = NULL;

typedef struct {
    int adc_value;
    unsigned long delta_time;
} ADCData_t;

// ======================
// MQTT Config
// ======================
volatile bool mqtt_connected = false;  // flag to track MQTT status

const char* ssid     = "FRITZ!Box 7530 LP";
const char* password = "70403295595551907386";

// MQTT broker (e.g., Raspberry Pi, Mosquitto server, etc.)
const char* mqtt_server = "192.168.178.50";
const int mqtt_port = 1883;
const char* mqtt_topic = "iot/aggregate";

WiFiClient espClient;
PubSubClient client(espClient);


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
            int sineValue = (int)(amplitude * sin(phase) + offset);
            dacWrite(DAC_PIN, sineValue);
            phase += phaseIncrement;
            if (phase >= 2 * PI) phase -= 2 * PI;
            lastUpdate = now;
        }
        vTaskDelay(1);
    }
}

// ======================
// ADC SAMPLING TASK (Core 1)
// ======================
void TaskADCRead(void *parameter) {
    ADCData_t adc_data;
    

    unsigned long t_prev = micros();

    while (1) {
      unsigned long samplePeriod = 1000000UL / sampleFrequency;
        while ((micros() - t_prev) < samplePeriod) {
            vTaskDelay(1);
        }

        unsigned long t_now = micros();
        adc_data.adc_value = analogRead(ADC_PIN);

        // To use plot2.py uncomment this:
        // Serial.print(micros());
        // Serial.print(",");
        // Serial.println(adc_data.adc_value);



        adc_data.delta_time = t_now - t_prev;

        // Send data to queue (drop if full to avoid blocking)
        xQueueSend(sampleQueue, &adc_data, 0);
        xQueueSend(aggQueue, &adc_data, 0);
        

        t_prev = t_now;
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
    float mean_sampling_freq;

    while(true){

    float sum = 0.0;

    for (int i = 0; i < FFT_SAMPLE_SIZE; i++) {
        if (xQueueReceive(sampleQueue, &data, portMAX_DELAY) == pdTRUE) {
            freqs[i] = 1e6 / data.delta_time;
            vReal[i] = data.adc_value;
            vImag[i] = 0.0;
            sum += freqs[i];
        } else {
            Serial.println("Process Error: Queue broken!");
            break;
        }
    }

    float mean_sampling_freq = sum / FFT_SAMPLE_SIZE;

    // Serial.println("\n=== ANALYSIS RESULTS ===");
    // Serial.printf("Average Frequency: %.2f Hz\n", mean_sampling_freq);
    // Serial.println("========================");

    // Serial.println("\n=== FFT PARAMETERS ===");
    // Serial.printf("  n: %d\n", FFT_SAMPLE_SIZE);
    // Serial.printf("  dt: %.6f s\n", 1.0 / mean_sampling_freq);
    // Serial.printf("  fs: %.2f Hz\n", mean_sampling_freq);
    // Serial.printf("  Nyquist Frequency: %.2f Hz\n", mean_sampling_freq / 2.0);

    int min_adc = 1024;
    int max_adc = 0;

    for (int i = 0; i < FFT_SAMPLE_SIZE; i++) {
        if (vReal[i] < min_adc) min_adc = vReal[i];
        if (vReal[i] > max_adc) max_adc = vReal[i];
    }

    // Serial.printf("  ADC Min: %d, Max: %d\n", min_adc, max_adc);

    ArduinoFFT<float> FFT(vReal, vImag, FFT_SAMPLE_SIZE, mean_sampling_freq, false);

    FFT.windowing(vReal, FFT_SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, FFT_SAMPLE_SIZE, FFT_FORWARD);

    for (int i = 0; i < 20; i++) {
        vReal[i] = 0.0;
    }

    FFT.complexToMagnitude(vReal, vImag, FFT_SAMPLE_SIZE);

        float peakFrequency = FFT.majorPeak(vReal, FFT_SAMPLE_SIZE, mean_sampling_freq);

        float binWidth = mean_sampling_freq / FFT_SAMPLE_SIZE;
        float maxFreq = 0.0;
        int N = FFT_SAMPLE_SIZE / 2;  // only look at first half (positive frequencies)

        // Step 1: compute mean magnitude
        float sum2 = 0.0;
        for (int i = 1; i < N; i++) {
            sum2 += vReal[i];
        }
        float mean = sum2 / (N - 1);

        // Step 2: compute standard deviation
        float sq_diff = 0.0;
        for (int i = 1; i < N; i++) {
            float diff = vReal[i] - mean;
            sq_diff += diff * diff;
        }
        float stddev = sqrt(sq_diff / (N - 1));

        // Step 3: define a dynamic threshold
        float threshold = mean + 2 * stddev;  // adjust this factor (2) to be more/less strict

        // Step 4: find the highest frequency with magnitude above threshold
        for (int i = 1; i < N; i++) {
            if (vReal[i] > threshold) {
                maxFreq = i * binWidth;
            }
        }


    // Serial.println("\n=== FFT RESULTS ===");
    // Serial.printf("Dominant Frequency: %.2f Hz\n", peakFrequency);
    // Serial.println("Frequency (Hz)\tMagnitude");
    // Serial.println("------------------------");

    if (maxFreq > 1.0 && maxFreq <= 10000.0) {  // Only update if it's a valid frequency (not noise or aliasing)
    sampleFrequency = 3 * maxFreq;  // Set new sample rate as 3x the max frequency to satisfy Nyquist with a safety margin
    Serial.printf("[FTT] sampleFrequency set to %.2f Hz (max freq: %.2f Hz)\n", 
                  sampleFrequency, maxFreq);  
    // Print the result
}

}

    
    }



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

            if (elapsed_time >= 5000000UL) { // 5 seconds
                float average = (count > 0) ? (float)sum / count : 0.0;
                Serial.printf("[Agg] Average over 5s: %.2f (%d samples)\n", average, count);

                 // Publish to MQTT
                if (mqtt_connected) {
                    char payload[32];
                    snprintf(payload, sizeof(payload), "%.2f", average);

                    // Try to publish and check result
                    if (client.publish(mqtt_topic, payload)) {
                        Serial.printf("[MQTT] Sent average: %.2f\n", average);
                    } else {
                        Serial.println("[MQTT] Failed to send data!");
                    }
                } else {
                    Serial.println("[MQTT] Not connected, can't send data.");
                }

                // Reset for next window
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

    connectToWiFi();

    client.setServer(mqtt_server, mqtt_port);
    connectToMQTT();  // only once

    mqtt_connected = client.connected();

    dacWrite(DAC_PIN, offset);
    analogReadResolution(8);
    analogSetAttenuation(ADC_11db);

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

    xTaskCreatePinnedToCore(
        TaskDACWrite,
        "DAC_Gen",
        4096,
        NULL,
        1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        TaskADCRead,
        "ADC_Sample",
        4096,
        NULL,
        2,
        &ADC_TaskHandle,
        1
    );

    xTaskCreatePinnedToCore(
        TaskProcess,
        "Data_Process",
        8192,
        NULL,
        1,
        &Process_TaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
    TaskAggregation,
    "RollingAverage",
    4096,
    NULL,
    1,
    NULL,
    0
);

    Serial.println("System Initialized: Tasks Running");
}

// ======================
// MAIN LOOP (Unused)
// ======================
void loop() {
    vTaskDelete(NULL);
    if (!client.connected()) {
        connectToMQTT();
        mqtt_connected = client.connected();
    }

    client.loop();  // handles keepalive and pings
    delay(10);
}


void connectToWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void connectToMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected.");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
        }
    }
}