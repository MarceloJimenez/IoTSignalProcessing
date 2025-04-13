/*
 * ESP32 Dual-Core Signal Processing System
 * - Core 0: DAC Signal Generation + Data Processing
 * - Core 1: High-Precision ADC Sampling
 * 
 * To connect to WiFi and MQTT broker, follow these steps:  
 * 1. Set up your WiFi credentials (SSID and password) in the code.
 *  -   Both your PC and ESP32 should be connected to the same WiFi network.
 * 2. Set up your MQTT broker address, config and port. 
 *  - C:\Program Files\mosquitto\mosquitto.conf should have the following uncomented lines:
 *      - listener 1883
 *      - allow_anonymous true
 *  - mqtt_server = Use Win + r, open command prompt using cmd, and type ipconfig to find your local IP address 192.168.XXX.XX
 *  - mqtt_port = 1883; This is default port for MQTT. You could change it in the file mosquitto.conf
 *  - mqtt_topic = This is the topic for publishing data. Can be any name you want.
 * 3. To start the MQTT broker open a terminal and paste: mosquitto -c "C:\Program Files\mosquitto\mosquitto.conf" -v .
 *  - netstat -ano | findstr :1883  to show the port is open 
 * 

 * 4. Open another terminal and paste: mosquitto_sub -h <mqtt_server> -t "<mqtt_topic>" -v to subscribe to the topic.
 * 
 * 5. Upload the code to your ESP32 board.
 * 6. Open the Serial Monitor in Arduino IDE to see the output.
 * 7. You can use a the python script to plot the data received from the ESP32.
 */

#include <WiFi.h>          // WiFi library for ESP32
#include <PubSubClient.h>  // MQTT client library
#include <arduinoFFT.h>    // FFT processing library

// ======================
// CONFIGURATION CONSTANTS
// ======================
#define FFT_SAMPLE_SIZE 128                 // FFT window size
#define QUEUE_LENGTH (FFT_SAMPLE_SIZE * 4)  // Size of FreeRTOS queues
#define ADC_PIN 34                          // GPIO34 used for ADC input
#define DAC_PIN 25                          // GPIO25 used for DAC output
#define SINE_DEBUG false
const bool toPlot = true;          //Flag to use Plot2 python code
const bool toPlot_fft = true;       //Flag to use Plot2 python code
const int offset = 128;             // DC offset for sine wave
const int amplitude = 30;           // Amplitude of sine wave1
const float frequency = 4.0;        // Base frequency for DAC signal  1
const int amplitude2 = 60;          // Amplitude of sine wave 2
const float frequency2 = 10.0;       // Base frequency for DAC signal 2
const int dacUpdateRate = 500;      // DAC update rate in Hz
volatile float sampleFrequency = 50.0;  // Initial maximun ADC sampling frequency

// ======================
// RTOS GLOBAL VARIABLES
// ======================
QueueHandle_t sampleQueue;               // Queue for FFT samples
QueueHandle_t aggQueue;                  // Queue for averaging
TaskHandle_t ADC_TaskHandle = NULL;      // Handle for ADC task
TaskHandle_t Process_TaskHandle = NULL;  // Handle for processing task

// Struct for ADC data and its timing
typedef struct {
  int adc_value;
  unsigned long delta_time;
} ADCData_t;


// MQTT CONFIGURATION

volatile bool mqtt_connected = false;  // MQTT connection status flag
const char* ssid = "S23 de Marcelo";
const char* password = "12345678";
const char* mqtt_server = "192.168.125.32";
const int mqtt_port = 1883;                // MQTT port
const char* mqtt_topic = "iot-homework";  // MQTT topic for publishing

WiFiClient espClient;            // WiFi client for MQTT
PubSubClient client(espClient);  // MQTT client object

portMUX_TYPE samplingMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t serialMutex;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ======================
// DAC SIGNAL GENERATOR (Core 0)
// ======================
void TaskDACWrite(void* pvParameters) {
  float phase = 0;
  float phase2 = 4 * PI / 3;  // Phase offset for second sine wave
  const float phaseIncrement = 2 * PI * frequency / dacUpdateRate;
  const float phaseIncrement2 = 2 * PI * frequency2 / dacUpdateRate;
  unsigned long lastUpdate = micros();

  while (1) {
    unsigned long now = micros();
    if (now - lastUpdate >= (1000000UL / dacUpdateRate)) {
      int sineValue = (int)(amplitude * sin(phase) + offset);  // Generate sine wave value
      int sineValue2 = (int)(amplitude * sin(phase2));         // Generate sine wave value
      int sineSum = sineValue + sineValue2;
      dacWrite(DAC_PIN, sineSum);  // Output to DAC
      phase += phaseIncrement;
      if (phase >= 2 * PI) phase -= 2 * PI;  // Keep phase within 0 to 2π
      phase2 += phaseIncrement2;
      if (phase2 >= 2 * PI) phase2 -= 2 * PI;  // Keep phase within 0 to 2π
      lastUpdate = now;

      // ------ PLOT SIGNAL TO DEBUG (SHOULD BE COMMENTED)--------
      if (SINE_DEBUG == true) {  //Using dashboard
        Serial.printf("DATA:%lu,%d\n", micros(), sineSum);
      }
    }
    vTaskDelay(1);  // Yield to other tasks
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ======================
// ADC SAMPLING TASK (Core 1)
// ======================
void TaskADCRead(void* parameter) {
  ADCData_t adc_data;
  unsigned long t_prev = micros();  // Initial timestamp
  portENTER_CRITICAL(&samplingMux);
  unsigned long samplePeriod = 1000000UL / sampleFrequency;
  portEXIT_CRITICAL(&samplingMux);
  unsigned long lastSampleTime = micros();
  while (1) {
    if ((micros() - lastSampleTime) >= samplePeriod) {
      lastSampleTime += samplePeriod;
      unsigned long t_now = micros();            // Capture time
      adc_data.adc_value = analogRead(ADC_PIN);  // Read ADC value
      adc_data.delta_time = t_now - t_prev;      // Time since last sample

      // Send sample to both queues (non-blocking)
      xQueueSend(sampleQueue, &adc_data, 0);
      xQueueSend(aggQueue, &adc_data, 0);

      if (toPlot == true) {  //Using dashboard
        Serial.printf("DATA:%lu,%d\n", micros(), adc_data.adc_value);
      }
      t_prev = t_now;  // Update timestamp
    }
    vTaskDelay(1);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------


// ======================
// DATA PROCESSING TASK (Core 0)
// ======================
void TaskProcess(void* pvParameters) {
  ADCData_t data;
  float vReal[FFT_SAMPLE_SIZE];
  float vImag[FFT_SAMPLE_SIZE];
  float freqs[FFT_SAMPLE_SIZE];
  int count = 0;
  float prev_maxFreq = 0.0;
  float maxFreq = 0.0;


  while (count <= 5) {
    float sum = 0.0;

    // Receive FFT_SAMPLE_SIZE samples
    for (int i = 0; i < FFT_SAMPLE_SIZE; i++) {
      if (xQueueReceive(sampleQueue, &data, portMAX_DELAY) == pdTRUE) {
        freqs[i] = 1e6 / data.delta_time;  // Calculate sampling frequency from delta time
        vReal[i] = data.adc_value;         // Real part of FFT input
        vImag[i] = 0.0;                    // Imaginary part initialized to 0
        sum += freqs[i];                   // Accumulate sampling frequencies
      } else {
        Serial.print("Process Error: Queue broken!");
        break;
      }
    }

    float mean_sampling_freq = sum / FFT_SAMPLE_SIZE;

    ArduinoFFT<float> FFT(vReal, vImag, FFT_SAMPLE_SIZE, mean_sampling_freq, false);

    // Track ADC min and max values
    int min_adc = 1024, max_adc = 0;
    for (int i = 0; i < FFT_SAMPLE_SIZE; i++) {
      if (vReal[i] < min_adc) min_adc = vReal[i];
      if (vReal[i] > max_adc) max_adc = vReal[i];
    }


    FFT.windowing(vReal, FFT_SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Apply window
    FFT.compute(vReal, vImag, FFT_SAMPLE_SIZE, FFT_FORWARD);                  // Compute FFT

    for (int i = 0; i < 20; i++) vReal[i] = 0.0;  // Zero out low-frequency bins

    FFT.complexToMagnitude(vReal, vImag, FFT_SAMPLE_SIZE);  // Convert to magnitude

    float peakFrequency = FFT.majorPeak(vReal, FFT_SAMPLE_SIZE, mean_sampling_freq);


    float binWidth = mean_sampling_freq / FFT_SAMPLE_SIZE;

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
      portENTER_CRITICAL(&samplingMux);
      sampleFrequency = 2.0 * maxFreq;
      portEXIT_CRITICAL(&samplingMux);
    }


    if (toPlot_fft) {
      // Serial.Printf("[FFT] sampleFrequency set to %.2f Hz (max freq: %.2f Hz)\n", sampleFrequency, maxFreq);
      // Serial.printf("[FFT] Max freq: %.2f Hz\n", maxFreq);
      for (int i = 1; i < N; i++) {
        float freq = i * binWidth;
        Serial.printf("FFT:%.2f:%.2f\n", freq, vReal[i]);  // FFT:<frecuencia_en_Hz>:<magnitud>
      }
    }
    count++;

    Serial.printf("[FFT] Max freq: %.2f Hz\n", maxFreq);
    Serial.printf("[FFT] FINAL Sample freq: %.2f Hz\n", sampleFrequency);
    Serial.printf("[FTT] Dominant Frequency is %.2f Hz \n", peakFrequency);

    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1000 milliseconds
  }



  vTaskDelete(NULL);
}



// ======================
// AGGREGATION + MQTT PUBLISH TASK (Core 0)
// ======================
void TaskAggregation(void* param) {
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
        Serial.printf("MQTT:%.2f\n", average);
        Serial.printf("SAMPLE_FREQ:%.2f\n", sampleFrequency);  // Add sampling frequency output
        Serial.printf("[FFT] FINAL Sample freq: %.2f Hz\n", sampleFrequency);

        if (mqtt_connected) {
          char payload[32];
          snprintf(payload, sizeof(payload), "%.2f", average);

          if (client.publish(mqtt_topic, payload)) {
            Serial.printf("[MQTT] Sent average: %.2f\n", average);
          } else {
            Serial.print("[MQTT] Failed to send data!");
          }
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
unsigned long lastSleepTime = 0;  // Tracks the last time deep sleep was triggered

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Check if the ESP32 woke up from deep sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("{\"status\":\"Woke up from deep sleep\"}");
  } else {
    Serial.println("{\"status\":\"Power-on or reset\"}");
  }

  connectToWiFi();  // Connect to WiFi
  client.setServer(mqtt_server, mqtt_port);
  connectToMQTT();                      // Connect to MQTT broker
  mqtt_connected = client.connected();  // Set connection flag

  // Send WiFi and MQTT status
  Serial.printf(
    "{\"wifi_status\":\"%s\",\"wifi_ip\":\"%s\",\"mqtt_status\":\"%s\"}\n",
    (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected",
    WiFi.localIP().toString().c_str(),
    mqtt_connected ? "connected" : "disconnected"
  );

  dacWrite(DAC_PIN, offset);       // Initialize DAC output
  analogReadResolution(8);         // 8-bit ADC resolution
  analogSetAttenuation(ADC_11db);  // ADC attenuation setting

  sampleQueue = xQueueCreate(QUEUE_LENGTH, sizeof(ADCData_t));
  if (sampleQueue == NULL) {
    Serial.print("FATAL: Queue creation failed!");
    while (1)
      ;
  }

  aggQueue = xQueueCreate(QUEUE_LENGTH, sizeof(ADCData_t));
  if (aggQueue == NULL) {
    Serial.print("FATAL: Queue creation failed!");
    while (1)
      ;

    serialMutex = xSemaphoreCreateMutex();
    if (serialMutex == NULL) {
      Serial.print("FATAL: serialMutex creation failed!");
      while (1)
        ;
    }
  }

  // Create tasks and assign them to specific cores
  xTaskCreatePinnedToCore(TaskDACWrite, "DAC_Gen", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskADCRead, "ADC_Sample", 4096, NULL, 2, &ADC_TaskHandle, 1);
  xTaskCreatePinnedToCore(TaskProcess, "Data_Process", 8192, NULL, 1, &Process_TaskHandle, 0);
  xTaskCreatePinnedToCore(TaskAggregation, "RollingAverage", 4096, NULL, 1, NULL, 0);

  Serial.print("System Initialized: Tasks Running");

  // Set up deep sleep for 10 seconds
  esp_sleep_enable_timer_wakeup(10 * 1000000);  // 10 seconds in microseconds
}

// ======================
// MAIN LOOP (Unused)
// ======================
void loop() {
  unsigned long currentTime = millis();

  // Check if 1 minute has passed since the last deep sleep
  if (currentTime - lastSleepTime >= 60000) {  // 60,000 ms = 1 minute
    Serial.println("{\"status\":\"Entering deep sleep for 10 seconds\"}");
    delay(100);  // Allow time for the message to be sent
    lastSleepTime = currentTime;  // Update the last sleep time
    esp_deep_sleep_start();       // Enter deep sleep
  }

  // Perform other tasks here (e.g., sampling, MQTT communication)
  delay(1000);  // Add a delay to avoid busy looping
}


// ======================
// WIFI CONNECTION HELPER
// ======================
void connectToWiFi() {
  Serial.println("[WiFi] Connecting to WiFi...");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 10000;  // 10 seconds timeout

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] Failed to connect to WiFi. Restarting...");
    ESP.restart();  // Restart the ESP32 if WiFi connection fails
  }
}

// ======================
// MQTT CONNECTION HELPER
// ======================
void connectToMQTT() {
  Serial.println("[MQTT] Connecting to MQTT broker...");
  int attempts = 0;
  const int maxAttempts = 5;  // Maximum number of connection attempts

  while (!client.connected() && attempts < maxAttempts) {
    Serial.printf("[MQTT] Attempt %d/%d...\n", attempts + 1, maxAttempts);

    if (client.connect("ESP32Client")) {  // Replace "ESP32Client" with a unique client ID if needed
      Serial.println("[MQTT] Connected to broker!");
      return;
    } else {
      Serial.printf("[MQTT] Connection failed, rc=%d. Retrying in 2 seconds...\n", client.state());
      Serial.printf("[MQTT] Connection state: %d\n", client.state());
      delay(2000);
      attempts++;
    }
  }

  if (!client.connected()) {
    Serial.println("[MQTT] Failed to connect to MQTT broker after multiple attempts. Restarting...");
    ESP.restart();  // Restart the ESP32 if MQTT connection fails
  }
}
