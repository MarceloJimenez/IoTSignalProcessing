#define TOTAL_SAMPLES 100             // Total de muestras a tomar
#define ADC_PIN 34                    // Pin ADC (GPIO34)
QueueHandle_t deltaQueue;            // Cola para almacenar los delta_t

void TaskADCRead(void *pvParameters) {
  int dummy;
  unsigned long t_prev = micros();   // Tiempo inicial antes de la primera lectura
  unsigned long t_now;
  unsigned long delta;

  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    dummy = analogRead(ADC_PIN);     // Lectura del ADC
    t_now = micros();                // Tiempo después de la lectura

    if (i > 0) {
      delta = t_now - t_prev;        // Delta entre esta muestra y la anterior
      xQueueSend(deltaQueue, &delta, portMAX_DELAY); // Enviar delta a la cola
    }

    t_prev = t_now;                  // Actualizar tiempo anterior
  }

  vTaskDelete(NULL);                 // Finaliza la tarea de muestreo
}

void TaskProcess(void *pvParameters) {
  unsigned long delta;
  float freqs[TOTAL_SAMPLES - 1];    // Arreglo local para frecuencias
  float sum = 0.0;

  for (int i = 0; i < TOTAL_SAMPLES - 1; i++) {
    // Espera y recibe delta desde la cola
    if (xQueueReceive(deltaQueue, &delta, portMAX_DELAY)) {
      freqs[i] = 1e6 / delta;        // f = 1 / T
      sum += freqs[i];               // Acumular frecuencia
    }
  }

  float mean = sum / (TOTAL_SAMPLES - 1); // Promedio de frecuencias

  // Calcular desviación estándar
  float sq_diff_sum = 0.0;
  for (int i = 0; i < TOTAL_SAMPLES - 1; i++) {
    sq_diff_sum += pow(freqs[i] - mean, 2);
  }
  float stddev = sqrt(sq_diff_sum / (TOTAL_SAMPLES - 1));

  // Mostrar resultados
  Serial.printf("Frecuencia promedio: %.2f Hz\n", mean);
  Serial.printf("Desviación estándar: %.2f Hz\n", stddev);

  vTaskDelete(NULL); // Finaliza la tarea de procesamiento
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Crear la cola para almacenar TOTAL_SAMPLES - 1 valores de tipo unsigned long
  deltaQueue = xQueueCreate(TOTAL_SAMPLES - 1, sizeof(unsigned long));

  // Verificar que la cola se haya creado correctamente
  if (deltaQueue == NULL) {
    Serial.println("Error al crear la cola");
    while (true); // Detener ejecución si falla
  }

  // Crear tarea de lectura del ADC
  xTaskCreatePinnedToCore(
    TaskADCRead,
    "ADC Read",
    4096,
    NULL,
    2,
    NULL,
    0 // Core 0
  );

  // Crear tarea de procesamiento de datos
  xTaskCreatePinnedToCore(
    TaskProcess,
    "Process Data",
    4096,
    NULL,
    1,
    NULL,
    1 // Core 1
  );
}

void loop() {
  // Nada aquí, todo se ejecuta en tareas FreeRTOS
}
