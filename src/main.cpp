#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"
#include <DHT.h>

//prueba

// Sensor queue
static QueueHandle_t s_sensorDataQueue;
// Actuator queue
static QueueHandle_t s_actuatorDataQueue;

DHT dht(TEMPERATURE_SENSOR_PIN, DHT11);
/**
 * @brief Alarm Button Handler
 * 
 */
static void IRAM_ATTR alarm_button_handler()
{
}

/**
 * @brief Main Task Handler
 * 
 */
static void main_task_handler(void *pvParameters)
{
}

/**
 * @brief Temperature Task Handler
 * 
 */
static void temperature_task_handler(void *pvParameters)
{
  const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
  float lastTemperature = 0;
  bool firstExecution = true;
  for (;;)
  {

    // Tomamos la temperatura
    float actualTemperature = dht.readTemperature();

    // Si es la primera ejecucion o la diferencia es mayor al limite
    if (firstExecution || abs(actualTemperature - lastTemperature) > TEMPERATURE_THRESHOLD)
    {
      firstExecution = false;
      lastTemperature = actualTemperature;

      // Mandamos el mensaje
      SensorDataMsg temperatureMsg;
      temperatureMsg.pin = TEMPERATURE_SENSOR_PIN;
      // Se multiplica la temperatura por 100 y se convierte a entero
      temperatureMsg.value = (int)(actualTemperature * 100);
      xQueueSend(s_sensorDataQueue, (void *)&temperatureMsg, (TickType_t)0);
    }

    Serial.println(actualTemperature);
    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}

/**
 * @brief Air quality Task Handler
 * 
 */
static void air_quality_task_handler(void *pvParameters)
{
  const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
  int lastAirMeasure = 0;
  bool firstExecution = true;
  for (;;)
  {

    // Medimos la calidad del aire
    int actualAirMeasure = analogRead(MQ_SENSOR_PIN);

    // Si es la primera ejecucion o el valor actual esta por encima del limite y el anterior estaba por debajo
    // (evita mandar mensajes continuamente si nos mantenemos por encima del limite)
    if (firstExecution || (actualAirMeasure > AIR_QUALITY_THRESHOLD && lastAirMeasure <= AIR_QUALITY_THRESHOLD))
    {

      firstExecution = false;
      lastAirMeasure = actualAirMeasure;

      // Se envia el mensaje
      SensorDataMsg airMsg;
      airMsg.pin = MQ_SENSOR_PIN;
      airMsg.value = actualAirMeasure;
      xQueueSend(s_sensorDataQueue, (void *)&airMsg, (TickType_t)0);
    }
    else if (actualAirMeasure <= AIR_QUALITY_THRESHOLD && lastAirMeasure > AIR_QUALITY_THRESHOLD)
    {
      // Si el valor leido es menor que el limite y el anterior es mayor que el limite,
      // mandamos el mensaje para avisar de un cambio de estado
      lastAirMeasure = actualAirMeasure;

      // Se envia el mensaje
      SensorDataMsg airMsg;
      airMsg.pin = MQ_SENSOR_PIN;
      airMsg.value = actualAirMeasure;
      xQueueSend(s_sensorDataQueue, (void *)&airMsg, (TickType_t)0);
    }
    else if (actualAirMeasure <= AIR_QUALITY_THRESHOLD)
    {
      // Siempre que el valor sea menor que el limite lo actualizamos
      lastAirMeasure = actualAirMeasure;
    }

    Serial.println(actualAirMeasure);

    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}

/**
 * @brief Rotation pot task handler. Read the target temperature.
 * 
 */
static void temp_task_handler(void *pvParameters)
{
}

void setup()
{
  Serial.begin(115200);

  // Sensor initialization
  dht.begin();
  delay(2000);

  s_sensorDataQueue = xQueueCreate(10, sizeof(SensorDataMsg));
  s_actuatorDataQueue = xQueueCreate(10, sizeof(ActuatorDataMsg));
  attachInterrupt(digitalPinToInterrupt(ALARM_BUTTON_PIN), &alarm_button_handler, FALLING);

  xTaskCreatePinnedToCore(main_task_handler, "mainTask", 1024, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(temp_task_handler, "rotPotTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(temperature_task_handler, "temperatureTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(air_quality_task_handler, "airQualityTask", 1024, NULL, 3, NULL, 1);
}

void loop()
{
  // Do Nothing
}
