#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"
#include <DHT.h>

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

  int presence = LOW;
  bool presenceChange = false;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    SensorDataMsg currentPinRead;
    vTaskDelayUntil(&xLastWakeTime, 50 / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
    // Se lee de la cola de sensores
    if (xQueueReceive(s_sensorDataQueue, &currentPinRead, portMAX_DELAY) == pdPASS)
    {
      // En función del tipo de msg
      switch (currentPinRead.pin)
      {
      case PRESENCE_PIN:
        if(presence != currentPinRead.value) {
          presence = currentPinRead.value;
          presenceChange = true;
        }
        break;

      default:
        break;
      }
    }

    // TODO Si han variado una o más condiciones se
    // crea el mensaje para topic de salida
    if(presenceChange) {
      presenceChange = false;
      Serial.print("PRESENCE ");
      Serial.println(currentPinRead.value);
    }
  }
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

/**
 * @brief Presence task handler. Read the target temperature.
 * 
 */
static void presence_task_handler(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, PRESENCE_READ_PERIOD / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
    send_sensor_msg(PRESENCE_PIN, digitalRead(PRESENCE_PIN));
  }
}
/**
 * @brief Light quantity Task Handler
 * 
 */
static void light_quantity_task_handler(void *pvParameters)
{
  const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
  const int R_DARKNESS = 1000;
  const int R_LIGHT = 15;
  const int R_CALIBRATION = 10;

  float lastLightQuantity = 0;
  bool firstExecution = true;
  for (;;)
  {

    // Tomamos la medida de la cantidad de luz
    int actualLightQuantity = analogRead(LDR_PIN);
    actualLightQuantity = ((long)actualLightQuantity*R_DARKNESS*10)/((long)R_LIGHT*R_CALIBRATION*(1024-actualLightQuantity));
    // Si es la primera ejecucion o la diferencia es mayor al limite
    if (firstExecution || abs(actualLightQuantity - lastLightQuantity) > LIGHT_QUANTITY_THRESHOLD)
    {
      firstExecution = false;
      lastLightQuantity = actualLightQuantity;

      // Mandamos el mensaje
      SensorDataMsg lightQuantityMsg;
      lightQuantityMsg.pin = LDR_PIN;
      lightQuantityMsg.value = actualLightQuantity;
      xQueueSend(s_sensorDataQueue, (void *)&lightQuantityMsg, (TickType_t)0);
    }

    Serial.println(actualLightQuantity);
    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}


static void send_sensor_msg(int sensorPin, int value)
{
  SensorDataMsg currentPinRead;
  currentPinRead.pin = sensorPin;
  currentPinRead.value = value;
  xQueueSend(s_sensorDataQueue, &currentPinRead, portMAX_DELAY);
}

void setup()
{
  Serial.begin(115200);
  pinMode(PRESENCE_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  // Sensor initialization
  dht.begin();
  delay(2000);

  s_sensorDataQueue = xQueueCreate(10, sizeof(SensorDataMsg));
  s_actuatorDataQueue = xQueueCreate(10, sizeof(ActuatorDataMsg));
  attachInterrupt(digitalPinToInterrupt(ALARM_BUTTON_PIN), &alarm_button_handler, FALLING);

  xTaskCreatePinnedToCore(main_task_handler, "mainTask", 1024, NULL, 5, NULL, 0);
  // xTaskCreatePinnedToCore(temp_task_handler, "tempPotTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(presence_task_handler, "presencePotTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(temperature_task_handler, "temperatureTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(air_quality_task_handler, "airQualityTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(light_quantity_task_handler, "lightQuantityTask", 1024, NULL, 3, NULL, 1);
}

void loop()
{
  // Do Nothing
}
