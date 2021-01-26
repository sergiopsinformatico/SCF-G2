#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"

// Sensor queue
static QueueHandle_t s_sensorDataQueue;
// Actuator queue
static QueueHandle_t s_actuatorDataQueue;

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
 * @brief Temp task handler. Read the target temperature.
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

  s_sensorDataQueue = xQueueCreate(10, sizeof(SensorDataMsg));
  s_actuatorDataQueue = xQueueCreate(10, sizeof(ActuatorDataMsg));
  attachInterrupt(digitalPinToInterrupt(ALARM_BUTTON_PIN), &alarm_button_handler, FALLING);

  xTaskCreatePinnedToCore(main_task_handler, "mainTask", 1024, NULL, 5, NULL, 0);
  // xTaskCreatePinnedToCore(temp_task_handler, "tempPotTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(presence_task_handler, "presencePotTask", 1024, NULL, 3, NULL, 1);
}

void loop()
{
  // Do Nothing
}
