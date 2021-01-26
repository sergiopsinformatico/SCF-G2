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

  s_sensorDataQueue = xQueueCreate(10, sizeof(SensorDataMsg));
  s_actuatorDataQueue = xQueueCreate(10, sizeof(ActuatorDataMsg));
  attachInterrupt(digitalPinToInterrupt(ALARM_BUTTON_PIN), &alarm_button_handler, FALLING);

  xTaskCreatePinnedToCore(main_task_handler, "mainTask", 1024, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(temp_task_handler, "rotPotTask", 1024, NULL, 3, NULL, 1);
}

void loop()
{
  // Do Nothing
}
