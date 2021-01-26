#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ALARM_BUTTON_PIN D2

/**
 * Sensor data message, used on all sensor tasks.
 * 
 */ 
typedef struct 
{
  int pin; /*!< Sensor pin */
  int value; /*!< Value readed */
} SensorDataMsg;

/**
 * Actuator data message.
 * 
 */ 
typedef struct 
{
  int pin; /*!< Sensor pin */
  int value; /*!< Value readed */
} ActuatorDataMsg;


// Sensor queue
static QueueHandle_t s_sensorDataQueue;
// Actuator queue
static QueueHandle_t s_actuatorDataQueue;

static TaskHandle_t s_main_task;
static TaskHandle_t s_printLCDTask;



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
static void main_task_handler(void* pvParameters) 
{

}

/**
 * @brief Rotation pot task handler. Read the target temperature.
 * 
 */
static void temp_task_handler(void* pvParameters) 
{
}

void setup() {
  delay(3000);
  Serial.begin(115200);

  s_sensorDataQueue = xQueueCreate(10, sizeof(SensorDataMsg));
  s_actuatorDataQueue = xQueueCreate(10, sizeof(ActuatorDataMsg));
  attachInterrupt(digitalPinToInterrupt(ALARM_BUTTON_PIN), &alarm_button_handler, FALLING);

  xTaskCreatePinnedToCore(main_task_handler, "mainTask", 1024, NULL, 5, &s_printLCDTask, 0);
  xTaskCreatePinnedToCore(temp_task_handler, "rotPotTask", 1024, NULL, 3, &s_main_task, 1);
    
}

void loop() {
  // put your main code here, to run repeatedly:
}