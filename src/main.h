#ifndef MAIN_H_ /* Include guard */
#define MAIN_H_

#include "environment.pb.h"
#include "pb.h"

// Pines
#define HUMIDITY_SENSOR_PIN A0
#define TEMPERATURE_SENSOR_PIN D4
#define SERVO_PIN D5
#define PRESENCE_PIN D13
#define RELAY_PIN D11
#define ALARM_BUTTON_PIN D12
#define MQ_SENSOR_PIN A3
#define LDR_PIN A2

// Rangos
#define AIR_QUALITY_THRESHOLD 25
#define TEMPERATURE_THRESHOLD 1
#define HUMIDITY_THRESHOLD 3
#define LIGHT_QUANTITY_THRESHOLD 100
#define PRESENCE_READ_PERIOD 2000

#define SID_WIFI PIO_WIFI
#define ENVIRONMENT_TOPIC "1/1/bedroom/1/environment"
#define PRESENCE_TOPIC "1/1/bedroom/1/presence"
#define ALARM_TOPIC "1/1/bedroom/1/alarm"
#define ACTIONS_TOPIC "1/1/bedroom/1/actions"
#define BROKER_PORT 2883

// Structs
/**
 * Sensor data message, used on all sensor tasks.
 * 
 */
typedef struct
{
    int pin;   /*!< Sensor pin */
    int value; /*!< Value readed */
} SensorDataMsg;

/**
 * Actuator data message.
 * 
 */
typedef struct
{
    int pin;   /*!< Sensor pin */
    int value; /*!< Value readed */
} ActuatorDataMsg;

/**
 * Actuator data message.
 * 
 */
typedef struct
{
    int lightLevel;
    float temperature;
    float humidity;
    int airQuality;
} EnvironmentTopicMsg;

// Funciones
static void wifiConnect();
void mqttConnect();
void debug_print_alarm(alartMessage message, bool result);
static void IRAM_ATTR alarm_button_handler();
static void main_task_handler(void *pvParameters);
static void temperature_task_handler(void *pvParameters);
static void air_quality_task_handler(void *pvParameters);
static void light_quantity_task_handler(void *pvParameters);
static void send_sensor_msg(int sensorPin, int value);
static void IRAM_ATTR pir_interrupt_handler();
void debug_print(environmentMessage message, bool result);
static environmentMessage load_environment_message();
static void environment_send_task_handler(void *pvParameters);
static void alarm_send_task_handler(void *pvParameters);
static void testing_task_handler(void *pvParameters);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void networkConnect();

#endif