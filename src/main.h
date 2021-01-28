// Pines
#define ALARM_BUTTON_PIN D2
#define PRESENCE_PIN D9
#define PRESENCE_READ_PERIOD 5000
#define TEMPERATURE_SENSOR_PIN D4
#define MQ_SENSOR_PIN A3
#define LDR_PIN A2


// Rangos
#define AIR_QUALITY_THRESHOLD 3500
#define TEMPERATURE_THRESHOLD 1
#define LIGHT_QUANTITY_THRESHOLD 1000

#define SID_WIFI PIO_WIFI 
#define ENVIRONMENT_TOPIC "/1/1/bedroom/1/environment"
#define PRESENCE_TOPIC "/1/1/bedroom/1/presence"
#define ALARM_TOPIC "/1/1/bedroom/1/alarm"
// #define BROKER_IP "192.168.100.2"
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
static void main_task_handler(void *pvParameters);
static void temp_task_handler(void *pvParameters);
static void send_sensor_msg(int sensorPin, int value);
static void temperature_task_handler(void *pvParameters);
static void air_quality_task_handler(void *pvParameters);
static void light_quantity_task_handler(void *pvParameters);
static void wifiConnect();

