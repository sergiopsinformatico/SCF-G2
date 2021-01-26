// Pines
#define ALARM_BUTTON_PIN D2
#define TEMPERATURE_SENSOR_PIN D4
#define MQ_SENSOR_PIN A3


// Rangos
#define AIR_QUALITY_THRESHOLD 3500
#define TEMPERATURE_THRESHOLD 1

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

// Funciones
static void main_task_handler(void *pvParameters);
static void temp_task_handler(void *pvParameters);
static void temperature_task_handler(void *pvParameters);
static void air_quality_task_handler(void *pvParameters);