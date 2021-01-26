// Pines
#define ALARM_BUTTON_PIN D2
#define PRESENCE_PIN D9
#define PRESENCE_READ_PERIOD 500

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
static void send_sensor_msg(int sensorPin, int value);