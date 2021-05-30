#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"
#include <DHT.h>
#include <Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "environment.pb.h"

#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Sensor queue
static QueueHandle_t s_sensorDataQueue;
// Actuator queue
static QueueHandle_t s_actuatorDataQueue;

static bool s_alarm = false;
static int s_presence = -1;
static float s_temperature = -1;
static int s_lightLevel = -1;
static int s_airQuality = -1;
static float s_humidity = -1;
static Servo servoMotor;
#define ONBOARD_LED 2

DHT dht(TEMPERATURE_SENSOR_PIN, DHT22);
WiFiClient espClient;
PubSubClient client(espClient);

// ----- SETUP -----
void setup()
{
  delay(2000);
  Serial.begin(115200);

  // Pin initialization
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(ALARM_BUTTON_PIN, INPUT);
  pinMode(PRESENCE_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(0);

  // Sensor initialization
  dht.begin();

  delay(2000);
  networkConnect();

  s_sensorDataQueue = xQueueCreate(10, sizeof(SensorDataMsg));
  s_actuatorDataQueue = xQueueCreate(10, sizeof(ActuatorDataMsg));

  attachInterrupt(digitalPinToInterrupt(ALARM_BUTTON_PIN), &alarm_button_handler, FALLING);
  attachInterrupt(digitalPinToInterrupt(PRESENCE_PIN), &pir_interrupt_handler, CHANGE);

  xTaskCreatePinnedToCore(mqtt_reconnect_task_handler, "mqReconnectTask", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(main_task_handler, "mainTask", 1024, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(temperature_task_handler, "temperatureTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(air_quality_task_handler, "airQualityTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(light_quantity_task_handler, "lightQuantityTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(environment_send_task_handler, "envTask", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(testing_task_handler, "testingTask", 1024, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(alarm_send_task_handler, "alarmTask", 2048, NULL, 3, NULL, 1);

  //Suscripción al topic ENVIROMENT_TOPIC MQTT MARCOS
  
}

void loop()
{
  client.loop();
}

// ----- WIFI Y MQTT -----

/**
 * @brief Function to connect to a wifi network
 *
 */
static void wifiConnect()
{
  // WiFi.begin(SID_WIFI, PIO_PASS);
  WiFi.begin("mixmi", "dmmqlssac");
  Serial.print(F("Status "));
  Serial.println(WiFi.status());

  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(ONBOARD_LED, HIGH);
    delay(1000);
    digitalWrite(ONBOARD_LED, LOW);
    delay(1000);
    Serial.print(F("Connecting to WiFi  "));
    Serial.print(SID_WIFI);
    Serial.print(F(" "));
    Serial.print(PIO_PASS);
    Serial.print(F(" ... Status "));
    Serial.println(WiFi.status());
  }
  digitalWrite(ONBOARD_LED, HIGH);
  Serial.println(F("Connected to the WiFi network"));
  Serial.print(F("IP Address: "));
  Serial.println(WiFi.localIP());
}

/**
 * @brief Function to connect to mqtt broker
 *
 */
void mqttConnect()
{
  client.setServer(BROKER_IP, BROKER_PORT);
  client.subscribe(ACTIONS_TOPIC);
  client.setCallback(mqttCallback);
  // client.setSocketTimeout(30);
  // client.setKeepAlive(0);
  while (!client.connected())
  {
    digitalWrite(ONBOARD_LED, HIGH);
    Serial.print(F("MQTT connecting ... "));
    Serial.print(BROKER_IP);

    if (client.connect("ESP32Client1"))
    {
      Serial.println(F("connected"));
    }
    else
    {
      Serial.print(F("failed, status code ="));
      Serial.print(client.state());
      Serial.println(F("try again in 5 seconds"));
      delay(1000);
      digitalWrite(ONBOARD_LED, LOW);
    }
  }
  digitalWrite(ONBOARD_LED, HIGH);
}

/**
 * @brief Connects to wifi and then to MQTT
 *
 */
void networkConnect()
{
#ifdef PIO_WIFI
  wifiConnect();
#endif
  mqttConnect();
}

// ----- INTERRUPCIONES -----

/**
 * @brief Alarm Button Handler
 *
 */
static void IRAM_ATTR alarm_button_handler()
{
  s_alarm = true;
}

/**
 * @brief PIR Handler
 *
 */
static void IRAM_ATTR pir_interrupt_handler()
{
  int newPresence = digitalRead(PRESENCE_PIN);
  digitalWrite(RELAY_PIN, newPresence);
  send_sensor_msg(PRESENCE_PIN, newPresence);
}

// ----- TASKS -----

/**
 * @brief Main Task Handler
 *
 */
static void main_task_handler(void *pvParameters)
{
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
        s_presence = currentPinRead.value;
        break;
      case TEMPERATURE_SENSOR_PIN:
        s_temperature = currentPinRead.value / 100;
        break;
      case HUMIDITY_SENSOR_PIN:
        s_humidity = currentPinRead.value / 100;
        break;
      case MQ_SENSOR_PIN:
        s_airQuality = currentPinRead.value;
        break;
      case LDR_PIN:
        s_lightLevel = currentPinRead.value;
        break;
      default:
        break;
      }
    }
  }
  vTaskDelete(NULL);
}

/**
 * @brief Temperature Task Handler
 *
 */
static void temperature_task_handler(void *pvParameters)
{
  const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
  float lastTemperature = -100;
  float lastHumidity = -100;
  for (;;)
  {
    vTaskDelay(xDelay);
    // Tomamos la temperatura
    float currentTemperature = dht.readTemperature();
    if ((lastTemperature == -100 && !isnan(currentTemperature)) || (!isnan(currentTemperature) && abs(currentTemperature - lastTemperature) > TEMPERATURE_THRESHOLD))
    {
      lastTemperature = currentTemperature;
      // Mandamos el mensaje
      send_sensor_msg(TEMPERATURE_SENSOR_PIN, (int)(currentTemperature * 100));
    }

    float currentHumidity = dht.readHumidity();
    if ((lastHumidity == -100 && !isnan(currentHumidity)) || (!isnan(currentHumidity) && abs(currentHumidity - lastHumidity) > HUMIDITY_THRESHOLD))
    {
      lastHumidity = currentHumidity;
      send_sensor_msg(HUMIDITY_SENSOR_PIN, (int)(currentHumidity * 100));
    }
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
    vTaskDelay(xDelay);
    // Medimos la calidad del aire
    int actualAirMeasure = analogRead(MQ_SENSOR_PIN);
    // Si es la primera ejecucion o el valor actual esta por encima del limite y el anterior estaba por debajo
    // (evita mandar mensajes continuamente si nos mantenemos por encima del limite)

    if (firstExecution && !isnan(actualAirMeasure))
    {
      firstExecution = false;
      // Mandamos el mensaje
      send_sensor_msg(MQ_SENSOR_PIN, actualAirMeasure);
    }
    else if (!isnan(actualAirMeasure) && abs(actualAirMeasure - lastAirMeasure) > AIR_QUALITY_THRESHOLD)
    {
      // Si el valor leido es menor que el limite y el anterior es mayor que el limite,
      // mandamos el mensaje para avisar de un cambio de estado

      // Mandamos el mensaje
      send_sensor_msg(MQ_SENSOR_PIN, actualAirMeasure);
    }

    lastAirMeasure = actualAirMeasure;
  }
  vTaskDelete(NULL);
}

/**
 * @brief Light quantity Task Handler
 *
 */
static void light_quantity_task_handler(void *pvParameters)
{
  const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
  // const int R_DARKNESS = 1000;
  // const int R_LIGHT = 15;
  // const int R_CALIBRATION = 10;

  int lastLightQuantity = 10000;
  bool firstExecution = true;
  for (;;)
  {
    vTaskDelay(xDelay);
    // Tomamos la medida de la cantidad de luz
    int actualLightQuantity = analogRead(LDR_PIN);
    // actualLightQuantity = ((long)actualLightQuantity * R_DARKNESS * 10) / ((long)R_LIGHT * R_CALIBRATION * (1024 - actualLightQuantity));
    // Si es la primera ejecucion o la diferencia es mayor al limite
    if (firstExecution && !isnan(actualLightQuantity))
    {
      firstExecution = false;
      lastLightQuantity = actualLightQuantity;
      // Mandamos el mensaje
      send_sensor_msg(LDR_PIN, actualLightQuantity);
    }
    else if (abs(actualLightQuantity - lastLightQuantity) > LIGHT_QUANTITY_THRESHOLD)
    {
      lastLightQuantity = actualLightQuantity;
      // Mandamos el mensaje
      send_sensor_msg(LDR_PIN, actualLightQuantity);
    }
  }
  vTaskDelete(NULL);
}

/**
 * @brief This task sends environment messages only if it's necessary .
 */
static void environment_send_task_handler(void *pvParameters)
{

  const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;

  for (;;)
  {
    vTaskDelay(xDelay);
    // Load environment message
    environmentMessage message = load_environment_message();

    // If data loaded sends message
    if (message.has_airQuality || message.has_humidity || message.has_lightLevel || message.has_temperature || message.has_presence)
    {
      uint8_t buffer[500];
      pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
      bool status = pb_encode(&stream, environmentMessage_fields, &message);
      if (!status)
      {
        Serial.println(F("Failed to encode"));
        return;
      }

      bool result = client.publish(ENVIRONMENT_TOPIC, buffer, stream.bytes_written);
      debug_print(message, result);
    }
    // Reset message
    s_lightLevel = s_temperature = s_humidity = s_airQuality = s_presence = -1;
  }
  vTaskDelete(NULL);
}

/**
 * @brief Send alarm message
 *
 */
static void alarm_send_task_handler(void *pvParameters)
{
  const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

  for (;;)
  {
    vTaskDelay(xDelay);
    if (s_alarm)
    {
      s_alarm = false;
      Serial.print("Sending alarm...");
      uint8_t buffer[500];
      pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

      // Creates empty message
      alartMessage message = alartMessage_init_zero;
      message.alarm = true;
      bool status = pb_encode(&stream, alartMessage_fields, &message);
      if (!status)
      {
        Serial.println(F("Failed to encode"));
        return;
      }

      bool result = client.publish(ALARM_TOPIC, buffer, stream.bytes_written);
      debug_print_alarm(message, result);
    }
  }
  vTaskDelete(NULL);
}

/**
 * @brief Testing 
 */
static void testing_task_handler(void *pvParameters)
{
  const TickType_t xDelay = 300000 / portTICK_PERIOD_MS;

  int valueServo = 0;
  int increment = -20;
  for (;;)
  {
    vTaskDelay(xDelay);

    servoMotor.write(valueServo);
    if (valueServo == 180 || valueServo == 0)
    {
      increment = increment * -1;
    }
    valueServo += increment;
  }
  vTaskDelete(NULL);
}

// ----- UTILS -----

void debug_print_alarm(alartMessage message, bool result)
{
  Serial.println(F("\nSending alarm"));
  Serial.println(result);
}

static void send_sensor_msg(int sensorPin, int value)
{
  SensorDataMsg currentPinRead;
  currentPinRead.pin = sensorPin;
  currentPinRead.value = value;
  xQueueSend(s_sensorDataQueue, &currentPinRead, portMAX_DELAY);
}

void debug_print(environmentMessage message, bool result)
{
  Serial.println(F("\nEnvironment change"));
  if (message.has_airQuality)
  {
    Serial.print(F("AirQuality"));
    Serial.println(message.airQuality);
  }
  if (message.has_humidity)
  {
    Serial.print(F("Humidity "));
    Serial.println(message.humidity);
  }
  if (message.has_lightLevel)
  {
    Serial.print(F("LightLevel "));
    Serial.println(message.lightLevel);
  }
  if (message.has_temperature)
  {
    Serial.print(F("Temperature "));
    Serial.println(message.temperature);
  }
  if (message.has_presence)
  {
    Serial.print(F("Presence "));
    Serial.println(message.presence);
  }
  Serial.print("Now millis ");
  Serial.println(millis());
  Serial.print(F("MQTT SEND RESULT"));
  Serial.println(result);
}

/**
 * @brief Creates a new environment message
 *
 */
static environmentMessage load_environment_message()
{
  // Creates empty message
  environmentMessage message = environmentMessage_init_zero;

  // Load data
  message.airQuality = s_airQuality;
  message.has_airQuality = s_airQuality != -1;

  message.humidity = s_humidity;
  message.has_humidity = s_humidity != -1;

  message.temperature = s_temperature;
  message.has_temperature = s_temperature != -1;

  message.lightLevel = s_lightLevel;
  message.has_lightLevel = s_lightLevel != -1;

  message.presence = s_presence == HIGH;
  message.has_presence = s_presence != -1;

  return message;
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{ //MQTT MARCOS
  Serial.printf("Topic: %s\r\n", ACTIONS_TOPIC);
  Serial.print("Payload: ");
  for (int i = 0; i < length; i++)
  {

    pb_istream_t stream = pb_istream_from_buffer(payload, length);
    ActuatorMessage message = ActuatorMessage_init_zero;

    pb_decode(&stream, ActuatorMessage_fields, &message);
    Serial.print("IT WORKS!!!");
    Serial.print("Window ");
    Serial.println(message.window);
    Serial.print("Light ");
    Serial.println(message.light);
  }
  Serial.println();
}


/**
 * @brief Reconnect to MQTT
 *
 */
static void mqtt_reconnect_task_handler(void *pvParameters)
{
  const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;

  for (;;)
  {
    if (!client.connected())
    {
      Serial.print(F("MQTT connecting ... "));
      Serial.print(BROKER_IP);

      if (client.connect("ESP32Client1"))
      {
        Serial.println(F("connected"));
        client.subscribe(ACTIONS_TOPIC);
      }
    }

    if (client.connected())
    {
      client.loop();
    }
    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}