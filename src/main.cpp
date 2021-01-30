#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "environment.pb.h"

#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"

// Sensor queue
static QueueHandle_t s_sensorDataQueue;
// Actuator queue
static QueueHandle_t s_actuatorDataQueue;

static bool s_alarm = false;

DHT dht(TEMPERATURE_SENSOR_PIN, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);

static void wifiConnect()
{
  WiFi.begin(SID_WIFI, PIO_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print (F("Connecting to WiFi... status "));
    Serial.println(WiFi.status());
  }

  Serial.println(F("Connected to the WiFi network"));
  Serial.print(F("IP Address: "));
  Serial.println(WiFi.localIP());
}


void mqttConnect() {
  client.setServer(BROKER_IP, BROKER_PORT);
  while (!client.connected()) {
    Serial.print(F("MQTT connecting ... "));
    Serial.print(BROKER_IP);

    if (client.connect("ESP32Client1")) {
      Serial.println(F("connected"));
    } else {
      Serial.print(F("failed, status code ="));
      Serial.print(client.state());
      Serial.println(F("try again in 5 seconds"));

      delay(5000);  //* Wait 5 seconds before retrying
    }
  }
}

/**
 * @brief Alarm Button Handler
 * 
 */
static void IRAM_ATTR alarm_button_handler()
{
  s_alarm = true;
}


static void send_environment_mqtt(EnvironmentTopicMsg msg) {
  Serial.println(F("Environment change"));
  Serial.print(F("AirQuality"));
  Serial.println(msg.airQuality);
  Serial.print(F("Humidity "));
  Serial.println(msg.humidity);
  Serial.print(F("LightLevel "));
  Serial.println(msg.lightLevel);
  Serial.print(F("Temperature "));
  Serial.println(msg.temperature);
  uint8_t buffer[500];

  Serial.println(F("1"));
  environmentMessage message = environmentMessage_init_zero;
Serial.println(F("2"));
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
Serial.println(F("3"));
  message.airQuality = 100;
  message.has_airQuality = true;
  message.lightLevel = 1000;
  message.has_lightLevel = true;
  Serial.println(F("4"));
  //message.f = 0.5;
  //strcpy(message.message, F("Hello Protobuf!"));
 // message.op = 0x48656c6c;
  Serial.println(F("5"));
  bool status = pb_encode(&stream, environmentMessage_fields, &message);
Serial.println(F("6"));
  if (!status)
  {
      Serial.println(F("Failed to encode"));
      return;
  }
  Serial.println(F("7"));
  client.publish(ENVIRONMENT_TOPIC, buffer, stream.bytes_written);
  Serial.println(F("8"));
}

/**
 * @brief Main Task Handler
 * 
 */
static void main_task_handler(void *pvParameters)
{

  int presence = LOW;
  bool envChange = false;
  bool presenceChange = false;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    SensorDataMsg currentPinRead;
    vTaskDelayUntil(&xLastWakeTime, 50 / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
    // Se lee de la cola de sensores
    EnvironmentTopicMsg envMsg;
    if (xQueueReceive(s_sensorDataQueue, &currentPinRead, portMAX_DELAY) == pdPASS)
    {
      // En función del tipo de msg
      switch (currentPinRead.pin)
      {
      case PRESENCE_PIN:
        presence = currentPinRead.value;
        presenceChange = true;
        break;
      case TEMPERATURE_SENSOR_PIN:
        envChange = true;
        envMsg.temperature = currentPinRead.value / 100;
        break;
      case MQ_SENSOR_PIN:
        envChange = true;
        envMsg.airQuality = currentPinRead.value;
        break;
      case LDR_PIN:
        envChange = true;
        envMsg.lightLevel = currentPinRead.value;
        break;
      default:
        break;
      }
    }

  
    // TODO Si han variado una o más condiciones se
    // crea el mensaje para topic de salida
    if (presenceChange)
    {
      presenceChange = false;
      Serial.print(F("PRESENCE "));
      Serial.println(presence);
    }

    if (envChange)
    {
      envChange = false;
      Serial.println(F("Sending mqtt"));
      send_environment_mqtt(envMsg);
      
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
  for (;;)
  {
    vTaskDelay(xDelay);
    // Tomamos la temperatura
    float currentTemperature = dht.readTemperature();
    if (!isnan(currentTemperature) && (currentTemperature - lastTemperature) > TEMPERATURE_THRESHOLD)
    {
      lastTemperature = currentTemperature;
      // Mandamos el mensaje
      Serial.print(F("Temp"));
      Serial.println(currentTemperature);
      send_sensor_msg(TEMPERATURE_SENSOR_PIN, (int)(currentTemperature * 100));
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
    if (firstExecution || (actualAirMeasure > AIR_QUALITY_THRESHOLD && lastAirMeasure <= AIR_QUALITY_THRESHOLD))
    {

      firstExecution = false;
      lastAirMeasure = actualAirMeasure;

      // Mandamos el mensaje
      send_sensor_msg(MQ_SENSOR_PIN, actualAirMeasure);
    }
    else if (actualAirMeasure <= AIR_QUALITY_THRESHOLD && lastAirMeasure > AIR_QUALITY_THRESHOLD)
    {
      // Si el valor leido es menor que el limite y el anterior es mayor que el limite,
      // mandamos el mensaje para avisar de un cambio de estado
      lastAirMeasure = actualAirMeasure;

      // Mandamos el mensaje
      send_sensor_msg(MQ_SENSOR_PIN, actualAirMeasure);
    }
    else if (actualAirMeasure <= AIR_QUALITY_THRESHOLD)
    {
      // Siempre que el valor sea menor que el limite lo actualizamos
      lastAirMeasure = actualAirMeasure;
    }
  }
  vTaskDelete(NULL);
}

// /**
//  * @brief Presence task handler. 
//  * 
//  */
// static void presence_task_handler(void *pvParameters)
// {
//   int presence = LOW;
//   for (;;)
//   {
//     int newPresence = digitalRead(PRESENCE_PIN);
//     Serial.print(F("newPresence "));
//     Serial.println(newPresence);
//     if(presence != newPresence) {
//       presence = newPresence;
//       send_sensor_msg(PRESENCE_PIN, presence);
//     }
//     vTaskDelay(PRESENCE_READ_PERIOD / portTICK_RATE_MS);
//   }
//   vTaskDelete(NULL);
// }

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

  for (;;)
  {
    vTaskDelay(xDelay);
    // Tomamos la medida de la cantidad de luz
    int actualLightQuantity = analogRead(LDR_PIN);
    // actualLightQuantity = ((long)actualLightQuantity * R_DARKNESS * 10) / ((long)R_LIGHT * R_CALIBRATION * (1024 - actualLightQuantity));
    // Si es la primera ejecucion o la diferencia es mayor al limite
    if (abs(actualLightQuantity - lastLightQuantity) > LIGHT_QUANTITY_THRESHOLD)
    {
      lastLightQuantity = actualLightQuantity;
      // Mandamos el mensaje
      send_sensor_msg(LDR_PIN, actualLightQuantity);
    }
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

static void IRAM_ATTR pir_interrupt_handler() {
  Serial.println(F("PIR INTERRUPT"));
  send_sensor_msg(PRESENCE_PIN, HIGH);
}

void setup()
{
  delay(2000);

  Serial.begin(115200);
  // pinMode(PRESENCE_PIN, INPUT);
  
  pinMode(ALARM_BUTTON_PIN, INPUT);

  // Sensor initialization
  dht.begin();

  delay(1000);
  #ifdef PIO_WIFI
    wifiConnect();
  #endif
  mqttConnect();

  s_sensorDataQueue = xQueueCreate(10, sizeof(SensorDataMsg));
  s_actuatorDataQueue = xQueueCreate(10, sizeof(ActuatorDataMsg));
  attachInterrupt(digitalPinToInterrupt(ALARM_BUTTON_PIN), &alarm_button_handler, FALLING);
  attachInterrupt(digitalPinToInterrupt(PRESENCE_PIN), &pir_interrupt_handler,RISING);

  xTaskCreatePinnedToCore(main_task_handler, "mainTask", 2048, NULL, 5, NULL, 0);
  // xTaskCreatePinnedToCore(presence_task_handler, "presenceTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(temperature_task_handler, "temperatureTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(air_quality_task_handler, "airQualityTask", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(light_quantity_task_handler, "lightQuantityTask", 1024, NULL, 3, NULL, 1);

  // Tarea para envío de mensajes a mqtt entorno
  // Tarea para envío de mensajes a mqtt presencia
  // Tarea para envío de mensajes a mqtt emergencia
  // Tarea para recibir de mensajes a mqtt acción
}

void loop()
{
  // Do Nothing
  // TODO 
}
