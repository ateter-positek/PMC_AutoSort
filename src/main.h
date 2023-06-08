#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Arduino_MachineControl.h>
#include <AsyncMqtt_Generic.h>
#include <PortentaEthernet.h>
#include <ArduinoJson.h>
#include "mbed.h"
#include <chrono>
#include "Wire.h"
#include <math.h>

using namespace std::chrono;
using namespace machinecontrol;
using namespace rtos;

#define DEBUG_MODE false

#define MQTT_HOST IPAddress(192, 168, 12, 110)
#define MQTT_PORT 1883
#define MQTT_CHECK_INTERVAL_MS 50
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xA1, 0x11 };
#define WATCHDOG_TIMEOUT_MS 20000

#define DEBOUNCE_DELAY 50
#define ENCODER_UPDATE_COUNTS 100
#define TEMPERATURE_UPDATE_INTERVAL 1000

// Define an enum for your log levels
enum LogLevel {
    LOG_POST = 0,
    LOG_ERROR = 1,
    LOG_WARNING = 2,
    LOG_STATUS = 3,
    LOG_INFORMATIONAL = 4,
    LOG_DEBUG = 5
};

// Set the default log level
LogLevel globalLogLevel = LOG_STATUS;

// Create a lookup table of string representations for the log levels
const char* LOG_LEVEL_STRINGS[] = {
    "POST",
    "Error",
    "Warning",
    "Status",
    "Informational",
    "Debug"
};

// Define an enum for your tags
enum Tag {
    t_SENSOR,
    t_INITIALIZATION,
    t_ERROR,
    t_BOOT,
    t_POST,
    t_MQTT,
    t_DIGITAL_INPUTS,
    t_ENCODERS,
    t_TEMPERATURE_SENSORS,
    t_DIGITAL_OUTPUTS,
    t_RS232,
    t_RS485,
    t_SUCCESS,
    t_FAILURE,
    t_WARNING,
    t_STATUS,
    t_INFORMATIONAL,
    t_DEBUG,
    t_DISCONNECTED,
    t_CONNECTED,
    t_ETHERNET,
    // add more tags here...
    t_COUNT  // this should always be the last item in the enum
};

// Create a lookup table of string representations for the enum values
const char* TAG_STRINGS[t_COUNT] = {
    "SENSOR",
    "INITIALIZATION",
    "ERROR",
    "BOOT",
    "POST",
    "MQTT",
    "DIGITAL_INPUTS",
    "ENCODERS",
    "TEMPERATURE_SENSORS",
    "DIGITAL_OUTPUTS",
    "RS232",
    "RS485",
    "SUCCESS",
    "FAILURE",
    "WARNING",
    "STATUS",
    "INFORMATIONAL",
    "DEBUG",
    "DISCONNECTED",
    "CONNECTED",
    "ETHERNET"
    // add more tag strings here...
};

extern LogLevel globalLogLevel;
extern AsyncMqttClient mqttClient;
extern watchdog_config_t wdt_config;
extern Thread threadWatchdog;
extern Thread connectThread;
extern Thread threadDigitalInputs;
extern Thread threadEncoders;
extern Thread threadTemperatureSensors;
extern bool connectedEthernet;
extern bool connectedMQTT;
extern volatile bool flagDigitalInputs;
extern volatile bool flagEncoders;
extern volatile bool flagTemperatureSensors;
extern volatile bool flagMQTTLoop;
extern String USN;
extern const int numInputs;
extern int inputPins[];
extern int inputStates[];
extern unsigned long lastDebounceTime[];
extern const int numOutputs;
extern int outputPins[];
extern int outputStates[];
extern const int numEncoders;
extern long encoderCounts[];
extern const int numTemperatures;
extern float temperatures[];

void watchdogThread();
void mqttCallback(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void connectToMqttLoop();
void connectToMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);
void checkDigitalInputs();
void checkEncoders();
void checkTemperatureSensors();
void reboot();

#endif //MAIN_H
