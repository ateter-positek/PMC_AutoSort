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

#define DEBUG_MODE false // Set this to true to enable debug logs

// ***************** LOGGING *****************
//#define MQTT_LOG_TOPIC "your/log/topic"
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




// Function prototypes
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








// MQTT and Ethernet settings
#define MQTT_HOST IPAddress(192, 168, 12, 110)
#define MQTT_PORT 1883
#define MQTT_CHECK_INTERVAL_MS 50
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xA1, 0x11 };


// MQTT client
AsyncMqttClient mqttClient;

// Create a Watchdog

// Define the watchdog timeout (in milliseconds)
#define WATCHDOG_TIMEOUT_MS 20000

// Define the watchdog configuration
watchdog_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT_MS
};

Thread threadWatchdog(osPriorityHigh);




// RTOS thread for MQTT connection management
Thread connectThread;
// RTOS threads for reading inputs
Thread threadDigitalInputs;
Thread threadEncoders;
Thread threadTemperatureSensors;

// Connection status flags
bool connectedEthernet = false;
bool connectedMQTT = false;

// Watchdog status flags
volatile bool flagDigitalInputs = true;
volatile bool flagEncoders = true;
volatile bool flagTemperatureSensors = true;
volatile bool flagMQTTLoop = true;

// Unique Serial Number (USN) - MAC Address
String USN;

// Digital inputs
const int numInputs = 8;
int inputPins[numInputs] = {0, 1, 2, 3, 4, 5, 6, 7};
int inputStates[numInputs] = {0};
unsigned long lastDebounceTime[numInputs] = {0};
#define DEBOUNCE_DELAY 50  // debounce time in milliseconds

// Digital outputs
const int numOutputs = 8;
int outputPins[numOutputs] = {8, 9, 10, 11, 12, 13, 14, 15};
int outputStates[numOutputs] = {0};

// Encoders
const int numEncoders = 2;
long encoderCounts[numEncoders] = {0};
#define ENCODER_UPDATE_COUNTS 100


// Temperature sensors
const int numTemperatures = 3;
float temperatures[numTemperatures] = {0.0};
#define TEMPERATURE_UPDATE_INTERVAL 1000 // Update every 1 seconds

template <size_t N>
void sendLog(LogLevel log_level, const char* message, const char* error_code, Tag (&tags)[N]) {
    // If the log level of this message is greater than the global log level, discard the log
    if (log_level > globalLogLevel) {
        return;
    }

    String topic = "ENDPOINT/" + USN + "/LOGS";
    StaticJsonDocument<1024> doc;
    doc["log_level"] = LOG_LEVEL_STRINGS[log_level];
    doc["device_serial_number"] = USN;
    doc["device_timestamp"] = millis();
    doc["message"] = message;
    JsonArray arr = doc.createNestedArray("tags");
    for (size_t i = 0; i < N; ++i) {
        arr.add(TAG_STRINGS[tags[i]]);
    }
    doc["error_code"] = error_code;

    char payload[1024];
    serializeJson(doc, payload);

    if (mqttClient.connected()) {
        // Publish the MQTT message
        uint16_t packetIdPub = mqttClient.publish(topic.c_str(), 1, false, payload);
        Serial.print("Publishing at QoS 1, packetId: ");
        Serial.println(packetIdPub);
    } else {
        Serial.println("MQTT client not connected");
    }
}


void setup() {

  // Start the serial console
  Serial.begin(115200);
  
  // Wait for the serial console to open
  while (!Serial) {
    // Do nothing
  }


  // Start the watchdog with a timeout of 5 seconds
  // Initialize the watchdog
  if (hal_watchdog_init(&wdt_config) != WATCHDOG_STATUS_OK) {
      // Handle error
      Serial.println("Watchdog status not ok :(");

  }else{
    Serial.println("+++++++++++++++++++  Watchdog STARTED  ++++++++++++++++++++++++++++++");

  }



  Serial.println("Positek Machine Control");

  // Start the Ethernet interface
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    connectedEthernet = false;
  } else {
    // Show the network address
    Serial.print("IP address: ");
    Serial.println(Ethernet.localIP());
    connectedEthernet = true;
  }


  // Get the MAC address
  USN = "";
  for (int i = 0; i < 6; i++) {
    if (i > 0) USN += ":";
    if (mac[i] < 0x10) USN += "0";
    USN += String(mac[i], HEX);
  }
  Serial.print("MAC address (USN): ");
  Serial.println(USN);

  // Set the MQTT server
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  // Register MQTT callback functions
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);


  //Setup the Digital Inputs
  Wire.begin();
  if (Wire.available()) 
  {
      Wire.read();
  }
  
  if (!digital_inputs.init()) 
  {
    Serial.println("Digital input GPIO expander initialization fail!!");
  }
  else 
  {
    Serial.println("Digital input GPIO expander initialization done");
  }

  //Set initial Digital Output configuration
  digital_outputs.setLatch();    //Set over current behavior of all channels to latch mode:
  digital_outputs.setAll(0);  //At startup set all channels to OPEN
  
  Serial.println("Encoders initialization done");
  
  
  // Initialize temperature probes
  temp_probes.tc.begin();  
  temp_probes.enableTC();  // Enables Thermocouples chip select
  Serial.println("Thermocouples enabled");


  // Initialize RS232 serial

  comm_protocols.init();// Set the PMC Communication Protocols to default config
  comm_protocols.rs485Enable(true); // Enable the RS485/RS232 system
  comm_protocols.rs485ModeRS232(true); // Enable the RS232 mode
  comm_protocols.rs485.begin(115200); // Specify baudrate for RS232 communication
  comm_protocols.rs485.receive();  // Start in receive mode
  Serial.println("RS232 initialization complete!");
  
  // Connect to the MQTT server
  connectToMqtt();

  // Start the RTOS threads
  connectThread.start(connectToMqttLoop);
  threadDigitalInputs.start(checkDigitalInputs);
  threadEncoders.start(checkEncoders);
  threadTemperatureSensors.start(checkTemperatureSensors);
  threadWatchdog.start(watchdogThread);



  Serial.println("Boot Complete");
  delay(1000);
  Tag tags1[] = {t_BOOT, t_POST, t_INITIALIZATION, t_SUCCESS};
  sendLog(LOG_POST, "PMC - BOOT COMPLETE", "E101", tags1);
}

void loop() {
//All looping is now threaded on the Mbed OS


}

void watchdogThread() {
  while (true) {
    bool wdKicked = false;
    if (flagDigitalInputs && flagEncoders && flagTemperatureSensors && flagMQTTLoop) {
      hal_watchdog_kick();
      wdKicked = true;
      // Reset the flags
      flagDigitalInputs = false;
      flagEncoders = false;
      flagTemperatureSensors = false;
      flagMQTTLoop = false;
    }

    Serial.println(" ");
    Serial.println("-------------  Watchdog Loop  ------------------ ");
    if (wdKicked){Serial.println("*************** KICKED THE DOG *************");}
    Serial.print("flagDigitalInputs: ");
    Serial.println(flagDigitalInputs);
    Serial.print("flagEncoders: ");
    Serial.println(flagEncoders);
    Serial.print("flagTemperatureSensors: ");
    Serial.println(flagTemperatureSensors);
    Serial.print("flagMQTTLoop: ");
    Serial.println(flagMQTTLoop);
    Serial.println("-------------------------------------------------");
    Serial.println(" ");

    ThisThread::sleep_for(2000ms);
    
  }
}


void mqttCallback(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Convert the payload to a string
  payload[len] = '\0';
  String strPayload = String(payload);

  // Parse the JSON payload
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, strPayload);
  if (error) {
    Serial.println("Failed to parse JSON payload");
    return;
  }

  //Check if topic is REBOOT
  String rebootTopic = "ENDPOINT/" + USN + "/REBOOT";
  if (String(topic) == rebootTopic) {
    // The topic is the reboot topic
    // Reboot the device
    reboot();
  }

  // Check if the topic is a digital output topic
  for (int i = 0; i < numOutputs; i++) {
    String outputTopic = "ENDPOINT/" + USN + "/GPIO/OUTPUT/DIGITAL/" + String(i);
    if (String(topic) == outputTopic) {
      // The topic is a digital output topic
      int state = doc["state"];
      digital_outputs.set(i, state);
      outputStates[i] = state;
      break;
    }
  }

  // Check if the topic is the programmable serial topic
  String serialTopic = "ENDPOINT/" + USN + "/GPIO/PROGRAMMABLE_SERIAL";
  if (String(topic) == serialTopic) {
    // The topic is the programmable serial topic
    String rs232 = doc["RS232"];
    // Disable receive mode before transmission
    comm_protocols.rs485.noReceive();
    comm_protocols.rs485.beginTransmission();
    comm_protocols.rs485.println(rs232);
    comm_protocols.rs485.endTransmission();
    comm_protocols.rs485.receive();
  }
}

void connectToMqttLoop()
{
  while (true)
  {
    if (Ethernet.linkStatus() == LinkON)
    {
      if (!connectedMQTT)
      {
        mqttClient.connect();
      }

      if (!connectedEthernet)
      {
        Serial.println("Ethernet reconnected.");
        connectedEthernet = true;
      }
    }
    else
    {
      if (connectedEthernet)
      {
        Serial.println("Ethernet disconnected");
        connectedEthernet = false;
      }
    }

   flagMQTTLoop = true;
   ThisThread::sleep_for(50ms);
  }
}



void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.print("Connected to MQTT broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(", port: ");
  Serial.println(MQTT_PORT);

  connectedMQTT = true;

  // Subscribe to the digital output topics
  for (int i = 0; i < numOutputs; i++) {
    String outputTopic = "ENDPOINT/" + USN + "/GPIO/OUTPUT/DIGITAL/" + String(i);
    uint16_t packetIdSub = mqttClient.subscribe(outputTopic.c_str(), 2);
    Serial.print("Subscribing at QoS 2, packetId: ");
    Serial.println(packetIdSub);
  }

  // Subscribe to the programmable serial topic
  String serialTopic = "ENDPOINT/" + USN + "/GPIO/PROGRAMMABLE_SERIAL";
  uint16_t packetIdSub = mqttClient.subscribe(serialTopic.c_str(), 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);

  // Subscribe to the reboot topic
  String rebootTopic = "ENDPOINT/" + USN + "/REBOOT";
  packetIdSub = mqttClient.subscribe(rebootTopic.c_str(), 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  (void) reason;

  connectedMQTT = false;

  Serial.println("Disconnected from MQTT.");
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  mqttCallback(topic, payload, properties, len, index, total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged - PacketId: ");
  Serial.println(packetId);
}


void checkDigitalInputs() {
  while (true) {
    uint32_t inputs = digital_inputs.readAll();
    for (int i = 0; i < numInputs; i++) {
      int reading = (inputs & (1 << i)) >> i;
      flagDigitalInputs = true;
      if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state if the state has actually changed 
        if (inputStates[i] != reading) {
          inputStates[i] = reading;
          
          Serial.print("SEND READING: $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  ");
          Serial.print(reading);
          Serial.println("  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
          // Create the MQTT topic
          String topic = "ENDPOINT/" + USN + "/GPIO/INPUT/DIGITAL/" + String(i);

          // Create the JSON payload
          StaticJsonDocument<200> doc;
          doc["state"] = inputStates[i];
          char payload[200];
          serializeJson(doc, payload);

          // Publish the MQTT message
          uint16_t packetIdPub = mqttClient.publish(topic.c_str(), 1, false, payload);
        }
        flagDigitalInputs = true;
      }

      }
    }
    
    flagDigitalInputs = true;
    ThisThread::sleep_for(50ms);
  }
void checkEncoders() {
   while (true) {
  for (int i = 0; i < numEncoders; i++) { 
    long count = encoders[i].getCurrentState();
    if (abs(count - encoderCounts[i]) >= ENCODER_UPDATE_COUNTS) {
      // The count has changed by the specified amount
      encoderCounts[i] = count;

      // Create the MQTT topic
      String topic = "ENDPOINT/" + USN + "/GPIO/INPUT/ENCODER/" + String(i + 1);

      // Create the JSON payload
      StaticJsonDocument<200> doc;
      doc["count"] = count;
      char payload[200];
      serializeJson(doc, payload);

      // Publish the MQTT message
      uint16_t packetIdPub = mqttClient.publish(topic.c_str(), 1, false, payload);
     // Serial.print("Publishing at QoS 1, packetId: ");
     // Serial.println(packetIdPub);
     // Serial.println(topic);
     //   Serial.println(payload);
    }
  }
    flagEncoders = true;
    ThisThread::sleep_for(100ms);
    
  }
}

void checkTemperatureSensors() {
  while (true) {
    for (int i = 0; i < numTemperatures; i++) {
      // Set the channel, has internal 150 ms delay
      temp_probes.selectChannel(i);
      // Take the measurement
      float newTemperature = temp_probes.tc.readTemperature(PROBE_K);

      // Check if the temperature has changed or if it's the first null reading
     if ((newTemperature != temperatures[i]) || !(isnan(newTemperature) && isnan(temperatures[i])))
      {
        temperatures[i] = newTemperature;    
         // Create the MQTT topic
        String topic = "ENDPOINT/" + USN + "/GPIO/INPUT/TEMPERATURE/" + String(i + 1);

        // Create the JSON payload
        StaticJsonDocument<200> doc;
        if (isnan(newTemperature)) {
          doc["temperature"] = "-.-- °C";

        } else {
          char tempStr[8];
          sprintf(tempStr, "%.2f °C", newTemperature); // Convert float to string, rounded to 2 decimal places
          doc["temperature"] = String(tempStr);
     
        }
        char payload[200];
        serializeJson(doc, payload);

        // Publish the MQTT message
        uint16_t packetIdPub = mqttClient.publish(topic.c_str(), 1, false, payload);
      //  Serial.print("Publishing at QoS 1, packetId: ");
      //  Serial.println(packetIdPub);
      //  Serial.println(topic);
      //  Serial.println(payload);
      }
      else
      {
        temperatures[i] = newTemperature; 
      }
    }
    flagTemperatureSensors = true;
    ThisThread::sleep_for(500ms);
  }
}

void reboot() {
  NVIC_SystemReset();
} 

