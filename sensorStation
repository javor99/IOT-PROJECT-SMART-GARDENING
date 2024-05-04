#include <WiFi.h>           // WiFi library for connecting to wireless networks
#include <PubSubClient.h>   // PubSubClient library for MQTT communication
#include "Cipher.h"         // Cipher library for data encryption
#include <HardwareSerial.h> // HardwareSerial library for serial communication

// Initialize hardware serial for LoRa communication on Serial port 1
HardwareSerial loraSerial(1); 

// LoRa communication pin definitions
#define RxPin 18
#define TxPin 19
#define RST   23

int Lora_loop = 0;
int status = 0;
int packageID = 0;

// WiFi credentials
const char *ssid = "iPhone-Kacper";
const char *password = "12345678";

// Encryption key and Cipher object for data encryption
char *key = "abcdefghijklmnop";
Cipher *cipher = new Cipher();

// Variables to hold sensor readings
String str = "";
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int waterLevelValue = 0;
int PIRValue = 0;

// Variables for LoRa sensor values
int LorasensorValue1 = 0;
int LorasensorValue2 = 0;
int LorasensorValue3 = 0;
int LorawaterLevelValue = 0;
int LoraMotionValue = 0;

const char *motionStatus;

// MQTT settings
const char *mqtt_broker = "test.mosquitto.org";
const char *topic1 = "esp32/sensorData";
const char *topic2 = "project/startSensor";
const char *topic3 = "esp32/sensor_sleep";
const int mqtt_port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

// Device settings
const char *deviceId = "12345678";
bool enableSensing = false;
int PIRPin = 2;
unsigned long TIME_TO_SLEEP = 35;

// Variables for LoRa bandwidth management
const int Imax = 50;           // Define Imax as needed
unsigned long onVal[Imax] = {0}; // Array to track LoRa duty cycle
unsigned long offVal[Imax] = {50};
unsigned long stopSend;
unsigned long startSend;
int currentI = 0;
#define BWthresh 100

// Arduino setup function, initializes serial communication and LoRa
void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud rate
  setupLora();           // Setup LoRa communication
  pinMode(PIRPin, INPUT); // Set PIR sensor pin as input
}

// Function to setup LoRa communication with various configurations
void setupLora() {
  Serial.begin(115200);  // Start serial communication to the PC
  Serial.println("Test");

  // Initialize LoRa serial communication with specified settings
  loraSerial.begin(9600, SERIAL_8N1, RxPin, TxPin);

  // Reset the LoRa module
  digitalWrite(RST, LOW);
  delay(200);
  digitalWrite(RST, HIGH);

  // Set LoRa serial communication timeout
  loraSerial.setTimeout(1000);
  Serial.println("Initing LoRa");

  // Send commands to configure the LoRa module
  loraSerial.println("sys get ver");
  delay(500);
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  // Pause the LoRa MAC layer
  loraSerial.println("mac pause");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  // Set LoRa communication parameters
  loraSerial.println("radio set mod lora");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set freq 865000000");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set pwr 14");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set sf sf12");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set afcbw 41.7");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set rxbw 20.8");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set prlen 8");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set crc on");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set iqi off");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set cr 4/5");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set wdt 60000");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set sync 12");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  loraSerial.println("radio set bw 125");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  Serial.println("starting loop");
  stopSend = millis();  // Initialize stopSend
}

// Function to check if there's bandwidth available to send via LoRa
int checkLora() {
  int sending = 0;
  int noting = 0;
  for (int i = 0; i < Imax; i++) {
    sending += onVal[i];
    noting += offVal[i];
  }
  sending = sending * BWthresh;
  if (sending < noting) {
    return 1;
  } else {
    return 0;
  }
}

// Main loop function, manages sensor data and communication
void loop() {
  client.loop();  // Process MQTT messages

  if (enableSensing) {
    measureSensorData();  // Measure sensor data
    if (status < 3) {
      startSend = millis();
      offVal[currentI] = startSend - stopSend;
      if (checkLora()) {
        lora_sending(LorasensorValue1);
        lora_sending(LorasensorValue2);
        lora_sending(LorasensorValue3);
        lora_sending(LorawaterLevelValue);
        lora_sending(LoraMotionValue);
        stopSend = millis();
        onVal[currentI] = stopSend - startSend;
        currentI++;
        if (currentI == Imax) {
          currentI = 0;
        }
      }
    } else {
      publishSensorData();  // Publish sensor data to MQTT broker
    }
  } else {
    if (status < 3) {
      lora_listening();  // Listen for LoRa commands
    }
    if (status == 3) {
      setupWiFi();   // Connect to WiFi
      status += 1;
      setupMQTT();   // Setup MQTT broker connection
    }
  }
}

// Function to connect to WiFi network
void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}

// Function to setup MQTT broker connection
void setupMQTT() {
  client.setServer(mqtt_broker, mqtt_port); // Set MQTT broker address and port
  client.setCallback(mqttCallback);         // Set MQTT message callback function

  // Loop until connected to MQTT broker
  while (!client.connected()) {
    String client_id = "esp32-client-" + String(WiFi.macAddress());
    if (client.connect(client_id.c_str())) {
      Serial.println("Connected to MQTT broker");
      client.subscribe(topic3);  // Subscribe to topic for sleeping command
    } else {
      Serial.print("Failed to connect to MQTT, state: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// Callback function to handle incoming MQTT messages
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  // Convert payload to string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // Check if message is for sleeping
  if (message == "sleep") {
    startDeepSleep();
  }

  // Enable sensing if the message matches the device ID and "sensor" keyword
  if (message.startsWith(String(deviceId) + " sensor")) {
    Serial.println("enabled Sensing");
    enableSensing = true;
  }
}

// Function to measure sensor data and update relevant variables
void measureSensorData() {
  sensorValue1 = analogRead(32);   // Read analog value from pin 32
  sensorValue2 = analogRead(33);   // Read analog value from pin 33
  sensorValue3 = analogRead(34);   // Read analog value from pin 34
  waterLevelValue = analogRead(35); // Read analog value from pin 35
  PIRValue = digitalRead(PIRPin);  // Read digital value from PIR pin

  // Update LoRa sensor values based on sensor readings
  LorasensorValue1 = sensorValue1 * 100 + 1;
  LorasensorValue2 = sensorValue2 * 100 + 2;
  LorasensorValue3 = sensorValue3 * 100 + 3;
  LorawaterLevelValue = waterLevelValue * 100 + 4;
  LoraMotionValue = PIRValue * 100 + 5;

  // Update motion status
  motionStatus = PIRValue == HIGH ? "0" : "1";
  Serial.println("data measured");
}

// Function to publish sensor data to MQTT broker
void publishSensorData() {
  char sensorData[200];
  snprintf(sensorData, sizeof(sensorData), "{\"deviceId\":\"%s\",\"sensor1\":%d,\"sensor2\":%d,\"sensor3\":%d,\"waterLevel\":%d,\"motion\":\"%s\"}", deviceId, sensorValue1, sensorValue2, sensorValue3, waterLevelValue, motionStatus);
  String encryptedPayload = cipher->encryptString(sensorData);
  client.publish(topic1, encryptedPayload.c_str());
  Serial.println("data published");
}

// Function to listen for LoRa commands
void lora_listening() {
  Serial.println("waiting for command");

  loraSerial.println("radio rx 0"); // Wait for 60 seconds to receive
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  delay(20);
  if (str.indexOf("ok") == 0) {
    str = String("");
    while (str == "") {
      str = loraSerial.readStringUntil('\n');
    }

    // Check if data was received
    if (str.indexOf("radio_rx") == 0) {
      Serial.println("Radio rx");

      if (str == "radio tx received") {
        Serial.println(str); // Print received data
      }

      if (str == "7654321") {
        enableSensing = true;
      }
    } else {
      Serial.println("Received nothing");
      status += 1;
    }
  } else {
    Serial.println("radio not going into receive mode");
    status += 1;
    delay(1000);
  }
}

// Function to send data via LoRa
void lora_sending(int measurement_value) {
  Serial.print("packageID = ");
  Serial.println(packageID);

  // Send measurement value via LoRa
  loraSerial.print("radio tx ");
  loraSerial.println(measurement_value);
  str = loraSerial readStringUntil('\n');
  Serial.println(str);
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  delay(2000);
  packageID = packageID + 1;

  Serial.println("waiting vor reply");

  loraSerial.println("radio rx 0"); // Wait for 60 seconds to receive
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);

  delay(20);
  if (str.indexOf("ok") == 0) {
    str = String("");
    while (str == "") {
      str = loraSerial.readStringUntil('\n');
    }

    // Check if data was received
    if (str.indexOf("radio_rx") == 0) {
      Serial.println("Radio rx");

      if (str == "radio tx received") {
        Serial.println(str); // Print received data
      }

      // Go to deep sleep mode if receiving specific code
      if (str == "99999909") {
        startDeepSleep();
      }
    } else {
      Serial.println("Received nothing");
      status += 1;
    }
  } else {
    Serial.println("radio not going into receive mode");
    status += 1;
    delay(1000);
  }
}

// Function to autobaud the LoRa communication
void lora_autobaud() {
  String response = "";
  while (response == "") {
    delay(1000);
    loraSerial.write((byte)0x00);
    loraSerial.write(0x55);
    loraSerial.println();
    loraSerial.println("sys get ver");
    response = loraSerial.readStringUntil('\n');
  }
}

// Function to start deep sleep mode for power saving
void startDeepSleep() {
  Serial.println("Going to sleep...");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000);
  esp_deep_sleep_start();
}
