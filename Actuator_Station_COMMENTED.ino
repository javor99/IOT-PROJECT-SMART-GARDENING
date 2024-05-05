
#include <ArduinoJson.h>            //Include JSON library

#define Buzzer_Pin  21              // pin 21 for piezo buzzer/ scarecrow

#include "mbedtls/aes.h"            //Include enryption library
#include "Cipher.h"                 //Include enryption library

//Lora
  #include <HardwareSerial.h>       //we're gonna be using the hardware serial for LoRa
  HardwareSerial loraSerial(1);     //using port one
  String received_message_string="";//
  int packageID = 0;                //
  int received_message_integer;                      

  #define RxPin 18                  //LoRa pin for Rx
  #define TxPin 19                  //LoRa pin for Tx
  #define RST   23                  //LoRa pin for Reset

//DeepSleep
  int Sleeping_Time=35000000;       //Sleeping time


//Decryption
char *key = "abcdefghijklmnop";     //Decryption key
Cipher *cipher = new Cipher();   


// Servo
#include <ESP32Servo.h>             //Servo library
Servo valveservo;

// WiFi/ MQTT
#include <WiFi.h>                   //Include WiFi library
#include <PubSubClient.h>           //Incldue MQTT library
const char *ssid = "iPhone (27)";   //WiFi name
const char *password = "12345ab6789";  //WiFi password

// MQTT Broker
const char *mqtt_broker = "test.mosquitto.org"; //Mosquitto as the broker
const char *topic2 = "esp32/actuatorData";      //MQTT topic we are using
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;


int status=0;                         //Variable which is used for switching between LoRa and MQTT

WiFiClient espClient;
PubSubClient client(espClient);

//Variables in which received values are saved
int senID = 0;
int Servo_Value = 0;
int Buzzer_Value = 0;
int UV_Lamp_Value=0;

const int UV_Lamp_Pin = 23;           //UV lamp pin 23


void setup() {
  cipher->setKey(key);
  pinMode(UV_Lamp_Pin, OUTPUT);       //UV lamp pin 23 as output
  valveservo.attach(13);
  Serial.begin(9600);                 //open serial port and set baud rate to 9600 bit/s
  Serial.println("Started Program");
  Lora_Setup();
  Serial.println("Lora setup done");
  valveservo.attach(13); 
}


void loop() {
  client.loop();                        //Check for callbacks
  if(status<3){                         //if less than 3 times tried to receive message with LoRa
    Lora();                             //receiving messages with LoRa
  }
  delay(500);
  if(status==3){                        //if exactly 3 times tried to receive message with LoRa, ...
      WiFi.begin(ssid, password);       //...Connect with WiFi
      status+=1;                        //Increment status

    while (WiFi.status() != WL_CONNECTED) { //Wait for WiFi to be connected
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");
    
    //connecting to mqtt broker
    client.setServer(mqtt_broker, mqtt_port); 
    client.setCallback(callback);
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), "", "")) {
            Serial.println("Public emqx mqtt broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // publish and subscribe
    client.subscribe(topic2);
  }
 delay(2000);
}

void callback(char *topic2, byte *payload, unsigned int length) {
String Received_Json="";

  //Print message arrived in topic
  Serial.println(topic2);
  Serial.print("Message:");

   //Print received payload and save it in variable "Received_Json"
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
    Received_Json+=((char) payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");

  //Decrypt received JSON
  Received_Json = cipher->decryptString(Received_Json);
  StaticJsonDocument<200> doc;

  //Deserialize JSON
  deserializeJson(doc, Received_Json);
  Serial.println(Received_Json);

  //Save received values from JSON in specfic variables
  senID = doc["deviceId"];
  Servo_Value = doc["sprinkler"];
  Buzzer_Value = doc["scarecrow"];
  UV_Lamp_Value = doc["uvlamp"];
  Serial.println(senID);
  Serial.println(Servo_Value);
  Serial.println(Buzzer_Value);
  Serial.println(UV_Lamp_Value);

  
  //Control UV Lamp based on received value
  if(UV_Lamp_Value==1){
    digitalWrite(UV_Lamp_Pin, HIGH);                // Turn on LED/ UV Lamp
    delay(100);                                     // Delay of 0,1 s
    Serial.println("UV_Lamp_Pin, HIGH");
  }
  else if(UV_Lamp_Value==0){
    digitalWrite(UV_Lamp_Pin, LOW);                 // Turn off LED/ UV Lamp
    delay(100);                                     // Delay of 0,1 s
    Serial.println("UV_Lamp_Pin, LOW");
  }
  
  //Control Servo based on received value
  if(Servo_Value==1){
    valveservo.write(0);                            //Rotate Servo/ Valve to 0 degrees
    delay(1000);                                    //Delay of 1 s
    Serial.println("Servo, true");
  }
  else if(Servo_Value==0){
    valveservo.write(90);                           //Rotate Servo/ Valve to 90 degrees
    delay(100);                                     //Delay of 0,1 s
    Serial.println("Servo, false");
  }
  
  //Control Buzzer based on received value
  if(Buzzer_Value==1){
    tone(Buzzer_Pin, 659, 1000);                    //Play tone for 1 second
    delay(1000);                                    // Delay of 1 s
    Serial.println("buzzer, true");
  }
  else if(Buzzer_Value==0){
    noTone(Buzzer_Pin);                             //Turn buzzer tone off
    delay(1000);                                    // Delay of 1 s
    Serial.println("buzzer, false");
  }
  
  status=0;                                         // Reset status
  delay(500);
  Serial.println("Commands finished, going to sleep.......");
  start_DeepSleep();                                //Starting DeepSleep
}

void Lora_Setup() {
  //Reset LoRa module
  digitalWrite(RST, LOW);
  delay(200);
  digitalWrite(RST, HIGH);

  //Initialize serial communication with RN2483
  loraSerial.begin(9600);  
  loraSerial.setTimeout(1000);

  Serial.println("Initializing LoRa");

  //Send command to get firmware version
  loraSerial.println("sys get ver");
  delay(500);
  //Read response from LoRa module
  received_message_string = loraSerial.readStringUntil('\n');
  delay(500);
  Serial.println(received_message_string);
  Serial.println("hey");
  
  loraSerial.println("mac pause");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set LoRa modulation mode to LoRa
  loraSerial.println("radio set mod lora");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set frequency to 865 MHz
  loraSerial.println("radio set freq 865000000");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set transmission power to 14 dBm
  loraSerial.println("radio set pwr 14");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set spreading factor to 12
  loraSerial.println("radio set sf sf12");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set AFC bandwidth to 41.7 kHz
  loraSerial.println("radio set afcbw 41.7");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set receiver bandwidth to 20.8 kHz
  loraSerial.println("radio set rxbw 20.8");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set preamble length to 8
  loraSerial.println("radio set prlen 8");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Enable CRC
  loraSerial.println("radio set crc on");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Disable IQ inversion
  loraSerial.println("radio set iqi off");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set coding rate to 4/5
  loraSerial.println("radio set cr 4/5");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  // Set watchdog timer to 60000 ms
  loraSerial.println("radio set wdt 60000");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set synchronization word to 12
  loraSerial.println("radio set sync 12");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);
  
  //Set bandwidth to 125 kHz
  loraSerial.println("radio set bw 125");
  received_message_string = loraSerial.readStringUntil('\n');
  Serial.println(received_message_string);

  Serial.println("Parameters set");
}



void Lora() {
  Serial.println("starting lora function");
  bool actuators_activated=false;                                     //Initialize a variable to track whether message to control actuators has been received
  Serial.println("waiting for a message");
  loraSerial.println("radio rx 0");                                   //Command to wait for 60 seconds to receive
  received_message_string = loraSerial.readStringUntil('\n');         //Read received message until a newline character
  Serial.println(received_message_string);                            //Print the received message
  Serial.println("Between received_message_string");                
  Serial.println(received_message_string);                        
  delay(20); 

  if ( received_message_string.indexOf("ok") == 0 )                   //Check if the received message starts with "ok"
  {
    received_message_string = String("");                             //Clear the received message string
    while(received_message_string=="")                                //Loop until a non-empty message is received
    {
      received_message_string = loraSerial.readStringUntil('\n');     //Read the received message until newline character
    }
    if ( received_message_string.indexOf("radio_rx") == 0 )           //Check if the received message contains "radio_rx" indicating data reception
    {
      Serial.println("Radio rx");
      Serial.println(received_message_string);                        //Print received data
      
      Serial.println("Sending reply");

      Serial.print("packageID = "); 
      Serial.println(packageID);  
      loraSerial.print("88888888");                                   //Send acknowledgement message
      received_message_string = loraSerial.readStringUntil('\n');     //Read received message
      Serial.println(received_message_string);    



      received_message_integer=received_message_string.toInt();       //Convert the received message to an integer
      if (received_message_string.substring(0) == "1234567800") {     //Check if the specific message for this device is received
        actuators_activated=true;                                     //Enable receival of actuator commands
      }
      if(actuators_activated==true) {
        //Write received commands into integers
        if(received_message_integer%100==21){
          Servo_Value = 1;
        }
        else if(received_message_integer%100==11){
          Servo_Value = 0;
        }

        if(received_message_integer%100==22){
          Buzzer_Value = 1;
        }
        else if(received_message_integer%100==12){
          Buzzer_Value = 0;
        }

        if(received_message_integer%100==23){
          UV_Lamp_Value = 1;
        }
        else if(received_message_integer%100==13){
          UV_Lamp_Value = 0;
        }

        if(received_message_integer==99999919) {
          actuators_activated=false;
          start_DeepSleep();
        }
        if(UV_Lamp_Value==1){
            digitalWrite(UV_Lamp_Pin, HIGH);                  //Setzt den Wert der LED auf HIGH -> LED leuchtet
          delay(100);                               //Verzoegerung von 100ms
          Serial.println("UV_Lamp_Pin, HIGH");
          
        }
        else if(UV_Lamp_Value==0){
          digitalWrite(UV_Lamp_Pin, LOW); 
          delay(100);                               //Verzoegerung von 100ms
          Serial.println("UV_Lamp_Pin, LOW");
        }
        if(Servo_Value==1){
          valveservo.write(0); // Drehe den Servo auf 90 Grad
          delay(100);                               //Verzoegerung von 100ms
          Serial.println("Servo, true");       
        }
        else if(Servo_Value==0){
          valveservo.write(90); // Drehe den Servo auf 90 Grad
          delay(100);                               //Verzoegerung von 100ms
          Serial.println("Servo, false");
        }

        if(Buzzer_Value==1){
          tone(Buzzer_Pin, 659, 1000);
          delay(1000);                             //Verzoegerung von 1000ms
          Serial.println("buzzer, true");
        }
        else if(Buzzer_Value==0){     
          noTone(Buzzer_Pin);
          delay(1000);                              //Verzoegerung von 1000ms
          Serial.println("buzzer, false");
        }
        status=0;
        Serial.println("Commands finished, going to sleep.......");
        start_DeepSleep();

      }
      delay(2000);
      packageID = packageID + 1;
    }
    else
    {
      Serial.println("Received nothing");
      status+=1;
    }
  }
  else
  {
    Serial.println("radio not going into receive mode");
    delay(1000);
    status+=1;
  }
}


void start_DeepSleep() {
  Serial.println("Start sleeping");               
  Serial.flush();                                 //Flush the serial buffer to ensure all data is transmitted
  esp_sleep_enable_timer_wakeup(Sleeping_Time);   //Enable timer wakeup for the specified sleeping time
  esp_deep_sleep_start();                         //Start deep sleep mode
}

