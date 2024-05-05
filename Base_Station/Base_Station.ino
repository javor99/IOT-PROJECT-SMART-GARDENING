//All of our wifi includes
  #include <WiFi.h>
  #include <DNSServer.h>
  #include <WebServer.h>
  #include <WiFiManager.h> // For WiFi configuration
  #include <PubSubClient.h> //For MQTT
  #include <OpenWeatherOneCall.h> //For the weather
  #include <WifiLocation.h> //Also for the weather
  //ALL WEATHER RELATED CODE IN THE FOLLOWING DOCUMENT IS INSPIRED BY THE OPENWEATHEONECALL LIBRARY AND IT'S EXAMPLES
  #include <string>


//Encryption includes
  #include <ArduinoJson.h> //Important
  #include "mbedtls/aes.h" //Important
  #include "Cipher.h"      //Important

//Hardware for Lora
  #include <HardwareSerial.h>   //we're gonna be using the hardware serial because it is faster and better I think
  HardwareSerial loraSerial(1);   //using port one, we could use whatever
  String str="";
  int packageID = 0;
  int integer;
  int sensor_feedback = 0;
  int sensor_sleep_counter = 0;
  int actuator_sleep_counter = 0;

//Defines for lora pins, we need just rx, tx and the reset
  #define RxPin 18
  #define TxPin 19
  #define RST   23

  int Lora_loop = 0;

//Defines for the location services
//had values here but removed them so people can't steal our values

//Threshold Defines including lora duty cycle, but these should be lora 
  #define lightSenT 61 //change this based on the value we recieve from photo resistor
  #define waterForT 75 //change this based on the value of the rain for future 
  #define lightForT 150 //change this based on the value of the sun for future
  #define Imax 10
  //the BWthresh is the threshold of how much time we can have not sending to sending, should be 100 generally
  #define interval 30000

//Globl variables
  //these are for the sensor values
  float sunFor; //for if we dont want to turn on the uv lamp? Called cloudCover
  float precipFor; //for if it will rain and we don't need to turn on the water. Called rainVolume
  //unsure how I should set this up, if I want an array or if I should just hardcode stuff to the values or sensors
  //I think hardcoding is fine given we only have three but IDK
  int senWater; //also out of 4096
  int senLight; //out of 4096
  int senMove; //out of 4096
  bool senMotion; //this is wether or not we recieve motion from the sensor
  bool catLocation; //A boolean that we set for the location of the cat, wether or not we should cancel scarecrow
  //these are for the duty cycle restrictions
  int currentI = 0;
  unsigned long onVal[Imax] = {0}; //the following four variables and arrays are for keeping track of the lora duty cycle
  unsigned long offVal[Imax] = {50};
  unsigned long stopSend;
  unsigned long startSend;
  #define BWthresh 100
  
  unsigned long weatherTime;
  unsigned long lastReset;
  unsigned long previousMillis;

  int uvlamp_int=0;
  int motor_int=0;
  int scarecrow_int=0;
  int status=0;
  //different booleans to store if we currently want to turn on the motor(water), the scarecrow(motion), or the uvlamp
  bool motor = false;
  bool scarecrow = false;
  bool uvlamp = false;
  bool catData = true;
  //the number of devices we currently have in our system, the max amount of devices we can hold is ten
  int numDevices = 0;
  //two arrays that store the device IDS and the humidity desried for each device
  int deviceIDS[10] = {0};
  int humidityS[10] = {50};
  //where we are in sending the data in the loop, as in if we are sending to device 1 or device 2, and so on
  int idCurrent = 0;

//ENCRYPTION
  char *key = "abcdefghijklmnop"; //Important
  Cipher *cipher = new Cipher();  //Important //UNCOMMENT LINE FOR THE REAL THING
//when sending a string cipher->encryptString(JSONmessage)
//cipher->decryptString(JSONmessage)
// MQTT Broker settings
  const char* mqtt_server = "test.mosquitto.org";
  const int mqtt_port = 1883; // Default MQTT TCP port number
  const char* mqtt_topic = "esp32/sensorData";
  const char* mqtt_sensor_sleep = "esp32/sensor_sleep";
  const char* mqtt_to_publish = "esp32/actuatorData";
  const char* mqtt_cat = "esp32/bluetoothData/coordinates"; //add the specific hande here
  const char* mqtt_newDevice = "project/newDevice";
  const char* mqtt_humid = "project/newHumidity";
  //probably will need a inTopic and outTopic
  //and then we also want someting to store the message in
  char* message;
  char* incoming_byte;
  const char* deviceId = "12345678";
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);
  const char *ssid = "iPhone (27)"; // Enter your WiFi name
  const char *password = "12345ab6789";  // Enter WiFi password

//Weather settings
  int myUNITS = METRIC;          //<-----METRIC, IMPERIAL, KELVIN (IMPERIAL is default)

  //Can't get current and historical at the same time
  int myHISTORY = NULL;            //<-----Only required for historical data up to 5 days

  //See manual for excludes, only CURRENT Data allows 1,000,000 calls a month
  int myEXCLUDES = EXCL_C + EXCL_H + EXCL_M + EXCL_A;              //<-----0 Excludes is default
  // int myEXCLUDES = 0;
  //for debugging loop counting
  int nextCall = 0;
  //int callAttempt = 1;

//Weather initialization
  OpenWeatherOneCall OWOC;              
  WiFiManager wifiManager;

  WifiLocation location (GOOGLEKEY);

void setup() {
      
    Serial.begin(57600);  // Serial communication to PC
    delay(1); 
    Serial.println("Test"); //for testing purposes
    
  // Next Setup (wifi)

  //toggle the reset before we setup lora
  digitalWrite(RST, LOW);
  delay(200);
  digitalWrite(RST, HIGH);
  
  // loraSerial.begin(9600);  // Serial communication to RN2483
  loraSerial.setTimeout(1000);
  //lora_autobaud();
  
  Serial.println("Initing LoRa");
 
  //loraSerial.listen();
  //the following are all of our setting for the lora communication
  loraSerial.println("sys get ver");
  delay(500);
  str = loraSerial.readStringUntil('\n');
  delay(500);
  Serial.println(str);
  Serial.println("hey");
  
  loraSerial.println("mac pause");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  
//  loraSerial.println("radio set bt 0.5");
//  wait_for_ok();
  
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
  
  loraSerial.println("radio set rxbw 20.8");  // Receiver bandwidth can be adjusted here. Lower BW equals better link budget / SNR (less noise). 
  str = loraSerial.readStringUntil('\n');   // However, the system becomes more sensitive to frequency drift (due to temp) and PPM crystal inaccuracy. 
  Serial.println(str);
  
//  loraSerial.println("radio set bitrate 50000");
//  wait_for_ok();
  
//  loraSerial.println("radio set fdev 25000");
//  wait_for_ok();  
  
  loraSerial.println("radio set prlen 8");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  
  loraSerial.println("radio set crc on");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  
  loraSerial.println("radio set iqi off");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  
  loraSerial.println("radio set cr 4/5"); // Maximum reliability is 4/8 ~ overhead ratio of 2.0
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  
  loraSerial.println("radio set wdt 60000"); //disable for continuous reception
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  
  loraSerial.println("radio set sync 12");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  
  loraSerial.println("radio set bw 125");
  str = loraSerial.readStringUntil('\n');
  Serial.println(str);
  
  //now that the lora is set up we want to connect to the mqtt
  MQTT_Setup();

  // Weather location setup

    //get the time data
    setClock();
    //get the location
    location_t loc = location.getGeoFromWiFi();

    OWOC.setOpenWeatherKey(ONECALLKEY);

    OWOC.setLatLon(loc.lat, loc.lon);

    // Third set any EXCLUDES if required (Here we are not using any
    OWOC.setExcl(myEXCLUDES);

    // Set History if you want historical weather other wise NULL
    OWOC.setHistory(myHISTORY);

    // Set UNITS of MEASURE otherwise default is IMPERIAL
    OWOC.setUnits(METRIC);

    //Now call the weather. Please notice no arguments are required in this call
    OWOC.parseWeather();
    weatherTime = millis();

    //Now display some information, note the pointer requirement for current and alert, this is NEW for v3.0.0

    // Location info is available for ALL modes (History/Current)

    lastReset = millis();
    previousMillis = lastReset;
    stopsend = lastReset;
}

void loop() {
  // Our actual loop

  //get the weather data every two hours
    //currently set to get the weather every hour, not necesarily on the hour but once an hour
    if (millis() - weatherTime > 7200000) {
      //every two hours this is what we do, updating the weather 
      getWeather(); //get the weather value
      weatherTime = millis(); //a new time for when we reset the weather data

    }
    unsigned long current = millis(); 
    //get the current millis time to know if we should reset or go to the next device
    if (current - lastReset > (interval * 10) ) {
      //so if we want to reset back to the first device
      lastReset = current;
      idCurrent = 0;
      Serial.println("Reseting the timer and ID here");
    }
    //if we want to move on to the next device
    if (current - previousMillis >= interval) {
      //set the last time we were in this to the current time
      previousMillis = current;
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Attempting to reconnect to wifi");
      }
      
      if ((deviceIDS[idCurrent] != 0) && (idCurrent % 2 == 0)) {
        //so if we have a sensor that we want to activate and receive data from
        char payload[150];
        //send the sensor station the current ID
        snprintf(payload, sizeof(payload), "%d sensor", deviceIDS[idCurrent]);
        
        //publish to the correspondign topic for the actuator station
        mqttClient.publish(mqtt_sensor_sleep, payload);
        
        //check if we can send to lora
        startSend = millis();
        offVal[currentI] = startSend - stopSend;
        if (checkLora()){
          //we know we can send to lora
          //send a wakeup call over lora
          loraSerial.print("radio tx ");
          loraSerial.println(deviceIDS[idCurrent]);
          Lora_RX_Sensor();
          stopSend = millis();
          onVal[currentI] = stopSend - startSend;
          currentI++;
          if(currentI == 50) {
            currentI = 0;
          }
        }        
      } else if (deviceIDS[idCurrent] != 0) {
        //if we now want to send to the actuator station
        Value_Check(deviceIDS[idCurrent]);
        sendDataMQTT();
        startSend = millis();
        offVal[currentI] = startSend - stopSend;
        if (checkLora()){
          //we know we can send to lora
          //send a wakeup call over lora
          //send the data via lora
          Lora_RX_Sensor();
          Value_toLora();
          Lora_TX_Actuator(motor_int);
          Lora_TX_Actuator(scarecrow_int);
          Lora_TX_Actuator(uvlamp_int);
          stopSend = millis();
          onVal[currentI] = stopSend - startSend;
          currentI++;
          if(currentI == 50) {
            currentI = 0;
          }
        

      }      
      idCurrent++;
      //incerase the current ID so that next time we send to the next sensor
    }              
    mqttClient.loop();
    //what we need to have in the loop so that we can get the mqtt messages
}


//function that updates the weather values in our global variables, we really only need this to be called once or a couple times a day
void getWeather() {
  OWOC.parseWeather();
  //reparse the weather to update it
  precipFor = OWOC.forecast[1].rainVolume + OWOC.forecast[2].rainVolume;
  //set the precipFor(cast) to the combined rain volume over the next two days
  sunFor = OWOC.forecast[1].cloudCover + OWOC.forecast[2].cloudCover;
  //set the sunFor(cast) to the combined sun over the next two days
  //both thr rainVolume and cloudCover are a double of 0-100, 
}



// function for getting the current time, also prints it to the serial monitor
void setClock () {
    configTime (0, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print ("Waiting for NTP time sync: ");
    time_t now = time (nullptr);
    while (now < 8 * 3600 * 2) {
        delay (500);
        Serial.print (".");
        now = time (nullptr);
    }
    struct tm timeinfo;
    gmtime_r (&now, &timeinfo);
    Serial.print ("\n");
    Serial.print ("Current time: ");
    Serial.print (asctime (&timeinfo));
}

//so we'r
String assignID() {
  char formattedTime[30];
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);

  strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M", &timeinfo);
  return String(formattedTime);
}

//for connecting to MQTT, will run infinitely tho, so maybe we like don't have that or change it depending on what our code needs, although I'm not sure if this gets called
void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("connected to MQTT broker");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 1 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}



//callback for subscribing to the MQTT topic
void callback(char* topic, byte* payload, unsigned int length) {
  //print out which topic the message is being recieved from
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message = "";
  //get the payload into a string
  for (int i = 0; i < length; i++) {
    message += ((char)payload[i]);
  }
  

  //if we receive a message from the cat tracking station that means we do this 
  if (strcmp(topic, mqtt_cat) == 0) {
    Serial.println(message);
    Serial.println("THIS IS THE CAT TRACKING DATA");
    StaticJsonDocument<200> doc;
    deserializeJson(doc, message);
    double xcor = doc["x"];
    double ycor = doc["y"];
    if((xcor < 4) && (ycor < 4)) {
      catData = false;
    } else {
      catData = true;
    }
    Serial.println(catData);

  } else if (strcmp(topic, mqtt_topic) == 0) { //what we do when we receive sensor data over mqtt

    //if we receive the message from the sensor station that means it is an ACK and we do this _____

    //we know the message is from the sensor station, so instead we do this


    message = cipher->decryptString(message);
    //get and decrypt the messafe into a string

    Serial.println(message);
    //print it for testing

    //extract the data from the string, put it into ints for the sensor id, the water level, three light sensors, and then the motion as well
    StaticJsonDocument<200> doc;
    deserializeJson(doc, message);
    int senID = doc["deviceId"];
    int value1 = doc["sensor1"];
    int value2 = doc["sensor2"];
    int value3 = doc["sensor3"];
    int valueM = doc["waterLevel"];
    int motionval = doc["motion"];
    //put the sensor values into the global variables that we have
    senWater = valueM;
    senLight = value1 + value2 + value3;
    senLight = senLight * 100 / (3 * 4096);
    if (motionval == 0) {
      senMotion = false;
    } else {
      senMotion = true;
    }

    mqttClient.publish(mqtt_sensor_sleep, "sleep");
  } else if (strcmp(topic, mqtt_humid) == 0) {
    //so this is what we want to do when we are receiving a new device to add to our list
    int idNumber; // to store the extracted ID number
    int humidity; // to store the extracted humidity

    idNumber = message.substring(3,11).toInt();
    humidity = message.substring(20,22).toInt();
    for (int i = 0; i < 9; i++) {
      if(idNumber == deviceIDS[i]) {
        humidityS[i] = humidity;
      }
    }
  } else if (strcmp(topic, mqtt_newDevice) == 0) {
    //so this is when there's a new humidity that we want updated
    String numbers = message.substring(3);


      // Convert the substring (ID number) to integer using atoi()
      int idNumber2 = message.substring(3, 11).toInt();
      deviceIDS[numDevices] = idNumber2;
      Serial.print("We just received a new device, Sensor has ID:");
      Serial.println(idNumber2);
      numDevices++;
      //store the device ID, one for sensors one for actuator, and print both of them
      deviceIDS[numDevices] = idNumber2*100;
      Serial.print("Actuator has ID:");
      Serial.println(deviceIDS[numDevices]);
      numDevices++;
    }
}



//for sending data to the actuator station
void sendDataMQTT() {
  

  char payload[150];
  char* deviceId = "12345678";
  char motor_str[5]; // Buffer to hold the string representation of motor
  char scarecrow_str[5]; // Buffer to hold the string representation of scarecrow
  char uvlamp_str[5]; // Buffer to hold the string representation of uvlamp

// Convert boolean variables to string representations
  strcpy(motor_str, motor ? "1" : "0");
  strcpy(scarecrow_str, scarecrow ? "1" : "0");
  strcpy(uvlamp_str, uvlamp ? "1" : "0");
  snprintf(payload, sizeof(payload), "{\"deviceId\":\"%s\",\"sprinkler\":%s,\"scarecrow\":%s,\"uvlamp\":%s}", deviceId, motor_str, scarecrow_str, uvlamp_str);
  //publish to the correspondign topic for the actuator station
  String temp1 = String(payload);
  temp1 = cipher->encryptString(temp1);
  const char* temp1_char = temp1.c_str();

  mqttClient.publish(mqtt_to_publish, temp1_char);

}

void Value_Check(int humidCheck) {
    //create the three booleans corresponding to the sensors
  //in general true means turn the sensor on, false means have the sensor off
  motor = true; //motor is the water level
  scarecrow = false; //scarecrow is the motion sensor and hord
  uvlamp = true; //uv lamp is photoresistor and uv lamp
  if(senLight > lightSenT || sunFor > lightForT) {
    //same logic as for the water sensor, if the measured value or the forcast value are greater than the threshold, we don't need to turn it on
    uvlamp = false;
  }
  if (senWater > humidCheck || precipFor > waterForT){
    //same logic for the 
    motor = false;
  }
  if (senMotion && catData) {
    //if the sensor says there is motion and the cat is not in the garden
    //so cat data is high if the cat is not in the garden
    scarecrow = true;
  }
  
}

//converting our global variables into the ints that we will send to the acuator station
void Value_toLora() {
  if(motor==true) {
    motor_int=21;
  }
  else if(motor==false){
    motor_int=11;
  }

  if(scarecrow==true) {
    scarecrow_int=22;
  }
  else if(scarecrow==false){
    scarecrow_int=12;
  }

  if(uvlamp==true) {
    uvlamp_int=23;
  }
  else if(uvlamp==false){
    uvlamp_int=13;
  }

}


//for connecting to our wifi and setting up the mqtt connection
void MQTT_Setup() {
    //hardcode connection
    //connect to wifi here
    WiFi.begin(ssid, password);

    //while we are not connected repeat until we are connected
    while (WiFi.status() != WL_CONNECTED) {

      delay(500);

      Serial.println("Connecting to WiFi..");
    }
    //we are now connected to wifi, so let's connect to our mosquito broker
    mqttClient.setServer(mqtt_server, mqtt_port);

    //we want callbacks to be on
    mqttClient.setCallback(callback);
    while (!mqttClient.connected()) {

     String client_id = "esp32-client-";

     client_id += String(WiFi.macAddress());

     Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());

     if (mqttClient.connect(client_id.c_str(), "", "")) {

         Serial.println("Public emqx mqtt broker connected");

     } else {

         Serial.print("failed with state ");

         Serial.print(mqttClient.state());

         delay(2000);

     }

    }
    //OK were good wohoo! connected to our broker
    Serial.println("Published");

    //mqtt_topic is the data we receive from the sensor station
    mqttClient.subscribe(mqtt_topic);
    //mqtt_cat is from the home assistant that tells us if the cat is in the garden
    mqttClient.subscribe(mqtt_cat);
    //mqtt_newDevice is for receiving a new device from the webpage
    mqttClient.subscribe(mqtt_newDevice);
    //mqtt_humid is for receiving from the web page again but instead for when the user sets humidity
    mqttClient.subscribe(mqtt_humid);

    Serial.println("Received");

}


void Lora_RX_Sensor() {

  Serial.println("waiting for a message");

  loraSerial.println("radio rx 0");                       //Command to receive LoRa message, waits for 60 seconds
  str = loraSerial.readStringUntil('\n');                 //Reading received message
  Serial.println(str);                                    //Printing received message

  int sensorValue1 = 0;                                   //Initializing sensor values for photoresistors
  int sensorValue2 = 0;
  int sensorValue3 = 0;
  delay(20);
  if ( str.indexOf("ok") == 0 )                           //Checking if "ok" is received with LoRa
  {
    str = String("");                                     //Clearing the string for receival
    while(str=="")                                        //Loop until non-empty string is received
    {
      str = loraSerial.readStringUntil('\n');             //Reading new message
    }
    if ( str.indexOf("radio_rx") == 0 )                   //Checking if data was received
    {
      Serial.println("Radio rx");                         //Debugging print statement
      Serial.println(str);                                //printing received data

      str = loraSerial.readStringUntil('\n');             //Reading received data
      Serial.println(str);                                //Printing received data

      integer=str.toInt();                                //Converting received data to integer

      if(integer%100==1){                                 //Checking, which sensor type the received message is from
        sensorValue1 = integer/100;                       //Extracting sensor value
      }
      if(integer%100==2){                                 //Same as before
        sensorValue2 = integer/100;
      }
      if(integer%100==3){
        sensorValue3 = integer/100;
      }
      if(integer%100==4){
        senWater= integer/100;
      }
      if(integer%100==5){
        senMove= integer/100;
      }

      sensor_sleep_counter+=1;                            //Incrementing sensor sleep counter

      if(sensor_sleep_counter==5) {                       //Sending sleep command after 5 iterations, because 5 messages have to be received for the ESP to start sleeping
        loraSerial.print("radio tx ");
        loraSerial.println("99999909");                   //Specific command for starting sleeping
        sensor_sleep_counter=0;
      }

      if((sensorValue1!=0)&&(sensorValue2!=0)&&(sensorValue3!=0)) {  //Calculating light sensor value
        senLight = sensorValue1 + sensorValue2 + sensorValue3;
        senLight = senLight * 100 / (3 * 4096);
      }

      int senLight;
      delay(2000);         
      packageID = packageID + 1;                          //Incrementing package ID
    }
    else
    {
      Serial.println("Received nothing");  
    }
  }
  else
  {
    Serial.println("radio not going into receive mode");  
    delay(1000); 
  }
}

void Lora_TX_Actuator(bool actuator_value) {              //Function to transmit actuator data via LoRa

  Serial.print("packageID = ");  
  Serial.println(packageID);                              //Printing package ID
  loraSerial.print("radio tx ");                          //Command to transmit via LoRa
  loraSerial.println(actuator_value);                     //Transmitting actuator value
  str = loraSerial.readStringUntil('\n');                 //Reading received message
  Serial.println(str);                             

  delay(2000);                                            //Delay before next iteration
  packageID = packageID + 1;                              //Incrementing package ID

  loraSerial.println("radio rx 0");                       //Command to receive acknowledgement over LoRa, waits for 60 seconds
  str = loraSerial.readStringUntil('\n');                 //Reading received message
  Serial.println(str);                                    //Printing received message
  delay(20);
  if ( str.indexOf("ok") == 0 )                           //Checking if "ok" is received
  {
    str = String("");                                     //Clearing the string
    while(str=="")                                        //Loop until non-empty string is received
    {
      str = loraSerial.readStringUntil('\n');             //Reading new message
    }
    if ( str.indexOf("radio_rx") == 0 )                   //Checking if data was received
    {
      Serial.println("Radio rx");                         //Debugging print statement
      actuator_sleep_counter+=1;                          //Incrementing actuator sleep counter
     
      if(actuator_sleep_counter==10) {                    //Sending sleep command after 10 iterations
        loraSerial.println("99999919");
        actuator_sleep_counter=0;
      }

    }
    else
    {
      Serial.println("Received nothing");                 //Debugging print statement
    }
  }
  else
  {
    Serial.println("radio not going into receive mode");  //Debugging print statement
    delay(1000);                                          //Delay for 1 second
  }
}

//function for checking wether or not we have bandwidth avaiable to send over lora
int checklora() {
  //create two ints we stores the arrays in
  int sending = 0;
  int noting = 0;
  for (int i = 0, i < Imax, i++) {
      //add each of the onVal and offVal sums into the two ints
      sending += onVal[i];
      noting += offVal[i];
  }
  //mutliply the sending by the threshold
  sending = sending * BWthresh;
  //check if the not sending is more than the sending * threshold, if it is we have BW to send
  if (sending < noting) {
    return 1;
  } else {
    //we have no bandwidth to send
    return 0;
  }
}

void lora_autobaud()
{
  String response = "";
  while (response=="")
  {
    delay(1000);
    loraSerial.write((byte)0x00);
    loraSerial.write(0x55);
    loraSerial.println();
    loraSerial.println("sys get ver");
    response = loraSerial.readStringUntil('\n');
  }
}
