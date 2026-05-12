

/*

  ESP-NOW Demo - Receive

  esp-now-demo-rcv.ino

  Reads data from Initiator

 

  DroneBot Workshop 2022

  https://dronebotworkshop.com

 

  Opdateret så funktionskaldet til call-backfunktionen har de rigtige parametre. Endvidre cleares wifi inden det startes op

 

*/

 

#define debug true

 

// Include Libraries

#include <esp_now.h>

#include <WiFi.h>

 

// Define a data structure

typedef struct struct_message {

  uint32_t time; //32 bit

  int lt; //left thruster

  int rt; //right thruster

  int nm; //navmode. 1: manual, 2: route. 3: return to home

  int wpf; //waypoint function: 0: go to current, 1 erstat aktuel position i listen # angivet i wpid

  int wpid; // waypointid. nummer på wp der skal håndteres:

  int wla; // waypoint lattitude

  int wlo; //waypoint longitude

} struct_message;

 

// Create a structured object

struct_message myData;



#include <ESP32Servo.h>

 

// create four servo objects

Servo portThruster;

Servo starboardThruster;

int minUs = 1000;//1000;

int maxUs = 2000;//2000;

 

// These are all GPIO pins on the ESP32

// Recommended pins include 2,4,12-19,21-23,25-27,32-33

 

int portThrusterPin = 15; //   <----- Note these are used

int starboardThrusterPin = 18;// skal bruges

 

//serial på 16 og 17 til GPS

 

int pos = 0;      // position in degrees

ESP32PWM pwm;





// Callback function executed when data is received

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {

 

//void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  memcpy(&myData, incomingData, sizeof(myData));

 

  if(debug){

    Serial.print("Data received: ");

    Serial.println(len);

    Serial.print("time: ");

    Serial.println(myData.time);

    Serial.print("leftThuster ");

    Serial.println(myData.lt);

    Serial.print("right Thruster: ");

    Serial.println(myData.rt);

    Serial.print("Navigation mode: ");

    Serial.println(myData.nm);

    Serial.print("Waypoint function: ");

    Serial.println(myData.wpf);

    Serial.print("Waypoint id: ");

    Serial.println(myData.wpid);

    Serial.print("Waypoint latitude: ");

    Serial.println(myData.wla);

    Serial.print("Waypoint longitude: ");

    Serial.println(myData.wlo);

    Serial.println();

    }

 

  if(myData.lt > 100 || myData.lt < 80){// hysterese

    portThruster.write(myData.lt);

  } else {

    portThruster.write(90);

  }

 

  if(myData.rt > 100 || myData.rt < 80){

    starboardThruster.write(myData.rt);

  } else {

    starboardThruster.write(90);

  }

 

 

}

 

void setup() {

  // Set up Serial Monitor

  Serial.begin(115200);

  delay(200);

  Serial.print("Så har vi serial...");

 

  // Set ESP32 as a Wi-Fi Station

  WiFi.disconnect(true); // Sørg for at frakoble eksisterende forbindelser

  delay(100);

  Serial.print("Så sætter vi wifi op.");

  WiFi.mode(WIFI_STA);  // Sætter WiFi-modulet i station mode

  Serial.print("Wifi er sat op");

 

  // Initilize ESP-NOW

  if (esp_now_init() != ESP_OK) {

    Serial.println("Error initializing ESP-NOW");

    return;

  }

 

    Serial.print("ESP NOW er initialisere");

 

  // Register callback function

  esp_now_register_recv_cb(OnDataRecv);

    Serial.print("callback er registreret. Setup afsluttet");

 

// Allow allocation of all timers

  ESP32PWM::allocateTimer(0);

  ESP32PWM::allocateTimer(1);

  ESP32PWM::allocateTimer(2);

  ESP32PWM::allocateTimer(3);

  Serial.begin(115200);

  portThruster.setPeriodHertz(200);      // Standard 50hz servo

  starboardThruster.setPeriodHertz(200);      // Standard 50hz servo

  portThruster.attach(portThrusterPin, minUs, maxUs);

  starboardThruster.attach(starboardThrusterPin, minUs, maxUs);

  #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)

    pwm.attachPin(37, 10000);//10khz

  #elif defined(CONFIG_IDF_TARGET_ESP32C3)

    pwm.attachPin(7, 10000);//10khz

  #else

    pwm.attachPin(27, 10000);//10khz

  #endif



  //arming:

  Serial.println("Arming sequence started...");

 

  //set thrusters to 0

  portThruster.write(0);

  starboardThruster.write(0);

  delay(3000);  // waits a while

  Serial.print(".");

  //set thrusters to full

  portThruster.write(180);

  starboardThruster.write(180);

  delay(2000);

  Serial.print(".");            // waits a while

  //set thrusters to middle (stop)

  portThruster.write(90);

  starboardThruster.write(90);

  delay(2000);             // waits a while

 

  Serial.println("Arming sequence done.");

  //initialisation done

 

delay(3000);

 

}

 

void loop() {

 

}

 

void stop(){

  portThruster.write(90);

  starboardThruster.write(90);

}

 
'




// start i 3 sekunder, derefter sæt begge postthruster til 180 og derefter sæt det itl 180 og derefter 90 for at stoppe. Dette er for at arming af ESC'erne, så de ikke starter i fuld fart når de modtager signal første gang. Det er vigtigt at gøre dette