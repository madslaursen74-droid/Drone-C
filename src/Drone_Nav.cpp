#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

#define GPS_RX 34


void updateGPS()
{
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }
}


void GetGPSData()
{
  updateGPS();

  if (gps.satellites.value() > 5 && gps.location.isValid())
  {
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 7);
    Serial.print("Latitude: "); 
    Serial.println(gps.location.lat(), 7);

  }
}

void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
 delay(3000);
 Wire.begin(21, 22);
 Wire.setClock(100000); 
 Serial.println("starting GPS... ");
 gpsSerial.begin(9600, SERIAL_8N1, GPS_RX);

 if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
delay(1000);
    
bno.setExtCrystalUse(true);

}

void readBNOsensor()
{
  sensors_event_t event; 
  bno.getEvent(&event);
  Serial.print("Orientation: ");
  Serial.print(event.orientation.x, 4);
  Serial.print(" ");
  Serial.print(event.orientation.y, 4);
  Serial.print(" ");
  Serial.println(event.orientation.z, 4);


  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print("CALIB: ");
  Serial.print(sys); Serial.print(" ");
  Serial.print(gyro); Serial.print(" ");
  Serial.print(accel); Serial.print(" ");
  Serial.println(mag);

}

    
void loop() {
  readBNOsensor();
  Serial.println("sats");
  Serial.print(gps.satellites.value());
  delay(1000);
  updateGPS();
  GetGPSData();
  delay(1000);
}
