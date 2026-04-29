#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#define GPS_RX 16 

void updateGPS()
{
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }
}

bool hasGoodGPSFix()
{
  if (!gps.location.isValid()) return false;
  if (gps.satellites.value() < 5) return false;

  // Optional: also require fresh data
  if (gps.location.age() > 2000) return false;

  return true;
}

void GetGPSData()
{
  updateGPS();

  if (!hasGoodGPSFix())
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
 Serial.println("starting GPS... ");
 gpsSerial.begin(9600, SERIAL_8N1, GPS_RX);

  while (!hasGoodGPSFix())
  {
    updateGPS();

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  }
}
    
void loop() {
  
  updateGPS();
  GetGPSData();
}
