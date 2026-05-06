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

#define GPS_RX 16

double HardcodedTargets[][2] = {{55.6761, 12.5683},
                                {57.5353, 13.2683},
                                {60.3262, 13.3483}};


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
  Serial.print("Heading: ");
  Serial.print(event.orientation.x, 4);


  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print("CALIB: ");
  Serial.println(mag);

}

double bearingToPoint(double lat1, double lon1, double lat2, double lon2)
{
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLon = lon2 - lon1;

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) -
             sin(lat1) * cos(lat2) * cos(dLon);

  double bearing = degrees(atan2(y, x));

  if (bearing < 0) bearing += 360;

  return bearing;
}

double angleDifference(double targetBearing, double currentHeading)
{
  double diff = targetBearing - currentHeading;

  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;

  return diff;
}

void turnToPoint(double targetLat, double targetLon)
{
  if (!gps.location.isValid()) {
    Serial.println("GPS location not valid");
    return;
  }

  double currentLat = gps.location.lat();
  double currentLon = gps.location.lng();

  double targetBearing = bearingToPoint(currentLat, currentLon, targetLat, targetLon);
  
  sensors_event_t event; 
  bno.getEvent(&event);
  double currentHeading = event.orientation.x;

  double turnAngle = angleDifference(targetBearing, currentHeading);

  
  if (turnAngle > 10) {
    Serial.println("Turn right");
  } else if (turnAngle < -10) {
    Serial.println("Turn left");
  } else {
    Serial.println("Already facing the target point");
  }
}

double distanceToPoint(double lat1, double lon1, double lat2, double lon2)
{
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double a = sin(dLat/2) * sin(dLat/2) +
             cos(lat1) * cos(lat2) *
             sin(dLon/2) * sin(dLon/2);

  double c = 2 * atan2(sqrt(a), sqrt(1-a));

  double distance = 6371000 * c; // Earth radius in meters

  return distance;
}

    
void loop() {
  readBNOsensor();
  Serial.println("sats");
  Serial.println(gps.satellites.value());
  delay(1000);
  updateGPS();
  GetGPSData();

  for (int i = 0; i < 3; i++) {
    if (distanceToPoint(gps.location.lat(), gps.location.lng(), HardcodedTargets[i][0], HardcodedTargets[i][1]) > 5) {
      turnToPoint(HardcodedTargets[i][0], HardcodedTargets[i][1]);
      updateGPS();
      // turn function
      // Forward function
      delay(500);
    }
    else {
      Serial.print("Reached target point ");
    }
  }
}
