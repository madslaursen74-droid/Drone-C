#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>


TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

#define GPS_RX 16

constexpr int LEFT_ESC_PIN = 18;
constexpr int RIGHT_ESC_PIN = 4;

constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 2000;
constexpr int ESC_STOP_US = 1500;
constexpr int ESC_FORWARD_US = 1640;
constexpr int ESC_SLOW_US = 1575;

constexpr float ARRIVAL_RADIUS_METERS = 3.0f;
constexpr float HEADING_KP = 2.0f;
constexpr float HEADING_DEADBAND = 4.0f;

int currentWaypointIndex = 0;

Servo leftEsc;
Servo rightEsc;

double HardcodedTargets[][2] = {{55.6761, 12.5683},
                                {57.5353, 13.2683},
                                {60.3262, 13.3483}};

constexpr int WAYPOINT_COUNT = sizeof(HardcodedTargets) / sizeof(HardcodedTargets[0]);
constexpr unsigned long WAYPOINT_STOP_TIME_MS = 5000;

bool waitingAtWaypoint = false;
unsigned long waypointStopStartTime = 0;


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
void setMotors(int leftUs, int rightUs) {
  leftUs = constrain(leftUs, ESC_MIN_US, ESC_MAX_US);
  rightUs = constrain(rightUs, ESC_MIN_US, ESC_MAX_US);

  leftEsc.writeMicroseconds(leftUs);
  rightEsc.writeMicroseconds(rightUs);
}
void stopBoat() {
  setMotors(ESC_STOP_US, ESC_STOP_US);
}

void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
 ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  leftEsc.setPeriodHertz(50);
  rightEsc.setPeriodHertz(50);
  leftEsc.attach(LEFT_ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  rightEsc.attach(RIGHT_ESC_PIN, ESC_MIN_US, ESC_MAX_US);

  stopBoat();
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

  Serial.print(" Pitch: ");
  Serial.print(event.orientation.y, 4);

  Serial.print(" Roll: ");
  Serial.println(event.orientation.z, 4);


  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print("CALIB: ");
  Serial.print(sys); Serial.print(" ");
  Serial.print(gyro); Serial.print(" ");
  Serial.print(accel); Serial.print(" ");
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

double turnToPoint(double targetLat, double targetLon)
{
  if (!gps.location.isValid()) {
    Serial.println("GPS location not valid");
    return 0;
  }

  double currentLat = gps.location.lat();
  double currentLon = gps.location.lng();

  double targetBearing = bearingToPoint(currentLat, currentLon, targetLat, targetLon);
  
  sensors_event_t event; 
  bno.getEvent(&event);
  double currentHeading = event.orientation.x;

  double turnAngle = angleDifference(targetBearing, currentHeading);

  
 return turnAngle;
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

float headingErrorDegrees(float target, float current) {
  float error = target - current;

  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  if (fabs(error) < HEADING_DEADBAND) {
    error = 0.0f;
  }

  return error;
}



void goToCurrentWaypoint(double distance, float error) {

 int baseSpeed = ESC_FORWARD_US;
  if (distance < 8.0 || fabs(error) > 35.0f) {
    baseSpeed = ESC_SLOW_US;
  }

  const int correction = static_cast<int>(round(HEADING_KP * error));
  const int leftSpeed = constrain(baseSpeed + correction, ESC_STOP_US, ESC_MAX_US);
  const int rightSpeed = constrain(baseSpeed - correction, ESC_STOP_US, ESC_MAX_US);

  setMotors(leftSpeed, rightSpeed);
}

    
void loop() {
  readBNOsensor();
  Serial.println("sats");
  Serial.println(gps.satellites.value());
  delay(1000);
  updateGPS();
  GetGPSData();
  delay(1000);
  updateGPS();

  if (gps.location.isValid()) {
    if (currentWaypointIndex >= WAYPOINT_COUNT) {
      Serial.println("All target points reached");
      stopBoat();
      return;
    }

    if (waitingAtWaypoint) {
      stopBoat();

      if (millis() - waypointStopStartTime >= WAYPOINT_STOP_TIME_MS) {
        Serial.print("Leaving target point ");
        Serial.println(currentWaypointIndex);
        currentWaypointIndex++;
        waitingAtWaypoint = false;
      }

      delay(200);
      return;
    }

    double targetLat = HardcodedTargets[currentWaypointIndex][0];
    double targetLon = HardcodedTargets[currentWaypointIndex][1];
    double distance = distanceToPoint(
      gps.location.lat(),
      gps.location.lng(),
      targetLat,
      targetLon
    );

    if (distance <= ARRIVAL_RADIUS_METERS) {
      Serial.print("Reached target point ");
      Serial.println(currentWaypointIndex);
      stopBoat();
      waitingAtWaypoint = true;
      waypointStopStartTime = millis();
      delay(200);
      return;
    }

    double error = turnToPoint(targetLat, targetLon);
    Serial.println(distance);
    Serial.println(error);
    goToCurrentWaypoint(distance, error);
    delay(200);
  }
  else {
    Serial.println("Waiting for valid GPS location...");
    stopBoat();
    updateGPS();
    delay(2000);
  }
}
