#include <Wire.h>
#include <Arduino.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <Preferences.h>

Preferences prefs;

// ---------- GPS + COMPASS ----------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

#define GPS_RX 16

#pragma region Variables and Constants
// ---------- MOTOR PINS ----------
constexpr int LEFT_ESC_PIN  = 4;
constexpr int RIGHT_ESC_PIN = 18;

// ---------- ESC VALUES ----------
constexpr int ESC_MIN_US     = 1000;
constexpr int ESC_MAX_US     = 2000;
constexpr int ESC_STOP_US    = 1500;
constexpr int ESC_SLOW_US    = 1575;
constexpr int ESC_FORWARD_US = 1640;

// Maximum extra motor difference while turning
constexpr int MAX_TURN_POWER = 180;

// ---------- NAVIGATION SETTINGS ----------
constexpr float ARRIVAL_RADIUS_METERS = 3.0;
constexpr float SLOW_DISTANCE_METERS  = 8.0;
constexpr float HEADING_DEADBAND      = 8.0;
constexpr float HEADING_OFFSET = 171.0;  // tune this value

// Higher = turns harder for same angle error
constexpr float TURN_KP = 3.0;

// Load BNO055 calibration data from EEPROM
#define EEPROM_SIZE 128
#define CALIB_FLAG_ADDR 0
#define CALIB_DATA_ADDR 1
#pragma endregion

adafruit_bno055_offsets_t calibData;

#pragma region BNO055 Calibration
bool loadCalibration() {
  adafruit_bno055_offsets_t calibData;

  prefs.begin("bno055", true);

  if (!prefs.isKey("calib")) {
    Serial.println("No saved calibration found");
    prefs.end();
    return false;
  }

  prefs.getBytes("calib", &calibData, sizeof(calibData));
  prefs.end();

  bno.setSensorOffsets(calibData);
  Serial.println("Calibration loaded");
  return true;
}
#pragma endregion

// ---------- WAYPOINTS ----------
double waypoints[][2] = {
  {56.458813, 9.402023},
  {56.458836, 9.401664},
  {56.459139, 9.401754}
};

constexpr int WAYPOINT_COUNT = sizeof(waypoints) / sizeof(waypoints[0]);

int currentWaypoint = 0;
bool waitingAtWaypoint = false;
unsigned long waitStartTime = 0;
constexpr unsigned long WAIT_TIME_MS = 5000;

// ---------- MOTORS ----------
Servo leftEsc;
Servo rightEsc;

// ---------- GPS UPDATE ----------
void updateGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

#pragma region Boat functions
// ---------- MOTOR CONTROL ----------
void setMotors(int leftUs, int rightUs) {
  leftUs = constrain(leftUs, ESC_MIN_US, ESC_MAX_US);
  rightUs = constrain(rightUs, ESC_MIN_US, ESC_MAX_US);

  leftEsc.writeMicroseconds(leftUs);
  rightEsc.writeMicroseconds(rightUs);

  Serial.print("Left: ");
  Serial.print(leftUs);
  Serial.print(" Right: ");
  Serial.println(rightUs);
}

void stopBoat() {
  setMotors(ESC_STOP_US, ESC_STOP_US);
}
#pragma endregion

#pragma region Navigation calculations
// ---------- DISTANCE TO WAYPOINT ----------
double distanceToPoint(double lat1, double lon1, double lat2, double lon2) {
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) *
             sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return 6371000.0 * c;
}

// ---------- BEARING TO WAYPOINT ----------
double bearingToPoint(double lat1, double lon1, double lat2, double lon2) {
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

// ---------- ANGLE DIFFERENCE ----------
float angleDifference(float target, float current) {
  float headingError = target - current;

  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;

  if (fabs(headingError) < HEADING_DEADBAND) {
    headingError = 0;
  }

  return headingError;
}

// ---------- GET CURRENT HEADING ----------
float getHeading() {
  sensors_event_t event;
  bno.getEvent(&event);

  float heading = event.orientation.x + HEADING_OFFSET;

  while (heading >= 360) heading -= 360;
  while (heading < 0) heading += 360;

  return heading;
}
#pragma endregion

// BNO055 calibration printout
void printCalibration() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print("CALIB -> ");
  Serial.print("SYS:");
  Serial.print(sys);
  Serial.print(" G:");
  Serial.print(gyro);
  Serial.print(" A:");
  Serial.print(accel);
  Serial.print(" M:");
  Serial.println(mag);
}

#pragma region Boat Movement
// ---------- DRIVE TOWARDS WAYPOINT ----------
void driveToWaypoint(double targetLat, double targetLon) {
  double currentLat = gps.location.lat();
  double currentLon = gps.location.lng();

  double distance = distanceToPoint(currentLat, currentLon, targetLat, targetLon);
  double targetBearing = bearingToPoint(currentLat, currentLon, targetLat, targetLon);

  float currentHeading = getHeading();
  float headingError = angleDifference(targetBearing, currentHeading);

  Serial.print("Waypoint: ");
  Serial.println(currentWaypoint);

  Serial.print("Distance: ");
  Serial.println(distance);

  Serial.print("Target bearing: ");
  Serial.println(targetBearing);

  Serial.print("Heading: ");
  Serial.println(currentHeading);

  Serial.print("Turn error: ");
  Serial.println(headingError);

  // If arrived, stop and wait
  if (distance <= ARRIVAL_RADIUS_METERS) {
    Serial.println("Reached waypoint");
    stopBoat();

    waitingAtWaypoint = true;
    waitStartTime = millis();
    return;
  }

  // Slow down close to waypoint
  int baseSpeed = ESC_FORWARD_US;
  if (distance < SLOW_DISTANCE_METERS) {
    baseSpeed = ESC_SLOW_US;
  }

  // Bigger angle error = stronger turn
  int turnPower = abs(headingError) * TURN_KP;
  turnPower = constrain(turnPower, 0, MAX_TURN_POWER);

  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;

  if (headingError > 0) {
    // Turn right
    leftSpeed  = baseSpeed + turnPower;
    rightSpeed = baseSpeed - turnPower;
    Serial.println("Turning RIGHT");
  }
  else if (headingError < 0) {
    // Turn left
    leftSpeed  = baseSpeed - turnPower;
    rightSpeed = baseSpeed + turnPower;
    Serial.println("Turning LEFT");
  }
  else {
    Serial.println("Going STRAIGHT");
  }

  // Do not allow reverse here, only stop-to-forward range
  leftSpeed = constrain(leftSpeed, ESC_STOP_US, ESC_MAX_US);
  rightSpeed = constrain(rightSpeed, ESC_STOP_US, ESC_MAX_US);

  setMotors(leftSpeed, rightSpeed);
}
#pragma endregion

#pragma region Setup
// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);


  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  leftEsc.setPeriodHertz(50);
  rightEsc.setPeriodHertz(50);

  leftEsc.attach(LEFT_ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  rightEsc.attach(RIGHT_ESC_PIN, ESC_MIN_US, ESC_MAX_US);

  Serial.println("Arming ESCs...");
  stopBoat();
  delay(5000);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }

  // Your remap
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);

  loadCalibration();

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Ready");
}
#pragma endregion

// ---------- MAIN LOOP ----------
#pragma region Main Loop
void loop() {
  updateGPS();

  if (!gps.location.isValid()) {
    Serial.print("Waiting for GPS. Sats: ");
    Serial.println(gps.satellites.value());
    stopBoat();
    delay(500);
    return;
  }

  if (currentWaypoint >= WAYPOINT_COUNT) {
    Serial.println("All waypoints reached");
    stopBoat();
    delay(1000);
    return;
  }

  if (waitingAtWaypoint) {
    stopBoat();

    if (millis() - waitStartTime >= WAIT_TIME_MS) {
      currentWaypoint++;
      waitingAtWaypoint = false;

      Serial.print("Going to next waypoint: ");
      Serial.println(currentWaypoint);
    }

    delay(200);
    return;
  }

  double targetLat = waypoints[currentWaypoint][0];
  double targetLon = waypoints[currentWaypoint][1];

  driveToWaypoint(targetLat, targetLon);

  printCalibration();

  delay(200);
}
#pragma endregion