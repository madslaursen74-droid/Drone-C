#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <utility/imumaths.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Servo leftEsc;
Servo rightEsc;

constexpr int GPS_RX = 17;
constexpr int LEFT_ESC_PIN = 18;
constexpr int RIGHT_ESC_PIN = 4;

constexpr float kArrivalRadiusMeters = 3.0f;
constexpr int kEscMinUs = 1000;
constexpr int kEscMaxUs = 2000;
constexpr int kEscNeutralUs = 1500;
constexpr int kEscArmDelayMs = 3000;
constexpr int kEscSignalHz = 50;
constexpr int kBaseForwardUs = 1640;
constexpr int kNearTargetForwardUs = 1575;
constexpr int kMinForwardUs = 1535;
constexpr int kMaxForwardUs = 1750;
constexpr float kHeadingKp = 2.1f;
constexpr float kHeadingDeadbandDegrees = 4.0f;
constexpr float kStrongTurnThresholdDegrees = 35.0f;
constexpr float kSlowTurnThresholdDegrees = 70.0f;
constexpr float kHeadingBlendAlpha = 0.25f;
constexpr float kMaxHdop = 3.5f;
constexpr int kMinSatellites = 6;
constexpr int kThrusterMinAngle = 0;
constexpr int kThrusterNeutralAngle = 90;
constexpr int kThrusterMaxAngle = 180;
constexpr double kWaypointE7Scale = 10000000.0;

// Keep this false unless the ESP-NOW sender is trusted and the boat is safely restrained.
constexpr bool kAllowEspNowManualThrusterOverride = false;

enum class BoatMode {
  Idle,
  Auto,
  Manual
};

struct Waypoint {
  double latitude = 0.0;
  double longitude = 0.0;
  bool valid = false;
};

struct __attribute__((packed)) RecoveredControlPacket {
  uint32_t time;
  uint32_t leftThruster;
  uint32_t rightThruster;
  uint32_t navigationMode;
  uint32_t waypointFunction;
  uint32_t waypointId;
  int32_t waypointLatitude;
  int32_t waypointLongitude;
};

String serialBuffer;
Waypoint targetWaypoint;
BoatMode currentMode = BoatMode::Idle;
bool bnoReady = false;
bool espNowReady = false;
unsigned long lastStatusMs = 0;
float filteredHeadingDegrees = 0.0f;
bool hasFilteredHeading = false;
RecoveredControlPacket pendingEspNowPacket;
int pendingEspNowPacketLength = 0;
bool hasPendingEspNowPacket = false;
portMUX_TYPE espNowPacketMux = portMUX_INITIALIZER_UNLOCKED;

float normalizeHeadingError(float errorDegrees);

const char* modeName(BoatMode mode) {
  switch (mode) {
    case BoatMode::Auto:
      return "AUTO";
    case BoatMode::Manual:
      return "MANUAL";
    case BoatMode::Idle:
    default:
      return "IDLE";
  }
}

float blendHeadingDegrees(float previous, float current, float alpha) {
  const float delta = normalizeHeadingError(current - previous);
  float blended = previous + alpha * delta;

  while (blended < 0.0f) blended += 360.0f;
  while (blended >= 360.0f) blended -= 360.0f;
  return blended;
}

bool hasUsableCalibration() {
  if (!bnoReady) return false;

  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  return sys >= 1 && mag >= 2;
}

void updateGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

bool hasGoodGPSFix() {
  if (!gps.location.isValid()) return false;
  if (gps.satellites.value() < kMinSatellites) return false;
  if (gps.location.age() > 2000) return false;
  if (gps.hdop.isValid() && gps.hdop.hdop() > kMaxHdop) return false;
  return true;
}

double toRadians(double degrees) {
  return degrees * PI / 180.0;
}

double toDegrees(double radians) {
  return radians * 180.0 / PI;
}

double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double earthRadiusMeters = 6371000.0;

  const double dLat = toRadians(lat2 - lat1);
  const double dLon = toRadians(lon2 - lon1);
  const double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
                   cos(toRadians(lat1)) * cos(toRadians(lat2)) *
                   sin(dLon / 2.0) * sin(dLon / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return earthRadiusMeters * c;
}

double calculateBearingDegrees(double lat1, double lon1, double lat2, double lon2) {
  const double phi1 = toRadians(lat1);
  const double phi2 = toRadians(lat2);
  const double deltaLon = toRadians(lon2 - lon1);

  const double y = sin(deltaLon) * cos(phi2);
  const double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLon);
  double bearing = toDegrees(atan2(y, x));

  while (bearing < 0.0) bearing += 360.0;
  while (bearing >= 360.0) bearing -= 360.0;
  return bearing;
}

float normalizeHeadingError(float errorDegrees) {
  while (errorDegrees > 180.0f) errorDegrees -= 360.0f;
  while (errorDegrees < -180.0f) errorDegrees += 360.0f;
  return errorDegrees;
}

bool readHeadingDegrees(float& headingDegrees) {
  if (!bnoReady) return false;
  if (!hasUsableCalibration()) return false;

  sensors_event_t event;
  bno.getEvent(&event);
  if (!isfinite(event.orientation.x)) return false;

  float rawHeadingDegrees = event.orientation.x;
  if (rawHeadingDegrees < 0.0f) {
    rawHeadingDegrees += 360.0f;
  }

  if (!hasFilteredHeading) {
    filteredHeadingDegrees = rawHeadingDegrees;
    hasFilteredHeading = true;
  } else {
    filteredHeadingDegrees = blendHeadingDegrees(
        filteredHeadingDegrees, rawHeadingDegrees, kHeadingBlendAlpha);
  }

  headingDegrees = filteredHeadingDegrees;
  return true;
}

int thrusterAngleToMicroseconds(uint32_t angle) {
  const int constrainedAngle = constrain(
      static_cast<int>(angle), kThrusterMinAngle, kThrusterMaxAngle);
  return map(constrainedAngle, kThrusterMinAngle, kThrusterMaxAngle, kEscMinUs, kEscMaxUs);
}

void writeEscMicroseconds(int leftSpeed, int rightSpeed, const char* source) {
  leftSpeed = constrain(leftSpeed, kEscMinUs, kEscMaxUs);
  rightSpeed = constrain(rightSpeed, kEscMinUs, kEscMaxUs);

  leftEsc.writeMicroseconds(leftUs);
  rightEsc.writeMicroseconds(rightUs);
}

void stopBoat() {
  setMotors(ESC_STOP_US, ESC_STOP_US);
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

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("BOOT,AUTO_NAV");

  setupEspNowReceiver();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  leftEsc.setPeriodHertz(kEscSignalHz);
  rightEsc.setPeriodHertz(kEscSignalHz);
  leftEsc.attach(LEFT_ESC_PIN, kEscMinUs, kEscMaxUs);
  rightEsc.attach(RIGHT_ESC_PIN, kEscMinUs, kEscMaxUs);
  leftEsc.writeMicroseconds(kEscNeutralUs);
  rightEsc.writeMicroseconds(kEscNeutralUs);
  Serial.println("ESC,ARMING_NEUTRAL");
  delay(kEscArmDelayMs);
  Serial.println("ESC,READY");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  bnoReady = bno.begin();
  if (bnoReady) {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055,READY");
  } else {
    Serial.println("BNO055,NOT_FOUND");
  }

  Serial.println("READY,COMMANDS=GOTO,lat,lon|THRUST,left,right|AUTO|STOP|STATUS");
}

void loop() {
  updateGPS();
  readSerialCommands();
  pollEspNowPacket();
  runAutonomousController();

  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    printStatus();
  }

  delay(50);
}
