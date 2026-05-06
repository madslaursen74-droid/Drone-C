#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <utility/imumaths.h>

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

enum class BoatMode {
  Idle,
  Auto
};

struct Waypoint {
  double latitude = 0.0;
  double longitude = 0.0;
  bool valid = false;
};

String serialBuffer;
Waypoint targetWaypoint;
BoatMode currentMode = BoatMode::Idle;
bool bnoReady = false;
unsigned long lastStatusMs = 0;
float filteredHeadingDegrees = 0.0f;
bool hasFilteredHeading = false;

float normalizeHeadingError(float errorDegrees);

const char* modeName(BoatMode mode) {
  switch (mode) {
    case BoatMode::Auto:
      return "AUTO";
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

  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
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

void writeEscMicroseconds(int leftSpeed, int rightSpeed, const char* source) {
  leftSpeed = constrain(leftSpeed, kEscMinUs, kEscMaxUs);
  rightSpeed = constrain(rightSpeed, kEscMinUs, kEscMaxUs);

  leftEsc.writeMicroseconds(leftSpeed);
  rightEsc.writeMicroseconds(rightSpeed);

  Serial.print("MOTORS,LEFT=");
  Serial.print(leftSpeed);
  Serial.print(",RIGHT=");
  Serial.print(rightSpeed);
  Serial.print(",SOURCE=");
  Serial.println(source);
}

void stopBoat() {
  currentMode = BoatMode::Idle;
  writeEscMicroseconds(kEscNeutralUs, kEscNeutralUs, "STOP");
}

void setAutoMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, kEscNeutralUs, kMaxForwardUs);
  rightSpeed = constrain(rightSpeed, kEscNeutralUs, kMaxForwardUs);
  writeEscMicroseconds(leftSpeed, rightSpeed, "AUTO");
}

void printStatus() {
  Serial.print("STATUS,MODE=");
  Serial.print(modeName(currentMode));

  if (hasGoodGPSFix()) {
    Serial.print(",LAT=");
    Serial.print(gps.location.lat(), 7);
    Serial.print(",LON=");
    Serial.print(gps.location.lng(), 7);
    Serial.print(",HDOP=");
    if (gps.hdop.isValid()) {
      Serial.print(gps.hdop.hdop(), 1);
    } else {
      Serial.print("NA");
    }
  } else {
    Serial.print(",GPS=NO_FIX");
  }

  float heading = 0.0f;
  if (readHeadingDegrees(heading)) {
    Serial.print(",HEADING=");
    Serial.print(heading, 1);
  } else {
    Serial.print(",HEADING=NA");
  }

  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
  if (bnoReady) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(",CAL=");
    Serial.print(sys);
    Serial.print("/");
    Serial.print(gyro);
    Serial.print("/");
    Serial.print(accel);
    Serial.print("/");
    Serial.print(mag);
  }

  if (targetWaypoint.valid) {
    Serial.print(",TARGET_LAT=");
    Serial.print(targetWaypoint.latitude, 7);
    Serial.print(",TARGET_LON=");
    Serial.print(targetWaypoint.longitude, 7);
  }

  Serial.println();
}

void handleGotoCommand(const String& payload) {
  const int commaIndex = payload.indexOf(',');
  if (commaIndex < 0) {
    Serial.println("ERROR,INVALID_GOTO_FORMAT");
    return;
  }

  const String latText = payload.substring(0, commaIndex);
  const String lonText = payload.substring(commaIndex + 1);

  targetWaypoint.latitude = latText.toDouble();
  targetWaypoint.longitude = lonText.toDouble();
  targetWaypoint.valid = true;
  currentMode = BoatMode::Auto;

  Serial.print("TARGET_SET,LAT=");
  Serial.print(targetWaypoint.latitude, 7);
  Serial.print(",LON=");
  Serial.println(targetWaypoint.longitude, 7);
}

void handleSerialCommand(const String& rawCommand) {
  String command = rawCommand;
  command.trim();
  if (command.length() == 0) return;

  if (command.equalsIgnoreCase("STOP")) {
    stopBoat();
    Serial.println("ACK,STOP");
    return;
  }

  if (command.equalsIgnoreCase("STATUS")) {
    printStatus();
    return;
  }

  if (command.startsWith("GOTO,")) {
    handleGotoCommand(command.substring(5));
    return;
  }

  Serial.print("ERROR,UNKNOWN_COMMAND=");
  Serial.println(command);
}

void readSerialCommands() {
  while (Serial.available()) {
    const char input = static_cast<char>(Serial.read());
    if (input == '\n' || input == '\r') {
      if (serialBuffer.length() > 0) {
        handleSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += input;
    }
  }
}

void runAutonomousController() {
  if (currentMode != BoatMode::Auto || !targetWaypoint.valid) {
    return;
  }

  if (!hasGoodGPSFix()) {
    Serial.println("WARN,NO_GPS_FIX");
    stopBoat();
    return;
  }

  float headingDegrees = 0.0f;
  if (!readHeadingDegrees(headingDegrees)) {
    Serial.println("WARN,NO_HEADING_OR_CAL");
    stopBoat();
    return;
  }

  const double currentLat = gps.location.lat();
  const double currentLon = gps.location.lng();
  const double distanceMeters = calculateDistanceMeters(
      currentLat, currentLon, targetWaypoint.latitude, targetWaypoint.longitude);

  if (distanceMeters <= kArrivalRadiusMeters) {
    Serial.println("ARRIVED");
    stopBoat();
    return;
  }

  const float targetBearing = static_cast<float>(calculateBearingDegrees(
      currentLat, currentLon, targetWaypoint.latitude, targetWaypoint.longitude));
  float headingError = normalizeHeadingError(targetBearing - headingDegrees);

  if (fabs(headingError) < kHeadingDeadbandDegrees) {
    headingError = 0.0f;
  }

  const float correction = kHeadingKp * headingError;

  int baseThrottleUs = kBaseForwardUs;
  if (distanceMeters < 8.0 || fabs(headingError) > kStrongTurnThresholdDegrees) {
    baseThrottleUs = kNearTargetForwardUs;
  }

  if (fabs(headingError) > kSlowTurnThresholdDegrees) {
    baseThrottleUs = kMinForwardUs;
  }

  int leftSpeed = static_cast<int>(round(baseThrottleUs + correction));
  int rightSpeed = static_cast<int>(round(baseThrottleUs - correction));

  leftSpeed = max(leftSpeed, kMinForwardUs);
  rightSpeed = max(rightSpeed, kMinForwardUs);
  leftSpeed = min(leftSpeed, kMaxForwardUs);
  rightSpeed = min(rightSpeed, kMaxForwardUs);

  Serial.print("AUTO,DIST=");
  Serial.print(distanceMeters, 2);
  Serial.print(",BEARING=");
  Serial.print(targetBearing, 1);
  Serial.print(",HEADING=");
  Serial.print(headingDegrees, 1);
  Serial.print(",ERROR=");
  Serial.print(headingError, 1);
  Serial.print(",BASE=");
  Serial.println(baseThrottleUs);

  setAutoMotorSpeeds(leftSpeed, rightSpeed);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("BOOT,AUTO_NAV");

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

  Serial.println("READY,COMMANDS=GOTO,lat,lon|STOP|STATUS");
}

void loop() {
  updateGPS();
  readSerialCommands();
  runAutonomousController();

  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    printStatus();
  }

  delay(50);
}
