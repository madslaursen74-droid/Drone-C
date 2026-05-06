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

  leftEsc.writeMicroseconds(leftSpeed);
  rightEsc.writeMicroseconds(rightSpeed);

  Serial.print("MOTORS,LEFT=");
  Serial.print(leftSpeed);
  Serial.print(",RIGHT=");
  Serial.print(rightSpeed);
  Serial.print(",SOURCE=");
  Serial.println(source);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, kEscNeutralUs, kMaxForwardUs);
  rightSpeed = constrain(rightSpeed, kEscNeutralUs, kMaxForwardUs);
  writeEscMicroseconds(leftSpeed, rightSpeed, "AUTO");
}

void setManualThrusterAngles(uint32_t leftAngle, uint32_t rightAngle, const char* source) {
  if (leftAngle > kThrusterMaxAngle || rightAngle > kThrusterMaxAngle) {
    Serial.println("ERROR,THRUST_RANGE_0_180");
    return;
  }

  currentMode = BoatMode::Manual;
  writeEscMicroseconds(
      thrusterAngleToMicroseconds(leftAngle),
      thrusterAngleToMicroseconds(rightAngle),
      source);
}

void stopBoat() {
  currentMode = BoatMode::Idle;
  writeEscMicroseconds(kEscNeutralUs, kEscNeutralUs, "STOP");
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

  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
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

  Serial.print(",ESPNOW=");
  Serial.print(espNowReady ? "READY" : "OFF");

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

bool parseInteger(const String& text, int& value) {
  String trimmed = text;
  trimmed.trim();
  if (trimmed.length() == 0) return false;

  int startIndex = 0;
  if (trimmed[0] == '-' || trimmed[0] == '+') {
    if (trimmed.length() == 1) return false;
    startIndex = 1;
  }

  for (int i = startIndex; i < trimmed.length(); ++i) {
    if (!isDigit(trimmed[i])) return false;
  }

  value = trimmed.toInt();
  return true;
}

void handleThrustCommand(const String& payload) {
  const int commaIndex = payload.indexOf(',');
  if (commaIndex < 0) {
    Serial.println("ERROR,INVALID_THRUST_FORMAT");
    return;
  }

  int leftAngle = 0;
  int rightAngle = 0;
  if (!parseInteger(payload.substring(0, commaIndex), leftAngle) ||
      !parseInteger(payload.substring(commaIndex + 1), rightAngle)) {
    Serial.println("ERROR,INVALID_THRUST_FORMAT");
    return;
  }

  setManualThrusterAngles(leftAngle, rightAngle, "SERIAL");
  Serial.print("ACK,THRUST,LEFT=");
  Serial.print(leftAngle);
  Serial.print(",RIGHT=");
  Serial.println(rightAngle);
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

  if (command.equalsIgnoreCase("AUTO")) {
    if (!targetWaypoint.valid) {
      Serial.println("ERROR,NO_TARGET");
      return;
    }

    currentMode = BoatMode::Auto;
    Serial.println("ACK,AUTO");
    return;
  }

  if (command.startsWith("GOTO,")) {
    handleGotoCommand(command.substring(5));
    return;
  }

  if (command.startsWith("THRUST,")) {
    handleThrustCommand(command.substring(7));
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

bool isValidLatitudeLongitude(double latitude, double longitude) {
  return isfinite(latitude) && isfinite(longitude) &&
         latitude >= -90.0 && latitude <= 90.0 &&
         longitude >= -180.0 && longitude <= 180.0;
}

void printRecoveredPacket(const RecoveredControlPacket& packet, int len) {
  Serial.print("ESPNOW_PACKET,LEN=");
  Serial.print(len);
  Serial.print(",TIME=");
  Serial.print(packet.time);
  Serial.print(",LEFT=");
  Serial.print(packet.leftThruster);
  Serial.print(",RIGHT=");
  Serial.print(packet.rightThruster);
  Serial.print(",NAV_MODE=");
  Serial.print(packet.navigationMode);
  Serial.print(",WP_FUNC=");
  Serial.print(packet.waypointFunction);
  Serial.print(",WP_ID=");
  Serial.print(packet.waypointId);
  Serial.print(",WP_LAT_E7=");
  Serial.print(packet.waypointLatitude);
  Serial.print(",WP_LON_E7=");
  Serial.println(packet.waypointLongitude);
}

void applyRecoveredWaypoint(const RecoveredControlPacket& packet) {
  const double latitude = static_cast<double>(packet.waypointLatitude) / kWaypointE7Scale;
  const double longitude = static_cast<double>(packet.waypointLongitude) / kWaypointE7Scale;

  if (!isValidLatitudeLongitude(latitude, longitude)) {
    Serial.print("WARN,ESPNOW_WAYPOINT_INVALID,LAT=");
    Serial.print(latitude, 7);
    Serial.print(",LON=");
    Serial.println(longitude, 7);
    return;
  }

  targetWaypoint.latitude = latitude;
  targetWaypoint.longitude = longitude;
  targetWaypoint.valid = true;
  currentMode = BoatMode::Auto;

  Serial.print("TARGET_SET,SOURCE=ESPNOW,ID=");
  Serial.print(packet.waypointId);
  Serial.print(",LAT=");
  Serial.print(targetWaypoint.latitude, 7);
  Serial.print(",LON=");
  Serial.println(targetWaypoint.longitude, 7);
}

void processRecoveredControlPacket(const RecoveredControlPacket& packet, int len) {
  printRecoveredPacket(packet, len);

  if (packet.waypointFunction != 0) {
    applyRecoveredWaypoint(packet);
  }

  if (kAllowEspNowManualThrusterOverride &&
      packet.leftThruster <= kThrusterMaxAngle &&
      packet.rightThruster <= kThrusterMaxAngle) {
    setManualThrusterAngles(packet.leftThruster, packet.rightThruster, "ESPNOW");
  }
}

void queueRecoveredControlPacket(const uint8_t* incomingData, int len) {
  if (incomingData == nullptr || len < static_cast<int>(sizeof(RecoveredControlPacket))) {
    return;
  }

  RecoveredControlPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  portENTER_CRITICAL(&espNowPacketMux);
  pendingEspNowPacket = packet;
  pendingEspNowPacketLength = len;
  hasPendingEspNowPacket = true;
  portEXIT_CRITICAL(&espNowPacketMux);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowDataRecv(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len) {
  (void)info;
  queueRecoveredControlPacket(incomingData, len);
}
#else
void onEspNowDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  (void)mac;
  queueRecoveredControlPacket(incomingData, len);
}
#endif

void pollEspNowPacket() {
  RecoveredControlPacket packet;
  int len = 0;
  bool shouldProcess = false;

  portENTER_CRITICAL(&espNowPacketMux);
  if (hasPendingEspNowPacket) {
    packet = pendingEspNowPacket;
    len = pendingEspNowPacketLength;
    hasPendingEspNowPacket = false;
    shouldProcess = true;
  }
  portEXIT_CRITICAL(&espNowPacketMux);

  if (shouldProcess) {
    processRecoveredControlPacket(packet, len);
  }
}

void setupEspNowReceiver() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNOW,NOT_READY");
    return;
  }

  espNowReady = true;
  esp_now_register_recv_cb(onEspNowDataRecv);
  Serial.println("ESPNOW,READY");
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

  float correction = kHeadingKp * headingError;

  int baseThrottleUs = kBaseForwardUs;
  if (distanceMeters < 8.0) {
    baseThrottleUs = kNearTargetForwardUs;
  }

  if (fabs(headingError) > kStrongTurnThresholdDegrees) {
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

  setMotorSpeeds(leftSpeed, rightSpeed);
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
