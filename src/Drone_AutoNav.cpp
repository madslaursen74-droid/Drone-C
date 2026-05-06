#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

Servo leftEsc;
Servo rightEsc;

constexpr int GPS_RX = 17;
constexpr int LEFT_ESC_PIN = 18;
constexpr int RIGHT_ESC_PIN = 4;

constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 2000;
constexpr int ESC_NEUTRAL_US = 1500;
constexpr int ESC_FORWARD_US = 1640;
constexpr int ESC_SLOW_US = 1575;

constexpr float ARRIVAL_RADIUS_METERS = 3.0f;
constexpr float HEADING_KP = 2.0f;
constexpr float HEADING_DEADBAND = 4.0f;

double targetLat = 0.0;
double targetLon = 0.0;
bool hasTarget = false;

void updateGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

bool hasGoodGPSFix() {
  if (!gps.location.isValid()) return false;
  if (gps.satellites.value() < 6) return false;
  if (gps.location.age() > 2000) return false;
  return true;
}

bool readHeading(float& heading) {
  sensors_event_t event;
  bno.getEvent(&event);

  if (!isfinite(event.orientation.x)) {
    return false;
  }

  heading = event.orientation.x;
  if (heading < 0) {
    heading += 360.0f;
  }

  return true;
}

double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double earthRadius = 6371000.0;

  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  const double dLat = lat2 - lat1;
  const double dLon = lon2 - lon1;

  const double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
                   cos(lat1) * cos(lat2) *
                   sin(dLon / 2.0) * sin(dLon / 2.0);

  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return earthRadius * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  const double dLon = lon2 - lon1;
  const double y = sin(dLon) * cos(lat2);
  const double x = cos(lat1) * sin(lat2) -
                   sin(lat1) * cos(lat2) * cos(dLon);

  double bearing = degrees(atan2(y, x));
  if (bearing < 0) {
    bearing += 360.0;
  }

  return bearing;
}

float angleDifference(float target, float current) {
  float diff = target - current;

  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;

  return diff;
}

void setMotorSignals(int leftUs, int rightUs, const char* source) {
  leftUs = constrain(leftUs, ESC_MIN_US, ESC_MAX_US);
  rightUs = constrain(rightUs, ESC_MIN_US, ESC_MAX_US);

  leftEsc.writeMicroseconds(leftUs);
  rightEsc.writeMicroseconds(rightUs);

  Serial.print("MOTORS,LEFT=");
  Serial.print(leftUs);
  Serial.print(",RIGHT=");
  Serial.print(rightUs);
  Serial.print(",SOURCE=");
  Serial.println(source);
}

void stopBoat() {
  setMotorSignals(ESC_NEUTRAL_US, ESC_NEUTRAL_US, "STOP");
}

void printStatus() {
  Serial.print("STATUS,MODE=");
  Serial.print(hasTarget ? "AUTO" : "IDLE");

  if (hasGoodGPSFix()) {
    Serial.print(",LAT=");
    Serial.print(gps.location.lat(), 7);
    Serial.print(",LON=");
    Serial.print(gps.location.lng(), 7);
  } else {
    Serial.print(",GPS=NO_FIX");
  }

  float heading = 0.0f;
  if (readHeading(heading)) {
    Serial.print(",HEADING=");
    Serial.print(heading, 1);
  } else {
    Serial.print(",HEADING=NA");
  }

  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print(",CAL=");
  Serial.print(sys);
  Serial.print("/");
  Serial.print(gyro);
  Serial.print("/");
  Serial.print(accel);
  Serial.print("/");
  Serial.print(mag);

  if (hasTarget) {
    Serial.print(",TARGET_LAT=");
    Serial.print(targetLat, 7);
    Serial.print(",TARGET_LON=");
    Serial.print(targetLon, 7);
  }

  Serial.println();
}

void setTargetFromCommand(const String& payload) {
  const int commaIndex = payload.indexOf(',');
  if (commaIndex < 0) {
    Serial.println("ERROR,INVALID_GOTO_FORMAT");
    return;
  }

  targetLat = payload.substring(0, commaIndex).toDouble();
  targetLon = payload.substring(commaIndex + 1).toDouble();
  hasTarget = true;

  Serial.print("TARGET_SET,LAT=");
  Serial.print(targetLat, 7);
  Serial.print(",LON=");
  Serial.println(targetLon, 7);
}

void readSerialCommands() {
  static String input;

  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      input.trim();

      if (input.equalsIgnoreCase("STOP")) {
        hasTarget = false;
        stopBoat();
        Serial.println("ACK,STOP");
      } else if (input.equalsIgnoreCase("STATUS")) {
        printStatus();
      } else if (input.startsWith("GOTO,")) {
        setTargetFromCommand(input.substring(5));
      } else if (input.length() > 0) {
        Serial.print("ERROR,UNKNOWN_COMMAND=");
        Serial.println(input);
      }

      input = "";
    } else {
      input += c;
    }
  }
}

void runAutoNavigation() {
  if (!hasTarget) {
    return;
  }

  if (!hasGoodGPSFix()) {
    Serial.println("WARN,NO_GPS_FIX");
    stopBoat();
    return;
  }

  float currentHeading = 0.0f;
  if (!readHeading(currentHeading)) {
    Serial.println("WARN,NO_HEADING");
    stopBoat();
    return;
  }

  const double currentLat = gps.location.lat();
  const double currentLon = gps.location.lng();
  const double distance = calculateDistanceMeters(currentLat, currentLon, targetLat, targetLon);

  if (distance <= ARRIVAL_RADIUS_METERS) {
    Serial.println("ARRIVED");
    hasTarget = false;
    stopBoat();
    return;
  }

  const float targetBearing = static_cast<float>(calculateBearing(currentLat, currentLon, targetLat, targetLon));
  float error = angleDifference(targetBearing, currentHeading);

  if (fabs(error) < HEADING_DEADBAND) {
    error = 0.0f;
  }

  int baseSpeed = ESC_FORWARD_US;
  if (distance < 8.0 || fabs(error) > 35.0f) {
    baseSpeed = ESC_SLOW_US;
  }

  const int correction = static_cast<int>(round(HEADING_KP * error));
  const int leftSpeed = constrain(baseSpeed + correction, ESC_NEUTRAL_US, ESC_MAX_US);
  const int rightSpeed = constrain(baseSpeed - correction, ESC_NEUTRAL_US, ESC_MAX_US);

  Serial.print("AUTO,DIST=");
  Serial.print(distance, 2);
  Serial.print(",BEARING=");
  Serial.print(targetBearing, 1);
  Serial.print(",HEADING=");
  Serial.print(currentHeading, 1);
  Serial.print(",ERROR=");
  Serial.println(error, 1);

  setMotorSignals(leftSpeed, rightSpeed, "AUTO");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("BOOT,AUTO_NAV");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  leftEsc.setPeriodHertz(50);
  rightEsc.setPeriodHertz(50);
  leftEsc.attach(LEFT_ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  rightEsc.attach(RIGHT_ESC_PIN, ESC_MIN_US, ESC_MAX_US);

  stopBoat();
  Serial.println("ESC,ARMING_NEUTRAL");
  delay(3000);
  Serial.println("ESC,READY");

  Wire.begin(21, 22);
  Wire.setClock(100000);

  if (!bno.begin()) {
    Serial.println("BNO055,NOT_FOUND");
  } else {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055,READY");
  }

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX);

  Serial.println("READY,COMMANDS=GOTO,lat,lon|STOP|STATUS");
}

void loop() {
  updateGPS();
  readSerialCommands();
  runAutoNavigation();

  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 1000) {
    lastStatus = millis();
    printStatus();
  }

  delay(50);
}
