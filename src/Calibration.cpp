#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Preferences.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
Preferences prefs;

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

constexpr char CALIB_NAMESPACE[] = "bno055";
constexpr char CALIB_KEY[] = "calib";

bool savedThisBoot = false;
bool loadedOnBoot = false;

bool loadCalibration()
{
  adafruit_bno055_offsets_t offsets;

  prefs.begin(CALIB_NAMESPACE, true);

  if (!prefs.isKey(CALIB_KEY)) {
    prefs.end();
    Serial.println("Saved calibration: NONE");
    return false;
  }

  size_t savedSize = prefs.getBytesLength(CALIB_KEY);
  if (savedSize != sizeof(offsets)) {
    prefs.end();
    Serial.println("Saved calibration: WRONG SIZE");
    return false;
  }

  prefs.getBytes(CALIB_KEY, &offsets, sizeof(offsets));
  prefs.end();

  bno.setSensorOffsets(offsets);
  Serial.println("Saved calibration: LOADED");
  return true;
}

bool isFullyCalibrated()
{
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  return sys == 3 && gyro == 3 && accel == 3 && mag == 3;
}

void saveCalibrationIfReady()
{
  if (savedThisBoot || !isFullyCalibrated()) {
    return;
  }

  adafruit_bno055_offsets_t offsets;
  if (!bno.getSensorOffsets(offsets)) {
    Serial.println("Calibration save failed: sensor offsets not ready");
    return;
  }

  prefs.begin(CALIB_NAMESPACE, false);
  size_t bytesWritten = prefs.putBytes(CALIB_KEY, &offsets, sizeof(offsets));
  prefs.end();

  if (bytesWritten != sizeof(offsets)) {
    Serial.println("Calibration save failed: flash write failed");
    return;
  }

  savedThisBoot = true;
  Serial.println("Calibration SAVED to Preferences: bno055/calib");
}

void printCalibrationStatus()
{
  sensors_event_t event;
  bno.getEvent(&event);

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.println();
  Serial.println("---- BNO055 CALIBRATION ----");

  Serial.print("Heading: ");
  Serial.println(event.orientation.x, 2);

  Serial.print("Pitch: ");
  Serial.println(event.orientation.y, 2);

  Serial.print("Roll: ");
  Serial.println(event.orientation.z, 2);

  Serial.print("Calibration SYS/GYRO/ACCEL/MAG: ");
  Serial.print(sys); Serial.print("/");
  Serial.print(gyro); Serial.print("/");
  Serial.print(accel); Serial.print("/");
  Serial.println(mag);

  Serial.print("Loaded On Boot: ");
  Serial.println(loadedOnBoot ? "YES" : "NO");

  Serial.print("Saved This Boot: ");
  Serial.println(savedThisBoot ? "YES" : "NO");

  if (isFullyCalibrated()) {
    Serial.println("Status: READY - calibration can be saved");
  }
  else {
    Serial.println("Status: CALIBRATE - move/rotate sensor slowly");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Calibration firmware starting");
  Serial.println("This saves BNO055 offsets for Drone_Nav.cpp");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected - check wiring or I2C address");
    while (1);
  }

  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);
  bno.setExtCrystalUse(true);

  loadedOnBoot = loadCalibration();

  Serial.println("Hold still for gyro, then rotate/tilt slowly until 3/3/3/3");
}

void loop()
{
  printCalibrationStatus();
  saveCalibrationIfReady();
  delay(1000);
}
