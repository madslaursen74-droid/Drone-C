#include <Arduino.h>
#include <ESP32Servo.h>

Servo leftEsc;
Servo rightEsc;

constexpr int LEFT_ESC_PIN = 18;
constexpr int RIGHT_ESC_PIN = 4;

constexpr int ESC_NEUTRAL_US = 1500;
constexpr int ESC_TEST_FORWARD_US = 1560;
constexpr int ESC_ARM_DELAY_MS = 3000;
constexpr int TEST_RUN_MS = 2000;
constexpr int TEST_PAUSE_MS = 2000;

void setBothNeutral() {
  leftEsc.writeMicroseconds(ESC_NEUTRAL_US);
  rightEsc.writeMicroseconds(ESC_NEUTRAL_US);
  Serial.println("Both ESCs -> NEUTRAL");
}

void runLeftMotorTest() {
  Serial.println("Testing LEFT motor...");
  leftEsc.writeMicroseconds(ESC_TEST_FORWARD_US);
  rightEsc.writeMicroseconds(ESC_NEUTRAL_US);
  delay(TEST_RUN_MS);
  setBothNeutral();
}

void runRightMotorTest() {
  Serial.println("Testing RIGHT motor...");
  leftEsc.writeMicroseconds(ESC_NEUTRAL_US);
  rightEsc.writeMicroseconds(ESC_TEST_FORWARD_US);
  delay(TEST_RUN_MS);
  setBothNeutral();
}

void runBothMotorsTest() {
  Serial.println("Testing BOTH motors...");
  leftEsc.writeMicroseconds(ESC_TEST_FORWARD_US);
  rightEsc.writeMicroseconds(ESC_TEST_FORWARD_US);
  delay(TEST_RUN_MS);
  setBothNeutral();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESC ground test starting");
  Serial.println("LEFT ESC pin = GPIO18");
  Serial.println("RIGHT ESC pin = GPIO4");

  leftEsc.setPeriodHertz(50);
  rightEsc.setPeriodHertz(50);

  leftEsc.attach(LEFT_ESC_PIN, 1000, 2000);
  rightEsc.attach(RIGHT_ESC_PIN, 1000, 2000);

  setBothNeutral();
  Serial.println("Arming ESCs at neutral...");
  delay(ESC_ARM_DELAY_MS);
  Serial.println("ESC arming complete");
}

void loop() {
  runLeftMotorTest();
  delay(TEST_PAUSE_MS);

  runRightMotorTest();
  delay(TEST_PAUSE_MS);

  runBothMotorsTest();
  delay(4000);
}
