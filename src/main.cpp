#include <Arduino.h>
#include <TMCStepper.h>
#include <HardwareSerial.h>

#define EN_PIN 12           // Driver Enable pin
#define LED_PIN 2           // D1 Mini Hardware LED
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address
#define R_SENSE 0.11f       // SilentStepStick series use 0.11

#define SW_RX 16 // Software RX -- must be interrupt capable
#define SW_TX 18 // Software TX

#define MICROSTEPS 4    // Microstepping
#define RMS_CURRENT 636 // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A = 636mA RMS

HardwareSerial hardwareSerial(2);
TMC2209Stepper driver(&hardwareSerial, R_SENSE, DRIVER_ADDRESS); // Create TMC driver

void setup()
{
  Serial.begin(115200);
  hardwareSerial.begin(115200, 134217756U, SW_RX, SW_TX);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.microsteps(4);
}

void loop()
{
  Serial.printf("Version %d, Microsteps: %d\n", driver.version(), driver.microsteps());
  delay(1000);
}