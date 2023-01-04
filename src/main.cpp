#include <Arduino.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

#define EN_PIN 12           // Driver Enable pin
#define LED_PIN 2           // D1 Mini Hardware LED
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address
#define R_SENSE 0.11f       // SilentStepStick series use 0.11

#define SW_RX 14 // Software RX -- must be interrupt capable
#define SW_TX 12 // Software TX

#define MICROSTEPS 4    // Microstepping
#define RMS_CURRENT 636 // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A = 636mA RMS

SoftwareSerial softwareSerial(SW_RX, SW_TX);
TMC2209Stepper driver(&softwareSerial, R_SENSE, DRIVER_ADDRESS); // Create TMC driver

void setup()
{
  Serial.begin(115200);

  while (!Serial)
    ;

  softwareSerial.begin(57600);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  driver.begin();

  driver.toff(5);
  driver.microsteps(16);
  driver.VACTUAL(1000);
}

bool shaft = false;

void loop()
{
  Serial.printf("Conn %d, Version %d, Microsteps: %d\n", driver.test_connection(), driver.version(), driver.microsteps());
  delay(1000);
  shaft = !shaft;
  driver.shaft(shaft);
}