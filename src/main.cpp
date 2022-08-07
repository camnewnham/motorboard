#include <Arduino.h>
 

#include <TMCStepper.h>           // TMCstepper - https://github.com/teemuatlut/TMCStepper

#define LED_PIN           2       // Visual feedback
#define EN_PIN            0       // Enable pin
#define DRIVER_ADDRESS    0b00    // TMC2209 Driver address
#define R_SENSE           0.11f   // SilentStepStick series use 0.11
#define SERIAL_PORT       Serial  // TMC2208/TMC2224 HardwareSerial port

#define MICROSTEPS        0       // Microstepping
#define RMS_CURRENT       600     // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  SERIAL_PORT.begin(115200);

  digitalWrite(EN_PIN, LOW);            // Enable TMC2209 board

  driver.begin();
  driver.toff(5);                       // Enables driver in software
  driver.rms_current(RMS_CURRENT);      // Set motor RMS current (mA)
  driver.microsteps(MICROSTEPS);                 // Set microsteps

  driver.intpol(true);                  // Interpolate to 256
  driver.TCOOLTHRS(0);                  // Disable coolstep
  driver.ihold(0);                      // hold current 0-31

  driver.en_spreadCycle(false);          // Toggle spreadCycle
  driver.pwm_autoscale(true);           // Needed for stealthChop
}

int32_t curr_speed = 0;
int32_t dest_speed = 1000 * (MICROSTEPS < 1 ? 1 : MICROSTEPS);
int32_t acceleration = 10* (MICROSTEPS < 1 ? 1 : MICROSTEPS);

void ramp() {
  digitalWrite(LED_PIN, dest_speed > 0);
  while ((dest_speed > 0 && curr_speed < dest_speed) || (dest_speed < 0 && curr_speed > dest_speed)) {
    curr_speed += acceleration;
    driver.VACTUAL(curr_speed);
    delayMicroseconds(100);
  }
}

void loop() {
  ramp();
  delay(1000);
  dest_speed *= -1;
  acceleration *= -1;
}
