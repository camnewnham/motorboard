#include <Arduino.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <Ticker.h>

#define EN_PIN 15           // Driver Enable pin
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address
#define R_SENSE 0.11f       // SilentStepStick series use 0.11

#define SW_RX 2 // Software RX -- must be interrupt capable
#define SW_TX 0 // Software TX

#define DIAG_PIN 13 // MAY NEED TO SWAP WITH 12 DEPENDING ON BOARD CONF

#define STALL_VALUE     100 // [0..255]

#define MICROSTEPS 0    // Microstepping
#define RMS_CURRENT 636 // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A = 636mA RMS

SoftwareSerial softwareSerial(SW_RX, SW_TX);
TMC2209Stepper driver(&softwareSerial, R_SENSE, DRIVER_ADDRESS); // Create TMC driver

Ticker resetter;

bool interrupted = false;

bool shaft = false;

void IRAM_ATTR onStall() {
    interrupted = true;
}

void setup()
{
  Serial.begin(115200);

  while (!Serial)
    ;

  softwareSerial.begin(57600);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT);

  driver.begin();

  driver.toff(4);
  driver.blank_time(24);
  driver.microsteps(MICROSTEPS);
  driver.rms_current(RMS_CURRENT);

// STALLGUARD CONFIG
  driver.pwm_autoscale(true); 
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.TCOOLTHRS(0xFFFFF); 
  driver.SGTHRS(STALL_VALUE);
  
  digitalWrite(EN_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), onStall, RISING);
  
  driver.VACTUAL(150);
}


void loop()
{
  Serial.printf("SG: %d. DIAG: %d\n", driver.SG_RESULT(), digitalRead(DIAG_PIN));
  //Serial.printf("Conn %d, Version %d, Microsteps: %d\n", driver.test_connection(), driver.version(), driver.microsteps());
  if (interrupted || digitalRead(DIAG_PIN) == HIGH) {
    interrupted = false;
    Serial.println("Interrupted!");
    shaft = !shaft;
    driver.shaft(shaft);
    // Reset the interrupt
    digitalWrite(EN_PIN, HIGH);
    delay(10);
    digitalWrite(EN_PIN, LOW);
  }
  delay(100);

}