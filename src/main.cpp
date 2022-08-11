// TODO: Use "freewheel" to stop motor 0x01


#include <Arduino.h>
#include <WiFiManager.h> 
#include <TMCStepper.h>           // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <SoftwareSerial.h>

// My TMC https://www.aliexpress.com/item/1005002965406330.html

#define LED_PIN           2       // Visual feedback
#define EN_PIN            0       // Enable pin
#define DRIVER_ADDRESS    0b00    // TMC2209 Driver address
#define R_SENSE           0.11f   // SilentStepStick series use 0.11
//#define DRIVER_SERIAL     Serial  // TMC2208/TMC2224 HardwareSerial port
#define SWITCH_PIN        13      // Physical input

#define SW_RX             14       // Software RX
#define SW_TX             12       // Software TX

#define MICROSTEPS        0       // Microstepping
#define RMS_CURRENT       636     // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A = 636mA RMS

#define MOTOR_SPEED       1000    // Motor speed. Multiply by microsteps for consistency
#define MOTOR_ACCEL       10      // Acceleration

WiFiManager wifiManager;
SoftwareSerial DRIVER_SERIAL(SW_RX,SW_TX);
TMC2209Stepper driver(&DRIVER_SERIAL, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver

void configureDriver() {
  pinMode(EN_PIN, OUTPUT);
  DRIVER_SERIAL.begin(115200);

  driver.begin();

  driver.toff(5);                       // Enables driver in software
  driver.rms_current(RMS_CURRENT);      // Set motor RMS current (mA)
  driver.microsteps(MICROSTEPS);        // Set microsteps

  driver.intpol(true);                  // Interpolate to 256
  driver.TCOOLTHRS(0);                  // Disable coolstep
  driver.ihold(0);                      // hold current 0-31

  driver.freewheel(1);                  // Freewheel stop

  driver.en_spreadCycle(false);         // Toggle spreadCycle
  driver.pwm_autoscale(true);           // Needed for stealthChop

  driver.VACTUAL(0);                    // Ensure motor is stopped
  digitalWrite(EN_PIN, HIGH);           // Start disabled

}

void configureWiFi() {
  if (false) { // TODO configure a boot process that clears wifi config, i.e. if both endstops are pressed.
    wifiManager.erase();
  }
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.autoConnect();
}

enum CoverState {
  COVER_STOPPED,
  COVER_CLOSED,
  COVER_OPENING,
  COVER_CLOSING,
  COVER_OPEN
};

CoverState state = COVER_STOPPED;

unsigned long switchDebounce = 0;
void IRAM_ATTR switchPressed() {  
  if (millis() - switchDebounce < 50) return;
  switchDebounce = millis();

  switch (state) {
    case COVER_OPEN:
      state = COVER_CLOSING;
      break;
    case COVER_STOPPED:
    case COVER_CLOSED:
      state = COVER_OPENING;
      break;
    case COVER_CLOSING:
      state = COVER_CLOSED;
      break;
    case COVER_OPENING:
      state = COVER_OPEN;
      break;
    default:
      state = COVER_STOPPED;
      break;
  };
}

void enableMotor(bool state) {
  digitalWrite(EN_PIN, state ? LOW : HIGH);
}

void move() {
  switch (state) {
    case COVER_OPENING:
      //digitalWrite(LED_PIN, LOW);
      enableMotor(true);
      driver.VACTUAL(-MOTOR_SPEED);
      break;
    case COVER_CLOSING:
      //digitalWrite(LED_PIN, LOW);
      enableMotor(true);
      driver.VACTUAL(MOTOR_SPEED);
      break;
    default:
      //digitalWrite(LED_PIN, HIGH);
      enableMotor(false);
      driver.VACTUAL(0);
      break;
  };
  delay(10);
}

void setup() {
  Serial.begin(115200); 
  pinMode(LED_PIN, OUTPUT);

  configureWiFi();
  configureDriver();

  pinMode(SWITCH_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switchPressed, RISING);
}

uint64 counter = 0;

void loop() {
  wifiManager.process();
  move();
  
  counter++;
  if (counter%100 == 0) {
    if (driver.test_connection() == 2) {
      digitalWrite(LED_PIN, HIGH);
    }
    else {
      // led on is good
      digitalWrite(LED_PIN, LOW);
    }


      Serial.print("v=");
      Serial.println(driver.test_connection(), HEX);

  }
}
