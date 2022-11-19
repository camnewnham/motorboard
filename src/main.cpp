// TODO: Use "freewheel" to stop motor 0x01


#include <Arduino.h>
#include <WiFiManager.h> 
#include <TMCStepper.h>           // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <SoftwareSerial.h>

// My TMC https://www.aliexpress.com/item/1005002965406330.html

#define EN_PIN            5       // Driver Enable pin
#define LED_PIN           2       // D1 Mini Hardware LED
#define DRIVER_ADDRESS    0b00    // TMC2209 Driver address
#define R_SENSE           0.11f   // SilentStepStick series use 0.11
#define SWITCH_PIN        13      // Physical input

#define SW_RX             14      // Software RX
#define SW_TX             12      // Software TX

#define MICROSTEPS        0       // Microstepping
#define RMS_CURRENT       636     // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A = 636mA RMS

#define MOTOR_SPEED       960     // Motor speed. Multiply by microsteps for consistency

enum CoverState {
  COVER_STOPPED,
  COVER_CLOSED,
  COVER_OPENING,
  COVER_CLOSING,
  COVER_OPEN
};

WiFiManager wifiManager;

SoftwareSerial DRIVER_SERIAL(SW_RX,SW_TX);
TMC2209Stepper driver(&DRIVER_SERIAL, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver

void configureDriver() {
  pinMode(LED_PIN, OUTPUT);
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

  //driver.en_spreadCycle(true);        // Toggle spreadCycle
  driver.pwm_autoscale(true);           // Needed for stealthChop

  driver.VACTUAL(0);                    // Ensure motor is stopped
  digitalWrite(EN_PIN, LOW);            // Start disabled

}

void configureWiFi() {
  if (false) { // TODO configure a boot process that clears wifi config, i.e. if both endstops are pressed.
    wifiManager.erase();
  }

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.autoConnect();
}

CoverState next_state = COVER_STOPPED;
CoverState current_state = COVER_STOPPED;

void setMotorState() {
 if (next_state == current_state) return;
  current_state = next_state;

  switch (current_state) {
    case COVER_OPENING:
      digitalWrite(EN_PIN, LOW);
      driver.VACTUAL(-MOTOR_SPEED);
      break;
    case COVER_CLOSING:
      digitalWrite(EN_PIN, LOW);
      driver.VACTUAL(MOTOR_SPEED);
      break;
    default:
      driver.VACTUAL(0);
      digitalWrite(EN_PIN, HIGH);
      break;
  };
}

uint8_t switch_state = HIGH;

void checkSwitch() {
  if (digitalRead(SWITCH_PIN) == switch_state) return;
  switch_state = digitalRead(SWITCH_PIN);

  if (switch_state == HIGH) return; // Trigger on press (not release)

  switch (current_state) {
      case COVER_OPEN:
        next_state = COVER_CLOSING;
        Serial.println("Cover closing");
        break;
      case COVER_STOPPED:
      case COVER_CLOSED:
        next_state = COVER_OPENING;
        Serial.println("Cover opening");
        break;
      case COVER_CLOSING:
        next_state = COVER_CLOSED;
        Serial.println("Cover closed");
        break;
      case COVER_OPENING:
        next_state = COVER_OPEN;
        Serial.println("Cover open");
        break;
      default:
        next_state = COVER_STOPPED;
        break;
    };
}

void setup() {
  Serial.begin(115200); 

  configureWiFi();
  configureDriver();

  pinMode(SWITCH_PIN, INPUT);
  switch_state = digitalRead(SWITCH_PIN);
}

uint64_t counter = 0;

void loop() {
  checkSwitch();
  setMotorState();
  
  wifiManager.process();
  
  counter++;
  if (counter%1000 == 0) {
      digitalWrite(LED_PIN, driver.test_connection() == 2 ? HIGH : LOW);
      //Serial.print("Status: "); Serial.println(driver.test_connection());
  }
}