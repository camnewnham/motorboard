#include <Arduino.h>
#include <WiFiManager.h> 
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <ArduinoHA.h>

#define EN_PIN            15       // Driver Enable pin
#define R_SENSE           0.11f   // SilentStepStick series use 0.11


#define ENDSTOP_0_PIN       14       // Physical input
#define ENDSTOP_0_INVERTED  true
#define ENDSTOP_0_PINMODE   INPUT_PULLUP

#define ENDSTOP_1_PIN       16       // Physical input
#define ENDSTOP_1_INVERTED  false
#define ENDSTOP_1_PINMODE   INPUT_PULLDOWN_16

#define SW_RX             2      // Software RX
#define SW_TX             0      // Software TX

#define MICROSTEPS        0       // Microstepping
#define RMS_CURRENT       900     // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A = 636mA RMS

// Motor speed. Multiply by microsteps for consistency
#define MAX_SPEED         500 * (MICROSTEPS == 0 ? 1 : MICROSTEPS)     
#define MOTOR_ACCEL       2       // Acceleration steps per microprocessor frequency

#define MAX_MOVE_DURATION 30000   // Number of milliseconds before a safety stop is triggered

#define MQTT_SERVER "homeassistant.local"
#define MQTT_USERNAME "user"
#define MQTT_PASSWORD "passwd"

WiFiClient wifiClient;   // MQTT Controller
HADevice haDevice;
HAMqtt mqtt(wifiClient, haDevice, 4);
HACover haCover("coverboard", HACover::Features::PositionFeature);
HANumber haSpeed("coverboard_speed", HABaseDeviceType::NumberPrecision::PrecisionP0);

WiFiManager wifiManager; // WiFi AP controller

SoftwareSerial DRIVER_SERIAL(SW_RX,SW_TX);
TMC2209Stepper driver(&DRIVER_SERIAL, R_SENSE, 0b00);   // Create TMC driver

enum CoverState {
  COVER_STOPPED,
  COVER_CLOSED,
  COVER_OPENING,
  COVER_CLOSING,
  COVER_OPEN
};

CoverState cover_state_next = COVER_STOPPED;
CoverState cover_state_current = COVER_STOPPED;

int32_t current_speed = 0;
int32_t dest_speed = 0;

uint8_t endstop_0_state = LOW;
uint8_t endstop_1_state = LOW;

uint64_t move_start_time = 0;

bool mqtt_initialized = false;

int move_speed = MAX_SPEED;

void configureDriver() {
  pinMode(EN_PIN, OUTPUT);
  DRIVER_SERIAL.begin(57600);

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

void configureWiFiAP() {
  // Collect some details for MQTT

  wifiManager.setConfigPortalBlocking(false);

  if (endstop_0_state && endstop_1_state) { // Open portal if both endstops are pressed at boot
    wifiManager.startConfigPortal();
  }
  else {
    wifiManager.autoConnect();
  }
}

void onCoverCommand(HACover::CoverCommand cmd, HACover* sender) {
    if (cmd == HACover::CommandOpen) {
        cover_state_next = COVER_OPENING;
    } else if (cmd == HACover::CommandClose) {
        cover_state_next = COVER_CLOSING;
    } else if (cmd == HACover::CommandStop) {
        cover_state_next = COVER_STOPPED;
    }
}

void onSpeedChange(HANumeric cmd, HANumber* num) {
  move_speed = cmd.toUInt16();
  Serial.printf("move_speed set to %d\n", move_speed);
}

void configureMQTT() {
    byte mac[WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);

    haDevice.setUniqueId(mac, sizeof(mac));
    haDevice.setName("Coverboard");
    haDevice.setModel("Coverboard V1.1");
    haDevice.setManufacturer("CDN");
    haDevice.setSoftwareVersion("0.0.2");

    haCover.onCommand(onCoverCommand);
    haCover.setName("Coverboard");
    
    haSpeed.setName("Coverboard Speed");
    haSpeed.setMin(100);
    haSpeed.setMax(1000);
    haSpeed.setMode(HANumber::Mode::ModeSlider);
    haSpeed.onCommand(onSpeedChange);
    haSpeed.setOptimistic(true);
    haSpeed.setState(move_speed);
    haSpeed.setRetain(true);

    mqtt.begin(MQTT_SERVER, 1883U, MQTT_USERNAME, MQTT_PASSWORD);
}

void loopMQTT() {
  if (!mqtt_initialized)
  {
    if (WiFi.isConnected())
    {
      mqtt_initialized = true;
      configureMQTT();
    }
  }
  else {
    mqtt.loop();
  }
}

void setTargetSpeed(int32_t speed) {
  dest_speed = speed;

  if (dest_speed == 0) { // Stops are immediate
    current_speed = dest_speed;
    driver.VACTUAL(current_speed);
    digitalWrite(EN_PIN, HIGH);
  }
  else {    
    digitalWrite(EN_PIN, LOW);
  }
}

void setCoverState() {
  if (cover_state_next != cover_state_current) { // State change
  cover_state_current = cover_state_next;

    switch (cover_state_current) {
      case COVER_OPENING:
        move_start_time = millis();
        setTargetSpeed(move_speed);
        if (mqtt_initialized) 
        {
          haCover.setState(HACover::CoverState::StateOpening);
          haCover.setPosition(50);
        }
        Serial.println("Cover state: Opening"); 
        break;
      case COVER_CLOSING:
        move_start_time = millis();
        setTargetSpeed(-move_speed);
        if (mqtt_initialized) 
        {
          haCover.setState(HACover::CoverState::StateClosing);
          haCover.setPosition(0);
        }
        Serial.println("Cover state: Closing"); 
        break;
      case COVER_OPEN:
        setTargetSpeed(0);
        if (mqtt_initialized) 
        {
          haCover.setState(HACover::CoverState::StateOpen);
          haCover.setPosition(100);
        }
        Serial.println("Cover state: Open"); 
        break;
      case COVER_CLOSED:
        setTargetSpeed(0);
        if (mqtt_initialized) 
        {
          haCover.setState(HACover::CoverState::StateClosed);
          haCover.setPosition(0);
        }
        Serial.println("Cover state: Closed"); 
        break;
      case COVER_STOPPED:
      default:
        setTargetSpeed(0);
        if (mqtt_initialized) 
        {
          haCover.setState(HACover::CoverState::StateStopped);
          haCover.setPosition(50);
        }
        Serial.println("Cover state: Stopped"); 
        break;
    };
  }

  if (dest_speed > current_speed) { // Acceleration
    current_speed += MOTOR_ACCEL;
    driver.VACTUAL(current_speed);
    if (current_speed > dest_speed) current_speed = dest_speed;
  }
  else if (dest_speed < current_speed) {
    current_speed -= MOTOR_ACCEL;
    driver.VACTUAL(current_speed);
    if (current_speed < dest_speed) current_speed = dest_speed;
  }  
}

void checkEndstop(uint8_t newState, uint8_t *state, bool is_closed_endstop) {
  if (newState == *state) return;
  *state = newState;

  if (*state == LOW) return; // Trigger on press (not release)

  if (is_closed_endstop) 
  {
    switch (cover_state_current) {
      case COVER_STOPPED:
      case COVER_OPEN:
        cover_state_next = COVER_CLOSING;
        break;
      case COVER_CLOSING:
        cover_state_next = COVER_CLOSED;
        break;
      default:
        cover_state_next = COVER_STOPPED;
        break;
    };
  }
  else 
  {
    switch (cover_state_current) {
      case COVER_STOPPED:
      case COVER_CLOSED:
        cover_state_next = COVER_OPENING;
        break;
      case COVER_OPENING:
        cover_state_next = COVER_OPEN;
        break;
      default:
        cover_state_next = COVER_STOPPED;
        break;
    };
  }
}

void setup() {
  Serial.begin(115200); 
  pinMode(ENDSTOP_0_PIN, ENDSTOP_0_PINMODE);
  pinMode(ENDSTOP_1_PIN, ENDSTOP_1_PINMODE);
  endstop_0_state = digitalRead(ENDSTOP_0_PIN);
  if (ENDSTOP_0_INVERTED) endstop_0_state = !endstop_0_state;
  endstop_1_state = digitalRead(ENDSTOP_1_PIN);
  if (ENDSTOP_1_INVERTED) endstop_1_state = !endstop_1_state;

  configureWiFiAP();
  configureDriver();
}

void loop() {
  if (millis() - move_start_time > MAX_MOVE_DURATION) { // Safety stop
    cover_state_next = COVER_STOPPED;
  }

  wifiManager.process();
  loopMQTT();

  uint8_t e0state = digitalRead(ENDSTOP_0_PIN);
  if (ENDSTOP_0_INVERTED) e0state = !e0state;
  checkEndstop(e0state, &endstop_0_state, true);

  uint8_t e1state = digitalRead(ENDSTOP_1_PIN);
  if (ENDSTOP_1_INVERTED) e1state = !e1state;
  checkEndstop(e1state, &endstop_1_state, false);
  setCoverState();
}