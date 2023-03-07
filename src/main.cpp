// Board v1.0 notes:
// Due to miswiring, pin 0 must be manually disconnected during programming

#include <Arduino.h>
#include <WiFiManager.h> 
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <ArduinoHA.h>
#include <EEPROM.h>

#define EN_PIN            15       // Driver Enable pin
#define DRIVER_ADDRESS    0b00    // TMC2209 Driver address
#define R_SENSE           0.11f   // SilentStepStick series use 0.11

#define SWITCH_PIN_0      14
#define SWITCH_PIN_1      16

#define SW_RX             2      // Software RX
#define SW_TX             0      // Software TX
#define DIAG_PIN 13 // MAY NEED TO SWAP WITH 12 DEPENDING ON BOARD CONF
#define STALL_VALUE     100 // [0..255]

#define MICROSTEPS        0       // Microstepping
#define RMS_CURRENT       900     // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A = 636mA RMS

// Motor speed. Multiply by microsteps for consistency
#define MOTOR_ACCEL       6       // Acceleration steps per microprocessor frequency

#define MAX_MOVE_DURATION 30000   // Number of milliseconds before a safety stop is triggered

#define MQTT_SERVER       "homeassistant.local"
#define MQTT_USER         "user"
#define MQTT_PASS         "passwd"

WiFiClient wifiClient;   // MQTT Controller

HADevice haDevice;
HAMqtt mqtt(wifiClient, haDevice, 4); // Num devices

HACover haCover("cover", HACover::Features::PositionFeature);
HANumber haSpeed("speed", HABaseDeviceType::NumberPrecision::PrecisionP0);
HANumber haStall("stall", HABaseDeviceType::NumberPrecision::PrecisionP0);

WiFiManager wifiManager; // WiFi AP controller

SoftwareSerial softwareSerial(SW_RX, SW_TX);
TMC2209Stepper driver(&softwareSerial, R_SENSE, DRIVER_ADDRESS); // Create TMC driver

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

bool stall_triggered = false;

uint64_t move_start_time = 0;

bool mqtt_initialized = false;

uint16_t move_speed = 500; 
uint8_t stall_threshold = 100;

void IRAM_ATTR onStall() {
    stall_triggered = true;
}

void configureDriver() {
  softwareSerial.begin(57600);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT);

  driver.begin();

  driver.toff(4);
  driver.blank_time(24);
  driver.microsteps(MICROSTEPS);
  driver.rms_current(RMS_CURRENT);
  driver.en_spreadCycle(false);         // Toggle spreadCycle
  driver.pwm_autoscale(true);           // Needed for stealthChop

  driver.ihold(0);                      // hold current 0-31
  driver.freewheel(1);                  // Freewheel stop

  // STALLGUARD CONFIG
  driver.pwm_autoscale(true); 
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.TCOOLTHRS(0xFFFFF); 
  driver.SGTHRS(STALL_VALUE);

  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), onStall, RISING);

  driver.VACTUAL(0);                    // Ensure motor is stopped
  digitalWrite(EN_PIN, HIGH);           // Start disabled

}

void configureWiFiAP() {
  // Collect some details for MQTT
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(600);

  if (digitalRead(SWITCH_PIN_0) == LOW && digitalRead(SWITCH_PIN_1) == LOW) { // Open portal if both endstops are pressed at boot
    wifiManager.startConfigPortal("Coverboard");
  }
  else {
    wifiManager.autoConnect("Coverboard");
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

void onStallThresholdChange(HANumeric cmd, HANumber* num) {
  stall_threshold = cmd.toUInt8();
  Serial.printf("stall_threshold set to %d\n", stall_threshold);
  driver.SGTHRS(stall_threshold);
}

void configureMQTT() {
    byte mac[WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);

    haDevice.setUniqueId(mac, sizeof(mac));

    haDevice.setName("Motorboard");
    haDevice.setModel("Motorboard V1.1");
    haDevice.setManufacturer("CDN");
    haDevice.setSoftwareVersion("1.1.0");

    haCover.onCommand(onCoverCommand);
    haCover.setName("Cover");

    haSpeed.setName("Speed");
    haSpeed.setMin(0);
    haSpeed.setMax(2000);
    haSpeed.setMode(HANumber::Mode::ModeSlider);
    haSpeed.onCommand(onSpeedChange);
    haSpeed.setOptimistic(true);
    haSpeed.setState(move_speed);
    haSpeed.setRetain(true);

    haStall.setName("Stall");
    haStall.setMin(0);
    haStall.setMax(255);
    haStall.setMode(HANumber::Mode::ModeSlider);
    haStall.onCommand(onStallThresholdChange);
    haStall.setOptimistic(true);
    haStall.setState(stall_threshold);
    haStall.setRetain(true);

    mqtt.begin(MQTT_SERVER, 1883U, MQTT_USER, MQTT_PASS);

    Serial.printf("Begin MQTT on server \"%s\" with user \"%s\" and pass \"%s\"\n", MQTT_SERVER, MQTT_USER, MQTT_PASS);
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

void loopMotor() {
  if (stall_triggered) {
    Serial.printf("Stall triggered\n");
    // Reset 
    stall_triggered = false;
    digitalWrite(EN_PIN, HIGH);
    delay(100);
    digitalWrite(EN_PIN, LOW);
    switch (cover_state_current) {
      case COVER_OPENING:
        cover_state_next = CoverState::COVER_OPEN;
        break;
      case COVER_CLOSING:
        cover_state_next = CoverState::COVER_CLOSED;
        break;
      default:
        cover_state_next = CoverState::COVER_STOPPED;
        break;
    }
  }

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

void setup() {
  Serial.begin(115200); 
  while (!Serial)
    ;
  pinMode(SWITCH_PIN_0, INPUT_PULLUP);
  pinMode(SWITCH_PIN_1, INPUT_PULLUP);
  configureWiFiAP();
  configureDriver();
}

void loop() {
  if (millis() - move_start_time > MAX_MOVE_DURATION) { // Safety stop
    cover_state_next = COVER_STOPPED;
  }

  wifiManager.process();

  loopMQTT();
  loopMotor();
}