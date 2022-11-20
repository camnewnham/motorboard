#include <Arduino.h>
#include <WiFiManager.h> 
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <ArduinoHA.h>

// My TMC https://www.aliexpress.com/item/1005002965406330.html

#define EN_PIN            5       // Driver Enable pin
#define LED_PIN           2       // D1 Mini Hardware LED
#define DRIVER_ADDRESS    0b00    // TMC2209 Driver address
#define R_SENSE           0.11f   // SilentStepStick series use 0.11
#define ENDSTOP_0_PIN     4       // Physical input
#define ENDSTOP_1_PIN     0       // Physical input

#define SW_RX             14      // Software RX
#define SW_TX             12      // Software TX

#define MICROSTEPS        0       // Microstepping
#define RMS_CURRENT       636     // Motor RMS current in MA. My stepper is 42SHDC3025-24B which has peak current 0.9A = 636mA RMS

#define MOTOR_SPEED       960     // Motor speed. Multiply by microsteps for consistency
#define MOTOR_ACCEL       2       // Acceleration steps per microprocessor frequency

WiFiClient wifiClient;   // MQTT Controller
HADevice device;
HAMqtt mqtt(wifiClient, device);
HACover cover("coverboard", HACover::Features::PositionFeature);

WiFiManager wifiManager; // WiFi AP controller
WiFiManagerParameter wfm_mqtt_server("mqtt_s", "MQTT Server", "homeassistant.local", 40);
WiFiManagerParameter wfm_mqtt_username("mqtt_u", "MQTT UserName", "user", 24);
WiFiManagerParameter wfm_mqtt_password("mqtt_p", "MQTT Password", "passwd", 24);

SoftwareSerial DRIVER_SERIAL(SW_RX,SW_TX);
TMC2209Stepper driver(&DRIVER_SERIAL, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver

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

uint8_t endstop_0_state = HIGH;
uint8_t endstop_1_state = HIGH;

bool mqtt_initialized = false;

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

  driver.en_spreadCycle(false);         // Toggle spreadCycle
  driver.pwm_autoscale(true);           // Needed for stealthChop

  driver.VACTUAL(0);                    // Ensure motor is stopped
  digitalWrite(EN_PIN, HIGH);           // Start disabled

}

void configureWiFiAP() {
  // Collect some details for MQTT
  wifiManager.addParameter(&wfm_mqtt_server);
  wifiManager.addParameter(&wfm_mqtt_username);
  wifiManager.addParameter(&wfm_mqtt_password);

  wifiManager.setConfigPortalBlocking(false);

  if (digitalRead(ENDSTOP_0_PIN) == LOW && digitalRead(ENDSTOP_1_PIN) == LOW) { // Open portal if both endstops are pressed at boot
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

void configureMQTT() {
    byte mac[WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);

    device.setUniqueId(mac, sizeof(mac));
    device.setName("Coverboard");
    device.setModel("Coverboard V1");
    device.setManufacturer("CDN");
    device.setSoftwareVersion("0.0.1");

    cover.onCommand(onCoverCommand);
    cover.setName("Coverboard");

    mqtt.begin(wfm_mqtt_server.getValue(), 1883U, wfm_mqtt_username.getValue(), wfm_mqtt_password.getValue());
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
        setTargetSpeed(MOTOR_SPEED);
        if (mqtt_initialized) 
        {
          cover.setState(HACover::CoverState::StateOpening);
          cover.setPosition(50);
        }
        Serial.println("Cover state: Opening"); 
        break;
      case COVER_CLOSING:
        setTargetSpeed(-MOTOR_SPEED);
        if (mqtt_initialized) 
        {
          cover.setState(HACover::CoverState::StateClosing);
          cover.setPosition(0);
        }
        Serial.println("Cover state: Closing"); 
        break;
      case COVER_OPEN:
        setTargetSpeed(0);
        if (mqtt_initialized) 
        {
          cover.setState(HACover::CoverState::StateOpen);
          cover.setPosition(100);
        }
        Serial.println("Cover state: Open"); 
        break;
      case COVER_CLOSED:
        setTargetSpeed(0);
        if (mqtt_initialized) 
        {
          cover.setState(HACover::CoverState::StateClosed);
          cover.setPosition(0);
        }
        Serial.println("Cover state: Closed"); 
        break;
      case COVER_STOPPED:
      default:
        setTargetSpeed(0);
        if (mqtt_initialized) 
        {
          cover.setState(HACover::CoverState::StateStopped);
          cover.setPosition(50);
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

void checkEndstop(uint8_t pin, uint8_t *state, bool is_closed_endstop) {
  if (digitalRead(pin) == *state) return;
  *state = digitalRead(pin);
  if (*state == HIGH) return; // Trigger on press (not release)

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
  pinMode(LED_PIN, OUTPUT);

  pinMode(ENDSTOP_0_PIN, INPUT_PULLUP);
  pinMode(ENDSTOP_1_PIN, INPUT_PULLUP);
  endstop_0_state = digitalRead(ENDSTOP_0_PIN);
  endstop_1_state = digitalRead(ENDSTOP_1_PIN);

  configureWiFiAP();
  configureDriver();
}

void loop() {
  wifiManager.process();
  loopMQTT();

  checkEndstop(ENDSTOP_0_PIN, &endstop_0_state, true);
  checkEndstop(ENDSTOP_1_PIN, &endstop_1_state, false);

  setCoverState();

  digitalWrite(LED_PIN, wifiManager.getConfigPortalActive() ? LOW : HIGH);
}