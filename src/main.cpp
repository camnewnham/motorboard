#include <Arduino.h>
#include <TMCStepper.h>

#define EN_PIN            5       // Driver Enable pin
#define STEP_PIN          0

#define STEPDELAY         2

void setup() {
  Serial.begin(115200); 
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
}

void loop() {
  digitalWrite(STEP_PIN, LOW);
  delay(STEPDELAY);
  digitalWrite(STEP_PIN, HIGH);
  delay(STEPDELAY);
}