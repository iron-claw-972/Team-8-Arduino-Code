#include <MPU6050.h>

#include <VL53L0X.h>

#include "CytronMotorDriver.h"

#include <Wire.h>
int motorLeftA = 3;
int motorLeftB = 9;
int motorRightA = 10;
int motorRightB = 11;
int mosfetPump = 5;
int buzzer = 6;

bool handSensed = false;
int beepCountdown = 0;

CytronMD motorL(PWM_PWM, motorLeftA, motorLeftB);
CytronMD motorR(PWM_PWM, motorRightA, motorRightB);
VL53L0X sensor;


void setup() {
  Serial.begin(9600);

  pinMode(buzzer, OUTPUT);
  pinMode(mosfetPump, OUTPUT); 
  
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();

}

void loop() {
  // Handle the hand
  Serial.print(sensor.readRangeContinuousMillimeters());

  handSensed = !sensor.timeoutOccurred() && sensor.readRangeContinuousMillimeters()<1400;
  if (handSensed){
    motorL.setSpeed(0);   
    motorR.setSpeed(0); 
    beepCountdown++;
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(600);
  }
  else{
    handSensed = false;
    beepCountdown = 0;
    noTone(buzzer);
  }


  // Sanitize
  if (beepCountdown>2){
    analogWrite(mosfetPump, 230);
    delay(1000);
    analogWrite(mosfetPump, 0);
  }

  //Drive
  Serial.println();
  if (!handSensed && pumpTriggered==0){
      motorL.setSpeed(100);   
      motorR.setSpeed(255);  
  }

}
