#include <MPU6050.h>

#include <VL53L0X.h>

#include "CytronMotorDriver.h"

#include <Wire.h>
int motorLeftA = 3;
int motorLeftB = 9;
int motorRightA = 10;
int motorRightB = 11;
int mosfetPump = 5;
int buzzer = 4;

bool sensed = false;
bool handSensed = false;
int beepCountdown = 0;

CytronMD motorL(PWM_PWM, motorLeftA, motorLeftB);
CytronMD motorR(PWM_PWM, motorRightA, motorRightB);
VL53L0X sensor;


void setup() {
  Serial.begin(9600);
  Serial.println("hi");

  pinMode(buzzer, OUTPUT);
  pinMode(mosfetPump, OUTPUT); 
  
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");

  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
  Serial.println("hi2");

}

void loop() {
  // Handle the hand
  Serial.print(sensor.readRangeContinuousMillimeters());
  
  handSensed = !sensor.timeoutOccurred() && sensor.readRangeContinuousMillimeters()<700 && sensed;
  sensed = !sensor.timeoutOccurred() && sensor.readRangeContinuousMillimeters()<700;
  if (handSensed){
    motorL.setSpeed(0);   
    motorR.setSpeed(0); 
    beepCountdown++;
    digitalWrite(buzzer,HIGH);
    //tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(1000);
  }
  else{
    handSensed = false;
    beepCountdown = 0;
    //noTone(buzzer);
    digitalWrite(buzzer,LOW);
  }


  // Sanitize
  if (beepCountdown>2){
    digitalWrite(mosfetPump, HIGH);
    delay(2000);
    digitalWrite(mosfetPump, LOW);
  }

  //Drive
  Serial.println();
  if (!handSensed && beepCountdown==0){
      motorL.setSpeed(100);   
      motorR.setSpeed(-100);  
  }

}
