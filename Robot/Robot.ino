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
//TODO: make all the limit switches (left, right, front, back)
int limSwitchL = -1;
int limSwitchR = -1;
int limSwitchF1 = -1;
int limSwitchF2 = -1;

bool sensed = false;
bool handSensed = false;
int beepCountdown = 0;

CytronMD motorL(PWM_PWM, motorLeftA, motorLeftB);
CytronMD motorR(PWM_PWM, motorRightA, motorRightB);
VL53L0X distSensor;


void setup() {
  Serial.begin(9600);
  Serial.println("hi");

  pinMode(buzzer, OUTPUT);
  pinMode(mosfetPump, OUTPUT);

  pinMode(limSwitchL, INPUT);
  pinMode(limSwitchR, INPUT);
  pinMode(limSwitchF1, INPUT);
  pinMode(limSwitchF2, INPUT);

  Wire.begin();

  distSensor.setTimeout(500);
  if (!distSensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");

  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. distSensor.startContinuous(100)).
  distSensor.startContinuous();
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
    //not sanitizing, chooses which direction to drive
    //whichever limit switch has the greatest output will decide the direction

    if (digitalRead(limSwitchL)==LOW && digitalRead(limSwitchR)==LOW && digitalRead(limSwitchF1)==LOW && digitalRead(limSwitchF2)==LOW){
      //noting detected, keep moving foward
      motorL.setSpeed(200);
      motorR.setSpeed(-200);

    } else if(digitalRead(limSwitchL)==HIGH && digitalRead(limSwitchR)==LOW){
      motorL.setSpeed(200);
      motorR.setSpeed(200);
      delay(750); //TODO: Find correct time to do 135 degree turn
      motorL.setSpeed(200);
      motorR.setSpeed(-200);

    } else if (digitalRead(limSwitchR)==HIGH && digitalRead(limSwitchL)==LOW) {
      motorL.setSpeed(-200);
      motorR.setSpeed(-200);
      delay(750); //TODO: Find correct time to do 135 degree turn
      motorL.setSpeed(200);
      motorR.setSpeed(-200);

    } else {
      //otherwise if not left or right or nothing it will do 180
      motorL.setSpeed(200);
      motorR.setSpeed(200);
      delay(1000); //TODO: Find correct time to do 180 degree turn
      motorL.setSpeed(200);
      motorR.setSpeed(-200);
  }
}
