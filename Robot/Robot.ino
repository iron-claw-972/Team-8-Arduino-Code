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
VL53L0X distSensor;

//TODO: make and init all the limit switches (left, right, front, back)
int limSwitchL = -1;
int limSwitchR = -1;
int limSwitchF = -1;


void setup() {
  Serial.begin(9600);

  pinMode(buzzer, OUTPUT);
  pinMode(mosfetPump, OUTPUT);

  pinMode(limSwitchL, INPUT);
  pinMode(limSwitchR, INPUT);
  pinMode(limSwitchF, INPUT);

  Wire.begin();

  distSensor.setTimeout(500);
  if (!distSensor.init()){
    Serial.println("Failed to detect and initialize distSensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. distSensor.startContinuous(100)).
  distSensor.startContinuous();

}

void loop() {
  // Handle the hand
  Serial.print(distSensor.readRangeContinuousMillimeters());

  handSensed = !distSensor.timeoutOccurred() && distSensor.readRangeContinuousMillimeters()<1400;
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
  if (!handSensed && beepCountdown==0){
    //not sanitizing, chooses which direction to drive
    //which ever limit switch has the greatest output will decide the direction
    if (analogRead(limSwitchL) == 0 && analogRead(limSwitchR) == 0 && analogRead(limSwitchF) == 0){
      //noting detected, keep moving foward
      motorL.setSpeed(200);
      motorR.setSpeed(200);

    } else if(analogRead(limSwitchL) > analogRead(limSwitchF) && analogRead(limSwitchL) > analogRead(limSwitchR)){
      motorL.setSpeed(200);
      motorR.setSpeed(-200);
      delay(750); //TODO: Find correct time to do 135 degree turn
      motorL.setSpeed(200);
      motorR.setSpeed(200);

    } else if (analogRead(limSwitchR) > analogRead(limSwitchF) && analogRead(limSwitchR) > analogRead(limSwitchL)) {
      motorL.setSpeed(-200);
      motorR.setSpeed(200);
      delay(750); //TODO: Find correct time to do 135 degree turn
      motorL.setSpeed(200);
      motorR.setSpeed(200);

    } else {
      //otherwise if left or right aren't larger then it will do a 180
      motorL.setSpeed(200);
      motorR.setSpeed(-200);
      delay(1000); //TODO: Find correct time to do 180 degree turn
      motorL.setSpeed(200);
      motorR.setSpeed(200);
  }
}
