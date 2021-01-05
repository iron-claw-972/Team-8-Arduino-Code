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
int limSwitchL = -1 //TODO: put correct pins here
int limSwitchR = -1
int limSwitchF1 = -1
int limSwitchF2 = -1
int IR_SENSOR = 0; // Sensor is connected to the analog A0

MPU6050 mpu;

int intSensorResult = 0; //Sensor result
float fltSensorCalc = 0; //Calculated value
unsigned long myTime;
int timeDiff;
int rotation=0;

bool sensed = false;
bool handSensed = false;
int beepCountdown = 0;

CytronMD motorL(PWM_PWM, motorLeftA, motorLeftB);
CytronMD motorR(PWM_PWM, motorRightA, motorRightB);
VL53L0X distSensor;

void checkSettings()
{
  Serial.println();

  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:          ");
  switch (mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Accelerometer:         ");
  switch (mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

  Serial.println();
}


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
  // ms (e.g. sensor.startContinuous(100)).
  distSensor.startContinuous();

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("yay");
    delay(500);
  }
  mpu.calibrateGyro();
  checkSettings();
  myTime = millis();

}

void loop() {


  // Handle the distance sensor
  //Serial.print(sensor.readRangeContinuousMillimeters());

  intSensorResult = analogRead(IR_SENSOR); //Get sensor value
  fltSensorCalc = (6787.0 / (intSensorResult - 3.0)) - 4.0; //Calculate distance in cm

//  Serial.print(fltSensorCalc); //Send distance to computer
//  Serial.println(" cm"); //Add cm to result

  //handle accelerometer
  Vector normGyro = mpu.readNormalizeGyro();
  timeDiff = millis() - myTime;
  myTime = millis();
  //Vector normAccel = mpu.readNormalizeAccel();

//  Serial.print(" Xnorm = ");
//  Serial.print(normGyro.XAxis);
//  Serial.print(" Ynorm = ");
//  Serial.print(normGyro.YAxis);
//  Serial.print(" Znorm = ");
//  Serial.println(normGyro.ZAxis);
  rotation+=normGyro.ZAxis * timeDiff/1000;
  rotation = rotation%360;
  Serial.println(rotation);
  //  Serial.print(" Xnorm = ");
  //  Serial.print(normAccel.XAxis);
  //  Serial.print(" Ynorm = ");
  //  Serial.print(normAccel.YAxis);
  //  Serial.print(" Znorm = ");
  //  Serial.println(normAccel.ZAxis);
  //
  delay(10);


  // Handle the hand
  handSensed = !distSensor.timeoutOccurred() && distSensor.readRangeContinuousMillimeters() < 700 && sensed
  sensed = !distSensor.timeoutOccurred() && distSensor.readRangeContinuousMillimeters() < 700;
  if (handSensed) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    beepCountdown++;
    digitalWrite(buzzer, HIGH);
    //tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(1000);
    digitalWrite(buzzer, LOW);
  }
  else {
    handSensed = false;
    beepCountdown = 0;
    //noTone(buzzer);
  }

  // Sanitize
  if (beepCountdown > 2) {
    digitalWrite(mosfetPump, HIGH);
    delay(150);
    digitalWrite(mosfetPump, LOW);
    delay(1000);
  }

  //Drive
  Serial.println();
  if (!handSensed && beepCountdown==0){
    //not sanitizing, chooses which direction to drive
    //whichever limit switch has the greatest output will decide the direction

    if (digitalRead(limSwitchL)==LOW && digitalRead(limSwitchR)==LOW && digitalRead(limSwitchF1)==LOW && digitalRead(limSwitchF2)==LOW && fltSensorCalc > 25){
      //noting detected, keep moving foward
      motorL.setSpeed(105);
      motorR.setSpeed(-100);

    } else if(digitalRead(limSwitchL)==HIGH && digitalRead(limSwitchR)==LOW){
      motorL.setSpeed(105);
      motorR.setSpeed(100);
      delay(750); //TODO: Find correct time to do 135 degree turn
      motorL.setSpeed(105);
      motorR.setSpeed(-100);

    } else if (digitalRead(limSwitchR)==HIGH && digitalRead(limSwitchL)==LOW) {
      motorL.setSpeed(-105);
      motorR.setSpeed(-100);
      delay(750); //TODO: Find correct time to do 135 degree turn
      motorL.setSpeed(105);
      motorR.setSpeed(-100);

    } else {
      //otherwise if not left or right or nothing it will do 180
      motorL.setSpeed(105);
      motorR.setSpeed(100);
      delay(1000); //TODO: Find correct time to do 180 degree turn
      motorL.setSpeed(105);
      motorR.setSpeed(-100);
  }
}
