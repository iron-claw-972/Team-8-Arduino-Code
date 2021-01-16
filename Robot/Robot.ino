#include <MPU6050.h>
#include <math.h>

#include <VL53L0X.h>

#include "CytronMotorDriver.h"

#include <Wire.h>
int motorLeftA = 3;
int motorLeftB = 9;
int motorRightA = 10;
int motorRightB = 11;
int mosfetPump = 5;
int buzzer = 4;
int limSwitchL = -1; //TODO: put correct pins here
int limSwitchR = -1;
int limSwitchF1 = -1;
int limSwitchF2 = -1;
int PIR = 2;
int IR_SENSOR = 0; // Sensor is connected to the analog A0

MPU6050 mpu;

int intSensorResult = 0; //Sensor result
float fltSensorCalc = 0; //Calculated value
unsigned long myTime = millis();
double turnTarget = 0;

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

  pinMode(PIR, INPUT);

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
  delay(500);

}


double getRotation(double currentAngle, int timeDiff){
    Vector normGyro = mpu.readNormalizeGyro();
    currentAngle+=normGyro.ZAxis * timeDiff/1000.0;
    //w3currentAngle=fmod(currentAngle,360.0);
    return currentAngle;
}


void turn(){
  turnTarget = turnTarget - getRotation(turnTarget, millis() - myTime);
  myTime = millis();
  if (turnTarget > 4 or turnTarget < -4){
    double rotationSpeed = sqrt(sqrt(abs(turnTarget/180))) * 200 * turnTarget/abs(turnTarget);

    motorL.setSpeed(rotationSpeed);
    motorR.setSpeed(-(-rotationSpeed));
  } else {
    motorL.setSpeed(105);
    motorR.setSpeed(-100);
  }
}

void loop() {

  // Handle the distance sensor
  //Serial.print(sensor.readRangeContinuousMillimeters());

  intSensorResult = analogRead(IR_SENSOR); //Get sensor value
  fltSensorCalc = (6787.0 / (intSensorResult - 3.0)) - 4.0; //Calculate distance in cm

//  Serial.print(fltSensorCalc); //Send distance to computer
//  Serial.println(" cm"); //Add cm to result

  if (beepCountdown > 3) {
    digitalWrite(mosfetPump, LOW);
  }

  // Handle the hand
  handSensed = sensed && !distSensor.timeoutOccurred() && distSensor.readRangeContinuousMillimeters() < 700;
  sensed = !distSensor.timeoutOccurred() && distSensor.readRangeContinuousMillimeters() < 700;
  if (handSensed && digitalRead(PIR)==HIGH) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    beepCountdown++;
    if (beepCountdown % 2 == 0) {
      digitalWrite(buzzer, LOW)
    } else {
      digitalWrite(buzzer, HIGH);
      tone(buzzer, 1000); // Send 1KHz sound signal...
    }

  } else if (handSensed && digitalRead(PIR)==LOW) {
    turnTarget = 1600; //is under something, (no heat so not a hand), will do 180

    handSensed = false;
    beepCountdown = 0;
  } else {
    handSensed = false;
    beepCountdown = 0;
    noTone(buzzer);
  }

  // Sanitize
  if (beepCountdown > 3) {
    digitalWrite(mosfetPump, HIGH);
  }

  //Drive
  Serial.println();

  if (!handSensed && beepCountdown==0){
    //not sanitizing, chooses which direction to drive
    if (turnTarget > 4 or turnTarget < -4) {
      if (digitalRead(limSwitchL)==LOW && digitalRead(limSwitchR)==LOW && digitalRead(limSwitchF1)==LOW && digitalRead(limSwitchF2)==LOW && fltSensorCalc > 25){
        turnTarget = 0;
      } else if(digitalRead(limSwitchL)==HIGH && digitalRead(limSwitchR)==LOW){
        turnTarget = 900;
      } else if (digitalRead(limSwitchR)==HIGH && digitalRead(limSwitchL)==LOW) {
        turnTarget = -900;
      } else {
        turnTarget = 1600;
      }
    }

  turn();
}
