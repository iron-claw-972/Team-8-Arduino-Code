#include <MPU6050.h>
#include <math.h>

#include <VL53L0X.h>

#include "CytronMotorDriver.h"

#include <Wire.h>
int motorLeftA = 3;
int motorLeftB = 9;
int motorRightA = 10;
int motorRightB = 11;
int encoderLeftA = 8;
int encoderLeftB = 12;
int encoderRightA = 7;
int encoderRightB = 6;


int mosfetPump = 5;
int buzzer = 4;
int PIR = 2;
int IR_SENSOR = 0; // Sensor is connected to the analog A0

MPU6050 mpu;

int intSensorResult = 0; //Sensor result
float fltSensorCalc = 0; //Calculated value
unsigned long myTime;


bool sensed = false;
bool handSensed = false;
bool PIRSensed = false;
int beepCountdown = 0;

CytronMD motorL(PWM_PWM, motorLeftA, motorLeftB);
CytronMD motorR(PWM_PWM, motorRightA, motorRightB);
VL53L0X sensor;

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
  pinMode(PIR, INPUT);

  pinMode(encoderLeftA, INPUT);
  pinMode(encoderLeftB, INPUT);
  pinMode(encoderRightA, INPUT);
  pinMode(encoderRightB, INPUT);

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
  sensor.startContinuous(4);

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
    currentAngle=fmod(currentAngle,360.0);
    return currentAngle;
}


void turn(double targetAngle){
  int timeDiff = 0;
  double currentAngle = 0;
  double angleDiff;
  double rotationSpeed;
  unsigned long myTime = millis();
  while (((currentAngle-targetAngle)>4) or ((currentAngle-targetAngle)<-4)){
    angleDiff=targetAngle-currentAngle;
    rotationSpeed = sqrt(sqrt(abs(angleDiff/180))) * 200 * angleDiff/abs(angleDiff);
    
    motorL.setSpeed(-rotationSpeed);
    motorR.setSpeed(-(rotationSpeed));
    delay(30);
    
    timeDiff = millis() - myTime;
    myTime = millis();
    currentAngle = getRotation(currentAngle,timeDiff);
    Serial.println(angleDiff);

  }
  motorL.setSpeed(0);
  motorR.setSpeed(0);
  
}

void loop() {
  
  
  // Handle the distance sensor
  //Serial.print(sensor.readRangeContinuousMillimeters());
  
  intSensorResult = analogRead(IR_SENSOR); //Get sensor value
  fltSensorCalc = (6787.0 / (intSensorResult - 3.0)) - 4.0; //Calculate distance in cm
  
//  Serial.print(fltSensorCalc); //Send distance to computer
//  Serial.println(" cm"); //Add cm to result

  
  if (fltSensorCalc<20) {
    turn(90);
  }

  // Handle the hand
  PIRSensed = digitalRead(PIR);
  handSensed = !sensor.timeoutOccurred() && sensor.readRangeContinuousMillimeters() < 700 && sensed && PIRSensed;
  sensed = !sensor.timeoutOccurred() && sensor.readRangeContinuousMillimeters() < 700;
  if (handSensed) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    beepCountdown++;
    //digitalWrite(buzzer, HIGH);
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
    //digitalWrite(mosfetPump, HIGH);
    delay(150);
    digitalWrite(mosfetPump, LOW);
    delay(1000);
  }


  //Drive
  
  if (!handSensed && beepCountdown == 0) {
    motorL.setSpeed(sqrt(fltSensorCalc)*20+10);
    motorR.setSpeed(-sqrt(fltSensorCalc)*20+10);
  }
  delay(30);

}
