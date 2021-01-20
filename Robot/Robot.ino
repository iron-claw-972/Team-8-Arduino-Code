#include <SimpleTimer.h>

#include <Encoder.h>

#include <MPU6050.h>
#include <math.h>

#include <VL53L0X.h>

#include "CytronMotorDriver.h"

#include <Wire.h>
const int motorLeftA = 3;
const int motorLeftB = 9;
const int motorRightA = 10;
const int motorRightB = 11;
const int encoderLeftA = 8;
const int encoderLeftB = 12;
const int encoderRightA = 7;
const int encoderRightB = 6;


int mosfetPump = 5;
int buzzer = 4;
int PIR = 2;
int IR_SENSOR = 0; // Sensor is connected to the analog A0

MPU6050 mpu;


float distanceUp = 0; //Calculated value
unsigned long prevTime;
int timeDiff = 1;
double distanceForward;
bool PIRup = false;

//state variables
bool sensed = false;

bool handSensed = false;
bool underSomething = false;
bool objectClose = false;
bool objectTooClose = false;
bool crashed = false;
bool backingUp = false;
bool turning = false;

bool alarmActivated = true;
bool alarmLoading = false;
int buzzerCount = 0;

int motorVelLeft = 100;
int motorVelRight = 100;
bool motorsStopped = false;

double encoderLeft = 0;
double encoderRight = 0;

double encoderLeftStarting = 0;
double encoderRightStarting = 0;


static uint8_t prevNextCode = 0;
static uint16_t store=0;

static uint8_t prevNextCode1 = 0;
static uint16_t store1=0;


SimpleTimer timer;
CytronMD motorL(PWM_PWM, motorLeftA, motorLeftB);
CytronMD motorR(PWM_PWM, motorRightA, motorRightB);
VL53L0X sensor;



void setup() {
  Serial.begin(9600);
  Serial.println("hi");

  pinMode(buzzer, OUTPUT);
  pinMode(mosfetPump, OUTPUT);
  pinMode(PIR, INPUT);

  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);

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

  timer.setInterval(100, updateState);

}

void updateState(){
  // Handle the distance sensor
  distanceForward = (6787.0 / (analogRead(IR_SENSOR) - 3.0)) - 4.0; //Calculate distance in cm
  objectClose = distanceForward<26 and distanceForward>10;
  objectTooClose = distanceForward<10;


  // Handle the up sensors
  // aka Handling the Hand
  distanceUp=sensor.readRangeContinuousMillimeters();
  PIRup=digitalRead(PIR);
  handSensed = !sensor.timeoutOccurred() && distanceUp < 700 && sensed && PIRup;
  underSomething = !sensor.timeoutOccurred() && distanceUp < 700 && sensed && !PIRup;
  sensed = !sensor.timeoutOccurred() && distanceUp < 700;

  // Handle the spray / buzz
  if (handSensed){
    alarm();
    motorsStopped = true;
    if (buzzerCount>3){
      pump();
      buzzerCount=0;
      motorsStopped = false;
    }
  }
  else{
    buzzerCount = 0;
    motorsStopped = false;
  }

 //Drive
 if (objectTooClose or crashed or underSomething){
  if (not backingUp){
    encoderLeftStarting = encoderLeft;
    encoderRightStarting = encoderRight;
  }
  backingUp = true;
  turning = false;
 }
 else if (objectClose){
  if (not turning and not backingUp){
    encoderLeftStarting = encoderLeft;
    encoderRightStarting = encoderRight;
  }
  turning = true;
 }

 if (motorsStopped){
  stopMotors();
 }
 else if (turning){
   turning = turn(0.5, encoderLeftStarting, encoderLeft);
 }
 else if (objectTooClose){
   backingUp = backUp(-0.3, encoderLeftStarting, encoderLeft);
   turning=false;
 }
 else {
   forward(sqrt(distanceUp)*20+10);
 }
}

void loop() {
  // Handle encoder
  encoderLeft += -read_rotary()/63.0;
  encoderRight += -read_rotary1()/78.0;
  //Serial.println(encoderRight);
  timer.run();
}

void stopMotors(){
  motorL.setSpeed(0);
  motorR.setSpeed(0);
}

void forward(double vel){
  motorL.setSpeed(vel);
  motorR.setSpeed(-vel);
}

bool turn(double rotations, double start, double current){
  if ((start+math.abs(rotations))>current){
    motorL.setSpeed(150 * rotations/math.abs(rotations));
    motorR.setSpeed(0);
    return true;
  }
  else{
    return false;
  }
}
bool backUp(double rotations, double start, double current){
  if ((start+rotations)<current){
    motorL.setSpeed(-150);
    motorR.setSpeed(150);
    return true;
  }
  else{
    return false;
  }

}
void pump(){
  digitalWrite(mosfetPump, HIGH);
  timer.setTimeout(150,[](){digitalWrite(mosfetPump, LOW);});
}
void alarm(){
  if (!alarmLoading){
    if (alarmActivated){
      digitalWrite(buzzer, HIGH);
      buzzerCount++;
      timer.setTimeout(1000,[](){digitalWrite(buzzer, LOW);alarmActivated=false;alarmLoading=false;});
      alarmLoading=true;
    }
    else{
      timer.setTimeout(200,[](){alarmActivated=true;alarmLoading=false;});
      alarmLoading=true;
    }
  }
}

double getRotation(double currentAngle, int timeDiff){
    Vector normGyro = mpu.readNormalizeGyro();
    currentAngle+=normGyro.ZAxis * timeDiff/1000.0;
    currentAngle=fmod(currentAngle,360.0);
    return currentAngle;
}



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

// A vald CW or  CCW move returns 1, invalid returns 0.
int8_t read_rotary() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(encoderLeftA)) prevNextCode |= 0x02;
  if (digitalRead(encoderLeftB)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode] ) {
      store <<= 4;
      store |= prevNextCode;
      //if (store==0xd42b) return 1;
      //if (store==0xe817) return -1;
      if ((store&0xff)==0x2b) return -1;
      if ((store&0xff)==0x17) return 1;
   }
   return 0;
}

int8_t read_rotary1() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode1 <<= 2;
  if (digitalRead(encoderRightA)) prevNextCode1 |= 0x02;
  if (digitalRead(encoderRightB)) prevNextCode1 |= 0x01;
  prevNextCode1 &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode1] ) {
      store1 <<= 4;
      store1 |= prevNextCode1;
      //if (store1==0xd42b) return 1;
      //if (store1==0xe817) return -1;
      if ((store1&0xff)==0x2b) return -1;
      if ((store1&0xff)==0x17) return 1;
   }
   return 0;
}
