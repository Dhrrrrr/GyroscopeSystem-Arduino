#include <Wire.h>
#include "mpu9250.h"
#include <math.h>
#include <stdlib.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <stdio.h>

bfs::Mpu9250 IMU(&Wire, 0x68);

SoftwareSerial Slave(7,8); // RX TX

const float estimatedAccVar = 0.2;
const float estimatedGyrVar = 0.5;
const float doubleUpAngle = 0.8;
const float emergencyModAngle = 1.2;

const int timePerTick = 0;
const int tickPerPrint = 0;

float pitch = 0;
float roll = 0;
String fullInfo = "";

void setup() {
   
  Wire.begin();
  IMU.Begin();

  pinMode(9,OUTPUT);
  
  Serial.begin(38400);
  Slave.begin(38400);

  pinMode(10,OUTPUT);
  digitalWrite(10, HIGH);

  while(!Serial) {}

  IMU.Begin();

  Serial.print("Starting");
  Serial.println();

  IMU.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G);
  IMU.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_500DPS);

  while(!IMU.Read()) {}

  pitch = FindPitchAcc(IMU.accel_x_mps2());
  roll = FindRollAcc(IMU.accel_y_mps2(), IMU.accel_z_mps2());
}

int previousTime = 0;

int tick = 0;
int commaCount = 0;
float expectedPitch = 0;
float expectedRoll = 0;
float pressedDown = 0;


void loop() {
  int currentTime = millis();

  // Get information from master into slave
  long incomingData = Slave.read();

  bool pitchFor, pitchBack, rollLef, rollRig, down;

  pitchFor = Find1Digit(incomingData, 1);
  pitchBack = Find1Digit(incomingData, 2);
  rollLef = Find2Digit(incomingData, 1);
  rollRig = Find2Digit(incomingData, 3);

  down = Find3Digit(incomingData, 1);

  if (down == true) {
    digitalWrite(9, HIGH);

  }
  else {
    digitalWrite(9, LOW);

  }

  Serial.println(incomingData);

  if(rollLef == true){
    expectedRoll = -0.2;
  }
  else if (rollRig == true) {
    expectedRoll = 0.2;

  }
  else {
    expectedRoll = 0;

  }

  if(pitchFor == true){
    expectedPitch = -0.2;
  }

  
  else if (pitchBack == true) {
    expectedPitch = 0.2;

  }
  else {
    expectedPitch = 0;

  }



  

  if (IMU.Read()) {

    int timeElapsed = currentTime - previousTime;
    previousTime = currentTime;

    float accX = IMU.accel_x_mps2();
    float accY = IMU.accel_y_mps2();
    float accZ = IMU.accel_z_mps2();

    float gyrX = IMU.gyro_x_radps();
    float gyrY = IMU.gyro_y_radps();
    float gyrZ = IMU.gyro_z_radps();
    

    float gyrPitch = FindPitchGyr(gyrY, timeElapsed / 1000, pitch);
    float gyrRoll = FindRollGyr(gyrX, timeElapsed / 1000, roll);

    float accPitch = FindPitchAcc(accX);
    float accRoll = FindRollAcc(accY, accZ);

    pitch = filterItem(accPitch, gyrPitch, estimatedAccVar, estimatedGyrVar);
    roll = filterItem(accRoll, gyrRoll, estimatedAccVar, estimatedGyrVar);

    int tickNum = tick % tickPerPrint;
    if (tickNum == 0 )
    {

    }

    if (pitch != NULL && roll != NULL) {
      RunDroneControl(pitch, roll, expectedPitch, expectedRoll);
    }
  }
  else {
     Serial.print("read errors");
    Serial.println();   
  }



  
  tick += 1;
  //delay(timePerTick);
}

void RunDroneControl(float pitch, float roll, float attemptedPitchAngle, float attemptedRollAngle) {
  bool LeftFrontMotor = false;
  bool RightFrontMotor = false;
  bool LeftBackMotor = false;
  bool RightBackMotor = false;

  bool emergencyMode = false;

  float dualAngleP = doubleUpAngle;
  float negativeDualAngleP = -doubleUpAngle;

  float dualAngleR = doubleUpAngle;
  float negativeDualAngleR = -doubleUpAngle;

  float modulo = sqrt((pitch * pitch) + (roll * roll));

  if (modulo > emergencyModAngle) {
    emergencyMode = true;

  }


  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);

  if((roll > dualAngleR || roll < negativeDualAngleR) && (pitch > dualAngleP && pitch < negativeDualAngleP)) {
   // Both are over

   if (pitch > 0){
    RollCalc(true, roll, attemptedRollAngle, -10, 10);

   }
   else {
    RollCalc(false, roll, attemptedRollAngle, -10, 10);

   }
  }
  else if (pitch < attemptedPitchAngle && pitch > negativeDualAngleP) {
    // pitch is far back, below dual
    RollCalc(false, roll, attemptedRollAngle, negativeDualAngleR, dualAngleR);

  }
  else if (pitch > attemptedPitchAngle && pitch < dualAngleP) {
    // pitch is far forward, below dual
    RollCalc(true, roll, attemptedRollAngle, negativeDualAngleR, dualAngleR);
    
  }
  else if (pitch >= dualAngleP) {
    // pitch is far forward, above dual
    ActivateLight(true, true, false, false);

  }
  else {
    // pitch is far back, above dual

    ActivateLight(false, false, true, true);
  }


}

float filterItem(float A, float B, float varA, float varB) {
  float num;

  num = ((A * varA) + (B * varB)) / (varA + varB);

  return num;
}


float FindPitchAcc(float accX) {
  float pitchAcc = asin(accX / 9.8);

  if (isnan(pitchAcc)) {
    return 0;
  }

  return pitchAcc;
}

float FindRollAcc(float accY, float accZ) {
  float rollAcc = atan(accY / accZ);

  return rollAcc;
}

float FindPitchGyr(float gyrX, float timeElapsed, float prevPitch) {
  float pitchF = prevPitch + (0.5 * gyrX * (timeElapsed * timeElapsed));

  return pitchF;
}

float FindRollGyr(float gyrY, float timeElapsed, float prevRoll) {
  float rollF = prevRoll + (0.5 * gyrY * (timeElapsed * timeElapsed));
  
  return rollF;
}

float CalcTime(float input, float timeElapsed) {
  float num = input * (timeElapsed / 1000);
  return num;
}

void ActivateLight(bool leftFront, bool rightFront, bool leftBack, bool rightBack) {
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);

  digitalWrite(2, leftBack);
  digitalWrite(3, rightFront);
  digitalWrite(4, leftFront);
  digitalWrite(5, rightBack);
}

void RollCalc(bool isForward, float roll, float attemptPoint, float negDual, float dual) {

  bool activeList [4] = {false, false, false, false};

  if(isForward == true) {
    activeList[0] = true;
    activeList[1] = true;

  }
  else {
    activeList[2] = true;
    activeList[3] = true;

  }

  if (roll > dual) {
    activeList[0] = true;
    activeList[1] = false;
    activeList[2] = true;
    activeList[3] = false;
  }
  else if (roll < negDual) {
    activeList[0] = false;
    activeList[1] = true;
    activeList[2] = false;
    activeList[3] = true;
  }
  else if (roll > attemptPoint) {
    if (activeList[1]) {
      activeList[1] = false;
    }
    else {
      activeList[3] = false;
    }
  }
  else {
        if (activeList[0]) {
      activeList[0] = false;
    }
    else {
      activeList[2] = false;
    }
  }

  ActivateLight(activeList[0], activeList[1], activeList[2], activeList[3]);
}

bool Find1Digit(int num, int Digit) 
{
  
  if (num < ((Digit * 100) + 100) && num >= (Digit * 100)) {
    return true;
  }
  else {
    return false;
  }
}

bool Find2Digit(int pureNum ,int Digit) 
{
  int pureNumHun = floor(pureNum / 100);
  int num = (pureNum - (pureNumHun * 100));
  
  if (num < ((Digit * 10) + 10) && num >= (Digit * 10)) {
    return true;
  }
  else {
    return false;
  }
}

bool Find3Digit(int pureNum ,int Digit) 
{
  int pureNumTen = floor(pureNum / 10);
  int num = pureNum - (pureNumTen * 10);
  
  if (num == Digit) {
    return true;
  }
  else {
    return false;
  }
}

