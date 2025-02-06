/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.1  6 February 2025 - Added roll, pitch, and yaw angles with drift prevention
*/

#include <Wire.h>
#include <math.h>

const int MPU = 0x68;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AngleRoll = 0, AnglePitch = 0, AngleYaw = 0;
float AccRoll, AccPitch;
int RateCalibrationNumber;
unsigned long lastTime;

// LED Pins
const int ledRollPos = 1;
const int ledRollNeg = 2;
const int ledPitchPos = 3;
const int ledPitchNeg = 4;

// Thresholds
const float angleThreshold = 20.0;

void gyro_signals(void) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);
  int16_t AccX = Wire.read() << 8 | Wire.read();
  int16_t AccY = Wire.read() << 8 | Wire.read();
  int16_t AccZ = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  AccRoll = atan2(AccY, AccZ) * 180 / M_PI;
  AccPitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / M_PI;
}

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.println("Calibrating Gyro...");
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  Serial.println("Calibration Complete.");
  
  lastTime = millis();

  // Setup LED pins as output
    pinMode(ledRollPos, OUTPUT);
    pinMode(ledRollNeg, OUTPUT);
    pinMode(ledPitchPos, OUTPUT);
    pinMode(ledPitchNeg, OUTPUT);
}

void loop() {
  gyro_signals();
  
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  AngleRoll = 0.98 * (AngleRoll + RateRoll * dt) + 0.02 * AccRoll;
  AnglePitch = 0.98 * (AnglePitch + RatePitch * dt) + 0.02 * AccPitch;
  AngleYaw += RateYaw * dt; 

  // LED Control
  digitalWrite(ledRollPos, AngleRoll > angleThreshold ? HIGH : LOW);
  digitalWrite(ledRollNeg, AngleRoll < -angleThreshold ? HIGH : LOW);
  digitalWrite(ledPitchPos, AnglePitch > angleThreshold ? HIGH : LOW);
  digitalWrite(ledPitchNeg, AnglePitch < -angleThreshold ? HIGH : LOW);
  
  Serial.print("Roll [°] = ");
  Serial.print(AngleRoll);
  Serial.print(" | Pitch [°] = ");
  Serial.println(AnglePitch);
  
  delay(50);
}
