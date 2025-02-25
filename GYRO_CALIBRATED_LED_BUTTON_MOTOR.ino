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

// Button and Indicator LEDs
const int buttonPin = 7;  
const int redLED = 8;
const int greenLED = 9;
bool systemActive = true;

// Motor Control (DRV8871)
const int motorIN1 = 5;
const int motorIN2 = 6;

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

  // Setup LED and motor pins
  pinMode(ledRollPos, OUTPUT);
  pinMode(ledRollNeg, OUTPUT);
  pinMode(ledPitchPos, OUTPUT);
  pinMode(ledPitchNeg, OUTPUT);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
}

void loop() {
  // Button Debounce Logic (Press to Start/Stop System)
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(buttonPin);
  
  if (buttonState == LOW && lastButtonState == HIGH) {  
    systemActive = !systemActive;
    delay(200);  // Debounce delay
  }
  lastButtonState = buttonState;
  
  // System State LED Indicator
  digitalWrite(redLED, systemActive ? LOW : HIGH);
  digitalWrite(greenLED, systemActive ? HIGH : LOW);

  if (!systemActive) {
    stopMotor();
    return;  // Exit loop if system is off
  }

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

  // Motor Control Logic
  if (AngleRoll > angleThreshold || AnglePitch > angleThreshold) {
    spinMotor();
  } else if (AngleRoll < -angleThreshold || AnglePitch < -angleThreshold) {
    stopMotor();
  }

  Serial.print("Roll [°] = ");
  Serial.print(AngleRoll);
  Serial.print(" | Pitch [°] = ");
  Serial.println(AnglePitch);
  
  delay(50);
}

void spinMotor() {
  analogWrite(motorIN1, 255); // Full Speed Forward
  analogWrite(motorIN2, 0);
}

void stopMotor() {
  analogWrite(motorIN1, 0);
  analogWrite(motorIN2, 0);
}
