#include <Wire.h>
#include <math.h>

// Pin Definitions
const int buttonPin = 5;
const int redLED = 6;
const int greenLED = 7;
const int ledRollPos = 1;
const int ledRollNeg = 2;
const int ledPitchPos = 3;
const int ledPitchNeg = 4;
const int MOTOR_IN1 = 9;
const int MOTOR_IN2 = 10;

// MPU6050 Setup
const int MPU = 0x68;
float RateRoll, RatePitch;
float AngleRoll = 0, AnglePitch = 0;
float AccRoll, AccPitch;
unsigned long lastTime;

// Button Debounce
bool systemEnabled = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Motor Control
const int PWM_HALF_SPEED = 127;  // 50% speed
const int PWM_FULL_SPEED = 255;  // 100% speed

// Tilt Thresholds
const float angleThreshold = 20.0;  // Angle threshold in degrees
const float rateThreshold = 3.0;   // Angular velocity threshold in deg/s

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.setClock(400000);

    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(ledRollPos, OUTPUT);
    pinMode(ledRollNeg, OUTPUT);
    pinMode(ledPitchPos, OUTPUT);
    pinMode(ledPitchNeg, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

    // Start with system OFF
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
    digitalWrite(ledRollPos, LOW);
    digitalWrite(ledRollNeg, LOW);
    digitalWrite(ledPitchPos, LOW);
    digitalWrite(ledPitchNeg, LOW);
    stopMotor();
    lastTime = millis();
}

void loop() {
    static bool lastButtonState = HIGH;
    bool currentButtonState = digitalRead(buttonPin);

    // Button press with debounce
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        if (millis() - lastDebounceTime > debounceDelay) {
            systemEnabled = !systemEnabled;
            lastDebounceTime = millis();
        }
    }
    lastButtonState = currentButtonState;

    // If system is OFF, stop motor and exit loop early
    if (!systemEnabled) {
        stopMotor();
        digitalWrite(redLED, HIGH);
        digitalWrite(greenLED, LOW);
        digitalWrite(ledRollPos, LOW);
        digitalWrite(ledRollNeg, LOW);
        digitalWrite(ledPitchPos, LOW);
        digitalWrite(ledPitchNeg, LOW);
        return;
    }

    // System is ON: Read gyro, control motor and LEDs
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, HIGH);
    gyro_signals();

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    AngleRoll = 0.98 * (AngleRoll + RateRoll * dt) + 0.02 * AccRoll;
    AnglePitch = 0.98 * (AnglePitch + RatePitch * dt) + 0.02 * AccPitch;

    Serial.print("Roll [°]: ");
    Serial.print(AngleRoll);
    Serial.print(" | Pitch [°]: ");
    Serial.println(AnglePitch);

    bool rollExceeded = (abs(AngleRoll) > angleThreshold) && (abs(RateRoll) > rateThreshold);
    bool pitchExceeded = (abs(AnglePitch) > angleThreshold) && (abs(RatePitch) > rateThreshold);

    digitalWrite(ledRollPos, (AngleRoll > angleThreshold) ? HIGH : LOW);
    digitalWrite(ledRollNeg, (AngleRoll < -angleThreshold) ? HIGH : LOW);
    digitalWrite(ledPitchPos, (AnglePitch > angleThreshold) ? HIGH : LOW);
    digitalWrite(ledPitchNeg, (AnglePitch < -angleThreshold) ? HIGH : LOW);

    if (rollExceeded || pitchExceeded) {
        spinMotor(PWM_FULL_SPEED);  // Only spin motor if both angle and rate suggest tipping
    } else {
        spinMotor(PWM_HALF_SPEED);
    }
}

void gyro_signals() {
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

    AccRoll = atan2(AccY, AccZ) * 180 / M_PI;
    AccPitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / M_PI;
}

void spinMotor(int speed) {
    digitalWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, speed);
}

void stopMotor() {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
}
