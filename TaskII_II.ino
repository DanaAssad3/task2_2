#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal.h>
#include <Encoder.h>

const int rs = 12, en = 11, d4 = 5, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

MPU6050 mpu;

const int motorEnableA = 4;
const int motorInput1 = 7;
const int motorInput2 = 6;

Encoder myEnc(2, 3);

int motorSpeed = 0;
long targetPosition = 0;
long lastPosition = 0;

const float Kp = 2.0;
const float Ki = 0.05;
const float Kd = 0.5;

float integral = 0;
long lastError = 0;
unsigned long lastTime = 0;

int16_t ax, ay, az, gx, gy, gz;
unsigned long lastMPURead = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU connected!" : "MPU failed!");
  lcd.begin(16, 2);
  lcd.print("Starting...");
  pinMode(motorEnableA, OUTPUT);
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  long currentPosition = myEnc.read();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float steeringRate = gz / 131.0;
  targetPosition = steeringRate * 2;
  long error = targetPosition - currentPosition;
  integral += error * deltaTime;
  integral = constrain(integral, -50, 50);
  float derivative = (error - lastError) / deltaTime;
  lastError = error;
  motorSpeed = (Kp * error) + (Ki * integral) + (Kd * derivative);
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (abs(motorSpeed) < 30 && abs(error) > 2) {
    motorSpeed = (motorSpeed > 0) ? 30 : -30;
  }
  setMotorSpeed(motorSpeed);
  if (currentPosition != lastPosition) {
    updateLCD(currentPosition, targetPosition, error);
    updateSerial(currentPosition, targetPosition, error);
    lastPosition = currentPosition;
  }
  delay(10);
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (abs(speed) < 20) {
    speed = 0;
  }
  if (speed >= 0) {
    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
  } else {
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, HIGH);
  }
  analogWrite(motorEnableA, abs(speed));
}

void updateLCD(long pos, long target, long error) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("P:");
  lcd.print(pos);
  lcd.print(" T:");
  lcd.print(target);
  lcd.setCursor(0, 1);
  lcd.print("E:");
  lcd.print(error);
  lcd.print(" S:");
  lcd.print(motorSpeed);
}

void updateSerial(long pos, long target, long error) {
  Serial.print("Pos: ");
  Serial.print(pos);
  Serial.print(" | Target: ");
  Serial.print(target);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | Dir: ");
  Serial.println(motorSpeed >= 0 ? "CW" : "CCW");
}