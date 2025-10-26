#include <Wire.h>
#include "xmotionV3.h"
#include <MPU6050.h>

// ------------------- Hardware setup -------------------
MPU6050 mpu;

// IR sensor pins
#define RIGHT_IR A1
#define LEFT_IR  A2

// ------------------- Motor Calibration -------------------
/*** NEW ***/
// Tune this value (try 8 → 12 → 15...) until the robot drives straight
#define CALIBRATION_OFFSET 8 

// ------------------- Variables -------------------
int16_t gx, gy, gz;
float gyroZoffset = 0;
float yaw = 0;
unsigned long prevTime = 0;

// Motion tuning
int baseSpeed = 20;       // forward speed 0–100
int wallThreshold = 375;  // IR threshold for wall

float dt;

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  calibrateGyro();
  Serial.println("Maze robot ready!");
}

// ------------------- Gyro calibration -------------------
void calibrateGyro() {
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroZoffset = sum / 500.0;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZoffset);
}

// ------------------- Main loop -------------------
void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  mpu.getRotation(&gx, &gy, &gz);
  float gzCorrected = gz - gyroZoffset;
  yaw += gzCorrected * dt / 131.0;

  int rightDist = analogRead(RIGHT_IR);
  int leftDist = analogRead(LEFT_IR);

  bool wallAhead = ((rightDist >= wallThreshold && leftDist >= wallThreshold ) || (rightDist - leftDist >= 200 || leftDist - rightDist >= 200));
  bool wallLeft  = (leftDist >= wallThreshold);
  bool wallRight = (rightDist >= wallThreshold);

  Serial.print("L: "); Serial.print(leftDist);
  Serial.print("\tR: "); Serial.print(rightDist);
  Serial.print("\tYaw: "); Serial.println(yaw);

  // ------------------- Navigation logic -------------------
  if (wallAhead) {
    xmotion.StopMotors(100);

    if (rightDist <= leftDist) {
      Serial.println("Turning RIGHT");
      xmotion.Right0(20, 140); // right turn
    } 
    else if (rightDist >= leftDist) {
      Serial.println("Turning LEFT");
      xmotion.Left0(20, 140);  // left turn
    } 
    
  }
  else {
    // Forward movement with wall correction
    int error = rightDist - leftDist;
    int correction = error / 20; // adjust for smoother/stronger steering

    // Apply motor calibration offset (left motor slightly slower)
    int leftSpeed = constrain(baseSpeed + correction - CALIBRATION_OFFSET, 0, 100);
    int rightSpeed = constrain(baseSpeed - correction, 0, 100);

    xmotion.ArcTurn(leftSpeed, rightSpeed, 20);
  }

  delay(20);
}
