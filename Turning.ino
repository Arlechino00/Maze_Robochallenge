#include <Wire.h>
#include "xmotionV3.h"
#include <MPU6050.h>

// ------------------- Hardware setup -------------------
MPU6050 mpu;
int16_t gx, gy, gz;
float gyroZoffset = 0;
float yaw = 0;
unsigned long prevTime = 0;

// ------------------- TUNING PARAMETERS -------------------
// These are the values you will adjust to get a perfect turn.

// 1. Proportional Gain (THE MOST IMPORTANT)
//    - Too HIGH = Overshoots (turns > 90)
//    - Too LOW  = Undershoots or turns very slowly
//    - STARTING VALUE: 0.8
float Kp = 0.5;

// 2. Max/Min Speeds
//    - maxTurnSpeed: The fastest the robot will turn (at the start)
//    - minTurnSpeed: The slowest speed to overcome friction and finish the turn
//    - STARTING VALUES: 15 / 10
int maxTurnSpeed = 20;
int minTurnSpeed = 15;

// 3. Tolerance
//    - How many degrees from the target is "good enough"?
//    - STARTING VALUE: 2.0
float tolerance = 2.0;

// 4. Gyro Scale (Should be correct for MPU6050)
float GYRO_SCALE = 131.0;


// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600); // Use the USB Serial Monitor
  Wire.begin();
  
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  Serial.println("Calibrating gyro...");
  calibrateGyro();
  Serial.println("Calibration done!");
  
  Serial.println("\n=== Turn Tuning Script ===");
  Serial.println("Send commands via Serial Monitor:");
  Serial.println("  'R' = Turn Right 90°");
  Serial.println("  'L' = Turn Left 90°");
  Serial.println("  'U' = Turn 180°");
  Serial.println("==========================");
}

// ------------------- Main Loop -------------------
void loop() {
  // Check for a command from the Serial Monitor
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'R' || cmd == 'r') {
      Serial.println("\nTesting: RIGHT 90°");
      turnByAngle(90);
    } 
    else if (cmd == 'L' || cmd == 'l') {
      Serial.println("\nTesting: LEFT 90°");
      turnByAngle(-90);
    }
    else if (cmd == 'U' || cmd == 'u') {
      Serial.println("\nTesting: 180° (U-Turn)");
      turnByAngle(180);
    }
  }
  
  delay(50); // Don't spam the loop
}

// ------------------- Gyro Calibration -------------------
void calibrateGyro() {
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }
  gyroZoffset = sum / 500.0;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZoffset);
}

// ------------------- Get Current Yaw -------------------
// (The non-wrapping version)
// ------------------- Get Current Yaw -------------------
// (The non-wrapping version)
float getCurrentYaw() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  
  mpu.getRotation(&gx, &gy, &gz);
  float gzCorrected = gz - gyroZoffset;

  // [!] FIX: Invert the gyro reading.
  // We add * -1.0 because a right turn (positive target)
  // was causing a negative yaw.
  float deltaYaw = (gzCorrected * dt / GYRO_SCALE) * -1.0; 
  
  yaw += deltaYaw;
  
  return yaw;
}

// ------------------- Precise Turn with P-Control -------------------
// This is the function we are tuning.
// It uses the global tuning parameters.

void turnByAngle(float targetDelta) {
    yaw = 0; // Reset cumulative yaw
    prevTime = millis();
    unsigned long startTime = millis();
    bool turningRight = (targetDelta > 0);

    // Print a header for the data
    Serial.println("Target | Current | Error | Speed");
    Serial.println("---------------------------------");
    
    while (true) {
        // Get the current accumulated angle
        float currentYaw = getCurrentYaw();
        
        // Calculate how far we are from the target
        float error = abs(targetDelta - currentYaw);

        // Check if we have reached the target
        if (error <= tolerance) {
            break; // Done!
        }

        // --- Proportional Control ---
        // Calculate the speed based on the error.
        int motorSpeed = (int)(error * Kp);
        
        // Constrain the speed to be within our min/max limits
        motorSpeed = constrain(motorSpeed, minTurnSpeed, maxTurnSpeed);

        // Apply the calculated speed
        if (turningRight) {
            xmotion.Right0(motorSpeed, 0);
        } else {
            xmotion.Left0(motorSpeed, 0);
        }

        // Safety timeout
        if (millis() - startTime > 4000) { 
            Serial.println("⚠️ Turn timeout!");
            break;
        }

        // --- REAL-TIME DEBUGGING ---
        // Print all data on one line to watch it stream by
        Serial.print(targetDelta); Serial.print("   | ");
        Serial.print(currentYaw, 2); Serial.print("    | ");
        Serial.print(error, 2); Serial.print(" | ");
        Serial.println(motorSpeed);
        
        delay(10); // Loop delay
    }
    
    xmotion.StopMotors(100);
    delay(300); // Wait for robot to settle
    
    float finalYaw = getCurrentYaw();
    Serial.println("---------------------------------");
    Serial.print("✅ Turn complete. Final Yaw: ");
    Serial.println(finalYaw, 4); // Print with high precision
    Serial.println("\nWaiting for next command (R, L, U)...");
}