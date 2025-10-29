/*
  Corridor Following & Turning Robot (with ToF Sensor)
 
  Merges three logics:
  1. PID Corridor Following (using side-facing IR sensors)
  2. Gyro-based 90-degree turns (using MPU6050)
  3. Precise Front Wall Detection (using VL53L0X ToF Sensor)
 
  Logic:
  - Follows corridor using PID on (leftCm - rightCm) from IRs.
  - If a single side IR is too close, pivots away.
  - If a FRONT wall is detected (using ToF sensor):
    1. Approaches the wall to a set distance (using ToF).
    2. Performs a precise 90-degree gyro turn.
    
  *** FLASH OPTIMIZATIONS ***
  1. Removed MPU6050.h library. Replaced with direct I2C calls.
  2. Reverted RawToCm() to use pow() from math.h per user request.
  3. REMOVED ALL Serial and Bluetooth communication to save Flash.
  4. Robot now auto-starts on boot.
*/

#include <Wire.h>
#include "xmotionV3.h"
#include <math.h> // <-- RE-ADDED for pow()
#include "Adafruit_VL53L0X.h"

// ------------------- Hardware setup -------------------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ------------------- MPU6050 Manual I2C Setup -------------------
#define MPU_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_GYRO_CONFIG 0x1B
#define MPU_GYRO_XOUT_H 0x43
// -------------------------------------------------------------

// --- Sensor Pins ---
#define SENSOR_RIGHT A1 // Side-facing IR (at 30-deg)
#define SENSOR_LEFT  A2 // Side-facing IR (at 30-deg)
// ToF Sensor uses I2C pins D2 (SDA) and D3 (SCL)

// ------------------- Global Variables -------------------
unsigned long prevTime = 0;
float dt; // delta time

// --- Sensor Readings ---
float leftCm, rightCm; // From IR sensors
float frontCm = 80.0;  // From ToF sensor
static int tofErrorCount = 0; // Error counter for ToF sensor

// --- Gyro Variables ---
int16_t gx, gy, gz;
float gyroZoffset = 0;
float yaw = 0.0;
float GYRO_SCALE = 131.0; // MPU6050 at 250dps

// --- Bluetooth ---
// ALL BLUETOOTH AND SERIAL FUNCTIONALITY REMOVED TO SAVE FLASH

// --- State Machine ---
enum RobotState { 
  STOPPED, 
  DRIVING_CORRIDOR
};
// Auto-start on boot, since we have no 'start' command
RobotState currentState = DRIVING_CORRIDOR; 

// --- Tuning: Sensors ---
// *** TUNING UPDATE for 30cm corridor ***
// With 30-deg sensors, center reading is ~17.3cm.
// Pivot logic must be for emergencies only.
float WALL_DISTANCE_CM = 10.0; // Pivot distance (was 15.0)
float FRONT_WALL_CM = 20.0;     // Detect distance for *front* wall
float APPROACH_STOP_DISTANCE_CM = 10.0; // Stop dist for front wall
float TOF_SLOWDOWN_START_CM = 40.0; 

// --- Tuning: Motion ---
// *** TUNING UPDATE for 30cm corridor ***
int baseSpeed = 25; // forward speed 0–100 (Reduced from 40 for narrow corridor)
int pivotSpeed = 30; // Slow speed for pivoting & approaching

// --- Tuning: PID (Wall Following) ---
// *** TUNING UPDATE for 30cm corridor ***
float Kp = 0.1; // Proportional
float Ki = 0.05; // Integral (Reduced from 0.05 to prevent wall-hugging)
float Kd = 0.01;  // Derivative (ADDED to prevent oscillation)
float integral = 0;
float integral_clamp = 50.0; // (Reduced from 100.0 to prevent wall-hugging)
float previous_wall_error = 0.0;

// --- Tuning: P-Control (Turning) ---
float Kp_turn = 0.5;    // P-gain for turning
int maxTurnSpeed = 20; // Max speed for turning (0-100)
int minTurnSpeed = 15; // Min speed to overcome friction
float turn_tolerance = 2.0;    // How close is "good enough"? (in degrees)


// ------------------- Setup -------------------
void setup() {
  Wire.begin(); 
  
  // --- Robust I2C Device Init ---
  // Wait for ToF sensor to be ready (prevents startup hang)
  while (!lox.begin()) {
    delay(200); // Wait 200ms and retry
  }
 
  // Wait for MPU sensor to be ready (prevents startup hang)
  while (true) {
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() == 0) {
      break; // Device found, exit loop
    }
    delay(200); // Wait 200ms and retry
  }
  // --- End Robust Init ---

  // Gyro Init
  setupMPU();
  calibrateGyro(); 
 
  prevTime = millis();
}

// ------------------- Main loop: The State Machine -------------------
void loop() {
  // checkBluetoothCommands(); // <-- REMOVED

  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  switch(currentState) {
    case STOPPED:
      stopMotorsAndResetPID();
      break;
      
    case DRIVING_CORRIDOR:
      if (dt > 0) {
        handleCorridorFollowing();
      }
      break;
  }
  
  delay(10); // Main loop delay
}

// ------------------- MPU6050 Helper Functions (replaces library) ---

void setupMPU() {
  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0); // Set to 0 (wakes up)
  Wire.endTransmission(true);

  // Set Gyro sensitivity to 250dps
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_CONFIG);
  Wire.write(0x00); // Sets FS_SEL=0 for 250dps
  Wire.endTransmission(true);
}

void getGyroRotation(int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_XOUT_H); // Start register
  Wire.endTransmission(false); // false = restart

  // Request 6 bytes (GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L)
  Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true); 

  // Read the 6 bytes
  *gx = (Wire.read() << 8) | Wire.read();
  *gy = (Wire.read() << 8) | Wire.read();
  *gz = (Wire.read() << 8) | Wire.read();
}
// --------------------------------------------------------

// ------------------- Gyro calibration -------------------
void calibrateGyro() {
  currentState = STOPPED; // Stop motors during calibration
  xmotion.MotorControl(0, 0);
  
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    getGyroRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }
  gyroZoffset = sum / 500.0;
  
  yaw = 0;
  integral = 0;
  previous_wall_error = 0;
  
  currentState = DRIVING_CORRIDOR; // Ready to go
}

// ------------------- State Functions -------------------

void stopMotorsAndResetPID() {
  xmotion.MotorControl(0, 0); 
  integral = 0;
  previous_wall_error = 0;
}

void handleCorridorFollowing() {
  // --- Read Sensors ---
  leftCm = RawToCm(analogRead(SENSOR_LEFT));
  rightCm = RawToCm(analogRead(SENSOR_RIGHT));
  frontCm = readToFDistance();

  // --- Logic Priority 1: Head-On Wall (NOW USES ToF) ---
  if (frontCm < FRONT_WALL_CM) { 
      // 1. Approach the wall slowly using ToF
      while (frontCm > APPROACH_STOP_DISTANCE_CM) {
          xmotion.MotorControl(pivotSpeed, pivotSpeed); 
          delay(10); 
          
          frontCm = readToFDistance();
          
          // checkBluetoothCommands(); // <-- REMOVED
          // No way to stop now, it's committed
      }
      
      // 2. Stop and perform the 90-degree "scan" turn
      xmotion.MotorControl(0, 0);
      delay(100); 
      
      TurnToAngle(90.0); 
      
      // 3. Reset PID and yaw
      integral = 0;
      previous_wall_error = 0;
      yaw = 0; // Reset gyro yaw
      return; 
  }
  
  // --- Logic Priority 2: Front-Quarter Pivot (Using 30-deg IRs) ---
  // This is now an emergency pivot, as WALL_DISTANCE_CM is low
  else if (leftCm < WALL_DISTANCE_CM || rightCm < WALL_DISTANCE_CM) {
    if (leftCm < rightCm) {
        xmotion.MotorControl(pivotSpeed, -pivotSpeed);
    } else {
        xmotion.MotorControl(-pivotSpeed, pivotSpeed);
    }
    integral = 0;
    previous_wall_error = 0;
    return; 
  }

  // --- Logic Priority 3: PID Corridor Following (OLD - Uses IR) ---
  
  // --- NEW ToF-based Speed Scaling ---
  float speedFactor = 1.0; // Full speed
  
  if (frontCm < TOF_SLOWDOWN_START_CM) {
    speedFactor = (frontCm - APPROACH_STOP_DISTANCE_CM) / (TOF_SLOWDOWN_START_CM - APPROACH_STOP_DISTANCE_CM);
    speedFactor = constrain(speedFactor, 0.0, 1.0);
  }
  int currentBaseSpeed = (int)(baseSpeed * speedFactor);
  // --- END NEW SPEED SCALING ---

  // *** LOGIC FIX (USER): Error calculation swapped to leftCm - rightCm ***
  // A positive error (too close to right) now results in a
  // positive correction.
  float wall_error = leftCm - rightCm;

  float p_term = Kp * wall_error;
  integral += wall_error * dt;
  integral = constrain(integral, -integral_clamp, integral_clamp);
  float i_term = Ki * integral;

  // --- NEW DERIVATIVE TERM ---
  float derivative = 0;
  if (dt > 0) {
    derivative = (wall_error - previous_wall_error) / dt;
  }
  previous_wall_error = wall_error;
  float d_term = Kd * derivative;
  // --- END DERIVATIVE ---

  float correction = p_term + i_term + d_term;

  // *** LOGIC FIX (USER): Motor logic set to - and + ***
  // A positive error (too close to right) now *decreases* left speed
  // and *increases* right speed, forcing a turn to the LEFT.
  int leftSpeed = constrain(currentBaseSpeed + correction, 0, 100);
  int rightSpeed = constrain(currentBaseSpeed - correction, 0, 100);

  xmotion.MotorControl(leftSpeed, rightSpeed);

  // --- Print tuning data ---
  // ALL SERIAL1 PRINTS REMOVED
}

// ------------------- Helper Functions -------------------

// --- ROBUST ToF SENSOR HELPER ---
float readToFDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  
  lox.rangingTest(&measure, false);

  // Check for "Out of Range"
  if (measure.RangeStatus == 4) {
    tofErrorCount++;
    if (tofErrorCount > 10) { // Sensor might be stuck
      lox.begin(); // Try to re-initialize
      tofErrorCount = 0;
    }
    return 80.0; // "far"
  }

  // Check for any other error state
  if (measure.RangeStatus != 0) {
    tofErrorCount++;
    if (tofErrorCount > 10) {
      lox.begin();
      tofErrorCount = 0;
    }
    return 80.0; // Treat other errors as "far" too
  }

  // If we get here, RangeStatus == 0 (Valid)
  tofErrorCount = 0; // Reset error count
  
  float cm = (float)measure.RangeMilliMeter / 10.0; // Convert mm to cm
  
  // --- Check for dangerously low values to prevent sensor lock-up ---
  if (cm < 2.0) { // If reading is < 20mm (dangerously close)
    return 0.0;   // Return 0.0, which will trigger stop/turn logic
  }
  return cm;
}

// --- MODIFIED HELPER: Read IR Sensors ---
// Reverted to use pow() per user request
float RawToCm(int rawValue) {
  float voltage = rawValue * (5.0 / 1023.0);
  if (voltage < 0.25) return 80.0; // Return a "far" value
  float distance = 27.86 * pow(voltage, -1.15); 
  if (distance > 80) return 80; // Clamp max distance
  if (distance < 10) return 10; // Clamp min distance (deadzone)
  return distance;
}


// ------------------- Precise Turn Function (from User) -------------------
float getCurrentYawForTurn() {
  unsigned long currentTime_turn = millis();
  float turn_dt = (currentTime_turn - prevTime) / 1000.0;
  prevTime = currentTime_turn;
  
  getGyroRotation(&gx, &gy, &gz);
  float gzCorrected = gz - gyroZoffset;

  float deltaYaw = (gzCorrected * turn_dt / GYRO_SCALE) * -1.0; 
  
  yaw += deltaYaw; 
  return yaw;
}

void TurnToAngle(float targetDelta) {
    yaw = 0; 
    prevTime = millis();
    unsigned long startTime = millis();
    bool turningRight = (targetDelta > 0);
    
    while (true) {
        float currentYaw = getCurrentYawForTurn();
        float error = abs(targetDelta - currentYaw);

        if (error <= turn_tolerance) {
            break; 
        }

        int motorSpeed = (int)(error * Kp_turn);
        motorSpeed = constrain(motorSpeed, minTurnSpeed, maxTurnSpeed);

        if (turningRight) {
            xmotion.Right0(motorSpeed, 0); 
        } else {
            xmotion.Left0(motorSpeed, 0);  
        }

        if (millis() - startTime > 4000) { 
            // Turn timeout
            break;
        }
        
        delay(10); 
    }
    
    xmotion.StopMotors(100);
    delay(100);
}


// ------------------- Bluetooth Command Functions -------------------
// ALL BLUETOOTH AND SERIAL FUNCTIONALITY REMOVED TO SAVE FLASH

