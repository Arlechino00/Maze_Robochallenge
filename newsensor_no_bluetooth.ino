/*
  Corridor Following & Gyro-Turning Robot
  
  (AUTONOMOUS VERSION: All Bluetooth commands have been removed.)
  (Robot will calibrate and run immediately on power-on.)
  
  (OPTIMIZATION: Removed Adafruit_VL53L0X.h library.
   Replaced with minimal Wire.h calls.)
*/

#include <Wire.h>
#include "xmotionV3.h"
// #include <MPU6050.h>       // <-- REMOVED to save Flash and RAM
// #include <Adafruit_VL53L0X.h> // <-- REMOVED to save Flash
// #include <math.h>          // <-- REMOVED to save flash

// ------------------- Hardware setup -------------------
// MPU6050 mpu;              // <-- REMOVED to save RAM
#define MPU6050_ADDR 0x68   // <-- NEW: I2C address for MPU6050
// Adafruit_VL53L0X lox = Adafruit_VL53L0X(); // <-- REMOVED to save RAM
#define VL53L0X_ADDR 0x29 // <-- NEW: I2C address for VL53L0X

// --- IR Sensor Pins ---
#define SENSOR_RIGHT A1
#define SENSOR_LEFT  A2
// (Front sensor is on I2C pins D2/SDA and D3/SCL)

// ------------------- Global Variables -------------------
unsigned long prevTime = 0;
float dt; // delta time

// --- Gyro Variables ---
// int16_t gx, gy, gz;       // <-- No longer needed, handled in function
float gyroZoffset = 0;
float yaw = 0.0;
float GYRO_SCALE = 131.0; 

// --- VL53L0X Minimal Driver Globals ---
uint8_t stop_variable; // Required for VL53L0X init
uint32_t measurement_timing_budget_us;

// --- Bluetooth ---
// *** ALL COMMAND LOGIC REMOVED ***

// --- Sensor Filtering (NEW) ---
float leftFiltered = 40.0;  // Initial "far" value
float rightFiltered = 40.0; // Initial "far" value
const float EMA_ALPHA = 0.35; // Smoothing factor (0.0 - 1.0)
int frontDistance_mm = 8190;  // *** NEW: Holds front sensor distance (mm) ***

// --- State Machine ---
enum RobotState { 
  STOPPED, // Used only for error
  DRIVING_CORRIDOR,
  APPROACHING_WALL,
  TURN_AT_WALL,
  CLEAR_CORNER 
};
RobotState currentState = DRIVING_CORRIDOR; // Start in this state
unsigned long stateTimer = 0; // Timer for CLEAR_CORNER
long clearCornerDuration = 400; 

// --- Tuning: IR Sensors ---
float WALL_DISTANCE_CM = 13.0; // Pivot distance for *side* walls

// --- Tuning: ToF Front Sensor (mm) ---
// *** NEW: Distances are now in MILLIMETERS ***
int FRONT_WALL_MM = 250; // (25cm) Distance to switch from DRIVING to APPROACHING
int APPROACH_STOP_DISTANCE_MM = 120; // (12cm) Final stopping distance

// --- Tuning: Motion ---
int baseSpeed = 40; // forward speed 0–100
int pivotSpeed = 22; // Slow speed for pivoting & approaching

// --- Tuning: PID (Wall Following) ---
float Kp = 0.1;   // Proportional
float Ki = 0.2;   // Integral
float Kd = 0.01;  // Derivative
float integral = 0;
float integral_clamp = 25.0;
float previous_wall_error = 0.0; 

// --- Tuning: P-Control (Turning) ---
float Kp_turn = 0.5;   // P-gain for turning
int maxTurnSpeed = 20; // Max speed for turning (0-100)
int minTurnSpeed = 15; // Min speed to overcome friction
float turn_tolerance = 5.0;   // How close is "good enough"? (in degrees)


// ------------------- Setup -------------------
void setup() {
  // Serial.begin(9600);       // <-- REMOVED to save space
  Serial1.begin(9600);      // Bluetooth Module (For error messages)
  Wire.begin(); 
  
  // Gyro Init
  initMPU6050(); // <-- NEW minimal init function
  
  // *** NEW: Initialize VL53L0X Front Sensor ***
  if (!initVL53L0X()) { // <-- NEW minimal init function
    Serial1.println("Error: VL53L0X not found."); // <-- CRITICAL ERROR, KEEP
    currentState = STOPPED; // Don't run if sensor fails
    while (1) { xmotion.UserLed2(100); } // Blink error
  }
  // *** END NEW SENSOR SETUP ***
  
  calibrateGyro(); // Calibrate gyro on startup
  
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

    case APPROACHING_WALL:
      handleApproachingWall();
      break;

    case TURN_AT_WALL:
      handleTurnAtWall(); 
      break;
      
    case CLEAR_CORNER:
      handleClearCorner(); 
      break;
  }
  
  delay(10); // Main loop delay
}

// ------------------- Gyro calibration -------------------
void calibrateGyro() {
  // *** CHANGED: Set state to DRIVING right after calibration ***
  currentState = DRIVING_CORRIDOR; 
  xmotion.MotorControl(0, 0);
  
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += readGyroZ(); 
    xmotion.UserLed2(1); // <-- Visual feedback for calibration
    delay(3); 
  }
  gyroZoffset = sum / 500.0;
  
  yaw = 0;
  integral = 0;
  previous_wall_error = 0;
}


// ------------------- State Functions -------------------
void stopMotorsAndResetPID() {
  xmotion.MotorControl(0, 0); // Ensure motors are off
  integral = 0;
  previous_wall_error = 0;
}

void handleCorridorFollowing() {
  // --- Read Sensors (FILTERED + FRONT LIDAR) ---
  float leftCm, rightCm;
  readAndFilterSensors(leftCm, rightCm, frontDistance_mm);

  // --- Logic Priority 1: Head-On Wall (Using LIDAR) ---
  if (frontDistance_mm < FRONT_WALL_MM) {
      currentState = APPROACHING_WALL; 
      return; 
  }
  
  // --- Logic Priority 2: Side Wall Pivot ---
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

  // --- Logic Priority 3: PID Corridor Following ---
  float wall_error = leftCm - rightCm;

  float p_term = Kp * wall_error;
  integral += wall_error * dt;
  integral = constrain(integral, -integral_clamp, integral_clamp);
  float i_term = Ki * integral;

  float derivative = 0;
  if (dt > 0) {
    derivative = (wall_error - previous_wall_error) / dt;
  }
  previous_wall_error = wall_error;
  float d_term = Kd * derivative;

  float correction = p_term + i_term + d_term;

  int leftSpeed = constrain(baseSpeed + correction, 0, 100);
  int rightSpeed = constrain(baseSpeed - correction, 0, 100);

  xmotion.MotorControl(leftSpeed, rightSpeed);
}

void handleApproachingWall() {
  // --- Read Sensors (NOW FILTERED + FRONT LIDAR) ---
  float leftCm, rightCm;
  readAndFilterSensors(leftCm, rightCm, frontDistance_mm);
  
  int avgDistance = frontDistance_mm;

  if (avgDistance <= APPROACH_STOP_DISTANCE_MM) {
    xmotion.MotorControl(0, 0); // Stop
    delay(100); // Settle
    currentState = TURN_AT_WALL;
  } else {
    // Not close enough, keep creeping forward
    xmotion.MotorControl(pivotSpeed, pivotSpeed);
  }
}

// This state decides which way to turn based on *side* sensors.
void handleTurnAtWall() {
  // --- Read Sensors (We only care about side sensors here) ---
  float leftCm, rightCm;
  readAndFilterSensors(leftCm, rightCm, frontDistance_mm);

  float turnAngle;

  if (leftCm > rightCm) {
    turnAngle = -90.0;
  } else {
    turnAngle = 90.0;
  }
  
  TurnToAngle(turnAngle); // Call the blocking gyro turn

  // When turn is done:
  yaw = 0; 
  integral = 0;
  previous_wall_error = 0;
  stateTimer = millis(); 
  currentState = CLEAR_CORNER;
}

// New state function to drive straight for a bit
void handleClearCorner() {
  if (millis() - stateTimer > clearCornerDuration) {
    currentState = DRIVING_CORRIDOR;
    return;
  } 
  
  // --- Drive straight using the corridor PID ---
  float leftCm, rightCm;
  readAndFilterSensors(leftCm, rightCm, frontDistance_mm);
  
  if (frontDistance_mm < FRONT_WALL_MM) {
      currentState = APPROACHING_WALL;
      return; 
  }
  
  float wall_error = leftCm - rightCm;
  float p_term = Kp * wall_error;
  integral += wall_error * dt;
  integral = constrain(integral, -integral_clamp, integral_clamp);
  float i_term = Ki * integral;

  float derivative = 0;
  if (dt > 0) {
    derivative = (wall_error - previous_wall_error) / dt;
  }
  previous_wall_error = wall_error;
  float d_term = Kd * derivative;

  float correction = p_term + i_term + d_term;

  int leftSpeed = constrain(baseSpeed + correction, 0, 100);
  int rightSpeed = constrain(baseSpeed - correction, 0, 100);

  xmotion.MotorControl(leftSpeed, rightSpeed);
}


// ------------------- Helper Functions -------------------

// *** FLASH FIX: Replaced pow() with map() ***
float RawToCm(int rawValue) {
  if (rawValue < 82) return 80.0; 
  if (rawValue > 470) return 10.0; 
  return map(rawValue, 82, 470, 80, 10);
}

/**
 * @brief Reads sensors, applies EMA filter, and gets ToF distance.
 */
void readAndFilterSensors(float &leftCm, float &rightCm, int &frontMm) {
  // 1. Get the raw distance for side sensors
  float rawL = RawToCm(analogRead(SENSOR_LEFT));
  float rawR = RawToCm(analogRead(SENSOR_RIGHT));

  // 2. Apply the EMA filter to side sensors
  leftFiltered = (EMA_ALPHA * rawL) + ((1.0 - EMA_ALPHA) * leftFiltered);
  rightFiltered = (EMA_ALPHA * rawR) + ((1.0 - EMA_ALPHA) * rightFiltered);

  // 3. Output the smoothed side values
  leftCm = leftFiltered;
  rightCm = rightFiltered;
  
  // 4. Read the front I2C sensor (VL53L0X)
  frontMm = readVL53L0X(); // <-- NEW minimal read function

  // 5. Check if the reading timed out
  if (frontMm == 8190) { // <-- NEW check
    frontMm = 8190; // Set to max range, which means "clear path"
  }
}


// ------------------- Precise Turn Function -------------------
// This is a BLOCKING function.
float getCurrentYawForTurn() {
  unsigned long currentTime_turn = millis();
  float turn_dt = (currentTime_turn - prevTime) / 1000.0;
  prevTime = currentTime_turn;
  
  float gzCorrected = readGyroZ() - gyroZoffset; 

  float deltaYaw = (gzCorrected * turn_dt / GYRO_SCALE) * -1.0; 
  
  yaw += deltaYaw; // Modifies the global 'yaw'
  return yaw;
}

void TurnToAngle(float targetDelta) {
    yaw = 0; // Reset cumulative yaw
    prevTime = millis();
    unsigned long startTime = millis();
    bool turningRight = (targetDelta > 0);
    
    while (true) {
        // Check for bluetooth stop command // <-- REMOVED
        
        float currentYaw = getCurrentYawForTurn();
        float error = abs(targetDelta - currentYaw);

        if (error <= turn_tolerance) {
            break; // Done!
        }

        int motorSpeed = (int)(error * Kp_turn);
        motorSpeed = constrain(motorSpeed, minTurnSpeed, maxTurnSpeed);

        // *** OPTIMIZATION: Replaced Left0/Right0 with MotorControl ***
        if (turningRight) {
            xmotion.MotorControl(-motorSpeed, motorSpeed);
        } else {
            xmotion.MotorControl(motorSpeed, -motorSpeed);
        }

        if (millis() - startTime > 3000) { 
            break;
        }
        
        delay(10); // Loop delay
    }
    
    xmotion.StopMotors(100);
    delay(100); // Wait for robot to settle
}


// ------------------- Bluetooth Command Functions -------------------
// *** ALL COMMAND FUNCTIONS REMOVED ***


// *** NEW LIGHTWEIGHT MPU6050 FUNCTIONS ***

/**
 * @brief Wakes up MPU6050 and sets gyro range to 250dps
 */
void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00); // Set to 250dps (which matches GYRO_SCALE 131.0)
  Wire.endTransmission(true);
}

/**
 * @brief Reads the raw GZ register from MPU6050
 * @return int16_t raw GZ value
 */
int16_t readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47); // Start at GZ high byte
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true); // Read 2 bytes
  int16_t gz_raw = (Wire.read() << 8) | Wire.read();
  return gz_raw;
}

// *** NEW LIGHTWEIGHT VL53L0X FUNCTIONS ***
// These functions replace the Adafruit_VL53L0X library

bool initVL53L0X() {
  // Check sensor ID
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(0xC0);
  Wire.endTransmission();
  Wire.requestFrom(VL53L0X_ADDR, 1);
  if (Wire.read() != 0xEE) { return false; } // Sensor not found

  // Minimal init sequence
  vlx_writeReg(0x88, 0x00);
  vlx_writeReg(0x80, 0x01);
  vlx_writeReg(0xFF, 0x01);
  vlx_writeReg(0x00, 0x00);
  stop_variable = vlx_readReg(0x91);
  vlx_writeReg(0x00, 0x01);
  vlx_writeReg(0xFF, 0x00);
  vlx_writeReg(0x80, 0x00);

  // Set high-speed, 33ms timing budget (default)
  vlx_writeReg(0x80, 0x01);
  vlx_writeReg(0xFF, 0x01);
  vlx_writeReg(0x00, 0x00);
  vlx_writeReg(0x91, stop_variable);
  vlx_writeReg(0x00, 0x01);
  vlx_writeReg(0xFF, 0x00);
  vlx_writeReg(0x80, 0x00);
  vlx_writeReg(0x50, 0x00); // Set high speed
  vlx_writeReg(0x44, 0xFF); 
  vlx_writeReg(0x45, 0x00); 
  vlx_writeReg(0x50, 0x20); // Set 33ms budget
  vlx_writeReg(0x60, 0x0B); 
  vlx_writeReg(0x80, 0x01);
  vlx_writeReg(0xFF, 0x01);
  vlx_writeReg(0x00, 0x00);
  vlx_writeReg(0x91, stop_variable);
  vlx_writeReg(0x00, 0x01);
  vlx_writeReg(0xFF, 0x00);
  
  // Start continuous mode
  vlx_writeReg(0x80, 0x01);
  vlx_writeReg(0xFF, 0x01);
  vlx_writeReg(0x00, 0x00);
  vlx_writeReg(0x91, stop_variable);
  vlx_writeReg(0x00, 0x01);
  vlx_writeReg(0xFF, 0x00);
  vlx_writeReg(0x80, 0x00);
  vlx_writeReg(0x0B, 0x01);
  vlx_writeReg(0x80, 0x01);
  vlx_writeReg(0xFF, 0x01);
  vlx_writeReg(0x00, 0x00);
  vlx_writeReg(0x91, stop_variable);
  vlx_writeReg(0x00, 0x01);
  vlx_writeReg(0xFF, 0x00);
  vlx_writeReg(0x80, 0x00);
  vlx_writeReg(0x00, 0x01);
  vlx_writeReg(0x0B, 0x02); // Start continuous ranging
  vlx_writeReg(0x80, 0x00);

  return true;
}

uint16_t readVL53L0X() {
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(0x14); // VL53L0X_REG_RESULT_RANGE_STATUS
  Wire.endTransmission();
  Wire.requestFrom(VL53L0X_ADDR, 12); // Read 12 bytes
  
  if (Wire.available() < 12) { return 8190; } // Read failed
  
  uint8_t status = Wire.read();
  Wire.read(); // Read and discard
  Wire.read(); // Read and discard
  Wire.read(); // Read and discard
  Wire.read(); // Read and discard
  Wire.read(); // Read and discard
  Wire.read(); // Read and discard
  Wire.read(); // Read and discard
  Wire.read(); // Read and discard
  Wire.read(); // Read and discard
  
  uint16_t range = (Wire.read() << 8) | Wire.read(); // Read 2-byte distance
  
  // Clear interrupt
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  
  if ((status & 0x78) != 0) { return 8190; } // Error in measurement

  return range;
}

// --- VL53L0X I2C Helper Functions ---
void vlx_writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t vlx_readReg(uint8_t reg) {
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(VL53L0X_ADDR, 1);
  return Wire.read();
}

