/*
  Corridor Following & Gyro-Turning Robot
  
  Merges two logics:
  1. PID Corridor Following (using IR sensors)
  2. Precise Gyro-based 90-degree turns at dead ends.
  
  Logic:
  - Follows corridor using PID on (leftCm - rightCm)
  - If a single wall is too close, pivots away.
  - If a FRONT wall is detected (both sensors):
    1. Switches to APPROACHING_WALL state.
    2. Creeps forward to a set distance.
    3. Switches to TURN_AT_WALL state.
    4. Calls blocking TurnToAngle(90.0) gyro function.
    5. Switches to CLEAR_CORNER state to move past the turn.
    6. Switches back to DRIVING_CORRIDOR.
*/

#include <Wire.h>
#include "xmotionV3.h"
#include <MPU6050.h> // Gyro re-added
#include <math.h> // For pow() in RawToCm()

// ------------------- Hardware setup -------------------
MPU6050 mpu; // Gyro re-added

// --- IR Sensor Pins ---
#define SENSOR_RIGHT A1
#define SENSOR_LEFT  A2

// ------------------- Global Variables -------------------
unsigned long prevTime = 0;
float dt; // delta time

// --- Gyro Variables ---
int16_t gx, gy, gz;
float gyroZoffset = 0;
float yaw = 0.0;
float GYRO_SCALE = 131.0; // MPU6050 at 250dps

// --- Bluetooth ---
String serial1Buffer = ""; 

// --- State Machine ---
enum RobotState { 
  STOPPED, 
  DRIVING_CORRIDOR,
  APPROACHING_WALL,
  TURN_AT_WALL,     // Replaces SCANNING_TURN
  CLEAR_CORNER      // New state
};
RobotState currentState = STOPPED;
unsigned long stateTimer = 0; // Timer for CLEAR_CORNER
long clearCornerDuration = 300; // Drive forward 300ms

// --- Tuning: IR Sensors ---
float WALL_DISTANCE_CM = 15.0; // Pivot distance for *side* walls
float FRONT_WALL_CM = 20.0;    // Detect distance for *front* wall

// --- CRITICAL: SENSOR SAFETY LIMIT ---
// Setting this stop distance below ~12cm will cause the sensor
// to give false "far" readings when it gets too close,
// and the robot will *fail to stop*. 15cm is a safe default.
float APPROACH_STOP_DISTANCE_CM = 15.0; // Stop dist for front wall


// --- Tuning: Motion ---
int baseSpeed = 40; // forward speed 0–100
int pivotSpeed = 30; // Slow speed for pivoting & approaching

// --- Tuning: PID (Wall Following) ---
float Kp = 0.3;  // Proportional
float Ki = 0.0;  // Integral
float Kd = 0.0;  // Derivative
float integral = 0;
float integral_clamp = 25.0;
float previous_wall_error = 0.0; // For derivative term

// --- Tuning: P-Control (Turning) - RE-ADDED ---
float Kp_turn = 0.5;   // P-gain for turning
int maxTurnSpeed = 20; // Max speed for turning (0-100)
int minTurnSpeed = 15; // Min speed to overcome friction
float turn_tolerance = 2.0;   // How close is "good enough"? (in degrees)


// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);       // USB Serial Monitor
  Serial1.begin(9600);      // Bluetooth Module (TX, RX pins)
  Wire.begin(); 
  
  // Gyro Init Re-added
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  calibrateGyro(); // Calibrate gyro on startup
  
  Serial.println("Corridor/Gyro-Turner Ready!");
  Serial1.println("Corridor/Gyro-Turner Ready! Send 'help'.");
  prevTime = millis();
}

// ------------------- Main loop: The State Machine -------------------
void loop() {
  // Always check for commands
  checkBluetoothCommands();

  // Always calculate delta-time
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Run the function for our current state
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
      handleTurnAtWall(); // New state
      break;
      
    case CLEAR_CORNER:
      handleClearCorner(); // New state
      break;
  }
  
  delay(10); // Main loop delay
}

// ------------------- Gyro calibration -------------------
// Re-added from previous sketch
void calibrateGyro() {
  currentState = STOPPED; 
  xmotion.MotorControl(0, 0);
  
  Serial.println("Calibrating gyro... do not move!");
  Serial1.println("Calibrating gyro... do not move!");
  
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3); // Match user's sketch
  }
  gyroZoffset = sum / 500.0;
  
  Serial.print("Gyro Z offset: "); Serial.println(gyroZoffset);
  Serial1.print("Gyro Z offset: "); Serial1.println(gyroZoffset);
  
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
  // --- Read Sensors ---
  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));

  // --- Logic Priority 1: Head-On Wall ---
  if (leftCm < FRONT_WALL_CM && rightCm < FRONT_WALL_CM) {
      Serial1.println("Head-on wall! Switching to APPROACH.");
      currentState = APPROACHING_WALL; // Switch state
      return; // Exit function for this loop
  }
  
  // --- Logic Priority 2: Side Wall Pivot ---
  else if (leftCm < WALL_DISTANCE_CM || rightCm < WALL_DISTANCE_CM) {
    Serial1.println("Side wall detect! Pivoting...");
    if (leftCm < rightCm) {
        xmotion.MotorControl(pivotSpeed, -pivotSpeed);
    } else {
        xmotion.MotorControl(-pivotSpeed, pivotSpeed);
    }
    // Reset PID integrators so it doesn't "jerk" back
    integral = 0;
    previous_wall_error = 0;
    return; 
  }

  // --- Logic Priority 3: PID Corridor Following ---
  // (This code only runs if no walls are close)
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

  // *** UPDATED SIGNS BASED ON USER FEEDBACK ***
  int leftSpeed = constrain(baseSpeed + correction, 0, 100);
  int rightSpeed = constrain(baseSpeed - correction, 0, 100);

  xmotion.MotorControl(leftSpeed, rightSpeed);

  // --- Print tuning data ---
  Serial1.print("L_cm: "); Serial1.print(leftCm, 0);
  Serial1.print("\t R_cm: "); Serial1.print(rightCm, 0);
  Serial1.print("\t Err: "); Serial1.print(wall_error, 1);
  Serial1.print("\t L: "); Serial1.print(leftSpeed);
  Serial1.print("\t R: "); Serial1.println(rightSpeed);
}

void handleApproachingWall() {
  // This state creeps forward until it's close enough
  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));
  float avgDistance = (leftCm + rightCm) / 2.0;

  if (avgDistance <= APPROACH_STOP_DISTANCE_CM) {
    // We are close enough, stop and switch to turning
    Serial1.println("At wall. Switching to TURN_AT_WALL.");
    xmotion.MotorControl(0, 0); // Stop
    delay(100); // Settle
    currentState = TURN_AT_WALL;
  } else {
    // Not close enough, keep creeping forward
    xmotion.MotorControl(pivotSpeed, pivotSpeed);
    Serial1.print("Approaching... AvgDist: ");
    Serial1.println(avgDistance);
  }
}

// New state function to handle the blocking gyro turn
void handleTurnAtWall() {
  Serial1.println("Calling gyro turn...");
  TurnToAngle(90.0); // Call the blocking gyro turn

  // When turn is done:
  Serial1.println("Turn complete. Switching to CLEAR_CORNER.");
  yaw = 0; // Reset yaw for the new "forward"
  integral = 0;
  previous_wall_error = 0;
  stateTimer = millis(); // Set timer for clearing the corner
  currentState = CLEAR_CORNER;
}

// New state function to drive straight for a bit
void handleClearCorner() {
  if (millis() - stateTimer > clearCornerDuration) {
     // Time is up! Go back to normal driving.
    Serial1.println("Corner cleared. Resuming wall following.");
    currentState = DRIVING_CORRIDOR;
    return;
  } 
  
  // --- Drive straight using the corridor PID ---
  // (This code is copied from handleCorridorFollowing)
  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));
  
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

float RawToCm(int rawValue) {
  float voltage = rawValue * (5.0 / 1023.0);
  if (voltage < 0.25) return 80.0; // Return a "far" value
  float distance = 27.86 * pow(voltage, -1.15); 
  if (distance > 80) distance = 80;
  if (distance < 10) distance = 10;
  return distance;
}

// ------------------- Precise Turn Function (Re-added) -------------------
// This is a BLOCKING function.
float getCurrentYawForTurn() {
  unsigned long currentTime_turn = millis();
  float turn_dt = (currentTime_turn - prevTime) / 1000.0;
  prevTime = currentTime_turn;
  
  mpu.getRotation(&gx, &gy, &gz);
  float gzCorrected = gz - gyroZoffset;

  // [!] FIX from user: Invert the gyro reading.
  float deltaYaw = (gzCorrected * turn_dt / GYRO_SCALE) * -1.0; 
  
  yaw += deltaYaw; // Modifies the global 'yaw'
  return yaw;
}

void TurnToAngle(float targetDelta) {
    yaw = 0; // Reset cumulative yaw *at the start of the turn*
    prevTime = millis();
    unsigned long startTime = millis();
    bool turningRight = (targetDelta > 0);

    Serial1.println("--- Starting Gyro Turn ---");
    Serial1.println("Target | Current | Error | Speed");
    
    while (true) {
        // Check for bluetooth stop command *inside* this blocking loop
        checkBluetoothCommands();
        if (currentState == STOPPED) {
          xmotion.StopMotors(100);
          return;
        }
        
        float currentYaw = getCurrentYawForTurn();
        float error = abs(targetDelta - currentYaw);

        if (error <= turn_tolerance) {
            break; // Done!
        }

        int motorSpeed = (int)(error * Kp_turn);
        motorSpeed = constrain(motorSpeed, minTurnSpeed, maxTurnSpeed);

        if (turningRight) {
            xmotion.Right0(motorSpeed, 0); // Timeless turn right
        } else {
            xmotion.Left0(motorSpeed, 0);  // Timeless turn left
        }

        if (millis() - startTime > 4000) { 
            Serial1.println("⚠️ Turn timeout!");
            break;
        }
        
        Serial1.print(targetDelta); Serial1.print(" | ");
        Serial1.print(currentYaw, 2); Serial1.print(" | ");
        Serial1.print(error, 2); Serial1.print(" | ");
        Serial1.println(motorSpeed);
        
        delay(10); // Loop delay
    }
    
    xmotion.StopMotors(100);
    delay(100); // Wait for robot to settle
    
    Serial1.print("✅ Gyro Turn complete. Final Yaw: ");
    Serial1.println(yaw, 4);
}


// ------------------- Bluetooth Command Functions -------------------
void checkBluetoothCommands() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      parseCommand(serial1Buffer);
      serial1Buffer = "";
    } else if (c != '\r') {
      serial1Buffer += c;
    }
  }
}

void parseCommand(String command) {
  command.trim();
  if (command.startsWith("TX=")) {
    command = command.substring(3);
    command.trim();
  }
  if (command.length() == 0) return;
  
  Serial.print("Received command: "); Serial.println(command);

  if (command.equals("start")) {
    integral = 0;
    previous_wall_error = 0;
    yaw = 0; 
    currentState = DRIVING_CORRIDOR; 
    Serial1.println("Starting maze run.");
  } 
  else if (command.equals("stop")) {
    currentState = STOPPED;
    Serial1.println("Stopping robot.");
  } 
  else if (command.equals("cal")) {
    calibrateGyro(); // Re-enabled
    Serial1.println("Gyro calibrated.");
  }
  // --- PID Tuning ---
  else if (command.startsWith("kp=")) {
    Kp = command.substring(3).toFloat();
    Serial1.print("Wall Kp set: "); Serial1.println(Kp);
  }
  else if (command.startsWith("ki=")) {
    Ki = command.substring(3).toFloat();
    Serial1.print("Wall Ki set: "); Serial1.println(Ki);
  }
  else if (command.startsWith("kd=")) {
    Kd = command.substring(3).toFloat();
    Serial1.print("Wall Kd set: "); Serial1.println(Kd);
  }
  // --- Turn Tuning (Re-added) ---
  else if (command.startsWith("tkp=")) {
    Kp_turn = command.substring(4).toFloat();
    Serial1.print("Turn Kp set: "); Serial1.println(Kp_turn);
  }
  else if (command.startsWith("tmax=")) {
    maxTurnSpeed = command.substring(5).toInt();
    Serial1.print("Turn Max Speed set: "); Serial1.println(maxTurnSpeed);
  }
  else if (command.startsWith("tmin=")) {
    minTurnSpeed = command.substring(5).toInt();
    Serial1.print("Turn Min Speed set: "); Serial1.println(minTurnSpeed);
  }
  
  // --- General ---
  else if (command.startsWith("speed=")) {
    baseSpeed = command.substring(6).toInt();
    baseSpeed = constrain(baseSpeed, 0, 100);
    Serial1.print("BaseSpeed set: "); Serial1.println(baseSpeed);
  }
  else if (command.startsWith("pivot=")) {
    pivotSpeed = command.substring(6).toInt();
    pivotSpeed = constrain(pivotSpeed, 0, 100);
    Serial1.print("PivotSpeed/Approach set: "); Serial1.println(pivotSpeed);
  }
  else if (command.startsWith("thresh=")) { // Side wall
    WALL_DISTANCE_CM = command.substring(7).toFloat();
    Serial1.print("Side Wall Dist (cm) set: "); Serial1.println(WALL_DISTANCE_CM);
  }
  else if (command.startsWith("front=")) { // Front wall detect
    FRONT_WALL_CM = command.substring(6).toFloat();
    Serial1.print("Front Wall Detect (cm) set: "); Serial1.println(FRONT_WALL_CM);
  }
  else if (command.startsWith("approach=")) { // Front wall stop
    APPROACH_STOP_DISTANCE_CM = command.substring(9).toFloat();
    Serial1.print("Approach Stop Dist (cm) set: "); Serial1.println(APPROACH_STOP_DISTANCE_CM);
  }
  
  else if (command.equals("help")) {
    Serial1.println("--- Corridor/Gyro-Turner Help ---");
    Serial1.println("start/stop/cal");
    Serial1.println("speed=X: Base speed (0-100)");
    Serial1.println("pivot=X: Pivot/Approach speed (0-100)");
    Serial1.println("thresh=X.X: Side wall pivot dist (cm)");
    Serial1.println("front=X.X: Front wall detect dist (cm)");
    Serial1.println("approach=X.X: Front wall stop dist (cm)");
    Serial1.println("--- Wall PID (kp, ki, kd) ---");
    Serial1.println("--- Turn PID (tkp, tmax, tmin) ---");
  }
  else {
    Serial.print("Unknown command: "); Serial1.println(command);
  }
}

