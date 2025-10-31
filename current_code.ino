#include <Wire.h>
#include "xmotionV3.h"
#include <MPU6050.h>
#include <VL53L0X.h>  // Pololu library

// ------------------- Hardware setup -------------------
MPU6050 mpu;
VL53L0X frontSensor;  // Pololu library

// --- IR Sensor Pins ---
#define SENSOR_RIGHT A1
#define SENSOR_LEFT  A2
#define start_pin A0

bool robot_ready_state = false;
int current_start_pin_state = 0;

// ------------------- Simplified Maze Mapping -------------------
#define MAZE_WIDTH 10
#define MAZE_HEIGHT 9

uint8_t maze[MAZE_WIDTH]; // Simplified - only track dead ends per column
uint8_t visitedCells = 0; // Simple visited counter

int robotX = 0, robotY = 0;
uint8_t robotDir = 1; // 0=N, 1=E, 2=S, 3=W

// ------------------- Global Variables -------------------
unsigned long prevTime = 0;
float dt;

// --- Gyro Variables ---
int16_t gx, gy, gz;
float gyroZoffset = 0;
float yaw = 0.0;
float GYRO_SCALE = 131.0;

// --- State Machine ---
enum RobotState { 
  STOPPED, 
  DRIVING,
  APPROACHING,
  TURNING,
  CLEARING
};
RobotState currentState = STOPPED;
unsigned long stateTimer = 0;

// ------------------- Tuning Parameters -------------------

// --- IR Sensor Calibration ---
const float VOLTAGE_TOO_CLOSE = 2.5f; 
const float VOLTAGE_TOO_FAR   = 0.25f;
const float MIN_CM            = 10.0f;
const float MAX_CM            = 80.0f;
const float IR_CAL_FACTOR     = 27.86;
const float IR_CAL_EXPONENT   = -1.15;

// --- Distances ---
float WALL_DISTANCE_CM = 14.0;
float FRONT_WALL_CM = 12.0;
float APPROACH_STOP_CM = 10.0;

// --- Speeds ---
int baseSpeed = 40;
int pivotSpeed = 22;
int MIN_APPROACH_SPEED = 15;
int MIN_TURN_SPEED = 15;
int MAX_TURN_SPEED = 20;

// --- PID (Wall Following) ---
float Kp = 0.25;
float Ki = 0.2;
float Kd = 0.01;
float integral = 0;
float previous_error = 0.0;
float INTEGRAL_CLAMP = 25.0;
float WALL_FOLLOW_BIAS = 0.4;

// --- P-Control (Turning) ---
float KP_TURN = 0.5;
float TURN_TOLERANCE = 5.0;

// --- Gyro Calibration ---
int GYRO_CAL_SAMPLES = 200;
int GYRO_CAL_DELAY = 5;

// --- State Machine Delays & Timeouts ---
long STOP_RESTART_DELAY = 2000;
long POST_APPROACH_STOP_DELAY = 100;
long CLEAR_CORNER_DURATION = 300;
long TURN_TIMEOUT_MS = 2000;
long POST_TURN_DELAY = 100;

// --- Mapping ---
int DEAD_END_VISIT_COUNT = 3;

bool vl53l0x_available = false;

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);
  
  // Initialize pins
  pinMode(start_pin, INPUT);
  xmotion.MotorControl(0, 0);
  
  // Start pin check
  int initial_state = digitalRead(start_pin);
  if (initial_state == 1) {
    Serial.println("ERROR: Power cycle required");
    while(true) { delay(1000); }
  } else {
    robot_ready_state = true;
    Serial.println("Ready - waiting for start");
  }
  
  Wire.begin(); 
  
  // Initialize VL53L0X sensor with Pololu library
  Serial.println("Initializing VL53L0X (Pololu)...");
  
  // Set I2C bus to 400kHz for faster readings (optional)
  Wire.setClock(20000);
  
  // Initialize the sensor
  if (frontSensor.init()) {
    vl53l0x_available = true;
    
    // Configure sensor for better performance
    frontSensor.setTimeout(0);
    
    // Start continuous back-to-back mode (fastest)
    frontSensor.startContinuous();

    digitalWrite(9, HIGH);
    
    Serial.println("VL53L0X (Pololu) initialized successfully");
  } else {
    vl53l0x_available = false;
    Serial.println("VL53L0X (Pololu) failed - using IR sensors");
  }
  
  // Initialize MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    calibrateGyro();
    Serial.println("MPU6050 initialized");
  } else {
    Serial.println("MPU6050 connection failed");
  }
  
  // Initialize maze tracking
  for(int i = 0; i < MAZE_WIDTH; i++) {
    maze[i] = 0;
  }
  maze[0] = 1; // Mark entrance as visited
}

// ------------------- Main Loop -------------------
void loop() {
  current_start_pin_state = 0;
  if (robot_ready_state) {
    // Wait for start command
    while (current_start_pin_state == 0) {
      current_start_pin_state = digitalRead(start_pin);
    }
    
    currentState = DRIVING;
    stateTimer = millis();
    
    // Main operation loop
    while (current_start_pin_state == 1 && robot_ready_state) {
      current_start_pin_state = digitalRead(start_pin);
      
      unsigned long currentTime = millis();
      dt = (currentTime - prevTime) / 1000.0;
      prevTime = currentTime;

      updateSimpleMapping();

      switch(currentState) {
        case STOPPED:
          stopMotors();
          if (millis() - stateTimer > STOP_RESTART_DELAY) {
            currentState = DRIVING;
            stateTimer = millis();
          }
          break;
          
        case DRIVING:
          handleDriving();
          break;

        case APPROACHING:
          handleApproaching();
          break;

        case TURNING:
          handleTurning();
          break;
          
        case CLEARING:
          handleClearing();
          break;
      }
      
      delay(10);
    }
    
    // Stop command received
    emergencyStop();
  }
}

// ------------------- Simplified Mapping -------------------
void updateSimpleMapping() {
  // Mark current cell as visited
  if (robotX >= 0 && robotX < MAZE_WIDTH && robotY >= 0 && robotY < MAZE_HEIGHT) {
    maze[robotX] |= (1 << robotY);
  }
  visitedCells++;
}

bool isDeadEnd(int x, int y) {
  static uint8_t visitCount[MAZE_WIDTH] = {0};
  if (x >= 0 && x < MAZE_WIDTH) {
    visitCount[x]++;
    return (visitCount[x] > DEAD_END_VISIT_COUNT);
  }
  return false;
}

// ------------------- State Functions -------------------
void handleDriving() {
  float leftCm = readIRSensor(SENSOR_LEFT);
  float rightCm = readIRSensor(SENSOR_RIGHT);
  float frontCm = getFrontDistance();

  // Front wall detection
  if (frontCm < FRONT_WALL_CM) {
    currentState = APPROACHING;
    stateTimer = millis();
    return;
  }
  
  // Simple wall following
  followWalls(leftCm, rightCm);
}

void handleApproaching() {
  float frontCm = getFrontDistance();
  float leftCm = readIRSensor(SENSOR_LEFT);
  float rightCm = readIRSensor(SENSOR_RIGHT);

  if (frontCm <= APPROACH_STOP_CM) {
    stopMotors();
    delay(POST_APPROACH_STOP_DELAY);
    currentState = TURNING;
    stateTimer = millis();
  } else {
    // Slow approach
    int speed = map((int)(frontCm * 10), (int)(APPROACH_STOP_CM * 10), (int)(FRONT_WALL_CM * 10), MIN_APPROACH_SPEED, baseSpeed);
    speed = constrain(speed, MIN_APPROACH_SPEED, baseSpeed);
    xmotion.MotorControl(speed, speed);
  }
}

void handleTurning() {
  float leftCm = readIRSensor(SENSOR_LEFT);
  float rightCm = readIRSensor(SENSOR_RIGHT);
  float frontCm = getFrontDistance();

  float turnAngle = 0;
  
  // Check for dead end
  if (frontCm < FRONT_WALL_CM && leftCm < WALL_DISTANCE_CM && rightCm < WALL_DISTANCE_CM) {
    if (isDeadEnd(robotX, robotY)) {
      turnAngle = 180;
    } else {
      turnAngle = (leftCm > rightCm) ? -90 : 90;
    }
  } else {
    // Normal turn decision
    if (robotX <= 1 && robotY == 0 && robotDir == 3) { // Avoid entrance
      turnAngle = (leftCm > WALL_DISTANCE_CM) ? -90 : 90;
    } else {
      turnAngle = (leftCm > rightCm) ? -90 : 90;
    }
  }

  turn(turnAngle);
  yaw = 0;
  integral = 0;
  previous_error = 0;
  updateDirection(turnAngle);
  currentState = CLEARING;
  stateTimer = millis();
}

void handleClearing() {
  if (millis() - stateTimer > CLEAR_CORNER_DURATION) {
    currentState = DRIVING;
  } else {
    xmotion.MotorControl(baseSpeed, baseSpeed);
  }
}

// ------------------- Core Navigation -------------------
void followWalls(float leftCm, float rightCm) {
  float error = leftCm - rightCm;
  
  // Add small bias to compensate for right drift
  error += WALL_FOLLOW_BIAS;

  float p_term = Kp * error;
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
  float i_term = Ki * integral;

  float derivative = 0;
  if (dt > 0) {
    derivative = (error - previous_error) / dt;
  }
  previous_error = error;
  float d_term = Kd * derivative;

  float correction = p_term + i_term + d_term;

  int leftSpeed = constrain(baseSpeed + correction, 0, 100);
  int rightSpeed = constrain(baseSpeed - correction, 0, 100);

  xmotion.MotorControl(leftSpeed, rightSpeed);
}

void turn(float angle) {
  yaw = 0;
  unsigned long turnStartTime = millis();
  bool rightTurn = (angle > 0);
  float targetAngle = angle;

  Serial.print("Turning ");
  Serial.print(angle);
  Serial.println(" degrees");
  
  while (digitalRead(start_pin) == 1) {
    float currentYaw = getCurrentYaw();
    float error = targetAngle - currentYaw;
    
    // Check if we've reached the target angle
    if (abs(error) <= TURN_TOLERANCE) {
      break;
    }

    // Calculate turn speed based on error
    int turnSpeed = constrain((int)(abs(error) * KP_TURN), MIN_TURN_SPEED, MAX_TURN_SPEED);
    
    // Slow down as we approach target
    if (abs(error) < 20) {
      turnSpeed = MIN_TURN_SPEED;
    }
    
    if (!rightTurn) {
      xmotion.Right0(turnSpeed, 0);
    } else {
      xmotion.Left0(turnSpeed, 0);
    }

    // Safety timeout
    if (millis() - turnStartTime > TURN_TIMEOUT_MS) {
      Serial.println("Turn timeout");
      break;
    }
    
    delay(10);
  }
  
  stopMotors();
  delay(POST_TURN_DELAY);
  
  Serial.print("Turn completed. Final yaw: ");
  Serial.println(getCurrentYaw());
}

void updateDirection(float turnAngle) {
  // Update robot direction based on turn angle
  if (turnAngle == 90) {
    // Right turn
    robotDir = (robotDir + 1) % 4;
  } else if (turnAngle == -90) {
    // Left turn
    robotDir = (robotDir + 3) % 4;
  } else if (abs(turnAngle) == 180) {
    // 180 degree turn
    robotDir = (robotDir + 2) % 4;
  }
  
  // Update grid position based on movement after turn
  switch(robotDir) {
    case 0: // North
      robotY = max(0, robotY - 1);
      break;
    case 1: // East
      robotX = min(MAZE_WIDTH - 1, robotX + 1);
      break;
    case 2: // South
      robotY = min(MAZE_HEIGHT - 1, robotY + 1);
      break;
    case 3: // West
      robotX = max(0, robotX - 1);
      break;
  }
  
  // Mark new cell as visited
  if (robotX >= 0 && robotX < MAZE_WIDTH && robotY >= 0 && robotY < MAZE_HEIGHT) {
    maze[robotX] |= (1 << robotY);
  }
  
  Serial.print("New position: (");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.print(") Direction: ");
  switch(robotDir) {
    case 0: Serial.println("NORTH"); break;
    case 1: Serial.println("EAST"); break;
    case 2: Serial.println("SOUTH"); break;
    case 3: Serial.println("WEST"); break;
  }
}

// ------------------- Sensor Functions -------------------
float readIRSensor(int pin) {
  int raw = analogRead(pin);
  float voltage = raw * (5.0 / 1023.0);
  
  if (voltage > VOLTAGE_TOO_CLOSE) return MIN_CM;
  if (voltage < VOLTAGE_TOO_FAR) return MAX_CM;
  
  float distance = IR_CAL_FACTOR * pow(voltage, IR_CAL_EXPONENT); 
  return constrain(distance, MIN_CM, MAX_CM);
}

float getFrontDistance() {
  if (vl53l0x_available) {
    // Pololu library - read distance in mm and convert to cm
    uint16_t distance_mm = frontSensor.readRangeContinuousMillimeters();
    
    // Check for timeout or invalid reading
    if (frontSensor.timeoutOccurred()) {
      Serial.println("VL53L0X timeout");
      digitalWrite(8,HIGH);
      // Fall through to IR sensors
    } else if (distance_mm < 12000) { // Valid range check (12m max)
      float distance_cm = (distance_mm -40)/ 10.0;
      return distance_cm;
    }
  }
  
  // Fallback to IR sensors if VL53L0X is not available or measurement failed
  float left = readIRSensor(SENSOR_LEFT);
  float right = readIRSensor(SENSOR_RIGHT);
  return min(left, right);
}

// ------------------- Gyro Functions -------------------
void calibrateGyro() {
  xmotion.MotorControl(0, 0);
  Serial.println("Calibrating gyro...");
  
  long sum = 0;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(GYRO_CAL_DELAY);
  }
  gyroZoffset = sum / (float)GYRO_CAL_SAMPLES;
  
  Serial.print("Gyro offset: ");
  Serial.println(gyroZoffset);
  
  yaw = 0;
  integral = 0;
  previous_error = 0;
}

float getCurrentYaw() {
  static unsigned long lastGyroTime = 0;
  unsigned long currentTime = millis();
  
  if (lastGyroTime == 0) {
    lastGyroTime = currentTime;
    return yaw;
  }
  
  float delta_t = (currentTime - lastGyroTime) / 1000.0;
  lastGyroTime = currentTime;
  
  mpu.getRotation(&gx, &gy, &gz);
  float corrected = gz - gyroZoffset;
  float deltaYaw = (corrected * delta_t / GYRO_SCALE);
  yaw += deltaYaw;
  
  return yaw;
}

// ------------------- Utility Functions -------------------
void stopMotors() {
  xmotion.StopMotors(100);
  integral = 0;
  previous_error = 0;
}

void emergencyStop() {
  xmotion.MotorControl(0, 0);
}