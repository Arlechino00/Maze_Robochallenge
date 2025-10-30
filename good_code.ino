#include <Wire.h>
#include "xmotionV3.h"
#include <MPU6050.h>
#include <math.h>
#include <Adafruit_VL53L0X.h>

// ------------------- Hardware setup -------------------
MPU6050 mpu;
Adafruit_VL53L0X frontSensor = Adafruit_VL53L0X();

// --- IR Sensor Pins ---
#define SENSOR_RIGHT A1
#define SENSOR_LEFT  A2
#define start_pin A0

bool robot_ready_state = false;
int current_start_pin_state = 0;

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
  DRIVING_CORRIDOR,
  APPROACHING_WALL,
  TURN_AT_WALL,
  CLEAR_CORNER
};
RobotState currentState = STOPPED;  // Start stopped, then begin after setup
unsigned long stateTimer = 0;
long clearCornerDuration = 300;

// --- Tuning: IR Sensors ---
float WALL_DISTANCE_CM = 14.0;
float FRONT_WALL_CM = 13.0;
float APPROACH_STOP_DISTANCE_CM = 10.0;

// --- Tuning: Motion ---
int baseSpeed = 40;
int pivotSpeed = 22;

// --- Tuning: PID (Wall Following) ---
float Kp = 0.25;
float Ki = 0.2;
float Kd = 0.01;
float integral = 0;
float integral_clamp = 25.0;
float previous_wall_error = 0.0;

// --- Tuning: P-Control (Turning) ---
float Kp_turn = 0.5;
int maxTurnSpeed = 20;
int minTurnSpeed = 15;
float turn_tolerance = 5.0;

// VL53L0X sensor status
bool vl53l0x_available = false;

// ------------------- Setup -------------------
void setup() {
  delay(3000);
  int initial_start_pin_state = digitalRead(start_pin);
  if (initial_start_pin_state == 1) {
  while (true) {
  }

} else if (initial_start_pin_state == 0) {
  robot_ready_state = true;
}
  Serial.begin(9600);
  Serial.println("Starting Maze Robot...");
  
  Wire.begin(); 
  
  // Initialize VL53L0X front sensor with error handling
  Serial.println("Initializing VL53L0X...");
  if (!frontSensor.begin()) {
    Serial.println("Failed to initialize VL53L0X sensor! Continuing without front sensor.");
    vl53l0x_available = false;
  } else {
    Serial.println("VL53L0X front sensor initialized!");
    vl53l0x_available = true;
  }
  
  // Initialize MPU6050 with error handling
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed! Check wiring.");
    while(1) {
      delay(1000);
      Serial.println("MPU6050 not found - please check connections");
    }
  }
  Serial.println("MPU6050 found!");
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  // Calibrate gyro
  calibrateGyro();
  
  // Start the robot
  currentState = DRIVING_CORRIDOR;
  Serial.println("Maze Robot Ready - Starting Autonomous Mode");
  prevTime = millis();
}

// ------------------- Main loop: The State Machine -------------------
void loop() {
  while (current_start_pin_state == 0) {
  current_start_pin_state = digitalRead(start_pin);

}
if(current_start_pin_state == 1 && robot_ready_state == true) {
  while (current_start_pin_state == 1) {

    unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  switch(currentState) {
    case STOPPED:
      stopMotorsAndResetPID();
      // Auto-restart after 2 seconds if stopped
      if (millis() > 5000) { // Only after 5 seconds of operation
        currentState = DRIVING_CORRIDOR;
        Serial.println("Auto-restarting...");
      }
      break;
      
    case DRIVING_CORRIDOR:
      handleCorridorFollowing();
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
  
  //delay(10);
    current_start_pin_state = digitalRead(start_pin);

  }

  if(current_start_pin_state == 0) {

    xmotion.StopMotors(0);

    while (true) {

    }

  }

}
  
}

// ------------------- Gyro calibration -------------------
void calibrateGyro() {
  Serial.println("Calibrating gyro... do not move!");
  xmotion.MotorControl(0, 0); // Ensure motors are off
  
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }
  gyroZoffset = sum / 500.0;
  
  Serial.print("Gyro Z offset: "); Serial.println(gyroZoffset);
  
  yaw = 0;
  integral = 0;
  previous_wall_error = 0;
}

// ------------------- State Functions -------------------

void stopMotorsAndResetPID() {
  xmotion.MotorControl(0, 0);
  integral = 0;
  previous_wall_error = 0;
}

void handleCorridorFollowing() {
  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));
  float frontCm = getFrontDistance();

  // Debug output every 20 cycles to reduce spam
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    Serial.print("L:"); Serial.print(leftCm, 0);
    Serial.print("cm R:"); Serial.print(rightCm, 0);
    Serial.print("cm F:"); Serial.print(frontCm, 0);
    Serial.println("cm - Running");
    lastDebug = millis();
  }

  // Priority 1: Head-On Wall Detection
  if (frontCm < FRONT_WALL_CM) {
      Serial.print("Front wall! ");
      Serial.print(frontCm);
      Serial.println("cm -> APPROACH");
      currentState = APPROACHING_WALL;
      return;
  }
  
  // Priority 2: Side Wall Pivot
  else if (leftCm < WALL_DISTANCE_CM || rightCm < WALL_DISTANCE_CM) {
    Serial.println("Side wall -> PIVOT");
    if (leftCm < rightCm) {
        xmotion.MotorControl(pivotSpeed, -pivotSpeed);
    } else {
        xmotion.MotorControl(-pivotSpeed, pivotSpeed);
    }
    integral = 0;
    previous_wall_error = 0;
    return; 
  }

  // Priority 3: PID Corridor Following
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
  float frontCm = getFrontDistance();

  if (frontCm <= APPROACH_STOP_DISTANCE_CM) {
    Serial.println("At wall -> TURN");
    xmotion.MotorControl(0, 0);
    delay(100);
    currentState = TURN_AT_WALL;
  } else {
    xmotion.MotorControl(pivotSpeed, pivotSpeed);
    Serial.print("Approaching: ");
    Serial.print(frontCm);
    Serial.println("cm");
  }
}

void handleTurnAtWall() {
  Serial.println("Deciding turn direction...");

  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));

  float turnAngle;

  if (leftCm > rightCm) {
    Serial.println("Turning LEFT -90deg");
    turnAngle = -90.0;
  } else {
    Serial.println("Turning RIGHT +90deg");
    turnAngle = 90.0;
  }
  
  TurnToAngle(turnAngle);

  Serial.println("Turn complete -> CLEAR CORNER");
  yaw = 0;
  integral = 0;
  previous_wall_error = 0;
  stateTimer = millis();
  currentState = CLEAR_CORNER;
}

void handleClearCorner() {
  if (millis() - stateTimer > clearCornerDuration) {
    Serial.println("Corner cleared -> DRIVING");
    currentState = DRIVING_CORRIDOR;
    return;
  } 
  
  // Drive straight forward
  xmotion.MotorControl(baseSpeed, baseSpeed);
}

// ------------------- Helper Functions -------------------

float getFrontDistance() {
  if (!vl53l0x_available) {
    // If VL53L0X is not available, use IR sensors as fallback
    float leftCm = RawToCm(analogRead(SENSOR_LEFT));
    float rightCm = RawToCm(analogRead(SENSOR_RIGHT));
    return min(leftCm, rightCm); // Return the closer distance
  }
  
  VL53L0X_RangingMeasurementData_t measure;
  frontSensor.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    float distance = measure.RangeMilliMeter / 10.0;
    return distance;
  } else {
    // Measurement failed, return safe distance
    return FRONT_WALL_CM + 10.0;
  }
}

float RawToCm(int rawValue) {
  const float VOLTAGE_TOO_CLOSE = 2.5f; 
  const float VOLTAGE_TOO_FAR = 0.25f;
  const float MIN_CM = 10.0f;
  const float MAX_CM = 80.0f;

  float voltage = rawValue * (5.0 / 1023.0);

  if (voltage > VOLTAGE_TOO_CLOSE) return MIN_CM;
  if (voltage < VOLTAGE_TOO_FAR) return MAX_CM;

  float distance = 27.86 * pow(voltage, -1.15); 
  if (distance > MAX_CM) distance = MAX_CM;
  if (distance < MIN_CM) distance = MIN_CM;
  
  return distance;
}

float getCurrentYawForTurn() {
  unsigned long currentTime_turn = millis();
  float turn_dt = (currentTime_turn - prevTime) / 1000.0;
  prevTime = currentTime_turn;
  
  mpu.getRotation(&gx, &gy, &gz);
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

    Serial.println("Starting gyro turn...");
    
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

        if (millis() - startTime > 2000) { 
            Serial.println("Turn timeout - forcing completion");
            break;
        }
        
        delay(10);
    }
    
    xmotion.StopMotors(100);
    delay(100);
    
    Serial.println("Gyro turn complete");
}