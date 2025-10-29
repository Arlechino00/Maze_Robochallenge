/*
  Maze Solving Robot with Flood Fill Mapping
  
  Features:
  - Real-time maze mapping using flood fill algorithm
  - Optimized path finding to goal
  - Prevents 180-degree turns at start
  - Works within 10x10cm robot, 30cm cell maze
  - Maze: 2800mm x 2000mm (approx 9x6 cells)
*/

#include <Wire.h>
#include "xmotionV3.h"
#include <MPU6050.h>
#include <math.h>

// ------------------- Hardware setup -------------------
MPU6050 mpu;

#define SENSOR_RIGHT A1
#define SENSOR_LEFT  A2

// ------------------- Maze Mapping Constants -------------------
#define MAZE_WIDTH 10   // 2800mm / 30cm ≈ 9-10 cells
#define MAZE_HEIGHT 7   // 2000mm / 30cm ≈ 6-7 cells
#define CELL_SIZE 300   // 30cm in mm

// Direction encoding
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

// Wall encoding (bitwise)
#define WALL_NORTH 0x01
#define WALL_EAST  0x02
#define WALL_SOUTH 0x04
#define WALL_WEST  0x08

// ------------------- Maze Data Structures -------------------
struct Cell {
  uint8_t walls;      // Bitfield for walls
  uint8_t distance;   // Flood fill distance
  bool visited;       // Has robot been here?
};

Cell maze[MAZE_HEIGHT][MAZE_WIDTH];

// Robot position and orientation
int robotX = 0;
int robotY = 0;
int robotDir = NORTH; // Assume starts facing north

// Goal position (opposite corner from start)
int goalX = MAZE_WIDTH - 1;
int goalY = MAZE_HEIGHT - 1;

bool mazeComplete = false;
bool firstMove = true; // Prevent 180 turn at start

// ------------------- Existing Variables -------------------
unsigned long prevTime = 0;
float dt;

// Gyro
int16_t gx, gy, gz;
float gyroZoffset = 0;
float yaw = 0.0;
float GYRO_SCALE = 131.0;

// Bluetooth
String serial1Buffer = ""; 

// State Machine
enum RobotState { 
  STOPPED, 
  DRIVING_CORRIDOR,
  APPROACHING_WALL,
  TURN_AT_WALL,
  CLEAR_CORNER,
  PLANNING_ROUTE  // New state for path planning
};
RobotState currentState = STOPPED;
unsigned long stateTimer = 0;
long clearCornerDuration = 300;

// Sensor thresholds
float WALL_DISTANCE_CM = 15.0;
float FRONT_WALL_CM = 18.0;
float APPROACH_STOP_DISTANCE_CM = 15.0;

// Motion parameters
int baseSpeed = 40;
int pivotSpeed = 22;

// PID parameters
float Kp = 0.2;
float Ki = 0.1;
float Kd = 0.02;
float integral = 0;
float integral_clamp = 25.0;
float previous_wall_error = 0.0;

// Turn parameters
float Kp_turn = 0.5;
int maxTurnSpeed = 20;
int minTurnSpeed = 15;
float turn_tolerance = 5.0;

// Planned moves queue
#define MAX_MOVES 50
int plannedMoves[MAX_MOVES];
int moveIndex = 0;
int moveCount = 0;

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Wire.begin(); 
  
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  calibrateGyro();
  
  initializeMaze();
  
  Serial.println("Maze Solver Ready!");
  Serial1.println("Maze Solver Ready! Send 'help'.");
  prevTime = millis();
}

// ------------------- Maze Functions -------------------
void initializeMaze() {
  // Initialize all cells
  for (int y = 0; y < MAZE_HEIGHT; y++) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      maze[y][x].walls = 0;
      maze[y][x].distance = 255;
      maze[y][x].visited = false;
      
      // Add boundary walls
      if (y == 0) maze[y][x].walls |= WALL_SOUTH;
      if (y == MAZE_HEIGHT - 1) maze[y][x].walls |= WALL_NORTH;
      if (x == 0) maze[y][x].walls |= WALL_WEST;
      if (x == MAZE_WIDTH - 1) maze[y][x].walls |= WALL_EAST;
    }
  }
  
  // Initialize flood fill from goal
  floodFill();
}

void updateWalls() {
  // Read sensors to detect walls
  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));
  
  // Check front wall (both sensors see it)
  bool frontWall = (leftCm < FRONT_WALL_CM && rightCm < FRONT_WALL_CM);
  
  // Check left wall
  bool leftWall = (leftCm < WALL_DISTANCE_CM);
  
  // Check right wall
  bool rightWall = (rightCm < WALL_DISTANCE_CM);
  
  // Update maze based on robot direction
  uint8_t &currentWalls = maze[robotY][robotX].walls;
  
  if (frontWall) {
    currentWalls |= (1 << robotDir);
    updateAdjacentWall(robotX, robotY, robotDir);
  }
  
  int leftDir = (robotDir + 3) % 4;
  if (leftWall) {
    currentWalls |= (1 << leftDir);
    updateAdjacentWall(robotX, robotY, leftDir);
  }
  
  int rightDir = (robotDir + 1) % 4;
  if (rightWall) {
    currentWalls |= (1 << rightDir);
    updateAdjacentWall(robotX, robotY, rightDir);
  }
  
  maze[robotY][robotX].visited = true;
  
  // Log the detected walls
  Serial1.print("Pos["); Serial1.print(robotX); 
  Serial1.print(","); Serial1.print(robotY); Serial1.print("] ");
  Serial1.print("Walls: ");
  if (frontWall) Serial1.print("F ");
  if (leftWall) Serial1.print("L ");
  if (rightWall) Serial1.print("R ");
  Serial1.println();
}

void updateAdjacentWall(int x, int y, int dir) {
  int nx = x, ny = y;
  int oppositeDir = (dir + 2) % 4;
  
  switch(dir) {
    case NORTH: ny++; break;
    case EAST:  nx++; break;
    case SOUTH: ny--; break;
    case WEST:  nx--; break;
  }
  
  if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
    maze[ny][nx].walls |= (1 << oppositeDir);
  }
}

void floodFill() {
  // Reset distances
  for (int y = 0; y < MAZE_HEIGHT; y++) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      maze[y][x].distance = 255;
    }
  }
  
  // Start from goal
  maze[goalY][goalX].distance = 0;
  
  bool changed = true;
  while (changed) {
    changed = false;
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      for (int x = 0; x < MAZE_WIDTH; x++) {
        if (maze[y][x].distance == 255) continue;
        
        uint8_t currentDist = maze[y][x].distance;
        
        // Check all 4 directions
        for (int dir = 0; dir < 4; dir++) {
          if (maze[y][x].walls & (1 << dir)) continue; // Wall blocks
          
          int nx = x, ny = y;
          switch(dir) {
            case NORTH: ny++; break;
            case EAST:  nx++; break;
            case SOUTH: ny--; break;
            case WEST:  nx--; break;
          }
          
          if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT) continue;
          
          if (maze[ny][nx].distance > currentDist + 1) {
            maze[ny][nx].distance = currentDist + 1;
            changed = true;
          }
        }
      }
    }
  }
}

int getBestDirection() {
  uint8_t minDist = 255;
  int bestDir = -1;
  
  // Check all 4 directions
  for (int dir = 0; dir < 4; dir++) {
    // Skip if there's a wall
    if (maze[robotY][robotX].walls & (1 << dir)) continue;
    
    // Prevent 180 turn at start
    if (firstMove && dir == SOUTH) continue;
    
    int nx = robotX, ny = robotY;
    switch(dir) {
      case NORTH: ny++; break;
      case EAST:  nx++; break;
      case SOUTH: ny--; break;
      case WEST:  nx--; break;
    }
    
    if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT) continue;
    
    if (maze[ny][nx].distance < minDist) {
      minDist = maze[ny][nx].distance;
      bestDir = dir;
    }
  }
  
  return bestDir;
}

int getTurnAngle(int targetDir) {
  int diff = targetDir - robotDir;
  
  // Normalize to -180 to 180
  if (diff > 2) diff -= 4;
  if (diff < -2) diff += 4;
  
  return diff * 90; // Convert to degrees
}

// ------------------- Main Loop -------------------
void loop() {
  checkBluetoothCommands();

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
      
    case PLANNING_ROUTE:
      handleRoutePlanning();
      break;
  }
  
  delay(10);
}

// ------------------- State Handlers -------------------
void stopMotorsAndResetPID() {
  xmotion.MotorControl(0, 0);
  integral = 0;
  previous_wall_error = 0;
}

void handleCorridorFollowing() {
  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));

  // Check if reached goal
  if (robotX == goalX && robotY == goalY) {
    Serial1.println("GOAL REACHED!");
    currentState = STOPPED;
    mazeComplete = true;
    return;
  }

  // Update wall information
  updateWalls();

  // Head-on wall detected
  if (leftCm < FRONT_WALL_CM && rightCm < FRONT_WALL_CM) {
    Serial1.println("Front wall! Planning turn...");
    currentState = APPROACHING_WALL;
    return;
  }
  
  // Side wall pivot (emergency correction)
  if (leftCm < WALL_DISTANCE_CM || rightCm < WALL_DISTANCE_CM) {
    if (leftCm < rightCm) {
      xmotion.MotorControl(pivotSpeed, -pivotSpeed);
    } else {
      xmotion.MotorControl(-pivotSpeed, pivotSpeed);
    }
    integral = 0;
    previous_wall_error = 0;
    return;
  }

  // PID corridor following
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
  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));
  float avgDistance = (leftCm + rightCm) / 2.0;

  if (avgDistance <= APPROACH_STOP_DISTANCE_CM) {
    Serial1.println("At wall. Switching to route planning.");
    xmotion.MotorControl(0, 0);
    delay(100);
    currentState = PLANNING_ROUTE;
  } else {
    xmotion.MotorControl(pivotSpeed, pivotSpeed);
  }
}

void handleRoutePlanning() {
  // Update walls one more time at intersection
  updateWalls();
  
  // Recalculate flood fill with new wall info
  floodFill();
  
  // Get best direction
  int bestDir = getBestDirection();
  
  if (bestDir == -1) {
    Serial1.println("ERROR: No valid path found!");
    currentState = STOPPED;
    return;
  }
  
  // Calculate turn needed
  int turnAngle = getTurnAngle(bestDir);
  
  Serial1.print("Best direction: "); Serial1.print(bestDir);
  Serial1.print(" Turn: "); Serial1.println(turnAngle);
  
  if (turnAngle != 0) {
    TurnToAngle(turnAngle);
    robotDir = bestDir; // Update robot direction
  }
  
  firstMove = false; // Allow all moves after first
  
  // Update position (we're moving forward into next cell)
  switch(robotDir) {
    case NORTH: robotY++; break;
    case EAST:  robotX++; break;
    case SOUTH: robotY--; break;
    case WEST:  robotX--; break;
  }
  
  Serial1.print("New position: ["); 
  Serial1.print(robotX); Serial1.print(","); 
  Serial1.print(robotY); Serial1.println("]");
  
  // Reset PID and continue
  yaw = 0;
  integral = 0;
  previous_wall_error = 0;
  stateTimer = millis();
  currentState = CLEAR_CORNER;
}

void handleTurnAtWall() {
  // This shouldn't be called in new logic, but keep for safety
  handleRoutePlanning();
}

void handleClearCorner() {
  if (millis() - stateTimer > clearCornerDuration) {
    Serial1.println("Corner cleared. Resuming.");
    currentState = DRIVING_CORRIDOR;
    return;
  }
  
  // Drive straight
  float leftCm = RawToCm(analogRead(SENSOR_LEFT));
  float rightCm = RawToCm(analogRead(SENSOR_RIGHT));
  
  float wall_error = leftCm - rightCm;
  float correction = Kp * wall_error;

  int leftSpeed = constrain(baseSpeed + correction, 0, 100);
  int rightSpeed = constrain(baseSpeed - correction, 0, 100);

  xmotion.MotorControl(leftSpeed, rightSpeed);
}

// ------------------- Helper Functions (Existing) -------------------
float RawToCm(int rawValue) {
  float voltage = rawValue * (5.0 / 1023.0);
  if (voltage < 0.25) return 80.0;
  float distance = 27.86 * pow(voltage, -1.15); 
  if (distance > 80) distance = 80;
  if (distance < 10) distance = 10;
  return distance;
}

void calibrateGyro() {
  currentState = STOPPED; 
  xmotion.MotorControl(0, 0);
  
  Serial.println("Calibrating gyro...");
  Serial1.println("Calibrating gyro...");
  
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }
  gyroZoffset = sum / 500.0;
  
  Serial1.print("Gyro Z offset: "); Serial1.println(gyroZoffset);
  yaw = 0;
  integral = 0;
  previous_wall_error = 0;
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

  Serial1.println("--- Gyro Turn ---");
  
  while (true) {
    checkBluetoothCommands();
    if (currentState == STOPPED) {
      xmotion.StopMotors(100);
      return;
    }
    
    float currentYaw = getCurrentYawForTurn();
    float error = abs(targetDelta - currentYaw);

    if (error <= turn_tolerance) break;

    int motorSpeed = (int)(error * Kp_turn);
    motorSpeed = constrain(motorSpeed, minTurnSpeed, maxTurnSpeed);

    if (turningRight) {
      xmotion.Right0(motorSpeed, 0);
    } else {
      xmotion.Left0(motorSpeed, 0);
    }

    if (millis() - startTime > 5000) { 
      Serial1.println("Turn timeout!");
      break;
    }
    
    delay(10);
  }
  
  xmotion.StopMotors(100);
  delay(100);
  
  Serial1.println("Turn complete.");
}

// ------------------- Bluetooth Commands -------------------
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

  int eqIndex = command.indexOf('=');

  if (eqIndex != -1) {
    String key = command.substring(0, eqIndex);
    String val = command.substring(eqIndex + 1);

    if (key.equals("kp")) { Kp = val.toFloat(); Serial1.print("Kp: "); Serial1.println(Kp, 3); }
    else if (key.equals("ki")) { Ki = val.toFloat(); Serial1.print("Ki: "); Serial1.println(Ki, 3); }
    else if (key.equals("kd")) { Kd = val.toFloat(); Serial1.print("Kd: "); Serial1.println(Kd, 3); }
    else if (key.equals("tkp")) { Kp_turn = val.toFloat(); Serial1.print("Turn Kp: "); Serial1.println(Kp_turn, 3); }
    else if (key.equals("tmax")) { maxTurnSpeed = val.toInt(); Serial1.print("Max Turn: "); Serial1.println(maxTurnSpeed); }
    else if (key.equals("tmin")) { minTurnSpeed = val.toInt(); Serial1.print("Min Turn: "); Serial1.println(minTurnSpeed); }
    else if (key.equals("speed")) { baseSpeed = constrain(val.toInt(), 0, 100); Serial1.print("Speed: "); Serial1.println(baseSpeed); }
    else if (key.equals("pivot")) { pivotSpeed = constrain(val.toInt(), 0, 100); Serial1.print("Pivot: "); Serial1.println(pivotSpeed); }
    else if (key.equals("thresh")) { WALL_DISTANCE_CM = val.toFloat(); Serial1.print("Side Wall: "); Serial1.println(WALL_DISTANCE_CM, 1); }
    else if (key.equals("front")) { FRONT_WALL_CM = val.toFloat(); Serial1.print("Front Wall: "); Serial1.println(FRONT_WALL_CM, 1); }
    else if (key.equals("approach")) { APPROACH_STOP_DISTANCE_CM = val.toFloat(); Serial1.print("Approach: "); Serial1.println(APPROACH_STOP_DISTANCE_CM, 1); }
    else { Serial1.print("Unknown: "); Serial1.println(key); }
  
  } else {
    if (command.equals("start")) {
      integral = 0;
      previous_wall_error = 0;
      yaw = 0;
      robotX = 0;
      robotY = 0;
      robotDir = NORTH;
      firstMove = true;
      mazeComplete = false;
      initializeMaze();
      currentState = DRIVING_CORRIDOR; 
      Serial1.println("Starting maze!");
    } 
    else if (command.equals("stop")) { currentState = STOPPED; Serial1.println("Stopped."); }
    else if (command.equals("cal")) { calibrateGyro(); }
    else if (command.equals("map")) { printMaze(); }
    else if (command.equals("help")) {
      Serial1.println("=== Maze Solver ===");
      Serial1.println("start/stop/cal/map");
      Serial1.println("Use key=val to set params");
      Serial1.println("speed, pivot, kp, ki, kd");
      Serial1.println("tkp, tmax, tmin");
      Serial1.println("thresh, front, approach");
    }
    else if (command.equals("kp")) { Serial1.print("kp="); Serial1.println(Kp, 3); }
    else if (command.equals("ki")) { Serial1.print("ki="); Serial1.println(Ki, 3); }
    else if (command.equals("kd")) { Serial1.print("kd="); Serial1.println(Kd, 3); }
    else if (command.equals("speed")) { Serial1.print("speed="); Serial1.println(baseSpeed); }
    else { Serial1.print("Unknown: "); Serial1.println(command); }
  }
}

void printMaze() {
  Serial1.println("=== Current Maze ===");
  for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      if (x == robotX && y == robotY) {
        Serial1.print("R");
      } else if (maze[y][x].visited) {
        Serial1.print(".");
      } else {
        Serial1.print(" ");
      }
    }
    Serial1.println();
  }
  Serial1.println("==================");
}