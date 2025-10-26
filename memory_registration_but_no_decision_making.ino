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
#define CALIBRATION_OFFSET 9 

// ------------------- Path Memory (NEW) -------------------
#define MAZE_SIZE 10
byte mazeGrid[MAZE_SIZE][MAZE_SIZE]; // 10x10 grid, 'byte' uses 1 byte per cell
int currentX = 5; // Start in the middle (5,5)
int currentY = 5;
int heading = 0; // 0=North, 1=East, 2=South, 3=West
bool justTurned = false; // Flag to track when we enter a new cell

// Cell States
#define VISITED 1
// -----------------------------------------------------

// ------------------- Variables -------------------
int16_t gx, gy, gz;
float gyroZoffset = 0;
float yaw = 0;
unsigned long prevTime = 0;

// Motion tuning
int baseSpeed = 30;      // forward speed 0–100
int wallThreshold = 320; // IR threshold for wall

float dt;

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  calibrateGyro();

  // --- Initialize Maze (NEW) ---
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      mazeGrid[i][j] = 0; // 0 = unvisited
    }
  }
  // Mark the starting cell as visited
  mazeGrid[currentX][currentY] = VISITED;
  // ---------------------------
  
  Serial.println("Maze robot ready!");
  Serial.print("Starting at grid: (");
  Serial.print(currentX);
  Serial.print(", ");
  Serial.print(currentY);
  Serial.println(")");
}

// ------------------- Gyro calibration -------------------
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
    xmotion.StopMotors(105);
    if (abs(rightDist - leftDist) <= 20){
      Serial.println("Turning RIGHT*2");
      xmotion.Right0(40, 140); // right turn
      
      // --- Update Heading (NEW) ---
      heading = (heading + 1) % 4; // 0->1, 1->2, 2->3, 3->0
      justTurned = true; // Set flag to update coordinates on next forward move
      Serial.print("New Heading: "); Serial.println(heading);
      // ---------------------------
    }
    else if (rightDist <= leftDist) {
      Serial.println("Turning RIGHT");
      xmotion.Right0(20, 140); // right turn
      
      // --- Update Heading (NEW) ---
      heading = (heading + 1) % 4; // 0->1, 1->2, 2->3, 3->0
      justTurned = true; // Set flag to update coordinates on next forward move
      Serial.print("New Heading: "); Serial.println(heading);
      // ---------------------------
    } 
    else if (rightDist >= leftDist) {
      Serial.println("Turning LEFT");
      xmotion.Left0(20, 140);  // left turn
      
      // --- Update Heading (NEW) ---
      heading = (heading - 1 + 4) % 4; // 0->3, 3->2, 2->1, 1->0
      justTurned = true; // Set flag to update coordinates on next forward move
      Serial.print("New Heading: "); Serial.println(heading);
      // ---------------------------
    } 
    else if (abs(rightDist - leftDist) <= 10){
      Serial.println("Turning RIGHT*2");
      xmotion.Right0(40, 200); // right turn
      
      // --- Update Heading (NEW) ---
      heading = (heading + 1) % 4; // 0->1, 1->2, 2->3, 3->0
      justTurned = true; // Set flag to update coordinates on next forward move
      Serial.print("New Heading: "); Serial.println(heading);
      // ---------------------------
    }
    
  }
  else {
    // --- Check if entering a new cell (NEW) ---
    if (justTurned) {
      // Update coordinates based on new heading
      if (heading == 0) { // North
        currentY--;
      } else if (heading == 1) { // East
        currentX++;
      } else if (heading == 2) { // South
        currentY++;
      } else if (heading == 3) { // West
        currentX--;
      }

      // Constrain to grid boundaries to prevent writing outside the array
      currentX = constrain(currentX, 0, MAZE_SIZE - 1);
      currentY = constrain(currentY, 0, MAZE_SIZE - 1);

      // Mark new cell as visited
      mazeGrid[currentX][currentY] = VISITED;
      justTurned = false; // We are now *inside* the new cell

      Serial.print("--- Entered new cell: (");
      Serial.print(currentX);
      Serial.print(", ");
      Serial.print(currentY);
      Serial.println(") ---");
    }
    // ------------------------------------------

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