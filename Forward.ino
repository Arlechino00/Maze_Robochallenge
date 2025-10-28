#include <Wire.h>
#include "xmotionV3.h"
#include <MPU6050.h>

// ------------------- Hardware setup -------------------
MPU6050 mpu;

// ------------------- Variables -------------------
int16_t gx, gy, gz;
float gyroZoffset = 0;
float yaw = 0;
unsigned long prevTime = 0;

// --- Robot State ---
bool isRunning = false; // Controls if the robot should move
String serial1Buffer = ""; // Buffer for incoming Bluetooth commands

// Motion tuning
int baseSpeed = 40; // forward speed 0–100

// --- PID Tuning Constants ---
float Kp = 1.3;  // Proportional
float Ki = 0.1;  // Integral
float Kd = 0.02;  // Derivative

float dt; // delta time
float integral = 0; // Accumulator for the integral term
float integral_clamp = 25.0; // Clamps the integral to prevent runaway

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);       // USB Serial Monitor
  Serial1.begin(9600);      // HC-05 Bluetooth Module (TX, RX pins)
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  calibrateGyro();
  
  Serial.println("Maze robot ready! Begin tuning.");
  Serial1.println("Maze robot ready! Send 'help' for commands.");
  prevTime = millis(); // Initialize prevTime in setup
}

// ------------------- Gyro calibration -------------------
void calibrateGyro() {
  // Stop motors during calibration for safety
  isRunning = false; 
  xmotion.MotorControl(0, 0); // Use MotorControl to stop
  
  Serial.println("Calibrating gyro... do not move!");
  Serial1.println("Calibrating gyro... do not move!");
  
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroZoffset = sum / 500.0;
  
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZoffset);
  Serial1.print("Gyro Z offset: ");
  Serial1.println(gyroZoffset);

  // Reset PID controllers
  yaw = 0;
  integral = 0;
}

// ------------------- Main loop -------------------
void loop() {
  // Always check for new commands
  checkBluetoothCommands();

  // --- Time Calculation ---
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // --- Gyro Reading and Yaw Calculation ---
  // We do this even when stopped to keep 'yaw' current
  mpu.getRotation(&gx, &gy, &gz);
  float gzCorrected = gz - gyroZoffset;
  float angular_velocity_dps = gzCorrected / 131.0;
  
  if (isRunning) {
    // Integrate angular velocity (dps) to get angle (yaw in degrees)
    yaw += angular_velocity_dps * dt;

    // --- PID Controller Calculation ---
    float p_term = Kp * yaw;

    // I (Integral) term: Accumulates error over time
    integral += yaw * dt;
    integral = constrain(integral, -integral_clamp, integral_clamp); // Anti-windup
    float i_term = Ki * integral;

    // D (Derivative) term: Proportional to the rate of change of error
    float d_term = Kd * angular_velocity_dps;

    // Total correction is the sum of all three terms
    float correction = p_term + i_term + d_term;

    // --- Motor Speed Calculation ---
    // Calculate speed as a percentage (0-100)
    int leftSpeed = constrain(baseSpeed - correction, 0, 100);
    int rightSpeed = constrain(baseSpeed + correction, 0, 100);

    // --- Motor Control ---
    // Map the 0-100 percentage to the 0-255 speed range for MotorControl()
    int leftMapped = map(leftSpeed, 0, 100, 0, 255);
    int rightMapped = map(rightSpeed, 0, 100, 0, 255);
    
    // Use timeless MotorControl for continuous PID adjustment
    xmotion.MotorControl(leftMapped, rightMapped);

    // --- Serial Monitor for Tuning ---
    // (This will only print when running)
    // --- CHANGED: Send this data to Serial1 (iPhone) instead of Serial (USB) ---
    Serial1.print("Y: ");
    Serial1.print(yaw);
    Serial1.print("\t C: ");
    Serial1.print(correction);
    Serial1.print("\t L: ");
    Serial1.print(leftSpeed);
    Serial1.print("\t R: ");
    Serial1.println(rightSpeed);

  } else {
    // --- ROBOT IS STOPPED ---
    xmotion.MotorControl(0, 0); // Ensure motors are off
    
    // Reset integral to prevent "windup" when stopped
    integral = 0;
    // You might want to reset yaw as well, so it aims straight
    // from its new position when you type 'start'
    yaw = 0;
  }
  
  delay(10);
}

// ------------------- Bluetooth Command Functions -------------------

void checkBluetoothCommands() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    
    // Check for newline character, which marks end of command
    if (c == '\n') {
      parseCommand(serial1Buffer);
      serial1Buffer = ""; // Clear the buffer
    } 
    // Ignore carriage returns
    else if (c == '\r') {
      // Do nothing
    }
    else {
      serial1Buffer += c; // Add character to the buffer
    }
  }
}

void parseCommand(String command) {
  command.trim(); // Remove leading/trailing whitespace
  
  // --- NEW FIX: Check for and remove the "TX=" prefix ---
  if (command.startsWith("TX=")) {
    command = command.substring(3); // Get the string *after* "TX="
    command.trim(); // Trim again in case of "TX= start"
  }
  
  // Add a check to ignore empty commands (e.g., if just "TX=" was sent)
  if (command.length() == 0) {
    return;
  }
  
  Serial.print("Received command: ");
  Serial.println(command);

  if (command.equals("start")) {
    isRunning = true;
    yaw = 0; // Reset yaw on start
    integral = 0;
    Serial1.println("Starting robot.");
  } 
  else if (command.equals("stop")) {
    isRunning = false;
    Serial1.println("Stopping robot.");
  } 
  else if (command.equals("cal")) {
    calibrateGyro(); // This function already stops the robot
  }
  else if (command.startsWith("kp=")) {
    Kp = command.substring(3).toFloat();
    Serial1.print("Kp set to: ");
    Serial1.println(Kp);
  }
  else if (command.startsWith("ki=")) {
    Ki = command.substring(3).toFloat();
    Serial1.print("Ki set to: ");
    Serial1.println(Ki);
  }
  else if (command.startsWith("kd=")) {
    Kd = command.substring(3).toFloat();
    Serial1.print("Kd set to: ");
    Serial1.println(Kd);
  }
  else if (command.startsWith("speed=")) {
    baseSpeed = command.substring(6).toInt();
    baseSpeed = constrain(baseSpeed, 0, 100);
    Serial1.print("BaseSpeed set to: ");
    Serial1.println(baseSpeed);
  }
  else if (command.equals("help")) {
    Serial1.println("--- Available Commands ---");
    Serial1.println("start: Start robot");
    Serial1.println("stop: Stop robot");
    Serial1.println("cal: Recalibrate gyro");
    Serial1.println("kp=X.X (e.g., kp=1.2)");
    Serial1.println("ki=X.X (e.g., ki=0.5)");
    Serial1.println("kd=X.X (e.g., kd=0.1)");
    Serial1.println("speed=XX (e.g., speed=45)");
  }
  else {
    // --- FIX: Comment out the reply to stop the BLE flood ---
    // Serial1.print("Unknown command: ");
    // Serial1.println(command);
    
    // We still want to see it on the USB monitor
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

