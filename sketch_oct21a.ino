// PROPORTIONAL OBSTACLE AVOIDANCE SCRIPT
//
// This robot uses two angled sensors to smoothly steer away
// from the closest obstacle using the ArcTurn function.

#include "xmotionV3.h"

// The library creates the 'xmotion' object for you.

// --- SENSOR PINS ---
#define LEFT_SENSOR_PIN   A2
#define RIGHT_SENSOR_PIN  A1

// --- TUNING PARAMETERS ---

// Proportional Gain: A multiplier to control steering sensitivity.
#define P_GAIN 2

// If an object is closer than this (cm), perform a sharp pivot turn.
#define MIN_DISTANCE 20

// The robot's default speed when the path is clear.
#define BASE_SPEED 50

// Speed for sharp pivot turns when an escape maneuver is needed.
#define TURN_SPEED 30
#define TURN_DURATION 15

// * NEW *: A small value to slow down the faster (left) motor.
// Tune this value until the robot drives perfectly straight.
#define CALIBRATION_OFFSET 8

void setup() {
  // A 3-second delay to give you time to position the robot.
  delay(3000);
}

void loop() {
  // Read both sensors and calculate their distance in cm.
  float distanceLeft = 18770 * pow(analogRead(LEFT_SENSOR_PIN), -1.253);
  float distanceRight = 18770 * pow(analogRead(RIGHT_SENSOR_PIN), -1.253);

  // --- SAFETY OVERRIDE ---
  if (distanceLeft < MIN_DISTANCE || distanceRight < MIN_DISTANCE) {
    if (distanceLeft < distanceRight) {
      xmotion.Right0(TURN_SPEED, TURN_DURATION);
    } else {
      xmotion.Left0(TURN_SPEED, TURN_DURATION);
    }
  }
  // --- PROPORTIONAL STEERING LOGIC ---
  else {
    float error = constrain(distanceLeft, 0, 60) - constrain(distanceRight, 0, 60);
    int steeringAdjustment = (int)(error * P_GAIN);

    // Calculate the final speed for each motor.
    // * MODIFIED *: Apply the calibration offset to the left motor's speed.
    int leftSpeed = BASE_SPEED - steeringAdjustment - CALIBRATION_OFFSET;
    int rightSpeed = BASE_SPEED + steeringAdjustment;

    // Constrain motor speeds to the valid 0-100 range for the library.
    leftSpeed = constrain(leftSpeed, 0, 100);
    rightSpeed = constrain(rightSpeed, 0, 100);

    // Send the calculated speeds to the motors.
    xmotion.ArcTurn(leftSpeed, rightSpeed, 10);
  }
}