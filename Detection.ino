/*
  IR Wall Detection Sketch
  
  Reads from two Sharp IR sensors (like GP2Y0A21) to detect walls.
  - Right Sensor -> A1
  - Left Sensor  -> A2

  These sensors output a HIGH analog value when CLOSE
  and a LOW analog value when FAR.
*/

// --- Sensor Pins ---
#define SENSOR_RIGHT A1
#define SENSOR_LEFT  A2

// --- Tuning ---
// This is the ADC value (0-1023) we consider a "wall".
// You MUST tune this value!
// 1. Run this code and open the Serial Monitor.
// 2. Point the robot at a wall from a "too close" distance.
// 3. See what "Raw" value is printed, and set this threshold just BELOW that.
// (Higher value = closer object)
int WALL_THRESHOLD = 450; 


// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);
  
  // Analog pins are INPUT by default, so no pinMode() is needed.
  
  Serial.println("--- IR Wall Detection Test ---");
  Serial.print("Wall Threshold set to: ");
  Serial.println(WALL_THRESHOLD);
  Serial.println("\nPlace a wall in front of the sensors and check values.");
  Serial.println("----------------------------------------------------");
}

// ------------------- Main loop -------------------
void loop() {
  
  // --- Step 1: Read the raw analog values ---
  // Value is 0-1023. Higher value = closer.
  int rawLeft = analogRead(SENSOR_LEFT);
  int rawRight = analogRead(SENSOR_RIGHT);

  // --- Step 2: Check if the value is past our threshold ---
  bool wallLeft = (rawLeft > WALL_THRESHOLD);
  bool wallRight = (rawRight > WALL_THRESHOLD);

  // --- Step 3: Print all data to the Serial Monitor ---
  Serial.print("Left Cm: ");
  Serial.print(rawToCm(rawLeft));
  Serial.print("\t Wall: ");
  Serial.print(wallLeft ? "YES" : "NO"); // Prints "YES" or "NO"
  
  Serial.print("\t | \t"); // Separator
  
  Serial.print("Right Cm: ");
  Serial.print(rawToCm(rawRight));
  Serial.print("\t Wall: ");
  Serial.println(wallRight ? "YES" : "NO");

  // --- (Optional) Advanced Method: Convert to CM ---
  // This is more complex because the formula is different
  // for every sensor model (e.g., GP2Y0A21 vs GP2Y0A41).
  // The threshold method above is simpler and often better.
  
  // float cmLeft = rawToCm(rawLeft);
  // float cmRight = rawToCm(rawRight);
  // Serial.print("Left: "); Serial.print(cmLeft); Serial.print("cm");
  // Serial.print("\t | \t");
  // Serial.print("Right: "); Serial.print(cmRight); Serial.println("cm");
  
  delay(200); // Slow down the printing
}



// ------------------- (Optional) Advanced Function -------------------
// Function to convert raw ADC value to centimeters.
// !!! WARNING: This formula is ONLY for the GP2Y0A21YK0F (10-80cm) sensor.
// !!! It will be WRONG for any other model.
float rawToCm(int rawValue) {
  // Convert the 0-1023 reading to a 0-5V voltage
  float voltage = rawValue * (5.0 / 1023.0);
  
  // This formula is a "power fit" from the sensor's datasheet graph.
  // It is highly non-linear and specific to this model.
  float distance = 27.86 * pow(voltage, -1.15); 
  
  // Clamp values to the sensor's effective range (10-80cm)
  if (distance > 80) distance = 80;
  if (distance < 10) distance = 10;
  
  return distance;
}

