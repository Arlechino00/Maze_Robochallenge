/* Memory-lean maze robot sketch — fixed: added stopMotorsAndResetPID() */

#include <Wire.h>
#include "xmotionV3.h"
#include <MPU6050.h>
#include <math.h>
#include <stdlib.h>

using namespace std; // user preference

MPU6050 mpu;

// Pins
#define SENSOR_RIGHT A1
#define SENSOR_LEFT  A2

// Timing
unsigned long prevTime = 0;
float dt = 0.001f;

// Gyro
int16_t gx, gy, gz;
float gyroZoffset = 0.0f;
float yaw = 0.0f;
const float GYRO_SCALE = 131.0f;

// Bluetooth command buffer (fixed)
char cmdBuf[64];
uint8_t cmdIdx = 0;

// State machine
enum RobotState { STOPPED, DRIVING_CORRIDOR, APPROACHING_WALL, TURN_AT_WALL, CLEAR_CORNER };
RobotState currentState = STOPPED;
unsigned long stateTimer = 0;
long clearCornerDuration = 300;

// IR tuning
float WALL_DISTANCE_CM = 15.0f;
float HARD_PIVOT_CM = 10.0f;
float FRONT_WALL_CM = 20.0f;
float APPROACH_STOP_DISTANCE_CM = 15.0f;

// Motion
int baseSpeed = 40;
int pivotSpeed = 30;

// PID
float Kp = 0.3f, Ki = 0.0f, Kd = 0.0f;
float integral = 0.0f;
const float integral_clamp = 25.0f;
float previous_wall_error = 0.0f;

// Avoidance + deadzone
float K_avoid = 1.2f;
float wall_deadzone = 0.8f;

// Turn tuning
float Kp_turn = 0.5f;
int maxTurnSpeed = 20;
int minTurnSpeed = 15;
float turn_tolerance = 2.0f;

// EMA smoothing
float leftFiltered = 50.0f;
float rightFiltered = 50.0f;
const float EMA_ALPHA = 0.35f;

// Emergency pivot profile
int emergencyPivotBoost = 20;
unsigned long emergencyPivotDuration = 160;
unsigned long emergencyPivotRampMs = 60;

// Prefer-open-direction
bool preferOpenDirection = true;
float openDirectionThreshold = 18.0f;
float openDirectionBiasGain = 1.8f;

// Telemetry (minimal)
bool telemetryEnabled = false;
unsigned long telemetryFreqMs = 500;
unsigned long lastTelemetry = 0;

// ----------------- Setup -----------------
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Wire.begin();

  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  calibrateGyro();

  Serial1.println(F("Maze robot ready. Send 'help'."));
  prevTime = millis();
}

// ----------------- Loop -----------------
void loop() {
  handleSerialInput();

  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  prevTime = currentTime;

  if (telemetryEnabled && (millis() - lastTelemetry >= telemetryFreqMs)) {
    lastTelemetry = millis();
    dumpTelemetry();
  }

  switch (currentState) {
    case STOPPED: stopMotorsAndResetPID(); break;
    case DRIVING_CORRIDOR: handleCorridorFollowing(); break;
    case APPROACHING_WALL: handleApproachingWall(); break;
    case TURN_AT_WALL: handleTurnAtWall(); break;
    case CLEAR_CORNER: handleClearCorner(); break;
  }

  delay(8);
}

// ----------------- Gyro calibration -----------------
void calibrateGyro() {
  currentState = STOPPED;
  xmotion.MotorControl(0,0);
  Serial1.println(F("Calibrating gyro... do not move!"));
  long sum = 0;
  for (int i=0;i<400;i++){
    mpu.getRotation(&gx,&gy,&gz);
    sum += gz;
    delay(3);
  }
  gyroZoffset = sum / 400.0f;
  yaw = 0.0f;
  integral = 0.0f;
  previous_wall_error = 0.0f;
  Serial1.print(F("Gyro offset: ")); Serial1.println(gyroZoffset);
}

// ----------------- Sensors -----------------
float RawToCm(int rawValue) {
  float voltage = rawValue * (5.0f / 1023.0f);
  if (voltage < 0.25f) return 80.0f;
  float distance = 27.86f * powf(voltage, -1.15f);
  if (distance > 80.0f) distance = 80.0f;
  if (distance < 10.0f) distance = 10.0f;
  return distance;
}

void readAndFilterSensors(float &leftCm, float &rightCm) {
  float rawL = RawToCm(analogRead(SENSOR_LEFT));
  float rawR = RawToCm(analogRead(SENSOR_RIGHT));
  leftFiltered = EMA_ALPHA * rawL + (1.0f - EMA_ALPHA) * leftFiltered;
  rightFiltered = EMA_ALPHA * rawR + (1.0f - EMA_ALPHA) * rightFiltered;
  leftCm = leftFiltered;
  rightCm = rightFiltered;
}

// ----------------- Added missing function -----------------
void stopMotorsAndResetPID() {
  xmotion.MotorControl(0, 0);
  integral = 0.0f;
  previous_wall_error = 0.0f;
}

// ----------------- Emergency pivot (blocking, short) -----------------
void doEmergencyPivot(bool leftIsClose) {
  int dir = leftIsClose ? 1 : -1;
  unsigned long start = millis();
  unsigned long endTime = start + emergencyPivotDuration;
  unsigned long rampEnd = start + min(emergencyPivotRampMs, emergencyPivotDuration);
  while (millis() < endTime) {
    unsigned long now = millis();
    float t = (now <= rampEnd && emergencyPivotRampMs>0) ? (float)(now - start)/emergencyPivotRampMs : 1.0f;
    int boost = (int)roundf(t * emergencyPivotBoost);
    int s = constrain(pivotSpeed + boost, 0, 100);
    if (dir == 1) xmotion.MotorControl(s, -s);
    else xmotion.MotorControl(-s, s);
    handleSerialInput(); // allow commands
    if (currentState == STOPPED) { xmotion.StopMotors(80); return; }
    delay(12);
  }
  xmotion.StopMotors(80);
  delay(30);
}

// ----------------- Corridor logic -----------------
void handleCorridorFollowing() {
  float leftCm, rightCm;
  readAndFilterSensors(leftCm, rightCm);

  if (leftCm < FRONT_WALL_CM && rightCm < FRONT_WALL_CM) {
    Serial1.println(F("Front wall -> approach"));
    currentState = APPROACHING_WALL;
    return;
  }

  if (leftCm <= HARD_PIVOT_CM || rightCm <= HARD_PIVOT_CM) {
    Serial1.println(F("Emergency pivot"));
    if (leftCm <= HARD_PIVOT_CM && leftCm < rightCm) doEmergencyPivot(true);
    else if (rightCm <= HARD_PIVOT_CM && rightCm < leftCm) doEmergencyPivot(false);
    else { xmotion.MotorControl(-pivotSpeed,-pivotSpeed); delay(120); xmotion.StopMotors(60); }
    integral = 0.0f; previous_wall_error = 0.0f;
    return;
  }

  float wall_error = rightCm - leftCm;
  float effective_error = (fabsf(wall_error) < wall_deadzone) ? 0.0f : wall_error;

  float p_term = Kp * effective_error;
  integral += effective_error * dt;
  if (integral > integral_clamp) integral = integral_clamp;
  if (integral < -integral_clamp) integral = -integral_clamp;
  float i_term = Ki * integral;
  float derivative = (effective_error - previous_wall_error) / dt;
  previous_wall_error = effective_error;
  float d_term = Kd * derivative;
  float correction = p_term + i_term + d_term;

  float avoidBias = 0.0f;
  if (leftCm < WALL_DISTANCE_CM) avoidBias -= (WALL_DISTANCE_CM - leftCm) * K_avoid;
  if (rightCm < WALL_DISTANCE_CM) avoidBias += (WALL_DISTANCE_CM - rightCm) * K_avoid;

  float openBias = 0.0f;
  float diff = rightCm - leftCm;
  if (preferOpenDirection && fabsf(diff) > openDirectionThreshold) openBias = openDirectionBiasGain * diff;

  float totalCorrection = correction + avoidBias + openBias;

  int leftSpeed = constrain((int)roundf(baseSpeed + totalCorrection), 0, 100);
  int rightSpeed = constrain((int)roundf(baseSpeed - totalCorrection), 0, 100);
  xmotion.MotorControl(leftSpeed, rightSpeed);

  if (!telemetryEnabled) {
    static unsigned long last = 0;
    if (millis() - last > 800) {
      last = millis();
      Serial1.print(F("L:")); Serial1.print(leftCm,1);
      Serial1.print(F(" R:")); Serial1.print(rightCm,1);
      Serial1.print(F(" Err:")); Serial1.println(wall_error,2);
    }
  }
}

void handleApproachingWall() {
  float leftCm, rightCm;
  readAndFilterSensors(leftCm, rightCm);
  float avg = (leftCm + rightCm) * 0.5f;
  if (avg <= APPROACH_STOP_DISTANCE_CM) {
    Serial1.println(F("At wall -> turn"));
    xmotion.MotorControl(0,0);
    delay(80);
    currentState = TURN_AT_WALL;
  } else {
    xmotion.MotorControl(pivotSpeed,pivotSpeed);
  }
}

void handleTurnAtWall() {
  Serial1.println(F("Gyro turn"));
  TurnToAngle(90.0f);
  yaw = 0.0f; integral = 0.0f; previous_wall_error = 0.0f;
  stateTimer = millis();
  currentState = CLEAR_CORNER;
}

void handleClearCorner() {
  if (millis() - stateTimer > (unsigned long)clearCornerDuration) {
    currentState = DRIVING_CORRIDOR;
    return;
  }
  float leftCm, rightCm; readAndFilterSensors(leftCm, rightCm);
  float wall_error = rightCm - leftCm;
  float eff = (fabsf(wall_error) < wall_deadzone) ? 0.0f : wall_error;
  float p_term = Kp * eff;
  float d_term = Kd * ((eff - previous_wall_error) / dt);
  previous_wall_error = eff;
  float corr = p_term + d_term;
  int ls = constrain((int)roundf(baseSpeed + corr), 0, 100);
  int rs = constrain((int)roundf(baseSpeed - corr), 0, 100);
  xmotion.MotorControl(ls, rs);
}

// ----------------- Gyro turn -----------------
float getCurrentYawForTurn() {
  unsigned long t = millis();
  float dt_turn = (t - prevTime) / 1000.0f;
  if (dt_turn <= 0.0f) dt_turn = 0.001f;
  prevTime = t;
  mpu.getRotation(&gx,&gy,&gz);
  float gzCorrected = gz - gyroZoffset;
  float deltaYaw = (gzCorrected * dt_turn / GYRO_SCALE) * -1.0f;
  yaw += deltaYaw;
  return yaw;
}

void TurnToAngle(float targetDelta) {
  yaw = 0.0f; prevTime = millis();
  unsigned long start = millis();
  bool turningRight = (targetDelta > 0.0f);
  while (true) {
    handleSerialInput();
    if (currentState == STOPPED) { xmotion.StopMotors(100); return; }
    float cur = getCurrentYawForTurn();
    float err = fabsf(targetDelta - cur);
    if (err <= turn_tolerance) break;
    int motorSpeed = (int)(err * Kp_turn);
    motorSpeed = constrain(motorSpeed, minTurnSpeed, maxTurnSpeed);
    if (turningRight) xmotion.Right0(motorSpeed,0);
    else xmotion.Left0(motorSpeed,0);
    if (millis() - start > 4000) break;
    delay(10);
  }
  xmotion.StopMotors(100);
  delay(80);
  Serial1.print(F("Gyro done Yaw: ")); Serial1.println(yaw,3);
}

// ----------------- Telemetry -----------------
void dumpTelemetry() {
  float L,R; readAndFilterSensors(L,R);
  Serial1.print(F("DUMP L:")); Serial1.print(L,2);
  Serial1.print(F(" R:")); Serial1.print(R,2);
  Serial1.print(F(" base:")); Serial1.print(baseSpeed);
  Serial1.print(F(" Kp:")); Serial1.print(Kp,3);
  Serial1.print(F(" avoid:")); Serial1.print(K_avoid,2);
  Serial1.print(F(" preferOpen:")); Serial1.print(preferOpenDirection?1:0);
  Serial1.println();
}

// ----------------- Command handling (no Arduino String) -----------------
void handleSerialInput() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == '\n' || cmdIdx >= (sizeof(cmdBuf)-2)) {
      cmdBuf[cmdIdx] = 0;
      if (cmdIdx > 0) parseCommand(cmdBuf);
      cmdIdx = 0;
      cmdBuf[0] = 0;
    } else {
      cmdBuf[cmdIdx++] = c;
    }
  }
}

void parseCommand(const char *s) {
  while (*s == ' ' || *s == '\t' || *s == 'T')  s++;
  if (*s == 0) return;
  if (strcmp(s, "start") == 0) { integral = 0; previous_wall_error = 0; yaw=0; currentState = DRIVING_CORRIDOR; Serial1.println(F("start")); return; }
  if (strcmp(s, "stop") == 0)  { currentState = STOPPED; Serial1.println(F("stop")); return; }
  if (strcmp(s, "cal") == 0)   { calibrateGyro(); return; }
  if (strcmp(s, "dump") == 0)  { dumpTelemetry(); return; }
  const char *eq = strchr(s,'=');
  if (!eq) {
    if (strcmp(s,"help")==0) {
      Serial1.println(F("help: start/stop/cal/dump"));
      Serial1.println(F("kp= ki= kd= speed= pivot="));
      Serial1.println(F("thresh= hard= front= approach="));
      Serial1.println(F("avoid= dead= eb= ed= er="));
      Serial1.println(F("preferopen=on/off openthresh= openbias="));
      Serial1.println(F("telemetry=on/off telemfreq="));
    }
    return;
  }
  char key[24]; size_t klen = min((size_t)(eq - s), sizeof(key)-1);
  memcpy(key, s, klen); key[klen]=0;
  const char *val = eq+1;
  if (strcmp(key,"kp")==0) Kp = atof(val);
  else if (strcmp(key,"ki")==0) Ki = atof(val);
  else if (strcmp(key,"kd")==0) Kd = atof(val);
  else if (strcmp(key,"speed")==0) baseSpeed = constrain(atoi(val),0,100);
  else if (strcmp(key,"pivot")==0) pivotSpeed = constrain(atoi(val),0,100);
  else if (strcmp(key,"thresh")==0) WALL_DISTANCE_CM = atof(val);
  else if (strcmp(key,"hard")==0) HARD_PIVOT_CM = atof(val);
  else if (strcmp(key,"front")==0) FRONT_WALL_CM = atof(val);
  else if (strcmp(key,"approach")==0) APPROACH_STOP_DISTANCE_CM = atof(val);
  else if (strcmp(key,"avoid")==0) K_avoid = atof(val);
  else if (strcmp(key,"dead")==0) wall_deadzone = atof(val);
  else if (strcmp(key,"eb")==0) emergencyPivotBoost = atoi(val);
  else if (strcmp(key,"ed")==0) emergencyPivotDuration = (unsigned long)atoi(val);
  else if (strcmp(key,"er")==0) emergencyPivotRampMs = (unsigned long)atoi(val);
  else if (strcmp(key,"preferopen")==0) {
    if (val[0]=='o' || val[0]=='O' || val[0]=='1') preferOpenDirection = true; else preferOpenDirection = false;
  }
  else if (strcmp(key,"openthresh")==0) openDirectionThreshold = atof(val);
  else if (strcmp(key,"openbias")==0) openDirectionBiasGain = atof(val);
  else if (strcmp(key,"telemetry")==0) {
    if (val[0]=='o' || val[0]=='O' || val[0]=='1') telemetryEnabled = true; else telemetryEnabled = false;
  }
  else if (strcmp(key,"telemfreq")==0) telemetryFreqMs = (unsigned long)atoi(val);
  //Serial1.print(F("set ")); Serial1.println(key);
}
