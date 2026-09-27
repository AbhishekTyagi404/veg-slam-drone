/*
 * veg_flight_controller.ino — inner-loop attitude controller for the Veg quadcopter
 *
 * Replaces the old veg_main_flight_loop / veg_attitude_controller_pid /
 * veg_attitude_controller_lqr / veg_fdi_module sketches (which could not be
 * compiled together and could not drive ESCs).
 *
 * Board : Arduino Uno / Nano (ATmega328P), Arduino AVR Boards core >= 1.8.3
 * Libs  : Wire, Servo (built in), "MPU6050" by Electronic Cats (i2cdevlib) — Library Manager
 *
 * ---------------- Serial protocol (115200 baud, one command per line) ----------------
 *  Pi -> Arduino
 *    SET <roll_deg> <pitch_deg> <yaw_rate_dps> <throttle_0_to_1>   setpoint + heartbeat (send at >= 10 Hz)
 *    ARM                                                            arm (level, throttle low, link alive)
 *    DISARM                                                         motors off immediately
 *    MODE PID | MODE LQR                                            switch controller (disarmed only)
 *    CAL                                                            re-calibrate gyro (disarmed, keep still)
 *  Arduino -> Pi
 *    TEL <state> <roll> <pitch> <yaw_rate> <throttle> <m1> <m2> <m3> <m4> <loop_us>   at 10 Hz
 *    ACK <cmd> | ERR <reason> | FDI <message> | INFO <message>
 *
 *  NOTE: the 4th SET field is THROTTLE (0..1), not altitude. There is no altitude
 *  sensor on the Arduino, so altitude hold must be closed on the Pi (SLAM z).
 *  Values outside 0..1 are rejected, so an old "SET r p y 1.2" can never command full throttle.
 */

#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

// ============================ State ============================
enum FlightState : uint8_t { DISARMED = 0, ARMED = 1, FAILSAFE = 2 };
const char *STATE_NAMES[] = { "DISARMED", "ARMED", "FAILSAFE" };

MPU6050 imu;
Servo esc[4];

FlightState state = DISARMED;
uint8_t controlMode = DEFAULT_CONTROL_MODE;
bool imuOk = false;

// Attitude estimate
float roll = 0, pitch = 0;               // deg
float rollRate = 0, pitchRate = 0, yawRate = 0;  // deg/s
float gyroBias[3] = {0, 0, 0};           // raw LSB
bool attitudeInitialised = false;

// Setpoints (from Pi)
float rollSet = 0, pitchSet = 0, yawRateSet = 0, throttleSet = 0;
unsigned long lastSetMs = 0;
bool linkEverSeen = false;

// Controller memory
float iRoll = 0, iPitch = 0, iYaw = 0;

// Failsafe
float fsThrottle = 0;
unsigned long failsafeStartMs = 0;

// Outputs
int motorUs[4] = { MOTOR_OFF_US, MOTOR_OFF_US, MOTOR_OFF_US, MOTOR_OFF_US };

// Timing
unsigned long lastLoopUs = 0, lastTelemetryMs = 0, loopTimeUs = 0;

// FDI
unsigned long satSinceMs[4] = {0, 0, 0, 0};
unsigned long lastSatWarnMs = 0;
unsigned long attErrSinceMs = 0;

// ============================ Helpers ============================
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void writeAllMotors(int us) {
  for (uint8_t i = 0; i < 4; i++) { motorUs[i] = us; esc[i].writeMicroseconds(us); }
}

void resetIntegrators() { iRoll = iPitch = iYaw = 0; }

void disarm(const char *reason) {
  writeAllMotors(MOTOR_OFF_US);
  resetIntegrators();
  if (state != DISARMED) { Serial.print(F("INFO DISARMED ")); Serial.println(reason); }
  state = DISARMED;
  throttleSet = 0;
}

void enterFailsafe(const char *reason) {
  if (state != ARMED) return;
  if (throttleSet <= I_ENABLE_THROTTLE) { disarm(reason); return; }   // still on the ground: just stop
  state = FAILSAFE;
  failsafeStartMs = millis();
  fsThrottle = throttleSet;           // start from where we were, then only ever ramp DOWN
  Serial.print(F("FDI FAILSAFE ")); Serial.println(reason);
}

// ============================ IMU ============================
bool readImuRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.clearWireTimeoutFlag();
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if (Wire.getWireTimeoutFlag()) return false;
  return true;
}

bool calibrateGyro() {
  long sum[3] = {0, 0, 0};
  int16_t ax, ay, az, gx, gy, gz;
  int16_t gmin[3] = { 32767, 32767, 32767 }, gmax[3] = { -32768, -32768, -32768 };
  for (int n = 0; n < GYRO_CAL_SAMPLES; n++) {
    if (!readImuRaw(ax, ay, az, gx, gy, gz)) return false;
    int16_t g[3] = { gx, gy, gz };
    for (uint8_t k = 0; k < 3; k++) {
      sum[k] += g[k];
      if (g[k] < gmin[k]) gmin[k] = g[k];
      if (g[k] > gmax[k]) gmax[k] = g[k];
    }
    delay(2);
  }
  for (uint8_t k = 0; k < 3; k++) {
    if ((gmax[k] - gmin[k]) / GYRO_LSB_PER_DPS > 2.0f * GYRO_CAL_MAX_DEV) return false;  // it moved
    gyroBias[k] = (float)sum[k] / GYRO_CAL_SAMPLES;
  }
  attitudeInitialised = false;
  return true;
}

// Complementary filter in the chip frame, then map to Veg conventions with the sign flags.
bool updateAttitude(float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!readImuRaw(ax, ay, az, gx, gy, gz)) return false;

  float gxd = (gx - gyroBias[0]) / GYRO_LSB_PER_DPS;
  float gyd = (gy - gyroBias[1]) / GYRO_LSB_PER_DPS;
  float gzd = (gz - gyroBias[2]) / GYRO_LSB_PER_DPS;

  // Chip-frame angles from gravity (X fwd, Y left, Z up): rotation about +X / +Y
  float axf = ax, ayf = ay, azf = az;
  float rollAccChip  = atan2f(ayf, azf) * RAD_TO_DEG;
  float pitchAccChip = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * RAD_TO_DEG;
  float aNormG = sqrtf(axf * axf + ayf * ayf + azf * azf) / 8192.0f;   // +/-4 g range

  static float rollChip = 0, pitchChip = 0;
  if (!attitudeInitialised) {
    rollChip = rollAccChip; pitchChip = pitchAccChip; attitudeInitialised = true;
  } else {
    rollChip  += gxd * dt;
    pitchChip += gyd * dt;
    if (aNormG > ACCEL_TRUST_MIN_G && aNormG < ACCEL_TRUST_MAX_G) {   // ignore accel during hard manoeuvres
      rollChip  = COMP_FILTER_ALPHA * rollChip  + (1.0f - COMP_FILTER_ALPHA) * rollAccChip;
      pitchChip = COMP_FILTER_ALPHA * pitchChip + (1.0f - COMP_FILTER_ALPHA) * pitchAccChip;
    }
  }

  roll      = IMU_ROLL_SIGN  * rollChip  - ROLL_TRIM_DEG;
  pitch     = IMU_PITCH_SIGN * pitchChip - PITCH_TRIM_DEG;
  rollRate  = IMU_ROLL_SIGN  * gxd;
  pitchRate = IMU_PITCH_SIGN * gyd;
  yawRate   = IMU_YAW_SIGN   * gzd;
  return true;
}

// ============================ Serial (non-blocking) ============================
void handleCommand(char *line) {
  char *tok = strtok(line, " \t\r");
  if (!tok) return;

  if (strcmp(tok, "SET") == 0) {
    float v[4];
    for (uint8_t i = 0; i < 4; i++) {
      char *t = strtok(NULL, " \t\r");
      if (!t) { Serial.println(F("ERR SET needs 4 values")); return; }
      char *end;
      v[i] = (float)strtod(t, &end);
      if (end == t) { Serial.println(F("ERR SET bad number")); return; }
    }
    if (v[3] < 0.0f || v[3] > 1.0f) { Serial.println(F("ERR throttle must be 0..1")); return; }
    rollSet     = clampf(v[0], -MAX_ANGLE_CMD_DEG, MAX_ANGLE_CMD_DEG);
    pitchSet    = clampf(v[1], -MAX_ANGLE_CMD_DEG, MAX_ANGLE_CMD_DEG);
    yawRateSet  = clampf(v[2], -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);
    throttleSet = v[3];
    lastSetMs = millis();
    linkEverSeen = true;
    return;
  }

  if (strcmp(tok, "DISARM") == 0) { disarm("by command"); Serial.println(F("ACK DISARM")); return; }

  if (strcmp(tok, "ARM") == 0) {
    if (state != DISARMED)                                    { Serial.println(F("ERR already armed")); return; }
    if (!imuOk)                                               { Serial.println(F("ERR IMU not ok")); return; }
    if (!linkEverSeen || millis() - lastSetMs > LINK_TIMEOUT_MS) { Serial.println(F("ERR send SET first")); return; }
    if (throttleSet > ARM_MAX_THROTTLE)                       { Serial.println(F("ERR throttle not low")); return; }
    if (fabsf(roll) > ARM_MAX_TILT_DEG || fabsf(pitch) > ARM_MAX_TILT_DEG) { Serial.println(F("ERR not level")); return; }
    resetIntegrators();
    state = ARMED;
    Serial.println(F("ACK ARM"));
    return;
  }

  if (strcmp(tok, "MODE") == 0) {
    char *m = strtok(NULL, " \t\r");
    if (state != DISARMED) { Serial.println(F("ERR disarm first")); return; }
    if (m && strcmp(m, "PID") == 0) controlMode = CONTROL_MODE_PID;
    else if (m && strcmp(m, "LQR") == 0) controlMode = CONTROL_MODE_LQR;
    else { Serial.println(F("ERR MODE PID|LQR")); return; }
    Serial.print(F("ACK MODE ")); Serial.println(m);
    return;
  }

  if (strcmp(tok, "CAL") == 0) {
    if (state != DISARMED) { Serial.println(F("ERR disarm first")); return; }
    imuOk = calibrateGyro();
    Serial.println(imuOk ? F("ACK CAL") : F("ERR CAL failed (keep still)"));
    return;
  }

  Serial.println(F("ERR unknown command"));
}

void pollSerial() {
  static char buf[64];
  static uint8_t len = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (!overflow) { buf[len] = '\0'; handleCommand(buf); }
      len = 0; overflow = false;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      overflow = true;   // drop this line entirely
    }
  }
}

// ============================ Control ============================
void computeAndWriteMotors(float dt, float throttle) {
  float uRoll, uPitch;
  float eRoll = rollSet - roll, ePitch = pitchSet - pitch;
  bool integrate = throttle > I_ENABLE_THROTTLE;
  if (!integrate) resetIntegrators();

  if (controlMode == CONTROL_MODE_LQR) {
    // u = -K (x - x_ref), x = [angle, rate], x_ref = [setpoint, 0]
    uRoll  = K_ROLL[0]  * eRoll  - K_ROLL[1]  * rollRate;
    uPitch = K_PITCH[0] * ePitch - K_PITCH[1] * pitchRate;
  } else {
    if (integrate) {
      iRoll  = clampf(iRoll  + PID_ROLL[1]  * eRoll  * dt, -I_LIMIT_US, I_LIMIT_US);
      iPitch = clampf(iPitch + PID_PITCH[1] * ePitch * dt, -I_LIMIT_US, I_LIMIT_US);
    }
    uRoll  = PID_ROLL[0]  * eRoll  + iRoll  - PID_ROLL[2]  * rollRate;
    uPitch = PID_PITCH[0] * ePitch + iPitch - PID_PITCH[2] * pitchRate;
  }

  float eYaw = yawRateSet - yawRate;
  if (integrate) iYaw = clampf(iYaw + PID_YAW_RATE[1] * eYaw * dt, -I_LIMIT_US, I_LIMIT_US);
  float uYaw = YAW_MIX_SIGN * (PID_YAW_RATE[0] * eYaw + iYaw);

  float base = MOTOR_IDLE_US + throttle * (MOTOR_MAX_US - MOTOR_IDLE_US);
  float m[4], hi = -1e9f, lo = 1e9f;
  for (uint8_t i = 0; i < 4; i++) {
    m[i] = base + MIX[i][0] * uRoll + MIX[i][1] * uPitch + MIX[i][2] * uYaw;
    if (m[i] > hi) hi = m[i];
    if (m[i] < lo) lo = m[i];
  }
  // Desaturate: shift all motors together so attitude authority is kept
  float shift = 0;
  if (hi > MOTOR_MAX_US) shift = MOTOR_MAX_US - hi;
  else if (lo < MOTOR_IDLE_US) shift = MOTOR_IDLE_US - lo;
  for (uint8_t i = 0; i < 4; i++) {
    motorUs[i] = (int)clampf(m[i] + shift, MOTOR_IDLE_US, MOTOR_MAX_US);
    esc[i].writeMicroseconds(motorUs[i]);
  }
}

// ============================ FDI ============================
void runFDI(unsigned long now) {
  if (state == DISARMED) { attErrSinceMs = 0; for (uint8_t i = 0; i < 4; i++) satSinceMs[i] = 0; return; }

  // 1) Crash / flip: cut motors
  if (fabsf(roll) > CRASH_TILT_DEG || fabsf(pitch) > CRASH_TILT_DEG) { disarm("crash tilt"); return; }

  // 2) Persistent motor saturation = warning only (it is not proof of a failed rotor)
  for (uint8_t i = 0; i < 4; i++) {
    bool sat = motorUs[i] >= MOTOR_MAX_US || (motorUs[i] <= MOTOR_IDLE_US && throttleSet > I_ENABLE_THROTTLE);
    if (!sat) satSinceMs[i] = 0;
    else if (satSinceMs[i] == 0) satSinceMs[i] = now;
    else if (now - satSinceMs[i] > 1000 && now - lastSatWarnMs > 2000) {
      Serial.print(F("FDI WARN motor ")); Serial.print(i + 1); Serial.println(F(" saturated >1s"));
      lastSatWarnMs = now;
    }
  }

  // 3) Attitude can't follow the setpoint for 0.5 s (e.g. rotor/prop loss) -> failsafe descent
  if (state == ARMED && throttleSet > I_ENABLE_THROTTLE &&
      (fabsf(rollSet - roll) > 25.0f || fabsf(pitchSet - pitch) > 25.0f)) {
    if (attErrSinceMs == 0) attErrSinceMs = now;
    else if (now - attErrSinceMs > 500) enterFailsafe("attitude tracking lost");
  } else {
    attErrSinceMs = 0;
  }
}

// ============================ Telemetry ============================
void sendTelemetry() {
  if (Serial.availableForWrite() < 48) return;   // never block the control loop on serial
  Serial.print(F("TEL "));
  Serial.print(STATE_NAMES[state]); Serial.print(' ');
  Serial.print(roll, 1); Serial.print(' ');
  Serial.print(pitch, 1); Serial.print(' ');
  Serial.print(yawRate, 1); Serial.print(' ');
  Serial.print(state == FAILSAFE ? fsThrottle : throttleSet, 2);
  for (uint8_t i = 0; i < 4; i++) { Serial.print(' '); Serial.print(motorUs[i]); }
  Serial.print(' '); Serial.println(loopTimeUs);
}

// ============================ Setup / loop ============================
void setup() {
  // ESCs must see a LOW pulse from the very first frame (Servo defaults to 1500 us on attach!)
  for (uint8_t i = 0; i < 4; i++) {
    esc[i].writeMicroseconds(MOTOR_OFF_US);
    esc[i].attach(MOTOR_PINS[i], 1000, 2000);
    esc[i].writeMicroseconds(MOTOR_OFF_US);
  }

  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);   // a hung I2C bus must not freeze the flight loop (AVR core >= 1.8.3)

  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  imu.setDLPFMode(MPU6050_DLPF_BW_42);

  if (!imu.testConnection()) {
    Serial.println(F("ERR MPU6050 not found — will not arm"));
    imuOk = false;
  } else {
    delay(1000);   // let the frame settle
    imuOk = calibrateGyro();
    Serial.println(imuOk ? F("INFO gyro calibrated") : F("ERR gyro cal failed (keep still, send CAL)"));
  }
  Serial.print(F("INFO Veg FC ready, mode "));
  Serial.println(controlMode == CONTROL_MODE_LQR ? F("LQR") : F("PID"));
  lastLoopUs = micros();
}

void loop() {
  pollSerial();   // every pass, not only on control ticks

  unsigned long nowUs = micros();
  if (nowUs - lastLoopUs < LOOP_US) return;
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  if (dt > 0.02f) dt = 0.02f;           // guard against a stalled pass
  lastLoopUs = nowUs;
  unsigned long now = millis();

  if (!updateAttitude(dt)) {
    imuOk = false;
    if (state != DISARMED) disarm("IMU read failed");
  }

  // Link watchdog
  if (state == ARMED && now - lastSetMs > LINK_TIMEOUT_MS) enterFailsafe("link lost");

  runFDI(now);

  switch (state) {
    case DISARMED:
      writeAllMotors(MOTOR_OFF_US);
      break;

    case ARMED:
      computeAndWriteMotors(dt, throttleSet);
      break;

    case FAILSAFE: {
      // Level out, stop yawing, ease throttle to a descent value, then stop motors.
      rollSet = pitchSet = yawRateSet = 0;
      float step = FS_RAMP_PER_S * dt;
      if (fsThrottle > FS_THROTTLE) fsThrottle = max(FS_THROTTLE, fsThrottle - step);  // never ramps up
      computeAndWriteMotors(dt, fsThrottle);
      if (now - failsafeStartMs > FS_DESCENT_MS) disarm("failsafe descent complete");
      break;
    }
  }

  if (now - lastTelemetryMs >= 1000 / TELEMETRY_HZ) { lastTelemetryMs = now; sendTelemetry(); }
  loopTimeUs = micros() - nowUs;
}
