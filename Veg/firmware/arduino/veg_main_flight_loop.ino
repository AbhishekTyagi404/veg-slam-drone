
/*
 * veg_main_flight_loop.ino
 * Unified main loop for surveillance quadcopter – वेग
 * Allows switching between PID and LQR controllers
 * Includes hooks for Failure Detection and Identification (FDI)
 */

#include <Wire.h>
#include <MPU6050.h>

#define CONTROL_MODE_PID 1
#define CONTROL_MODE_LQR 2

int control_mode = CONTROL_MODE_LQR; // Change to CONTROL_MODE_PID if needed

MPU6050 imu;

unsigned long prevTime = 0;
float dt;

float roll = 0, pitch = 0, yaw = 0;
float gyroX, gyroY, gyroZ;

float roll_set = 0, pitch_set = 0, yaw_set = 0, alt_set = 0;

// PID variables
float pid_integral[3] = {0}, last_error[3] = {0};

// LQR gains
float K_roll[2] = {4.5, 0.85};
float K_pitch[2] = {4.5, 0.85};

// Motor pins
const int motorPins[4] = {3, 5, 6, 9};
int motor[4];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.initialize();

  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  if (dt < 0.01) return;
  prevTime = currentTime;

  readIMU();
  receiveSetpoints();

  if (control_mode == CONTROL_MODE_PID) {
    applyPID();
  } else {
    applyLQR();
  }

  checkFDI();  // simple fault detection placeholder
  writeMotors();
}

void readIMU() {
  imu.getMotion6(NULL, NULL, NULL, &gyroX, &gyroY, &gyroZ);
  roll += gyroX * dt / 131.0;
  pitch += gyroY * dt / 131.0;
  yaw += gyroZ * dt / 131.0;
}

void receiveSetpoints() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("SET")) {
      sscanf(line.c_str(), "SET %f %f %f %f", &roll_set, &pitch_set, &yaw_set, &alt_set);
    }
  }
}

void applyLQR() {
  float err_roll = roll_set - roll;
  float err_pitch = pitch_set - pitch;

  float u_roll = K_roll[0] * err_roll - K_roll[1] * gyroX / 131.0;
  float u_pitch = K_pitch[0] * err_pitch - K_pitch[1] * gyroY / 131.0;

  int base = 1300;

  motor[0] = base + u_roll - u_pitch;
  motor[1] = base - u_roll - u_pitch;
  motor[2] = base - u_roll + u_pitch;
  motor[3] = base + u_roll + u_pitch;
}

void applyPID() {
  float kp = 6.0, ki = 1.2, kd = 2.2;

  float err[2] = {roll_set - roll, pitch_set - pitch};
  for (int i = 0; i < 2; i++) {
    pid_integral[i] += err[i] * dt;
  }

  float roll_d = (roll - last_error[0]) / dt;
  float pitch_d = (pitch - last_error[1]) / dt;

  last_error[0] = roll;
  last_error[1] = pitch;

  float u_roll = kp * err[0] + ki * pid_integral[0] - kd * roll_d;
  float u_pitch = kp * err[1] + ki * pid_integral[1] - kd * pitch_d;

  int base = 1300;

  motor[0] = base + u_roll - u_pitch;
  motor[1] = base - u_roll - u_pitch;
  motor[2] = base - u_roll + u_pitch;
  motor[3] = base + u_roll + u_pitch;
}

void checkFDI() {
  for (int i = 0; i < 4; i++) {
    if (motor[i] >= 1900 || motor[i] <= 1100) {
      Serial.println("FDI: Potential motor anomaly detected.");
    }
  }
}

void writeMotors() {
  for (int i = 0; i < 4; i++) {
    motor[i] = constrain(motor[i], 1100, 1900);
    analogWrite(motorPins[i], map(motor[i], 1000, 2000, 0, 255));
  }
}
