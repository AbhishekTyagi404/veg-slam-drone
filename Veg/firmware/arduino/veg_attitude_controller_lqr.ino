
/*
 * veg_attitude_controller_lqr.ino
 * Arduino-based inner-loop LQR controller for attitude stabilization (Roll, Pitch)
 * Communicates via Serial with Raspberry Pi for setpoints and telemetry
 */

#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

unsigned long prevTime = 0;
float dt;

// IMU raw readings
float roll, pitch, yaw;
float gyroX, gyroY, gyroZ;

// Control inputs
float roll_setpoint = 0.0;
float pitch_setpoint = 0.0;

// LQR Gains (manually tuned or from MATLAB)
float K_roll[2] = {4.5, 0.85};   // Kp, Kd for roll (phi, phi_dot)
float K_pitch[2] = {4.5, 0.85};  // Kp, Kd for pitch (theta, theta_dot)

// Motor outputs (PWM)
int motor1, motor2, motor3, motor4;
const int motorPins[4] = {3, 5, 6, 9}; // PWM capable pins

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.initialize();

  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  if (dt < 0.01) return; // 100 Hz
  prevTime = currentTime;

  readIMU();
  receiveSetpoints();
  computeLQR();
  writeMotors();
}

// IMU reading (roll/pitch estimation simplified)
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
      sscanf(line.c_str(), "SET %f %f", &roll_setpoint, &pitch_setpoint);
    }
  }
}

void computeLQR() {
  float roll_error = roll_setpoint - roll;
  float pitch_error = pitch_setpoint - pitch;

  float roll_u = K_roll[0] * roll_error - K_roll[1] * gyroX / 131.0;
  float pitch_u = K_pitch[0] * pitch_error - K_pitch[1] * gyroY / 131.0;

  // Base throttle for hover
  int base = 1300;

  motor1 = base + roll_u - pitch_u;
  motor2 = base - roll_u - pitch_u;
  motor3 = base - roll_u + pitch_u;
  motor4 = base + roll_u + pitch_u;

  motor1 = constrain(motor1, 1100, 1900);
  motor2 = constrain(motor2, 1100, 1900);
  motor3 = constrain(motor3, 1100, 1900);
  motor4 = constrain(motor4, 1100, 1900);
}

void writeMotors() {
  analogWrite(motorPins[0], map(motor1, 1000, 2000, 0, 255));
  analogWrite(motorPins[1], map(motor2, 1000, 2000, 0, 255));
  analogWrite(motorPins[2], map(motor3, 1000, 2000, 0, 255));
  analogWrite(motorPins[3], map(motor4, 1000, 2000, 0, 255));
}
