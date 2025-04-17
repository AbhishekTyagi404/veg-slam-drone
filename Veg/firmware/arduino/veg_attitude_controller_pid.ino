
/*
 * veg_attitude_controller_pid.ino
 * Arduino-based PID attitude controller (roll, pitch, yaw, throttle)
 * Communicates with Raspberry Pi for setpoints and sends motor commands via PWM
 */

#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

unsigned long prevTime = 0;
float dt;

// IMU angles and gyro data
float roll, pitch, yaw;
float gyroX, gyroY, gyroZ;

// PID parameters (tuned for smooth performance)
float Kp_roll = 6.0, Ki_roll = 1.5, Kd_roll = 1.75;
float Kp_pitch = 5.0, Ki_pitch = 3.0, Kd_pitch = 3.0;
float Kp_yaw = 6.0, Ki_yaw = 1.5, Kd_yaw = 1.75;
float Kp_alt = 15.0, Ki_alt = 10.0, Kd_alt = 10.0;

// PID errors
float roll_err_i = 0, pitch_err_i = 0, yaw_err_i = 0, alt_err_i = 0;
float roll_last = 0, pitch_last = 0, yaw_last = 0, alt_last = 0;

// Setpoints
float roll_set = 0, pitch_set = 0, yaw_set = 0, alt_set = 0;

// Motors
int motor1, motor2, motor3, motor4;
const int motorPins[4] = {3, 5, 6, 9};

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
  if (dt < 0.01) return;  // ~100Hz loop
  prevTime = currentTime;

  readIMU();
  receiveSetpoints();
  computePID();
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

void computePID() {
  float err_roll = roll_set - roll;
  float err_pitch = pitch_set - pitch;
  float err_yaw = yaw_set - yaw;
  float err_alt = alt_set - 0.0; // Placeholder for altitude sensor

  roll_err_i += err_roll * dt;
  pitch_err_i += err_pitch * dt;
  yaw_err_i += err_yaw * dt;
  alt_err_i += err_alt * dt;

  float roll_d = (roll - roll_last) / dt;
  float pitch_d = (pitch - pitch_last) / dt;
  float yaw_d = (yaw - yaw_last) / dt;
  float alt_d = (0.0 - alt_last) / dt;

  roll_last = roll;
  pitch_last = pitch;
  yaw_last = yaw;
  alt_last = 0.0;

  float u_roll = Kp_roll * err_roll + Ki_roll * roll_err_i - Kd_roll * roll_d;
  float u_pitch = Kp_pitch * err_pitch + Ki_pitch * pitch_err_i - Kd_pitch * pitch_d;
  float u_yaw = Kp_yaw * err_yaw + Ki_yaw * yaw_err_i - Kd_yaw * yaw_d;
  float u_alt = Kp_alt * err_alt + Ki_alt * alt_err_i - Kd_alt * alt_d;

  int base = 1300 + u_alt;

  motor1 = base + u_roll - u_pitch + u_yaw;
  motor2 = base - u_roll - u_pitch - u_yaw;
  motor3 = base - u_roll + u_pitch + u_yaw;
  motor4 = base + u_roll + u_pitch - u_yaw;

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
