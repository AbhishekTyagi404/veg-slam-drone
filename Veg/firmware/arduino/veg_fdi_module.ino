
/*
 * veg_fdi_module.ino
 * Simple Fault Detection and Identification (FDI) module
 * Monitors motor outputs and control response to detect anomalies
 */

void runFDI(int motors[4], float roll, float pitch, float yaw) {
  static int fdi_triggered = 0;

  for (int i = 0; i < 4; i++) {
    if (motors[i] >= 1900 || motors[i] <= 1100) {
      Serial.print("FDI: Motor ");
      Serial.print(i + 1);
      Serial.println(" is hitting saturation limit.");
    }
  }

  // Check for persistent abnormal roll/pitch deviation
  if (abs(roll) > 45 || abs(pitch) > 45) {
    if (!fdi_triggered) {
      Serial.println("FDI ALERT: Unstable orientation detected. Activating failsafe...");
      fdi_triggered = 1;
      // Add logic to land or hover safely
    }
  } else {
    fdi_triggered = 0;
  }
}
