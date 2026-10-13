// --- Setup ---
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Setup starting...");

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);

  Serial.println("Initializing IMU DMP...");
  while (imu.begin(CS_PIN, SPI) != ICM_20948_Stat_Ok) {
    Serial.println("IMU.begin failed; retrying...");
    delay(500);
  }
  if (imu.initializeDMP() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: initializeDMP failed!");
    while (1)
      ;
  }
  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: enableDMPSensor failed!");
    while (1)
      ;
  }
  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: setDMPODRrate failed!");
    while (1)
      ;
  }
  if (imu.enableFIFO() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: enableFIFO failed!");
    while (1)
      ;
  }
  if (imu.enableDMP() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: enableDMP failed!");
    while (1)
      ;
  }
  if (imu.resetDMP() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: resetDMP failed!");
    while (1)
      ;
  }
  if (imu.resetFIFO() != ICM_20948_Stat_Ok) {
    Serial.println("FATAL: resetFIFO failed!");
    while (1)
      ;
  }
  Serial.println("ICM-20948 DMP ready.");

  // Calibrate bias axes (vertical stance) → roll, yaw, and now pitch
  Serial.println("Calibrating biases... hold sensor vertical");
  const int N = 200;
  float sum_r = 0.0f, sum_y = 0.0f, sum_p = 0.0f;
  for (int i = 0; i < N; i++) {
    icm_20948_DMP_data_t d;
    if (imu.readDMPdataFromFIFO(&d) == ICM_20948_Stat_Ok
        && (d.header & DMP_header_bitmap_Quat6)) {
      double q1 = d.Quat6.Data.Q1 / 1073741824.0;
      double q2 = d.Quat6.Data.Q2 / 1073741824.0;
      double q3 = d.Quat6.Data.Q3 / 1073741824.0;
      double sumsq = q1*q1 + q2*q2 + q3*q3;
      double q0 = (sumsq < 1.0) ? sqrt(1.0 - sumsq) : 0.0;

      // Roll  (X-axis)
      double r0 = 2 * (q0*q1 + q2*q3);
      double r1 = 1 - 2 * (q1*q1 + q2*q2);
      float roll = atan2(r0, r1) * (180.0 / PI);

      // Yaw   (Z-axis)
      double y0 = 2 * (q0*q3 + q1*q2);
      double y1 = 1 - 2 * (q2*q2 + q3*q3);
      float yaw = atan2(y0, y1) * (180.0 / PI);

      // Pitch (Y-axis)
      // pitch = asin(2*(q0*q2 - q1*q3))
      double argument = 2 * (q0*q2 - q1*q3);
      if (argument > 1.0) argument = 1.0;
      if (argument < -1.0) argument = -1.0;
      float pitch = asin(argument) * (180.0 / PI);

      sum_r += roll;
      sum_y += yaw;
      sum_p += pitch;
    }
    delay(10);
  }
  roll_bias  = sum_r / N;
  yaw_bias   = sum_y / N;
  pitch_bias = sum_p / N;
  Serial.printf("Biases: roll=%.1f°, yaw=%.1f°, pitch=%.1f°\n",
                roll_bias, yaw_bias, pitch_bias);

  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  servoX.attach(xPin, 500, 2400);
  servoY.attach(yPin, 500, 2400);
  servoX.write(90);
  servoY.write(90);

  ledcAttach(escPin, escFreq, escRes);
  ledcAttach(leg_pin, 50, 16);
  Serial.println("Arming Reaction Wheel ESC: Sending 1000us. Please wait ~5 seconds...");
  ledcWrite(escPin, usToDuty(1000));
  delay(5000);
  Serial.println("ESC presumed armed.");

  tvc_prev_time_micros = micros();
  prevTime_rw_micros = micros();
  start_time = millis();

  firstMotorTriggered = false;

  Serial.println("Setup complete.");
}
