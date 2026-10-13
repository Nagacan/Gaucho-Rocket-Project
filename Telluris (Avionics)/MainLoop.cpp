// --- Main loop ---
void loop() {
  // handleBT();
  // if (!launchSequence) {
  //   delay(50);
  //   return;
  // }

  unsigned long loop_start_micros = micros();

  // 1) TVC control using DMP Game Rotation Vector
  icm_20948_DMP_data_t dmp_data;
  if (imu.readDMPdataFromFIFO(&dmp_data) == ICM_20948_Stat_Ok
      && (dmp_data.header & DMP_header_bitmap_Quat6)) {

    double q1 = static_cast<double>(dmp_data.Quat6.Data.Q1) / 1073741824.0;  // X-axis rotation
    double q2 = static_cast<double>(dmp_data.Quat6.Data.Q2) / 1073741824.0;  // Y-axis rotation
    double q3 = static_cast<double>(dmp_data.Quat6.Data.Q3) / 1073741824.0;  // Z-axis rotation
    double q_sum_sq = q1*q1 + q2*q2 + q3*q3;
    double q0 = (q_sum_sq < 1.0) ? sqrt(1.0 - q_sum_sq) : 0.0;

    // ---- Compute Roll (around IMU X-axis) ----
    double t0_roll = 2.0 * (q0*q1 + q2*q3);
    double t1_roll = 1.0 - 2.0 * (q1*q1 + q2*q2);
    float current_roll_raw = atan2(t0_roll, t1_roll) * (180.0 / PI);

    // ---- Compute Pitch (around IMU Y-axis) ----
    // pitch = asin(2*(q0*q2 - q1*q3))
    double arg_p = 2.0 * (q0*q2 - q1*q3);
    if (arg_p > 1.0) arg_p = 1.0;
    if (arg_p < -1.0) arg_p = -1.0;
    float current_pitch_raw = asin(arg_p) * (180.0 / PI);

    // DLPF + remove bias
    current_roll_lpf  = lpf(current_roll_lpf,  current_roll_raw,  LPF_BETA) - roll_bias;
    current_pitch_lpf = lpf(current_pitch_lpf, current_pitch_raw, LPF_BETA) - pitch_bias;

    // ---------- TVC Limp Mode Logic ------------------------------
    if (!tvc_in_limp_mode &&
        (fabs(current_roll_lpf)  > TVC_MAX_ANGLE_LIMIT ||
         fabs(current_pitch_lpf) > TVC_MAX_ANGLE_LIMIT)) {
      Serial.println("!!! TVC Entering LIMP MODE: Angle limit exceeded !!!");
      tvc_in_limp_mode = true;
      servoX.write(90);  // Go to neutral
      servoY.write(90);
      tvc_error_integral[0] = 0.0f;
      tvc_error_integral[1] = 0.0f;
      tvc_prev_error[0]    = 0.0f;
      tvc_prev_error[1]    = 0.0f;
    }

    if (tvc_in_limp_mode &&
        fabs(current_roll_lpf)  < TVC_RESET_ANGLE_LIMIT &&
        fabs(current_pitch_lpf) < TVC_RESET_ANGLE_LIMIT) {
      Serial.println("TVC Exiting LIMP MODE: Angles back in range.");
      tvc_in_limp_mode = false;
      // PID state will rebuild on next active cycle
    }

    // --- TVC PID Control ---
    if (!tvc_in_limp_mode) {
      unsigned long current_tvc_micros = micros();
      float dt_tvc = (tvc_prev_time_micros == 0)
                         ? TVC_TIME_STEP_TARGET
                         : static_cast<float>(current_tvc_micros - tvc_prev_time_micros) * 1e-6f;
      if (dt_tvc <= 0.00001f) {
        dt_tvc = TVC_TIME_STEP_TARGET;
      }
      tvc_prev_time_micros = current_tvc_micros;

      float tvc_error[2];
      tvc_error[0] = -current_roll_lpf;
      tvc_error[1] = -current_pitch_lpf;

      float servo_command_angle_calculated[2] = { 90.0f, 90.0f };

      for (int axis = 0; axis < 2; ++axis) {
        if (fabs(tvc_error[axis]) < tvc_deadzone) {
          servo_command_angle_calculated[axis] = 90.0f;
          tvc_error_integral[axis] = 0.0f;
        } else {
          tvc_error_integral[axis] += tvc_error[axis] * dt_tvc;
          float derivative = (dt_tvc > 0.00001f)
                                 ? (tvc_error[axis] - tvc_prev_error[axis]) / dt_tvc
                                 : 0.0f;
          float pid_correction = Kp_tvc * tvc_error[axis]
                                 + Ki_tvc * tvc_error_integral[axis]
                                 + Kd_tvc * derivative;
          float max_deflection = 30.0f;
          pid_correction = constrain(pid_correction, -max_deflection, max_deflection);
          servo_command_angle_calculated[axis] = 90.0f + round(pid_correction);
        }
        tvc_prev_error[axis] = tvc_error[axis];
      }

      servoX.write(static_cast<int>(servo_command_angle_calculated[0]));
      servoY.write(static_cast<int>(servo_command_angle_calculated[1]));

      Serial.print("ACTIVE Roll: ");
      Serial.print(current_roll_lpf, 1);
      Serial.print(", Pitch: ");
      Serial.print(current_pitch_lpf, 1);
      Serial.print(" | ServoX: ");
      Serial.print(servo_command_angle_calculated[0], 1);
      Serial.print(", ServoY: ");
      Serial.println(servo_command_angle_calculated[1], 1);

    } else {  // TVC is in LIMP MODE
      servoX.write(90);
      servoY.write(90);
      Serial.print("LIMP MODE Roll: ");
      Serial.print(current_roll_lpf, 1);
      Serial.print(", Pitch: ");
      Serial.print(current_pitch_lpf, 1);
      Serial.println(" | Servos at Neutral.");
    }
  }  // End of DMP data processing

  // 2) Reaction-wheel Controller
  // If TVC is not in limp mode, run RW PID
  if (!tvc_in_limp_mode && imu.dataReady()) {
    imu.getAGMT();
    float yawRate = imu.gyrX();      // Z-axis gyro rate
    float targetYawRate = 0.0f;
    unsigned long current_rw_micros = micros();
    float dt_rw = (prevTime_rw_micros == 0)
                      ? TVC_TIME_STEP_TARGET
                      : static_cast<float>(current_rw_micros - prevTime_rw_micros) * 1e-6f;
    if (dt_rw <= 0.00001f) {
      dt_rw = TVC_TIME_STEP_TARGET;
    }
    prevTime_rw_micros = current_rw_micros;

    // — Reaction‐wheel linear function —
    float u = (yawRate * I_rocket * 10) / (I_wheel);
    int pulse = constrain(1500 - int(u), 1000, 2000);
    ledcWrite(escPin, usToDuty(pulse));

  } else if (tvc_in_limp_mode) {
    // If TVC is in limp mode, set reaction wheel to neutral
    ledcWrite(escPin, usToDuty(1500));
  }

  // fire the 2nd motor
  // if ((millis() - lastMotorTime) >= motorInterval) {
  //   triggerMotor();
  //   lastMotorTime = loop_start_micros;
  //   triggerLegs();
  // }

  // --- Maintain loop rate ---
  long loop_duration_micros      = micros() - loop_start_micros;
  long delay_needed_micros       = static_cast<long>(TVC_TIME_STEP_TARGET * 1e6f) - loop_duration_micros;
  if (delay_needed_micros > 0) {
    delayMicroseconds(delay_needed_micros);
  }
}
