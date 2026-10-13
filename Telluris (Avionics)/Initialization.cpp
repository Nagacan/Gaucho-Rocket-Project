// === SPI VERSION with TVC + Reaction‐Wheel Control ===
// Make sure to uncomment "#define ICM_20948_USE_DMP" in ICM_20948_C.h
#include <SPI.h>
#include <ESP32Servo.h>
#include "ICM_20948.h"
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>
#include <array>  // added for lookup table
#include "BluetoothSerial.h"

// --- SPI pins for VSPI (default) ---
#define SPI_SCLK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define CS_PIN    5  // Chip‐select for ICM-20948

// --- Reaction‐wheel ESC on GPIO27 ---
const int escPin = 4;
const int escFreq = 50; // 50 Hz for typical ESC PWM
const int escRes = 16;  // 16-bit PWM resolution

// --- TVC servos on two GPIOs ---
Servo servoX, servoY;
int xPin = 2;
int yPin = 32;

// --- PID constants for reaction wheel (yaw rate) ---
const float Kp_rw = 3.3125f, Ki_rw = 0.2f, Kd_rw = 1.3f;
float prevError_rw   = 0.0f, 
      integral_rw    = 0.0f;
unsigned long prevTime_rw_micros = 0;

// --- PID constants for TVC (roll/pitch) ---
const float Kp_tvc = 1.5f;
const float Ki_tvc = 0.1f;
const float Kd_tvc = 0.05f;  // START VERY LOW (e.g., 0.0) AND TUNE UP
const float TVC_TIME_STEP_TARGET = 0.01f;
const float tvc_deadzone = 2.0f;
const float LPF_BETA = 0.2f;

// Variables for the LPF-based TVC PID
float current_roll_lpf  = 0.0f;
float current_pitch_lpf = 0.0f;
float tvc_error_integral[2] = { 0.0f, 0.0f };
float tvc_prev_error[2]     = { 0.0f, 0.0f };
unsigned long tvc_prev_time_micros = 0;

// TVC Limp Mode (Shutdown)
const float TVC_MAX_ANGLE_LIMIT   = 30.0f;  // Max filtered angle before TVC enters limp mode
const float TVC_RESET_ANGLE_LIMIT = 25.0f;  // Angle below which TVC can exit limp mode
bool tvc_in_limp_mode = false;

// --- IMU object ---
ICM_20948_SPI imu;

// --- Bias offsets ---
float roll_bias  = 0.0f;
float yaw_bias   = 0.0f;    // still computed but not used in TVC
float pitch_bias = 0.0f;

// Bluetooth
BluetoothSerial SerialBT;
char btCmd;
bool launchSequence = false;  // main logic flag

// MoI Consts
const float I_rocket = 0.002395f;
const float I_wheel  = 0.002051f;

// Motor Constants
unsigned long lastMotorTime  = 0,
              motorInterval = 1000;  // ms between motor triggers

// --- Helpers ---
uint32_t usToDuty(int us) {
  return (uint32_t)us * ((1UL << escRes) - 1) / 20000;
}

static float lpf(float prev_lpf_val, float current_measurement, float beta) {
  return beta * current_measurement + (1.0f - beta) * prev_lpf_val;
}

std::array<std::pair<int, int>, 163> lookupTable = { { { -24, 0 }, { -24, 1 }, { -24, 2 }, { -24, 3 }, { -24, 4 },
  { -23, 5 }, { -23, 6 }, { -23, 7 }, { -23, 8 }, { -23, 9 }, { -22, 10 }, { -22, 11 }, { -22, 12 }, { -22, 13 }, { -22, 14 },
  { -21, 15 }, { -21, 16 }, { -21, 17 }, { -21, 18 }, { -21, 19 }, { -20, 20 }, { -20, 21 }, { -20, 22 }, { -20, 23 }, { -20, 24 },
  { -19, 25 }, { -19, 26 }, { -19, 27 }, { -19, 28 }, { -19, 29 }, { -18, 30 }, { -18, 31 }, { -18, 32 }, { -18, 33 }, { -18, 34 },
  { -17, 35 }, { -17, 36 }, { -17, 37 }, { -17, 38 }, { -17, 39 }, { -16, 40 }, { -16, 41 }, { -16, 42 }, { -16, 43 }, { -16, 44 },
  { -15, 45 }, { -15, 46 }, { -15, 47 }, { -15, 48 }, { -15, 49 }, { -14, 50 }, { -13, 51 }, { -13, 52 }, { -13, 53 }, { -12, 54 },
  { -12, 55 }, { -12, 56 }, { -11, 57 }, { -11, 58 }, { -11, 59 }, { -11, 60 }, { -10, 61 }, { -10, 62 }, { -10, 63 }, { -9, 64 },
  { -9, 65 }, { -9, 66 }, { -8, 67 }, { -8, 68 }, { -8, 69 }, { -7, 70 }, { -7, 71 }, { -7, 72 }, { -6, 73 }, { -6, 74 }, { -5, 75 },
  { -5, 76 }, { -5, 77 }, { -4, 78 }, { -4, 79 }, { -4, 80 }, { -3, 81 }, { -3, 82 }, { -3, 83 }, { -2, 84 }, { -2, 85 }, { -2, 86 },
  { -1, 87 }, { -1, 88 }, { -1, 89 }, { 0, 90 }, { 0, 91 }, { 0, 92 }, { 1, 93 }, { 1, 94 }, { 1, 95 }, { 2, 96 }, { 2, 97 }, { 3, 98 },
  { 3, 99 }, { 4, 100 }, { 4, 101 }, { 4, 102 }, { 4, 103 }, { 4, 104 }, { 5, 105 }, { 5, 106 }, { 5, 107 }, { 6, 108 }, { 6, 109 },
  { 6, 110 }, { 7, 111 }, { 7, 112 }, { 7, 113 }, { 8, 114 }, { 8, 115 }, { 8, 116 }, { 8, 117 }, { 9, 118 }, { 9, 119 }, { 10, 120 },
  { 10, 121 }, { 10, 122 }, { 10, 123 }, { 10, 124 }, { 11, 125 }, { 11, 126 }, { 11, 127 }, { 12, 128 }, { 12, 129 }, { 12, 130 },
  { 13, 131 }, { 13, 132 }, { 13, 133 }, { 13, 134 }, { 14, 135 }, { 14, 136 }, { 14, 137 }, { 14, 138 }, { 14, 139 }, { 15, 140 },
  { 15, 141 }, { 15, 142 }, { 15, 143 }, { 15, 144 }, { 15, 145 }, { 15, 146 }, { 15, 147 }, { 15, 148 }, { 15, 149 }, { 16, 150 },
  { 16, 151 }, { 16, 152 }, { 16, 153 }, { 16, 154 }, { 16, 155 }, { 16, 156 }, { 16, 157 }, { 16, 158 }, { 16, 159 }, { 16, 160 },
  { 16, 161 }, { 16, 162 } } };

// right now 70 degrees is index 0 in the lookup table (can shift offset if needed)
int getTheta4(int theta2) {
  int offset = 70;
  int index;
  if (theta2 > 110) {
    index = 110 - offset;
  } else if (theta2 < 70) {
    index = 70 - offset;
  } else {
    index = theta2 - offset;
  }
  return lookupTable[index].second;
}

void handleBT() {
  if (SerialBT.available()) {
    btCmd = SerialBT.read();
    switch (btCmd) {
      case '1':
        Serial.println("BT: start launch sequence");
        launchSequence = true;
        break;
      case '0':
        Serial.println("BT: stop launch sequence");
        launchSequence = false;
        break;
    }
  }
}

void triggerMotor() {
  // e.g. ledcWrite(escPin, usToDuty(2000));
}

void triggerLegs() {
  // e.g. digitalWrite(legPin, HIGH); delay(100); digitalWrite(legPin, LOW);
}
