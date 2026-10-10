// Teensy 4.1 pin map and hardware constants for the SLR-1 flight computer.
// See docs/05-avionics.md for the schematic description.
#pragma once
#include <stdint.h>

namespace board {

// I2C buses
constexpr uint32_t I2C0_HZ = 400000;   // Wire  (18 SDA, 19 SCL): BMP390 0x77, GPS 0x42
constexpr uint32_t I2C1_HZ = 1000000;  // Wire1 (17 SDA1, 16 SCL1): BMI088 accel 0x18, gyro 0x68
constexpr uint8_t BMI088_ACC_ADDR = 0x18;
constexpr uint8_t BMI088_GYR_ADDR = 0x68;
constexpr uint8_t BMP390_ADDR = 0x77;

// Lidar (Benewake TFmini-S, UART 115200) on Serial1: pin 0 = RX1, pin 1 = TX1
constexpr uint32_t LIDAR_BAUD = 115200;
constexpr float LIDAR_OFFSET_M = 0.02f;  // lens above the aft end of the airframe

// LoRa telemetry (RFM95W) on SPI: 10 CS, 11 MOSI, 12 MISO, 13 SCK
constexpr uint8_t RADIO_CS = 10, RADIO_IRQ = 30, RADIO_RST = 31;
constexpr float RADIO_MHZ = 915.0f;

// Servos. Pins on the same FlexPWM sub-module share a PWM frequency.
constexpr uint8_t SERVO_GIMBAL_Y = 2;     // FlexPWM4.2, 333 Hz digital servo
constexpr uint8_t SERVO_GIMBAL_Z = 3;     // FlexPWM4.2, 333 Hz
constexpr uint8_t SERVO_PADDLES = 4;      // FlexPWM2.0, 333 Hz
constexpr uint8_t SERVO_AIRBRAKE_LATCH = 7; // FlexPWM1.3, 50 Hz analog micro servo
constexpr uint8_t SERVO_LEG_PIN = 28;     // FlexPWM3.1, 50 Hz
constexpr uint8_t SERVO_CHUTE_LATCH = 29; // FlexPWM3.1, 50 Hz
constexpr float FAST_SERVO_HZ = 333.0f;
constexpr float SLOW_SERVO_HZ = 50.0f;

// Pyro (landing-motor igniters A and B, fired together). Logic-level
// N-MOSFET low-side switches with 10k gate pull-downs; the pyro battery
// passes through the physical ARM KEY before reaching the igniters.
constexpr uint8_t PYRO_A = 20, PYRO_B = 21;
constexpr uint32_t PYRO_PULSE_MS = 1000;
constexpr uint8_t CONT_A = 14, CONT_B = 15;     // ADC: continuity sense (A0, A1)
constexpr float CONT_THRESHOLD_V = 0.4f;        // above = igniter present (see schematic)
constexpr uint8_t ARM_SENSE = 22;               // LOW = arm key inserted & turned
constexpr uint8_t VBAT_SENSE = 23;              // ADC (A9) through a 1/3 divider
constexpr float VBAT_DIVIDER = 3.0f;
constexpr float VBAT_MIN = 7.4f;                // 2S LiPo: don't arm below this
constexpr uint8_t BUZZER = 37;

// Compass heading of fin 1 (body +y) when the rocket sits on the pad, in
// degrees clockwise from true north. Point fin 1 north and leave this 0, or
// measure it at the pad. Used to rotate GPS velocity into the attitude frame.
constexpr float PAD_HEADING_DEG = 0.0f;

// Sensor -> body axis mapping (body +x = toward the nose). Flight board is
// mounted with the IMU's +z axis pointing at the nose and +x toward fin 1.
// body = M * sensor, M has one +-1 per row.
constexpr int8_t IMU_MAP[3][3] = {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}};

// Servo calibration (fill from the laser-pointer calibration, docs/09).
// gimbal deflection (rad) -> pulse (us): pulse = center + gain * deflection
constexpr float GIMBAL_Y_CENTER_US = 1500, GIMBAL_Y_US_PER_RAD = 1800;
constexpr float GIMBAL_Z_CENTER_US = 1500, GIMBAL_Z_US_PER_RAD = 1800;
// Paddle servo pulse at throttle = 1.0, 0.75, 0.5, 0.25 (measured on the
// load-cell stand; interpolated linearly in between).
constexpr float PADDLE_US[4] = {1000, 1250, 1500, 1800};
constexpr float AIRBRAKE_LATCH_CLOSED_US = 1000, AIRBRAKE_LATCH_OPEN_US = 2000;
constexpr float LEG_PIN_HOLD_US = 1000, LEG_PIN_RELEASE_US = 2000;
constexpr float CHUTE_LATCH_CLOSED_US = 1000, CHUTE_LATCH_OPEN_US = 2000;

}  // namespace board
