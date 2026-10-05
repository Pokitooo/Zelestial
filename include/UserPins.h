#pragma once

#include <Arduino.h>

// ============================================================
// BOARD
// ============================================================

constexpr uint32_t USER_GPIO_LED = PC13;

// ============================================================
// SENSOR I2C
// ============================================================

constexpr uint32_t SENSOR_SDA = PB7;
constexpr uint32_t SENSOR_SCL = PB6;

constexpr uint32_t SENSOR_I2C_SPEED = 200000UL;

// ============================================================
// GNSS I2C
// ============================================================

constexpr uint32_t GPS_SDA = PB9_ALT1;
constexpr uint32_t GPS_SCL = PB10;

constexpr uint32_t GPS_RESET = PB8;

constexpr uint32_t GPS_I2C_SPEED = 200000UL;

// ============================================================
// SENSOR ADDRESSES
// ============================================================

constexpr uint8_t BME280_ADDR  = 0x76;
constexpr uint8_t AHT20_ADDR   = 0x38;
constexpr uint8_t LPS22_ADDR   = 0x5D;
constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t LIS3DH_ADDR  = 0x19;
constexpr uint8_t GNSS_ADDR    = 0x42;

// ============================================================
// SENSOR TASK RATES
// ============================================================

constexpr uint32_t BME280_PERIOD_MS = 100;
constexpr uint32_t AHT20_PERIOD_MS  = 100;
constexpr uint32_t LPS22_PERIOD_MS  = 100;

constexpr uint32_t MPU6050_PERIOD_MS = 20;
constexpr uint32_t LIS3DH_PERIOD_MS  = 20;

constexpr uint32_t GNSS_PERIOD_MS = 100;

// ============================================================
// TELEMETRY
// ============================================================

constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;

constexpr uint16_t TEAM_ID = 1043;

// ============================================================
// E22
// ============================================================

constexpr uint32_t E22_RX = PA3;
constexpr uint32_t E22_TX = PA2;

constexpr uint32_t E22_BAUDRATE = 9600;

// E22 control pins if connected
constexpr uint32_t E22_M0 = PA5;
constexpr uint32_t E22_M1 = PA6;

// ============================================================
// E22 CONFIGURATION
// ============================================================

constexpr uint16_t E22_ADDRESS = 0xffff;
constexpr uint8_t  E22_CHANNEL = 0x17;

constexpr uint16_t E22_DEST_ADDRESS = 0x0002;
constexpr uint8_t  E22_DEST_CHANNEL = E22_CHANNEL;
