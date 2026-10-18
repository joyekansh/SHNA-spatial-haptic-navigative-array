#ifndef SHNA_CONFIG_H
#define SHNA_CONFIG_H

// Core Allocation definitions for ESP32-S3 Dual-Core
#define SENSOR_TASK_CORE 0   // Core 0: High-frequency sensor polling & math
#define OUTPUT_TASK_CORE 1   // Core 1: Haptics, Actuation & BLE communication

// I2C & Sensor Configuration
#define I2C_MASTER_FREQ_HZ 400000  // Fast Mode 400kHz
#define TCA9548A_ADDRESS   0x70    // Default multiplexer address
#define TOTAL_SENSORS      3       // Left, Center, Right VL53L5CX sensors

// Timing Targets (in milliseconds)
#define SAFETY_LOOP_PERIOD_MS 50   // Strict sub-50ms latency target

#endif