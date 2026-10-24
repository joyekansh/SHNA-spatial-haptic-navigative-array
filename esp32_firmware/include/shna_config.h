#ifndef SHNA_CONFIG_H
#define SHNA_CONFIG_H

#include <stdint.h>

// Core Allocation definitions for ESP32-S3 Dual-Core
#define SENSOR_TASK_CORE 0   // Core 0: High-frequency sensor polling & 3D spatial math
#define OUTPUT_TASK_CORE 1   // Core 1: Haptic matrix actuation & BLE telemetry

// I2C & Sensor Configuration
#define I2C_MASTER_FREQ_HZ 400000 // Fast Mode 400kHz
#define TCA9548A_ADDRESS   0x70   // Default multiplexer address
#define TOTAL_SENSORS      3      // Left (0), Center (1), Right (2) VL53L5CX
#define SENSOR_GRID_DIM    8      // 8x8 zones per sensor
#define ZONES_PER_SENSOR   64     // 8 * 8 = 64
#define TOTAL_SPATIAL_ZONES (TOTAL_SENSORS * ZONES_PER_SENSOR) // 192 zones

// Haptic Matrix Configuration: 3x2 Matrix (3 columns x 2 rows)
#define HAPTIC_COLS        3      // Left (Col 0), Center (Col 1), Right (Col 2)
#define HAPTIC_ROWS        2      // Row 0: Top (Overhang / Upper-body), Row 1: Bottom (Ground / Drop-offs)
#define TOTAL_HAPTICS      6      // 6 DRV2605L drivers (Channels 0 to 5 on MUX)

// Spatial Geometry & Sensor Mounting Constants
#define SENSOR_MOUNT_HEIGHT_MM 1200.0f // Wearable chest/torso mount height (1.2 meters)
#define HORIZONTAL_FOV_DEG     45.0f   // VL53L5CX horizontal Field of View
#define VERTICAL_FOV_DEG       45.0f   // VL53L5CX vertical Field of View
#define SENSOR_YAW_LEFT_DEG   -30.0f   // Left sensor angled 30 degrees left
#define SENSOR_YAW_CENTER_DEG   0.0f   // Center sensor forward
#define SENSOR_YAW_RIGHT_DEG   30.0f   // Right sensor angled 30 degrees right

// Timing Targets (in milliseconds)
#define SAFETY_LOOP_PERIOD_MS 50 // Strict sub-50ms latency safety target

// Hazard Classifications
enum HazardType_t : uint8_t {
    HAZARD_NONE = 0,
    HAZARD_OBSTACLE = 1,  // Standard obstacle in travel path
    HAZARD_OVERHANG = 2,  // Upper-body overhang (branches, signs, doorframes)
    HAZARD_DROP_OFF = 3   // Ground-level drop-off (stairs, curbs, potholes)
};

// 3D Cartesian Coordinate representation (in mm)
struct Point3D_t {
    float x;                // Lateral offset: Left < 0, Right > 0
    float y;                // Forward distance/depth
    float z;                // Height relative to ground level (Z=0)
    uint16_t rawDistanceMm; // Raw distance reported by sensor
    uint8_t sensorId;       // Sensor index (0: Left, 1: Center, 2: Right)
    uint8_t zoneId;         // Zone index within sensor (0 to 63)
    bool isValid;           // True if reading is non-zero and within valid range
};

// Comprehensive Haptic Matrix Command passed from Core 0 to Core 1 via FreeRTOS queue
struct HapticMatrixCommand_t {
    uint8_t motorEffects[TOTAL_HAPTICS];      // DRV2605L effect ID per motor (0 = off)
    HazardType_t hazards[TOTAL_HAPTICS];       // Specific hazard type per motor zone
    uint16_t minDistanceMm[TOTAL_HAPTICS];     // Minimum distance observed per motor zone
    uint16_t globalMinDistanceMm;              // Overall closest obstacle distance
    uint32_t tSensorReadStartUs;              // Timestamp (micros) when sensor read began for latency benchmarking
};

// Legacy Queue message structure (maintained for compatibility)
struct SensorAlert_t {
    uint8_t sensorId;
    uint16_t distanceMm;
};

#endif // SHNA_CONFIG_H