#ifndef SPATIAL_MATH_H
#define SPATIAL_MATH_H

#include <Arduino.h>
#include <math.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "shna_config.h"

// Radians conversion helper
#define DEG_TO_RAD_F 0.017453292519943295f

// Unified 192-Zone Spatial Depth Cloud
extern Point3D_t unifiedSpatialMap[TOTAL_SPATIAL_ZONES];

class SpatialGeometryEngine {
private:
    float sensorYawRad[TOTAL_SENSORS];
    float zoneAzimuthRad[SENSOR_GRID_DIM];
    float zoneElevationRad[SENSOR_GRID_DIM];
    float expectedGroundDistMm[SENSOR_GRID_DIM];

public:
    SpatialGeometryEngine() {
        // Sensor mounting yaw angles in radians (-30 deg, 0 deg, +30 deg)
        sensorYawRad[0] = SENSOR_YAW_LEFT_DEG * DEG_TO_RAD_F;
        sensorYawRad[1] = SENSOR_YAW_CENTER_DEG * DEG_TO_RAD_F;
        sensorYawRad[2] = SENSOR_YAW_RIGHT_DEG * DEG_TO_RAD_F;

        // VL53L5CX FOV step per zone: 45 degrees / 8 zones = 5.625 degrees
        float angleStepRad = (HORIZONTAL_FOV_DEG / (float)SENSOR_GRID_DIM) * DEG_TO_RAD_F;

        for (int i = 0; i < SENSOR_GRID_DIM; i++) {
            // Column azimuth offset relative to sensor optical center
            // c=0 -> -19.6875 deg, c=7 -> +19.6875 deg
            zoneAzimuthRad[i] = ((float)i - 3.5f) * angleStepRad;

            // Row elevation offset relative to sensor optical center
            // r=0 (top) -> +19.6875 deg, r=7 (bottom) -> -19.6875 deg
            zoneElevationRad[i] = (3.5f - (float)i) * angleStepRad;

            // Expected distance to flat ground for downwards-looking rows
            // For downward pitch (elevation < 0), d = H_mount / sin(|elevation|)
            if (zoneElevationRad[i] < -0.05f) {
                float sinElev = sinf(fabsf(zoneElevationRad[i]));
                expectedGroundDistMm[i] = SENSOR_MOUNT_HEIGHT_MM / sinElev;
            } else {
                expectedGroundDistMm[i] = 4000.0f; // Above horizon or parallel
            }
        }
    }

    // 1. Transform raw polar depth readings into 3D Cartesian coordinates (X, Y, Z)
    void transformToCartesian(uint8_t sensorId, uint8_t row, uint8_t col, uint16_t distMm, Point3D_t &pt) {
        pt.sensorId = sensorId;
        pt.zoneId = row * SENSOR_GRID_DIM + col;
        pt.rawDistanceMm = distMm;

        if (distMm == 0 || distMm > 3500) {
            pt.isValid = false;
            pt.x = 0.0f;
            pt.y = 4000.0f;
            pt.z = 0.0f;
            return;
        }

        pt.isValid = true;

        // Composite azimuth angle = Sensor Mount Yaw + Zone Azimuth Offset
        float totalAzimuth = sensorYawRad[sensorId] + zoneAzimuthRad[col];
        float elevation = zoneElevationRad[row];

        float cosElevation = cosf(elevation);
        float sinElevation = sinf(elevation);

        // Cartesian frame:
        // X = Lateral offset (Left < 0, Right > 0)
        // Y = Forward depth (Distance ahead)
        // Z = Height relative to ground level (Z=0 is ground)
        pt.x = (float)distMm * sinf(totalAzimuth) * cosElevation;
        pt.y = (float)distMm * cosf(totalAzimuth) * cosElevation;
        pt.z = SENSOR_MOUNT_HEIGHT_MM + ((float)distMm * sinElevation);
    }

    // 2. Map target distance to DRV2605L waveform effect IDs
    uint8_t mapDistanceToEffect(uint16_t distMm, HazardType_t hazard) {
        if (hazard == HAZARD_DROP_OFF) {
            // Distinct sharp double/triple alert for drop-offs (potholes/stairs)
            return 12; // Sharp click alert 100%
        }

        if (distMm < 400) {
            return 47; // Strong urgent buzz for critical obstacle (< 40cm)
        } else if (distMm < 800) {
            return 14; // Sharp medium pulse for imminent hazard (40cm - 80cm)
        } else if (distMm < 1500) {
            return 1;  // Gentle single click for distant alert (80cm - 1.5m)
        }

        return 0; // Clear / no vibration
    }

    // 3. Complete 3D spatial mapping, hazard detection, and downsampling to 3x2 matrix
    void processSpatialDepthMap(
        VL53L5CX_ResultsData rawData[TOTAL_SENSORS],
        HapticMatrixCommand_t &cmd
    ) {
        // Initialize command structure
        for (uint8_t m = 0; m < TOTAL_HAPTICS; m++) {
            cmd.motorEffects[m] = 0;
            cmd.hazards[m] = HAZARD_NONE;
            cmd.minDistanceMm[m] = 4000;
        }
        cmd.globalMinDistanceMm = 4000;

        // Temporary tracking across 3 columns (0: Left, 1: Center, 2: Right)
        // Rows: [0] = Top (Overhang / Upper-body), [1] = Bottom (Ground / Drop-offs)
        uint16_t colTopMinDist[HAPTIC_COLS] = {4000, 4000, 4000};
        uint16_t colBottomMinDist[HAPTIC_COLS] = {4000, 4000, 4000};
        bool colDropOffFlag[HAPTIC_COLS] = {false, false, false};
        bool colOverhangFlag[HAPTIC_COLS] = {false, false, false};

        // Populate unified 192-zone spatial cloud
        for (uint8_t s = 0; s < TOTAL_SENSORS; s++) {
            for (uint8_t r = 0; r < SENSOR_GRID_DIM; r++) {
                for (uint8_t c = 0; c < SENSOR_GRID_DIM; c++) {
                    uint8_t zoneIdx = r * SENSOR_GRID_DIM + c;
                    uint16_t d = rawData[s].distance_mm[zoneIdx];

                    uint16_t globalIdx = s * ZONES_PER_SENSOR + zoneIdx;
                    transformToCartesian(s, r, c, d, unifiedSpatialMap[globalIdx]);
                    Point3D_t &pt = unifiedSpatialMap[globalIdx];

                    // Determine horizontal column bin (Left, Center, Right)
                    // Incorporate overlapping zone resolution:
                    // Left sensor's outer right zones blend with Center's outer left zones
                    uint8_t colBin = s;
                    if (s == 0 && c >= 6) {
                        colBin = 1; // Left sensor inner edge overlaps with center path
                    } else if (s == 2 && c <= 1) {
                        colBin = 1; // Right sensor inner edge overlaps with center path
                    }

                    // A. Upper-body overhang detection (Rows 0-2: elevated hazards)
                    if (r <= 2) {
                        if (pt.isValid && pt.y < 1800.0f && pt.z > 1300.0f) {
                            if (pt.rawDistanceMm < colTopMinDist[colBin]) {
                                colTopMinDist[colBin] = pt.rawDistanceMm;
                                colOverhangFlag[colBin] = true;
                            }
                        }
                    }
                    // B. Ground-level drop-off detection (Rows 5-7: downward looking rays)
                    else if (r >= 5) {
                        float expectedFloor = expectedGroundDistMm[r];
                        // If measured distance is significantly greater than expected floor distance,
                        // or sensor receives no return bounce where floor should be, detect drop-off
                        if ((pt.isValid && (float)pt.rawDistanceMm > expectedFloor + 350.0f) ||
                            (!pt.isValid && expectedFloor < 2200.0f)) {
                            colDropOffFlag[colBin] = true;
                        } else if (pt.isValid && pt.y < 1800.0f) {
                            if (pt.rawDistanceMm < colBottomMinDist[colBin]) {
                                colBottomMinDist[colBin] = pt.rawDistanceMm;
                            }
                        }
                    }
                    // C. Mid-torso standard obstacle path (Rows 3-4)
                    else {
                        if (pt.isValid && pt.y < 2000.0f) {
                            if (pt.rawDistanceMm < colBottomMinDist[colBin]) {
                                colBottomMinDist[colBin] = pt.rawDistanceMm;
                            }
                        }
                    }

                    // Track global minimum obstacle distance
                    if (pt.isValid && pt.rawDistanceMm < cmd.globalMinDistanceMm) {
                        cmd.globalMinDistanceMm = pt.rawDistanceMm;
                    }
                }
            }
        }

        // 4. Synthesize 3x2 Haptic Matrix Commands:
        // Top Row: Motor 0 (Left-Top), Motor 1 (Center-Top), Motor 2 (Right-Top)
        for (uint8_t c = 0; c < HAPTIC_COLS; c++) {
            uint8_t motorIdx = c; // 0, 1, 2
            cmd.minDistanceMm[motorIdx] = colTopMinDist[c];

            if (colOverhangFlag[c] && colTopMinDist[c] < 1500) {
                cmd.hazards[motorIdx] = HAZARD_OVERHANG;
                cmd.motorEffects[motorIdx] = mapDistanceToEffect(colTopMinDist[c], HAZARD_OVERHANG);
            }
        }

        // Bottom Row: Motor 3 (Left-Bottom), Motor 4 (Center-Bottom), Motor 5 (Right-Bottom)
        for (uint8_t c = 0; c < HAPTIC_COLS; c++) {
            uint8_t motorIdx = c + HAPTIC_COLS; // 3, 4, 5
            cmd.minDistanceMm[motorIdx] = colBottomMinDist[c];

            if (colDropOffFlag[c]) {
                cmd.hazards[motorIdx] = HAZARD_DROP_OFF;
                cmd.motorEffects[motorIdx] = mapDistanceToEffect(colBottomMinDist[c], HAZARD_DROP_OFF);
            } else if (colBottomMinDist[c] < 1500) {
                cmd.hazards[motorIdx] = HAZARD_OBSTACLE;
                cmd.motorEffects[motorIdx] = mapDistanceToEffect(colBottomMinDist[c], HAZARD_OBSTACLE);
            }
        }
    }
};

// Global spatial geometry engine instance & point cloud
extern SpatialGeometryEngine spatialEngine;

#endif // SPATIAL_MATH_H
