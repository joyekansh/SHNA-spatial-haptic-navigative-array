#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "shna_config.h"
#include "tof_mux.h"

// Array to hold instances for the 3 sensors (Left, Center, Right)
SparkFun_VL53L5CX myToFSensors[TOTAL_SENSORS];

bool initAllToFSensors() {
    bool allInitialized = true;
    
    for (uint8_t i = 0; i < TOTAL_SENSORS; i++) {
        // Switch MUX channel for sensor i (Channels 0, 1, 2)
        if (!tcaSelectChannel(i)) {
            Serial.printf("Failed to select MUX channel for sensor %d\n", i);
            allInitialized = false;
            continue;
        }

        if (!myToFSensors[i].begin()) {
            Serial.printf("VL53L5CX sensor %d not detected on MUX channel %d!\n", i, i);
            allInitialized = false;
            continue;
        }

        // Configure 8x8 zone depth mapping (64 zones per sensor)
        myToFSensors[i].setResolution(8 * 8);
        myToFSensors[i].setRangingFrequency(15); // 15 Hz continuous ranging
        myToFSensors[i].startRanging();

        Serial.printf("VL53L5CX sensor %d initialized successfully in 8x8 zone mode.\n", i);
    }
    
    return allInitialized;
}

#endif // TOF_SENSOR_H