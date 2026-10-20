#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "shna_config.h"
#include "tof_mux.h"

// Array to hold instances for the 3 sensors
SparkFun_VL53L5CX myToFSensors[TOTAL_SENSORS];

bool initAllToFSensors() {
    bool allInitialized = true;
    
    for (uint8_t i = 0; i < TOTAL_SENSORS; i++) {
        // Switch MUX channel for sensor i
        if (!tcaSelectChannel(i)) {
            Serial.printf("Failed to select MUX channel for sensor %d\n", i);
            allInitialized = false;
            continue;
        }
    }
    
    return allInitialized;
}

#endif // TOF_SENSOR_H