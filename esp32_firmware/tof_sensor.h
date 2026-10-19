#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Wire.h>
#include <SparkFun_VL53L5CX_Data_Literal.h> // Ensure library is added in platformio/arduino
#include <SparkFun_VL53L5CX_ArduinoLibrary.h>
#include "shna_config.h"
#include "tof_mux.h"

// Array to hold instances for the 3 sensors
SparkFun_VL53L5CX myToFSensors[TOTAL_SENSORS];

bool initAllToFSensors() {
    bool allInitialized = true;

    for (uint8_t i = 0; i < TOTAL_SENSORS; i++) {
        // Switch MUX channel for sensor i
        if (!tcaSelectChannel(i)) {
            Serial.printf("Failed to select MUX channel for Sensor %d\n", i);
            allInitialized = false;
            continue;
        }

        // Initialize the VL53L5CX sensor on this channel
        if (myToFSensors[i].begin() == false) {
            Serial.printf("Sensor %d (VL53L5CX) not detected on MUX channel %d!\n", i, i);
            allInitialized = false;
        } else {
            Serial.printf("Sensor %d initialized successfully.\n", i);
            myToFSensors[i].setResolution(8*8); // Set to 8x8 zones for spatial grid
            myToFSensors[i].setRangingFrequency(15); // 15Hz
            myToFSensors[i].startRanging();
        }
    }

    return allInitialized;
}

#endif