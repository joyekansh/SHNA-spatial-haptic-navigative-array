#ifndef HAPTIC_DRV_H
#define HAPTIC_DRV_H

#include <Wire.h>
#include "shna_config.h"
#include "tof_mux.h"

#define DRV2605L_ADDR 0x5A
#define TOTAL_HAPTICS 6

// Initialize all haptic drivers across the multiplexer channels
bool initAllHapticDrivers() {
    bool allInitialized = true;

    for (uint8_t i = 0; i < TOTAL_HAPTICS; i++) {
        if (!tcaSelectChannel(i)) {
            Serial.printf("Failed to select MUX channel for Haptic Driver %d\n", i);
            allInitialized = false;
            continue;
        }

        Wire.beginTransmission(DRV2605L_ADDR);
        Wire.write(0x01); // Mode register
        Wire.write(0x00); // Internal trigger mode
        byte error = Wire.endTransmission();

        if (error != 0) {
            Serial.printf("DRV2605L Haptic Driver %d not detected on MUX channel %d!\n", i, i);
            allInitialized = false;
        } else {
            Serial.printf("Haptic Driver %d initialized successfully.\n", i);
        }
    }

    return allInitialized;
}

// Function to trigger a specific haptic effect waveform on a targeted motor index
void triggerHapticEffect(uint8_t motorIndex, uint8_t effectId) {
    if (motorIndex >= TOTAL_HAPTICS) return;

    if (tcaSelectChannel(motorIndex)) {
        Wire.beginTransmission(DRV2605L_ADDR);
        Wire.write(0x03); // Waveform sequencer register 1
        Wire.write(effectId); // Effect library ID
        Wire.write(0x00); // End sequence terminator
        Wire.endTransmission();

        // Go/run register trigger
        Wire.beginTransmission(DRV2605L_ADDR);
        Wire.write(0x0C); // GO register
        Wire.write(0x01); // Start playback
        Wire.endTransmission();
    }
}

#endif