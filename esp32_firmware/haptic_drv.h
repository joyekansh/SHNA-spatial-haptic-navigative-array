#ifndef HAPTIC_DRV_H
#define HAPTIC_DRV_H

#include <Wire.h>
#include "shna_config.h"

// DRV2605L basic definitions
#define DRV2605L_ADDR 0x5A

// Simple initialization function for DRV2605L
bool initHapticDriver() {
    Wire.beginTransmission(DRV2605L_ADDR);
    Wire.write(0x01); // Mode register
    Wire.write(0x00); // Internal trigger mode (standby/active)
    byte error = Wire.endTransmission();
    return (error == 0);
}

// Function to trigger a specific haptic effect waveform
void triggerHapticEffect(uint8_t effectId) {
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

#endif