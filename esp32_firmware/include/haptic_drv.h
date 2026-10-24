#ifndef HAPTIC_DRV_H
#define HAPTIC_DRV_H

#include <Wire.h>
#include "shna_config.h"
#include "tof_mux.h"

#define DRV2605L_ADDR 0x5A

// Tracking motor state to prevent redundant I2C bus traffic
struct HapticState_t {
    uint8_t lastEffect;
    uint32_t lastTriggerTimeUs;
    bool isPlaying;
};

static HapticState_t motorStates[TOTAL_HAPTICS];

// Initialize all 6 haptic drivers across the multiplexer channels (0 to 5)
bool initAllHapticDrivers() {
    bool allInitialized = true;

    for (uint8_t i = 0; i < TOTAL_HAPTICS; i++) {
        motorStates[i].lastEffect = 0;
        motorStates[i].lastTriggerTimeUs = 0;
        motorStates[i].isPlaying = false;

        if (!tcaSelectChannel(i)) {
            Serial.printf("Failed to select MUX channel for Haptic Driver %d\n", i);
            allInitialized = false;
            continue;
        }

        // Configure DRV2605L
        Wire.beginTransmission(DRV2605L_ADDR);
        Wire.write(0x01); // Mode register
        Wire.write(0x00); // Internal trigger mode
        byte error = Wire.endTransmission();

        if (error != 0) {
            Serial.printf("DRV2605L Haptic Driver %d not detected on MUX channel %d!\n", i, i);
            allInitialized = false;
        } else {
            // Select Library 1 (LRA / ERM default library)
            Wire.beginTransmission(DRV2605L_ADDR);
            Wire.write(0x03); // Library selection register
            Wire.write(0x01); // Library 1 (Empty/Default standard TS2200 library)
            Wire.endTransmission();

            Serial.printf("Haptic Driver %d initialized successfully.\n", i);
        }
    }

    return allInitialized;
}

// Stop playback on a specific motor
void stopHaptic(uint8_t motorIndex) {
    if (motorIndex >= TOTAL_HAPTICS) return;

    if (tcaSelectChannel(motorIndex)) {
        Wire.beginTransmission(DRV2605L_ADDR);
        Wire.write(0x0C); // GO register
        Wire.write(0x00); // Stop playback
        Wire.endTransmission();

        motorStates[motorIndex].isPlaying = false;
        motorStates[motorIndex].lastEffect = 0;
    }
}

// Stop all motors across the matrix
void stopAllHaptics() {
    for (uint8_t i = 0; i < TOTAL_HAPTICS; i++) {
        stopHaptic(i);
    }
}

// Trigger a specific haptic effect waveform on a targeted motor index
void triggerHapticEffect(uint8_t motorIndex, uint8_t effectId) {
    if (motorIndex >= TOTAL_HAPTICS) return;

    // Rate limiting: avoid re-triggering the identical effect if triggered within last 80ms
    uint32_t now = micros();
    if (motorStates[motorIndex].lastEffect == effectId && 
        motorStates[motorIndex].isPlaying && 
        (now - motorStates[motorIndex].lastTriggerTimeUs < 80000)) {
        return;
    }

    if (tcaSelectChannel(motorIndex)) {
        // Load waveform into sequencer slot 1 (reg 0x04) and terminator into slot 2 (reg 0x05)
        Wire.beginTransmission(DRV2605L_ADDR);
        Wire.write(0x04);     // Waveform sequencer slot 1
        Wire.write(effectId); // Effect library ID
        Wire.write(0x00);     // Terminator (slot 2)
        Wire.endTransmission();

        // Fire GO register (reg 0x0C)
        Wire.beginTransmission(DRV2605L_ADDR);
        Wire.write(0x0C); // GO register
        Wire.write(0x01); // Start playback
        Wire.endTransmission();

        motorStates[motorIndex].lastEffect = effectId;
        motorStates[motorIndex].lastTriggerTimeUs = now;
        motorStates[motorIndex].isPlaying = true;
    }
}

// Map distance to DRV2605L effects (maintained for legacy compatibility)
uint8_t getHapticEffectForDistance(uint16_t distanceMm) {
    if (distanceMm < 400) {
        return 47; // Strong buzz / urgent alert (< 40cm)
    } else if (distanceMm < 800) {
        return 14; // Sharp medium pulse (40cm - 80cm)
    } else {
        return 1;  // Gentle single click for distant alerts (80cm - 1.5m)
    }
}

// Execute complete 3x2 matrix actuation command
void executeHapticMatrix(const HapticMatrixCommand_t &cmd) {
    for (uint8_t m = 0; m < TOTAL_HAPTICS; m++) {
        uint8_t effect = cmd.motorEffects[m];
        if (effect > 0) {
            triggerHapticEffect(m, effect);
        } else {
            // Stop motor if no effect is requested and it was active
            if (motorStates[m].isPlaying) {
                stopHaptic(m);
            }
        }
    }
}

#endif // HAPTIC_DRV_H