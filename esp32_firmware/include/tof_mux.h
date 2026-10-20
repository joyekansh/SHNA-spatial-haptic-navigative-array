#ifndef TOF_MUX_H
#define TOF_MUX_H

#include <Wire.h>
#include "shna_config.h"

// Function to select active channel on TCA9548A I2C Multiplexer
bool tcaSelectChannel(uint8_t channel) {
    if (channel > 7) return false;
    
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(1 << channel);
    byte error = Wire.endTransmission();
    
    return (error == 0);
}

#endif