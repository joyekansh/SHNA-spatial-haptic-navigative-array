# SHNA - ESP32-S3 Firmware

Official embedded firmware of the SHNA project

## Architecture Overview
- **Microcontroller:** ESP32-S3 (Dual-Core FreeRTOS implementation)
- **Core 0 (50Hz Loop):** High-frequency sensor polling via TCA9548A I2C Multiplexer for 3x VL53L5CX ToF sensors.
- **Core 1:** Asynchronous haptic actuation management via 6x DRV2605L motor drivers and BLE communication.
