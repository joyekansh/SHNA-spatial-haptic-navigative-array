# Spatial Haptic Navigation Array (SHNA)

SHNA is an advanced, wearable spatial orientation and obstacle-avoidance system designed for visually impaired individuals. It utilizes a high-density Time-of-Flight (ToF) sensor array and directional haptic feedback to map the immediate environment and translate spatial obstacles into intuitive tactile sensations.

---

## System Architecture & Hardware Stack

* **Microcontroller**: ESP32-S3 (Dual-Core Xtensa LX7)
* **Multiplexer**: TCA9548A I2C Multiplexer (Resolving address collisions for 3x ToF sensors and 6x haptic drivers at `0x29` and `0x5A`)
* **Sensors**: 3x VL53L5CX 8x8 Zone Time-of-Flight Sensors (Providing real-time spatial depth grids)
* **Actuators**: 6x DRV2605L Haptic Motor Drivers (Multiplexed linear resonant actuator / eccentric rotating mass drivers)
* **Wireless Telemetry**: Built-in ESP32-S3 Bluetooth Low Energy (BLE) server for real-time diagnostic logging and mobile app monitoring.

---

## Firmware Design & FreeRTOS Multi-Core Allocation

To guarantee zero latency in obstacle detection and prevent haptic feedback jitter, the firmware implements a strict dual-core architecture using FreeRTOS:

* **Core 0 (Sensor Ingestion & Spatial Math)**: 
  * Periodically polls all 3x VL53L5CX sensors across TCA9548A channels.
  * Evaluates 8x8 zone spatial distance arrays to find minimum obstacle proximity.
  * Pushes filtered distance alerts (`SensorAlert_t`) via a thread-safe FreeRTOS Queue.
* **Core 1 (Output Actuation & Telemetry)**:
  * Blocks on the FreeRTOS Queue waiting for obstacle events.
  * Dynamically maps distance thresholds to DRV2605L haptic waveforms (urgent buzzes vs. gentle clicks).
  * Broadcasts live telemetry packets (`S:id,D:mm,E:effect`) over BLE to connected client devices.

---

## Repository Structure

```text
shna_firmware/
│
├── include/
│   ├── shna_config.h       # Shared data structures, pinouts, and system constants
│   ├── tof_mux.h           # TCA9548A I2C channel multiplexing helper
│   ├── tof_sensor.h        # SparkFun VL53L5CX initialization and 8x8 zone parser
│   ├── haptic_drv.h        # DRV2605L multiplexed driver control and distance mapping
│   └── ble_telemetry.h     # BLE GATT server and notification telemetry stream
│
├── src/
│   └── main.cpp            # FreeRTOS task creation, core pinning, and execution loops
│
└── platformio.ini          # PlatformIO build configuration and library dependencies