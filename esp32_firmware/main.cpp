#include <Arduino.h>
#include <Wire.h>
#include "shna_config.h"
#include "tof_mux.h"

// Task handles for FreeRTOS
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t OutputTaskHandle = NULL;

// Forward declarations of core tasks
void sensorPollingTask(void *pvParameters);
void outputActuationTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Initializing SHNA Embedded Firmware...");

    // Initialize I2C bus for ESP32-S3 (default pins or standard pins)
    Wire.begin();
    Wire.setClock(I2C_MASTER_FREQ_HZ);

    // Create Sensor Task pinned to Core 0
    xTaskCreatePinnedToCore(
        sensorPollingTask,   // Task function
        "SensorPollTask",    // Name
        4096,                // Stack size (bytes)
        NULL,                // Parameter
        2,                   // Priority (higher)
        &SensorTaskHandle,   // Task handle
        SENSOR_TASK_CORE     // Core ID (0)
    );

    // Create Output Task pinned to Core 1
    xTaskCreatePinnedToCore(
        outputActuationTask, 
        "OutputActTask", 
        4096, 
        NULL, 
        1, 
        &OutputTaskHandle, 
        OUTPUT_TASK_CORE     // Core ID (1)
    );
}

void loop() {
    vTaskDelete(NULL);
}

// --- CORE 0: Sensor Ingestion & Spatial Math ---
void sensorPollingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_MS(SAFETY_LOOP_PERIOD_MS); // 50ms loop target

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Poll each of the 3 ToF sensors through the TCA9548A multiplexer
        for (uint8_t i = 0; i < TOTAL_SENSORS; i++) {
            if (tcaSelectChannel(i)) {
                // TODO: Add VL53L5CX read data logic here for channel i
                Serial.printf("Successfully accessed ToF Sensor on MUX Channel %d\n", i);
            } else {
                Serial.printf("Failed to switch to MUX Channel %d\n", i);
            }
        }
        
        // Delay until the next cycle to respect the 50ms safety timing target
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- CORE 1: Haptic Drivers & BLE Communication ---
void outputActuationTask(void *pvParameters) {
    while (1) {
        // TODO: Check queue from Core 0 and trigger DRV2605L haptic drivers
        vTaskDelay(pdMS_TO_MS(10)); 
    }
}