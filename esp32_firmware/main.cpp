#include <Arduino.h>
#include "shna_config.h"

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
    // Main loop left empty because FreeRTOS handles the tasks independently on both cores.
    vTaskDelete(NULL);
}

// --- CORE 0: Sensor Ingestion & Spatial Math ---
void sensorPollingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_MS(20); // Run at 50Hz (20ms)

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // TODO: Add TCA9548A multiplexer channel switching and VL53L5CX data read here
        
        // Delay until the next 20ms cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- CORE 1: Haptic Drivers & BLE Communication ---
void outputActuationTask(void *pvParameters) {
    while (1) {
        // TODO: Check queue from Core 0 and trigger DRV2605L haptic drivers / BLE updates
        vTaskDelay(pdMS_TO_MS(10)); 
    }
}