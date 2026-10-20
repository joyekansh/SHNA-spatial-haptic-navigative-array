#include <Arduino.h>
#include <Wire.h>
#include "shna_config.h"
#include "tof_mux.h"
#include "tof_sensor.h"
#include "haptic_drv.h"
#include "ble_telemetry.h"

// Task handles for FreeRTOS
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t OutputTaskHandle = NULL;

// FreeRTOS Queue Handle for Inter-Core Communication
QueueHandle_t sensorQueue = NULL;

// Forward declarations of core tasks
void sensorPollingTask(void *pvParameters);
void outputActuationTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Initializing SHNA Embedded Firmware...");

    // Initialize I2C bus for ESP32-S3
    Wire.begin();
    Wire.setClock(I2C_MASTER_FREQ_HZ);

    // Initialize all Haptic Drivers via MUX
    if (initAllHapticDrivers()) {
        Serial.println("All Haptic Drivers initialized successfully.");
    } else {
        Serial.println("Warning: Some Haptic Drivers failed to initialize.");
    }

    // Initialize all ToF Sensors via MUX
    if (initAllToFSensors()) {
        Serial.println("All ToF Sensors initialized successfully.");
    } else {
        Serial.println("Warning: Some ToF Sensors failed to initialize.");
    }

    // Create Queue for Inter-Core communication (capacity of 10 alerts)
    sensorQueue = xQueueCreate(10, sizeof(SensorAlert_t));
    if (sensorQueue == NULL) {
        Serial.println("Error: Failed to create sensor queue!");
    }

    // Create Core 0 Task: High-frequency sensor ingestion & spatial math
    xTaskCreatePinnedToCore(
        sensorPollingTask,
        "SensorPollingTask",
        4096,
        NULL,
        2,
        &SensorTaskHandle,
        SENSOR_TASK_CORE
    );

    // Create Core 1 Task: Haptic actuation & BLE telemetry
    xTaskCreatePinnedToCore(
        outputActuationTask,
        "OutputActuationTask",
        4096,
        NULL,
        1,
        &OutputTaskHandle,
        OUTPUT_TASK_CORE
    );
}

void loop() {
    // Arduino background loop is not used in a dual-core FreeRTOS architecture
    vTaskDelete(NULL);
}

// --- CORE 0: Sensor Ingestion & Spatial Math ---
void sensorPollingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(SAFETY_LOOP_PERIOD_MS);
    VL53L5CX_ResultsData measurementData;

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        for (uint8_t i = 0; i < TOTAL_SENSORS; i++) {
            if (tcaSelectChannel(i)) {
                // Check if new data is ready from the VL53L5CX
                if (myToFSensors[i].isDataReady()) {
                    if (myToFSensors[i].getRangingData(&measurementData)) {
                        // Find the minimum distance across 64 zones (8x8)
                        uint16_t minDistance = 4000;
                        for (int z = 0; z < 64; z++) {
                            uint16_t dist = measurementData.distance_mm[z];
                            if (dist > 0 && dist < minDistance) {
                                minDistance = dist;
                            }
                        }

                        // Send alert to Core 1 if obstacle is close
                        if (minDistance < 1000) {
                            SensorAlert_t alert;
                            alert.sensorId = i;
                            alert.distanceMm = minDistance;

                            xQueueSend(sensorQueue, &alert, 0);
                        }
                    }
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- CORE 1: Haptic Actuation & BLE Telemetry ---
void outputActuationTask(void *pvParameters) {
    SensorAlert_t incomingAlert;

    while (1) {
        // Wait for alerts from Core 0 via the queue
        if (xQueueReceive(sensorQueue, &incomingAlert, portMAX_DELAY) == pdPASS) {
            // Trigger corresponding haptic feedback
            triggerHapticEffect(incomingAlert.sensorId, incomingAlert.distanceMm);

            // Send telemetry via BLE (passing sensorId, distance, and intensity/status flag)
            sendBLETelemetry(incomingAlert.sensorId, incomingAlert.distanceMm, 0);
        }
    }
}