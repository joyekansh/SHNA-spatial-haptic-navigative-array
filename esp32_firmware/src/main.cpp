#include <Arduino.h>
#include <Wire.h>
#include "shna_config.h"
#include "tof_mux.h"
#include "tof_sensor.h"
#include "spatial_math.h"
#include "haptic_drv.h"
#include "ble_telemetry.h"

// Global instances for 192-zone spatial cloud & geometric engine
Point3D_t unifiedSpatialMap[TOTAL_SPATIAL_ZONES];
SpatialGeometryEngine spatialEngine;

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
    Serial.println("==================================================");
    Serial.println("  SHNA Spatial Haptic Navigation Array Firmware  ");
    Serial.println("  Member B: 3D Spatial Math & 3x2 Haptic Matrix  ");
    Serial.println("==================================================");

    // Initialize I2C bus for ESP32-S3 (400kHz Fast Mode)
    Wire.begin();
    Wire.setClock(I2C_MASTER_FREQ_HZ);

    // Initialize BLE Telemetry Server
    initBLETelemetry();

    // Initialize all 6 DRV2605L Haptic Drivers across MUX channels 0-5
    if (initAllHapticDrivers()) {
        Serial.println("[OK] All 6 Haptic Drivers initialized successfully.");
    } else {
        Serial.println("[WARN] Some Haptic Drivers failed to initialize.");
    }

    // Initialize all 3 VL53L5CX ToF Sensors (8x8 zone mode = 192 zones)
    if (initAllToFSensors()) {
        Serial.println("[OK] All 3 ToF Sensors initialized successfully (192 zones).");
    } else {
        Serial.println("[WARN] Some ToF Sensors failed to initialize.");
    }

    // Create FreeRTOS Queue for passing 3x2 Matrix Commands between cores
    sensorQueue = xQueueCreate(10, sizeof(HapticMatrixCommand_t));
    if (sensorQueue == NULL) {
        Serial.println("[ERR] Failed to create sensor queue!");
    }

    // Create Core 0 Task: High-frequency sensor ingestion & 3D spatial math (8KB stack)
    xTaskCreatePinnedToCore(
        sensorPollingTask,
        "SensorPollingTask",
        8192,
        NULL,
        2,
        &SensorTaskHandle,
        SENSOR_TASK_CORE
    );

    // Create Core 1 Task: 3x2 Haptic matrix actuation & BLE telemetry (4KB stack)
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

// --- CORE 0: Sensor Ingestion & 3D Spatial Geometry Math ---
void sensorPollingTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SAFETY_LOOP_PERIOD_MS);
    VL53L5CX_ResultsData rawData[TOTAL_SENSORS];

    while (1) {
        // High-precision timestamp marking sensor polling initiation (for latency benchmarking)
        uint32_t tStartUs = micros();

        // 1. Ingest raw 8x8 depth grids from all 3 ToF sensors via TCA9548A multiplexer
        for (uint8_t i = 0; i < TOTAL_SENSORS; i++) {
            if (tcaSelectChannel(i)) {
                if (myToFSensors[i].isDataReady()) {
                    myToFSensors[i].getRangingData(&rawData[i]);
                }
            }
        }

        // 2. Execute 3D coordinate transformations, overhang & drop-off detection, and 3x2 matrix reduction
        HapticMatrixCommand_t matrixCmd;
        matrixCmd.tSensorReadStartUs = tStartUs;
        spatialEngine.processSpatialDepthMap(rawData, matrixCmd);

        // 3. Non-blocking queue dispatch to Core 1
        if (sensorQueue != NULL) {
            xQueueSend(sensorQueue, &matrixCmd, 0);
        }

        // Strict 50ms periodic pacing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- CORE 1: 3x2 Haptic Matrix Actuation, BLE & Sub-50ms Benchmarking ---
void outputActuationTask(void *pvParameters) {
    HapticMatrixCommand_t incomingCmd;

    while (1) {
        // Block until a new matrix command arrives from Core 0
        if (xQueueReceive(sensorQueue, &incomingCmd, portMAX_DELAY) == pdPASS) {
            // 1. Actuate the 3x2 Haptic Matrix across the 6 DRV2605L drivers
            executeHapticMatrix(incomingCmd);

            // 2. Sub-50ms Safety Loop Latency Benchmarking (T_actuation_end - T_sensor_read_start)
            uint32_t tEndUs = micros();
            float latencyMs = (float)(tEndUs - incomingCmd.tSensorReadStartUs) / 1000.0f;

            if (latencyMs > (float)SAFETY_LOOP_PERIOD_MS) {
                Serial.printf("[LATENCY WARNING] Safety Loop deadline exceeded %dms: %.2f ms (Min Dist: %d mm)\n", 
                    SAFETY_LOOP_PERIOD_MS, latencyMs, incomingCmd.globalMinDistanceMm);
            } else {
                // Periodically log loop performance metrics (~1Hz)
                static uint32_t loopCount = 0;
                if (++loopCount % 20 == 0) {
                    Serial.printf("[PERF] Safety Loop Latency: %.2f ms | Closest: %d mm | Effects: [%d,%d,%d | %d,%d,%d]\n", 
                        latencyMs, incomingCmd.globalMinDistanceMm,
                        incomingCmd.motorEffects[0], incomingCmd.motorEffects[1], incomingCmd.motorEffects[2],
                        incomingCmd.motorEffects[3], incomingCmd.motorEffects[4], incomingCmd.motorEffects[5]);
                }
            }

            // 3. Stream live diagnostic telemetry via BLE
            sendBLEMatrixTelemetry(incomingCmd, latencyMs);
        }
    }
}