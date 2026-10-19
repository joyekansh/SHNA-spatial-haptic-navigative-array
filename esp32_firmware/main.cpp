#include <Arduino.h>
#include <Wire.h>
#include "shna_config.h"
#include "tof_mux.h"
#include "tof_sensor.h"
#include "haptic_drv.h"

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

    // Initialize Haptic Driver module
    if (initHapticDriver()) {
        Serial.println("DRV2605L Haptic Driver initialized successfully.");
    } else {
        Serial.println("Warning: Failed to initialize DRV2605L Haptic Driver!");
    }

    // Initialize all 3x VL53L5CX ToF Sensors via MUX
    if (initAllToFSensors()) {
        Serial.println("All VL53L5CX ToF Sensors initialized successfully.");
    } else {
        Serial.println("Warning: Some ToF sensors failed to initialize!");
    }

    // Create a Queue to hold up to 10 sensor alert messages
    sensorQueue = xQueueCreate(10, sizeof(SensorAlert_t));
    if (sensorQueue == NULL) {
        Serial.println("Error: Failed to create sensor queue!");
    }

    // Create Sensor Task pinned to Core 0
    xTaskCreatePinnedToCore(
        sensorPollingTask,   
        "SensorPollTask",    
        4096,                
        NULL,                
        2,                   
        &SensorTaskHandle,   
        SENSOR_TASK_CORE     
    );

    // Create Output Task pinned to Core 1
    xTaskCreatePinnedToCore(
        outputActuationTask, 
        "OutputActTask", 
        4096, 
        NULL, 
        1, 
        &OutputTaskHandle, 
        OUTPUT_TASK_CORE     
    );
}

void loop() {
    vTaskDelete(NULL);
}

// --- CORE 0: Sensor Ingestion & Spatial Math ---
void sensorPollingTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_MS(SAFETY_LOOP_PERIOD_MS); // 50ms loop target
    VL53L5CX_ResultsData measurementData;

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        for (uint8_t i = 0; i < TOTAL_SENSORS; i++) {
            if (tcaSelectChannel(i)) {
                // Check if new data is ready from the VL53L5CX sensor on this channel
                if (myToFSensors[i].isDataReady()) {
                    if (myToFSensors[i].getRangingData(&measurementData)) {
                        // Find the minimum distance across the 8x8 zone grid
                        uint16_t minDistance = 4000; 
                        for (int z = 0; z < 64; z++) {
                            uint16_t dist = measurementData.distance_mm[z];
                            if (dist > 0 && dist < minDistance) {
                                minDistance = dist;
                            }
                        }

                        // Send alert to Core 1 if obstacle is within 1 meter
                        if (minDistance < 1000) {
                            SensorAlert_t alert;
                            alert.sensorId = i;
                            alert.distanceMm = minDistance;

                            xQueueSend(sensorQueue, &alert, pdMS_TO_MS(5));
                        }
                    }
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- CORE 1: Haptic Drivers & BLE Communication ---
void outputActuationTask(void *pvParameters) {
    SensorAlert_t incomingAlert;

    while (1) {
        // Wait for incoming alert messages from Core 0
        if (xQueueReceive(sensorQueue, &incomingAlert, portMAX_DELAY) == pdTRUE) {
            Serial.printf("Core 1 received alert from Sensor %d: Distance = %d mm\n", 
                          incomingAlert.sensorId, incomingAlert.distanceMm);
            
            // Trigger haptic feedback based on the sensor event
            triggerHapticEffect(1); 
        }
    }
}