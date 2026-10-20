#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "shna_config.h"

// Unique UUIDs for SHNA Telemetry Service and Characteristic
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a1"

BLEServer *pServer = NULL;
BLECharacteristic *pTelemetryCharacteristic = NULL;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnected(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client Connected.");
    }
    
    void onDisconnected(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client Disconnected. Restarting advertising...");
        pServer->startAdvertising();
    }
};

void initBLETelemetry() {
    BLEDevice::init("SHNA-Navigation-Array");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pTelemetryCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ   |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pTelemetryCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Telemetry Server Initialized and Advertising.");
}

void sendBLETelemetry(uint8_t sensorId, uint16_t distanceMm, uint8_t effectId) {
    if (deviceConnected) {
        char payload[32];
        snprintf(payload, sizeof(payload), "S:%d,D:%dmm,E:%d", sensorId, distanceMm, effectId);
        pTelemetryCharacteristic->setValue((uint8_t*)payload, strlen(payload));
        pTelemetryCharacteristic->notify();
    }
}

#endif