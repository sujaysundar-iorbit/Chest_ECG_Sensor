#include <SPI.h>
#include "protocentral_max30001.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//For Max30001 AFE
#define CES_CMDIF_PKT_START_1 0x0A
#define CES_CMDIF_PKT_START_2 0xFA
#define CES_CMDIF_TYPE_DATA 0x02
#define CES_CMDIF_PKT_STOP 0x0B
#define DATA_LEN 0x0C
#define ZERO 0
#define MAX30001_CS_PIN D7


MAX30001 max30001(MAX30001_CS_PIN);

signed long ecg_data;


const int BUFFER_SIZE = 128;  // Increased to hold the full ECG waveform

int16_t ecgBuffer[BUFFER_SIZE];
volatile int writeIndex = 0;
volatile int readIndex = 0;

BLECharacteristic *pCharacteristic;
TaskHandle_t ecgTaskHandle;
uint16_t currentMTU = 47;  // Start with default MTU size

uint32_t RTC_Index = 0;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();
  }

  void onMtuChanged(BLEServer *pServer, uint16_t MTU) {
    currentMTU = MTU;
    Serial.printf("MTU changed to: %u\n", MTU);
  }
};

int16_t convertToInt12(signed long ecgSample) {
  // // Ensure we're only working with the 18 least significant bits
  // ecgSample &= 0x3FFFF;  // 18 bits mask

  // // Check if the value is negative (18th bit is 1)
  // if (ecgSample & 0x20000) {
  //     // It's negative, so extend the sign bit
  //     ecgSample |= 0xFFFC0000;
  // }

  // // Scale down from 18 bits to 12 bits
  // // We'll use bit shifting to divide by 64 (2^6)
  // int16_t scaledSample = (int16_t)(ecgSample >> 6);
  int16_t scaledSample = map(ecgSample,-3000,3000,0,4096);
    // Ensure the result fits in 12 bits (-2048 to 2047)
    if (scaledSample > 4096) {
    scaledSample = 4096;
  }
  else if (scaledSample < 0) {
    scaledSample = 0;
  }

  return scaledSample;
}

// Use the hardcoded ECG waveform
void generateECGWave() {
  writeIndex = (writeIndex + 1) % BUFFER_SIZE;
  signed long ecgSample = max30001.getECGSamples();

  // Scale down the ecgSample to fit into int16_t range
  int16_t ecgSampleInt16 = convertToInt12(ecgSample);


  //PRINT ORIGINAL and 16bit SAMPLE
  ecgBuffer[writeIndex] = ecgSampleInt16;
  Serial.print(ecgSample);
  Serial.print(", ");
  Serial.print(ecgSampleInt16);
  Serial.println(",");
}

void sendECGPacket() {
  uint8_t packet[43];  // 4 bytes RTC + 39 bytes ECG data

  if (RTC_Index == 26)
    RTC_Index = 0;


  uint32_t rtcTick = RTC_Index;  // millis() / (1000.0 / 4096.0);

  RTC_Index++;
  // Pack RTC Tick (Little Endian)
  packet[0] = (rtcTick >> 0) & 0xFF;
  packet[1] = (rtcTick >> 8) & 0xFF;
  packet[2] = (rtcTick >> 16) & 0xFF;
  packet[3] = (rtcTick >> 24) & 0xFF;

  for (int i = 0; i < 26; i++) {
    int packetIndex = 4 + (i * 3) / 2;
    int16_t sample = ecgBuffer[readIndex];
    readIndex = (readIndex + 1) % BUFFER_SIZE;

    // Pack ECG data (1.5 bytes per sample, Little Endian)
    if (i % 2 == 0) {
      packet[packetIndex] = sample & 0xFF;
      packet[packetIndex + 1] = (sample >> 8) & 0x0F;
    } else {
      packet[packetIndex] |= (sample & 0x0F) << 4;
      packet[packetIndex + 1] = (sample >> 4) & 0xFF;
    }
  }

  // DATA Sending Debug Prints
  // Serial.print("RTC: ");
  // for (int i = 0; i < 4; i++) {
  //   Serial.printf("%02X ", packet[i]);
  // }
  // Serial.print("ECG: ");
  // for (int i = 4; i < 43; i++) {
  //   Serial.printf("%02X ", packet[i]);
  // }
  // Serial.println();

  // Calculate the maximum payload size (MTU - 3 for ATT header)
  uint16_t maxPayloadSize = currentMTU - 3;

  // Determine how many bytes we can send
  uint16_t bytesToSend = min((uint16_t)sizeof(packet), maxPayloadSize);

  pCharacteristic->setValue(packet, bytesToSend);
  pCharacteristic->notify();
}

void ecgTask(void *parameter) {
  for (;;) {
    generateECGWave();
    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

void bleTask(void *parameter) {
  for (;;) {
    if ((writeIndex - readIndex + BUFFER_SIZE) % BUFFER_SIZE >= 26) {
      sendECGPacket();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  // Serial.begin(115200);

  //For MAX30001
  pinMode(MAX30001_CS_PIN, OUTPUT);
  digitalWrite(MAX30001_CS_PIN, HIGH);
  SPI.begin();
  bool ret = max30001.max30001ReadInfo();
  if (ret) {
    Serial.println("MAX 30001 read ID Success");
  } else {
    while (!ret) {
      // stay here untill the issue is fixed.
      ret = max30001.max30001ReadInfo();
      Serial.println("Failed to read ID, please make sure all the pins are connected");
      delay(5000);
    }
  }
  max30001.BeginECGBioZ();
  //For BLE Comm
  BLEDevice::init("Chest Sensor iO");
  BLEDevice::setMTU(517);  // Set maximum supported MTU size

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());  // Set server callbacks

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  xTaskCreate(ecgTask, "ECG Task", 8192, NULL, 1, &ecgTaskHandle);
  xTaskCreate(bleTask, "BLE Task", 8192, NULL, 1, NULL);

  Serial.println("BLE server started. Waiting for connections...");
}

void loop() {
  // Nothing to do here, everything is handled by tasks
}