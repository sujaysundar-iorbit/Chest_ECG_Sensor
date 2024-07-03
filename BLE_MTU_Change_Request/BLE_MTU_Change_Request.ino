#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
uint16_t mtu = 47;  // Start with default MTU size

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }

  void onMtuChanged(BLEServer* pServer, uint16_t newMTU) {
    mtu = newMTU;
    Serial.printf("MTU size updated to: %u\n", mtu);
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("ESP32_MTU_Test");
  
  pinMode(14, INPUT);
  digitalWrite(14, HIGH);

  // Set the maximum MTU size the server is willing to accept
  BLEDevice::setMTU(50);  // Maximum theoretical BLE MTU size

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  if (deviceConnected) {
    uint8_t packet[44];

    // Fill the packet with values from 0 to 43
    for (int i = 0; i < 44; i++) {
      packet[i] = i;
    }

    // Calculate the maximum payload size (MTU - 3 for ATT header)
    uint16_t maxPayloadSize = mtu - 3;

    // Determine how many bytes we can send
    uint16_t bytesToSend = min((uint16_t)sizeof(packet), maxPayloadSize);

    pCharacteristic->setValue(packet, bytesToSend);
    pCharacteristic->notify();

    Serial.printf("Sent %d bytes: ", bytesToSend);
    for (int i = 0; i < bytesToSend; i++) {
      Serial.printf("%d ", packet[i]);
    }
    Serial.println();

    delay(100);  // Wait for 2 seconds before sending the next packet
  }
}