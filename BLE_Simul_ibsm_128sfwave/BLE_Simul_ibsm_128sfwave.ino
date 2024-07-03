#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const int BUFFER_SIZE = 128;  // Increased to hold the full ECG waveform
// int16_t ecgData[BUFFER_SIZE] = {
//   661, 822, 974, 1107, 1212, 1284, 1318, 1312, 1266, 1184, 1069, 930, 774, 612,
//   452, 305, 180, 84, 23, 0, 10, 41, 64, 51, 30, 245, 1216, 3093,
//   4095, 2814, 979, 210, 205, 373, 512, 595, 636, 653, 659, 660, 661, 661,
//   661, 661, 661, 691, 745, 799, 853, 906, 959, 1011, 1062, 1112, 1161, 1209,
//   1255, 1300, 1344, 1385, 1425, 1464, 1500, 1534, 1566, 1596, 1623, 1648, 1671, 1691,
//   1709, 1724, 1736, 1746, 1754, 1758, 1760, 1700, 1650, 1561, 1501, 1461, 1361, 1261,
//   1161, 1061, 961, 861, 761, 661, 661, 661, 661, 661, 661, 661, 661, 661,
//   661, 661, 661, 661, 661, 661, 661, 661, 661, 661, 661, 661, 661, 661,
//   661, 661, 661, 661, 661, 661, 661, 661, 661, 661, 661, 661, 661, 661,
//   661, 661
// };
//Signed waveform
int16_t ecgData[BUFFER_SIZE] = {
  -1386, -1225, -1073, -940, -835, -763, -729, -735, -781, -863, -978, -1117,
  -1273, -1435, -1595, -1742, -1867, -1963, -2024, -2048, -2037, -2006, -1983, -1996,
  -2017, -1802, -831, 1045, 2047, 766, -1068, -1837, -1842, -1674, -1535, -1452,
  -1411, -1394, -1388, -1387, -1386, -1386, -1386, -1386, -1386, -1356, -1302, -1248,
  -1194, -1141, -1088, -1036, -985, -935, -886, -838, -792, -747, -703, -662,
  -622, -583, -547, -513, -481, -451, -424, -399, -376, -356, -338, -323,
  -311, -301, -293, -289, -287, -1386, -1386, -1386, -1386, -1386, -1386, -1386,
  -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386,
  -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386,
  -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386,
  -1386, -1386, -1386, -1386, -1386, -1386, -1386, -1386
};
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
  }

  void onMtuChanged(BLEServer *pServer, uint16_t MTU) {
    currentMTU = MTU;
    Serial.printf("MTU changed to: %u\n", MTU);
  }
};

// Use the hardcoded ECG waveform
void generateECGWave() {
  writeIndex = (writeIndex + 1) % BUFFER_SIZE;
  ecgBuffer[writeIndex] = ecgData[writeIndex];
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

  // Data debug print
  Serial.print("RTC: ");
  for (int i = 0; i < 4; i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.print("ECG: ");
  for (int i = 4; i < 43; i++) {
    Serial.printf("%02X ", packet[i]);
  }
  Serial.println();

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
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

void bleTask(void *parameter) {
  for (;;) {
    if ((writeIndex - readIndex + BUFFER_SIZE) % BUFFER_SIZE >= 26) {
      sendECGPacket();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void setup() {
  // Serial.begin(115200);

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