
#include <Arduino.h>

HardwareSerial MKS(2);

// ESP32 Dev Module UART2
constexpr int RX_PIN = 16;
constexpr int TX_PIN = 17;

constexpr uint32_t MKS_BAUD = 115200;
constexpr uint8_t MOTOR_ID = 0xE0;

int64_t zeroPosition = 0;
bool isZeroed = false;

uint32_t lastReadTime = 0;
constexpr uint32_t READ_INTERVAL_MS = 20;

bool readEncoder(int64_t &totalCounts,
                 uint16_t &rawEncoder) {

  // Clear stale incoming data
  while (MKS.available()) {
    MKS.read();
  }

  // Read encoder command
  const uint8_t command[] = {
    0xE0, 0x30, 0x10
  };

  MKS.write(command, sizeof(command));
  MKS.flush();

  uint8_t response[8];
  size_t received = 0;
  uint32_t start = millis();

  while (received < sizeof(response) &&
         millis() - start < 40) {

    if (MKS.available()) {
      response[received++] = MKS.read();
    }
  }

  if (received != 8 ||
      response[0] != MOTOR_ID) {
    return false;
  }

  // Checksum verification
  uint8_t checksum = 0;

  for (int i = 0; i < 7; i++) {
    checksum += response[i];
  }

  if (checksum != response[7]) {
    return false;
  }

  // Signed revolution counter
  uint32_t rawCarry =
      ((uint32_t)response[1] << 24) |
      ((uint32_t)response[2] << 16) |
      ((uint32_t)response[3] << 8) |
      response[4];

  int64_t carry =
      rawCarry < 0x80000000UL
          ? (int64_t)rawCarry
          : (int64_t)rawCarry - 4294967296LL;

  // Encoder position
  rawEncoder =
      ((uint16_t)response[5] << 8) |
      response[6];

  totalCounts =
      carry * 65536LL + rawEncoder;

  return true;
}

void setup() {
  Serial.begin(115200);

  MKS.begin(
    MKS_BAUD,
    SERIAL_8N1,
    RX_PIN,
    TX_PIN
  );

  delay(1000);

  Serial.println("MKS SERVO42C Angle Reader");
  Serial.println("ESP32 Dev Module");
  Serial.println("Send 'z' to reset zero.");
}

void loop() {

  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'z' || command == 'Z') {
      isZeroed = false;
      Serial.println("Zero reset requested");
    }
  }

  if (millis() - lastReadTime < READ_INTERVAL_MS) {
    return;
  }

  lastReadTime = millis();

  int64_t totalCounts;
  uint16_t encoder;

  if (readEncoder(totalCounts, encoder)) {

    if (!isZeroed) {
      zeroPosition = totalCounts;
      isZeroed = true;
    }

    double absoluteAngle =
        encoder * 360.0 / 65536.0;

    double relativeAngle =
        (double)(totalCounts - zeroPosition)
        * 360.0 / 65536.0;

    Serial.print("Absolute: ");
    Serial.print(absoluteAngle, 2);

    Serial.print(" deg | Relative: ");
    Serial.print(relativeAngle, 2);

    Serial.println(" deg");
  }
  else {
    Serial.println("Encoder read failed");
  }
}

