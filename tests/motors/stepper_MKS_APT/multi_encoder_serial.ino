
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// =====================================
// Seeed XIAO ESP32-S3 Plus
// =====================================

// MKS #1: D7 RX, D6 TX
HardwareSerial MKS1(1);
constexpr int MKS1_RX = 44;
constexpr int MKS1_TX = 43;

// MKS #2: D1 RX, D0 TX
HardwareSerial MKS2(2);
constexpr int MKS2_RX = 2;
constexpr int MKS2_TX = 1;

// White magnetic encoder (AS5600)
constexpr int I2C_SDA = 5;  // D4
constexpr int I2C_SCL = 6;  // D5;
constexpr uint8_t AS5600_ADDR = 0x36;

// Communication settings
constexpr uint32_t MKS_BAUD = 115200;
constexpr uint8_t MKS_ADDRESS = 0xE0;
constexpr uint32_t STREAM_PERIOD_MS = 20;

uint32_t lastStream = 0;

// =====================================
// Read MKS SERVO42C encoder
// 8-byte response:
// [address][signed carry x4]
// [encoder high][encoder low][checksum]
// =====================================

bool readMKS(HardwareSerial &port, double &angle) {

  // Remove stale bytes
  while (port.available() > 0) {
    port.read();
  }

  const uint8_t request[] = {
    0xE0, 0x30, 0x10
  };

  port.write(request, sizeof(request));
  port.flush();

  uint8_t response[8];
  size_t received = 0;
  uint32_t start = millis();

  while (received < sizeof(response) &&
         millis() - start < 15) {
    if (port.available()) {
      response[received++] = port.read();
    }
  }

  if (received != 8) return false;

  if (response[0] != MKS_ADDRESS) {
    return false;
  }

  // Validate checksum
  uint8_t checksum = 0;

  for (int i = 0; i < 7; i++) {
    checksum += response[i];
  }

  if (checksum != response[7]) {
    return false;
  }

  // Signed 32-bit rotation counter
  uint32_t rawCarry =
      ((uint32_t)response[1] << 24) |
      ((uint32_t)response[2] << 16) |
      ((uint32_t)response[3] << 8) |
      response[4];

  int64_t carry =
      rawCarry < 0x80000000UL
          ? (int64_t)rawCarry
          : (int64_t)rawCarry - 4294967296LL;

  uint16_t encoder =
      ((uint16_t)response[5] << 8) |
      response[6];

  int64_t totalCounts =
      carry * 65536LL + encoder;

  angle =
      (double)totalCounts * 360.0 / 65536.0;

  return true;
}

// =====================================
// Read AS5600 angle via I2C
// =====================================

bool readAS5600(double &angle) {

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);  // Raw angle register

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2)
      != 2) {
    return false;
  }

  uint16_t raw =
      ((uint16_t)Wire.read() << 8) |
      Wire.read();

  raw &= 0x0FFF;

  angle = (double)raw * 360.0 / 4096.0;

  return true;
}

// =====================================
// Setup
// =====================================

void setup() {

  Serial.begin(115200);

  MKS1.begin(
    MKS_BAUD,
    SERIAL_8N1,
    MKS1_RX,
    MKS1_TX
  );

  MKS2.begin(
    MKS_BAUD,
    SERIAL_8N1,
    MKS2_RX,
    MKS2_TX
  );

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(10);

  delay(1000);

  Serial.println("time_ms,mks1_deg,mks2_deg,as5600_deg");
}

// =====================================
// Main loop
// =====================================

void loop() {

  if (millis() - lastStream < STREAM_PERIOD_MS) {
    return;
  }

  lastStream = millis();

  double angle1 = NAN;
  double angle2 = NAN;
  double angle3 = NAN;

  bool ok1 = readMKS(MKS1, angle1);
  bool ok2 = readMKS(MKS2, angle2);
  bool ok3 = readAS5600(angle3);

  Serial.print(millis());
  Serial.print(",");

  if (ok1) Serial.print(angle1, 3);
  else Serial.print("nan");

  Serial.print(",");

  if (ok2) Serial.print(angle2, 3);
  else Serial.print("nan");

  Serial.print(",");

  if (ok3) Serial.println(angle3, 3);
  else Serial.println("nan");
}
