#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// ============================================
// Controller: every input of the device in one stream.
// WeAct Black Pill (STM32F411CE).
//
//   arm_roll   MKS SERVO42C 1, serial   PA10 (RX) / PA9 (TX)
//   arm_twist  MKS SERVO42C 2, serial   PA3 (RX) / PA2 (TX)
//   arm_v      AS5600 on the rail gear  PB7 (SDA) / PB6 (SCL)
//   joy_x      joystick X               A4 (PA4)
//   joy_y      joystick Y               A5 (PA5)
//   button     switch                   B9 (PB9), to GND when pressed
//   trigger    PS5 analog trigger       A1 (PA1), 0..3.3 V ONLY
//
// REWIRE trigger signal from PB10 to PA1 before flashing. Release the trigger
// when the board boots, then squeeze it FULLY once to learn 0% and 100%.
// Before that first full squeeze, trigger_percent is nan. A reset/power cycle
// clears trigger calibration. PB10 is not an analog input on STM32F411.
//
// Output over the Black Pill's USB-C, one line about every 20 ms:
//   time_ms,arm_roll,arm_twist,arm_v,joy_x,joy_y,button,trigger,trigger_percent
//     arm_roll, arm_twist  motor shaft angle in degrees (counts whole turns)
//     arm_v                rail gear angle 0..360 degrees
//     joy_x, joy_y         -1.000 .. +1.000, 0 = middle
//     button               1 = clutch pressed, 0 = off
//     trigger              legacy digital indication, 1 if >50%, else 0
//     trigger_percent      0..100, nan until calibrated
//   "nan" = that encoder did not answer (arm_v: also no magnet seen).
//
// The same values are kept in `controller_share` in RAM, so they can also
// be read through the ST-Link (SWD) with no USB cable to the board.
//
// Build: STM32 MCU based boards -> Generic STM32F4 -> BlackPill F411CE,
// USB support "CDC (generic 'Serial' supersede U(S)ART)".
// ============================================

// ---- Pins ----------------------------------------------------------------

Uart MKS1(PA10, PA9);  // RX, TX
Uart MKS2(PA3, PA2);   // RX, TX

constexpr uint32_t SDA_PIN = PB7;
constexpr uint32_t SCL_PIN = PB6;

constexpr uint32_t JOY_X_PIN = PA4;
constexpr uint32_t JOY_Y_PIN = PA5;
constexpr uint32_t BUTTON_PIN = PB9;
constexpr uint32_t TRIGGER_PIN = PA1;  // analog-capable; signal voltage <=3.3 V

// Motor driver enable lines (active low): held high so no motor switches
// on while this program runs.
constexpr uint32_t MOTOR_ENABLE_PINS[] = { PA8, PB12, PB15 };

// ---- Settings ------------------------------------------------------------

constexpr uint32_t MKS_BAUD = 115200;
constexpr uint8_t MKS_ADDRESS = 0xE0;
constexpr uint32_t MKS_TIMEOUT_MS = 15;

constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint32_t I2C_RECOVERY_INTERVAL_MS = 1000;

constexpr uint32_t STREAM_PERIOD_MS = 20;

// A sensor that did not answer is only asked again this often, so a
// missing sensor does not slow the whole stream down.
constexpr uint32_t RETRY_MISSING_MS = 1000;
constexpr int JOY_SAMPLES = 8;  // averaged, to cut noise
constexpr uint16_t TRIGGER_DETECT_COUNTS = 300;  // ignore tiny ADC drift

constexpr float ADC_FULL_SCALE = 4095.0f;
constexpr float ADC_MIDDLE = ADC_FULL_SCALE / 2.0f;

// ---- Shared with the ST-Link reader (layout is fixed) --------------------

constexpr uint32_t SHARE_MAGIC = 0x324C5443;  // "CTL2" (adds trigger_percent)

// Bits of ControllerShare::flags.
constexpr uint32_t FLAG_BUTTON = 1u << 0;
constexpr uint32_t FLAG_TRIGGER = 1u << 1;

struct ControllerShare {
  uint32_t magic;
  uint32_t count;      // goes up by one for every new set of readings
  uint32_t time_ms;    // millis() of the newest readings
  float arm_roll;      // degrees, nan if no answer
  float arm_twist;     // degrees, nan if no answer
  float arm_v;         // degrees 0..360, nan if no answer / no magnet
  float joy_x;         // -1..+1
  float joy_y;         // -1..+1
  uint16_t joy_x_raw;  // 0..4095
  uint16_t joy_y_raw;  // 0..4095
  uint32_t flags;      // FLAG_BUTTON, FLAG_TRIGGER
  uint16_t mks_ms;     // time the last loop spent on the two drivers
  uint16_t as5600_ms;  // time the last loop spent on the AS5600
  float trigger_percent;  // 0..100, NAN until trigger calibrated; word 11
};

volatile ControllerShare controller_share __attribute__((used)) = {
  SHARE_MAGIC, 0, 0, NAN, NAN, NAN, 0.0f, 0.0f, 0, 0, 0, 0, 0, NAN
};

uint32_t lastStream = 0;
uint32_t lastI2cRecoveryMs = 0;

uint16_t triggerRest = 0;
uint16_t triggerFurthest = 0;
bool triggerStarted = false;

// Retry pacing for one sensor: ask every loop while it answers; after a
// miss, only every RETRY_MISSING_MS.
struct Pacing {
  bool answering = true;
  uint32_t lastTry = 0;

  bool due(uint32_t now) const {
    return answering || now - lastTry >= RETRY_MISSING_MS;
  }

  void record(uint32_t now, bool ok) {
    answering = ok;
    lastTry = now;
  }
};

Pacing mks1Pacing;
Pacing mks2Pacing;
Pacing as5600Pacing;

// ---- MKS SERVO42C encoder ------------------------------------------------
// Request E0 30 10; reply: address, signed 32-bit turn count, 16-bit angle
// within the turn, checksum.

bool readMKS(HardwareSerial &port, float &degrees) {
  while (port.available() > 0) {
    port.read();
  }

  const uint8_t request[] = { MKS_ADDRESS, 0x30, (uint8_t)(MKS_ADDRESS + 0x30) };
  port.write(request, sizeof(request));
  port.flush();

  uint8_t reply[8];
  size_t received = 0;
  uint32_t start = millis();

  while (received < sizeof(reply) && millis() - start < MKS_TIMEOUT_MS) {
    if (port.available()) {
      reply[received++] = port.read();
    }
  }

  if (received != sizeof(reply) || reply[0] != MKS_ADDRESS) {
    return false;
  }

  uint8_t checksum = 0;
  for (size_t i = 0; i < 7; i++) {
    checksum += reply[i];
  }
  if (checksum != reply[7]) {
    return false;
  }

  int32_t turns = (int32_t)(((uint32_t)reply[1] << 24) | ((uint32_t)reply[2] << 16) |
                            ((uint32_t)reply[3] << 8) | reply[4]);
  uint16_t within = ((uint16_t)reply[5] << 8) | reply[6];

  degrees = (float)((double)((int64_t)turns * 65536LL + within) * 360.0 / 65536.0);
  return true;
}

// ---- AS5600 --------------------------------------------------------------

// Switch on the chip's own pull-up for a pin without changing its function.
void enablePullUp(uint32_t pin) {
  PinName name = digitalPinToPinName(pin);
  LL_GPIO_SetPinPull(get_GPIO_Port(STM_PORT(name)), STM_LL_GPIO_PIN(name), LL_GPIO_PULL_UP);
}

void beginI2c() {
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  enablePullUp(SDA_PIN);
  enablePullUp(SCL_PIN);
}

// Free a bus the sensor is holding low: clock SCL until it lets go of
// SDA, send a STOP, restart the I2C peripheral.
void recoverI2c() {
  Wire.end();

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(50);

  for (int i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(50);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(50);
  }

  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(50);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(50);
  pinMode(SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(50);

  beginI2c();
  lastI2cRecoveryMs = millis();
}

bool readAS5600Registers(uint8_t firstRegister, uint8_t *values, uint8_t count) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(firstRegister);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(AS5600_ADDR, count) != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    values[i] = (uint8_t)Wire.read();
  }
  return true;
}

// false with answered = true: the sensor answered but sees no magnet.
bool tryReadAS5600(float &degrees, bool &answered) {
  uint8_t status = 0;
  uint8_t angle[2] = { 0, 0 };

  answered = readAS5600Registers(0x0B, &status, 1) &&
             readAS5600Registers(0x0C, angle, sizeof(angle));

  if (!answered || (status & 0x20) == 0) {  // bit 5: magnet detected
    return false;
  }

  uint16_t raw = (((uint16_t)angle[0] << 8) | angle[1]) & 0x0FFF;
  degrees = raw * 360.0f / 4096.0f;
  return true;
}

bool readAS5600(float &degrees) {
  bool answered = false;

  if (tryReadAS5600(degrees, answered)) {
    return true;
  }

  // No answer at all: free the bus and try once more, at most once a second.
  if (!answered && millis() - lastI2cRecoveryMs > I2C_RECOVERY_INTERVAL_MS) {
    recoverI2c();
    return tryReadAS5600(degrees, answered);
  }

  return false;
}

// ---- Joystick, button, trigger -------------------------------------------

uint16_t readAveraged(uint32_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < JOY_SAMPLES; i++) {
    sum += analogRead(pin);
  }
  return (uint16_t)(sum / JOY_SAMPLES);
}

// 0..4095 -> -1.0..+1.0 with 0 at the middle.
float centred(uint16_t raw) {
  float value = (raw - ADC_MIDDLE) / ADC_MIDDLE;
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

// Return NAN until the analog signal has travelled far enough to calibrate.
// Unlike the 0/1 field this retains the proportion of trigger travel.
float readTriggerPercent(uint16_t raw) {
  if (!triggerStarted) {
    triggerRest = raw;
    triggerFurthest = raw;
    triggerStarted = true;
  }
  if (abs((int)raw - (int)triggerRest) >
      abs((int)triggerFurthest - (int)triggerRest)) {
    triggerFurthest = raw;
  }
  int span = (int)triggerFurthest - (int)triggerRest;
  if (abs(span) < TRIGGER_DETECT_COUNTS) return NAN;
  float value = 100.0f * ((int)raw - (int)triggerRest) / span;
  if (value < 0.0f) return 0.0f;
  if (value > 100.0f) return 100.0f;
  return value;
}

// ---- Output ----------------------------------------------------------------

void printValue(float value) {
  Serial.print(",");
  if (isnan(value)) {
    Serial.print("nan");
  } else {
    Serial.print(value, 3);
  }
}

void holdMotorsOff() {
  for (uint32_t pin : MOTOR_ENABLE_PINS) {
    // digitalWrite before pinMode is lost on STM32: pull up first.
    pinMode(pin, INPUT_PULLUP);
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
}

void setup() {
  holdMotorsOff();

  analogReadResolution(12);
  pinMode(JOY_X_PIN, INPUT_ANALOG);
  pinMode(JOY_Y_PIN, INPUT_ANALOG);
  pinMode(BUTTON_PIN, INPUT_PULLUP);     // pressed pulls it to GND
  pinMode(TRIGGER_PIN, INPUT_ANALOG);  // 12-bit proportional PS5 trigger

  Serial.begin(115200);  // USB CDC: the baud rate is ignored

  MKS1.begin(MKS_BAUD, SERIAL_8N1);
  MKS2.begin(MKS_BAUD, SERIAL_8N1);

  beginI2c();
}

void loop() {
  uint32_t now = millis();

  if (now - lastStream < STREAM_PERIOD_MS) {
    return;
  }
  lastStream = now;

  float roll = NAN;
  float twist = NAN;
  float rail = NAN;

  uint32_t mksStart = millis();

  if (mks1Pacing.due(now)) {
    bool ok = readMKS(MKS1, roll);
    mks1Pacing.record(now, ok);
    if (!ok) roll = NAN;
  }

  if (mks2Pacing.due(now)) {
    bool ok = readMKS(MKS2, twist);
    mks2Pacing.record(now, ok);
    if (!ok) twist = NAN;
  }

  uint32_t as5600Start = millis();

  if (as5600Pacing.due(now)) {
    bool ok = readAS5600(rail);
    as5600Pacing.record(now, ok);
    if (!ok) rail = NAN;
  }

  controller_share.mks_ms = (uint16_t)(as5600Start - mksStart);
  controller_share.as5600_ms = (uint16_t)(millis() - as5600Start);

  uint16_t xRaw = readAveraged(JOY_X_PIN);
  uint16_t yRaw = readAveraged(JOY_Y_PIN);
  float x = centred(xRaw);
  float y = centred(yRaw);

  bool button = digitalRead(BUTTON_PIN) == LOW;
  uint16_t triggerRaw = readAveraged(TRIGGER_PIN);
  float triggerPercent = readTriggerPercent(triggerRaw);
  bool trigger = isfinite(triggerPercent) && triggerPercent >= 50.0f;

  controller_share.arm_roll = roll;
  controller_share.arm_twist = twist;
  controller_share.arm_v = rail;
  controller_share.joy_x = x;
  controller_share.joy_y = y;
  controller_share.joy_x_raw = xRaw;
  controller_share.joy_y_raw = yRaw;
  controller_share.flags = (button ? FLAG_BUTTON : 0u) | (trigger ? FLAG_TRIGGER : 0u);
  controller_share.trigger_percent = triggerPercent;
  controller_share.time_ms = now;
  controller_share.count = controller_share.count + 1;

  if (!Serial) {
    return;
  }

  static bool headerSent = false;
  if (!headerSent) {
    Serial.println("time_ms,arm_roll,arm_twist,arm_v,joy_x,joy_y,button,trigger,trigger_percent");
    headerSent = true;
  }

  Serial.print(now);
  printValue(roll);
  printValue(twist);
  printValue(rail);
  printValue(x);
  printValue(y);
  Serial.print(",");
  Serial.print(button ? 1 : 0);
  Serial.print(",");
  Serial.print(trigger ? 1 : 0);
  printValue(triggerPercent);
  Serial.println();
}
