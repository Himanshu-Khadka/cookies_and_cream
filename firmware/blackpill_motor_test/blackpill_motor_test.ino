#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

// ============================================
// WeAct STM32 Black Pill (F411CE / F401)
// Motor test firmware for two MKS SERVO42C drivers.
//
// Type commands over the USB serial port (or use
// motor_test.py, which does it for you):
//
//   1 200      move motor 1 by 200 steps (DIR high)
//   2 -400     move motor 2 by 400 steps (DIR low)
//   speed 300  set the step rate in steps per second
//   hold 1     switch motor 1 on and hold it (60 s max)
//   r          read all three encoders once
//   x          release (switch off) both motors
//   help       list the commands
//
// Sending anything while a move is running stops it.
// Every reply ends with a line starting "ok" or "error".
//
// Arduino IDE (STM32 MCU based boards, core 3.x):
//   Board:             Generic STM32F4 series
//   Board part number: BlackPill F411CE
//   USB support:       CDC (generic 'Serial' supersede U(S)ART)
//   Upload method:     STM32CubeProgrammer (SWD)  -> ST-Link
// ============================================

// Motor 1 (wrist roll) on USART1
Uart MKS1(PA10, PA9);            // RX, TX

// Motor 2 (wrist twist) on USART2
Uart MKS2(PA3, PA2);             // RX, TX

// White AS5600 on I2C1
constexpr uint32_t SDA_PIN = PB7;
constexpr uint32_t SCL_PIN = PB6;
constexpr uint8_t AS5600_ADDR = 0x36;

// ============================================
// CONFIGURATION
// ============================================

// MKS setting En = L (active LOW)
constexpr bool ENABLE_ACTIVE_LOW = true;

constexpr uint32_t UART_BAUD = 115200;
constexpr uint8_t MKS_ADDRESS = 0xE0;

// Largest single move, so a typo can't spin a joint
// for minutes.
constexpr long MAX_STEPS_PER_MOVE = 20000;

// Step rate limits (steps per second)
constexpr long MIN_STEPS_PER_SEC = 20;
constexpr long MAX_STEPS_PER_SEC = 2000;
constexpr long DEFAULT_STEPS_PER_SEC = 300;

// The driver ignores step pulses for a moment after
// EN goes active.
constexpr uint32_t ENABLE_SETTLE_MS = 500;

// Let the motor finish moving before reading it.
constexpr uint32_t MOVE_SETTLE_MS = 300;

// "hold" switches itself off after this long.
constexpr uint32_t HOLD_TIMEOUT_MS = 60000;

constexpr size_t MAX_LINE_LENGTH = 32;

// ============================================
// MOTORS
// ============================================

struct Motor {
  const char* name;
  HardwareSerial* port;
  uint32_t enPin;
  uint32_t stepPin;
  uint32_t dirPin;
  bool enabled;
};

// Driver work modes (serial command 0x82). Defined
// up here so it is declared before any function.
enum class WorkMode : uint8_t {
  OpenLoop = 0x00,  // CR_OPEN: plain stepper, encoder not used for driving
  ClosedLoop = 0x01 // CR_vFOC: closed loop using the encoder
};

// Board pins the probe is allowed to drive. Serial,
// I2C, USB, debug, LED, button and boot pins are
// left out.
struct ProbePin {
  uint32_t pin;
  const char* name;
};

const ProbePin PROBE_PINS[] = {
  { PB12, "PB12" }, { PB13, "PB13" }, { PB14, "PB14" }, { PB15, "PB15" },
  { PA8,  "PA8"  }, { PA15, "PA15" }, { PB3,  "PB3"  }, { PB4,  "PB4"  },
  { PB5,  "PB5"  }, { PB8,  "PB8"  }, { PB9,  "PB9"  }, { PB10, "PB10" },
  { PB1,  "PB1"  }, { PB0,  "PB0"  }, { PA7,  "PA7"  }, { PA6,  "PA6"  },
  { PA5,  "PA5"  }, { PA4,  "PA4"  }, { PA1,  "PA1"  },
};

constexpr size_t PROBE_PIN_COUNT = sizeof(PROBE_PINS) / sizeof(PROBE_PINS[0]);
constexpr int PROBE_STEP_PULSES = 50;
constexpr int PROBE_DIR_PULSES = 20;

constexpr size_t MOTOR_COUNT = 2;

Motor motors[MOTOR_COUNT] = {
  // Motor 1 enable moved from PB12 to PA8 (the pin next to
  // PB15): the driver never saw the signal on PB12.
  { "motor 1", &MKS1, PA8,  PB13, PB14, false },
  { "motor 2", &MKS2, PB15, PB0,  PB1,  false },
};

long stepsPerSec = DEFAULT_STEPS_PER_SEC;
uint32_t holdStartedMs = 0;

// ============================================
// ENABLE / DISABLE
// ============================================

void setEnabled(Motor &motor, bool enabled) {
  bool level = ENABLE_ACTIVE_LOW ? !enabled : enabled;
  digitalWrite(motor.enPin, level ? HIGH : LOW);
  motor.enabled = enabled;
}

void releaseAll() {
  for (Motor &motor : motors) {
    setEnabled(motor, false);
  }
}

bool anyEnabled() {
  for (const Motor &motor : motors) {
    if (motor.enabled) return true;
  }
  return false;
}

// Make an EN pin an output that starts at the
// "disabled" level. On STM32, digitalWrite before
// pinMode is lost, so: pull the pin to the disabled
// level as an input, set the latch, then switch to
// output.
void initEnablePin(uint32_t pin) {
  bool disabledLevel = ENABLE_ACTIVE_LOW;
  pinMode(pin, disabledLevel ? INPUT_PULLUP : INPUT_PULLDOWN);
  digitalWrite(pin, disabledLevel ? HIGH : LOW);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, disabledLevel ? HIGH : LOW);
}

// ============================================
// MKS DRIVER SERIAL QUERIES
// ============================================

// Send a command (address and checksum are added
// here) and read a fixed-length, checksummed reply.
// False if the driver does not answer in time or
// the reply is damaged.
bool mksRequestBytes(Motor &motor,
                     const uint8_t* payload, size_t payloadLen,
                     uint8_t* reply, size_t replyLen,
                     uint32_t timeoutMs) {
  HardwareSerial &port = *motor.port;

  while (port.available()) {
    port.read();
  }

  uint8_t requestSum = MKS_ADDRESS;
  port.write(MKS_ADDRESS);

  for (size_t i = 0; i < payloadLen; i++) {
    port.write(payload[i]);
    requestSum += payload[i];
  }

  port.write(requestSum);
  port.flush();

  size_t count = 0;
  uint32_t start = millis();

  while (count < replyLen && millis() - start < timeoutMs) {
    if (port.available()) {
      reply[count++] = port.read();
    }
  }

  if (count != replyLen || reply[0] != MKS_ADDRESS) {
    return false;
  }

  uint8_t checksum = 0;
  for (size_t i = 0; i + 1 < replyLen; i++) {
    checksum += reply[i];
  }

  return checksum == reply[replyLen - 1];
}

// One-byte command, quick reply.
bool mksRequest(Motor &motor, uint8_t command,
                uint8_t* reply, size_t replyLen) {
  return mksRequestBytes(motor, &command, 1, reply, replyLen, 30);
}

// Change the driver's work mode. The setting is saved
// in the driver. False if the driver does not answer;
// "accepted" is the driver's own success flag.
bool setWorkMode(Motor &motor, WorkMode mode, bool &accepted) {
  const uint8_t payload[] = { 0x82, (uint8_t)mode };
  uint8_t reply[3];

  if (!mksRequestBytes(motor, payload, sizeof(payload),
                       reply, sizeof(reply), 500)) {
    return false;
  }

  accepted = (reply[1] == 1);
  return true;
}

int32_t toInt32(const uint8_t* bytes) {
  uint32_t raw =
      ((uint32_t)bytes[0] << 24) |
      ((uint32_t)bytes[1] << 16) |
      ((uint32_t)bytes[2] << 8) |
      bytes[3];

  return (int32_t)raw;
}

// Encoder angle in degrees, counting whole turns.
bool readEncoder(Motor &motor, double &degrees) {
  uint8_t reply[8];

  if (!mksRequest(motor, 0x30, reply, sizeof(reply))) {
    return false;
  }

  int64_t turns = toInt32(&reply[1]);
  uint16_t within = ((uint16_t)reply[5] << 8) | reply[6];
  int64_t counts = turns * 65536LL + within;

  degrees = (double)counts * 360.0 / 65536.0;
  return true;
}

// Step pulses the driver has counted so far.
bool readPulseCount(Motor &motor, int32_t &pulses) {
  uint8_t reply[6];

  if (!mksRequest(motor, 0x33, reply, sizeof(reply))) {
    return false;
  }

  pulses = toInt32(&reply[1]);
  return true;
}

// The driver's own "motor is blocked" flag.
bool readStalled(Motor &motor, bool &stalled) {
  uint8_t reply[3];

  if (!mksRequest(motor, 0x3E, reply, sizeof(reply))) {
    return false;
  }

  stalled = (reply[1] == 1);
  return true;
}

// What the driver itself sees on its EN input.
bool readDriverEnabled(Motor &motor, bool &enabled) {
  uint8_t reply[3];

  if (!mksRequest(motor, 0x3A, reply, sizeof(reply))) {
    return false;
  }

  enabled = (reply[1] == 1);
  return true;
}

// Release the driver's stall protection. Once the
// driver decides the motor is blocked it stops
// powering it, and toggling EN does not undo that.
// False if the driver does not answer; "released"
// is the driver's own success flag.
bool clearStallProtection(Motor &motor, bool &released) {
  uint8_t reply[3];

  if (!mksRequest(motor, 0x3D, reply, sizeof(reply))) {
    return false;
  }

  released = (reply[1] == 1);
  return true;
}

// "scan <1|2>": ask for the encoder value at every
// baud rate and address the driver can be set to,
// and print anything that comes back. Finds a driver
// whose serial settings no longer match UART_BAUD
// and MKS_ADDRESS (for example after a "Restore").
void scanDriver(Motor &motor) {
  HardwareSerial &port = *motor.port;

  const uint32_t bauds[] = {
    9600, 19200, 25000, 38400, 57600, 115200
  };

  bool heardAnything = false;

  for (uint32_t baud : bauds) {
    port.end();
    port.begin(baud, SERIAL_8N1);
    delay(20);

    for (uint8_t address = 0xE0; address <= 0xE9; address++) {
      while (port.available()) {
        port.read();
      }

      const uint8_t request[] = {
        address, 0x30, (uint8_t)(address + 0x30)
      };

      port.write(request, sizeof(request));
      port.flush();

      uint8_t reply[12];
      size_t count = 0;
      uint32_t start = millis();

      while (millis() - start < 40 && count < sizeof(reply)) {
        if (port.available()) {
          reply[count++] = port.read();
        }
      }

      if (count == 0) {
        continue;
      }

      heardAnything = true;

      uint8_t checksum = 0;
      for (size_t i = 0; i + 1 < count; i++) {
        checksum += reply[i];
      }

      bool valid = (count == 8 && reply[0] == address &&
                    checksum == reply[count - 1]);

      Serial.print("baud ");
      Serial.print(baud);
      Serial.print(", address ");
      Serial.print(address, HEX);
      Serial.print(": ");

      for (size_t i = 0; i < count; i++) {
        if (reply[i] < 0x10) Serial.print("0");
        Serial.print(reply[i], HEX);
        Serial.print(" ");
      }

      Serial.println(valid ? "<- valid encoder reply" : "(not a valid reply)");
    }
  }

  port.end();
  port.begin(UART_BAUD, SERIAL_8N1);

  if (!heardAnything) {
    Serial.println("nothing came back at any baud rate or address");
  }

  Serial.println("ok: scan done");
}

// ============================================
// AS5600
// ============================================

bool readAS5600(double &degrees) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) {
    return false;
  }

  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  raw &= 0x0FFF;

  degrees = raw * 360.0 / 4096.0;
  return true;
}

// ============================================
// COMMANDS
// ============================================

void printEncoders() {
  for (Motor &motor : motors) {
    double degrees;

    Serial.print(motor.name);
    Serial.print(" encoder: ");

    if (readEncoder(motor, degrees)) {
      Serial.print(degrees, 3);
      Serial.println(" deg");
    } else {
      Serial.println("no reply from driver");
    }
  }

  double degrees;
  Serial.print("AS5600: ");

  if (readAS5600(degrees)) {
    Serial.print(degrees, 3);
    Serial.println(" deg");
  } else {
    Serial.println("no reply");
  }
}

// ============================================
// PIN PROBE
// ============================================

const char* pinName(uint32_t pin) {
  for (const ProbePin &probe : PROBE_PINS) {
    if (probe.pin == pin) return probe.name;
  }
  return "?";
}

// Put every pin back the way setup() leaves it.
void restorePins() {
  for (const ProbePin &probe : PROBE_PINS) {
    pinMode(probe.pin, INPUT);
  }

  for (Motor &motor : motors) {
    initEnablePin(motor.enPin);
    pinMode(motor.stepPin, OUTPUT);
    pinMode(motor.dirPin, OUTPUT);
    digitalWrite(motor.stepPin, LOW);
    motor.enabled = false;
  }
}

void probePulses(uint32_t pin, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, LOW);
    delayMicroseconds(1000);
    digitalWrite(pin, HIGH);
    delayMicroseconds(1000);
  }
}

void printFound(const char* what, long found, uint32_t expectedPin) {
  Serial.print(what);
  Serial.print(": ");

  if (found < 0) {
    Serial.print("NOT FOUND on any pin (firmware uses ");
    Serial.print(pinName(expectedPin));
    Serial.println(")");
    return;
  }

  Serial.print(PROBE_PINS[found].name);
  Serial.print(PROBE_PINS[found].pin == expectedPin
    ? "  (matches the firmware)"
    : "  (firmware uses ");

  if (PROBE_PINS[found].pin != expectedPin) {
    Serial.print(pinName(expectedPin));
    Serial.print(": MISMATCH)");
  }

  Serial.println();
}

// "probe <1|2>": find which board pins this driver's
// EN, STEP and DIR inputs are really wired to, by
// driving each pin and asking the driver what it saw.
void probeMotor(Motor &motor) {
  bool baseline = false;

  if (!readDriverEnabled(motor, baseline)) {
    Serial.println("error: driver is not answering on serial, cannot probe");
    return;
  }

  // Everything idle high (EN inactive for En = L).
  for (const ProbePin &probe : PROBE_PINS) {
    pinMode(probe.pin, OUTPUT);
    digitalWrite(probe.pin, HIGH);
  }

  delay(100);

  if (!readDriverEnabled(motor, baseline)) {
    restorePins();
    Serial.println("error: driver stopped answering during the probe");
    return;
  }

  Serial.print("with every pin high the driver's enable input reads ");
  Serial.println(baseline ? "ON" : "OFF");

  // --- EN: which pin, pulled low, changes the driver's enable input?
  long enIndex = -1;

  for (size_t i = 0; i < PROBE_PIN_COUNT; i++) {
    digitalWrite(PROBE_PINS[i].pin, LOW);
    delay(60);

    bool seen = baseline;
    bool answered = readDriverEnabled(motor, seen);

    digitalWrite(PROBE_PINS[i].pin, HIGH);
    delay(20);

    if (answered && seen != baseline && enIndex < 0) {
      enIndex = (long)i;
    }
  }

  printFound("enable input is on", enIndex, motor.enPin);

  if (enIndex < 0 && !baseline) {
    restorePins();
    Serial.println("driver never switches on, so step and direction cannot be tested");
    Serial.println("ok: probe done");
    return;
  }

  // --- STEP: with the driver on, which pin makes it count pulses?
  if (enIndex >= 0) {
    digitalWrite(PROBE_PINS[enIndex].pin, LOW);
  }

  delay(ENABLE_SETTLE_MS);

  long stepIndex = -1;

  for (size_t i = 0; i < PROBE_PIN_COUNT; i++) {
    if ((long)i == enIndex) continue;

    int32_t before = 0;
    int32_t after = 0;

    if (!readPulseCount(motor, before)) continue;
    probePulses(PROBE_PINS[i].pin, PROBE_STEP_PULSES);
    if (!readPulseCount(motor, after)) continue;

    if (after != before && stepIndex < 0) {
      stepIndex = (long)i;
    }
  }

  printFound("step input is on", stepIndex, motor.stepPin);

  // --- DIR: which pin flips the sign of the count?
  long dirIndex = -1;

  if (stepIndex >= 0) {
    for (size_t i = 0; i < PROBE_PIN_COUNT; i++) {
      if ((long)i == enIndex || (long)i == stepIndex) continue;

      int32_t start = 0;
      int32_t afterLow = 0;
      int32_t afterHigh = 0;

      if (!readPulseCount(motor, start)) continue;

      digitalWrite(PROBE_PINS[i].pin, LOW);
      delay(2);
      probePulses(PROBE_PINS[stepIndex].pin, PROBE_DIR_PULSES);
      if (!readPulseCount(motor, afterLow)) continue;

      digitalWrite(PROBE_PINS[i].pin, HIGH);
      delay(2);
      probePulses(PROBE_PINS[stepIndex].pin, PROBE_DIR_PULSES);
      if (!readPulseCount(motor, afterHigh)) continue;

      long deltaLow = (long)afterLow - (long)start;
      long deltaHigh = (long)afterHigh - (long)afterLow;

      if (deltaLow != 0 && deltaHigh != 0 &&
          (deltaLow > 0) != (deltaHigh > 0) && dirIndex < 0) {
        dirIndex = (long)i;
      }
    }

    printFound("direction input is on", dirIndex, motor.dirPin);
  }

  restorePins();
  Serial.println("ok: probe done");
}

// "status <1|2>": compare what the board is asking
// for with what the driver reports.
void printStatus(Motor &motor) {
  bool driverEnabled = false;
  int32_t pulses = 0;
  bool stalled = false;
  double degrees = NAN;

  Serial.print(motor.name);
  Serial.print(": board is asking for ");
  Serial.println(motor.enabled ? "ON" : "OFF");

  Serial.print("driver sees its enable input as: ");
  if (readDriverEnabled(motor, driverEnabled)) {
    Serial.println(driverEnabled ? "ON" : "OFF");
  } else {
    Serial.println("no reply");
  }

  Serial.print("pulses counted since enabled: ");
  if (readPulseCount(motor, pulses)) {
    Serial.println(pulses);
  } else {
    Serial.println("no reply");
  }

  Serial.print("stall flag: ");
  if (readStalled(motor, stalled)) {
    Serial.println(stalled ? "BLOCKED" : "clear");
  } else {
    Serial.println("no reply");
  }

  Serial.print("encoder: ");
  if (readEncoder(motor, degrees)) {
    Serial.print(degrees, 3);
    Serial.println(" deg");
  } else {
    Serial.println("no reply");
  }
}

void printHelp() {
  Serial.println("1 <steps>    move motor 1 (negative = other way)");
  Serial.println("2 <steps>    move motor 2");
  Serial.println("speed <n>    step rate, steps per second");
  Serial.println("hold <1|2>   switch a motor on and hold it");
  Serial.println("clear <1|2>  release the driver's stall protection");
  Serial.println("mode <1|2> <open|closed>  set the driver's work mode");
  Serial.println("scan <1|2>   look for the driver at every baud/address");
  Serial.println("status <1|2> what the driver reports about itself");
  Serial.println("probe <1|2>  find which board pins EN/STEP/DIR are wired to");
  Serial.println("r            read all three encoders");
  Serial.println("x            release both motors");
}

// Step a motor and report what the driver saw.
// Positive steps use DIR high, negative DIR low.
void moveMotor(Motor &motor, long steps) {
  double before = NAN;
  double after = NAN;
  int32_t pulsesBefore = 0;
  int32_t pulsesAfter = 0;
  bool stalled = false;

  bool wasEnabled = motor.enabled;
  setEnabled(motor, true);

  if (!wasEnabled) {
    delay(ENABLE_SETTLE_MS);
  }

  // Read the starting values only after enabling: the
  // driver restarts its pulse counter at zero each time
  // EN goes active, so a reading taken before that is
  // left over from the previous move.
  bool haveBefore = readEncoder(motor, before);
  bool havePulsesBefore = readPulseCount(motor, pulsesBefore);

  digitalWrite(motor.dirPin, steps > 0 ? HIGH : LOW);
  delayMicroseconds(100);

  const uint32_t halfPeriodUs = (uint32_t)(500000L / stepsPerSec);
  const long total = labs(steps);
  long sent = 0;
  bool stopped = false;

  for (long i = 0; i < total; i++) {
    // Anything typed during a move stops it.
    if (Serial.available()) {
      stopped = true;
      break;
    }

    digitalWrite(motor.stepPin, HIGH);
    delayMicroseconds(halfPeriodUs);
    digitalWrite(motor.stepPin, LOW);
    delayMicroseconds(halfPeriodUs);
    sent++;
  }

  delay(MOVE_SETTLE_MS);

  bool haveAfter = readEncoder(motor, after);
  bool havePulsesAfter = readPulseCount(motor, pulsesAfter);
  bool haveStalled = readStalled(motor, stalled);

  // Leave a held motor on; otherwise switch it off.
  if (!wasEnabled) {
    setEnabled(motor, false);
  }

  Serial.print(motor.name);
  Serial.print(": sent ");
  Serial.print(sent);
  Serial.print(" of ");
  Serial.print(total);
  Serial.print(steps > 0 ? " steps (DIR high) at " : " steps (DIR low) at ");
  Serial.print(stepsPerSec);
  Serial.println(" steps/s");

  if (stopped) {
    Serial.println("stopped early: input received during the move");
  }

  Serial.print("encoder: ");
  if (haveBefore && haveAfter) {
    Serial.print(before, 3);
    Serial.print(" -> ");
    Serial.print(after, 3);
    Serial.print(" deg (moved ");
    Serial.print(after - before, 3);
    Serial.println(" deg)");
  } else {
    Serial.println("no reply from driver");
  }

  Serial.print("driver: ");
  if (havePulsesBefore && havePulsesAfter && haveStalled) {
    Serial.print("counted ");
    Serial.print(pulsesAfter - pulsesBefore);
    Serial.print(" pulses, ");
    Serial.println(stalled ? "motor BLOCKED (stalled)" : "not stalled");
  } else {
    Serial.println("no reply from driver");
  }

  Serial.println(stopped ? "ok: move stopped" : "ok: move done");
}

// Whole string must be a number. False otherwise.
bool parseLong(const String &text, long &value) {
  if (text.length() == 0) {
    return false;
  }

  char* end = nullptr;
  value = strtol(text.c_str(), &end, 10);

  return end != nullptr && *end == '\0';
}

// "1" or "2" -> index 0 or 1. False otherwise.
bool parseMotor(const String &text, size_t &index) {
  if (text == "1") {
    index = 0;
    return true;
  }

  if (text == "2") {
    index = 1;
    return true;
  }

  return false;
}

void handleLine(String line) {
  line.trim();
  line.toLowerCase();

  if (line.length() == 0) {
    return;
  }

  if (line == "x") {
    releaseAll();
    Serial.println("ok: both motors released");
    return;
  }

  if (line == "r") {
    printEncoders();
    Serial.println("ok");
    return;
  }

  if (line == "help" || line == "?") {
    printHelp();
    Serial.println("ok");
    return;
  }

  int space = line.indexOf(' ');
  String head = space < 0 ? line : line.substring(0, space);
  String arg = space < 0 ? String("") : line.substring(space + 1);
  arg.trim();

  long value = 0;
  size_t index = 0;

  if (head == "speed") {
    if (!parseLong(arg, value) ||
        value < MIN_STEPS_PER_SEC || value > MAX_STEPS_PER_SEC) {
      Serial.print("error: speed must be ");
      Serial.print(MIN_STEPS_PER_SEC);
      Serial.print(" to ");
      Serial.println(MAX_STEPS_PER_SEC);
      return;
    }

    stepsPerSec = value;
    Serial.print("ok: speed ");
    Serial.print(stepsPerSec);
    Serial.println(" steps/s");
    return;
  }

  if (head == "hold") {
    if (!parseMotor(arg, index)) {
      Serial.println("error: use hold 1 or hold 2");
      return;
    }

    setEnabled(motors[index], true);
    holdStartedMs = millis();
    Serial.print("ok: ");
    Serial.print(motors[index].name);
    Serial.println(" on and holding (x to release, 60 s max)");
    return;
  }

  if (head == "probe") {
    if (!parseMotor(arg, index)) {
      Serial.println("error: use probe 1 or probe 2");
      return;
    }

    probeMotor(motors[index]);
    return;
  }

  if (head == "status") {
    if (!parseMotor(arg, index)) {
      Serial.println("error: use status 1 or status 2");
      return;
    }

    printStatus(motors[index]);
    Serial.println("ok");
    return;
  }

  if (head == "scan") {
    if (!parseMotor(arg, index)) {
      Serial.println("error: use scan 1 or scan 2");
      return;
    }

    scanDriver(motors[index]);
    return;
  }

  // mode <1|2> <open|closed>
  if (head == "mode") {
    int split = arg.indexOf(' ');
    String motorText = split < 0 ? arg : arg.substring(0, split);
    String modeText = split < 0 ? String("") : arg.substring(split + 1);
    modeText.trim();

    bool isOpen = (modeText == "open");
    bool isClosed = (modeText == "closed");

    if (!parseMotor(motorText, index) || (!isOpen && !isClosed)) {
      Serial.println("error: use mode <1|2> <open|closed>");
      return;
    }

    setEnabled(motors[index], false);

    bool accepted = false;
    WorkMode mode = isOpen ? WorkMode::OpenLoop : WorkMode::ClosedLoop;

    if (!setWorkMode(motors[index], mode, accepted)) {
      Serial.println("error: no reply from driver");
      return;
    }

    if (!accepted) {
      Serial.println("error: driver refused the mode change");
      return;
    }

    Serial.print("ok: ");
    Serial.print(motors[index].name);
    Serial.println(isOpen ? " set to open loop (CR_OPEN)"
                          : " set to closed loop (CR_vFOC)");
    return;
  }

  if (head == "clear") {
    if (!parseMotor(arg, index)) {
      Serial.println("error: use clear 1 or clear 2");
      return;
    }

    bool released = false;

    if (!clearStallProtection(motors[index], released)) {
      Serial.println("error: no reply from driver");
      return;
    }

    Serial.print("ok: ");
    Serial.print(motors[index].name);
    Serial.println(released
      ? " stall protection released"
      : " driver answered: nothing to release");
    return;
  }

  if (parseMotor(head, index)) {
    if (!parseLong(arg, value) || value == 0 ||
        labs(value) > MAX_STEPS_PER_MOVE) {
      Serial.print("error: steps must be a non-zero number up to ");
      Serial.println(MAX_STEPS_PER_MOVE);
      return;
    }

    moveMotor(motors[index], value);
    holdStartedMs = millis();
    return;
  }

  Serial.println("error: unknown command (type help)");
}

// ============================================
// SETUP / LOOP
// ============================================

void setup() {
  // EN pins first, so the drivers stay off.
  for (Motor &motor : motors) {
    initEnablePin(motor.enPin);
    pinMode(motor.stepPin, OUTPUT);
    pinMode(motor.dirPin, OUTPUT);
    digitalWrite(motor.stepPin, LOW);
  }

  releaseAll();

  Serial.begin(115200);  // USB CDC: the baud rate is ignored

  MKS1.begin(UART_BAUD, SERIAL_8N1);
  MKS2.begin(UART_BAUD, SERIAL_8N1);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);
}

void loop() {
  static String line;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      handleLine(line);
      line = "";
    } else if (line.length() < MAX_LINE_LENGTH) {
      line += c;
    }
  }

  // Don't leave a held motor powered forever.
  if (anyEnabled() && millis() - holdStartedMs > HOLD_TIMEOUT_MS) {
    releaseAll();
    Serial.println("note: motors released after 60 s hold");
  }
}

