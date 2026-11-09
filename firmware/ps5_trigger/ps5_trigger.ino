#include <Arduino.h>
#include <stdint.h>

// ============================================
// PS5 trigger reader: WeAct Black Pill (STM32F411CE)
//
// Reads the trigger's analog output (its position sensor) on the analog
// pins below and makes the values available two ways:
//
//   1. Over the ST-Link (SWD), no USB cable needed: the newest values are
//      kept in `trigger_share` in RAM, and lib/read_trigger.py reads them
//      through the ST-Link while this program runs.
//   2. Over the Black Pill's USB-C, if plugged in: lines of
//      time_ms,PA0,PA1,PA4,PA5,PA6,PA7,PB0,PB1  (12-bit, 0..4095)
//
// Which pin the trigger is on does not need to be set here: every
// candidate pin is read, and read_trigger.py finds the one that moves.
//
// Power the trigger's sensor from 3.3V, never 5V: the analog pins
// read 0..3.3V.
//
// Build: STM32 MCU based boards -> Generic STM32F4 -> BlackPill F411CE,
// USB support "CDC (generic 'Serial' supersede U(S)ART)".
// ============================================

// Analog pins that are not used by the motor drivers' serial links.
constexpr uint32_t PINS[] = { PA0, PA1, PA4, PA5, PA6, PA7, PB0, PB1 };
constexpr size_t PIN_COUNT = sizeof(PINS) / sizeof(PINS[0]);

// Motor driver enable lines (active low): held high so no motor
// switches on while this program runs.
constexpr uint32_t MOTOR_ENABLE_PINS[] = { PA8, PB12, PB15 };

constexpr uint32_t SAMPLE_PERIOD_MS = 5;
constexpr uint32_t PRINT_PERIOD_MS = 20;
constexpr int SAMPLES_PER_READING = 8;  // averaged, to cut noise

// "TRG1": lets read_trigger.py find this block in RAM.
constexpr uint32_t SHARE_MAGIC = 0x31475254;

struct TriggerShare {
  uint32_t magic;
  uint32_t count;     // goes up by one for every new set of readings
  uint32_t time_ms;   // millis() of the newest readings
  uint16_t raw[PIN_COUNT];
};

// Read over SWD by read_trigger.py. Layout must match that script.
volatile TriggerShare trigger_share __attribute__((used)) = {
  SHARE_MAGIC, 0, 0, { 0 }
};

uint32_t lastSample = 0;
uint32_t lastPrint = 0;

uint16_t readAveraged(uint32_t pin) {
  uint32_t sum = 0;

  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    sum += analogRead(pin);
  }

  return (uint16_t)(sum / SAMPLES_PER_READING);
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

  for (uint32_t pin : PINS) {
    pinMode(pin, INPUT_ANALOG);
  }

  Serial.begin(115200);  // USB CDC: the baud rate is ignored
}

void loop() {
  uint32_t now = millis();

  if (now - lastSample < SAMPLE_PERIOD_MS) {
    return;
  }

  lastSample = now;

  for (size_t i = 0; i < PIN_COUNT; i++) {
    trigger_share.raw[i] = readAveraged(PINS[i]);
  }

  trigger_share.time_ms = now;
  trigger_share.count = trigger_share.count + 1;

  if (now - lastPrint >= PRINT_PERIOD_MS && Serial) {
    lastPrint = now;
    Serial.print(now);

    for (size_t i = 0; i < PIN_COUNT; i++) {
      Serial.print(",");
      Serial.print(trigger_share.raw[i]);
    }

    Serial.println();
  }
}
