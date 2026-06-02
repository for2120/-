#include <HX711.h>

// SmartCart loadcell sketch.
// RFID is read directly by Python from YRM1001 readers.
// Arduino only sends HX711 loadcell weight.
//
// Wiring:
//   Loadcell + HX711: DT/DOUT=D2, SCK/CLK=D3, VCC=5V, GND=GND

#define LOADCELL_DOUT_PIN 2
#define LOADCELL_SCK_PIN 3

HX711 upgradedScale;

const float CALIBRATION_FACTOR = -28.6356f;
const int WEIGHT_SAMPLE_COUNT = 3;
const float MIN_VALID_WEIGHT_G = 3.0f;
const float STABLE_EPSILON_G = 20.0f;
const float MIN_EMIT_DELTA_G = 10.0f;
const unsigned long WEIGHT_POLL_INTERVAL_MS = 120;
const unsigned long CONFIRM_HOLD_MS = 450;
const byte REQUIRED_STABLE_COUNT = static_cast<byte>((CONFIRM_HOLD_MS / WEIGHT_POLL_INTERVAL_MS) + 1);
const unsigned long POST_EMIT_SETTLE_MS = 300;
const unsigned long HX711_READY_WAIT_MS = 3000;
const unsigned long HX711_STATUS_INTERVAL_MS = 2000;

float lastObservedWeight = 0.0f;
float lastStableWeight = 0.0f;
bool hasStableWeight = false;
byte stableWeightCount = 0;
unsigned long lastWeightPollMs = 0;
unsigned long lastWeightEmitMs = 0;
unsigned long lastHx711StatusMs = 0;

void printHx711Status(bool ready) {
  Serial.print("HX711|READY:");
  Serial.println(ready ? 1 : 0);
}

bool waitForHx711Ready(unsigned long timeoutMs) {
  unsigned long startMs = millis();

  while (!upgradedScale.is_ready()) {
    if ((millis() - startMs) >= timeoutMs) {
      return false;
    }
    delay(20);
  }

  return true;
}

float readWeightGrams() {
  if (!upgradedScale.is_ready()) {
    unsigned long now = millis();
    if ((now - lastHx711StatusMs) >= HX711_STATUS_INTERVAL_MS) {
      printHx711Status(false);
      lastHx711StatusMs = now;
    }
    return NAN;
  }

  float value = upgradedScale.get_units(WEIGHT_SAMPLE_COUNT);
  if (isnan(value) || isinf(value)) {
    return NAN;
  }

  if (abs(value) < MIN_VALID_WEIGHT_G) {
    return 0.0f;
  }

  return value;
}

void emitStableWeightIfNeeded() {
  unsigned long now = millis();
  if ((now - lastWeightPollMs) < WEIGHT_POLL_INTERVAL_MS) {
    return;
  }

  lastWeightPollMs = now;
  float currentWeight = readWeightGrams();
  if (isnan(currentWeight)) {
    return;
  }

  if (!hasStableWeight) {
    printHx711Status(true);
    lastHx711StatusMs = now;

    lastObservedWeight = currentWeight;
    lastStableWeight = currentWeight;
    hasStableWeight = true;
    stableWeightCount = REQUIRED_STABLE_COUNT;

    Serial.print("WEIGHT|TOTAL_G:");
    Serial.println(lastStableWeight, 1);
    lastWeightEmitMs = now;
    return;
  }

  if (abs(currentWeight - lastObservedWeight) <= STABLE_EPSILON_G) {
    if (stableWeightCount < REQUIRED_STABLE_COUNT) {
      stableWeightCount++;
    }
  } else {
    stableWeightCount = 0;
  }

  lastObservedWeight = currentWeight;

  if (stableWeightCount < REQUIRED_STABLE_COUNT) {
    return;
  }

  if ((now - lastWeightEmitMs) < POST_EMIT_SETTLE_MS) {
    return;
  }

  if (abs(currentWeight - lastStableWeight) <= STABLE_EPSILON_G) {
    return;
  }

  if (abs(currentWeight - lastStableWeight) < MIN_EMIT_DELTA_G) {
    return;
  }

  lastStableWeight = currentWeight;
  Serial.print("WEIGHT|TOTAL_G:");
  Serial.println(lastStableWeight, 1);
  lastWeightEmitMs = now;
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("BOOT|LOADCELL_ONLY");
  Serial.println("HX711|PINS:DOUT=D2,SCK=D3");

  upgradedScale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  upgradedScale.set_scale(CALIBRATION_FACTOR);

  if (waitForHx711Ready(HX711_READY_WAIT_MS)) {
    printHx711Status(true);
    Serial.println("HX711|TARE_START");
    upgradedScale.tare();
    Serial.println("HX711|TARE_OK");
  } else {
    printHx711Status(false);
    Serial.println("HX711|TARE_SKIPPED");
  }

  Serial.println("READY|LOADCELL_ONLY");
}

void loop() {
  emitStableWeightIfNeeded();
}
