#include <HX711.h>

// 현재 배선 유지
// HX711 SCK -> D2
// HX711 DT  -> D3
// HX711 VCC -> 5V
// HX711 GND -> GND

constexpr byte LOADCELL_DOUT_PIN = 3;
constexpr byte LOADCELL_SCK_PIN = 2;

// 기존 코드에서 쓰던 시작값. 휴대폰 실측값에 맞게 조금씩 보정하면 된다.
float calibrationFactor = -28.6356f;
float calibrationReferenceWeightG = 1000.0f;

constexpr int SAMPLE_COUNT = 5;
constexpr float STABLE_EPSILON_G = 12.0f;
constexpr unsigned long READ_INTERVAL_MS = 150;
constexpr unsigned long CONFIRM_HOLD_MS = 1000;
constexpr byte REQUIRED_STABLE_COUNT = static_cast<byte>((CONFIRM_HOLD_MS / READ_INTERVAL_MS) + 1);
constexpr float NO_LOAD_THRESHOLD_G = 0.5f;
constexpr float DETECT_THRESHOLD_G = 2.0f;
constexpr float REMOVE_THRESHOLD_G = 0.8f;
constexpr float MIN_CONFIRMED_EVENT_WEIGHT_G = 60.0f;
constexpr float SAME_ITEM_TOLERANCE_PERCENT = 0.20f;
constexpr float SAME_ITEM_MIN_TOLERANCE_G = 35.0f;
constexpr float TOTAL_CONFIRM_TOLERANCE_PERCENT = 0.08f;
constexpr float TOTAL_CONFIRM_MIN_TOLERANCE_G = 45.0f;
constexpr unsigned long STARTUP_SETTLE_MS = 2000;
constexpr unsigned long READY_WAIT_MS = 1200;
constexpr unsigned long REMOVE_HOLD_MS = 1000;
constexpr unsigned long POST_EVENT_SETTLE_MS = 1200;
constexpr int MAX_TRACKED_ITEM_TYPES = 10;

HX711 scale;

float lastReading = 0.0f;
byte stableCount = 0;
unsigned long lastReadMs = 0;
float confirmedTotalWeightG = 0.0f;
unsigned long lastReadyErrorMs = 0;
unsigned long lastEventConfirmedMs = 0;
bool detectCandidateActive = false;
bool removeCandidateActive = false;
unsigned long detectCandidateStartedMs = 0;
unsigned long removeCandidateStartedMs = 0;
float candidateTargetWeightG = 0.0f;
float knownItemWeightsG[MAX_TRACKED_ITEM_TYPES] = {0.0f};
int knownItemCount = 0;

void printWeightLine(const char* statusText, float grams) {
  Serial.print(statusText);
  Serial.print(" | WEIGHT_G:");
  Serial.print(grams, 1);
  Serial.print(" | WEIGHT_KG:");
  Serial.println(grams / 1000.0f, 4);
}

void printCartEventLine(const char* statusText, float itemWeightG, float totalWeightG, int matchedItemIndex) {
  Serial.print(statusText);
  Serial.print(" | ITEM_G:");
  Serial.print(itemWeightG, 1);
  Serial.print(" | ITEM_KG:");
  Serial.print(itemWeightG / 1000.0f, 4);
  Serial.print(" | TOTAL_G:");
  Serial.print(totalWeightG, 1);
  Serial.print(" | TOTAL_KG:");
  Serial.print(totalWeightG / 1000.0f, 4);
  if (matchedItemIndex >= 0) {
    Serial.print(" | ITEM_MATCH:같은 물건 #");
    Serial.print(matchedItemIndex + 1);
  } else {
    Serial.print(" | ITEM_MATCH:새 물건");
  }
  Serial.println();
}

float calculateSameItemToleranceG(float referenceWeightG) {
  float percentToleranceG = abs(referenceWeightG) * SAME_ITEM_TOLERANCE_PERCENT;
  return max(percentToleranceG, SAME_ITEM_MIN_TOLERANCE_G);
}

float calculateTotalConfirmToleranceG(float referenceTotalWeightG) {
  float percentToleranceG = abs(referenceTotalWeightG) * TOTAL_CONFIRM_TOLERANCE_PERCENT;
  return max(percentToleranceG, TOTAL_CONFIRM_MIN_TOLERANCE_G);
}

int findMatchingItemIndex(float itemWeightG) {
  for (int index = 0; index < knownItemCount; ++index) {
    float toleranceG = calculateSameItemToleranceG(knownItemWeightsG[index]);
    if (abs(itemWeightG - knownItemWeightsG[index]) <= toleranceG) {
      return index;
    }
  }
  return -1;
}

int trackKnownItem(float itemWeightG) {
  int matchedIndex = findMatchingItemIndex(itemWeightG);
  if (matchedIndex >= 0) {
    knownItemWeightsG[matchedIndex] = (knownItemWeightsG[matchedIndex] + itemWeightG) / 2.0f;
    return matchedIndex;
  }

  if (knownItemCount >= MAX_TRACKED_ITEM_TYPES) {
    return -1;
  }

  knownItemWeightsG[knownItemCount] = itemWeightG;
  knownItemCount++;
  return -1;
}

bool waitForScaleReady(unsigned long timeoutMs) {
  unsigned long startMs = millis();
  while (!scale.is_ready()) {
    if ((millis() - startMs) >= timeoutMs) {
      return false;
    }
    delay(10);
  }
  return true;
}

void printCurrentSnapshot() {
  if (!waitForScaleReady(READY_WAIT_MS)) {
    Serial.println("[ERROR] HX711 not ready after wait");
    return;
  }

  long rawValue = scale.read_average(5);
  float grams = scale.get_units(SAMPLE_COUNT);
  Serial.print("[SNAPSHOT] RAW:");
  Serial.print(rawValue);
  Serial.print(" | GRAMS:");
  Serial.print(grams, 1);
  Serial.print(" | KG:");
  Serial.print(grams / 1000.0f, 4);
  Serial.print(" | CAL:");
  Serial.println(calibrationFactor);
}

void printHelp() {
  Serial.println("=== Loadcell Phone Test ===");
  Serial.println("Commands:");
  Serial.println("  t  -> tare again");
  Serial.println("  +  -> calibrationFactor + 50");
  Serial.println("  -  -> calibrationFactor - 50");
  Serial.println("  p  -> print current calibrationFactor");
  Serial.println("  r  -> print one raw sensor snapshot");
  Serial.println("  c  -> calibrate using current load and reference weight");
  Serial.println("  0  -> set reference weight to 500g");
  Serial.println("  1  -> set reference weight to 1000g");
  Serial.println("  2  -> set reference weight to 1500g");
  Serial.println("  5  -> set reference weight to 2000g");
  Serial.println("---------------------------");
}

void printCalibrationSettings() {
  Serial.print("[INFO] calibrationFactor = ");
  Serial.print(calibrationFactor);
  Serial.print(" | referenceWeightG = ");
  Serial.println(calibrationReferenceWeightG, 1);
}

void calibrateWithCurrentLoad() {
  if (!waitForScaleReady(READY_WAIT_MS)) {
    Serial.println("[ERROR] HX711 not ready after wait");
    return;
  }

  float measuredWeightG = scale.get_units(10);
  if (abs(measuredWeightG) < 0.1f) {
    Serial.println("[ERROR] Current measured weight is too small for calibration");
    return;
  }

  calibrationFactor = calibrationFactor * (measuredWeightG / calibrationReferenceWeightG);
  scale.set_scale(calibrationFactor);

  Serial.print("[INFO] Recalibrated with current load. measured=");
  Serial.print(measuredWeightG, 1);
  Serial.print("g | target=");
  Serial.print(calibrationReferenceWeightG, 1);
  Serial.print("g | new calibrationFactor=");
  Serial.println(calibrationFactor, 4);
}

void tareScale() {
  Serial.println("[INFO] Tare start. Remove all weight.");
  delay(1500);
  scale.tare();
  lastReading = 0.0f;
  stableCount = 0;
  confirmedTotalWeightG = 0.0f;
  lastEventConfirmedMs = 0;
  detectCandidateActive = false;
  removeCandidateActive = false;
  detectCandidateStartedMs = 0;
  removeCandidateStartedMs = 0;
  candidateTargetWeightG = 0.0f;
  Serial.println("[INFO] Tare complete.");
  printWeightLine("STATUS: 현재 총 무게", confirmedTotalWeightG);
}

void handleSerialCommand() {
  while (Serial.available() > 0) {
    char command = static_cast<char>(Serial.read());
    if (command == 't' || command == 'T') {
      tareScale();
    } else if (command == '+') {
      calibrationFactor += 50.0f;
      scale.set_scale(calibrationFactor);
      Serial.print("[INFO] calibrationFactor = ");
      Serial.println(calibrationFactor);
    } else if (command == '-') {
      calibrationFactor -= 50.0f;
      scale.set_scale(calibrationFactor);
      Serial.print("[INFO] calibrationFactor = ");
      Serial.println(calibrationFactor);
    } else if (command == 'p' || command == 'P') {
      printCalibrationSettings();
    } else if (command == 'r' || command == 'R') {
      printCurrentSnapshot();
    } else if (command == 'c' || command == 'C') {
      calibrateWithCurrentLoad();
    } else if (command == '0') {
      calibrationReferenceWeightG = 500.0f;
      printCalibrationSettings();
    } else if (command == '1') {
      calibrationReferenceWeightG = 1000.0f;
      printCalibrationSettings();
    } else if (command == '2') {
      calibrationReferenceWeightG = 1500.0f;
      printCalibrationSettings();
    } else if (command == '5') {
      calibrationReferenceWeightG = 2000.0f;
      printCalibrationSettings();
    }
  }
}

void setup() {
  Serial.begin(9600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibrationFactor);
  printHelp();
  Serial.println("[INFO] Waiting for HX711 startup...");
  delay(STARTUP_SETTLE_MS);
  tareScale();
}

void loop() {
  handleSerialCommand();

  unsigned long now = millis();
  if ((now - lastReadMs) < READ_INTERVAL_MS) {
    return;
  }

  lastReadMs = now;

  if (!waitForScaleReady(READY_WAIT_MS)) {
    if ((now - lastReadyErrorMs) >= 2000) {
      Serial.println("[ERROR] HX711 not ready after wait");
      lastReadyErrorMs = now;
    }
    return;
  }

  float grams = scale.get_units(SAMPLE_COUNT);
  if (abs(grams) < NO_LOAD_THRESHOLD_G) {
    grams = 0.0f;
  }

  if (abs(grams - lastReading) <= STABLE_EPSILON_G) {
    if (stableCount < REQUIRED_STABLE_COUNT) {
      stableCount++;
    }
  } else {
    stableCount = 0;
  }

  lastReading = grams;

  if (stableCount < REQUIRED_STABLE_COUNT) {
    return;
  }

  if ((now - lastEventConfirmedMs) < POST_EVENT_SETTLE_MS) {
    return;
  }

  float totalDeltaG = grams - confirmedTotalWeightG;

  if (totalDeltaG >= DETECT_THRESHOLD_G) {
    removeCandidateActive = false;
    float candidateToleranceG = calculateTotalConfirmToleranceG(max(candidateTargetWeightG, grams));

    if (!detectCandidateActive || abs(candidateTargetWeightG - grams) > candidateToleranceG) {
      detectCandidateActive = true;
      detectCandidateStartedMs = now;
      candidateTargetWeightG = grams;
      return;
    }

    candidateTargetWeightG = (candidateTargetWeightG + grams) / 2.0f;

    if ((now - detectCandidateStartedMs) < CONFIRM_HOLD_MS) {
      return;
    }

    float addedItemWeightG = grams - confirmedTotalWeightG;
    if (addedItemWeightG < MIN_CONFIRMED_EVENT_WEIGHT_G) {
      detectCandidateActive = false;
      candidateTargetWeightG = 0.0f;
      return;
    }

    int matchedItemIndex = trackKnownItem(addedItemWeightG);
    confirmedTotalWeightG = grams;
    lastEventConfirmedMs = now;
    detectCandidateActive = false;
    candidateTargetWeightG = 0.0f;
    printCartEventLine("STATUS: 물건 추가 담김 확정", addedItemWeightG, confirmedTotalWeightG, matchedItemIndex);
    return;
  }

  detectCandidateActive = false;

  if (totalDeltaG > -REMOVE_THRESHOLD_G) {
    removeCandidateActive = false;
    return;
  }

  float candidateToleranceG = calculateTotalConfirmToleranceG(max(candidateTargetWeightG, grams));

  if (!removeCandidateActive || abs(candidateTargetWeightG - grams) > candidateToleranceG) {
    removeCandidateActive = true;
    removeCandidateStartedMs = now;
    candidateTargetWeightG = grams;
    return;
  }

  candidateTargetWeightG = (candidateTargetWeightG + grams) / 2.0f;

  if ((now - removeCandidateStartedMs) < REMOVE_HOLD_MS) {
    return;
  }

  float removedWeightG = confirmedTotalWeightG - grams;
  if (removedWeightG < MIN_CONFIRMED_EVENT_WEIGHT_G) {
    removeCandidateActive = false;
    candidateTargetWeightG = 0.0f;
    return;
  }

  confirmedTotalWeightG = max(grams, 0.0f);
  lastEventConfirmedMs = now;
  removeCandidateActive = false;
  candidateTargetWeightG = 0.0f;
  printCartEventLine("STATUS: 물건 제거 확정", removedWeightG, confirmedTotalWeightG, findMatchingItemIndex(removedWeightG));
  if (confirmedTotalWeightG <= NO_LOAD_THRESHOLD_G) {
    Serial.println("STATUS: 물건 없음 (0g / 0.0000kg)");
  }
}
