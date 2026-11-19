#include <Arduino.h>
#include <TM1637Display.h>

// =====================================================
// CONFIGURATION
// =====================================================

// ---------- Pins ----------
const uint8_t DISPLAY_CLK_PIN = 50;
const uint8_t DISPLAY_DIO_PIN = 52;
const uint8_t RADAR_RX_PIN    = 15;   // Mega RX3
const uint8_t KNOB_ANALOG_PIN = A1;

// ---------- Display ----------
const uint8_t DISPLAY_BRIGHTNESS = 7;   // 0..7

// ---------- Radar UART ----------
const unsigned long RADAR_BAUD = 115200;

// ---------- Speed conversion ----------
const float KMH_TO_MPH = 0.621371f;

// Running indicator animation
const unsigned long RUN_INDICATOR_INTERVAL_MS = 1500;

// ---------- Knob calibration ----------
// User requested:
//   low voltage  -> 120 MPH
//   high voltage -> 5 MPH
//
// NOTE: 5.5V cannot be measured directly on a Mega analog input.
// Defaulting to 5.0V max. Change if your signal is scaled externally.
const float ADC_REFERENCE_VOLTAGE = 5.0f;
const float KNOBW_LOW_VOLTAGE_MPH_MIN = 0.1f;   // >= this voltage => min MPH
const float KNOBW_HIGH_VOLTAGE_MPH_MAX  = 4.2f;   // <= this voltage => max MPH

const int   SPEED_LIMIT_MIN_MPH = 5;
const int   SPEED_LIMIT_MAX_MPH = 120;

// ---------- Knob behavior ----------
const unsigned long KNOB_SETTLE_HIDE_MS = 2000;   // show set speed while changing, then hide after this long
const int KNOB_CHANGE_THRESHOLD_RAW = 1;          // ADC counts to consider "changed"
const float KNOB_FILTER_ALPHA = 0.05f;            // 0..1, larger = more responsive
const int KNOB_NOISE_DEADBAND_RAW   = 7;   // ignore tiny movement

// ---------- Radar display behavior ----------
const unsigned long RADAR_HOLD_AFTER_LOST_MS = 3000;   // keep last radar speed this long after signal disappears

// ---------- Over-limit blink behavior ----------
const int OVER_LIMIT_SLOW_MIN_MPH = 1;   // 1..4 over => slow blink
const int OVER_LIMIT_SLOW_MAX_MPH = 4;
const int OVER_LIMIT_FAST_MIN_MPH = 5;   // 5+ over => fast blink

const unsigned long SLOW_BLINK_ON_MS  = 500;
const unsigned long SLOW_BLINK_OFF_MS = 250;

const unsigned long FAST_BLINK_ON_MS  = 200;
const unsigned long FAST_BLINK_OFF_MS = 100;

// ---------- Radar parsing ----------
const uint8_t RADAR_MAX_TARGETS = 8;
const uint16_t RADAR_PAYLOAD_MAX = 256;

// =====================================================
// TYPES
// =====================================================

struct RadarData {
  byte targetQuantity;
  byte alarmInformation;

  byte angle;
  byte distance;
  byte speedDirection;
  byte speed;      // km/h
  byte snr;

  float speedMph;  // derived
};

// =====================================================
// GLOBALS
// =====================================================

TM1637Display display(DISPLAY_CLK_PIN, DISPLAY_DIO_PIN);

// Use Mega hardware UART3: RX3 = pin 15, TX3 = pin 14
#define RadarSerial Serial3

// ---------- Radar frame parsing ----------
static const uint8_t HDR[4]  = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t TAIL[4] = {0xF8, 0xF7, 0xF6, 0xF5};

enum ParseState : uint8_t {
  FIND_HDR,
  READ_LEN,
  READ_PAYLOAD,
  READ_TAIL
};

ParseState parseState = FIND_HDR;
uint8_t hdrPos = 0;
uint8_t tailPos = 0;
uint16_t payloadLen = 0;
uint16_t payloadRead = 0;
uint8_t payload[RADAR_PAYLOAD_MAX];

// ---------- Knob state ----------
float filteredKnobRaw = 0.0f;
int lastStableKnobRaw = -1000;
int currentSpeedLimitMph = SPEED_LIMIT_MAX_MPH;
unsigned long lastKnobChangeMs = 0;
bool knobPreviewActive = true;

// ---------- Radar state ----------
bool radarHasRecentSignal = false;
unsigned long lastRadarSignalMs = 0;
float lastRadarSpeedMph = 0.0f;

// ---------- On Indicator ----------
uint8_t runIndicatorPosition = 0;
unsigned long lastRunIndicatorUpdate = 0;

// =====================================================
// HELPERS
// =====================================================

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float rawToVoltage(float raw) {
  return raw * ADC_REFERENCE_VOLTAGE / 1023.0f;
}

// Maps knob voltage to MPH:
// low voltage -> high MPH
// high voltage -> low MPH
int knobVoltageToSpeedLimitMph(float volts)
{
  // Clamp to limits first
  if (volts <= KNOBW_LOW_VOLTAGE_MPH_MIN)
    return SPEED_LIMIT_MIN_MPH;

  if (volts >= KNOBW_HIGH_VOLTAGE_MPH_MAX)
    return SPEED_LIMIT_MAX_MPH;

  // Linear interpolation
  float t = (volts - KNOBW_LOW_VOLTAGE_MPH_MIN) /
            (KNOBW_HIGH_VOLTAGE_MPH_MAX - KNOBW_LOW_VOLTAGE_MPH_MIN);

  float mph = SPEED_LIMIT_MIN_MPH +
              t * (SPEED_LIMIT_MAX_MPH - SPEED_LIMIT_MIN_MPH);

  return clampInt((int)lroundf(mph), SPEED_LIMIT_MIN_MPH, SPEED_LIMIT_MAX_MPH);
}

void showNumberZeroPadded(int value) {
  value = clampInt(value, 0, 9999);
  display.showNumberDec(value, true, 4, 0);   // leading zeros on
}

void showNumberNormal(int value) {
  value = clampInt(value, 0, 9999);
  display.showNumberDec(value, false, 4, 0);  // no leading zeros
}

void blankDisplay() {
  display.clear();
}

bool blinkVisible(unsigned long nowMs, unsigned long onMs, unsigned long offMs) {
  unsigned long cycle = onMs + offMs;
  if (cycle == 0) return true;
  unsigned long phase = nowMs % cycle;
  return phase < onMs;
}

void drawRunIndicator(unsigned long nowMs) {

  if (nowMs - lastRunIndicatorUpdate < RUN_INDICATOR_INTERVAL_MS)
    return;

  lastRunIndicatorUpdate = nowMs;

  uint8_t segments[4] = {0,0,0,0};

  // bottom segment of the active digit
  segments[runIndicatorPosition] = 0x08;

  display.setSegments(segments);

  runIndicatorPosition++;
  if (runIndicatorPosition > 3)
    runIndicatorPosition = 0;
}

// =====================================================
// RADAR PAYLOAD PARSING
// =====================================================

const uint8_t BYTES_PER_TARGET = 5;
const uint8_t PAYLOAD_HEADER_BYTES = 2;

uint8_t parseRadarPayload(const uint8_t* payloadBuf,
                          uint16_t len,
                          RadarData* outTargets,
                          uint8_t maxTargets) {
  if (!payloadBuf || !outTargets) return 0;
  if (len < PAYLOAD_HEADER_BYTES) return 0;

  uint8_t targetQty = payloadBuf[0];
  uint8_t alarmInfo = payloadBuf[1];

  uint16_t availableBytes = len - PAYLOAD_HEADER_BYTES;
  uint8_t targetsThatFit = (uint8_t)(availableBytes / BYTES_PER_TARGET);

  uint8_t count = targetQty;
  if (count > targetsThatFit) count = targetsThatFit;
  if (count > maxTargets) count = maxTargets;

  for (uint8_t i = 0; i < count; i++) {
    uint16_t base = PAYLOAD_HEADER_BYTES + (uint16_t)i * BYTES_PER_TARGET;

    RadarData rd;
    rd.targetQuantity   = targetQty;
    rd.alarmInformation = alarmInfo;
    rd.angle            = payloadBuf[base + 0];
    rd.distance         = payloadBuf[base + 1];
    rd.speedDirection   = payloadBuf[base + 2];
    rd.speed            = payloadBuf[base + 3];
    rd.snr              = payloadBuf[base + 4];
    rd.speedMph         = rd.speed * KMH_TO_MPH;

    outTargets[i] = rd;
  }

  return count;
}

// Pick the fastest target, regardless of direction.
bool payloadToSpeedMph(float &outMph) {
  if (payloadLen < PAYLOAD_HEADER_BYTES) return false;

  RadarData targets[RADAR_MAX_TARGETS];
  uint8_t count = parseRadarPayload(payload, payloadLen, targets, RADAR_MAX_TARGETS);

  if (count == 0) {
    outMph = 0.0f;
    return false;
  }

  float maxMph = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    if (targets[i].speedMph > maxMph) {
      maxMph = targets[i].speedMph;
    }
  }

  outMph = maxMph;
  return true;
}

// =====================================================
// RADAR FRAME BYTE PARSER
// =====================================================

bool parseOneByte(uint8_t b) {
  switch (parseState) {
    case FIND_HDR:
      if (b == HDR[hdrPos]) {
        hdrPos++;
        if (hdrPos == 4) {
          hdrPos = 0;
          parseState = READ_LEN;
        }
      } else {
        hdrPos = 0;
      }
      break;

    case READ_LEN: {
      static uint8_t lenBytes[2];
      static uint8_t lenPos = 0;

      lenBytes[lenPos++] = b;
      if (lenPos == 2) {
        lenPos = 0;
        payloadLen = (uint16_t)lenBytes[0] | ((uint16_t)lenBytes[1] << 8);

        if (payloadLen > RADAR_PAYLOAD_MAX) {
          parseState = FIND_HDR;
          payloadLen = 0;
          payloadRead = 0;
          tailPos = 0;
          return false;
        }

        payloadRead = 0;
        parseState = READ_PAYLOAD;
      }
      break;
    }

    case READ_PAYLOAD:
      payload[payloadRead++] = b;
      if (payloadRead >= payloadLen) {
        tailPos = 0;
        parseState = READ_TAIL;
      }
      break;

    case READ_TAIL:
      if (b == TAIL[tailPos]) {
        tailPos++;
        if (tailPos == 4) {
          parseState = FIND_HDR;
          tailPos = 0;
          return true;
        }
      } else {
        parseState = FIND_HDR;
        tailPos = 0;
      }
      break;
  }

  return false;
}

// =====================================================
// INPUT / STATE UPDATES
// =====================================================

void updateKnob(unsigned long nowMs) {
  int raw = analogRead(KNOB_ANALOG_PIN);

  // Low-pass filter
  if (filteredKnobRaw == 0.0f) {
    filteredKnobRaw = raw;
  } else {
    filteredKnobRaw = (KNOB_FILTER_ALPHA * raw) +
                      ((1.0f - KNOB_FILTER_ALPHA) * filteredKnobRaw);
  }

  int filteredRawInt = (int)lroundf(filteredKnobRaw);

  // Ignore tiny jitter around the current stable value
  if (abs(filteredRawInt - lastStableKnobRaw) < KNOB_NOISE_DEADBAND_RAW) {
    if (knobPreviewActive && (nowMs - lastKnobChangeMs >= KNOB_SETTLE_HIDE_MS)) {
      knobPreviewActive = false;
    }
    return;
  }

  // Real change detected
  lastStableKnobRaw = filteredRawInt;

  float volts = rawToVoltage(filteredKnobRaw);
  int newSpeedLimit = knobVoltageToSpeedLimitMph(volts);

  // Only treat it as a visible change if the speed limit actually changed
  if (newSpeedLimit != currentSpeedLimitMph) {
    currentSpeedLimitMph = newSpeedLimit;
    knobPreviewActive = true;
    lastKnobChangeMs = nowMs;

    Serial.print("Knob volts: ");
    Serial.print(volts, 3);
    Serial.print("  Limit MPH: ");
    Serial.println(currentSpeedLimitMph);
  } else {
    // still moved, but not enough to change displayed MPH
    if (knobPreviewActive && (nowMs - lastKnobChangeMs >= KNOB_SETTLE_HIDE_MS)) {
      knobPreviewActive = false;
    }
  }
}

void updateRadar(unsigned long nowMs) {
  while (RadarSerial.available() > 0) {
    uint8_t b = (uint8_t)RadarSerial.read();

    if (parseOneByte(b)) {
      float mph = 0.0f;
      if (payloadToSpeedMph(mph) && mph > 0.0f) {
        lastRadarSpeedMph = mph;
        lastRadarSignalMs = nowMs;
        radarHasRecentSignal = true;

        Serial.print("Radar speed: ");
        Serial.print(mph, 1);
        Serial.println(" mph");
      }
    }
  }

  if (radarHasRecentSignal && (nowMs - lastRadarSignalMs > RADAR_HOLD_AFTER_LOST_MS)) {
    radarHasRecentSignal = false;
    lastRadarSpeedMph = 0.0f;
  }
}

// =====================================================
// DISPLAY LOGIC
// =====================================================

void updateDisplay(unsigned long nowMs) {
  // Priority 1: knob preview while user is adjusting
  if (knobPreviewActive) {
    showNumberZeroPadded(currentSpeedLimitMph);
    return;
  }

  // Priority 2: radar speed if present / within hold time
  if (radarHasRecentSignal) {
    int speedMphInt = (int)lroundf(lastRadarSpeedMph);
    int overBy = speedMphInt - currentSpeedLimitMph;

    bool shouldShow = true;

    if (overBy >= OVER_LIMIT_FAST_MIN_MPH) {
      shouldShow = blinkVisible(nowMs, FAST_BLINK_ON_MS, FAST_BLINK_OFF_MS);
    } else if (overBy >= OVER_LIMIT_SLOW_MIN_MPH && overBy <= OVER_LIMIT_SLOW_MAX_MPH) {
      shouldShow = blinkVisible(nowMs, SLOW_BLINK_ON_MS, SLOW_BLINK_OFF_MS);
    }

    if (shouldShow) {
      showNumberNormal(speedMphInt);
    } else {
      blankDisplay();
    }

    return;
  }

  // Priority 3: idle state → show running indicator
  drawRunIndicator(nowMs);
}

// =====================================================
// SETUP / LOOP
// =====================================================

void setup() {
  Serial.begin(115200);
  RadarSerial.begin(RADAR_BAUD);

  pinMode(KNOB_ANALOG_PIN, INPUT);

  display.setBrightness(DISPLAY_BRIGHTNESS, true);
  display.clear();

  // Initialize knob state immediately
  int raw = analogRead(KNOB_ANALOG_PIN);
  filteredKnobRaw = raw;
  lastStableKnobRaw = raw;
  currentSpeedLimitMph = knobVoltageToSpeedLimitMph(rawToVoltage(raw));
  lastKnobChangeMs = millis();
  knobPreviewActive = true;

  Serial.println("Mega + HLK-LD2451 + TM1637 speed display ready.");
  Serial.println("Radar: Serial3 RX=15");
  Serial.println("Display CLK=50 DIO=52");
  Serial.println("Knob=A15");
}

void loop() {
  unsigned long nowMs = millis();

  updateKnob(nowMs);
  updateRadar(nowMs);
  updateDisplay(nowMs);
}