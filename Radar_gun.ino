/*
 * =====================================================
 *  Radar Speed Display
 * =====================================================
 *
 *  Reads target speeds from an HLK-LD2451 24 GHz radar module and shows
 *  them on a 4-digit TM1637 7-segment display, much like a roadside
 *  "YOUR SPEED" sign.
 *
 *  Hardware:
 *    - Arduino Mega 2560 (uses hardware UART3 for the radar)
 *    - HLK-LD2451 radar module   -> Serial3 (RX3 = pin 15)
 *    - TM1637 4-digit display    -> CLK = pin 50, DIO = pin 52
 *    - Potentiometer / analog in -> A1 (sets the speed limit)
 *
 *  Display behaviour (highest priority first):
 *    1. While the knob is being turned, show the selected speed limit
 *       (zero padded, e.g. "0035") until it has been still for 2 s.
 *    2. If the radar sees a target, show the fastest target's speed in MPH.
 *       The reading is held for 3 s after the target disappears.
 *         - 1..4 MPH over the limit -> slow blink
 *         - 5+  MPH over the limit  -> fast blink
 *    3. Otherwise show an idle "running" animation: a single bottom
 *       segment stepping across the four digits.
 *
 *  Debug output is printed on the USB serial port at 115200 baud.
 */

#include <Arduino.h>
#include <TM1637Display.h>   // "TM1637" library by Avishay Orpaz

// =====================================================
// CONFIGURATION
// =====================================================

// ---------- Pins ----------
const uint8_t DISPLAY_CLK_PIN = 50;
const uint8_t DISPLAY_DIO_PIN = 52;
const uint8_t RADAR_RX_PIN    = 15;   // Mega RX3 (informational; Serial3 is fixed to this pin)
const uint8_t KNOB_ANALOG_PIN = A1;

// ---------- Display ----------
const uint8_t DISPLAY_BRIGHTNESS = 7;   // 0 (dimmest) .. 7 (brightest)

// ---------- Radar UART ----------
const unsigned long RADAR_BAUD = 115200;   // LD2451 default baud rate

// ---------- Speed conversion ----------
const float KMH_TO_MPH = 0.621371f;   // the radar reports km/h

// ---------- Idle animation ----------
const unsigned long RUN_INDICATOR_INTERVAL_MS = 1500;   // time between animation steps

// ---------- Knob calibration ----------
// The knob voltage is mapped linearly onto the speed-limit range:
//   voltage <= KNOBW_LOW_VOLTAGE_MPH_MIN  -> SPEED_LIMIT_MIN_MPH  (5 MPH)
//   voltage >= KNOBW_HIGH_VOLTAGE_MPH_MAX -> SPEED_LIMIT_MAX_MPH  (120 MPH)
//   anything in between is interpolated.
//
// The two voltage thresholds leave a small dead zone at each end of the
// knob's travel so the extremes are easy to hit.
//
// NOTE: The Mega's analog inputs measure 0..5 V. If your control signal can
// exceed 5 V, scale it down externally (e.g. a voltage divider) and adjust
// ADC_REFERENCE_VOLTAGE / the thresholds to suit.
const float ADC_REFERENCE_VOLTAGE      = 5.0f;
const float KNOBW_LOW_VOLTAGE_MPH_MIN  = 0.1f;   // at/below this voltage => min MPH
const float KNOBW_HIGH_VOLTAGE_MPH_MAX = 4.2f;   // at/above this voltage => max MPH

const int   SPEED_LIMIT_MIN_MPH = 5;
const int   SPEED_LIMIT_MAX_MPH = 120;

// ---------- Knob behaviour ----------
const unsigned long KNOB_SETTLE_HIDE_MS = 2000;   // show set speed while changing, then hide after this long
const int KNOB_CHANGE_THRESHOLD_RAW = 1;          // (currently unused) ADC counts to consider "changed"
const float KNOB_FILTER_ALPHA = 0.05f;            // low-pass filter weight, 0..1; larger = more responsive
const int KNOB_NOISE_DEADBAND_RAW   = 7;          // ignore filtered movement smaller than this (ADC counts)

// ---------- Radar display behaviour ----------
const unsigned long RADAR_HOLD_AFTER_LOST_MS = 3000;   // keep last radar speed this long after signal disappears

// ---------- Over-limit blink behaviour ----------
const int OVER_LIMIT_SLOW_MIN_MPH = 1;   // 1..4 over => slow blink
const int OVER_LIMIT_SLOW_MAX_MPH = 4;
const int OVER_LIMIT_FAST_MIN_MPH = 5;   // 5+ over => fast blink

const unsigned long SLOW_BLINK_ON_MS  = 500;
const unsigned long SLOW_BLINK_OFF_MS = 250;

const unsigned long FAST_BLINK_ON_MS  = 200;
const unsigned long FAST_BLINK_OFF_MS = 100;

// ---------- Radar parsing ----------
const uint8_t RADAR_MAX_TARGETS = 8;     // max targets decoded per frame
const uint16_t RADAR_PAYLOAD_MAX = 256;  // frames claiming a longer payload are discarded

// =====================================================
// TYPES
// =====================================================

// One target as reported by the LD2451. The first two fields are per-frame
// values copied into every target for convenience.
struct RadarData {
  byte targetQuantity;     // number of targets the radar reported in this frame
  byte alarmInformation;   // alarm / status byte from the frame

  byte angle;              // raw angle byte (sensor encodes with a 0x80 offset)
  byte distance;           // distance to target in metres
  byte speedDirection;     // approaching vs. moving away
  byte speed;              // km/h
  byte snr;                // signal-to-noise ratio

  float speedMph;          // derived from `speed`
};

// =====================================================
// GLOBALS
// =====================================================

TM1637Display display(DISPLAY_CLK_PIN, DISPLAY_DIO_PIN);

// Use Mega hardware UART3: RX3 = pin 15, TX3 = pin 14
#define RadarSerial Serial3

// ---------- Radar frame parsing ----------
// LD2451 data frames look like:
//   [F4 F3 F2 F1] [len lo] [len hi] [payload ... len bytes] [F8 F7 F6 F5]
static const uint8_t HDR[4]  = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t TAIL[4] = {0xF8, 0xF7, 0xF6, 0xF5};

// States of the byte-at-a-time frame parser (see parseOneByte()).
enum ParseState : uint8_t {
  FIND_HDR,       // scanning for the 4-byte header
  READ_LEN,       // reading the 2-byte little-endian payload length
  READ_PAYLOAD,   // copying payload bytes into `payload`
  READ_TAIL       // verifying the 4-byte tail
};

ParseState parseState = FIND_HDR;
uint8_t hdrPos = 0;                  // how many header bytes have matched so far
uint8_t tailPos = 0;                 // how many tail bytes have matched so far
uint16_t payloadLen = 0;             // payload length announced by the current frame
uint16_t payloadRead = 0;            // payload bytes received so far
uint8_t payload[RADAR_PAYLOAD_MAX];  // payload of the most recent frame

// ---------- Knob state ----------
float filteredKnobRaw = 0.0f;                     // low-pass filtered ADC reading
int lastStableKnobRaw = -1000;                    // last filtered value that passed the deadband
int currentSpeedLimitMph = SPEED_LIMIT_MAX_MPH;   // the active speed limit
unsigned long lastKnobChangeMs = 0;               // when the speed limit last changed
bool knobPreviewActive = true;                    // true while the set speed is being shown

// ---------- Radar state ----------
bool radarHasRecentSignal = false;   // true while a reading is fresh or within the hold time
unsigned long lastRadarSignalMs = 0; // when the last non-zero reading arrived
float lastRadarSpeedMph = 0.0f;      // most recent fastest-target speed

// ---------- Idle indicator ----------
uint8_t runIndicatorPosition = 0;         // digit (0..3) currently lit by the animation
unsigned long lastRunIndicatorUpdate = 0; // when the animation last stepped

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

// Convert a 10-bit ADC reading (0..1023) to volts.
float rawToVoltage(float raw) {
  return raw * ADC_REFERENCE_VOLTAGE / 1023.0f;
}

// Maps knob voltage to a speed limit in MPH:
//   low voltage  -> SPEED_LIMIT_MIN_MPH
//   high voltage -> SPEED_LIMIT_MAX_MPH
// with linear interpolation in between (see "Knob calibration" above).
int knobVoltageToSpeedLimitMph(float volts)
{
  // Clamp to limits first
  if (volts <= KNOBW_LOW_VOLTAGE_MPH_MIN)
    return SPEED_LIMIT_MIN_MPH;

  if (volts >= KNOBW_HIGH_VOLTAGE_MPH_MAX)
    return SPEED_LIMIT_MAX_MPH;

  // Linear interpolation: t runs 0..1 across the usable voltage range
  float t = (volts - KNOBW_LOW_VOLTAGE_MPH_MIN) /
            (KNOBW_HIGH_VOLTAGE_MPH_MAX - KNOBW_LOW_VOLTAGE_MPH_MIN);

  float mph = SPEED_LIMIT_MIN_MPH +
              t * (SPEED_LIMIT_MAX_MPH - SPEED_LIMIT_MIN_MPH);

  return clampInt((int)lroundf(mph), SPEED_LIMIT_MIN_MPH, SPEED_LIMIT_MAX_MPH);
}

// Show a number with leading zeros (e.g. "0035"). Used for the set speed so
// it is visually distinct from a live radar reading.
void showNumberZeroPadded(int value) {
  value = clampInt(value, 0, 9999);
  display.showNumberDec(value, true, 4, 0);   // leading zeros on
}

// Show a number right-aligned without leading zeros (e.g. "  35").
void showNumberNormal(int value) {
  value = clampInt(value, 0, 9999);
  display.showNumberDec(value, false, 4, 0);  // no leading zeros
}

void blankDisplay() {
  display.clear();
}

// Non-blocking blink helper: returns true during the "on" part of a
// repeating on/off cycle, based purely on the current time.
bool blinkVisible(unsigned long nowMs, unsigned long onMs, unsigned long offMs) {
  unsigned long cycle = onMs + offMs;
  if (cycle == 0) return true;
  unsigned long phase = nowMs % cycle;
  return phase < onMs;
}

// Idle animation: lights the bottom segment of one digit at a time, moving
// one digit to the right every RUN_INDICATOR_INTERVAL_MS. Shows the unit is
// powered and waiting for a target.
void drawRunIndicator(unsigned long nowMs) {

  if (nowMs - lastRunIndicatorUpdate < RUN_INDICATOR_INTERVAL_MS)
    return;

  lastRunIndicatorUpdate = nowMs;

  uint8_t segments[4] = {0,0,0,0};

  // Segment D (bottom) of the active digit
  segments[runIndicatorPosition] = 0x08;

  display.setSegments(segments);

  runIndicatorPosition++;
  if (runIndicatorPosition > 3)
    runIndicatorPosition = 0;
}

// =====================================================
// RADAR PAYLOAD PARSING
// =====================================================

// Payload layout:
//   [0]   target quantity
//   [1]   alarm information
//   then BYTES_PER_TARGET bytes per target:
//   [+0] angle  [+1] distance  [+2] direction  [+3] speed (km/h)  [+4] SNR
const uint8_t BYTES_PER_TARGET = 5;
const uint8_t PAYLOAD_HEADER_BYTES = 2;

// Decode a frame payload into `outTargets`. Returns the number of targets
// written, which is limited by what the radar reported, what actually fits
// in `len` bytes, and `maxTargets`.
uint8_t parseRadarPayload(const uint8_t* payloadBuf,
                          uint16_t len,
                          RadarData* outTargets,
                          uint8_t maxTargets) {
  if (!payloadBuf || !outTargets) return 0;
  if (len < PAYLOAD_HEADER_BYTES) return 0;

  uint8_t targetQty = payloadBuf[0];
  uint8_t alarmInfo = payloadBuf[1];

  // Never trust targetQty blindly: cap it by the bytes actually received.
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

// Decode the most recently received payload and report the speed of the
// fastest target, regardless of direction. Returns false if the frame
// contained no targets.
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

// Feed one byte from the radar UART into the frame state machine.
// Returns true exactly when a complete, valid frame (header + length +
// payload + tail) has been received; the payload is then in `payload` with
// length `payloadLen`. Any unexpected byte drops back to searching for the
// next header, so the parser resynchronises on its own after noise.
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
        payloadLen = (uint16_t)lenBytes[0] | ((uint16_t)lenBytes[1] << 8);   // little-endian

        // Reject frames that would overflow the payload buffer.
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
          // Complete frame received.
          parseState = FIND_HDR;
          tailPos = 0;
          return true;
        }
      } else {
        // Bad tail: discard the frame and look for the next header.
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

// Read and debounce the speed-limit knob.
// The raw ADC value is smoothed with an exponential low-pass filter, and
// changes smaller than KNOB_NOISE_DEADBAND_RAW are ignored. When the
// resulting speed limit changes, the display switches to "preview" mode;
// preview ends once the knob has been still for KNOB_SETTLE_HIDE_MS.
void updateKnob(unsigned long nowMs) {
  int raw = analogRead(KNOB_ANALOG_PIN);

  // Exponential low-pass filter (seeded with the first reading)
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
    // Still moved, but not enough to change the speed limit
    if (knobPreviewActive && (nowMs - lastKnobChangeMs >= KNOB_SETTLE_HIDE_MS)) {
      knobPreviewActive = false;
    }
  }
}

// Drain all pending radar bytes, update the latest speed when a complete
// frame with a moving target arrives, and expire the reading once no
// target has been seen for RADAR_HOLD_AFTER_LOST_MS.
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

  // Hold timeout: forget the last speed once the target has been gone a while
  if (radarHasRecentSignal && (nowMs - lastRadarSignalMs > RADAR_HOLD_AFTER_LOST_MS)) {
    radarHasRecentSignal = false;
    lastRadarSpeedMph = 0.0f;
  }
}

// =====================================================
// DISPLAY LOGIC
// =====================================================

// Decide what the display should show this pass (see the priority list at
// the top of the file).
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

    bool shouldShow = true;   // at or under the limit: steady display

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
  Serial.begin(115200);         // USB debug output
  RadarSerial.begin(RADAR_BAUD);

  pinMode(KNOB_ANALOG_PIN, INPUT);

  display.setBrightness(DISPLAY_BRIGHTNESS, true);
  display.clear();

  // Initialise knob state immediately so the first loop doesn't see a
  // spurious "change", and show the current set speed at power-up.
  int raw = analogRead(KNOB_ANALOG_PIN);
  filteredKnobRaw = raw;
  lastStableKnobRaw = raw;
  currentSpeedLimitMph = knobVoltageToSpeedLimitMph(rawToVoltage(raw));
  lastKnobChangeMs = millis();
  knobPreviewActive = true;

  Serial.println("Mega + HLK-LD2451 + TM1637 speed display ready.");
  Serial.println("Radar: Serial3 RX=15");
  Serial.println("Display CLK=50 DIO=52");
  Serial.println("Knob=A1");
}

// Everything is non-blocking: each pass reads inputs, then redraws.
void loop() {
  unsigned long nowMs = millis();

  updateKnob(nowMs);
  updateRadar(nowMs);
  updateDisplay(nowMs);
}
