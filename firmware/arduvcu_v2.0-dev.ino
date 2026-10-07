/* =====================================================
   ArduVCU Firmware v2.0-dev
   Refactor of the CREST Gold Release v1.3 (April 2026)
   Greenpower F24 — predictive telemetry & battery management

   STATUS: development refactor, hardware validation pending.
   v1.3 remains the assessed Gold Release until this is
   bench-tested against v1.3 behaviour.

   Changes vs v1.3:
     1. analogReadMilliVolts() — eFuse-calibrated ADC readings
     2. O(1) circular-buffer filtering on BOTH voltage and current
     3. Non-blocking 50 Hz sampling, 2 Hz telemetry (was 2 Hz delay())
     4. Live discharge-gradient telemetry vs the 0.05 V/min
        pacing target from the Excel digital twin
     5. NTC thermal monitoring with 90 C emergency cutoff
        (per the project risk assessment)
     6. Minimum 2 s relay dwell — contactor protection
     7. Fail-safe boot: relay OFF until first healthy reading
     8. Safety commands retried every pass — a dwell-blocked
        trip can never be silently lost; thermal trip needs
        2 consecutive 1 Hz readings (no noise-spike shutdowns)
   ===================================================== */

#include <math.h>

// ---------- PIN MAP (unchanged from v1.3) ----------
const int VOLT_PIN  = 34;   // 100k/10k divider (11:1), ADC1, input-only
const int AMP_PIN   = 35;   // ACS712-30A output,     ADC1, input-only
const int NTC_PIN   = 32;   // motor NTC 10k (B3950), ADC1, input-only
const int RELAY_PIN = 25;   // motor safety contactor

// ---------- SIGNAL FILTERING (O(1) circular buffers) ----------
const int   NUM_SAMPLES  = 10;     // per-channel ring depth
const int   SAMPLE_MS     = 20;    // 50 Hz filter update
const float NOISE_GATE_A  = 0.30f; // ACS712 quiescent noise band

// ---------- BATTERY MODEL (Greenpower regulations) ----------
const float CAPACITY_AH   = 20.0f;
const float PEUKERT_K     = 1.15f;  // lead-acid
const float DIVIDER_RATIO = 11.0f;   // (100k + 10k) / 10k

// ---------- PACING (Excel digital-twin target) ----------
const float TARGET_V_PER_MIN = 0.05f;
const int   SLOPE_WINDOW_S   = 60;   // rolling gradient window

// ---------- HYSTERESIS SAFETY (v1.3 values kept) ----------
const float CUTOFF_V      = 21.0f;   // deep sag -> Limp Mode
const float HYST_BUFFER_V = 1.5f;    // recovery dead-band
const unsigned long MIN_RELAY_DWELL_MS = 2000; // contactor protection

// ---------- THERMAL (per project risk assessment) ----------
const float NTC_PULLUP_OHMS    = 10000.0f;
const float NTC_R0_OHMS        = 10000.0f;  // at 25 C
const float NTC_BETA           = 3950.0f;
const float THERMAL_CUTOFF_C   = 90.0f;
const float THERMAL_REARM_C    = 70.0f;      // thermal dead-band

// ---------- TELEMETRY ----------
const unsigned long TELEMETRY_MS = 500;      // 2 Hz, as v1.3

// ---------- FILTER STATE ----------
long  voltTotal = 0, ampTotal = 0;
int   voltRing[NUM_SAMPLES], ampRing[NUM_SAMPLES];
int   ringIndex = 0;
float smoothedVolt = 0.0f, smoothedAmps = 0.0f;

// ---------- SAFETY STATE ----------
bool  limpMode    = false;
bool  thermalTrip = false;
bool  relayState  = false;       // software mirror of the contactor pin
bool  tempValid   = false;       // refreshed at the 1 Hz tick
int   hotCount    = 0;           // consecutive 1 Hz overheat readings
unsigned long lastRelayChange = 0;
float tempC = NAN;               // shared global, refreshed at 1 Hz (FAULT till then)

// ---------- SLOPE STATE (1 Hz voltage history) ----------
const int SLOPE_N = SLOPE_WINDOW_S + 1;
float slopeRing[SLOPE_N];
int   slopeIndex = 0;
int   slopeFilled = 0;           // counts up to SLOPE_N, then gradient is valid
float dischargeVPerMin = 0.0f;

unsigned long lastSample = 0, lastSlope = 0, lastTelemetry = 0;

// ---------------- HELPERS ----------------
float readTempC() {
  // NTC high side to 3V3, fixed 10k low side to GND
  float mv   = analogReadMilliVolts(NTC_PIN);
  if (mv <= 1.0f || mv >= 3299.0f) return NAN;  // wiring fault
  float rNtc = NTC_PULLUP_OHMS * (3300.0f - mv) / mv;
  float tK   = 1.0f / (1.0f / 298.15f + logf(rNtc / NTC_R0_OHMS) / NTC_BETA);
  return tK - 273.15f;
}

void updateFilters() {
  // O(1) circular buffers: subtract oldest, add newest, modulo wrap
  voltTotal -= voltRing[ringIndex];
  ampTotal  -= ampRing[ringIndex];
  voltRing[ringIndex] = analogReadMilliVolts(VOLT_PIN);
  ampRing[ringIndex]  = analogReadMilliVolts(AMP_PIN);
  voltTotal += voltRing[ringIndex];
  ampTotal  += ampRing[ringIndex];
  ringIndex = (ringIndex + 1) % NUM_SAMPLES;

  smoothedVolt = (float)voltTotal / NUM_SAMPLES * DIVIDER_RATIO / 1000.0f;
  float amps = ((float)ampTotal / NUM_SAMPLES - 1650.0f) / 66.0f;
  smoothedAmps = (fabs(amps) < NOISE_GATE_A) ? 0.0f : amps;
}

bool relayAllowed() {
  // contactor dwell: no relay transition inside the minimum window
  return (millis() - lastRelayChange) >= MIN_RELAY_DWELL_MS;
}

void setRelay(bool on) {
  if (on == relayState || !relayAllowed()) return;
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  relayState = on;
  lastRelayChange = millis();
}

void safetyStateMachine() {
  // --- thermal trip (hotCount is incremented only at the 1 Hz tick) ---
  if (!thermalTrip && hotCount >= 2) {         // 2 s sustained overheat, not a spike
    thermalTrip = true;
    limpMode = true;
  }
  if (thermalTrip && tempValid && tempC < THERMAL_REARM_C) {
    thermalTrip = false;                       // re-arm only below the dead-band
  }

  if (thermalTrip) { setRelay(false); return; }  // retried EVERY pass: can't be lost

  // --- voltage hysteresis (v1.3 logic, dwell-protected) ---
  if (smoothedVolt <= CUTOFF_V) {
    limpMode = true;                        // latch the flag...
    setRelay(false);                        // ...and command OFF every pass — a
                                             // dwell-blocked call is retried, never lost
  } else if (limpMode && smoothedVolt >= (CUTOFF_V + HYST_BUFFER_V)) {
    limpMode = false;
    setRelay(true);                         // recovered past the 1.5 V dead-band
  } else if (!limpMode) {
    setRelay(true);                         // NOMINAL: engage after healthy boot
  }
}

void updateSlope() {
  slopeRing[slopeIndex] = smoothedVolt;
  slopeIndex = (slopeIndex + 1) % SLOPE_N;
  if (slopeFilled < SLOPE_N) { slopeFilled++; return; }  // first 60 s = warm-up
  float oldest = slopeRing[slopeIndex];          // value 60 s ago
  dischargeVPerMin = (oldest - smoothedVolt);   // +ve = discharging
}

void telemetry() {
  Serial.print(F("BATT: "));
  Serial.print(smoothedVolt, 2);
  Serial.print(F("V | LOAD: "));
  Serial.print(fabs(smoothedAmps), 2);
  Serial.print(F("A"));

  if (fabs(smoothedAmps) > NOISE_GATE_A) {
    // Peukert: effective runtime shrinks as current rises
    float hoursRem = CAPACITY_AH / powf(fabs(smoothedAmps), PEUKERT_K);
    Serial.print(F(" | EST: "));
    Serial.print(hoursRem, 2);
    Serial.print(F("h"));
  } else {
    Serial.print(F(" | EST: IDLE (no load)"));
  }

  Serial.print(F(" | SLOPE: "));
  if (slopeFilled < SLOPE_N) {
    Serial.print(F("WARMUP"));                  // don't fake a 0.000 "on pace" reading
  } else {
    Serial.print(dischargeVPerMin, 3);
    Serial.print(F("V/min (target "));
    Serial.print(TARGET_V_PER_MIN, 2);
    Serial.print(F(")"));
  }

  Serial.print(F(" | TEMP: "));
  if (tempC != tempC) { Serial.print(F("FAULT")); }   // NAN check
  else                { Serial.print(tempC, 1); Serial.print(F("C")); }

  Serial.print(F(" | STATUS: "));
  if (thermalTrip)     Serial.println(F("THERMAL_SHUTDOWN"));
  else if (limpMode)   Serial.println(F("LIMP_MODE_ACTIVE"));
  else                  Serial.println(F("NOMINAL"));
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);          // FAIL-SAFE: boot OFF

  // Pre-fill both buffers with LIVE data (prevents false trip on boot)
  for (int i = 0; i < NUM_SAMPLES; i++) {
    voltRing[i] = analogReadMilliVolts(VOLT_PIN);
    ampRing[i]  = analogReadMilliVolts(AMP_PIN);
    voltTotal += voltRing[i];
    ampTotal  += ampRing[i];
    delay(10);                            // ADC mux settle, as v1.3
  }
  for (int i = 0; i < SLOPE_N; i++) slopeRing[i] = 0.0f;

  updateFilters();
  Serial.println(F("--- ArduVCU v2.0-dev SYSTEMS ONLINE ---"));
  Serial.println(F("Waiting for telemetry stream..."));
  // Relay engages only after the first healthy state-machine pass
}

void loop() {
  unsigned long now = millis();

  if (now - lastSample >= (unsigned)SAMPLE_MS) {     // 50 Hz
    lastSample = now;
    updateFilters();
  }

  if (now - lastSlope >= 1000UL) {                   // 1 Hz gradient + thermal
    lastSlope = now;
    updateSlope();
    tempC = readTempC();                            // 1 Hz: no noise-splice trips
    tempValid = (tempC == tempC);                   // NaN self-check
    if (tempValid && tempC >= THERMAL_CUTOFF_C) hotCount++;   // consecutive readings
    else hotCount = 0;
  }

  safetyStateMachine();

  if (now - lastTelemetry >= TELEMETRY_MS) {         // 2 Hz
    lastTelemetry = now;
    telemetry();
  }
  // No delay(): loop is non-blocking — ready for buttons/display later
}
