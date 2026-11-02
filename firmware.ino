/*
  Passive Battery Balancer + Protection  (v2)
  2S..5S Li-ion / LiFePO4  |  Target: ESP32 (Arduino-ESP32 core 2.x / 3.x)
  No external libraries (Arduino core + Espressif built-ins only).

  What it does
  - Reads each cell through resistor dividers (cumulative taps, cell = tap[k] - tap[k-1]).
    Every tap has a PC817 phototransistor in series with its R_TOP; one pin (PIN_SENSE_EN) lights all the
    opto LEDs, and only while sampling, so the dividers are disconnected from the cells the rest of the time.
  - Passive balancing: bleeds high cells only when near full (>= bal_start) and the spread is
    larger than bal_delta (hysteresis: stops at half). Bleeders are OFF during every measurement.
    Optional: balance only while a charger is present (chg_mode 1 + bal_chg_only 1).
  - Discharge FET (PIN_DSG): opens when the weakest cell stays <= uv_cut for uv_delay ms, closes again
    when it is >= uv_resume. While locked and idle the ESP32 deep-sleeps (timer + charger-plug wake).
  - Charge FET (PIN_CHG): opens on over-voltage (>= ov, closes again <= ov_resume), on a dead cell
    (< chg_min), and on charge over/under-temperature (NTC only).
  - Works with separate ports (one FET per port) or one port with back-to-back FETs: the firmware
    just drives two gates (see README).
  - Temperature: internal ESP32 sensor (coarse, bleed guard only) or external NTC (full limits).
  - Everything is tunable from the serial monitor (115200 baud, send "help").
  See README.md for wiring.
*/
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#if defined(ARDUINO_ARCH_ESP32)
  #include <Preferences.h>
  #include <esp_sleep.h>
  #include <driver/gpio.h>
#else
  #include <EEPROM.h>
#endif

// ============================ HARDWARE (edit for your board) ============================
#define MAX_CELLS 5
const uint8_t PIN_TAP[MAX_CELLS]   = {36, 39, 34, 35, 32};   // ADC1 pins. Tap k = B(k)+ measured from pack-
const uint8_t PIN_BLEED[MAX_CELLS] = {16, 17, 18, 19, 21};   // -> 330R -> PC817 LED (HIGH = bleed ON)
const uint8_t PIN_SENSE_EN = 22;   // -> 3.3k per sense-opto LED, one pin for all taps (HIGH = sampling)
const uint8_t PIN_DSG      = 23;   // discharge FET gate (HIGH = load connected)
const uint8_t PIN_CHG      = 25;   // charge FET gate    (HIGH = charging allowed)
const uint8_t PIN_CHG_DET  = 27;   // charger-present input, external 100k pull to the idle level (RTC-capable pin)
const uint8_t PIN_LED      = 2;    // status LED (onboard)
const uint8_t PIN_NTC      = 33;   // optional NTC (3V3 - 10k - pin - NTC10k - GND)

// Divider per tap (ohms): tap -> PC817 C-E -> R_TOP -> ADC pin -> R_BOT -> GND. ~2.4 V full scale at 4.25 V/cell.
const float R_TOP[MAX_CELLS] = {75000, 270000, 430000, 620000, 820000};
const float R_BOT[MAX_CELLS] = {100000, 100000, 100000, 100000, 100000};

#if !defined(ARDUINO_ARCH_ESP32)
const float ADC_REF_MV = 5000;   // generic MCU: set ADC reference and bits, and re-pick the dividers
const int   ADC_BITS   = 10;
#endif

const uint32_t LOCK_SLEEP_S  = 30;      // deep-sleep period while locked (ESP32)
const uint32_t SLEEP_IDLE_MS = 30000;   // after a cold boot, wait this long without serial activity before sleeping
const float NTC_R_FIXED = 10000.0f, NTC_R0 = 10000.0f, NTC_B = 3950.0f;
const float FAULT_LOW_MV = 1000.0f, FAULT_HIGH_MV = 4700.0f;   // implausible cell voltage = wiring fault
const float TEMP_HYST = 3.0f;                                  // degC

// ============================ CONFIG (all changeable via serial) ============================
#define CFG_MAGIC 0xBA1A0003UL
struct Cfg {
  uint32_t magic;
  uint16_t cells, chem;                                  // chem: 0 = Li-ion/LiPo, 1 = LiFePO4
  uint16_t balStart, balDelta;                           // mV
  uint16_t ov, ovResume;                                 // mV (charge FET cut / re-enable)
  uint16_t uvWarn, uvCut, uvResume, uvDelay;             // mV, mV, mV, ms
  uint16_t chgMin;                                       // mV: no charging if a cell is below this (0 = off)
  uint16_t maxBleed, period, settle;                     // count, ms, ms
  uint16_t tempSrc, tempMax, chgTmax, chgTmin, dsgTmax;  // 0 off/1 internal/2 NTC, degC
  uint16_t chgMode, chgLevel, balChgOnly;                // charger-detect pin: 0/1, active level, balance only when plugged
  float gain[MAX_CELLS];                                 // per-tap calibration
} cfg;

void applyProfile(uint16_t chem) {
  cfg.chem = chem;
  if (chem == 0) {   // Li-ion / LiPo, 4.20 V full
    cfg.balStart = 4050; cfg.balDelta = 30; cfg.ov = 4300; cfg.ovResume = 4200;
    cfg.uvWarn = 3400; cfg.uvCut = 3000; cfg.uvResume = 3300; cfg.chgMin = 2000;
  } else {           // LiFePO4, 3.65 V full
    cfg.balStart = 3400; cfg.balDelta = 30; cfg.ov = 3700; cfg.ovResume = 3600;
    cfg.uvWarn = 3100; cfg.uvCut = 2800; cfg.uvResume = 3000; cfg.chgMin = 2000;
  }
}

void setDefaults() {
  cfg.magic = CFG_MAGIC; cfg.cells = 3; applyProfile(0);
  cfg.uvDelay = 3000; cfg.maxBleed = 2; cfg.period = 5000; cfg.settle = 300;
  cfg.tempSrc = 0; cfg.tempMax = 60; cfg.chgTmax = 45; cfg.chgTmin = 0; cfg.dsgTmax = 60;
  cfg.chgMode = 0; cfg.chgLevel = 1; cfg.balChgOnly = 0;
  for (uint8_t i = 0; i < MAX_CELLS; i++) cfg.gain[i] = 1.0f;
}

#if defined(ARDUINO_ARCH_ESP32)
Preferences prefs;
void saveCfg() { prefs.begin("bal", false); prefs.putBytes("cfg", &cfg, sizeof(cfg)); prefs.end(); }
bool loadCfg() {
  Cfg t; prefs.begin("bal", false);
  size_t n = prefs.getBytes("cfg", &t, sizeof(t)); prefs.end();
  if (n == sizeof(t) && t.magic == CFG_MAGIC) { cfg = t; return true; }
  return false;
}
#else
void saveCfg() { EEPROM.put(0, cfg); }
bool loadCfg() { Cfg t; EEPROM.get(0, t); if (t.magic == CFG_MAGIC) { cfg = t; return true; } return false; }
#endif

struct Key { const char* name; uint16_t* p; uint16_t lo, hi; };
Key KEYS[] = {
  {"cells",        &cfg.cells,      2, 5},        // number of series cells
  {"bal_start",    &cfg.balStart,   2500, 4300},  // mV: balance only when a cell is above this
  {"bal_delta",    &cfg.balDelta,   5, 200},      // mV: bleed cells this far above the lowest cell
  {"bal_chg_only", &cfg.balChgOnly, 0, 1},        // 1 = balance only while charger present (needs chg_mode 1)
  {"ov",           &cfg.ov,         3000, 4500},  // mV: over-voltage -> charge FET OFF + forced bleeding
  {"ov_resume",    &cfg.ovResume,   2500, 4400},  // mV: charge FET back ON when highest cell <= this
  {"chg_min",      &cfg.chgMin,     0, 3500},     // mV: no charging if a cell is below this (0 = disabled)
  {"uv_warn",      &cfg.uvWarn,     2000, 4000},  // mV: low warning (LED)
  {"uv_cut",       &cfg.uvCut,      2000, 3500},  // mV: weakest cell this low -> discharge FET OFF
  {"uv_resume",    &cfg.uvResume,   2200, 4000},  // mV: weakest cell back above this -> discharge allowed
  {"uv_delay",     &cfg.uvDelay,    0, 60000},    // ms: must stay below uv_cut this long (ignores load sag)
  {"max_bleed",    &cfg.maxBleed,   0, 5},        // max simultaneous bleeders (heat limit)
  {"period",       &cfg.period,     1000, 60000}, // ms: measure/decide cycle
  {"settle",       &cfg.settle,     50, 2000},    // ms: bleeders off + sense optos on before measuring (RC ~10 ms)
  {"temp_src",     &cfg.tempSrc,    0, 2},        // 0 off, 1 internal ESP32 sensor (coarse), 2 external NTC
  {"temp_max",     &cfg.tempMax,    30, 90},      // degC: stop bleeding above this (internal or NTC)
  {"chg_tmax",     &cfg.chgTmax,    20, 70},      // degC: charge FET OFF above this (NTC only)
  {"chg_tmin",     &cfg.chgTmin,    0, 20},       // degC: charge FET OFF below this (NTC only)
  {"dsg_tmax",     &cfg.dsgTmax,    40, 90},      // degC: discharge FET OFF above this (NTC only)
  {"chg_mode",     &cfg.chgMode,    0, 1},        // 1 = use charger-detect pin
  {"chg_level",    &cfg.chgLevel,   0, 1},        // pin level that means "charger present"
};

// ============================ STATE ============================
float tapMv[MAX_CELLS], cellMv[MAX_CELLS];
float tempC = NAN, packMv = 0, minMv = 0, maxMv = 0;
bool  bleedOn[MAX_CELLS];
uint32_t forceUntil[MAX_CELLS];
bool fault = false, warn = false, ovLatch = false, badCell = false, anyBleed = false;
bool tempFault = false, hotBleed = false, chgTempBlock = false, dsgTempBlock = false, overTemp = false;
bool dsgOn = false, chgOn = false, chgPresent = false;
bool logOn = true, firstEval = true, firstCycle = true, uvTiming = false, wokeFromSleep = false;
uint32_t uvStart = 0, lastCycle = 0, lastActivity = 0;
#if defined(ARDUINO_ARCH_ESP32)
RTC_DATA_ATTR bool locked = false;     // survives deep sleep
#else
bool locked = false;
#endif

// ============================ MEASUREMENT ============================
#if defined(ARDUINO_ARCH_ESP32)
static inline uint32_t adcMv(uint8_t pin) { return analogReadMilliVolts(pin); }   // factory-calibrated
#else
static inline uint32_t adcMv(uint8_t pin) { return (uint32_t)((float)analogRead(pin) * ADC_REF_MV / ((1 << ADC_BITS) - 1)); }
#endif

void senseOn()  { digitalWrite(PIN_SENSE_EN, HIGH); }
void senseOff() { digitalWrite(PIN_SENSE_EN, LOW); }
void allBleedsOff() { for (uint8_t i = 0; i < MAX_CELLS; i++) digitalWrite(PIN_BLEED[i], LOW); }

float readTap(uint8_t i) {                       // returns tap voltage in mV (after divider + gain)
  adcMv(PIN_TAP[i]);                             // dummy read
  uint32_t acc = 0;
  for (uint8_t n = 0; n < 32; n++) { acc += adcMv(PIN_TAP[i]); delayMicroseconds(150); }
  return (acc / 32.0f) * (R_TOP[i] + R_BOT[i]) / R_BOT[i] * cfg.gain[i];
}

void measure() {
  float prev = 0;
  for (uint8_t i = 0; i < cfg.cells; i++) { tapMv[i] = readTap(i); cellMv[i] = tapMv[i] - prev; prev = tapMv[i]; }
}

void measureAll() {                              // bleeders off, sense optos on only while sampling
  allBleedsOff();
  senseOn();
  delay(cfg.settle);
  measure();
  senseOff();
}

float readTemp() {
  if (cfg.tempSrc == 1) {
#if defined(ARDUINO_ARCH_ESP32)
    return temperatureRead();                    // die temperature: runs warmer than ambient, +/-5 degC
#else
    return NAN;
#endif
  }
  if (cfg.tempSrc == 2) {
    float v = 0;
    for (uint8_t n = 0; n < 8; n++) v += adcMv(PIN_NTC);
    v /= 8.0f;
    if (v < 50 || v > 3250) return NAN;          // open / shorted sensor
    float r = NTC_R_FIXED * v / (3300.0f - v);
    return 1.0f / (1.0f / 298.15f + logf(r / NTC_R0) / NTC_B) - 273.15f;
  }
  return NAN;
}

void evalTemp() {
  tempC = readTemp();
  bool ok = cfg.tempSrc && !isnan(tempC);
  tempFault = cfg.tempSrc && !ok;
  bool packSensor = (cfg.tempSrc == 2);          // charge/discharge limits only make sense with a pack-mounted NTC
  hotBleed = tempFault || (ok && (tempC >= cfg.tempMax || (hotBleed && tempC > cfg.tempMax - TEMP_HYST)));
  chgTempBlock = packSensor && (tempFault || (ok &&
                 (tempC > cfg.chgTmax || tempC < cfg.chgTmin ||
                  (chgTempBlock && (tempC > cfg.chgTmax - TEMP_HYST || tempC < cfg.chgTmin + TEMP_HYST)))));
  dsgTempBlock = packSensor && ok && (tempC >= cfg.dsgTmax || (dsgTempBlock && tempC > cfg.dsgTmax - TEMP_HYST));
  overTemp = hotBleed || chgTempBlock || dsgTempBlock;
}

void readCharger() {
  if (!cfg.chgMode) { chgPresent = false; return; }
  chgPresent = (digitalRead(PIN_CHG_DET) == (cfg.chgLevel ? HIGH : LOW));
}

// ============================ OUTPUTS ============================
bool forceActive(uint8_t i) { return forceUntil[i] && (int32_t)(forceUntil[i] - millis()) > 0; }

void applyOutputs() {
  anyBleed = false;
  for (uint8_t i = 0; i < MAX_CELLS; i++) {
    bool on = (i < cfg.cells) && (bleedOn[i] || forceActive(i));
    digitalWrite(PIN_BLEED[i], on ? HIGH : LOW);
    if (on) anyBleed = true;
  }
}

// ============================ LOGIC ============================
void protect() {
  fault = false;
  for (uint8_t i = 0; i < cfg.cells; i++)
    if (cellMv[i] < FAULT_LOW_MV || cellMv[i] > FAULT_HIGH_MV) fault = true;

  if (!fault) {
    // low-cell: the weakest cell decides
    if (minMv <= cfg.uvCut) {
      if (!uvTiming) { uvTiming = true; uvStart = millis(); }
      if (firstEval || (millis() - uvStart) >= cfg.uvDelay) locked = true;   // at boot: no delay
    } else {
      uvTiming = false;
    }
    if (locked && minMv >= cfg.uvResume) locked = false;
    warn = (minMv <= cfg.uvWarn);
    // over-voltage latch for the charge FET
    if (maxMv >= cfg.ov) ovLatch = true; else if (maxMv <= cfg.ovResume) ovLatch = false;
    // dead cell: do not charge it blindly
    badCell = (cfg.chgMin > 0) && (minMv < cfg.chgMin);
  }
  firstEval = false;
  dsgOn = !fault && !locked && !dsgTempBlock;
  chgOn = !fault && !ovLatch && !badCell && !chgTempBlock;
  digitalWrite(PIN_DSG, dsgOn ? HIGH : LOW);
  digitalWrite(PIN_CHG, chgOn ? HIGH : LOW);
}

void decideBalance() {
  bool allowed = !fault && !hotBleed;
  bool balOk = allowed && !(cfg.chgMode && cfg.balChgOnly && !chgPresent);
  bool cand[MAX_CELLS] = {false};
  for (uint8_t i = 0; i < cfg.cells; i++) {
    float d = cellMv[i] - minMv;
    if (!allowed)                          cand[i] = false;
    else if (cellMv[i] >= cfg.ov)          cand[i] = true;                                              // over-voltage: always bleed
    else if (!balOk)                       cand[i] = false;
    else if (bleedOn[i])                   cand[i] = (cellMv[i] >= cfg.balStart - 20) && (d > cfg.balDelta / 2.0f);   // hysteresis
    else                                   cand[i] = (cellMv[i] >= cfg.balStart) && (d >= cfg.balDelta);
  }
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < cfg.cells; i++) if (cand[i]) cnt++;
  while (cnt > cfg.maxBleed) {                    // heat limit: keep only the highest cells
    int low = -1;
    for (uint8_t i = 0; i < cfg.cells; i++)
      if (cand[i] && (low < 0 || cellMv[i] < cellMv[low])) low = i;
    cand[low] = false; cnt--;
  }
  for (uint8_t i = 0; i < MAX_CELLS; i++) bleedOn[i] = cand[i];
}

bool bleeding(uint8_t i) { return bleedOn[i] || forceActive(i); }

void printLine() {
  char b[200]; int p = 0; uint8_t n = cfg.cells;
  for (uint8_t i = 0; i < n; i++) p += snprintf(b + p, sizeof(b) - p, "C%u=%d ", (unsigned)(i + 1), (int)(cellMv[i] + 0.5f));
  p += snprintf(b + p, sizeof(b) - p, "| pack=%d spread=%d | bleed=", (int)(packMv + 0.5f), (int)(maxMv - minMv + 0.5f));
  for (uint8_t i = 0; i < n && p < (int)sizeof(b) - 2; i++) b[p++] = bleeding(i) ? '1' : '0';
  b[p] = 0;
  Serial.print(b);
  Serial.print(" | dsg="); Serial.print(dsgOn ? 1 : 0);
  Serial.print(" chg="); Serial.print(chgOn ? 1 : 0);
  if (cfg.chgMode) { Serial.print(" plug="); Serial.print(chgPresent ? 1 : 0); }
  if (cfg.tempSrc) { Serial.print(" T="); if (isnan(tempC)) Serial.print(F("n/a")); else Serial.print(tempC, 1); }
  Serial.print(" | ");
  Serial.println(fault ? F("FAULT") : locked ? F("LOCKED(UV)") : ovLatch ? F("OVERVOLT") : badCell ? F("BADCELL") :
                 overTemp ? F("TEMP") : warn ? F("LOW") : F("OK"));
}

void maybeSleep() {
#if defined(ARDUINO_ARCH_ESP32)
  bool anyForce = false;
  for (uint8_t i = 0; i < MAX_CELLS; i++) if (forceActive(i)) anyForce = true;
  // sleep only if locked, nothing needs balancing and no charger is attached (monitoring must stay on while charging)
  bool eligible = locked && !fault && maxMv < cfg.balStart && !anyForce && !chgPresent;
  bool idle = wokeFromSleep || (millis() - lastActivity > SLEEP_IDLE_MS);
  if (!(eligible && idle)) return;

  Serial.println(F("LOCKED: weakest cell low, load off. Deep sleep (charge the pack to resume)."));
  Serial.flush();
  allBleedsOff(); senseOff(); digitalWrite(PIN_DSG, LOW);
  if (cfg.chgMode) {                              // charge FET off while asleep; wake when a charger is plugged in
    digitalWrite(PIN_CHG, LOW);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_CHG_DET, cfg.chgLevel ? 1 : 0);
  } else if (chgOn) {                             // no plug detect: keep the charge path open so a charger can recover the pack
    digitalWrite(PIN_CHG, HIGH);
    gpio_hold_en((gpio_num_t)PIN_CHG);
    gpio_deep_sleep_hold_en();
  }
  esp_sleep_enable_timer_wakeup((uint64_t)LOCK_SLEEP_S * 1000000ULL);
  esp_deep_sleep_start();
#endif
}

void cycle() {
  measureAll();                                   // bleeders off, optos on, sample, optos off
  minMv = 1e9f; maxMv = 0;
  for (uint8_t i = 0; i < cfg.cells; i++) { if (cellMv[i] < minMv) minMv = cellMv[i]; if (cellMv[i] > maxMv) maxMv = cellMv[i]; }
  packMv = tapMv[cfg.cells - 1];
  evalTemp();
  readCharger();
  protect();
  decideBalance();
  applyOutputs();
  if (logOn) printLine();
  maybeSleep();
}

// ============================ LED ============================
void ledUpdate() {
  uint32_t t = millis(); bool on;
  if (fault)                          on = (t / 100) % 2;                                   // 5 Hz
  else if (locked)                    on = (t % 2000) < 1000;                               // 0.5 Hz
  else if (ovLatch || badCell || overTemp) on = (t % 600) < 300;                            // ~3 Hz
  else if (warn)                      on = ((t % 1200) < 100) || ((t % 1200) > 250 && (t % 1200) < 350);  // double blink
  else if (anyBleed)                  on = (t % 1000) < 500;                                // 1 Hz = balancing
  else                                on = (t % 3000) < 60;                                 // heartbeat
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// ============================ SERIAL ============================
void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  status                 live line + all settings"));
  Serial.println(F("  set <key> <value>      change a parameter (mV / ms / degC / count); keys listed by 'status'"));
  Serial.println(F("  profile liion|lfp      load default thresholds for the chemistry"));
  Serial.println(F("  cal <tap> <mV>         calibrate: meter reading pack- -> tap (e.g. cal 3 12480)"));
  Serial.println(F("  test <cell> <sec>      force a bleeder ON for <sec> (0 = off, max 60)"));
  Serial.println(F("  log 0|1                periodic log off/on"));
  Serial.println(F("  save | defaults        store to flash | reset to defaults (then 'save')"));
}

void printCfg() {
  if (!firstEval) printLine();
  Serial.print(F("chem=")); Serial.println(cfg.chem ? F("LiFePO4") : F("Li-ion"));
  for (auto& e : KEYS) { Serial.print(e.name); Serial.print('='); Serial.println(*e.p); }
  Serial.print(F("gain:"));
  for (uint8_t i = 0; i < cfg.cells; i++) { Serial.print(' '); Serial.print(cfg.gain[i], 4); }
  Serial.println();
}

void cmdSet(char* k, char* v) {
  long val = atol(v);
  for (auto& e : KEYS) {
    if (strcmp(k, e.name)) continue;
    if (val < e.lo || val > e.hi) { Serial.print(F("out of range ")); Serial.print(e.lo); Serial.print('-'); Serial.println(e.hi); return; }
    *e.p = (uint16_t)val;
    Serial.println(F("ok (use 'save' to keep)"));
    if (cfg.uvCut >= cfg.uvResume) Serial.println(F("WARNING: uv_cut should be lower than uv_resume"));
    if (cfg.ovResume >= cfg.ov)    Serial.println(F("WARNING: ov_resume should be lower than ov"));
    return;
  }
  Serial.println(F("unknown key"));
}

void cmdCal(char* a, char* b) {
  int n = a ? atoi(a) : 0; float ref = b ? atof(b) : 0;
  if (n < 1 || n > cfg.cells || ref < 500) { Serial.println(F("usage: cal <tap 1..cells> <mV from meter, pack- to tap>")); return; }
  uint8_t i = n - 1;
  allBleedsOff(); senseOn(); delay(cfg.settle);
  float old = cfg.gain[i]; cfg.gain[i] = 1.0f;
  float raw = readTap(i), g = ref / raw;
  senseOff();
  if (g < 0.8f || g > 1.2f) { cfg.gain[i] = old; Serial.println(F("gain outside 0.8..1.2 - check wiring / divider values")); return; }
  cfg.gain[i] = g;
  Serial.print(F("tap ")); Serial.print(n); Serial.print(F(" gain=")); Serial.println(g, 4);
}

void handleLine(char* s) {
  char* cmd = strtok(s, " \t"); if (!cmd) return;
  char* a = strtok(NULL, " \t"); char* b = strtok(NULL, " \t");
  for (char* p = cmd; *p; p++) *p = tolower(*p);
  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) printHelp();
  else if (!strcmp(cmd, "status") || !strcmp(cmd, "get")) printCfg();
  else if (!strcmp(cmd, "set") && a && b) cmdSet(a, b);
  else if (!strcmp(cmd, "profile") && a) {
    applyProfile(tolower(a[0]) == 'l' ? 1 : 0);                    // "lfp" -> LiFePO4, anything else -> Li-ion
    Serial.println(F("profile applied (use 'save' to keep)"));
  }
  else if (!strcmp(cmd, "cal")) cmdCal(a, b);
  else if (!strcmp(cmd, "test") && a) {
    int n = atoi(a), sec = b ? atoi(b) : 10;
    if (n < 1 || n > cfg.cells) { Serial.println(F("bad cell")); return; }
    sec = constrain(sec, 0, 60);
    forceUntil[n - 1] = sec ? (millis() + sec * 1000UL) : 0;
    Serial.println(F("ok"));
  }
  else if (!strcmp(cmd, "log") && a) logOn = atoi(a) != 0;
  else if (!strcmp(cmd, "save")) { saveCfg(); Serial.println(F("saved")); }
  else if (!strcmp(cmd, "defaults")) { setDefaults(); Serial.println(F("defaults loaded (use 'save' to keep)")); }
  else Serial.println(F("? type 'help'"));
}

char lineBuf[64]; uint8_t lineLen = 0;
void serialPoll() {
  while (Serial.available()) {
    char c = Serial.read(); lastActivity = millis();
    if (c == '\r' || c == '\n') { if (lineLen) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; } }
    else if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
  }
}

// ============================ MAIN ============================
void setup() {
#if defined(ARDUINO_ARCH_ESP32)
  gpio_hold_dis((gpio_num_t)PIN_CHG);                              // release the pin held during deep sleep
  gpio_deep_sleep_hold_dis();
#endif
  pinMode(PIN_DSG, OUTPUT); digitalWrite(PIN_DSG, LOW);            // fail-safe: both FETs OFF until the first valid check
  pinMode(PIN_CHG, OUTPUT); digitalWrite(PIN_CHG, LOW);
  pinMode(PIN_SENSE_EN, OUTPUT); digitalWrite(PIN_SENSE_EN, LOW);
  for (uint8_t i = 0; i < MAX_CELLS; i++) { pinMode(PIN_BLEED[i], OUTPUT); digitalWrite(PIN_BLEED[i], LOW); }
  pinMode(PIN_CHG_DET, INPUT);                                     // use an external 100k pull to the idle level
  pinMode(PIN_LED, OUTPUT);
#if defined(ARDUINO_ARCH_ESP32)
  setCpuFrequencyMhz(80);                                          // lower idle current, WiFi/BT are never started
  for (uint8_t i = 0; i < MAX_CELLS; i++) analogSetPinAttenuation(PIN_TAP[i], ADC_11db);
  analogSetPinAttenuation(PIN_NTC, ADC_11db);
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  wokeFromSleep = (cause == ESP_SLEEP_WAKEUP_TIMER || cause == ESP_SLEEP_WAKEUP_EXT0);
#endif
  Serial.begin(115200);
  if (!loadCfg()) setDefaults();
  lastActivity = millis();
  if (!wokeFromSleep) { Serial.println(F("\nPassive balancer ready. Type 'help'.")); }
}

void loop() {
  serialPoll();
  if (firstCycle || (millis() - lastCycle) >= cfg.period) { firstCycle = false; lastCycle = millis(); cycle(); }
  ledUpdate();
  delay(2);
}
