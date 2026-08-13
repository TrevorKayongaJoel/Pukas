/* =============================================================================
 * ESP32 Automatic Weather Station - time-based sampling with relay power gating
 * =============================================================================
 *
 * WHAT CHANGED AND WHY
 * --------------------
 * The previous firmware was interrupt-driven and ran a blocking `delay(2000)`
 * loop with the sensor relay permanently energised. This version replaces both:
 *
 *   1. NO INTERRUPTS.  There is not a single attachInterrupt() or ISR in this
 *      file. Wind and rain pulses are counted by the ESP32's PCNT (Pulse
 *      Counter) peripheral - dedicated silicon that tallies edges completely
 *      independently of the CPU. The firmware simply reads a register on a
 *      schedule. This is genuinely time-based, and unlike polling a GPIO it
 *      cannot miss a pulse: PCNT keeps counting while the CPU is busy in an
 *      I2C transaction, a DHT22 read, or anything else.
 *
 *      Naive polling was rejected on purpose. A rain tip closes the contact for
 *      only a few milliseconds, so catching it by polling needs a ~2 ms loop -
 *      500 CPU wakeups a second, more power than the interrupts it replaced,
 *      and it *still* drops pulses whenever a sensor read blocks for longer
 *      than one pulse. Lost rain tips are unrecoverable: rainfall is
 *      cumulative, so every dropped tip is 0.2 mm permanently missing and the
 *      error only ever grows in one direction.
 *
 *   2. NO BLOCKING DELAYS.  loop() never calls delay(). Everything is a
 *      non-blocking state machine driven by wrap-safe millis() arithmetic
 *      (`millis() - last >= interval`, which survives the 49.7-day rollover;
 *      the `millis() >= deadline` form does NOT and would silently stop the
 *      station for ~49 days).
 *
 *   3. RELAY IS ACTUALLY USED.  The sensor rail is energised only for the
 *      ~2.3 s it takes to warm up and read the gated sensors, then switched
 *      off again. At the default 300 s interval that is a ~0.8% duty cycle.
 *
 * POWER TOPOLOGY (matches the existing board - no hardware changes)
 * ----------------------------------------------------------------
 *   ALWAYS POWERED   Wind reed switch (GPIO27), rain reed switch (GPIO26).
 *                    These are passive contacts - a magnet closes two pieces
 *                    of metal. They draw essentially nothing, so gating them
 *                    would save no power and would destroy data. They are
 *                    counted continuously, 24/7, including while the sensor
 *                    rail is off and while the console is in FORCE_OFF.
 *
 *   RELAY GATED      Everything on the sensor rail switched by GPIO13:
 *                    ADS1115, both INA219s, DS3231 RTC, and the DHT22.
 *                    (If the DHT22 turns out to be on permanent power, this
 *                    code still works unchanged - see readSensors().)
 *
 * CONSEQUENCES OF GATING THAT THIS CODE HANDLES
 * ---------------------------------------------
 *   - RE-INIT. A chip that loses power comes back at factory defaults. All
 *     begin()/config calls therefore live in sensorsInit(), which runs on
 *     EVERY rail power-up, not once in setup().
 *
 *   - WARMUP. Readings are invalid until the rail settles. The DHT22 is the
 *     bottleneck at ~2 s; the I2C parts need tens of ms. SENSOR_WARMUP_MS is
 *     sized for the slowest part and is waited out non-blockingly.
 *
 *   - I2C BACK-POWERING. This is the classic power-gated-I2C bug. If SDA/SCL
 *     stay pulled high while a sensor's Vcc is dead, current flows backwards
 *     through the sensor's ESD protection diodes into its supply pin. The chip
 *     half-wakes: NACKs, corrupt readings, occasional bus lockups that only
 *     appear after a power cycle. railOff() therefore calls Wire.end() and
 *     releases SDA/SCL/DHT to high-Z with no pull-up.
 *
 *   - RTC. The DS3231 is on the gated rail, so it can only be read during a
 *     sample window - which is fine, that is the only time we need a
 *     timestamp. It does need a healthy coin cell to keep time across power
 *     cuts; sensorsInit() checks rtc.lostPower() and warns loudly if not.
 *
 *   - RELAY WEAR. A mechanical relay is good for roughly 1e5 operations under
 *     load. Cycling once a minute is ~525k/year, i.e. failure inside a year in
 *     a box on a pole. Two mitigations here: below MIN_GATING_INTERVAL_S the
 *     rail is simply left on (cycling that fast saves nothing and burns life),
 *     and the console reports a running cycle count so wear is visible.
 *
 * KNOWN LIMITS - PLEASE READ
 * --------------------------
 *   - PCNT's hardware glitch filter maxes out at 1023 APB clocks = 12.8 us at
 *     80 MHz APB. That kills electrical noise and EMI spikes but NOT mechanical
 *     contact bounce, which lasts milliseconds. So rain gets an additional
 *     time-based software debounce (see serviceRainDebounce). Wind is left
 *     unfiltered beyond the 12.8 us, exactly as the previous firmware had it -
 *     if wind readings look inflated, an RC filter on the board is the fix.
 *
 *   - PCNT is clocked from APB, which is gated in light sleep. This firmware
 *     therefore does NOT light-sleep: the counters would stop. Power saving
 *     here comes from the relay plus a reduced CPU clock. Going to light or
 *     deep sleep later means moving pulse counting to the ULP coprocessor or
 *     an external counter chip.
 *
 *   - The PCNT counter is 16-bit. Counters free-run and are read as deltas
 *     (never cleared), so wraparound is handled correctly and there is no
 *     read-then-clear race, but a single interval must not exceed 32767
 *     pulses. At maximum wind that is ~24 minutes, so MAX_SAMPLE_INTERVAL_S
 *     is capped at 3600 s with margin to spare.
 *
 * SERIAL CONSOLE - type a command and press Enter
 * -----------------------------------------------
 *   h        help                    s        sample immediately
 *   t        status                  r        reset rainfall total
 *   a        AUTO rail mode          1        force rail permanently ON
 *   0        force rail OFF (sampling paused; wind/rain still counted)
 *   i<sec>   set sample interval, e.g. "i300" = 5 min, "i10" for bench testing
 * ========================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <DHT.h>
#include "driver/pcnt.h"

// ---- Pin definitions (UNCHANGED from the original firmware) ----
#define RELAY_PIN    13
#define RELAY_ACTIVE HIGH
#define I2C_SDA      21
#define I2C_SCL      22
#define WIND_PIN     27    // Davis 6410 anemometer (active-low pulse)
#define RAIN_PIN     26    // Davis rain gauge (active-low pulse)
#define DHTPIN       25    // DHT22 data pin

// ---- DHT Type ----
#define DHTTYPE DHT22

// =============================== CONFIGURATION ===============================
// Everything tunable lives here. Nothing below this block should need editing
// to change the station's behaviour.

// --- Sampling cadence -------------------------------------------------------
static const uint32_t DEFAULT_SAMPLE_INTERVAL_S = 300;   // 5 minutes
static const uint32_t MIN_SAMPLE_INTERVAL_S     = 5;
static const uint32_t MAX_SAMPLE_INTERVAL_S     = 3600;  // see PCNT 16-bit note

// --- Relay / sensor rail ----------------------------------------------------
// Time from rail-on to first valid reading. Sized for the DHT22 (~2 s), which
// is far slower to settle than any of the I2C parts.
static const uint32_t SENSOR_WARMUP_MS     = 2200;
// Below this interval, gating costs more relay life than the power it saves,
// so the rail is left permanently energised instead.
static const uint32_t MIN_GATING_INTERVAL_S = 30;

// --- Wind gust tracking -----------------------------------------------------
// A single count over the whole interval gives you AVERAGE wind only. 100
// pulses in 5 minutes reads identically whether the wind was steady or dead
// calm with one violent burst - gust cannot be recovered after the fact. To
// report gust we sub-sample the counter on a short cadence and keep the peak.
// 3 s is the meteorological standard gust window. Set to false to skip the
// sub-sampling entirely (no gust data, marginally less CPU work).
static const bool     GUST_TRACKING_ENABLED = true;
static const uint32_t GUST_SAMPLE_MS        = 3000;

// --- Rain -------------------------------------------------------------------
// PCNT guarantees the tip is registered; this debounce collapses the contact
// bounce that follows it. 100 ms matches the previous firmware. Two genuine
// tips inside 100 ms would be 0.4 mm / 0.1 s = ~52,000 mm/hr, so collapsing
// bursts is safe.
static const uint32_t RAIN_POLL_MS     = 20;
static const uint32_t RAIN_DEBOUNCE_MS = 100;
static const float    MM_PER_TIP       = 0.2f;

// --- Wind calibration -------------------------------------------------------
// !! REVIEW BEFORE PRODUCTION !!
// Carried over unchanged from the original firmware so this rewrite does not
// silently alter your dataset. Be aware: the Davis 6410 specification is
//     V(mph) = 2.25 * P / T
// so the correct constant for a 6410 is 2.25, and 1.5 reads ~33% LOW. The
// previous m/s constant (0.67) was internally consistent with 1.5
// (1.5 * 0.44704 = 0.67), so both were off by the same factor. Confirm the
// actual anemometer model, then set this to 2.25 if it is a 6410.
static const float WIND_MPH_PER_HZ = 1.5f;
static const float MPH_TO_MS       = 0.44704f;

// --- CPU clock --------------------------------------------------------------
// 80 MHz roughly halves CPU current versus 240 MHz. APB stays at 80 MHz, so
// PCNT, I2C and UART are all unaffected. Set to 240 to restore stock speed.
static const uint32_t CPU_FREQ_MHZ = 80;

// --- PCNT ---------------------------------------------------------------
static const pcnt_unit_t PCNT_UNIT_WIND = PCNT_UNIT_0;
static const pcnt_unit_t PCNT_UNIT_RAIN = PCNT_UNIT_1;
static const uint16_t    PCNT_FILTER_MAX = 1023;   // hardware maximum

// ---- Global Objects ----
Adafruit_ADS1115     ads;
Adafruit_INA219      batterySensor(0x40);
Adafruit_INA219      solarSensor(0x41);
RTC_DS3231           rtc;
DHT dht(DHTPIN, DHTTYPE);

// ---- Rail control ----
enum RailMode { RAIL_AUTO, RAIL_FORCE_ON, RAIL_FORCE_OFF };
static RailMode railMode      = RAIL_AUTO;
static bool     railPowered   = false;
static uint32_t railOnAtMs    = 0;
static uint32_t relayCycles   = 0;
static uint32_t lastRailOnDurationMs = 0;

// ---- Sample cycle state machine ----
enum CycleState { CYCLE_IDLE, CYCLE_WARMUP };
static CycleState cycleState        = CYCLE_IDLE;
static uint32_t   sampleIntervalS   = DEFAULT_SAMPLE_INTERVAL_S;
static uint32_t   lastCycleStartMs  = 0;
static uint32_t   cycleCount        = 0;
static bool       sampleRequested   = false;

// ---- Pulse counter bookkeeping (all plain variables - no ISR, no volatile) --
static int16_t  windRawPrev = 0;
static int16_t  rainRawPrev = 0;
static uint32_t windPulsesInterval = 0;   // accumulated since last record
static uint32_t windPulsesGustWin  = 0;   // accumulated since last gust sample
static uint32_t rainTipsInterval   = 0;   // debounced tips since last record
static uint32_t rainTipsTotal      = 0;
static float    gustMaxMph         = 0.0f;
static uint32_t lastGustSampleMs   = 0;
static uint32_t lastRainPollMs     = 0;
static uint32_t lastAcceptedTipMs  = 0;
static uint32_t intervalStartMs    = 0;

// ---- Latest record ----
struct WeatherRecord {
  char     timestamp[24];
  uint32_t intervalMs;      // period this record actually covers
  float    windAvgMph, windAvgMs, windGustMph, windGustMs;
  uint32_t rainTips;
  float    rainIntervalMm, rainTotalMm;
  float    temperature, humidity;
  float    solarV, solarMa, solarW;
  float    battV, battMa, battW;
  float    adc[4];
  bool     adsOk, solarOk, battOk, rtcOk, dhtOk;
};
static WeatherRecord rec;

// ---- Persisted last-good values (survive a failed read, as before) ----
static float lastGoodTemperature = NAN;
static float lastGoodHumidity    = NAN;
static uint32_t dhtFailures      = 0;

// ---- Serial console buffer ----
static char     cmdBuf[32];
static uint8_t  cmdLen = 0;
static uint32_t lastCmdCharMs = 0;

// ============================================================================
// PULSE COUNTING - ESP32 PCNT peripheral
// ============================================================================
//
// Each reed switch gets its own PCNT unit. Configuration counts FALLING edges
// only (the contact pulls the pin to ground against the internal pull-up),
// which matches the original attachInterrupt(..., FALLING) semantics exactly.
//
// The counters are never cleared after setup. They free-run and we read
// deltas in unsigned 16-bit arithmetic, which (a) wraps correctly at the
// counter's 16-bit boundary and (b) removes the read-then-clear race where a
// pulse arriving between the two calls would be lost.

static void pcntSetupUnit(pcnt_unit_t unit, int gpio) {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = gpio;
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  cfg.lctrl_mode     = PCNT_MODE_KEEP;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.pos_mode       = PCNT_COUNT_DIS;   // ignore rising edge
  cfg.neg_mode       = PCNT_COUNT_INC;   // count falling edge
  cfg.counter_h_lim  = 0;                // limits disabled -> free-running
  cfg.counter_l_lim  = 0;
  cfg.unit           = unit;
  cfg.channel        = PCNT_CHANNEL_0;

  pcnt_unit_config(&cfg);

  // pcnt_unit_config() claims the pin but does not set its pull mode, and the
  // reed switch only ever pulls DOWN - without this the input floats.
  gpio_set_pull_mode((gpio_num_t)gpio, GPIO_PULLUP_ONLY);

  // Explicitly disable every limit/threshold event. The PCNT hardware zeroes
  // the counter whenever an *enabled* limit event fires, and the reset state of
  // the enable bits is not something to rely on - with a limit value of 0 that
  // would clear the counter continuously and nothing would ever be counted.
  // Disabling them is what makes the counter genuinely free-running.
  pcnt_event_disable(unit, PCNT_EVT_H_LIM);
  pcnt_event_disable(unit, PCNT_EVT_L_LIM);
  pcnt_event_disable(unit, PCNT_EVT_THRES_0);
  pcnt_event_disable(unit, PCNT_EVT_THRES_1);
  pcnt_event_disable(unit, PCNT_EVT_ZERO);
  pcnt_intr_disable(unit);   // no ISR anywhere in this firmware

  // Hardware glitch filter. Rejects anything shorter than 1023 APB clocks
  // (12.8 us @ 80 MHz). Kills EMI spikes; see header note on bounce.
  pcnt_set_filter_value(unit, PCNT_FILTER_MAX);
  pcnt_filter_enable(unit);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

static void pulseCountersInit() {
  pcntSetupUnit(PCNT_UNIT_WIND, WIND_PIN);
  pcntSetupUnit(PCNT_UNIT_RAIN, RAIN_PIN);

  pcnt_get_counter_value(PCNT_UNIT_WIND, &windRawPrev);
  pcnt_get_counter_value(PCNT_UNIT_RAIN, &rainRawPrev);
}

// Returns edges seen since the previous call. Wrap-safe.
static uint16_t pcntTakeDelta(pcnt_unit_t unit, int16_t &prev) {
  int16_t raw = 0;
  pcnt_get_counter_value(unit, &raw);
  uint16_t delta = (uint16_t)raw - (uint16_t)prev;
  prev = raw;
  return delta;
}

// ---------------------------------------------------------------------------
// Wind: every falling edge is a genuine revolution, so accumulate all of them.
static void serviceWindCounter() {
  uint16_t delta = pcntTakeDelta(PCNT_UNIT_WIND, windRawPrev);
  if (delta) {
    windPulsesInterval += delta;
    if (GUST_TRACKING_ENABLED) windPulsesGustWin += delta;
  }
}

// Gust: sample the counter every GUST_SAMPLE_MS, convert that short window to
// a speed, and keep the interval's peak.
static void serviceGustSampling() {
  if (!GUST_TRACKING_ENABLED) return;

  uint32_t now = millis();
  uint32_t dt  = now - lastGustSampleMs;
  if (dt < GUST_SAMPLE_MS) return;

  float mph = ((float)windPulsesGustWin * WIND_MPH_PER_HZ * 1000.0f) / (float)dt;
  if (mph > gustMaxMph) gustMaxMph = mph;

  windPulsesGustWin = 0;
  lastGustSampleMs  = now;
}

// ---------------------------------------------------------------------------
// Rain: PCNT has already guaranteed the tip was captured no matter how brief.
// All this does is collapse the contact bounce that follows it - at most one
// accepted tip per RAIN_DEBOUNCE_MS, extra edges in that window discarded.
static void serviceRainDebounce() {
  uint32_t now = millis();
  if (now - lastRainPollMs < RAIN_POLL_MS) return;
  lastRainPollMs = now;

  uint16_t delta = pcntTakeDelta(PCNT_UNIT_RAIN, rainRawPrev);
  if (delta == 0) return;

  if (now - lastAcceptedTipMs >= RAIN_DEBOUNCE_MS) {
    rainTipsInterval++;
    rainTipsTotal++;
    lastAcceptedTipMs = now;
  }
  // Any remaining edges in `delta` are bounce from the tip just accepted.
}

static void servicePulseCounters() {
  serviceWindCounter();
  serviceGustSampling();
  serviceRainDebounce();
}

// ============================================================================
// SENSOR RAIL - relay control
// ============================================================================

static void railOn() {
  if (railPowered) return;
  digitalWrite(RELAY_PIN, RELAY_ACTIVE);
  railPowered = true;
  railOnAtMs  = millis();
  relayCycles++;
}

static void railOff() {
  if (!railPowered) return;

  // Release the bus BEFORE cutting power, so no pin is left driving or pulling
  // up into a rail that is about to go dead (see back-powering note in header).
  Wire.end();
  pinMode(I2C_SDA, INPUT);   // high-Z, no pull-up
  pinMode(I2C_SCL, INPUT);
  pinMode(DHTPIN,  INPUT);

  digitalWrite(RELAY_PIN, !RELAY_ACTIVE);
  railPowered = false;
  lastRailOnDurationMs = millis() - railOnAtMs;
}

// Runs on EVERY rail power-up. Anything that lost power came back at factory
// defaults, so every begin()/config call has to happen here rather than once
// in setup().
static void sensorsInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  rec.adsOk = ads.begin(0x48);
  if (rec.adsOk) ads.setGain(GAIN_TWOTHIRDS);

  rec.solarOk = solarSensor.begin();
  rec.battOk  = batterySensor.begin();

  rec.rtcOk = rtc.begin();
  if (rec.rtcOk && rtc.lostPower()) {
    // Power-gating the RTC only works if its coin cell holds the oscillator up
    // while Vcc is dead. This is the symptom of a missing or flat cell.
    Serial.println("!! RTC lost power - check the DS3231 coin cell. "
                   "Timestamps will be wrong until the RTC is set.");
  }

  dht.begin();   // re-arms the pin, which railOff() released to high-Z
}

// Is the relay allowed to cycle, or should the rail just stay on?
static bool gatingActive() {
  if (railMode == RAIL_FORCE_ON)  return false;
  if (railMode == RAIL_FORCE_OFF) return false;
  return sampleIntervalS >= MIN_GATING_INTERVAL_S;
}

// ============================================================================
// SENSOR READING
// ============================================================================

static void readSensors() {
  // --- Snapshot and reset the pulse accumulators -------------------------
  // Catch up on anything that arrived during warmup first, so the interval is
  // accounted for exactly once and nothing straddles the boundary.
  servicePulseCounters();

  uint32_t now         = millis();
  uint32_t elapsedMs   = now - intervalStartMs;
  uint32_t windPulses  = windPulsesInterval;
  uint32_t tips        = rainTipsInterval;
  float    gust        = gustMaxMph;

  windPulsesInterval = 0;
  rainTipsInterval   = 0;
  gustMaxMph         = 0.0f;
  intervalStartMs    = now;

  // The record's interval is measured read-to-read, not from the scheduler, so
  // the averages stay correct even when a cycle runs late or is triggered by
  // hand partway through an interval.
  rec.intervalMs = elapsedMs;

  float elapsedS = (elapsedMs > 0) ? (elapsedMs / 1000.0f) : 1.0f;
  rec.windAvgMph  = ((float)windPulses * WIND_MPH_PER_HZ) / elapsedS;
  rec.windAvgMs   = rec.windAvgMph * MPH_TO_MS;
  rec.windGustMph = GUST_TRACKING_ENABLED ? gust : rec.windAvgMph;
  rec.windGustMs  = rec.windGustMph * MPH_TO_MS;

  rec.rainTips       = tips;
  rec.rainIntervalMm = tips * MM_PER_TIP;
  rec.rainTotalMm    = rainTipsTotal * MM_PER_TIP;

  // --- DHT22 --------------------------------------------------------------
  // Can return NaN on a checksum error, and the first read after power-up is
  // the least reliable one. Hold the previous good value, as before, but count
  // the failures so a degrading sensor is visible rather than silent.
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  rec.dhtOk = (!isnan(t) && !isnan(h));
  if (!isnan(t)) lastGoodTemperature = t;
  if (!isnan(h)) lastGoodHumidity    = h;
  if (!rec.dhtOk) dhtFailures++;
  rec.temperature = lastGoodTemperature;
  rec.humidity    = lastGoodHumidity;

  // --- ADS1115 ------------------------------------------------------------
  for (int i = 0; i < 4; i++) {
    if (rec.adsOk) {
      int16_t raw = ads.readADC_SingleEnded(i);
      rec.adc[i] = ads.computeVolts(raw);
    } else {
      rec.adc[i] = NAN;
    }
  }

  // --- INA219 solar -------------------------------------------------------
  if (rec.solarOk) {
    rec.solarV  = solarSensor.getBusVoltage_V();
    rec.solarMa = solarSensor.getCurrent_mA();
    rec.solarW  = rec.solarV * rec.solarMa / 1000.0f;
  } else {
    rec.solarV = rec.solarMa = rec.solarW = NAN;
  }

  // --- INA219 battery -----------------------------------------------------
  if (rec.battOk) {
    rec.battV  = batterySensor.getBusVoltage_V();
    rec.battMa = batterySensor.getCurrent_mA();
    rec.battW  = rec.battV * rec.battMa / 1000.0f;
  } else {
    rec.battV = rec.battMa = rec.battW = NAN;
  }

  // --- RTC ----------------------------------------------------------------
  // Unlike the previous firmware, the begin() result is actually honoured and
  // the DateTime is validated, so a missing or unpowered RTC reports NO_RTC
  // instead of silently emitting a garbage timestamp.
  if (rec.rtcOk) {
    DateTime dt = rtc.now();
    if (dt.isValid()) {
      snprintf(rec.timestamp, sizeof(rec.timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
               dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
    } else {
      snprintf(rec.timestamp, sizeof(rec.timestamp), "RTC_INVALID");
      rec.rtcOk = false;
    }
  } else {
    snprintf(rec.timestamp, sizeof(rec.timestamp), "NO_RTC +%lus",
             (unsigned long)(millis() / 1000));
  }
}

// ============================================================================
// OUTPUT
// ============================================================================

static void publishRecord() {
  Serial.println("========================================");
  Serial.printf("  %s   [cycle %lu]\n", rec.timestamp, (unsigned long)cycleCount);
  Serial.println("----------------------------------------");
  Serial.printf("  Wind avg   %6.2f mph  (%5.2f m/s)\n", rec.windAvgMph, rec.windAvgMs);
  if (GUST_TRACKING_ENABLED)
    Serial.printf("  Wind gust  %6.2f mph  (%5.2f m/s)\n", rec.windGustMph, rec.windGustMs);
  Serial.printf("  Rain       %6.1f mm this interval (%lu tips) | %7.1f mm total\n",
                rec.rainIntervalMm, (unsigned long)rec.rainTips, rec.rainTotalMm);
  Serial.printf("  Temp       %6.2f C     %5.1f %%RH\n", rec.temperature, rec.humidity);
  Serial.printf("  Solar      %6.3f V  %8.2f mA  %7.3f W\n", rec.solarV, rec.solarMa, rec.solarW);
  Serial.printf("  Battery    %6.3f V  %8.2f mA  %7.3f W\n", rec.battV, rec.battMa, rec.battW);
  Serial.printf("  ADC     A0 %6.4f V  A1 %6.4f V  A2 %6.4f V  A3 %6.4f V\n",
                rec.adc[0], rec.adc[1], rec.adc[2], rec.adc[3]);
  Serial.println("----------------------------------------");
  if (gatingActive()) {
    Serial.printf("  interval %.1f s | rail on %.2f s | relay cycles %lu\n",
                  rec.intervalMs / 1000.0f, lastRailOnDurationMs / 1000.0f,
                  (unsigned long)relayCycles);
  } else {
    Serial.printf("  interval %.1f s | rail permanently on | relay cycles %lu\n",
                  rec.intervalMs / 1000.0f, (unsigned long)relayCycles);
  }
  Serial.printf("  health: ADS %s  SOLAR %s  BATT %s  RTC %s  DHT %s",
                rec.adsOk ? "ok" : "FAIL", rec.solarOk ? "ok" : "FAIL",
                rec.battOk ? "ok" : "FAIL", rec.rtcOk ? "ok" : "FAIL",
                rec.dhtOk ? "ok" : "FAIL");
  if (dhtFailures) Serial.printf("  (dht fails: %lu)", (unsigned long)dhtFailures);
  Serial.println();
  Serial.println("========================================\n");
}

// ============================================================================
// SAMPLE CYCLE STATE MACHINE
// ============================================================================
//
// CYCLE_IDLE   -> interval elapsed (or 's' typed): energise rail, go to WARMUP
// CYCLE_WARMUP -> warmup elapsed: re-init sensors, read, publish, drop rail
//
// Nothing here blocks, so pulse counting and the console keep being serviced
// throughout - including across the whole 2.2 s warmup.

static void startCycle() {
  // railOn() only stamps railOnAtMs on an actual off->on transition, so if the
  // rail has been up a while (FORCE_ON, or a sub-30 s interval) the WARMUP
  // state below falls straight through. No special case needed - and unlike a
  // hardcoded shortcut, a rail forced on one second ago still gets its full
  // warmup before anything is read.
  railOn();
  cycleState = CYCLE_WARMUP;
}

static void finishCycle() {
  sensorsInit();          // re-init: the rail may have just come back up
  readSensors();
  cycleCount++;
  if (gatingActive()) railOff();
  publishRecord();
  cycleState = CYCLE_IDLE;
}

static void serviceCycle() {
  uint32_t now = millis();

  switch (cycleState) {
    case CYCLE_IDLE: {
      if (railMode == RAIL_FORCE_OFF) {
        // Sensors are unpowered by operator request. Wind and rain keep being
        // counted regardless - they are on permanent power. Hold the schedule
        // at `now` so returning to AUTO does not fire a burst of catch-up
        // cycles for the time spent powered down.
        sampleRequested  = false;
        lastCycleStartMs = now;
        return;
      }

      bool due = sampleRequested ||
                 ((uint32_t)(now - lastCycleStartMs) >= sampleIntervalS * 1000UL);
      if (!due) return;

      sampleRequested = false;
      // Advance by whole intervals rather than resetting to `now`, so the
      // cadence does not drift by the sample duration on every cycle. If we
      // have fallen more than one interval behind, resync instead of trying to
      // catch up in a burst.
      lastCycleStartMs += sampleIntervalS * 1000UL;
      if ((uint32_t)(now - lastCycleStartMs) > sampleIntervalS * 1000UL)
        lastCycleStartMs = now;

      startCycle();
      return;
    }

    case CYCLE_WARMUP: {
      if ((uint32_t)(now - railOnAtMs) < SENSOR_WARMUP_MS) return;
      finishCycle();
      return;
    }
  }
}

// ============================================================================
// SERIAL CONSOLE
// ============================================================================

static void printHelp() {
  Serial.println("\n--- commands ---");
  Serial.println("  h       this help");
  Serial.println("  t       status");
  Serial.println("  s       sample now");
  Serial.println("  r       reset rainfall total");
  Serial.println("  a       rail AUTO (scheduled gating)");
  Serial.println("  1       rail forced ON");
  Serial.println("  0       rail forced OFF (sampling paused, wind/rain still counted)");
  Serial.println("  i<sec>  set sample interval, e.g. i300\n");
}

static void printStatus() {
  const char *mode = (railMode == RAIL_AUTO)     ? "AUTO"
                   : (railMode == RAIL_FORCE_ON) ? "FORCED ON"
                                                 : "FORCED OFF";
  Serial.println("\n--- status ---");
  Serial.printf("  rail mode        %s (%s now, gating %s)\n",
                mode, railPowered ? "powered" : "off",
                gatingActive() ? "on" : "off");
  Serial.printf("  sample interval  %lu s\n", (unsigned long)sampleIntervalS);
  Serial.printf("  cycles done      %lu\n", (unsigned long)cycleCount);
  Serial.printf("  relay operations %lu\n", (unsigned long)relayCycles);
  Serial.printf("  rain total       %.1f mm (%lu tips)\n",
                rainTipsTotal * MM_PER_TIP, (unsigned long)rainTipsTotal);
  Serial.printf("  pending wind     %lu pulses over %.1f s\n",
                (unsigned long)windPulsesInterval,
                (millis() - intervalStartMs) / 1000.0f);
  Serial.printf("  dht failures     %lu\n", (unsigned long)dhtFailures);
  Serial.printf("  cpu             %lu MHz\n", (unsigned long)getCpuFrequencyMhz());
  Serial.println();
}

static void handleCommand(const char *cmd) {
  if (cmd[0] == '\0') return;

  switch (cmd[0]) {
    case 'h': case 'H': case '?':
      printHelp();
      break;

    case 't': case 'T':
      printStatus();
      break;

    case 's': case 'S':
      if (railMode == RAIL_FORCE_OFF) {
        Serial.println("!! rail is forced OFF - type 'a' (auto) or '1' (on) first");
        break;
      }
      Serial.println(">> sampling now");
      sampleRequested = true;
      break;

    case 'r': case 'R':
      rainTipsTotal = 0;
      Serial.println(">> rainfall total reset");
      break;

    case 'a': case 'A':
      railMode = RAIL_AUTO;
      Serial.println(">> rail AUTO");
      break;

    case '1':
      railMode = RAIL_FORCE_ON;
      railOn();
      // Deliberately no sensorsInit() here - the rail has only just come up and
      // begin() on a cold bus would report spurious failures. The next cycle
      // initialises the sensors after the warmup has actually elapsed.
      Serial.println(">> rail forced ON (sensors stay powered)");
      break;

    case '0':
      railMode = RAIL_FORCE_OFF;
      railOff();
      Serial.println(">> rail forced OFF - sampling paused. "
                     "Wind and rain are still being counted.");
      break;

    case 'i': case 'I': {
      uint32_t v = strtoul(cmd + 1, nullptr, 10);
      if (v < MIN_SAMPLE_INTERVAL_S || v > MAX_SAMPLE_INTERVAL_S) {
        Serial.printf("!! interval must be %lu..%lu s\n",
                      (unsigned long)MIN_SAMPLE_INTERVAL_S,
                      (unsigned long)MAX_SAMPLE_INTERVAL_S);
        break;
      }
      sampleIntervalS  = v;
      lastCycleStartMs = millis();
      Serial.printf(">> sample interval = %lu s (relay gating %s)\n",
                    (unsigned long)v, gatingActive() ? "ON" : "OFF - too short, "
                    "rail will stay powered to save relay life");
      break;
    }

    default:
      Serial.printf("!! unknown command '%s' - type h for help\n", cmd);
      break;
  }
}

// Line-based, but with a short inactivity flush so terminals that do not send
// a newline still work.
static void serviceSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    lastCmdCharMs = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      handleCommand(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }

  if (cmdLen > 0 && (millis() - lastCmdCharMs) > 100) {
    cmdBuf[cmdLen] = '\0';
    handleCommand(cmdBuf);
    cmdLen = 0;
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
  // Set the clock before Serial.begin() so the UART divider is derived from
  // the final APB configuration and never has to be recalculated.
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  Serial.begin(115200);
  delay(1000);   // the only delay() in the firmware: USB-CDC enumeration only

  Serial.println("\n\n=== ESP32 AWS - time-based sampling, relay-gated sensors ===");
  Serial.printf("CPU %lu MHz | sample interval %lu s | warmup %lu ms\n",
                (unsigned long)getCpuFrequencyMhz(),
                (unsigned long)sampleIntervalS,
                (unsigned long)SENSOR_WARMUP_MS);

  // ---- Relay: start with the rail OFF -------------------------------------
  // The previous firmware energised the relay in setup() and left it on. Each
  // cycle now brings it up only for the read.
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, !RELAY_ACTIVE);
  railPowered = false;

  // ---- Wind + rain: hardware pulse counters, permanently powered ----------
  pulseCountersInit();
  Serial.printf("PCNT ready: wind GPIO%d (unit 0), rain GPIO%d (unit 1), "
                "filter %u APB clk\n", WIND_PIN, RAIN_PIN, PCNT_FILTER_MAX);
  Serial.println("Wind/rain are on permanent power and counted 24/7 in hardware.");

  uint32_t now = millis();
  intervalStartMs   = now;
  lastGustSampleMs  = now;
  lastRainPollMs    = now;
  lastAcceptedTipMs = now - RAIN_DEBOUNCE_MS;
  lastCycleStartMs  = now;

  printHelp();

  // One immediate cycle at boot so the station reports straight away rather
  // than staying silent for a full interval.
  Serial.println("--- initial cycle ---\n");
  sampleRequested = true;
}

void loop() {
  serviceSerial();        // console, non-blocking
  servicePulseCounters(); // wind accumulate + gust peak + rain debounce
  serviceCycle();         // relay / warmup / read state machine
}
