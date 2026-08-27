#include "config.h"
#include "portal_html.h"
#include "calibration.h"

// ---------- Object Instances ----------
Adafruit_ADS1115      ads;
Adafruit_INA219      batterySensor(0x40);
Adafruit_INA219      solarSensor(0x41);
RTC_DS3231           rtc;
DHT                  dht(DHTPIN, DHTTYPE);
Preferences          preferences;

// ---------- DS18B20 Objects ----------
OneWire              oneWire(DS18B20_PIN);
DallasTemperature    ds18b20(&oneWire);
bool                 ds18b20Ok = false;

// ---------- BME280 (pressure) ----------
// Not fitted yet. Probed on every rail power-up, so the sensor starts working
// the moment it is plugged in - no reflash required.
Adafruit_BME280      bme;
bool                 bmeOk = false;

// ---------- Global Variables ----------
int sleepIntervalSec = 300;
String stationCode = "AWS-UG-001";
bool relayOn = false;

// ---------- Clock State ----------
bool rtcOk = false;
bool rtcTimeValid = false;
static bool ntpEverSynced = false;          // NTP succeeded at least once this boot
static unsigned long lastNtpSyncMs = 0;     // millis() of last success
static unsigned long lastNtpAttemptMs = 0;  // millis() of last attempt, success or not
DataRecord lastRecord;
bool haveLastRecord = false;
uint32_t nextCycleEpoch = 0;
int queueDepthCached = 0;
unsigned long nextCycleMillis = 0;

// ================== WIND & RAIN PULSE COUNTING ==================
// Pulses have to be counted whether the CPU is awake or in light sleep, and
// light sleep only supports LEVEL-triggered GPIO wake - not edges. So there are
// two ways into the counters:
//
//   awake  - the edge ISRs below
//   asleep - esp_light_sleep_start() returns on a LOW level and the wake
//            handler counts the pulse directly
//
// Both funnel through countWindPulse()/countRainPulse(), which share one
// debounce timestamp per channel. Without that shared timestamp a single reed
// closure is counted twice: once by the level wake, once by the edge ISR as the
// CPU resumes.

volatile uint32_t windPulseCount = 0;
volatile uint32_t rainPulseCount = 0;
static volatile uint32_t lastWindPulseUs = 0;
static volatile uint32_t lastRainPulseUs = 0;

// GPIO wakes where neither line was still low by the time we read it - i.e.
// pulses too short to attribute. Reported each interval so it stays visible.
static volatile uint32_t ambiguousWakes = 0;

// Gust = the highest 3 s average within the interval. Tracked as a running
// "best" bucket. Deliberately integer-only: doing float maths inside an IRAM
// ISR means relying on FPU state being saved, which it is not by default.
static volatile uint32_t gustBucketStartUs = 0;
static volatile uint32_t gustBucketCount   = 0;
static volatile uint32_t gustBestCount     = 0;
static volatile uint32_t gustBestSpanUs    = 1;   // 1, not 0, so the first compare works

// When the current interval started. Seeded in setup() at the moment the
// interrupts are armed, so the first burst divides by the time counting has
// actually been running rather than by an assumed full interval.
static uint32_t      lastSnapshotEpoch  = 0;
static unsigned long lastSnapshotMillis = 0;

// Interval results, refreshed by snapshotPulseCounters().
float windSpeed_MPH = 0.0;
float windSpeed_MS  = 0.0;
float windGust_MS   = 0.0;
float rain_mm       = 0.0;   // mm accumulated in the current interval, zeroed each snapshot
float lastIntervalSec = 0.0;   // what windSpeed_MS and rain_mm actually average over

void IRAM_ATTR countWindPulse() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastWindPulseUs) < WIND_DEBOUNCE_US) return;
  lastWindPulseUs = now;
  windPulseCount++;

  // Close the gust bucket once it has run its full width and keep it if it beat
  // the best so far. Comparing count/span as a cross-multiply avoids a divide.
  uint32_t span = now - gustBucketStartUs;
  if (span >= GUST_WINDOW_US) {
    if ((uint64_t)gustBucketCount * (uint64_t)gustBestSpanUs >
        (uint64_t)gustBestCount * (uint64_t)span) {
      gustBestCount  = gustBucketCount;
      gustBestSpanUs = span;
    }
    gustBucketStartUs = now;
    gustBucketCount   = 0;
  }
  gustBucketCount++;
}

void IRAM_ATTR countRainPulse() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastRainPulseUs) < DEBOUNCE_US) return;
  lastRainPulseUs = now;
  rainPulseCount++;
}

void IRAM_ATTR windISR() { countWindPulse(); }
void IRAM_ATTR rainISR() { countRainPulse(); }

// ---------- Clock reference ----------
// The DS3231 sits on the gated sensor rail, so it is only readable during a
// burst. Between bursts the last good reading is carried forward on millis(),
// which keeps advancing across light sleep.
static uint32_t      clockRefEpoch  = 0;
static unsigned long clockRefMillis = 0;

// The DS3231 holds EAT, so this is not a true UTC epoch - but every use here is
// a difference between two of them, which is unaffected. Returns 0 only when
// the time is not known at all.
uint32_t currentEpoch() {
  if (rtcOk && relayOn) {
    DateTime now = rtc.now();
    if (now.isValid() && now.year() >= RTC_MIN_VALID_YEAR) {
      clockRefEpoch  = now.unixtime();
      clockRefMillis = millis();
      rtcTimeValid   = true;
      return clockRefEpoch;
    }
    // The rail is up and it still did not answer sensibly, so the clock really
    // is unset. Only latch invalid here: a failed read with the rail down means
    // nothing at all, and latching on it strands every later record at 1970.
    rtcTimeValid = false;
  }

  if (clockRefEpoch > 0) {
    return clockRefEpoch + (uint32_t)((millis() - clockRefMillis) / 1000UL);
  }
  return 0;
}

// ---------- Close out the interval ----------
// Called once at the top of each duty cycle, so every pulse is attributed to
// exactly one interval and nothing is double counted.
void snapshotPulseCounters() {
  noInterrupts();
  uint32_t windPulses = windPulseCount;  windPulseCount = 0;
  uint32_t rainTips   = rainPulseCount;  rainPulseCount = 0;
  uint32_t bestCount  = gustBestCount;
  uint32_t bestSpan   = gustBestSpanUs;
  gustBestCount     = 0;
  gustBestSpanUs    = 1;
  gustBucketCount   = 0;
  gustBucketStartUs = micros();
  interrupts();

  // Elapsed time comes from the RTC, not micros(), which wraps every ~71 min -
  // uncomfortably close to the 60 min maximum interval. millis() is the
  // fallback when the clock is unknown; it advances across light sleep too.
  uint32_t nowEpoch = currentEpoch();
  float elapsed;
  if (nowEpoch > 0 && lastSnapshotEpoch > 0 && nowEpoch > lastSnapshotEpoch) {
    elapsed = (float)(nowEpoch - lastSnapshotEpoch);
  } else if (lastSnapshotMillis > 0) {
    elapsed = (float)(millis() - lastSnapshotMillis) / 1000.0f;
  } else {
    elapsed = (float)sleepIntervalSec;
  }
  lastSnapshotEpoch  = nowEpoch;      // 0 is fine - the millis path covers it
  lastSnapshotMillis = millis();
  if (elapsed < 1.0f) elapsed = 1.0f;

  float hz = (float)windPulses / elapsed;
  windSpeed_MPH = hz * WIND_MPH_PER_HZ;
  windSpeed_MS  = hz * WIND_MS_PER_HZ;

  float gustHz = (bestSpan > 0)
                 ? ((float)bestCount * 1000000.0f / (float)bestSpan)
                 : 0.0f;
  windGust_MS = gustHz * WIND_MS_PER_HZ;
  // With few pulses no bucket ever closes, which would report a gust below the
  // interval mean. The gust can never be less than the average.
  if (windGust_MS < windSpeed_MS) windGust_MS = windSpeed_MS;

  rain_mm = (float)rainTips * MM_PER_TIP;
  lastIntervalSec = elapsed;

  uint32_t missed = ambiguousWakes;
  ambiguousWakes = 0;
  if (missed > 0) {
    Serial.printf("⚠️ %lu wake(s) not attributable to a pin - pulses may be undercounted.\n",
                  (unsigned long)missed);
  }

  Serial.printf("🌬️ %lu pulses / %.0f s -> %.2f m/s avg, %.2f m/s gust   🌧️ %lu tips -> %.1f mm\n",
                (unsigned long)windPulses, elapsed, windSpeed_MS, windGust_MS,
                (unsigned long)rainTips, rain_mm);
}

// ================== LIGHT SLEEP ==================

// Light-sleep wake is level-triggered, so re-arming while the contact is still
// closed wakes instantly and spins at full power. Bounded so a stuck-closed
// reed shows up as a fault instead of a silent flat battery.
static bool waitForReedRelease(uint8_t pin) {
  uint32_t start = millis();
  while (digitalRead(pin) == LOW) {
    if ((millis() - start) > REED_RELEASE_MS) return false;
    delay(1);
  }
  return true;
}

static void reportStuckReed(const char *which) {
  // Rate limited, and Serial-only: the SD rail is powered down during sleep.
  static uint32_t lastReportMs = 0;
  if ((millis() - lastReportMs) < 60000UL) return;
  lastReportMs = millis();
  Serial.printf("⚠️ %s reed still closed after %d ms - check for a stuck contact.\n",
                which, REED_RELEASE_MS);
}

void sleepUntilNextEventOrDeadline() {
  // How long we are allowed to stay down. Deliberately millis()-based rather
  // than reading the RTC: if the DS3231 shares the gated rail it is
  // unreachable right now, and esp_timer accounts for light sleep so millis()
  // keeps advancing. The deadline is re-derived from the RTC at the top of
  // every burst, so drift cannot accumulate across cycles.
  long msLeft = (long)(nextCycleMillis - millis());
  if (msLeft <= 0) return;                          // burst is due

  long remaining = msLeft / 1000;
  if (remaining < 1)                remaining = 1;
  if (remaining > SLEEP_CHUNK_SEC)  remaining = SLEEP_CHUNK_SEC;

  // The radio must be down: light sleep with WiFi up either draws far more or
  // is rejected outright.
  if (WiFi.getMode() != WIFI_OFF) WiFi.mode(WIFI_OFF);

  // Hand the pins from the Arduino edge ISRs to the level-triggered wake, then
  // hand them back afterwards. The two share the GPIO interrupt-type register,
  // and the driver source is not shipped with the core, so rather than assume
  // how gpio_wakeup_enable() treats an already-attached edge interrupt, this
  // makes the ownership explicit in both directions.
  detachInterrupt(digitalPinToInterrupt(WIND_PIN));
  detachInterrupt(digitalPinToInterrupt(RAIN_PIN));

  gpio_wakeup_enable((gpio_num_t)WIND_PIN, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)RAIN_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_timer_wakeup((uint64_t)remaining * 1000000ULL);

  esp_err_t err = esp_light_sleep_start();

  if (err == ESP_OK && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    // Classic ESP32 has no esp_sleep_get_gpio_wakeup_status() - that API sits
    // behind SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP, which only the S2/S3/C3 set.
    // The wake was level-triggered, so whichever line is still low is the one
    // that woke us. A closure shorter than the wake latency can slip through;
    // ambiguousWakes counts those so the loss is measurable rather than silent.
    // Sample both lines first: the contact that woke us may reopen within
    // milliseconds, and anything read later is already too late.
    bool windLow = (digitalRead(WIND_PIN) == LOW);
    bool rainLow = (digitalRead(RAIN_PIN) == LOW);

    // Give the pins back to the edge ISRs BEFORE the release waits below.
    // Those waits can run for REED_RELEASE_MS each, and with the ISRs still
    // detached a tip on the other channel during that window would vanish.
    // Re-arming here cannot double count: the line is already low, so there is
    // no falling edge to fire on, and the shared debounce covers the rest.
    gpio_wakeup_disable((gpio_num_t)WIND_PIN);
    gpio_wakeup_disable((gpio_num_t)RAIN_PIN);
    attachInterrupt(digitalPinToInterrupt(WIND_PIN), windISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, FALLING);

    if (windLow) {
      countWindPulse();
      if (!waitForReedRelease(WIND_PIN)) reportStuckReed("Wind");
    }
    if (rainLow) {
      countRainPulse();
      if (!waitForReedRelease(RAIN_PIN)) reportStuckReed("Rain");
    }
    if (!windLow && !rainLow) ambiguousWakes++;
  } else {
    // Timer wake, or sleep was rejected - nothing to attribute, just restore.
    gpio_wakeup_disable((gpio_num_t)WIND_PIN);
    gpio_wakeup_disable((gpio_num_t)RAIN_PIN);
    attachInterrupt(digitalPinToInterrupt(WIND_PIN), windISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, FALLING);
  }
}

// ---------- Config Load / Save ----------
void loadConfig() {
  preferences.begin("weather", false);
  // getInt() reports a missing key at log_v, but getString() uses log_e even
  // when a default is supplied - so probe with isKey() to keep a fresh NVS
  // from printing errors that are not errors.
  sleepIntervalSec = preferences.getInt("sleepInt", 300);
  stationCode = preferences.isKey("station")
                ? preferences.getString("station")
                : String("AWS-UG-001");
  preferences.end();
  Serial.printf("⏱️ Loaded: Sleep %d s, Station %s\n", sleepIntervalSec, stationCode.c_str());
}

void saveConfig(int newInterval, const String &newStation) {
  preferences.begin("weather", false);
  if (newInterval > 0) preferences.putInt("sleepInt", newInterval);
  if (newStation.length() > 0) preferences.putString("station", newStation);
  preferences.end();
  if (newInterval > 0) sleepIntervalSec = newInterval;
  if (newStation.length() > 0) stationCode = newStation;
  Serial.printf("💾 Saved: Interval %d s, Station %s\n", sleepIntervalSec, stationCode.c_str());
}

// ---------- Error Logging ----------
// The SD card shares the sensor rail, so an error raised during the config
// portal or a sleep window has nowhere to be written. Hold those in RAM and
// flush them at the next burst rather than losing them.
static String   errorBuffer[ERROR_BUFFER_SLOTS];
static uint8_t  errorBufferCount   = 0;
static uint32_t errorBufferDropped = 0;

static void bufferError(const String &line) {
  if (errorBufferCount < ERROR_BUFFER_SLOTS) errorBuffer[errorBufferCount++] = line;
  else                                       errorBufferDropped++;
}

void flushBufferedErrors() {
  if (errorBufferCount == 0 && errorBufferDropped == 0) return;

  uint8_t flushed = errorBufferCount;
  for (uint8_t i = 0; i < errorBufferCount; i++) {
    appendSD(errorBuffer[i], SD_ERROR_FILE);
    errorBuffer[i] = "";
  }
  if (errorBufferDropped > 0) {
    appendSD(getTimestamp() + " | " + String(errorBufferDropped) +
             " further error(s) dropped - RAM buffer full", SD_ERROR_FILE);
  }
  errorBufferCount   = 0;
  errorBufferDropped = 0;
  Serial.printf("📝 Flushed %u buffered error(s) to the log.\n", flushed);
}

void logError(const String &message) {
  String line = getTimestamp() + " | " + message;
  Serial.println("⚠️ " + line);
  if (!appendSD(line, SD_ERROR_FILE)) bufferError(line);
}

// ---------- Generic SD Append ----------
bool appendSD(const String &line, const char *filename) {
  File f = SD.open(filename, FILE_APPEND);
  if (!f) return false;
  f.println(line);
  f.close();
  return true;
}

// ---------- WiFi Connection (Preferences first, fallback hardcoded) ----------
bool connectWiFi() {
  // Check if already connected
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("📶 WiFi already connected.");
    return true;
  }

  // The radio is powered down between bursts, so the mode has to be put back
  // before begin() will do anything at all.
  if (WiFi.getMode() == WIFI_OFF) WiFi.mode(WIFI_STA);

  // Try to get saved WiFi from Preferences
  preferences.begin("wifi", false);
  String savedSSID     = preferences.isKey("ssid") ? preferences.getString("ssid") : String();
  String savedPassword = preferences.isKey("pass") ? preferences.getString("pass") : String();
  preferences.end();

  // If Preferences has saved credentials, use them
  if (savedSSID.length() > 0 && savedPassword.length() > 0) {
    Serial.printf("📶 Using saved WiFi: %s\n", savedSSID.c_str());
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
  } else {
    // Fallback to hard-coded credentials (only if no saved credentials)
    Serial.printf("📶 No saved credentials - using fallback: %s\n", WIFI_SSID_FALLBACK);
    WiFi.begin(WIFI_SSID_FALLBACK, WIFI_PASSWORD_FALLBACK);
  }

  // Wait for connection
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("📶 IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\n❌ WiFi connection failed.");
    return false;
  }
}

// ================== CONFIGURATION PORTAL ==================
// A purpose-built portal: the update interval is the front-page control and
// WiFiManager is one button among several rather than the entire UI.
//
// Served from our own WebServer so the page can be designed freely. WiFiManager
// is still the right tool for provisioning, but it wants port 80 too, so the
// two can never run at once - /api/wifi tears this server down, runs
// WiFiManager, then brings this one back for what remains of the window.

static WebServer portalServer(80);
static DNSServer portalDns;

static bool portalShouldClose    = false;   // "Done" pressed
static bool portalWantsWiFiSetup = false;   // hand off to WiFiManager
static bool portalRestartPending = false;
static bool portalUploadPending  = false;
static int  portalQueueDepth     = 0;       // cached: the SD rail may be off

// ---------- Helpers shared with the rest of the firmware ----------
int queuedRecordCount() {
  if (!SD.exists(SD_QUEUE_FILE)) return 0;
  File f = SD.open(SD_QUEUE_FILE, FILE_READ);
  if (!f) return 0;
  int n = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) n++;
  }
  f.close();
  return n;
}

void setSensorPower(bool on) {
  digitalWrite(RELAY_PIN, on ? RELAY_ACTIVE : !RELAY_ACTIVE);
  relayOn = on;
}

// The sensor rail is power-cycled every duty cycle, so everything on it comes
// back in its reset state. Device-side configuration therefore has to be
// re-established on every power-up, not just once in setup():
//
//   SD      the card needs its full init sequence again. Skip it and the
//           driver still believes it is mounted, so every call fails with
//           "Card Failed! cmd: 0x0d" (CMD13 SEND_STATUS) and writes are lost.
//   INA219  begin() writes the calibration register, and that register lives
//           on the chip. Skip it and the current readings come back silently
//           wrong rather than erroring.
//   ADS1115 gain is held in the library object, but re-begin() is cheap and
//           confirms the device actually answered.
static bool sdMounted = false;

bool mountSD() {
  if (sdMounted) return true;
  if (!SD.begin(SD_CS)) {
    sdMounted = false;
    return false;
  }
  sdMounted = true;
  return true;
}

void unmountSD() {
  if (!sdMounted) return;
  SD.end();          // flush and release the card before it loses power
  sdMounted = false;
}

// Try both addresses the breakout boards use. Forced mode means the chip takes
// one measurement on demand and returns to sleep, rather than free-running.
static bool detectBME280() {
  if (!bme.begin(BME280_ADDR_A) && !bme.begin(BME280_ADDR_B)) return false;
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,    // temperature (compensation only)
                  Adafruit_BME280::SAMPLING_X1,    // pressure
                  Adafruit_BME280::SAMPLING_NONE,  // humidity - DHT22 owns this
                  Adafruit_BME280::FILTER_OFF);
  return true;
}

void sensorRailUp() {
  setSensorPower(true);
  delay(SENSOR_RAIL_SETTLE_MS);

  // The DS3231 is on this rail too, so re-probe it before anything asks for a
  // timestamp - otherwise rtcOk/rtcTimeValid stay stuck at whatever the last
  // rail-down read concluded.
  if (rtc.begin()) {
    rtcOk = true;
    refreshRtcValidity();
  } else {
    rtcOk = false;
    logError("RTC silent after power-up.");
  }

  if (!mountSD()) {
    Serial.println("❌ SD did not remount after power-up.");
  }
  flushBufferedErrors();     // there is somewhere to write again

  if (!batterySensor.begin()) logError("INA219 Battery silent after power-up.");
  if (!solarSensor.begin())   logError("INA219 Solar silent after power-up.");

  if (ads.begin(0x48)) ads.setGain(GAIN_TWOTHIRDS);
  else                 logError("ADS1115 silent after power-up.");

  dht.begin();

  ds18b20.begin();
  ds18b20Ok = (ds18b20.getDeviceCount() > 0);

  bool bmeWasOk = bmeOk;
  bmeOk = detectBME280();
  if (bmeOk && !bmeWasOk) Serial.println("✅ BME280 detected - pressure now being recorded.");
}

void sensorRailDown() {
  unmountSD();
  setSensorPower(false);
}

static const char *clockSourceName() {
  if (rtcOk && rtcTimeValid) return ntpEverSynced ? "RTC, NTP synced" : "RTC";
  if (ntpEverSynced)         return "NTP only, no RTC";
  return "not set";
}

static void sendJson(bool ok, const String &message) {
  JsonDocument doc;
  doc["ok"] = ok;
  doc["message"] = message;
  String out;
  serializeJson(doc, out);
  portalServer.send(200, "application/json", out);
}

// ---------- Route handlers ----------
static void handleRoot() {
  portalServer.send_P(200, "text/html", PORTAL_INDEX_HTML);
}

static void handleStatus() {
  JsonDocument doc;
  doc["station"]     = stationCode;
  doc["intervalMin"] = sleepIntervalSec / 60;
  doc["time"]        = getTimestamp();
  doc["clockValid"]  = (rtcTimeValid || ntpEverSynced);
  doc["clockSource"] = clockSourceName();
  doc["uptimeSec"]   = (uint32_t)(millis() / 1000);
  doc["queueDepth"]  = portalQueueDepth;

  // Negative means "unknown" - the page renders that as "now".
  long next = -1;
  if (rtcOk && rtcTimeValid && nextCycleEpoch > 0) {
    next = (long)nextCycleEpoch - (long)rtc.now().unixtime();
  }
  doc["nextCycleSec"] = next;

  JsonObject w = doc["wifi"].to<JsonObject>();
  bool up = (WiFi.status() == WL_CONNECTED);
  w["connected"] = up;
  w["ssid"]      = up ? WiFi.SSID() : String("");
  w["ip"]        = up ? WiFi.localIP().toString() : String("");
  w["rssi"]      = up ? WiFi.RSSI() : 0;

  doc["haveReading"] = haveLastRecord;
  if (haveLastRecord) {
    doc["temp"]     = lastRecord.temperature;
    doc["humidity"] = lastRecord.humidity;
    doc["windMS"]   = lastRecord.windMS;
    doc["gustMS"]   = lastRecord.windGustMS;
    doc["rain"]     = lastRecord.rainInterval_mm;
    doc["battV"]    = lastRecord.batteryVoltage;
    doc["battI"]    = lastRecord.batteryCurrent_mA;
    doc["solarV"]   = lastRecord.solarVoltage;
    doc["solarI"]   = lastRecord.solarCurrent_mA;
  }

  String out;
  serializeJson(doc, out);
  portalServer.send(200, "application/json", out);
}

static void handleSettings() {
  int    newInterval = 0;    // 0 = leave unchanged
  String newStation  = "";   // "" = leave unchanged
  String msg         = "";
  bool   ok          = true;

  // ---- Update interval (minutes in the UI, seconds in NVS) ----
  if (portalServer.hasArg("interval")) {
    String v = portalServer.arg("interval");
    v.trim();
    if (v.length() > 0) {
      int mins = v.toInt();
      if (mins <= 0) {
        // toInt() yields 0 for junk, so this catches typos that would
        // otherwise look exactly like a successful save.
        ok = false;
        msg += "Interval must be a whole number of minutes. ";
      } else {
        int secs = mins * 60;
        if (secs < SLEEP_MIN_SEC) secs = SLEEP_MIN_SEC;
        if (secs > SLEEP_MAX_SEC) secs = SLEEP_MAX_SEC;
        if (secs != sleepIntervalSec) {
          newInterval = secs;
          msg += "Interval set to " + String(secs / 60) + " min. ";
        } else {
          msg += "Interval unchanged. ";
        }
      }
    }
  }

  // ---- Station code ----
  if (portalServer.hasArg("station")) {
    String v = portalServer.arg("station");
    v.trim();
    if (v.length() > 0 && v != stationCode) {
      newStation = v;
      msg += "Station code set to " + v + ". ";
    }
  }

  if (newInterval > 0 || newStation.length() > 0) {
    saveConfig(newInterval, newStation);
  }

  // ---- Manual clock set (blank keeps the current time) ----
  if (portalServer.hasArg("clock")) {
    String v = portalServer.arg("clock");
    v.trim();
    if (v.length() > 0) {
      if (setRtcFromString(v)) {
        msg += "Clock set to " + getTimestamp() + ". ";
      } else {
        ok = false;
        msg += "Clock rejected - use YYYY-MM-DD HH:MM:SS. ";
        logError("Portal clock value rejected: " + v);
      }
    }
  }

  if (msg.length() == 0) msg = "Nothing to change.";
  sendJson(ok, msg);
}

static void handleWiFiSetup() {
  portalWantsWiFiSetup = true;
  sendJson(true, "Starting WiFi setup - rejoin " PORTAL_AP_SSID);
}

static void handleUpload() {
  if (WiFi.status() != WL_CONNECTED) {
    sendJson(false, "Not connected to WiFi - configure a network first");
    return;
  }
  portalUploadPending = true;
  sendJson(true, "Upload started - watch the queued count");
}

static void handleRestart() {
  portalRestartPending = true;
  sendJson(true, "Restarting...");
}

static void handleDone() {
  portalShouldClose = true;
  sendJson(true, "Portal closing - station going back to sleep");
}

static void handleNotFound() {
  // Captive-portal probes land here; bounce them to the real page.
  portalServer.sendHeader("Location",
                          String("http://") + WiFi.softAPIP().toString() + "/", true);
  portalServer.send(302, "text/plain", "");
}

// ---------- Server lifecycle ----------
static void startPortalServer() {
  portalServer.on("/",             HTTP_GET,  handleRoot);
  portalServer.on("/api/status",   HTTP_GET,  handleStatus);
  portalServer.on("/api/settings", HTTP_POST, handleSettings);
  portalServer.on("/api/wifi",     HTTP_POST, handleWiFiSetup);
  portalServer.on("/api/upload",   HTTP_POST, handleUpload);
  portalServer.on("/api/restart",  HTTP_POST, handleRestart);
  portalServer.on("/api/done",     HTTP_POST, handleDone);
  portalServer.onNotFound(handleNotFound);
  portalServer.begin();
}

static void bringUpAccessPoint() {
  WiFi.mode(WIFI_AP_STA);          // keep STA so uploads still work while open
  WiFi.softAP(PORTAL_AP_SSID);
  delay(150);
  portalDns.start(53, "*", WiFi.softAPIP());
}

// Hand control to WiFiManager for provisioning, then take it back.
static void runWiFiManagerHandoff() {
  Serial.println("🔧 Handing off to WiFiManager for WiFi setup...");
  portalServer.stop();
  portalDns.stop();
  delay(200);

  WiFiManager wm;
  wm.setConfigPortalTimeout(PORTAL_TIMEOUT);
  wm.setAPCallback([](WiFiManager *) {
    Serial.println("📶 WiFiManager AP up: " PORTAL_AP_SSID);
  });

  // Same SSID as our own portal, so the phone reconnects to a familiar name.
  bool got = wm.startConfigPortal(PORTAL_AP_SSID, NULL);
  Serial.println(got ? "✅ New WiFi credentials saved."
                     : "ℹ️ WiFi setup closed without changes.");

  bringUpAccessPoint();
  startPortalServer();
}

static void doPortalUpload() {
  Serial.println("📤 Forced upload from portal...");
  sensorRailUp();                // the SD card shares the gated rail
  processPendingQueue();
  portalQueueDepth = queueDepthCached;
  sensorRailDown();
  Serial.printf("📤 Forced upload done, %d record(s) still queued.\n", portalQueueDepth);
}

// ---------- The portal itself ----------
void runConfigPortal() {
  Serial.printf("🌐 Config portal open for %d s - join \"%s\", then browse to http://192.168.4.1\n",
                PORTAL_TIMEOUT, PORTAL_AP_SSID);

  portalShouldClose    = false;
  portalWantsWiFiSetup = false;
  portalRestartPending = false;
  portalUploadPending  = false;

  // Use the cached depth: powering the rail back up here would mean another
  // full SD init cycle purely to count lines we already counted.
  portalQueueDepth = queueDepthCached;

  wifi_mode_t previousMode = WiFi.getMode();
  bringUpAccessPoint();
  startPortalServer();

  unsigned long start    = millis();
  unsigned long windowMs = (unsigned long)PORTAL_TIMEOUT * 1000UL;

  while ((millis() - start) < windowMs) {
    portalDns.processNextRequest();
    portalServer.handleClient();

    if (portalWantsWiFiSetup) {
      portalWantsWiFiSetup = false;
      runWiFiManagerHandoff();
      start = millis();          // fresh window after the detour
    }
    if (portalUploadPending) {
      portalUploadPending = false;
      doPortalUpload();
    }
    if (portalRestartPending) {
      Serial.println("🔁 Restarting on portal request.");
      delay(300);                // let the response flush first
      ESP.restart();
    }
    if (portalShouldClose) {
      Serial.println("✅ Portal closed by user.");
      break;
    }
    delay(2);
  }

  portalServer.stop();
  portalDns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(previousMode);

  Serial.printf("⚙️ Active settings: interval %d min, station %s, clock %s\n",
                sleepIntervalSec / 60, stationCode.c_str(), getTimestamp().c_str());
  Serial.println("✅ Configuration Portal finished.");
}

// ================== Display Sensor Values ==================
void printSensorValues(const DataRecord &rec) {
  Serial.println("\n");
  Serial.println("│               📊 SENSOR READINGS                    │");
  
  Serial.printf("│ 📅 Timestamp:   %-30s │\n", rec.timestamp.c_str());
  Serial.printf("│ ⏱️  Interval:    %6lu s                            │\n", (unsigned long)rec.intervalSec);

  Serial.printf("│ 🌬️  Wind avg:    %6.2f m/s  (%6.2f MPH)      │\n", rec.windMS, rec.windMPH);
  Serial.printf("│ 💨  Wind gust:   %6.2f m/s                       │\n", rec.windGustMS);
  Serial.printf("│ 🌧️  Rain:        %8.2f mm this interval          │\n", rec.rainInterval_mm);
  
  Serial.printf("│ 🌡️  Air Temp:     %6.2f °C                        │\n", rec.temperature);
  Serial.printf("│ 💧  Humidity:     %6.1f %% RH                      │\n", rec.humidity);
  Serial.printf("│ 🧭  Pressure:     %6.2f hPa  %-14s │\n", rec.pressure,
                bmeOk ? "" : "(no BME280)");
  Serial.printf("│ 🔥  Batt Temp:    %6.2f °C                        │\n", rec.batteryTemp);
  
  Serial.printf("│ ☀️  Solar:        %6.3f V  %7.2f mA              │\n", rec.solarVoltage, rec.solarCurrent_mA);
  Serial.printf("│ 🔋  Battery:      %6.3f V  %7.2f mA              │\n", rec.batteryVoltage, rec.batteryCurrent_mA);
  
  Serial.printf("│ 📊 ADC0: %6.4f V   ADC1: %6.4f V           │\n", rec.adc0, rec.adc1);
  Serial.printf("│ 📊 ADC2: %6.4f V   ADC3: %6.4f V           │\n", rec.adc2, rec.adc3);

  int dirDeg = vaneDirectionInt(rec.adc0);
  if (dirDeg >= 0) Serial.printf("│ 🧭  Wind dir:    %6d deg                         │\n", dirDeg);
  else             Serial.println("│ 🧭  Wind dir:        -- (out of range)              │");
  Serial.printf("│ 🌱  Soil:        %6.1f %%                            │\n", soilMoisturePct(rec.adc1));
  Serial.printf("│ ☀️   Irradiance: %6.1f W/m2                        │\n", irradianceWm2(rec.adc2));
  Serial.println("\n");
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 Weather Station (WIMEA API) ===");

  loadConfig();

  // Sample BOOT now - otherwise the user has to hold it down through the whole
  // WiFi connect attempt further below.
  pinMode(0, INPUT_PULLUP);
  delay(10);
  bool bootPressed = (digitalRead(0) == LOW);

  // ---- Hardware Init ----
  pinMode(RELAY_PIN, OUTPUT);
  setSensorPower(true);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);

  // ---- SD Card ----
  // Mounted first so that every logError() below actually reaches error.log.
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!mountSD()) {
    // Can't logError() this one - there is nowhere to write it.
    Serial.println("❌ SD Card mount failed (CS=5).");
  } else {
    Serial.println("✅ SD Card mounted.");
    if (!SD.exists(SD_DATA_FILE)) {
      File f = SD.open(SD_DATA_FILE, FILE_WRITE);
      if (f) {
        f.println(SD_CSV_HEADER);
        f.close();
        Serial.println("CSV header created.");
      }
    }
    if (!SD.exists(SD_ERROR_FILE)) {
      File f = SD.open(SD_ERROR_FILE, FILE_WRITE);
      if (f) { f.println("--- Error Log Started ---"); f.close(); }
    }
    if (!SD.exists(SD_QUEUE_FILE)) {
      File f = SD.open(SD_QUEUE_FILE, FILE_WRITE);
      if (f) { f.close(); }
    }
  }

  // ---- RTC ----
  // Second, so the sensor errors below get stamped with a real time.
  if (rtc.begin()) {
    rtcOk = true;
    Serial.println("RTC OK.");
    if (rtc.lostPower()) {
      // Flat coin cell, or first power-up of a new board.
      rtcTimeValid = false;
      Serial.println("⚠️ RTC lost power - time is not trustworthy until NTP sync.");
    } else {
      refreshRtcValidity();
    }
    Serial.printf("🕒 RTC reads %s EAT (%s)\n", getTimestamp().c_str(),
                  rtcTimeValid ? "valid" : "NEEDS SYNC");
  } else {
    rtcOk = false;
    rtcTimeValid = false;
    logError("RTC not found.");
  }

  // ---- Remaining I2C sensors ----
  if (ads.begin(0x48)) { ads.setGain(GAIN_TWOTHIRDS); Serial.println("ADS1115 OK."); }
  else logError("ADS1115 not found.");

  if (batterySensor.begin()) Serial.println("INA219 Batt OK.");
  else logError("INA219 Battery not found.");
  if (solarSensor.begin()) Serial.println("INA219 Solar OK.");
  else logError("INA219 Solar not found.");

  dht.begin();
  Serial.println("DHT22 ready.");

  pinMode(WIND_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), windISR, FALLING);
  pinMode(RAIN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, FALLING);
  // Counting starts now, so this is the start of the first interval.
  lastSnapshotEpoch  = currentEpoch();
  lastSnapshotMillis = millis();
  Serial.println("🌬️ Wind & 🌧️ Rain interrupts armed.");

  // ---- BME280 (pressure) ----
  bmeOk = detectBME280();

  // ---- DS18B20 ----
  ds18b20.begin();
  if (ds18b20.getDeviceCount() > 0) {
    ds18b20Ok = true;
    Serial.println("✅ DS18B20 found");
  } else {
    ds18b20Ok = false;
    Serial.println("❌ DS18B20 not found");
    logError("DS18B20 not found on pin " + String(DS18B20_PIN));
  }

  // ---- WiFi Connection ----
  WiFi.mode(WIFI_STA);
  bool wifiConnected = connectWiFi();   // Preferences first, fallback second

  // ---- Clock ----
  if (wifiConnected) {
    syncTimeFromNTP();
  }

  // ---- BOOT Button: Force Config Portal ----
  if (bootPressed) {
    Serial.println("🛠️ BOOT button pressed - forcing config portal.");
    runConfigPortal();
    // After portal, reconnect WiFi (might have new credentials)
    wifiConnected = connectWiFi();
    maybeResyncRTC();   // no-op if the portal already set the clock by hand
  }

  // ---- If still no WiFi, print warning ----
  if (!wifiConnected) {
    logError("No WiFi available at boot.");
  }

  if (!rtcTimeValid && !ntpEverSynced) {
    logError("Clock not set - readings will be logged to SD but not transmitted. "
             "Set it via the config portal or connect to WiFi.");
  }

  // Seed the schedule and the queue depth so the portal is accurate on cycle one.
  nextCycleMillis = millis() + (unsigned long)sleepIntervalSec * 1000UL;
  uint32_t bootEpoch = currentEpoch();
  if (bootEpoch > 0) nextCycleEpoch = bootEpoch + (uint32_t)sleepIntervalSec;
  queueDepthCached = queuedRecordCount();

  Serial.println(bmeOk ? "✅ BME280 present - pressure will be recorded."
                       : "ℹ️ No BME280 fitted - pressure stays empty until one is plugged in.");
  Serial.println("\n--- Entering Main Loop ---\n");
}

// ---------- LOOP (one duty cycle, then back to sleep) ----------
void loop() {
  // ============================================
  // PHASE 1: MEASURE (sensor rail up, radio still down)
  // ============================================
  sensorRailUp();             // power up, remount SD, re-init the I2C devices

  // Close out the interval: snapshot and reset the wind/rain counters, then
  // derive average wind, gust and accumulated rainfall from them.
  snapshotPulseCounters();

  // Fix the next burst time now, while the rail is up and the RTC is
  // reachable. Re-deriving it here every cycle means millis() drift over one
  // interval never accumulates across cycles.
  nextCycleMillis = millis() + (unsigned long)sleepIntervalSec * 1000UL;
  uint32_t burstEpoch = currentEpoch();
  nextCycleEpoch = (burstEpoch > 0) ? burstEpoch + (uint32_t)sleepIntervalSec : 0;

  DataRecord rec;
  readSensors(rec);
  lastRecord     = rec;       // the config portal reads this
  haveLastRecord = true;
  printSensorValues(rec);

  // The archive is permanent and takes everything - even an unstamped record,
  // because the readings are still real and the sentinel timestamp makes the
  // gap obvious. The upload queue only takes what the server can accept.
  archiveRecord(rec);
  if (rec.timestamp == TIMESTAMP_INVALID) {
    logError("Clock not set - record archived but not queued for upload.");
  } else {
    enqueueRecord(rec);
  }
  Serial.println("✅ Record written to SD.");

#if SD_INDEPENDENT_POWER
  sensorRailDown();
  Serial.println("🔌 Sensor rail OFF.");
#endif

  // ============================================
  // PHASE 2: TRANSMIT
  // ============================================
  // The queue drain covers the record just written as well as any backlog, so
  // there is no separate "send the current reading" step to duplicate it.
  if (connectWiFi()) {
    maybeResyncRTC();         // opportunistic: first chance, then once a day
    if (!processPendingQueue()) {
      Serial.println("⏳ Queue not fully drained - the rest waits for next cycle.");
    }
  } else {
    logError("No WiFi this cycle - the queue is left untouched.");
  }

  sensorRailDown();
  Serial.println("🔌 Sensor rail OFF.");

  // ============================================
  // PHASE 3: CONFIG PORTAL
  // ============================================
  runConfigPortal();

  // ============================================
  // PHASE 4: LIGHT SLEEP (still counting pulses)
  // ============================================
  // Default state from here until the next burst. The CPU is clock-gated but
  // RAM and peripherals survive, and a wind or rain pulse wakes it briefly to
  // count before going straight back down.
  //
  // The deadline was fixed at the top of this burst, while the rail was still
  // up. Nothing below reads the RTC, so this works whether or not the DS3231
  // shares the gated rail.
  long sleepLeft = (long)(nextCycleMillis - millis()) / 1000;
  if (sleepLeft < 0) sleepLeft = 0;
  Serial.printf("💤 Sleeping %ld s until the next burst (wind/rain still counted)...\n",
                sleepLeft);
  Serial.flush();   // the UART has to drain before the clock stops

  while ((long)(millis() - nextCycleMillis) < 0) {
    sleepUntilNextEventOrDeadline();
  }

  Serial.println("⏰ Sleep period finished.\n");
}

// ================== SENSOR READING ==================
void readSensors(DataRecord &rec) {
  rec.timestamp = getTimestamp();

  // ---- Wind & rain ----
  // Already computed by snapshotPulseCounters() at the top of the duty cycle;
  // reading them here would reset the counters a second time.
  rec.windMPH      = windSpeed_MPH;
  rec.windMS       = windSpeed_MS;
  rec.windGustMS   = windGust_MS;
  rec.rainInterval_mm = rain_mm;

  // ---- DHT22 (Air Temperature & Humidity) ----
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    logError("DHT22 read failed (NaN).");
    rec.temperature = NAN;      // NAN travels all the way to a NULL column
    rec.humidity    = NAN;
  } else {
    rec.temperature = t;
    rec.humidity    = h;
  }

  // ---- BME280 (station pressure) ----
  // Absent for now, so this stays NAN and the column stays NULL.
  if (bmeOk && bme.takeForcedMeasurement()) {
    float pa = bme.readPressure();                 // library returns pascals
    rec.pressure = isnan(pa) ? NAN : (pa / 100.0f);  // hPa
  } else {
    rec.pressure = NAN;
  }

  // ---- DS18B20 (Battery Temperature) ----
  if (ds18b20Ok) {
    ds18b20.requestTemperatures();
    float battTemp = ds18b20.getTempCByIndex(0);
    if (battTemp != DEVICE_DISCONNECTED_C) {
      rec.batteryTemp = battTemp;
    } else {
      logError("DS18B20 read failed (disconnected).");
      rec.batteryTemp = NAN;
      ds18b20Ok = false;
    }
  } else {
    rec.batteryTemp = NAN;
  }

  // ---- ADS1115 ----
  for (int i = 0; i < 4; i++) {
    int16_t raw = ads.readADC_SingleEnded(i);
    float v = ads.computeVolts(raw);
    switch (i) {
      case 0: rec.adc0 = v; break;
      case 1: rec.adc1 = v; break;
      case 2: rec.adc2 = v; break;
      case 3: rec.adc3 = v; break;
    }
  }

  // ---- INA219 Solar ----
  float sV = solarSensor.getBusVoltage_V();
  float sA = solarSensor.getCurrent_mA();
  if (isnan(sV) || isnan(sA)) {
    logError("INA219 Solar read failed.");
    rec.solarVoltage = NAN;     // 0.0 would read as a genuine measurement
    rec.solarCurrent_mA = NAN;
  } else {
    rec.solarVoltage = sV;
    rec.solarCurrent_mA = sA;
  }

  // ---- INA219 Battery ----
  float bV = batterySensor.getBusVoltage_V();
  float bA = batterySensor.getCurrent_mA();
  if (isnan(bV) || isnan(bA)) {
    logError("INA219 Battery read failed.");
    rec.batteryVoltage = NAN;
    rec.batteryCurrent_mA = NAN;
  } else {
    rec.batteryVoltage = bV;
    rec.batteryCurrent_mA = bA;
  }

  rec.intervalSec = (uint32_t)(lastIntervalSec + 0.5f);
  rec.sentMask    = 0;
  rec.retries     = 0;
}


// ================== CLOCK ==================
// Everything here works in East Africa Time (UTC+3). The DS3231 stores EAT
// directly, so SD rows and API payloads need no conversion.

static void formatDateTime(char *buf, size_t len,
                           int y, int mo, int d, int h, int mi, int s) {
  snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
}

// ---------- Timestamp ----------
// Returns TIMESTAMP_INVALID rather than a plausible-looking lie when the clock
// was never set - a wrong timestamp is worse than an obviously missing one.
String getTimestamp() {
  char buf[20];

  // currentEpoch() reads the DS3231 when its rail is up and carries the last
  // good reading forward on millis() when it is not.
  uint32_t e = currentEpoch();
  if (e > 0) {
    DateTime t(e);
    formatDateTime(buf, sizeof(buf), t.year(), t.month(), t.day(),
                   t.hour(), t.minute(), t.second());
    return String(buf);
  }

  // Fallback: the ESP32's own clock. Only trustworthy once NTP has answered
  // this boot, and it does not survive a power cut - hence the RTC.
  if (ntpEverSynced) {
    struct tm t;
    if (getLocalTime(&t, 0) && (t.tm_year + 1900) >= RTC_MIN_VALID_YEAR) {
      formatDateTime(buf, sizeof(buf), t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                     t.tm_hour, t.tm_min, t.tm_sec);
      return String(buf);
    }
  }

  return String(TIMESTAMP_INVALID);
}

// ---------- Is the RTC holding a believable time? ----------
bool refreshRtcValidity() {
  if (!rtcOk || !relayOn) {
    // Nothing to test with the rail down - leave the previous verdict alone.
    return rtcTimeValid;
  }
  DateTime now = rtc.now();
  rtcTimeValid = now.isValid() && now.year() >= RTC_MIN_VALID_YEAR;
  if (rtcTimeValid) {
    clockRefEpoch  = now.unixtime();
    clockRefMillis = millis();
  }
  return rtcTimeValid;
}

// ---------- NTP -> DS3231 ----------
bool syncTimeFromNTP() {
  if (WiFi.status() != WL_CONNECTED) return false;

  lastNtpAttemptMs = millis();
  Serial.println("🕒 Requesting time from NTP...");
  // The offset makes getLocalTime() hand back EAT directly, which is exactly
  // what we want to write into the DS3231.
  configTime(TZ_OFFSET_SEC, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);

  struct tm t;
  if (!getLocalTime(&t, NTP_SYNC_TIMEOUT_MS)) {
    logError("NTP sync failed (no response).");
    return false;
  }
  if ((t.tm_year + 1900) < RTC_MIN_VALID_YEAR) {
    logError("NTP returned implausible year " + String(t.tm_year + 1900));
    return false;
  }

  DateTime ntpTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                   t.tm_hour, t.tm_min, t.tm_sec);

  if (rtcOk) {
    DateTime before = rtc.now();
    long drift = before.isValid()
                 ? (long)ntpTime.unixtime() - (long)before.unixtime()
                 : 0;
    rtc.adjust(ntpTime);
    rtcTimeValid   = true;
    clockRefEpoch  = ntpTime.unixtime();
    clockRefMillis = millis();
    Serial.printf("✅ RTC set from NTP (was off by %+ld s)\n", drift);
  } else {
    Serial.println("⚠️ NTP time acquired, but there is no RTC to store it in.");
  }

  ntpEverSynced = true;
  lastNtpSyncMs = millis();
  Serial.printf("🕒 Time is now %s EAT\n", getTimestamp().c_str());
  return true;
}

// ---------- Sync on first opportunity, then once a day ----------
void maybeResyncRTC() {
  if (WiFi.status() != WL_CONNECTED) return;

  // Synced recently enough - the DS3231 drifts about a minute a year.
  if (ntpEverSynced && (millis() - lastNtpSyncMs) < NTP_RESYNC_INTERVAL_MS) return;

  // Never synced: keep trying, but not every cycle. A WiFi link that associates
  // without a route to the internet would otherwise burn NTP_SYNC_TIMEOUT_MS and
  // an error-log line on every single wake.
  if (!ntpEverSynced && lastNtpAttemptMs != 0 &&
      (millis() - lastNtpAttemptMs) < NTP_RETRY_INTERVAL_MS) return;

  syncTimeFromNTP();
}

// ---------- Manual clock set (config portal) ----------
// Accepts "YYYY-MM-DD HH:MM:SS", "YYYY-MM-DD HH:MM", or the browser's
// datetime-local form "YYYY-MM-DDTHH:MM". Interpreted as EAT.
bool setRtcFromString(const String &input) {
  String s = input;
  s.trim();
  s.replace("T", " ");
  if (s.length() < 16) return false;

  int y = 0, mo = 0, d = 0, h = 0, mi = 0, se = 0;
  int n = sscanf(s.c_str(), "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &se);
  if (n < 5) return false;
  if (n == 5) se = 0;

  if (y < RTC_MIN_VALID_YEAR || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
      h < 0 || h > 23 || mi < 0 || mi > 59 || se < 0 || se > 59) return false;

  DateTime dt(y, mo, d, h, mi, se);
  if (!dt.isValid()) return false;

  // The config portal runs with the sensor rail down and the DS3231 is on that
  // rail, so the write would go nowhere. Bring the rail up just long enough to
  // land it, then put it back as it was.
  bool railWasDown = !relayOn;
  if (railWasDown) {
    setSensorPower(true);
    delay(SENSOR_RAIL_SETTLE_MS);
    rtcOk = rtc.begin();
  }

  if (!rtcOk) {
    if (railWasDown) setSensorPower(false);
    logError("Manual clock set requested but the RTC did not answer.");
    return false;
  }

  rtc.adjust(dt);
  rtcTimeValid   = true;
  clockRefEpoch  = dt.unixtime();
  clockRefMillis = millis();

  if (railWasDown) setSensorPower(false);
  Serial.printf("🕒 RTC set manually to %s EAT\n", getTimestamp().c_str());
  return true;
}

// ================== RECORD STORAGE & UPLOAD QUEUE ==================
// Two files, two jobs:
//   datalog.txt  permanent archive, appended once per reading, never rewritten
//   queue.txt    upload buffer; a record leaves it only once delivered
// A record is written to both during the burst, so a failed upload can never
// lose data - the worst case is a delayed one.

// Queue lines carry two extra columns the archive does not need: the delivery
// mask (which endpoints already accepted this record) and a retry count.
// A missing reading is written as an empty field, not as a number. Writing
// String(NAN, 2) would emit "nan", and reading it back with toFloat() yields
// 0.0 - silently converting "no reading" into a real-looking zero on the first
// queue round-trip.
static String fmtField(float v, int dp) {
  if (isnan(v)) return String();
  return String(v, dp);
}

static float parseField(const String &f) {
  if (f.length() == 0) return NAN;
  // -999 was the old sentinel; treat it as missing so pre-existing queue lines
  // do not come back as a temperature of minus nine hundred.
  float v = f.toFloat();
  if (v <= -998.0f && v >= -1000.0f) return NAN;
  return v;
}

// Queue lines carry two extra columns the archive does not need: the delivery
// mask (which endpoints already accepted this record) and a retry count.
String recordToLine(const DataRecord &rec, bool includeQueueFields) {
  // Queue lines lead with a format tag so a stale one is rejected rather than
  // parsed with today's column positions. The archive has a CSV header instead
  // and is never read back, so it does not carry the tag.
  String line = includeQueueFields ? String(QUEUE_FORMAT_TAG) + "," : String();
  line += rec.timestamp + "," +
                fmtField(rec.windMPH, 2) + "," +
                fmtField(rec.windMS, 2) + "," +
                fmtField(rec.windGustMS, 2) + "," +
                fmtField(rec.rainInterval_mm, 2) + "," +
                fmtField(rec.temperature, 2) + "," +
                fmtField(rec.humidity, 1) + "," +
                fmtField(rec.pressure, 2) + "," +
                fmtField(rec.batteryTemp, 2) + "," +
                fmtField(rec.solarVoltage, 3) + "," +
                fmtField(rec.solarCurrent_mA, 2) + "," +
                fmtField(rec.batteryVoltage, 3) + "," +
                fmtField(rec.batteryCurrent_mA, 2) + "," +
                fmtField(rec.adc0, 4) + "," +
                fmtField(rec.adc1, 4) + "," +
                fmtField(rec.adc2, 4) + "," +
                fmtField(rec.adc3, 4) + "," +
                String(rec.intervalSec);
  if (includeQueueFields) {
    line += "," + String(rec.sentMask) + "," + String(rec.retries);
  }
  return line;
}

bool parseLineToRecord(const String &line, DataRecord &rec) {
  // Count separators before indexing anything - walking a corrupt line into a
  // fixed array is how this function used to write past the end.
  int commas = 0;
  for (unsigned int i = 0; i < line.length(); i++) {
    if (line.charAt(i) == ',') commas++;
  }
  if (commas + 1 != REC_FIELDS_QUEUE) return false;

  String fields[REC_FIELDS_QUEUE];
  int start = 0, n = 0;
  for (int i = 0; i < commas; i++) {
    int comma = line.indexOf(',', start);
    fields[n++] = line.substring(start, comma);
    start = comma + 1;
  }
  fields[n++] = line.substring(start);

  // Written by a different firmware layout - the columns would not line up.
  if (fields[0] != QUEUE_FORMAT_TAG) return false;

  rec.timestamp         = fields[1];
  rec.windMPH           = parseField(fields[2]);
  rec.windMS            = parseField(fields[3]);
  rec.windGustMS        = parseField(fields[4]);
  rec.rainInterval_mm   = parseField(fields[5]);
  rec.temperature       = parseField(fields[6]);
  rec.humidity          = parseField(fields[7]);
  rec.pressure          = parseField(fields[8]);
  rec.batteryTemp       = parseField(fields[9]);
  rec.solarVoltage      = parseField(fields[10]);
  rec.solarCurrent_mA   = parseField(fields[11]);
  rec.batteryVoltage    = parseField(fields[12]);
  rec.batteryCurrent_mA = parseField(fields[13]);
  rec.adc0              = parseField(fields[14]);
  rec.adc1              = parseField(fields[15]);
  rec.adc2              = parseField(fields[16]);
  rec.adc3              = parseField(fields[17]);
  rec.intervalSec       = (uint32_t)fields[18].toInt();
  rec.sentMask          = (uint8_t)fields[19].toInt();
  rec.retries           = (uint8_t)fields[20].toInt();
  return true;
}

void archiveRecord(const DataRecord &rec) {
  if (!appendSD(recordToLine(rec, false), SD_DATA_FILE)) {
    logError("Failed to append to the archive.");
  }
}

void enqueueRecord(const DataRecord &rec) {
  if (!appendSD(recordToLine(rec, true), SD_QUEUE_FILE)) {
    logError("Failed to append to the upload queue.");
    return;
  }
  queueDepthCached++;
}

// ================== TRANSMISSION ==================

enum SendOutcome {
  SEND_OK,          // every endpoint accepted it
  SEND_TRANSIENT,   // network or server problem - worth retrying later
  SEND_PERMANENT    // the server refused it; retrying will not help
};

static int httpPost(const String &url, const String &payload) {
  HTTPClient http;
  if (!http.begin(url)) return -1;
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(10000);
  int code = http.POST(payload);
  http.end();
  return code;
}

// Only add a key when the reading is real. An omitted key leaves the Django
// column NULL, which is the honest representation of a failed sensor - a -999
// or a 0.0 would be indistinguishable from a measurement.
static void putIfReal(JsonDocument &doc, const char *key, float v) {
  if (!isnan(v)) doc[key] = v;
}

static String buildWeatherJson(const DataRecord &rec) {
  JsonDocument doc;
  doc["station_code"] = stationCode;
  doc["timestamp"]    = rec.timestamp;
  doc["interval_s"]   = rec.intervalSec;

  putIfReal(doc, "temperature", rec.temperature);   // degC, DHT22
  putIfReal(doc, "humidity",    rec.humidity);      // %RH, DHT22
  putIfReal(doc, "pressure",    rec.pressure);      // hPa, BME280 station pressure
  putIfReal(doc, "rain",        rec.rainInterval_mm);  // mm this interval
  putIfReal(doc, "wind_speed",  rec.windMS);        // m/s, interval mean

  // Raw ADS1115 pin voltages, plus the values derived from them. Both are sent:
  // the raw volts stay ground truth, so a wrong constant in calibration.h is
  // repairable with one SQL UPDATE instead of being lost data.
  putIfReal(doc, "wind_direction_v",  rec.adc0);
  putIfReal(doc, "soil_moisture_v",   rec.adc1);
  putIfReal(doc, "solar_radiation_v", rec.adc2);

  int dir = vaneDirectionInt(rec.adc0);
  if (dir >= 0) doc["wind_direction"] = dir;             // integer 0-359
  putIfReal(doc, "soil_moisture",   soilMoisturePct(rec.adc1));
  putIfReal(doc, "solar_radiation", irradianceWm2(rec.adc2));

  String out;
  serializeJson(doc, out);
  return out;
}

static String buildVoltageJson(const DataRecord &rec) {
  JsonDocument doc;
  doc["station_code"] = stationCode;
  doc["timestamp"]    = rec.timestamp;

  putIfReal(doc, "volt_batt",    rec.batteryVoltage);
  putIfReal(doc, "volt_solar",   rec.solarVoltage);
  putIfReal(doc, "battery_temp", rec.batteryTemp);   // degC, DS18B20

  String out;
  serializeJson(doc, out);
  return out;
}

static String buildCurrentJson(const DataRecord &rec) {
  JsonDocument doc;
  doc["station_code"] = stationCode;
  doc["timestamp"]    = rec.timestamp;

  // The API stores amps; the INA219 reports milliamps.
  if (!isnan(rec.batteryCurrent_mA)) doc["curr_batt"]  = rec.batteryCurrent_mA / 1000.0;
  if (!isnan(rec.solarCurrent_mA))   doc["curr_solar"] = rec.solarCurrent_mA / 1000.0;

  String out;
  serializeJson(doc, out);
  return out;
}

// Post only the endpoints this record still owes. Without the mask a partial
// success would be re-sent in full next cycle and duplicate rows server-side.
static SendOutcome sendRecord(DataRecord &rec) {
  const char *urls[3]  = { WEATHER_URL, VOLTAGE_URL, CURRENT_URL };
  const char *names[3] = { "weather", "voltage", "current" };
  const uint8_t bits[3] = { SENT_WEATHER, SENT_VOLTAGE, SENT_CURRENT };

  bool anyTransient = false;
  bool anyPermanent = false;

  for (int i = 0; i < 3; i++) {
    if (rec.sentMask & bits[i]) continue;         // already delivered

    String payload;
    if (i == 0)      payload = buildWeatherJson(rec);
    else if (i == 1) payload = buildVoltageJson(rec);
    else             payload = buildCurrentJson(rec);

    // Print exactly what goes on the wire - this is the ground truth when a
    // field is not landing in the database.
    Serial.printf("📦 POST %s\n   %s\n", names[i], payload.c_str());

    int code = httpPost(urls[i], payload);

    if (code >= 200 && code < 300) {
      rec.sentMask |= bits[i];
      Serial.printf("✅ %s accepted (HTTP %d)\n", names[i], code);
    } else if (code >= 400 && code < 500) {
      anyPermanent = true;
      logError(String(names[i]) + " POST refused (HTTP " + String(code) + ")");
    } else {
      // 5xx, timeout or no connection: the link or the server is down, so
      // there is nothing to gain from trying the remaining endpoints now.
      anyTransient = true;
      logError(String(names[i]) + " POST failed (HTTP " + String(code) + ")");
      break;
    }
  }

  if (rec.sentMask == SENT_ALL) return SEND_OK;
  if (anyTransient)             return SEND_TRANSIENT;
  if (anyPermanent)             return SEND_PERMANENT;
  return SEND_TRANSIENT;
}

// ================== QUEUE DRAIN ==================
// Streams through a temp file rather than loading the queue into RAM: a week
// offline is ~2000 records, which as a vector of Strings will exhaust the heap.
// The live file is only replaced once the replacement is closed, so a power cut
// mid-drain costs nothing.
bool processPendingQueue() {
  if (!SD.exists(SD_QUEUE_FILE)) return true;

  File in = SD.open(SD_QUEUE_FILE, FILE_READ);
  if (!in) {
    logError("Cannot open queue for reading.");
    return false;
  }
  if (in.size() == 0) {
    in.close();
    return true;
  }

  if (SD.exists(SD_QUEUE_TMP)) SD.remove(SD_QUEUE_TMP);
  File out = SD.open(SD_QUEUE_TMP, FILE_WRITE);
  if (!out) {
    in.close();
    logError("Cannot open queue temp file - leaving queue untouched.");
    return false;
  }

  int sent = 0, refused = 0, kept = 0;
  int budget = MAX_PENDING_CYCLE;
  bool stopSending = false;

  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // Once the link has failed, copy the rest across verbatim and try again
    // next cycle. Same when this cycle's send budget is spent.
    if (stopSending || budget <= 0) {
      out.println(line);
      kept++;
      continue;
    }

    DataRecord rec;
    if (!parseLineToRecord(line, rec)) {
      logError("Malformed queue line moved to rejected.txt");
      appendSD(line, SD_REJECTED_FILE);
      refused++;
      continue;
    }

    budget--;
    SendOutcome outcome = sendRecord(rec);
    rec.retries++;

    if (outcome == SEND_OK) {
      sent++;
      continue;                                   // drops off the queue
    }

    if (outcome == SEND_PERMANENT || rec.retries >= MAX_RETRIES) {
      // Park it so one bad record cannot block everything behind it forever.
      appendSD(recordToLine(rec, true), SD_REJECTED_FILE);
      logError("Record " + rec.timestamp + " moved to rejected.txt after " +
               String(rec.retries) + " attempt(s).");
      refused++;
      continue;
    }

    out.println(recordToLine(rec, true));
    kept++;
    stopSending = true;
  }

  in.close();
  out.close();

  // Swap only now that the replacement is complete on disk.
  if (!SD.remove(SD_QUEUE_FILE)) {
    logError("Could not remove the old queue file.");
    SD.remove(SD_QUEUE_TMP);
    return false;
  }
  if (!SD.rename(SD_QUEUE_TMP, SD_QUEUE_FILE)) {
    logError("Could not rename the queue temp file - queue is in queue.tmp.");
    return false;
  }

  queueDepthCached = kept;
  Serial.printf("📤 Queue: %d sent, %d rejected, %d still waiting.\n",
                sent, refused, kept);
  return (kept == 0);
}
