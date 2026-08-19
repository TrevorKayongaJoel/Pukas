#include "config.h"

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

// Interrupt variables
volatile unsigned int windPulseCount = 0;
unsigned long lastWindReadTime = 0;
float windSpeed_MPH = 0.0;
float windSpeed_MS  = 0.0;
volatile unsigned int rainPulseCount = 0;
volatile unsigned long lastRainTriggerTime = 0;
float totalRain_mm = 0.0;

// ---------- Interrupt Service Routines ----------
void IRAM_ATTR windISR() { windPulseCount++; }
void IRAM_ATTR rainISR() {
  unsigned long now = micros();
  if (now - lastRainTriggerTime > DEBOUNCE_US) {
    rainPulseCount++;
    lastRainTriggerTime = now;
  }
}

// ---------- Config Load / Save ----------
void loadConfig() {
  preferences.begin("weather", false);
  sleepIntervalSec = preferences.getInt("sleepInt", 300);
  stationCode = preferences.getString("station", "AWS-UG-001");
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
void logError(const String &message) {
  String timestamp = getTimestamp();
  String line = timestamp + " | " + message;
  appendSD(line, SD_ERROR_FILE);
  Serial.println("⚠️ " + line);
}

// ---------- Generic SD Append ----------
void appendSD(const String &line, const char *filename) {
  File f = SD.open(filename, FILE_APPEND);
  if (f) {
    f.println(line);
    f.close();
  } else {
    Serial.printf("❌ Failed to write to %s\n", filename);
  }
}

// ---------- WiFi Connection (Preferences first, fallback hardcoded) ----------
bool connectWiFi() {
  // Check if already connected
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("📶 WiFi already connected.");
    return true;
  }

  // Try to get saved WiFi from Preferences
  preferences.begin("wifi", false);
  String savedSSID = preferences.getString("ssid", "");
  String savedPassword = preferences.getString("pass", "");
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
// The portal is primarily a settings page (sleep interval, station code,
// clock); provisioning WiFi is the secondary job.
//
// Settings are applied from setSaveParamsCallback(), which fires the moment the
// form is submitted. This matters: startConfigPortal() returns false whenever
// the portal simply times out - the normal outcome in the field - so anything
// read after it returns would be silently discarded.

static WiFiManagerParameter *g_paramSleep   = nullptr;
static WiFiManagerParameter *g_paramStation = nullptr;
static WiFiManagerParameter *g_paramTime    = nullptr;

static void onPortalParamsSaved() {
  if (g_paramSleep == nullptr || g_paramStation == nullptr) return;

  int    newInterval = 0;    // 0 = leave unchanged
  String newStation  = "";   // "" = leave unchanged

  // ---- Sleep interval ----
  String sleepStr = String(g_paramSleep->getValue());
  sleepStr.trim();
  if (sleepStr.length() > 0) {
    int requested = sleepStr.toInt();
    if (requested <= 0) {
      // toInt() returns 0 for non-numeric input, so this catches typos that
      // would otherwise look exactly like a successful save.
      Serial.printf("⚠️ Ignoring invalid sleep time '%s'.\n", sleepStr.c_str());
    } else {
      if (requested < SLEEP_MIN_SEC) requested = SLEEP_MIN_SEC;
      if (requested > SLEEP_MAX_SEC) requested = SLEEP_MAX_SEC;
      if (requested != sleepIntervalSec) {
        newInterval = requested;
        Serial.printf("⏱️ Sleep time %d s -> %d s\n", sleepIntervalSec, requested);
      }
    }
  }

  // ---- Station code ----
  String stationStr = String(g_paramStation->getValue());
  stationStr.trim();
  if (stationStr.length() > 0 && stationStr != stationCode) {
    newStation = stationStr;
  }

  if (newInterval > 0 || newStation.length() > 0) {
    saveConfig(newInterval, newStation);
  } else {
    Serial.println("ℹ️ Settings submitted, nothing changed.");
  }

  // ---- Manual clock set (blank = keep current time) ----
  if (g_paramTime != nullptr) {
    String timeStr = String(g_paramTime->getValue());
    timeStr.trim();
    if (timeStr.length() > 0) {
      if (!setRtcFromString(timeStr)) {
        logError("Portal clock value rejected: " + timeStr);
      }
    }
  }
}

void runConfigPortal() {
  Serial.printf("🌐 Starting Configuration Portal (%d seconds)...\n", PORTAL_TIMEOUT);

  // Save current WiFi mode and switch to AP+STA
  wifi_mode_t currentMode = WiFi.getMode();
  WiFi.mode(WIFI_AP_STA);
  delay(100);  // Allow mode change to settle

  WiFiManager wm;
  wm.setConfigPortalTimeout(PORTAL_TIMEOUT);
  wm.setAPCallback([](WiFiManager *myWiFiManager) {
    Serial.println("📶 Config portal AP started: ESP_Config");
    Serial.println("🔗 Connect to 'ESP_Config' and open http://192.168.4.1");
  });

  // Land on "Setup" rather than the WiFi scanner - settings are the usual reason
  // anyone opens this portal.
  std::vector<const char *> menu = {"param", "wifi", "info", "sep", "restart", "exit"};
  wm.setMenu(menu);

  // ---- Sleep interval ----
  char sleepValue[8];
  snprintf(sleepValue, sizeof(sleepValue), "%d", sleepIntervalSec);
  char sleepLabel[64];
  snprintf(sleepLabel, sizeof(sleepLabel),
           "Sleep Time (seconds, %d-%d)", SLEEP_MIN_SEC, SLEEP_MAX_SEC);
  char sleepAttrs[64];
  snprintf(sleepAttrs, sizeof(sleepAttrs),
           "type='number' min='%d' max='%d'", SLEEP_MIN_SEC, SLEEP_MAX_SEC);
  WiFiManagerParameter custom_sleep("sleep", sleepLabel, sleepValue, 6, sleepAttrs);

  // ---- Station code ----
  char stationValue[STATION_CODE_MAXLEN + 1];
  snprintf(stationValue, sizeof(stationValue), "%s", stationCode.c_str());
  WiFiManagerParameter custom_station("station", "Station Code", stationValue,
                                      STATION_CODE_MAXLEN);

  // ---- Clock ----
  // Deliberately rendered blank: pre-filling it would re-apply a stale time on
  // every save, losing however long the page sat open.
  String nowStr = getTimestamp();
  char timeLabel[96];
  snprintf(timeLabel, sizeof(timeLabel),
           "Set Clock, EAT (now: %s) - blank keeps current", nowStr.c_str());
  WiFiManagerParameter custom_time("rtctime", timeLabel, "", 19,
                                   "placeholder='YYYY-MM-DD HH:MM:SS'");

  wm.addParameter(&custom_sleep);
  wm.addParameter(&custom_station);
  wm.addParameter(&custom_time);

  g_paramSleep   = &custom_sleep;
  g_paramStation = &custom_station;
  g_paramTime    = &custom_time;
  wm.setSaveParamsCallback(onPortalParamsSaved);

  // false simply means "no new WiFi credentials were submitted" - a timeout is
  // the normal case, not an error.
  bool gotNewWiFi = wm.startConfigPortal("ESP_Config", NULL);

  // The parameters above are stack-local; drop the pointers before they die.
  g_paramSleep   = nullptr;
  g_paramStation = nullptr;
  g_paramTime    = nullptr;

  if (gotNewWiFi) {
    preferences.begin("wifi", false);
    String savedSSID = preferences.getString("ssid", "");
    preferences.end();
    Serial.printf("✅ WiFi credentials saved: %s\n",
                  savedSSID.length() > 0 ? savedSSID.c_str() : WiFi.SSID().c_str());
  } else {
    Serial.println("ℹ️ Portal closed (timed out or no WiFi change).");
  }

  // Restore previous WiFi mode
  WiFi.mode(currentMode);

  Serial.printf("⚙️ Active settings: sleep %d s, station %s, clock %s\n",
                sleepIntervalSec, stationCode.c_str(), getTimestamp().c_str());
  Serial.println("✅ Configuration Portal finished.");
}

// ================== Display Sensor Values ==================
void printSensorValues(const DataRecord &rec) {
  Serial.println("\n");
  Serial.println("│               📊 SENSOR READINGS                    │");
  
  Serial.printf("│ 📅 Timestamp:   %-30s │\n", rec.timestamp.c_str());

  Serial.printf("│ 🌬️  Wind Speed:  %6.2f MPH   (%6.2f m/s)     │\n", rec.windMPH, rec.windMS);
  Serial.printf("│ 🌧️  Rain Total:  %8.2f mm                        │\n", rec.rainTotal_mm);
  
  Serial.printf("│ 🌡️  Air Temp:     %6.2f °C                        │\n", rec.temperature);
  Serial.printf("│ 💧  Humidity:     %6.1f %% RH                      │\n", rec.humidity);
  Serial.printf("│ 🔥  Batt Temp:    %6.2f °C                        │\n", rec.batteryTemp);
  
  Serial.printf("│ ☀️  Solar:        %6.3f V  %7.2f mA              │\n", rec.solarVoltage, rec.solarCurrent_mA);
  Serial.printf("│ 🔋  Battery:      %6.3f V  %7.2f mA              │\n", rec.batteryVoltage, rec.batteryCurrent_mA);
  
  Serial.printf("│ 📊 ADC0: %6.4f V   ADC1: %6.4f V           │\n", rec.adc0, rec.adc1);
  Serial.printf("│ 📊 ADC2: %6.4f V   ADC3: %6.4f V           │\n", rec.adc2, rec.adc3);
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
  digitalWrite(RELAY_PIN, RELAY_ACTIVE);
  relayOn = true;
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);

  // ---- SD Card ----
  // Mounted first so that every logError() below actually reaches error.log.
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    // Can't logError() this one - there is nowhere to write it.
    Serial.println("❌ SD Card mount failed (CS=5).");
  } else {
    Serial.println("✅ SD Card mounted.");
    if (!SD.exists(SD_DATA_FILE)) {
      File f = SD.open(SD_DATA_FILE, FILE_WRITE);
      if (f) {
        f.println("Timestamp,WindMPH,WindMS,Rain_mm,Temp_C,Humidity%,BatteryTemp_C,Solar_V,Solar_mA,Batt_V,Batt_mA,ADC0,ADC1,ADC2,ADC3");
        f.close();
        Serial.println("CSV header created.");
      }
    }
    if (!SD.exists(SD_ERROR_FILE)) {
      File f = SD.open(SD_ERROR_FILE, FILE_WRITE);
      if (f) { f.println("--- Error Log Started ---"); f.close(); }
    }
    if (!SD.exists(SD_PENDING_FILE)) {
      File f = SD.open(SD_PENDING_FILE, FILE_WRITE);
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
  lastWindReadTime = millis();
  pinMode(RAIN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, FALLING);
  Serial.println("🌬️ Wind & 🌧️ Rain interrupts armed.");

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

  Serial.println("\n--- Entering Main Loop ---\n");
}

// ---------- LOOP (Duty Cycle) ----------
void loop() {
  // ============================================
  // PHASE 1: HIGH POWER (Read & Transmit)
  // ============================================
  digitalWrite(RELAY_PIN, RELAY_ACTIVE);
  relayOn = true;
  delay(500);

  // 1) Check/Reconnect WiFi first, then correct the clock - both have to happen
  //    before we stamp a record, or the reading carries a stale time.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("📶 WiFi disconnected. Attempting reconnect...");
    connectWiFi();   // Will try Preferences first, then fallback
  }
  maybeResyncRTC();

  // 2) Read current sensors
  DataRecord rec;
  readSensors(rec);

  // 3) Display values on serial monitor
  printSensorValues(rec);

  // 4) Log to SD - always, even with an unknown time. The readings are still
  //    worth keeping, and the sentinel timestamp makes the gap obvious.
  logToSD(rec);
  Serial.println("✅ Data logged to SD card.");

  // 5) Transmit
  if (rec.timestamp == TIMESTAMP_INVALID) {
    logError("Clock not set - record kept on SD only, not transmitted.");
  } else if (WiFi.status() == WL_CONNECTED) {
    Serial.println("📶 WiFi connected.");
    Serial.print("📶 IP Address: ");
    Serial.println(WiFi.localIP());

    // 5a) Process pending queue (FIFO)
    bool queueCleared = processPendingQueue();
    if (!queueCleared) {
      Serial.println("⏳ Pending queue still has records (some failed).");
    }

    // 5b) Send current reading
    if (sendToServer(rec)) {
      Serial.println("📤 Current reading sent to server.");
    } else {
      enqueueRecord(rec);
      Serial.println("📥 Current reading added to pending queue (send failed).");
    }
  } else {
    logError("WiFi not connected.");
    enqueueRecord(rec);
    Serial.println("📥 Current reading added to pending queue (WiFi unavailable).");
  }

  digitalWrite(RELAY_PIN, !RELAY_ACTIVE);
  relayOn = false;
  Serial.println("🔌 Relay OFF.");

  // ============================================
  // PHASE 2: CONFIGURATION PORTAL (ALWAYS STARTS)
  // ============================================
  runConfigPortal();   // <-- Always runs, even if WiFi is connected

  // ============================================
  // PHASE 3: LOW POWER IDLE (Counting interrupts)
  // ============================================
  Serial.printf("💤 Idle counting for %d seconds...\n", sleepIntervalSec);
  unsigned long idleStart = millis();
  while (millis() - idleStart < (unsigned long)sleepIntervalSec * 1000) {
    delay(1000);
    // heartbeat every 10 seconds
    if ((millis() - idleStart) % 10000 < 1000) {
      static unsigned long lastHeartbeat = 0;
      if (millis() - lastHeartbeat > 10000) {
        lastHeartbeat = millis();
        Serial.printf("⏳ Counting... %lu s elapsed.\n", (millis() - idleStart) / 1000);
      }
    }
  }
  Serial.println("⏰ Idle period finished.\n");
}

// ================== SENSOR READING ==================
void readSensors(DataRecord &rec) {
  rec.timestamp = getTimestamp();

  // ---- Wind ----
  unsigned long now = millis();
  float interval = (now - lastWindReadTime) / 1000.0;
  if (interval >= WIND_INTERVAL) {
    noInterrupts();
    unsigned int count = windPulseCount;
    windPulseCount = 0;
    interrupts();
    float freq = count / interval;
    windSpeed_MPH = freq * 1.5;
    windSpeed_MS  = freq * 0.67;
    lastWindReadTime = now;
  }
  rec.windMPH = windSpeed_MPH;
  rec.windMS  = windSpeed_MS;

  // ---- Rain ----
  noInterrupts();
  unsigned int tips = rainPulseCount;
  rainPulseCount = 0;
  interrupts();
  totalRain_mm += tips * MM_PER_TIP;
  rec.rainTotal_mm = totalRain_mm;

  // ---- DHT22 (Air Temperature & Humidity) ----
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    logError("DHT22 read failed (NaN).");
    rec.temperature = -999.0;
    rec.humidity    = -999.0;
  } else {
    rec.temperature = t;
    rec.humidity    = h;
  }

  // ---- DS18B20 (Battery Temperature) ----
  if (ds18b20Ok) {
    ds18b20.requestTemperatures();
    float battTemp = ds18b20.getTempCByIndex(0);
    if (battTemp != DEVICE_DISCONNECTED_C) {
      rec.batteryTemp = battTemp;
    } else {
      logError("DS18B20 read failed (disconnected).");
      rec.batteryTemp = -999.0;
      ds18b20Ok = false;
    }
  } else {
    rec.batteryTemp = -999.0;
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
    rec.solarVoltage = 0.0;
    rec.solarCurrent_mA = 0.0;
  } else {
    rec.solarVoltage = sV;
    rec.solarCurrent_mA = sA;
  }

  // ---- INA219 Battery ----
  float bV = batterySensor.getBusVoltage_V();
  float bA = batterySensor.getCurrent_mA();
  if (isnan(bV) || isnan(bA)) {
    logError("INA219 Battery read failed.");
    rec.batteryVoltage = 0.0;
    rec.batteryCurrent_mA = 0.0;
  } else {
    rec.batteryVoltage = bV;
    rec.batteryCurrent_mA = bA;
  }

  rec.retries = 0;
}

// ---------- SD Logging ----------
void logToSD(const DataRecord &rec) {
  String line = rec.timestamp + "," +
                String(rec.windMPH, 2) + "," +
                String(rec.windMS, 2) + "," +
                String(rec.rainTotal_mm, 2) + "," +
                String(rec.temperature, 2) + "," +
                String(rec.humidity, 1) + "," +
                String(rec.batteryTemp, 2) + "," +
                String(rec.solarVoltage, 3) + "," +
                String(rec.solarCurrent_mA, 2) + "," +
                String(rec.batteryVoltage, 3) + "," +
                String(rec.batteryCurrent_mA, 2) + "," +
                String(rec.adc0, 4) + "," +
                String(rec.adc1, 4) + "," +
                String(rec.adc2, 4) + "," +
                String(rec.adc3, 4);
  appendSD(line, SD_DATA_FILE);
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

  // Preferred source: the battery-backed DS3231.
  if (rtcOk && rtcTimeValid) {
    DateTime now = rtc.now();
    if (now.isValid() && now.year() >= RTC_MIN_VALID_YEAR) {
      formatDateTime(buf, sizeof(buf), now.year(), now.month(), now.day(),
                     now.hour(), now.minute(), now.second());
      return String(buf);
    }
    rtcTimeValid = false;   // RTC went bad underneath us
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
  if (!rtcOk) {
    rtcTimeValid = false;
    return false;
  }
  DateTime now = rtc.now();
  rtcTimeValid = now.isValid() && now.year() >= RTC_MIN_VALID_YEAR;
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
    rtcTimeValid = true;
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

  if (!rtcOk) {
    logError("Manual clock set requested but no RTC is present.");
    return false;
  }

  rtc.adjust(dt);
  rtcTimeValid = true;
  Serial.printf("🕒 RTC set manually to %s EAT\n", getTimestamp().c_str());
  return true;
}

// ================== PERSISTENT QUEUE FUNCTIONS ==================

String recordToLine(const DataRecord &rec, bool includeRetries = true) {
  String line = rec.timestamp + "," +
                String(rec.windMPH, 2) + "," +
                String(rec.windMS, 2) + "," +
                String(rec.rainTotal_mm, 2) + "," +
                String(rec.temperature, 2) + "," +
                String(rec.humidity, 1) + "," +
                String(rec.batteryTemp, 2) + "," +
                String(rec.solarVoltage, 3) + "," +
                String(rec.solarCurrent_mA, 2) + "," +
                String(rec.batteryVoltage, 3) + "," +
                String(rec.batteryCurrent_mA, 2) + "," +
                String(rec.adc0, 4) + "," +
                String(rec.adc1, 4) + "," +
                String(rec.adc2, 4) + "," +
                String(rec.adc3, 4);
  if (includeRetries) {
    line += "," + String(rec.retries);
  }
  return line;
}

bool parseLineToRecord(const String &line, DataRecord &rec) {
  int start = 0;
  int commaIdx;
  String fields[16];
  int fieldCount = 0;
  
  while ((commaIdx = line.indexOf(',', start)) != -1 && fieldCount < 16) {
    fields[fieldCount++] = line.substring(start, commaIdx);
    start = commaIdx + 1;
  }
  if (fieldCount >= 15) {
    fields[fieldCount++] = line.substring(start);
  } else {
    return false;
  }

  rec.timestamp = fields[0];
  rec.windMPH = fields[1].toFloat();
  rec.windMS = fields[2].toFloat();
  rec.rainTotal_mm = fields[3].toFloat();
  rec.temperature = fields[4].toFloat();
  rec.humidity = fields[5].toFloat();
  rec.batteryTemp = (fieldCount > 6) ? fields[6].toFloat() : -999.0;
  rec.solarVoltage = (fieldCount > 7) ? fields[7].toFloat() : 0.0;
  rec.solarCurrent_mA = (fieldCount > 8) ? fields[8].toFloat() : 0.0;
  rec.batteryVoltage = (fieldCount > 9) ? fields[9].toFloat() : 0.0;
  rec.batteryCurrent_mA = (fieldCount > 10) ? fields[10].toFloat() : 0.0;
  rec.adc0 = (fieldCount > 11) ? fields[11].toFloat() : 0.0;
  rec.adc1 = (fieldCount > 12) ? fields[12].toFloat() : 0.0;
  rec.adc2 = (fieldCount > 13) ? fields[13].toFloat() : 0.0;
  rec.adc3 = (fieldCount > 14) ? fields[14].toFloat() : 0.0;
  rec.retries = (fieldCount > 15) ? fields[15].toInt() : 0;
  return true;
}

void enqueueRecord(const DataRecord &rec) {
  String line = recordToLine(rec, true);
  appendSD(line, SD_PENDING_FILE);
}

void writePendingFile(const std::vector<DataRecord> &records) {
  File f = SD.open(SD_PENDING_FILE, FILE_WRITE);
  if (!f) {
    logError("Failed to open pending file for writing.");
    return;
  }
  for (const auto &rec : records) {
    String line = recordToLine(rec, true);
    f.println(line);
  }
  f.close();
}

bool processPendingQueue() {
  if (!SD.exists(SD_PENDING_FILE)) {
    return true;
  }

  File f = SD.open(SD_PENDING_FILE, FILE_READ);
  if (!f) {
    logError("Cannot open pending file for reading.");
    return true;
  }

  std::vector<DataRecord> records;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    DataRecord rec;
    if (parseLineToRecord(line, rec)) {
      records.push_back(rec);
    } else {
      logError("Malformed line in pending file: " + line);
    }
  }
  f.close();

  if (records.empty()) return true;

  int processed = 0;
  bool allSuccess = true;
  std::vector<DataRecord> remaining;

  while (!records.empty() && processed < MAX_PENDING_CYCLE) {
    DataRecord rec = records.front();
    records.erase(records.begin());
    processed++;

    Serial.printf("⏳ Sending queued record from %s (retry %d)\n", 
                  rec.timestamp.c_str(), rec.retries);

    if (sendToServer(rec)) {
      Serial.println("✅ Queued record sent successfully.");
    } else {
      rec.retries++;
      if (rec.retries < MAX_RETRIES) {
        records.push_back(rec);
        Serial.printf("⏳ Record re-queued (retry %d)\n", rec.retries);
        allSuccess = false;
      } else {
        logError("Pending record discarded after " + String(MAX_RETRIES) + " retries.");
        allSuccess = false;
      }
    }
  }

  writePendingFile(records);
  return records.empty();
}

// ================== SERVER TRANSMISSION ==================
bool sendToServer(const DataRecord &rec) {
  bool allOk = true;

  // ---- Weather payload ----
  JsonDocument weatherDoc;
  weatherDoc["station_code"] = stationCode;
  weatherDoc["timestamp"] = rec.timestamp;
  weatherDoc["pressure"] = 1013.2;
  weatherDoc["altitude"] = 1150.0;
  weatherDoc["temperature"] = rec.temperature;
  weatherDoc["humidity"] = rec.humidity;
  weatherDoc["light"] = 0.0;
  weatherDoc["soil_moisture"] = 0.0;
  weatherDoc["rain"] = rec.rainTotal_mm;
  weatherDoc["wind_speed"] = rec.windMS;
  weatherDoc["wind_direction"] = 0;

  String weatherJson;
  serializeJson(weatherDoc, weatherJson);

  // ---- Voltage payload ----
  JsonDocument voltageDoc;
  voltageDoc["station_code"] = stationCode;
  voltageDoc["timestamp"] = rec.timestamp;
  voltageDoc["volt_3v3"] = 0.0;
  voltageDoc["volt_5v"] = 0.0;
  voltageDoc["volt_batt"] = rec.batteryVoltage;
  voltageDoc["volt_solar"] = rec.solarVoltage;
  voltageDoc["volt_dc"] = rec.batteryVoltage;
  voltageDoc["battery_temp"] = rec.batteryTemp;

  String voltageJson;
  serializeJson(voltageDoc, voltageJson);

  // ---- Current payload ----
  JsonDocument currentDoc;
  currentDoc["station_code"] = stationCode;
  currentDoc["timestamp"] = rec.timestamp;
  currentDoc["curr_batt"] = rec.batteryCurrent_mA / 1000.0;
  currentDoc["curr_solar"] = rec.solarCurrent_mA / 1000.0;

  String currentJson;
  serializeJson(currentDoc, currentJson);

  // ---- Send each POST ----
  if (!sendPost(WEATHER_URL, weatherJson)) {
    logError("Weather POST failed");
    allOk = false;
  }
  if (!sendPost(VOLTAGE_URL, voltageJson)) {
    logError("Voltage POST failed");
    allOk = false;
  }
  if (!sendPost(CURRENT_URL, currentJson)) {
    logError("Current POST failed");
    allOk = false;
  }

  return allOk;
}

bool sendPost(const String &url, const String &payload) {
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(10000);   // 10 second timeout
  int httpCode = http.POST(payload);
  http.end();

  if (httpCode == 200 || httpCode == 201) {
    Serial.printf("✅ POST to %s succeeded (HTTP %d)\n", url.c_str(), httpCode);
    return true;
  } else {
    Serial.printf("❌ POST to %s failed (HTTP %d)\n", url.c_str(), httpCode);
    return false;
  }
}