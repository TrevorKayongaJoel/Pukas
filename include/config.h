#ifndef CONFIG_H
#define CONFIG_H

// ========== Libraries ==========
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <DHT.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <vector>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <time.h>

// ========== WiFi Credentials (FALLBACK - used only if no saved credentials) ==========
#define WIFI_SSID_FALLBACK     "IoT-ra"
#define WIFI_PASSWORD_FALLBACK "P9$y#F5x!b&"
#define PORTAL_TIMEOUT         120     // seconds the config portal stays open

// ========== Pin Definitions ==========
#define RELAY_PIN       13
#define RELAY_ACTIVE    HIGH
#define I2C_SDA         21
#define I2C_SCL         22
#define WIND_PIN        27
#define RAIN_PIN        26
#define DHTPIN          25
#define DS18B20_PIN     32

#define SD_CS           5
#define SD_MOSI         23
#define SD_MISO         19
#define SD_SCK          18

// ========== Sensor Constants ==========
#define DHTTYPE         DHT22
#define MM_PER_TIP      0.2
#define DEBOUNCE_US     100000      // 100 ms
#define WIND_INTERVAL   0.5         // seconds

// ========== Time / RTC ==========
// The DS3231 holds LOCAL time: East Africa Time (UTC+3, no DST).
// SD logs and API payloads therefore all carry EAT.
#define TZ_OFFSET_SEC          (3 * 3600)
#define NTP_SERVER_1           "pool.ntp.org"
#define NTP_SERVER_2           "time.google.com"
#define NTP_SERVER_3           "time.nist.gov"
#define NTP_SYNC_TIMEOUT_MS    10000UL
#define NTP_RESYNC_INTERVAL_MS (24UL * 60UL * 60UL * 1000UL)   // re-sync daily
#define NTP_RETRY_INTERVAL_MS  (30UL * 60UL * 1000UL)          // floor between failed attempts
#define RTC_MIN_VALID_YEAR     2025   // older than this means the clock was never set
#define TIMESTAMP_INVALID      "1970-01-01 00:00:00"

// ========== Config Limits ==========
#define SLEEP_MIN_SEC          60
#define SLEEP_MAX_SEC          3600
#define STATION_CODE_MAXLEN    20

// ========== Server Endpoints ==========
#define BASE_URL        "https://api.wimea-ict.net/api/ingest/"
#define WEATHER_URL     BASE_URL "weather/"
#define VOLTAGE_URL     BASE_URL "voltage/"
#define CURRENT_URL     BASE_URL "current/"

// ========== SD Log Files ==========
#define SD_DATA_FILE    "/datalog.csv"
#define SD_ERROR_FILE   "/error.log"
#define SD_PENDING_FILE "/pending.csv"

// ========== Queue Settings ==========
#define MAX_RETRIES         5
#define MAX_PENDING_CYCLE   10

// ========== Global Objects ==========
extern Adafruit_ADS1115      ads;
extern Adafruit_INA219      batterySensor;
extern Adafruit_INA219      solarSensor;
extern RTC_DS3231           rtc;
extern DHT                  dht;
extern Preferences          preferences;
extern OneWire              oneWire;
extern DallasTemperature    ds18b20;

extern int sleepIntervalSec;
extern String stationCode;
extern bool relayOn;
extern bool ds18b20Ok;
extern bool rtcOk;          // DS3231 responded on the I2C bus
extern bool rtcTimeValid;   // ...and the time it holds is trustworthy

// ========== Data Record ==========
struct DataRecord {
  String timestamp;
  float windMPH;
  float windMS;
  float rainTotal_mm;
  float temperature;        // Air temp (DHT22)
  float humidity;           // Air humidity (DHT22)
  float batteryTemp;        // Battery temperature (DS18B20)
  float solarVoltage;
  float solarCurrent_mA;
  float batteryVoltage;
  float batteryCurrent_mA;
  float adc0, adc1, adc2, adc3;
  uint8_t retries;
};

// ========== Function Prototypes ==========
void loadConfig();
void saveConfig(int newInterval, const String &newStation);
void readSensors(DataRecord &rec);
void logToSD(const DataRecord &rec);
bool sendToServer(const DataRecord &rec);
bool sendPost(const String &url, const String &payload);
void appendSD(const String &line, const char *filename);
void logError(const String &message);
String getTimestamp();
void runConfigPortal();
void printSensorValues(const DataRecord &rec);   // <-- ADDED
bool connectWiFi();                              // <-- ADDED

// ---- Time / RTC ----
bool syncTimeFromNTP();                      // fetch NTP time and store it in the DS3231
void maybeResyncRTC();                       // sync on first chance, then once a day
bool refreshRtcValidity();                   // re-check whether the RTC time is trustworthy
bool setRtcFromString(const String &input);  // manual set, "YYYY-MM-DD HH:MM:SS" (EAT)

// ---- Queue functions ----
void enqueueRecord(const DataRecord &rec);
bool processPendingQueue();
void writePendingFile(const std::vector<DataRecord> &records);
bool parseLineToRecord(const String &line, DataRecord &rec);
String recordToLine(const DataRecord &rec, bool includeRetries);

#endif