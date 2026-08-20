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
#include <WebServer.h>
#include <DNSServer.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

// ========== WiFi Credentials (FALLBACK - used only if no saved credentials) ==========
#define WIFI_SSID_FALLBACK     "IoT-ra"
#define WIFI_PASSWORD_FALLBACK "P9$y#F5x!b&"
#define PORTAL_TIMEOUT         120     // seconds the config portal stays open
#define PORTAL_AP_SSID         "AWS-Config"   // same SSID for our portal and WiFiManager

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

// ========== Pulse Counting ==========
#define WIND_DEBOUNCE_US   5000UL      // 5 ms; at 30 Hz max the period is 33 ms
#define GUST_WINDOW_US     3000000UL   // 3 s - the standard gust averaging window
#define WIND_MPH_PER_HZ    1.5f
#define WIND_MS_PER_HZ     0.67f
#define REED_RELEASE_MS    50          // bounded wait for a reed contact to reopen

// ========== Light Sleep ==========
#define SLEEP_CHUNK_SEC    60          // wake at least this often to re-check the RTC

// ========== Time / RTC ==========
// The DS3231 holds LOCAL time: East Africa Time (UTC+3, no DST).
// SD logs and API payloads therefore all carry EAT.
#define TZ_OFFSET_SEC          (3 * 3600)
#define NTP_SERVER_1           "pool.ntp.org"
#define NTP_SERVER_2           "time.google.com"
#define NTP_SERVER_3           "time.nist.gov"
#define NTP_SYNC_TIMEOUT_MS    20000UL   // 10 s was tight when DNS is slow
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

// ========== SD Files ==========
// datalog.txt is the permanent archive - append only, never rewritten.
// queue.txt is the upload buffer - records leave it only once delivered.
// rejected.txt holds what the server permanently refused, so one bad record
// can never block the queue behind it.
#define SD_DATA_FILE     "/datalog.txt"
#define SD_QUEUE_FILE    "/queue.txt"
#define SD_QUEUE_TMP     "/queue.tmp"
#define SD_REJECTED_FILE "/rejected.txt"
#define SD_ERROR_FILE    "/error.log"

#define SD_CSV_HEADER "Timestamp,WindMPH,WindMS,WindGust_MS,Rain_mm,Temp_C,Humidity%,BatteryTemp_C,Solar_V,Solar_mA,Batt_V,Batt_mA,ADC0,ADC1,ADC2,ADC3"

// Field counts for the CSV record format. Queue lines carry two extra columns
// (delivery mask + retry count) that the archive does not need.
#define REC_FIELDS_BASE   16
#define REC_FIELDS_QUEUE  18

// Bits in DataRecord::sentMask - one per ingest endpoint.
#define SENT_WEATHER  0x01
#define SENT_VOLTAGE  0x02
#define SENT_CURRENT  0x04
#define SENT_ALL      0x07

// The station has no barometer, light, soil or wind-vane sensor. Posting
// invented numbers for them pollutes the dataset, so those keys are omitted.
// Set to 1 if the ingest API rejects payloads that lack them.
#define SEND_PLACEHOLDER_FIELDS 0

// How many error lines to hold in RAM when the SD rail is powered down.
#define ERROR_BUFFER_SLOTS 8

// The SD card shares the sensor rail on this build, so the rail has to stay up
// while the upload queue is being drained. Set to 1 if SD has its own supply -
// the sensor rail then drops as soon as the readings are taken, as the pipeline
// describes.
#define SD_INDEPENDENT_POWER 0

// Time for the sensor rail, the SD card and the I2C devices to come up after
// the relay closes.
#define SENSOR_RAIL_SETTLE_MS  500

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
extern bool haveLastRecord;     // a reading has been taken since boot
extern uint32_t nextCycleEpoch; // RTC epoch of the next scheduled burst
extern int queueDepthCached;         // records waiting, tracked without touching SD
extern unsigned long nextCycleMillis; // millis() deadline for the next burst

// ========== Data Record ==========
struct DataRecord {
  String timestamp;
  float windMPH;
  float windMS;
  float windGustMS;         // highest 3 s average within the interval
  float rainTotal_mm;
  float temperature;        // Air temp (DHT22)
  float humidity;           // Air humidity (DHT22)
  float batteryTemp;        // Battery temperature (DS18B20)
  float solarVoltage;
  float solarCurrent_mA;
  float batteryVoltage;
  float batteryCurrent_mA;
  float adc0, adc1, adc2, adc3;
  uint8_t sentMask;         // which ingest endpoints already accepted this record
  uint8_t retries;
};


// Most recent reading, kept in RAM so the config portal can display it.
extern DataRecord lastRecord;

// ========== Function Prototypes ==========
void loadConfig();
void saveConfig(int newInterval, const String &newStation);
void readSensors(DataRecord &rec);
bool appendSD(const String &line, const char *filename);   // false if the write failed
void logError(const String &message);
String getTimestamp();
void runConfigPortal();
void printSensorValues(const DataRecord &rec);   // <-- ADDED
bool connectWiFi();                              // <-- ADDED

// ---- Config portal ----
int  queuedRecordCount();          // lines currently waiting in the upload queue
void setSensorPower(bool on);      // relay that gates the sensor rail
void sensorRailUp();               // power up AND re-initialise everything on it
void sensorRailDown();             // unmount SD cleanly, then cut power
bool mountSD();                    // (re)run the card init sequence
void unmountSD();                  // flush and release before power is removed

// ---- Pulse counting & light sleep ----
uint32_t currentEpoch();               // RTC epoch, or 0 when the clock is untrustworthy
void snapshotPulseCounters();          // close the interval: reset counters, compute results
void sleepUntilNextEventOrDeadline();  // light sleep until a pulse or the next burst

// ---- Time / RTC ----
bool syncTimeFromNTP();                      // fetch NTP time and store it in the DS3231
void maybeResyncRTC();                       // sync on first chance, then once a day
bool refreshRtcValidity();                   // re-check whether the RTC time is trustworthy
bool setRtcFromString(const String &input);  // manual set, "YYYY-MM-DD HH:MM:SS" (EAT)

// ---- Queue functions ----
void enqueueRecord(const DataRecord &rec);
bool processPendingQueue();
bool parseLineToRecord(const String &line, DataRecord &rec);
String recordToLine(const DataRecord &rec, bool includeQueueFields = false);
void archiveRecord(const DataRecord &rec);   // append to the permanent datalog
void flushBufferedErrors();                  // write RAM-held errors once SD is back

#endif
