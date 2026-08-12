#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_INA219.h>
#include <RTClib.h>
#include <DHT.h>                   // <-- NEW: DHT library

// ---- Pin definitions ----
#define RELAY_PIN    13
#define RELAY_ACTIVE HIGH
#define I2C_SDA      21 
#define I2C_SCL      22
#define WIND_PIN     27    // Davis 6410 anemometer (active-low pulse)
#define RAIN_PIN     26    // Davis rain gauge (active-low pulse)
#define DHTPIN       25    // <-- NEW: DHT22 data pin

// ---- DHT Type ----
#define DHTTYPE DHT22       // <-- NEW: Specify DHT22

// ---- Global Objects ----
Adafruit_ADS1115     ads;
Adafruit_INA219      batterySensor(0x40);
Adafruit_INA219      solarSensor(0x41);
RTC_DS3231           rtc;
DHT dht(DHTPIN, DHTTYPE);   // <-- NEW: DHT object

// ---- Relay state ----
bool relayOn = false;

// ---- Wind sensor variables ----
volatile unsigned int windPulseCount = 0;
unsigned long lastWindReadTime = 0;
float windSpeed_MPH = 0.0;
float windSpeed_MS  = 0.0;

// ---- Rain gauge variables (with DEBOUNCE) ----
volatile unsigned int rainPulseCount = 0;
volatile unsigned long lastRainTriggerTime = 0;
float totalRain_mm = 0.0;
const float MM_PER_TIP = 0.2;
const unsigned long DEBOUNCE_US = 100000;  // 100 ms

// ---- DHT22 variables (for error handling) ----
float temperature = -999.0;
float humidity    = -999.0;

// ---- Interrupt Service Routine for Wind ----
void IRAM_ATTR windISR() {
  windPulseCount++;
}

// ---- Interrupt Service Routine for Rain (with Debounce) ----
void IRAM_ATTR rainISR() {
  unsigned long now = micros();
  if ((now - lastRainTriggerTime) > DEBOUNCE_US) {
    rainPulseCount++;
    lastRainTriggerTime = now;
  }
}

// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 Solar/Battery/Wind/Rain/METEO Monitor + RTC ===");
  Serial.println("Relay ON – readings every 2 seconds.");
  Serial.println("Type '0' to turn relay OFF, '1' to turn ON.\n");

  // ---- Relay (powers the I2C sensors) ----
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ACTIVE);
  relayOn = true;
  Serial.println("🔌 Relay ON – sensors powered.");
  delay(500);

  // ---- Wind Sensor ----
  pinMode(WIND_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), windISR, FALLING);
  lastWindReadTime = millis();
  Serial.println("✅ Wind sensor ready on GPIO 27");

  // ---- Rain Gauge ----
  pinMode(RAIN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainISR, FALLING);
  Serial.println("✅ Rain gauge ready on GPIO 26 (debounce = 100ms)");

  // ---- DHT22 ----
  dht.begin();
  Serial.println("✅ DHT22 ready on GPIO 25");

  // ---- I2C Bus ----
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("✅ I2C ready");

  // ---- ADS1115 ----
  if (ads.begin(0x48)) {
    ads.setGain(GAIN_TWOTHIRDS);
    Serial.println("✅ ADS1115 (0x48)");
  } else {
    Serial.println("❌ ADS1115 – check wiring");
  }

  // ---- INA219 Battery ----
  if (batterySensor.begin()) {
    Serial.println("✅ INA219 Battery (0x40)");
  } else {
    Serial.println("❌ INA219 Battery – check wiring/power");
  }

  // ---- INA219 Solar ----
  if (solarSensor.begin()) {
    Serial.println("✅ INA219 Solar (0x41)");
  } else {
    Serial.println("❌ INA219 Solar – check wiring/power");
  }

  // ---- RTC DS3231 ----
  if (rtc.begin()) {
    Serial.println("✅ RTC DS3231 (0x68)");
    // Uncomment once to set RTC to compile time:
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else {
    Serial.println("❌ RTC – check wiring/power");
  }

  Serial.println("\n--- Starting readings ---\n");
}

// ------------------------------------------------------------------
void loop() {
  // ---- Manual Serial Override (Relay ON/OFF) ----
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    if (c == '0') {
      digitalWrite(RELAY_PIN, !RELAY_ACTIVE);
      relayOn = false;
      Serial.println("🔌 Relay OFF – sensors powered down.");
    }
    if (c == '1') {
      digitalWrite(RELAY_PIN, RELAY_ACTIVE);
      relayOn = true;
      Serial.println("🔌 Relay ON – sensors powered up.");
      delay(500);
    }
  }

  // ---- Only read I2C sensors if relay is ON ----
  if (!relayOn) {
    delay(2000);
    return;
  }

  // ---- 1. READ WIND SPEED ----
  unsigned long currentTime = millis();
  float intervalSec = (currentTime - lastWindReadTime) / 1000.0;

  if (intervalSec >= 0.5) {
    noInterrupts();
    unsigned int pulseCount = windPulseCount;
    windPulseCount = 0;
    interrupts();

    float freq = pulseCount / intervalSec;
    windSpeed_MPH = freq * 1.5;
    windSpeed_MS  = freq * 0.67;
    lastWindReadTime = currentTime;
  }

  // ---- 2. READ RAINFALL ----
  noInterrupts();
  unsigned int rainTips = rainPulseCount;
  rainPulseCount = 0;
  interrupts();

  totalRain_mm += rainTips * MM_PER_TIP;

  // ---- 3. READ DHT22 (Temperature & Humidity) ----
  // DHT22 can occasionally return NaN (checksum error). 
  // We keep the previous valid value if it fails.
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temperature = t;
  if (!isnan(h)) humidity = h;

  // ---- 4. READ ADS1115 ----
  float adc[4];
  for (int i = 0; i < 4; i++) {
    int16_t raw = ads.readADC_SingleEnded(i);
    adc[i] = ads.computeVolts(raw);
  }

  // ---- 5. READ INA219 Solar ----
  float sV = solarSensor.getBusVoltage_V();
  float sA = solarSensor.getCurrent_mA();
  float sW = sV * sA / 1000.0;

  // ---- 6. READ INA219 Battery ----
  float bV = batterySensor.getBusVoltage_V();
  float bA = batterySensor.getCurrent_mA();
  float bW = bV * bA / 1000.0;

  // ---- 7. READ RTC ----
  char ts[20] = "NO_RTC";
  DateTime now = rtc.now();
  sprintf(ts, "%04d-%02d-%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());

  // ---- 8. PRINT ALL RESULTS ----
  Serial.println("========================================");
  Serial.printf("📅 %s\n", ts);
  Serial.printf("🌬️ Wind:  %5.2f MPH   (%5.2f m/s)\n", windSpeed_MPH, windSpeed_MS);
  Serial.printf("🌧️ Rain:  %4d tips   %6.1f mm total\n", rainTips, totalRain_mm);
  Serial.printf("🌡️ Temp:  %6.2f °C   %5.1f %% RH\n", temperature, humidity);  // <-- NEW
  Serial.printf("☀️ Solar:  %6.3f V  %7.2f mA  %6.3f W\n", sV, sA, sW);
  Serial.printf("🔋 Battery: %6.3f V  %7.2f mA  %6.3f W\n", bV, bA, bW);
  Serial.printf("📊 A0: %6.4f V   A1: %6.4f V\n", adc[0], adc[1]);
  Serial.printf("📊 A2: %6.4f V   A3: %6.4f V\n", adc[2], adc[3]);
  Serial.println("========================================\n");

  delay(2000);
}