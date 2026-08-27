#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <math.h>

// Raw ADS1115 pin volts -> engineering units.
//
// The station transmits BOTH the raw voltage and the value derived here. That
// is deliberate: the raw volts are ground truth, so if a constant below turns
// out to be wrong, historical rows can be corrected with a single SQL UPDATE
// against the *_v columns. Only future rows need the reflash.
//
// Every function takes volts and returns NAN when the input is missing or the
// reading is outside the sensor's electrical range. NAN propagates to an
// omitted JSON key, which leaves the database column NULL - a failed sensor
// must never be indistinguishable from a real measurement.

// ---------------------------------------------------------------------------
// ADS1115 AIN0 - Wind vane
// ---------------------------------------------------------------------------
// Continuous-output (potentiometer) vane. Two facts define the mapping:
//
//   1. The electrical span. Sensors of this type usually swing 0.5 V - 4.5 V
//      over a full turn rather than 0 - 5 V, so the ends of the range stay
//      distinguishable from a broken wire.
//      >> VERIFY: rotate the vane slowly through 360 degrees and note the
//         lowest and highest wind_direction_v you see. If they are ~0 and ~5,
//         change VANE_V_MIN/MAX to 0.0 and 5.0, otherwise every heading is
//         wrong by a scale factor.
//
//   2. Where north sits. The vane's own electrical zero has nothing to do with
//      how it ended up bolted to the mast, so its reading when physically
//      pointed north is subtracted from every measurement.
#define VANE_V_MIN        0.5f    // volts at the vane's own 0 degrees
#define VANE_V_MAX        4.5f    // volts at a full 360 degrees
#define VANE_V_NORTH      1.169f  // measured 2026-08-27, vane pointed true north

// Slack outside the span before a reading is called a fault rather than a
// direction. Covers the dead band where the wiper crosses the track gap.
#define VANE_V_TOLERANCE  0.25f

static inline float vaneDirectionDeg(float v) {
  if (isnan(v)) return NAN;

  const float span = VANE_V_MAX - VANE_V_MIN;
  if (span <= 0.0f) return NAN;

  // Outside the electrical range means an open circuit, a disconnected sensor
  // or the wrong supply rail - not a bearing.
  if (v < (VANE_V_MIN - VANE_V_TOLERANCE)) return NAN;
  if (v > (VANE_V_MAX + VANE_V_TOLERANCE)) return NAN;

  const float raw   = (v - VANE_V_MIN) / span * 360.0f;
  const float north = (VANE_V_NORTH - VANE_V_MIN) / span * 360.0f;

  // fmodf after adding a full turn keeps the result in 0..360 regardless of
  // which side of north the reading falls.
  float heading = fmodf(raw - north + 360.0f, 360.0f);
  if (heading < 0.0f) heading += 360.0f;
  return heading;
}

// The database column is an integer 0-359.
static inline int vaneDirectionInt(float v) {
  float d = vaneDirectionDeg(v);
  if (isnan(d)) return -1;                 // caller checks for < 0
  int deg = (int)lroundf(d);
  return ((deg % 360) + 360) % 360;        // 360 wraps to 0
}

// ---------------------------------------------------------------------------
// ADS1115 AIN1 - Soil moisture
// ---------------------------------------------------------------------------
// No absolute calibration exists for these probes - output depends on the
// probe, the soil and how tightly it is packed. This is a two-point relative
// scale: percent of the range between bone dry and saturated. Useful for
// trends and irrigation triggers; not a soil-science figure.
//
// Most of these probes read HIGH when dry, hence the inversion.
// Measured 2026-08-27. Provisional - re-measure in the deployment soil at the
// probe's final insertion depth, then re-run the SQL backfill against
// soil_moisture_v to correct every historical row.
#define SOIL_V_DRY   4.927f   // dry soil
#define SOIL_V_WET   2.315f   // saturated soil

static inline float soilMoisturePct(float v) {
  if (isnan(v)) return NAN;
  const float span = SOIL_V_DRY - SOIL_V_WET;
  if (span == 0.0f) return NAN;
  float pct = (SOIL_V_DRY - v) / span * 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

// ---------------------------------------------------------------------------
// ADS1115 AIN2 - Davis 6450 solar radiation
// ---------------------------------------------------------------------------
// Silicon photodiode with a voltage output proportional to irradiance, so the
// conversion is a single multiply.
//
// Linear: the sensor is supplied at 3 V and swings 0 - 3 V, and the datasheet
// multiplier is 600 W/m2 per volt. Full electrical scale is therefore 1800
// W/m2, comfortably above anything the sky produces at ground level.
#define SOLAR_WM2_PER_V     600.0f

// Ground-level irradiance peaks near 1000-1100 W/m2, and briefly higher at a
// cloud edge. Anything past this is a fault - a short to the rail reads 3 V,
// i.e. 1800 W/m2 - so report nothing rather than a number that looks real.
#define IRRADIANCE_MAX_WM2  1500.0f

static inline float irradianceWm2(float v) {
  if (isnan(v)) return NAN;
  float w = v * SOLAR_WM2_PER_V;
  if (w < 0.0f) return 0.0f;                  // small negative offset at night
  if (w > IRRADIANCE_MAX_WM2) return NAN;     // fault, not sunlight
  return w;
}

#endif
