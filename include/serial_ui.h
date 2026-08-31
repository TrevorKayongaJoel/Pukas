#ifndef SERIAL_UI_H
#define SERIAL_UI_H

#include <Arduino.h>
#include <math.h>
#include <string.h>

// Serial output helpers. Everything here is pure ASCII on purpose.
//
// Emoji and box-drawing glyphs are multi-byte UTF-8, and most emoji render
// double-width, so any table built from them has a ragged right edge that no
// amount of padding can fix. They also mangle in non-UTF-8 terminals and in
// tooling that touches the source. Single-width ASCII lines up exactly and
// survives being pasted anywhere.
//
// The tags are fixed-width and greppable: `grep FAIL` on a captured log is
// worth more than a prettier glyph.

// ---------------------------------------------------------------------------
// Colour
// ---------------------------------------------------------------------------
// ANSI escapes are ASCII, so they carry none of the problems above. But they
// are not universally understood:
//   PlatformIO monitor  renders them properly
//   Arduino IDE monitor prints the escape codes literally
//   piping to a file    embeds them, making the log harder to grep
// Set to 0 in either of the latter cases.
#define SERIAL_COLOR 1

#if SERIAL_COLOR
  #define C_RESET   "\033[0m"
  #define C_DIM     "\033[2m"
  #define C_BOLD    "\033[1m"
  #define C_RED     "\033[31m"
  #define C_GREEN   "\033[32m"
  #define C_YELLOW  "\033[33m"
  #define C_BLUE    "\033[34m"
  #define C_CYAN    "\033[36m"
#else
  #define C_RESET   ""
  #define C_DIM     ""
  #define C_BOLD    ""
  #define C_RED     ""
  #define C_GREEN   ""
  #define C_YELLOW  ""
  #define C_BLUE    ""
  #define C_CYAN    ""
#endif

// Six characters wide, every one of them, so message text starts in the same
// column whatever the severity.
#define TAG_OK    C_GREEN  "[ OK ]" C_RESET
#define TAG_FAIL  C_RED    "[FAIL]" C_RESET
#define TAG_WARN  C_YELLOW "[WARN]" C_RESET
#define TAG_INFO  C_CYAN   "[INFO]" C_RESET
#define TAG_SEND  C_BLUE   "[ >> ]" C_RESET
#define TAG_STEP  C_DIM    "[ .. ]" C_RESET

// printf-style loggers. Note the tag is a string literal, so it concatenates
// with the format at compile time and costs nothing at runtime.
#define logOk(fmt, ...)   Serial.printf(TAG_OK   " " fmt "\n", ##__VA_ARGS__)
#define logFail(fmt, ...) Serial.printf(TAG_FAIL " " fmt "\n", ##__VA_ARGS__)
#define logWarn(fmt, ...) Serial.printf(TAG_WARN " " fmt "\n", ##__VA_ARGS__)
#define logInfo(fmt, ...) Serial.printf(TAG_INFO " " fmt "\n", ##__VA_ARGS__)
#define logSend(fmt, ...) Serial.printf(TAG_SEND " " fmt "\n", ##__VA_ARGS__)
#define logStep(fmt, ...) Serial.printf(TAG_STEP " " fmt "\n", ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// Readings table
// ---------------------------------------------------------------------------
// Fixed geometry: "| " + label(13) + " " + value(30) + " |" = 48 columns.
#define UI_LABEL_W 13
#define UI_VALUE_W 30
#define UI_RULE "+----------------------------------------------+"

static inline void uiRule() {
  Serial.println(C_DIM UI_RULE C_RESET);
}

static inline void uiTitle(const char *title) {
  uiRule();
  Serial.printf(C_DIM "|" C_RESET C_BOLD "%*s%s%*s" C_RESET C_DIM "|" C_RESET "\n",
                (46 - (int)strlen(title)) / 2, "",
                title,
                46 - (int)strlen(title) - (46 - (int)strlen(title)) / 2, "");
  uiRule();
}

static inline void uiRow(const char *label, const char *value) {
  Serial.printf(C_DIM "|" C_RESET " %-*s %-*s " C_DIM "|" C_RESET "\n",
                UI_LABEL_W, label, UI_VALUE_W, value);
}

// Formats a reading, or "--" when the sensor had nothing to report. Keeping
// NAN visibly distinct from a real zero is the whole point of the NAN handling
// further up the pipeline; it would be a shame to print it as "0.00" here.
static inline void uiRowNum(const char *label, float v, int dp,
                            const char *unit, const char *note = "") {
  char value[UI_VALUE_W + 1];
  if (isnan(v)) {
    snprintf(value, sizeof(value), "%8s  %-6s%s", "--", unit, note);
  } else {
    snprintf(value, sizeof(value), "%8.*f  %-6s%s", dp, v, unit, note);
  }
  uiRow(label, value);
}

static inline void uiRowInt(const char *label, long v, const char *unit) {
  char value[UI_VALUE_W + 1];
  snprintf(value, sizeof(value), "%8ld  %s", v, unit);
  uiRow(label, value);
}

#endif
