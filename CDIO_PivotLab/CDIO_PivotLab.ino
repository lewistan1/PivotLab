#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Font4x5Fixed.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include "frames/frames.h"
#include <Preferences.h>

// =====================================================
// Constants / Hardware
// =====================================================

// ---- General ----
constexpr int MAX_SERVO_POS = 10;
constexpr int SERVO_UNUSED = -1;

// ---- OLED ----
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET = -1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---- Pins ----
constexpr int ENCODER_CLK = 25;
constexpr int ENCODER_DT = 33;
constexpr int ENCODER_SW = 32;
constexpr int SERVO_PIN = 4;
const int AS5600_OUT_PIN = 34;

constexpr int SERVO_CH = 0;  // (kept for your structure)

// ---- AS5600 ----
constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t AS5600_RAW_ANGLE_H = 0x0C;
constexpr uint8_t AS5600_RAW_ANGLE_L = 0x0D;

// ---- Trigger (education detect window) ----

constexpr unsigned long STABLE_MS = 150;

bool invertServoDir = false;     // flips servo motion direction (mechanism)
bool invertFeedbackErr = false;  // flips feedback correction sign (if it runs away)

// Backlash take-up when direction changes (during the initial move-to-command)
int OVERSHOOT_DEG = 3;           // overshoot amount
float MOVE_DEADBAND_DEG = 0.6f;  // AS5600 must change at least this much to count as "moved"
int MAX_TAKEUP_STEPS = 10;       // max 1-deg nudges to take up slack

// Holding control (closed loop)
float HOLD_TOL_IN = 1.0f;   // enter hold when within ±1.0°
float HOLD_TOL_OUT = 1.8f;  // only correct again if error > ±2.5° (hysteresis)
int HOLD_STEP_NEAR = 1;     // servo degrees per correction
int HOLD_STEP_FAR = 1;      // bigger correction if far


const uint32_t HOLD_INTERVAL = 40;  // ms between hold corrections

// ---------- State ----------
bool active = false;           // stays false after init until user types a servo angle
float targetSensorDeg = 0.0f;  // AS5600 angle we want to hold
bool holding = false;

// =====================================================
// Globals / State
// =====================================================
Preferences prefs;

int servoPositions[MAX_SERVO_POS];
int servoPosCount = MAX_SERVO_POS;

int settingsScroll = 0;
int settingSelect = 0;

int editIdx = -1;
int editOriginal = SERVO_UNUSED;
int editTemp = SERVO_UNUSED;
bool editDirty = false;
int nanFrom = 0;  // 0=none, 1=crossed below 0, 2=crossed above 180

int servoPos = 90;
int SERVO_MIN = 0;
int SERVO_MAX = 180;

// ---------- Balance settings (Blank mode) ----------
float TARGET_DEG = 43.0f;  // editable (saved in NVS)
bool editingCenter = false;
float centerTemp = 41.0f;
float centerOriginal = 41.0f;
bool centerDirty = false;


constexpr int TRIGGER_TOL = 9;
float assist = 19.0f;
float TOL = 4.0f;
float TOL1 = 7.0f;
int SERVO_STEP = 1;

// Base update interval (ms)
unsigned long SETTLE_MS = 60;

// Zones
float SLOW_ZONE_DEG = 30.0f;        // within ±25 => slow down
float SUPER_SLOW_DEG = 10.0f;       // within ±10 => super slow
unsigned long SUPER_SLOW_MS = 600;  // NOTE: 600ms = 0.6s (set 400 if you want 0.4s)
int SLOW_MULT = 5;

// Direction-change pause
unsigned long DIR_PAUSE_MS = 700;
unsigned long rampUntilMs = 0;
const unsigned long RAMP_MS = 800;  // how long to ramp AFTER direction flip pause
const int RAMP_MULT = 6;            // start-of-ramp slowdown multiplier


// ---------- Timing / direction state (Blank mode) ----------
unsigned long lastMoveMs = 0;
unsigned long pauseUntilMs = 0;
int lastMoveDir = 0;  // -1,0,+1
bool invertDirection = false;


volatile bool btnEvent = false;
volatile uint32_t btnIsrMs = 0;


// ---------- UI / Mode flags ----------
bool irArmed = false;  // AS5600 trigger only works after press
bool editingServoAngle = false;

int servoCenterAngle = 90;

ESP32Encoder myEnc;
Servo myServo;

int currentAngle = 90;
int lastAngle = 90;
long lastEncPos = 0;

int menuIndex = 1;     // 0=INFO, 1=NORMAL, 2=INFINITE, 3=EDU, 4=EXPERIMENTAL(BLANK), 5=SETTINGS
int subMenuIndex = 0;  // 0=BACK, 1=main area
int diffIndex = 1;     // 1=EASY,2=NORMAL,3=HARD
int blankSelect = 0;

bool inMainMenu = true;
bool inDiffMenu = false;
bool inEduMode = false;
bool detecting = false;
bool showResult = false;
bool inSettingsMenu = false;
bool inBlankMode = false;
bool indisplaySystem = false;

// ---- Info page ----
bool inInfoPage = false;
int infoScroll = 0;
int infoScrollMax = 0;

// ---- Game timing ----
int moveCount = 0;
bool irDetected = false;
unsigned long detectStart = 0;
bool servoAllowed = true;
unsigned long startTime = 0;
unsigned long totalTime = 0;
unsigned long pausedTime = 0;
unsigned long irPauseStart = 0;
bool irPaused = false;
unsigned long armStart = 0;


constexpr float ADC_MAX = 4095.0f;

// Optional calibration (tune if needed)
float adcMin = 0.0f;
float adcMax = 4095.0f;

// Filtering
// EMA alpha: smaller = smoother (more lag), larger = faster response
constexpr float EMA_ALPHA = 0.10f;  // try 0.05..0.20

// Deadband (degrees): ignore tiny changes
constexpr float DEADBAND_DEG = 0.15f;  // set 0 to disable

// Trimmed-mean sampling
constexpr int NSAMPLES = 31;  // odd number recommended
constexpr int TRIM = 3;       // drop lowest 3 and highest 3

static float emaDeg = NAN;
static float lastOutDeg = NAN;

const float ASSIST_ENTER = 19.0f;
const float ASSIST_EXIT = 25.0f;

static bool inAssistMode = false;
static uint32_t modeChangeAt = 0;
const uint32_t MODE_DEBOUNCE_MS = 250;  // must stay past threshold for 250ms



// ---- Infinite mode ----
unsigned long lastMoveTime = 0;
unsigned long baseMaxDelay = 30000;
unsigned long baseMinDelay = 15000;
unsigned long maxDelay = baseMaxDelay;
unsigned long minDelay = baseMinDelay;
float difficultyFactor = 0.95f;
unsigned long irDetectDelay = 2000;  // (kept)

const int AS_MIN = 0;
const int AS_MAX = 359;

float eduOptions[4];

// ---- Education mode ----
int eduSelected = 0;
int eduAnswer = 0;
int servoUsage[7] = { 0, 0, 0, 0, 0, 0, 0 };
int servoUsageGame[MAX_SERVO_POS] = { 0 };
int lastGameIndex = -1;

constexpr int EDU_POS_COUNT = 6;
int eduAngles[EDU_POS_COUNT] = { 307, 266, 236, 212, 193, 176};
int eduUsage[EDU_POS_COUNT] = { 0 };
int eduLastIndex = -1;
// For word-based questions
bool eduIsWordQuestion = false;
bool eduIsTrueFalse = false;
String eduOptionsText[4];
String eduCorrectWord = "";
int eduOptionCount = 4;  // default

enum EduQuestionType {
  Q_find_force,
  Q_find_moment,
  Q_total_weight,
  Q_add_both,
  Q_remove
};

inline bool isGameplayActive() {
  return detecting
         && !inMainMenu
         && !inDiffMenu
         && !inInfoPage
         && !inSettingsMenu
         && !inBlankMode
         && !showResult;
}

float answerTable_force[EDU_POS_COUNT] = { 0.0, 9.8, 19.6, 29.4, 39.2, 49.0 };
float answerTable_moment[EDU_POS_COUNT] = { 0.0, 161.7, 294.0, 367.5, 431.2, 490.0 };
float answerTable_total_weight[EDU_POS_COUNT] = { 0, 1, 2, 3, 4, 5 };
float answerTable_add_both[EDU_POS_COUNT] = { 0, 0, 1, 0, 0, 0 };
float answerTable_remove[EDU_POS_COUNT] = { 2, 1, 1, 1, 1, 1 };

EduQuestionType currentQuestion;

int detectforedu = 0;
bool irMustReset = false;
unsigned long irDetectStart = 0;
bool irDetecting = false;

// =====================================================
// Bitmaps
// =====================================================

const uint8_t arrow[] PROGMEM = {
  0xff, 0xf0, 0xff, 0xf0, 0xfb, 0xf0, 0xf3, 0xf0, 0xc1, 0xf0, 0x00, 0x30,
  0x80, 0x10, 0xc0, 0x10, 0xf3, 0xc0, 0xfb, 0xe0, 0xff, 0xf0, 0xff, 0xf0
};
const uint8_t infos[] PROGMEM = {
  0xf8, 0x1f, 0xe7, 0xe7, 0xcf, 0xf3, 0x9e, 0x79, 0xbe, 0x7d, 0x7f, 0xfe, 0x7e, 0x7e, 0x7e, 0x7e,
  0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0xbe, 0x7d, 0x9e, 0x79, 0xcf, 0xf3, 0xe7, 0xe7, 0xf8, 0x1f
};
const uint8_t info[] PROGMEM = {
  0xf8, 0x1f, 0xe0, 0x07, 0xc0, 0x03, 0x81, 0x81, 0x81, 0x81, 0x00, 0x00, 0x01, 0x80, 0x01, 0x80,
  0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x81, 0x81, 0x81, 0x81, 0xc0, 0x03, 0xe0, 0x07, 0xf8, 0x1f
};
const uint8_t settingsIcon[] PROGMEM = {
  0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xf6, 0x6f, 0xe0, 0x07, 0xf3, 0xcf, 0xf7, 0xef, 0xc7, 0xe3,
  0xc7, 0xe3, 0xf7, 0xef, 0xf3, 0xcf, 0xe0, 0x07, 0xf6, 0x6f, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff
};
const uint8_t settingselected[] PROGMEM = {
  0xf8, 0x1f, 0xe0, 0x07, 0xc0, 0x03, 0x81, 0x81, 0x85, 0xa1, 0x0f, 0xf0, 0x07, 0xe0, 0x1e, 0x78,
  0x1e, 0x78, 0x07, 0xe0, 0x0f, 0xf0, 0x85, 0xa1, 0x81, 0x81, 0xc0, 0x03, 0xe0, 0x07, 0xf8, 0x1f
};
const uint8_t arrows[] PROGMEM = {
  0xff, 0xff, 0x80, 0xfc, 0xff, 0x80, 0xf8, 0x7f, 0x80, 0xf0, 0x7f, 0x80, 0xe0, 0x7f, 0x80, 0xc0,
  0x0f, 0x80, 0x80, 0x07, 0x80, 0x00, 0x03, 0x80, 0x80, 0x01, 0x80, 0xc0, 0x00, 0x80, 0xe0, 0x00,
  0x80, 0xf0, 0x78, 0x00, 0xf8, 0x7e, 0x00, 0xfc, 0x7f, 0x00, 0xff, 0xff, 0x80, 0xff, 0xff, 0x80
};

// =====================================================
// Forward declarations (so Arduino compiles cleanly)
// =====================================================
void playStartupGif();
void displayMainMenu();
void displaySystem();
void displayResult();
void displayInfoPage();
void displayDiffMenu();
void displaySettingsMenu();
void displayBlankMode();

void startNormalMode();
void startInfiniteMode();
void applyDifficulty(int level);

void startEducationMode();
void displayEducationPrompt();
void displayBalanceMessage();
void displayEducationFeedback();
void generateEducationOptions();
float getCorrectAnswerDynamic();

void handleEncoder();

void resetEncoderReference();

void moveServoRandom();
void moveServoRandomEducation();
int getSmartRandomGameIndex();
int getSmartRandomEduIndex();
void smoothMove(int targetAngle);
void fastMove(int ang);

void saveServoPositions();
void loadServoPositions();

uint16_t readAS5600Raw();
float readAS5600Deg1();
bool inWindowDeg(int a, int target, int tol);
void handleAS5600Trigger();

int getCenteredX(String text, int textSize);
int getCenteredX_Font5x7(String text);

void drawBackArrow(bool selected);
void drawSettingsIcon(bool selected);
void drawinfo(bool selected);

float wrap360(float x);
float signedDiff(float target, float current);
bool readAS5600Deg2(float& degOut);



// ==== CENTERING HELPER ====
int getCenteredX(String text, int textSize) {
  int width = text.length() * 6 * textSize;  // 6 = default font char width
  return (SCREEN_WIDTH - width) / 2;
}

void IRAM_ATTR onBtnFalling() {
  uint32_t now = millis();
  if (now - btnIsrMs > 20) {  // debounce in ISR
    btnEvent = true;
    btnIsrMs = now;
  }
}

// ==== SETUP ====
void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(ENCODER_CLK, INPUT_PULLUP);  // Pullups for encoder pins
  pinMode(ENCODER_DT, INPUT_PULLUP);
  myEnc.attachFullQuad(ENCODER_CLK, ENCODER_DT);
  myEnc.clearCount();  // reset position

  analogReadResolution(12);
  analogSetPinAttenuation(AS5600_OUT_PIN, ADC_11db);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW), onBtnFalling, FALLING);



  // Servo
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(90);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
    for (;;)
      ;
  }

  playStartupGif();
  displayMainMenu();
  prefs.begin("settings", false);  // namespace = "settings"
  servoCenterAngle = prefs.getInt("servoCenter", 90);
  prefs.end();

  // apply ito
  loadServoPositions();

  myServo.write(servoCenterAngle);
  lastAngle = servoCenterAngle;
}

// ==== LOOP ====
void loop() {
  handleEncoder();
  handleButtonInterrupt();


  // ✅ HOLD runs globally (including SETTINGS)
  static uint32_t lastHold = 0;
  if (active && !inBlankMode) {
    uint32_t nowHold = millis();
    if (nowHold - lastHold >= HOLD_INTERVAL) {
      lastHold = nowHold;
      holdLoopStep();

      // optional serial proof when in settings
      if (inSettingsMenu) {
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 200) {
          lastPrint = millis();
          Serial.print("[SET HOLD] Cur=");
          Serial.print(readAS5600Deg1(), 2);
          Serial.print(" Target=");
          Serial.print(targetSensorDeg, 2);
          Serial.print(" Servo=");
          Serial.println(servoPos);
        }
      }
    }
  }

  if (inBlankMode) {
    displayBlankMode();
    return;
  }

  if (!detecting && showResult && !inEduMode) {
    displayResult();
    return;
  }

  // Education Mode
  if (inEduMode) {
    int deg = readAS5600Deg();
    if (deg < 0) {
      displayBalanceMessage();
      return;
    }

    bool triggered = inWindowDeg(deg, (int)lroundf(TARGET_DEG), TRIGGER_TOL);

    if (detectforedu == 0) {
      if (irMustReset) {
        if (!triggered) {
          irMustReset = false;
          irDetecting = false;
        }
        displayBalanceMessage();
        return;
      }

      if (triggered) {
        if (!irDetecting) {
          irDetecting = true;
          irDetectStart = millis();
        }
        if (millis() - irDetectStart >= 500) {
          detectforedu = 1;
          irDetecting = false;
          delay(200);
          displayEducationPrompt();
        }
      } else {
        irDetecting = false;
        displayBalanceMessage();
      }
    }
    return;
  }

  // Not playing? nothing else (HOLD already done above)
  if (!isGameplayActive()) return;

  // Normal / Infinite
  handleAS5600Trigger();

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate >= 100) {
    displaySystem();
    lastUpdate = now;
  }

  if (menuIndex == 2 && detecting) {
    if (millis() - lastMoveTime > maxDelay) {
      totalTime = millis() - startTime - pausedTime;
      detecting = false;
      showResult = true;
    }
  }
}



// ==== Startup GIF ====
void playStartupGif() {
  bool startPressed = false;
  while (!startPressed) {
    for (int i = 0; i < FRAME_COUNT; i++) {
      const unsigned char* framePtr = (const unsigned char*)pgm_read_ptr(&frames[i]);
      display.clearDisplay();
      display.drawBitmap(0, 0, framePtr, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
      display.display();

      if (digitalRead(ENCODER_SW) == LOW) {
        delay(50);
        if (digitalRead(ENCODER_SW) == LOW) {
          startPressed = true;
          break;
        }
      }
      delay(50);
    }
  }
  while (digitalRead(ENCODER_SW) == LOW) delay(10);
}
void resetEncoderReference() {
  lastEncPos = myEnc.getCount() / 4;
}

// ==== ENCODER ====
void handleEncoder() {

  long encPos = myEnc.getCount() / 4;  // reliable on ESP32

  if (inBlankMode) {
    long encPos = myEnc.getCount() / 4;

    if (encPos != lastEncPos) {
      blankSelect += (encPos > lastEncPos) ? 1 : -1;
      blankSelect = constrain(blankSelect, 0, 1);  // only 2 states
      displayBlankMode();
      lastEncPos = encPos;
    }
    return;
  }
  // ---------------- SETTINGS MENU ----------------
  if (inSettingsMenu) {
    if (encPos != lastEncPos) {
      int delta = (encPos > lastEncPos) ? 1 : -1;
      // ===== CENTER EDIT MODE =====
      if (editingCenter) {
        int delta = (encPos > lastEncPos) ? 1 : -1;

        float prev = centerTemp;
        centerTemp += delta * 1.0f;  // step size (1.0 deg). Change to 0.5f if you want.
        if (centerTemp >= 360.0f) centerTemp -= 360.0f;
        if (centerTemp < 0.0f) centerTemp += 360.0f;

        if (centerTemp != prev) centerDirty = true;

        // live preview (optional): update your trigger/target now
        TARGET_DEG = centerTemp;

        displaySettingsMenu();
        lastEncPos = encPos;
        return;
      }
      // ===== EDIT MODE =====
      if (editingServoAngle && editIdx >= 0) {
        int prev = editTemp;

        // If currently nan, scrolling back snaps to boundary
        if (editTemp == SERVO_UNUSED) {
          if (nanFrom == 2 && delta < 0) editTemp = AS_MAX;
          else if (nanFrom == 1 && delta > 0) editTemp = AS_MIN;
          else if (nanFrom == 0) editTemp = (delta > 0) ? AS_MIN : AS_MAX;
        }
        // Normal numeric editing + wrap to nan
        else {
          editTemp += delta;

          if (editTemp > AS_MAX) {
            editTemp = SERVO_UNUSED;
            nanFrom = 2;
          } else if (editTemp < AS_MIN) {
            editTemp = SERVO_UNUSED;
            nanFrom = 1;
          } else {
            nanFrom = 0;
          }
        }

        if (editTemp != prev) editDirty = true;

        // If it's a real AS target, update hold target
        if (editTemp != SERVO_UNUSED) {
          targetSensorDeg = (float)editTemp;  // <-- AS5600 target
          holding = false;
          active = true;

          // Serial debug (as you wanted)
          float cur = readAS5600Deg1();
          Serial.print("[SET EDIT] targetAS=");
          Serial.print(targetSensorDeg, 1);
          Serial.print(" curAS=");
          Serial.print(cur, 1);
          Serial.print(" err=");
          Serial.println(shortestError(targetSensorDeg, cur), 1);
        } else {
          // nan: disable holding so it won't fight you
          active = false;
          holding = false;
          Serial.println("[SET EDIT] targetAS = nan (holding off)");
        }

        displaySettingsMenu();
        lastEncPos = encPos;
        return;
      }

      // ===== NAV MODE (cursor only) =====
      settingSelect += delta;
      settingSelect = constrain(settingSelect, 0, servoPosCount + 2);

      int visibleRows = 4;
      int totalItems = servoPosCount + 2;

      if (settingSelect != 0) {
        if (settingSelect > settingsScroll + visibleRows) settingsScroll++;
        if (settingSelect < settingsScroll + 1) settingsScroll--;
        settingsScroll = constrain(settingsScroll, 0, max(0, totalItems - visibleRows));
      }

      displaySettingsMenu();
      lastEncPos = encPos;
    }
    return;
  }






  if (inMainMenu) {
    if (encPos != lastEncPos) {
      menuIndex += (encPos > lastEncPos) ? 1 : -1;
      menuIndex = constrain(menuIndex, 0, 5);  // 0 = info, 1..3 modes
      displayMainMenu();
      lastEncPos = encPos;
    }
    return;
  }

  // Handle info page first (must be before the generic !showResult branch)
  if (inInfoPage) {
    if (encPos != lastEncPos) {
      int dir = (encPos > lastEncPos) ? +1 : -1;

      static uint32_t lastTurnMs = 0;
      uint32_t now = millis();
      uint32_t dt = now - lastTurnMs;
      lastTurnMs = now;

      int step = 6;            // base speed
      if (dt < 80) step = 16;  // fast spin
      else if (dt < 140) step = 12;
      else if (dt < 220) step = 8;

      infoScroll += dir * step;
      infoScroll = constrain(infoScroll, 0, infoScrollMax);

      displayInfoPage();
      lastEncPos = encPos;
    }
    return;
  }


  if (inDiffMenu) {
    if (encPos != lastEncPos) {
      diffIndex += (encPos > lastEncPos) ? 1 : -1;
      diffIndex = constrain(diffIndex, 0, 3);  // 0 = back
      displayDiffMenu();
      lastEncPos = encPos;
    }
    return;
  }

  if (inEduMode) {
    long encPos = myEnc.getCount() / 4;  // reliable on ESP32
    if (encPos != lastEncPos) {
      int prevSelected = eduSelected;
      if (detectforedu == 0) {
        eduSelected += (encPos > lastEncPos) ? 1 : -1;
        eduSelected = constrain(eduSelected, 0, 1);
      } else {
        // Question screen → options selectable
        eduSelected += (encPos > lastEncPos) ? 1 : -1;

        if (currentQuestion == Q_add_both) {
          eduSelected = constrain(eduSelected, 0, 2);
        } else {
          eduSelected = constrain(eduSelected, 0, 4);
        }
      }
      displayEducationPrompt();
      lastEncPos = encPos;
    }
    return;
  }


  // When not showing result and not in any special page, this navigates system submenu
  if (!showResult) {
    if (encPos != lastEncPos) {
      subMenuIndex += (encPos > lastEncPos) ? 1 : -1;
      subMenuIndex = constrain(subMenuIndex, 0, 1);
      displaySystem();
      lastEncPos = encPos;
    }
    return;
  }

  // if we reach here, either showResult == true (no encoder action)
}

// ==== BUTTON ====
void handleButtonInterrupt() {
  static bool confirming = false;
  static uint32_t confirmAt = 0;
  static bool waitingRelease = false;

  // 1) If ISR fired, start a short confirm window
  if (btnEvent) {
    noInterrupts();
    btnEvent = false;
    interrupts();

    confirming = true;
    confirmAt = millis() + 20;  // confirm after 20ms
  }

  // 2) After confirm delay, check pin is REALLY still LOW
  if (confirming && (int32_t)(millis() - confirmAt) >= 0) {
    confirming = false;

    if (!waitingRelease && digitalRead(ENCODER_SW) == LOW) {
      waitingRelease = true;  // latch until released
      onButtonPressed();      // run your big button logic (moved here)
    }
  }

  // 3) Wait until user releases button (so one press = one action)
  if (waitingRelease && digitalRead(ENCODER_SW) == HIGH) {
    waitingRelease = false;
  }
}
void onButtonPressed() {

  if (!inMainMenu && !inDiffMenu && detecting && !inEduMode && !inInfoPage && !inBlankMode) {
    if (subMenuIndex == 1) {
      irArmed = true;
      armStart = millis();
      Serial.println("IR armed. Waiting for detection...");
      displaySystem();
    }
  }

  if (inMainMenu) {
    if (menuIndex == 0) {
      inInfoPage = true;
      infoScroll = 0;
      inMainMenu = false;
      resetEncoderReference();
      displayInfoPage();
    } else if (menuIndex == 1) {
      resetEncoderReference();
      startNormalMode();
    } else if (menuIndex == 2) {
      inDiffMenu = true;
      inMainMenu = false;
      diffIndex = 1;
      resetEncoderReference();
      displayDiffMenu();
    } else if (menuIndex == 3) {
      resetEncoderReference();
      startEducationMode();
    } else if (menuIndex == 4) {
      inMainMenu = false;
      inBlankMode = true;
      detecting = true;
      showResult = false;
      blankSelect = 1;
      resetEncoderReference();
      displayBlankMode();
    } else if (menuIndex == 5) {
      inMainMenu = false;
      inSettingsMenu = true;
      resetEncoderReference();
      displaySettingsMenu();
    }
  } else if (inDiffMenu) {
    if (diffIndex == 0) {
      inDiffMenu = false;
      inMainMenu = true;
      displayMainMenu();
    } else {
      applyDifficulty(diffIndex);
      startInfiniteMode();
    }
  } else if (inEduMode) {
    if (eduSelected == 0) {
      inEduMode = false;
      irDetecting = false;
      detecting = false;
      inDiffMenu = false;
      inMainMenu = true;
      showResult = false;
      stopHold();
      smoothMove(90);
      displayMainMenu();
    } else if (detectforedu == 1) {
      displayEducationFeedback();
    }
  } else if (inInfoPage) {
    inInfoPage = false;
    inMainMenu = true;
    displayMainMenu();
  } else if (inBlankMode) {
    if (blankSelect == 0) {
      inBlankMode = false;
      blankSelect = 0;
      inMainMenu = true;
      stopHold();
      smoothMove(90);
      displayMainMenu();
    }
  } else if (inSettingsMenu) {
    if (settingSelect == 0) {
      editingServoAngle = false;
      inSettingsMenu = false;
      inMainMenu = true;
      stopHold();
      smoothMove(90);
      resetEncoderReference();
      displayMainMenu();
    } else if (settingSelect >= 1 && settingSelect <= servoPosCount) {
      if (!editingServoAngle) {
        nanFrom = 0;
        editIdx = settingSelect - 1;
        editOriginal = servoPositions[editIdx];

        editTemp = (editOriginal == SERVO_UNUSED) ? getCurASInt() : constrain(editOriginal, 0, 359);
        editDirty = false;

        editingServoAngle = true;

        stopHold();
        gotoAS5600Target((float)editTemp);
        displaySettingsMenu();
      } else {
        if (editDirty) servoPositions[editIdx] = editTemp;
        else servoPositions[editIdx] = editOriginal;

        editingServoAngle = false;

        saveServoPositions();
        Serial.println("Servo angle saved");

        stopHold();
        smoothMove(servoCenterAngle);

        editIdx = -1;
        displaySettingsMenu();
      }
    } else if (settingSelect == servoPosCount + 1) {
      // ENTER edit
      if (!editingCenter) {
        editingCenter = true;
        centerOriginal = TARGET_DEG;
        centerTemp = TARGET_DEG;
        centerDirty = false;
        displaySettingsMenu();
      }
      // EXIT edit (save)
      else {
        editingCenter = false;

        if (centerDirty) {
          TARGET_DEG = centerTemp;
          saveServoPositions();
          Serial.println("Center saved");
        } else {
          TARGET_DEG = centerOriginal;
        }

        displaySettingsMenu();
      }
    } else if (settingSelect == servoPosCount + 2) {
      prefs.begin("settings", false);
      prefs.clear();
      prefs.end();

      int defaults[MAX_SERVO_POS] = { 180, -1, -1, -1, -1, -1, -1, -1, -1, -1 };

      servoPosCount = MAX_SERVO_POS;
      memcpy(servoPositions, defaults, sizeof(defaults));

      editingServoAngle = false;
      settingsScroll = 0;
      settingSelect = 0;

      saveServoPositions();
      stopHold();
      smoothMove(90);

      Serial.println("ALL SETTINGS RESET");
      displaySettingsMenu();
    }
  } else if (showResult) {
    showResult = false;
    inMainMenu = true;
    detecting = false;
    inEduMode = false;
    pausedTime = 0;
    irPaused = false;
    irDetected = false;
    inBlankMode = false;
    inSettingsMenu = false;
    moveCount = 0;
    lastGameIndex = -1;
    stopHold();
    smoothMove(90);
    subMenuIndex = 1;
    displayMainMenu();
  } else if (subMenuIndex == 0) {
    inMainMenu = true;
    detecting = false;
    pausedTime = 0;
    irPaused = false;
    irDetected = false;
    moveCount = 0;
    stopHold();
    smoothMove(90);
    displayMainMenu();
  }
}



// ==== START MODES ====
void startNormalMode() {
  inMainMenu = false;
  lastAngle = 90;
  detecting = true;
  showResult = false;
  moveCount = 0;
  servoAllowed = true;
  subMenuIndex = 1;
  startTime = millis();
  moveServoRandom();
  displaySystem();
  Serial.println("Mode: NORMAL");
}


// Main Menu
void displayMainMenu() {
  Serial.println("Mode: main");
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  drawinfo(menuIndex == 0);
  display.setCursor(getCenteredX("SELECT MODE", 1), 0);
  display.println("SELECT MODE");

  String items[4] = { "NORMAL", "INFINITE", "EDUCATION", "AUTO BALANCE" };

  for (int i = 0; i < 4; i++) {
    int y = 20 + i * 10;

    String word = items[i];
    int xWord = getCenteredX(word, 1);  // keep word centred

    // draw word
    display.setCursor(xWord, y);
    display.print(word);

    // draw ">" just left of the word with a small gap
    if (menuIndex - 1 == i) {
      const int charW = 6;  // textSize=1
      const int gap = 3;    // ✅ increase gap here (pixels)

      int xL = xWord - charW - gap;
      if (xL < 0) xL = 0;

      display.setCursor(xL, y);
      display.print(">");
    }
  }




  drawSettingsIcon(menuIndex == 5);

  display.display();
}


// Difficulty Menu
void startInfiniteMode() {
  inMainMenu = false;
  inDiffMenu = false;
  detecting = true;
  showResult = false;
  moveCount = 0;
  servoAllowed = true;
  subMenuIndex = 1;
  startTime = millis();
  lastMoveTime = millis();
  moveServoRandom();
  displaySystem();
  Serial.println("Mode: INFINITE");
}
void applyDifficulty(int level) {
  if (level == 1) {
    maxDelay = baseMaxDelay * 1.5;
    minDelay = baseMinDelay * 1.5;
  } else if (level == 2) {
    maxDelay = baseMaxDelay;
    minDelay = baseMinDelay;
  } else if (level == 3) {
    maxDelay = baseMaxDelay * 0.5;
    minDelay = baseMinDelay * 0.5;
  }
  Serial.print("Difficulty set: ");
  Serial.println(level);
}
void displayDiffMenu() {
  Serial.println("Mode: diff menu");
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(getCenteredX("DIFFICULTY", 1), 0);
  display.println("DIFFICULTY");

  String items[3] = { "EASY", "NORMAL", "HARD" };

  for (int i = 0; i < 3; i++) {
    int y = 20 + i * 10;

    String word = items[i];
    int xWord = getCenteredX(word, 1);

    display.setCursor(xWord, y);
    display.print(word);

    if (diffIndex == i + 1) {
      const int charW = 6;
      const int gap = 3;  // ✅ increase gap here (pixels)

      int xL = xWord - charW - gap;
      if (xL < 0) xL = 0;

      display.setCursor(xL, y);
      display.print(">");
    }
  }



  drawBackArrow(diffIndex == 0);  // back arrow
  display.display();
}



//edu mode
void startEducationMode() {
  inMainMenu = false;
  inEduMode = true;
  detecting = true;
  showResult = false;
  moveCount = 0;
  eduSelected = 1;
  detectforedu = 0;
  moveServoRandomEducation();
  generateEducationOptions();
  displayBalanceMessage();
  Serial.println("Mode: EDUCATION");
}
String lookupToWord(float value, EduQuestionType q) {

  switch (q) {

    // -------------------------------
    // BALANCE / POSITION QUESTIONS
    // -------------------------------
    /*case Q_remove:
      if (value < 2.0) return "Very Left";
      if (value < 3.0) return "Left";
      if (value < 4.0) return "Slight Left";
      if (value < 5.0) return "Balanced";
      if (value < 6.0) return "Slight Right";
      if (value < 7.0) return "Right";
      return "Very Right";*/

    // -------------------------------
    // DIRECTION QUESTIONS
    // -------------------------------
    case Q_remove:
      if (value == 1) return "Rotate Left";
      if (value == 2) return "Stable";
      if (value == 3) return "Rotate Right";
      return "Oscillating";
    // -------------------------------
    // STABILITY QUESTIONS
    // -------------------------------


    // -------------------------------
    // FALLBACK
    // -------------------------------
    default:
      return "Unknown";
  }
}
float getCorrectAnswerDynamic() {
  int i = eduLastIndex;  // <-- use education index
  if (i < 0) i = 0;

  switch (currentQuestion) {
    case Q_find_force: return answerTable_force[i];
    case Q_find_moment: return answerTable_moment[i];
    case Q_total_weight: return answerTable_total_weight[i];
    case Q_add_both: return answerTable_add_both[i];
    case Q_remove: return answerTable_remove[i];
  }
  return 0;
}

void generateEducationOptions() {
  eduOptionCount = 4;  // reset default


  eduIsWordQuestion = false;
  eduIsTrueFalse = false;

  // -------------------------------------------------
  // TRUE / FALSE (1 / 0)
  // -------------------------------------------------
  if (currentQuestion == Q_add_both) {

    eduIsTrueFalse = true;
    eduIsWordQuestion = false;
    eduOptionCount = 2;

    float correctVal = getCorrectAnswerDynamic();  // 0 or 1
    eduAnswer = random(0, 2);                      // index 0 or 1

    for (int i = 0; i < 4; i++) eduOptionsText[i] = "";

    eduOptionsText[eduAnswer] = (correctVal == 1) ? "Yes" : "No";
    eduOptionsText[1 - eduAnswer] = (correctVal == 1) ? "No" : "Yes";

    return;
  }

  // -------------------------------------------------
  // WORD-BASED QUESTIONS
  // -------------------------------------------------
  if (/*currentQuestion == Q_total_weight || */ currentQuestion == Q_remove) {

    eduIsWordQuestion = true;

    float correctVal = getCorrectAnswerDynamic();
    eduCorrectWord = lookupToWord(correctVal, currentQuestion);

    eduAnswer = random(0, 4);
    for (int i = 0; i < 4; i++) eduOptionsText[i] = "";

    eduOptionsText[eduAnswer] = eduCorrectWord;

    // -------- word pool per question --------
    String pool[7];
    int poolSize = 0;

    switch (currentQuestion) {

      case Q_remove:
        pool[0] = "Rotate Left";
        pool[1] = "Stable";
        pool[2] = "Rotate Right";
        pool[3] = "Oscillating";
        poolSize = 4;
        break;



      default:  // balance / lookup
        pool[0] = "Very Left";
        pool[1] = "Left";
        pool[2] = "Slight Left";
        pool[3] = "Balanced";
        pool[4] = "Slight Right";
        pool[5] = "Right";
        pool[6] = "Very Right";
        poolSize = 7;
        break;
    }

    // -------- fill wrong options --------
    for (int i = 0; i < 4; i++) {
      if (i == eduAnswer) continue;

      String w;
      bool duplicate;

      do {
        duplicate = false;
        w = pool[random(0, poolSize)];

        // Check against correct answer
        if (w == eduCorrectWord) {
          duplicate = true;
          continue;
        }

        // Check against already-filled options
        for (int j = 0; j < 4; j++) {
          if (j == i) continue;
          if (eduOptionsText[j] == w && eduOptionsText[j] != "") {
            duplicate = true;
            break;
          }
        }

      } while (duplicate);

      eduOptionsText[i] = w;
    }


    return;
  }

  // -------------------------------------------------
  // NUMERIC QUESTIONS
  // -------------------------------------------------
  // -------------------------------
  float* answerPool = nullptr;
  int poolSize = EDU_POS_COUNT;

  switch (currentQuestion) {
    case Q_find_force:
      answerPool = answerTable_force;
      break;

    case Q_find_moment:
      answerPool = answerTable_moment;
      break;

    case Q_total_weight:
      answerPool = answerTable_total_weight;
      break;

    default:
      answerPool = answerTable_total_weight;  // safety fallback
      break;
  }

  float correctValue = getCorrectAnswerDynamic();

  int correctIndex = random(0, 4);
  eduAnswer = correctIndex;

  for (int i = 0; i < 4; i++) eduOptions[i] = -99999;
  eduOptions[correctIndex] = correctValue;

  // ------------------------------------
  // Build pool of other valid answers
  // ------------------------------------
  float pool[EDU_POS_COUNT];
  int wrongPoolSize = 0;

  for (int i = 0; i < poolSize; i++) {
    float v = answerPool[i];
    if (v != correctValue) {
      pool[wrongPoolSize++] = v;
    }
  }
  // ------------------------------------
  // Pick wrong answers from pool
  // ------------------------------------
  for (int i = 0; i < 4; i++) {
    if (i == correctIndex) continue;

    bool duplicate;
    float candidate;

    do {
      duplicate = false;
      candidate = pool[random(0, poolSize)];

      for (int j = 0; j < 4; j++) {
        if (eduOptions[j] == candidate) {
          duplicate = true;
          break;
        }
      }

    } while (duplicate);

    eduOptions[i] = candidate;
  }
}


void displayEducationPrompt() {
  if (detectforedu == 0) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Titles
  display.setCursor(getCenteredX("EDUCATION MODE", 1), 0);
  display.println("EDUCATION MODE");

  // You can customize this text later too
  String question;

  switch (currentQuestion) {
    case Q_find_force: question = "Force(N) applied by placed weight?"; break;
    case Q_find_moment: question = "What is the moment(Ncm) applied by the weight?"; break;
    case Q_total_weight: question = "What is the total placed weight(kg)?"; break;  //not done
    case Q_add_both: question = "Will it balance if 1kg is added to both sides?"; break;
    case Q_remove: question = "If placed weight is removed what will happen?"; break;
  }
  display.setFont(&Font5x7Fixed);
  display.setTextSize(1);

  int endOfQuestionY = printWrappedCenteredText5x7(question, 18);


  drawBackArrow(eduSelected == 0);

  int rowHeight = 20;
  int colWidth = SCREEN_WIDTH / 2;

  for (int i = 0; i < eduOptionCount; i++) {
    int y;

    int col, row;
    if (eduOptionCount == 2) {
      col = i;  // left / right
      row = 0;
      y = 46 + row * rowHeight;
    } else {
      col = i % 2;
      row = i / 2;
      y = 36 + row * rowHeight;
    }



    String optionText;

    if (eduIsWordQuestion || eduIsTrueFalse) {
      optionText = eduOptionsText[i];
    } else {
      optionText = String(eduOptions[i], 1);
    }

    if ((eduSelected - 1) == i)
      optionText = "> " + optionText;

    int textWidth = optionText.length() * 6;
    int x = col * colWidth + (colWidth - textWidth) / 2;
    display.setFont(&Font5x7Fixed);
    display.setTextSize(1);
    display.setCursor(x, y);
    display.println(optionText);
  }

  display.setFont();
  display.display();
}
void displayBalanceMessage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(getCenteredX("EDUCATION MODE", 1), 0);
  display.println("EDUCATION MODE");

  drawBackArrow(eduSelected == 0);  // draw arrow at top left

  display.setCursor(getCenteredX("Balance it first", 1), 28);
  display.println("Balance it first");

  display.display();
}
void displayEducationFeedback() {

  Serial.println("Mode: edu feed");
  display.setFont();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  String msg = (eduSelected == (eduAnswer + 1)) ? "Correct!" : "";
  display.setCursor(getCenteredX(msg, 2), 16);
  display.println(msg);
  if (eduSelected != (eduAnswer + 1)) {
    if (currentQuestion == Q_find_moment) {
      String msg = "Wrong \n Remember Moment(Ncm) is Force(N) X distance(cm)";
      printWrappedCenteredText(msg, 9, 1);
    } else if (currentQuestion == Q_find_force) {
      String msg = "Remember Force (N) is Mass (kg) X gravity \n Where gravity is 9.8m/s^2";
      printWrappedCenteredText(msg, 9, 1);
    } else if (currentQuestion == Q_total_weight) {
      String msg = "Wrong \n Total weight is the combine weight(kg) of right side";
      printWrappedCenteredText(msg, 9, 1);
    } else if (currentQuestion == Q_add_both) {
      String msg = "Wrong \n You can test this by adding 1kg to both sides";
      printWrappedCenteredText(msg, 9, 1);
    } else if (currentQuestion == Q_remove) {
      String msg = "Wrong \n You can test this by removing all placed weights";
      printWrappedCenteredText(msg, 9, 1);
    } 
    
    else {
      String msg = "error";
      printWrappedCenteredText(msg, 12, 1);
    }
  }

  display.setTextSize(1);
  display.setCursor(getCenteredX("Press to continue", 1), 50);
  display.println("Press to continue");

  display.display();

  while (digitalRead(ENCODER_SW) == LOW) {
    delay(10);
  }

  while (digitalRead(ENCODER_SW) == HIGH) {
    delay(10);
  }

  while (digitalRead(ENCODER_SW) == LOW) {
    delay(10);
  }


  moveServoRandomEducation();
  generateEducationOptions();

  irMustReset = true;
  eduSelected = 1;
  detectforedu = 0;

  displayEducationPrompt();
}



//font
int getCenteredX_Font5x7(String text) {
  int width = text.length() * 5;  // 5 px per character
  return (SCREEN_WIDTH - width) / 2;
}
void printWrappedCenteredText(String text, int startY, int textSize) {

  int charWidth = 6 * textSize;
  int lineHeight = 8 * textSize;
  int maxCharsPerLine = SCREEN_WIDTH / charWidth;

  display.setTextSize(textSize);

  int y = startY;

  int textLen = (int)text.length();
  int paraStart = 0;

  while (paraStart < textLen) {

    // -----------------------------
    // Find next newline
    // -----------------------------
    int paraEnd = text.indexOf('\n', paraStart);
    if (paraEnd == -1) {
      paraEnd = textLen;
    }

    // Extract paragraph
    String paragraph = text.substring(paraStart, paraEnd);
    int start = 0;
    int paraLen = paragraph.length();

    // -----------------------------
    // Wrap THIS paragraph
    // -----------------------------
    while (start < paraLen) {

      int remaining = paraLen - start;
      int len = min(maxCharsPerLine, remaining);
      int end = start + len;

      // Break at space if possible
      if (end < paraLen) {
        while (end > start && paragraph[end] != ' ') {
          end--;
        }
        if (end == start) {
          end = start + len;  // force break
        }
      }

      String line = paragraph.substring(start, end);
      line.trim();

      int x = getCenteredX(line, textSize);
      display.setCursor(x, y);
      display.println(line);

      start = end + 1;
      y += lineHeight;
    }

    // -----------------------------
    // Newline = extra line spacing
    // -----------------------------
    y += lineHeight;
    paraStart = paraEnd + 1;
  }
}
int printWrappedCenteredText5x7(String text, int startY) {

  display.setTextSize(1);
  int y = startY;

  int pos = 0;
  int len = text.length();

  while (pos < len) {

    // -------- paragraph handling --------
    int paraEnd = text.indexOf('\n', pos);
    if (paraEnd == -1) paraEnd = len;

    String paragraph = text.substring(pos, paraEnd);
    pos = paraEnd + 1;

    int i = 0;
    int paraLen = paragraph.length();

    while (i < paraLen) {

      // skip spaces
      while (i < paraLen && paragraph.charAt(i) == ' ') i++;
      if (i >= paraLen) break;

      String line = "";
      int nextIndex = i;
      bool wordPlaced = false;

      // -------- build line word-by-word --------
      while (nextIndex < paraLen) {

        int wordEnd = paragraph.indexOf(' ', nextIndex);
        if (wordEnd == -1) wordEnd = paraLen;

        String word = paragraph.substring(nextIndex, wordEnd);
        String testLine = line.length() ? line + " " + word : word;

        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(testLine, 0, 0, &x1, &y1, &w, &h);

        // ---------- word does not fit ----------
        if (w > SCREEN_WIDTH - 2) {

          // if no word yet, FORCE one word
          if (!wordPlaced) {
            line = word;
            nextIndex = wordEnd;
            wordPlaced = true;
          }
          break;
        }

        // ---------- word fits ----------
        line = testLine;
        nextIndex = wordEnd;
        wordPlaced = true;

        // skip space
        if (nextIndex < paraLen && paragraph.charAt(nextIndex) == ' ') {
          nextIndex++;
        }
      }

      // -------- draw guaranteed non-empty line --------
      if (line.length() > 0) {
        int x = getCenteredX5x7(line);
        display.setCursor(x, y);
        display.print(line);
        y += 8;
      }

      // -------- advance safely --------
      i = nextIndex;
    }
  }

  return y;
}





int getCenteredX5x7(const String& text) {
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  return (SCREEN_WIDTH - w) / 2 - x1;
}







// game
void displaySystem() {

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Header
  display.setCursor(getCenteredX(menuIndex == 1 ? "NORMAL MODE" : "INFINITE MODE", 1), 0);
  display.println(menuIndex == 1 ? "NORMAL MODE" : "INFINITE MODE");

  // Difficulty (Infinite mode only)
  if (menuIndex == 2) {
    String diffText = "Difficulty: ";
    switch (diffIndex) {
      case 1: diffText += "EASY"; break;
      case 2: diffText += "NORMAL"; break;
      case 3: diffText += "HARD"; break;
    }
    display.setCursor(getCenteredX(diffText, 1), 15);
    display.println(diffText);
  }

  drawBackArrow(subMenuIndex == 0);

  // Moves
  display.setCursor(getCenteredX("Moves: " + String(moveCount), 1), 30);
  display.println("Moves: " + String(moveCount));

  // Time
  unsigned long elapsed = irPaused ? (irPauseStart - startTime - pausedTime) : (millis() - startTime - pausedTime);
  display.setCursor(getCenteredX("Time: " + String(elapsed / 1000.0, 1) + "s", 1), 40);
  display.println("Time: " + String(elapsed / 1000.0, 1) + "s");

  // NEW: Instruction line (centered properly)
  display.setFont(&Font5x7Fixed);
  String inst = irArmed ? "scanning..." : "Press button when ready";
  display.setCursor(getCenteredX_Font5x7(inst), 55);
  display.println(inst);
  display.setFont();
  int deg = readAS5600Deg();
  display.setCursor(0, 52);
  display.print("AS:");
  display.print(deg);

  display.display();
}
void displayResult() {
  Serial.println("Mode: result");
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setFont();
  String resultText = menuIndex == 1 ? "COMPLETED!" : "GAME OVER";
  display.setCursor(getCenteredX(resultText, 2), 5);
  display.println(resultText);

  display.setTextSize(1);
  if (menuIndex == 2) {
    String diffText = "Difficulty: ";
    switch (diffIndex) {
      case 1: diffText += "EASY"; break;
      case 2: diffText += "NORMAL"; break;
      case 3: diffText += "HARD"; break;
    }
    display.setCursor(getCenteredX(diffText, 1), 24);
    display.println(diffText);
  }

  display.setCursor(getCenteredX("Moves: " + String(moveCount), 1), 37);
  display.println("Moves: " + String(moveCount));
  display.setCursor(getCenteredX("Time: " + String(totalTime / 1000.0, 2) + "s", 1), 47);
  display.println("Time: " + String(totalTime / 1000.0, 2) + "s");
  display.setCursor(getCenteredX("Press to return", 1), 56);
  display.println("Press to return");
  display.display();
}
// Draw Back Arrow
void drawBackArrow(bool selected) {
  int x = 0, y = 0;
  if (selected) {
    display.fillRect(x, y, 17, 16, SSD1306_WHITE);
    display.drawBitmap(x, y, arrows, 17, 16, SSD1306_BLACK);
  } else {
    display.fillRect(x, y, 12, 12, SSD1306_WHITE);
    display.drawBitmap(x, y, arrow, 12, 12, SSD1306_BLACK);
  }
}



//info
void drawinfo(bool selected) {
  int x = 110, y = 0;

  if (selected) {
    // Selected = normal white icon
    display.fillRect(x, y, 16, 16, SSD1306_WHITE);
    display.drawBitmap(x, y, info, 16, 16, SSD1306_BLACK);
  } else {
    // Not selected = inverted (white box + black icon)
    display.fillRect(x, y, 16, 16, SSD1306_WHITE);
    display.drawBitmap(x, y, infos, 16, 16, SSD1306_BLACK);
  }
}
void displayInfoPage() {
  Serial.println("Mode: info");
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Body font
  display.setFont(&Font5x7Fixed);
  display.setTextSize(1);

  const char* infoText =

    "\n"
    "[U][C]NORMAL MODE\n"
    "\n"
    "- Balance the lever using the weights given\n"
    "- The lever will move after the lever is balanced\n"
    "- New leverage can now be calculated\n"
    "- Complete 10 moves to win\n"
    "\n"
    "\n"
    "[U][C]INFINITE MODE\n"
    "\n"
    "- Endless gameplay\n"
    "- Balance the lever using the weights given\n"
    "- Game ends if it takes too long to balance\n"
    "- Time and score will be given\n"
    "\n"
    "\n"
    "[U][C]EDUCATION MODE\n"
    "\n"
    "- Balance the lever using the weights given\n"
    "- Questions will be given\n"
    "- 4 different options will show up\n"
    "- Select the correct option\n"
    "\n"
    "\n"
    "[U][C]AUTO BALANCE\n"
    "\n"
    "- Automatically balance the drum\n"
    "- Drum angle (180 deg = perfectly balanced)\n"
    "- Shows servo actual angle\n"
    "- Slows down when closer to balanced\n"
    "\n"
    "[C]==================\n"
    "[C]By:Lewis Tan\n"
    "[C]DCPE/FT/3B/04\n"
    "\n"
    "\n";

  int textMargin = 6;
  int textWidthLimit = SCREEN_WIDTH - textMargin;
  int lineHeight = 12;
  int sectionSpacing = 10;
  int y = -infoScroll;
  int dashIndent = 6;

  char lineBuffer[128];
  const char* ptr = infoText;

  while (*ptr) {
    int i = 0;
    while (*ptr && *ptr != '\n') {
      lineBuffer[i++] = *ptr++;
      if (i >= (int)sizeof(lineBuffer) - 1) break;
    }
    lineBuffer[i] = 0;
    if (*ptr == '\n') ptr++;

    String line = String(lineBuffer);

    // ---------- TAG PARSE ----------
    bool centerLine = false;
    bool underlineLine = false;

    bool tagsFound = true;
    while (tagsFound && line.startsWith("[")) {
      tagsFound = false;

      if (line.startsWith("[C]")) {
        centerLine = true;
        line = line.substring(3);
        tagsFound = true;
      }
      if (line.startsWith("[U]")) {
        underlineLine = true;
        line = line.substring(3);
        tagsFound = true;
      }
    }

    bool isDashLine = line.startsWith("- ");
    int start = isDashLine ? 2 : 0;
    int currentX = 0;
    int lineLimit = textWidthLimit;

    // only underline the first printed row of this line (not wrapped parts)
    bool underlineDone = false;

    while (start < line.length()) {
      int end = start;
      int linePixelWidth = 0;
      String subLine = "";

      while (end < line.length()) {
        int nextSpace = line.indexOf(' ', end);
        if (nextSpace == -1) nextSpace = line.length();

        String word = line.substring(end, nextSpace + (nextSpace < line.length() ? 1 : 0));

        int wordWidth = centerLine ? (word.length() * 6) : (word.length() * 5);
        if (linePixelWidth + wordWidth > lineLimit) break;

        subLine += word;
        linePixelWidth += wordWidth;
        end = nextSpace + 1;
      }

      // ---------- PRINT ----------
      int xPrint = 0;

      if (centerLine) {
        // switch to default font
        display.setFont();
        display.setTextSize(1);

        xPrint = getCenteredX(subLine, 1);
        display.setCursor(xPrint, y);
        display.println(subLine);

        // underline (for centered too) using default font width (6px/char)
        if (underlineLine && !underlineDone) {
          int w = subLine.length() * 6;
          int uy = y + 9;  // underline y (tweak if you want lower/higher)
          display.drawLine(xPrint, uy, xPrint + w - 1, uy, SSD1306_WHITE);
          underlineDone = true;
        }

        // back to body font
        display.setFont(&Font5x7Fixed);
        display.setTextSize(1);
      } else if (isDashLine && start == 2) {
        display.setCursor(0, y);
        display.println("- " + subLine);

        if (underlineLine && !underlineDone) {
          int w = (String("- ") + subLine).length() * 5;
          int uy = y + 9;
          display.drawLine(0, uy, w - 1, uy, SSD1306_WHITE);
          underlineDone = true;
        }
      } else {
        display.setCursor(currentX, y);
        display.println(subLine);

        if (underlineLine && !underlineDone) {
          int w = subLine.length() * 5;
          int uy = y + 9;
          display.drawLine(currentX, uy, currentX + w - 1, uy, SSD1306_WHITE);
          underlineDone = true;
        }
      }

      y += lineHeight;
      start = end;
      currentX = isDashLine ? dashIndent : 0;
      lineLimit = textWidthLimit - currentX;
    }

    if (strlen(lineBuffer) == 0 || line.endsWith("MODE:")) y += sectionSpacing;
  }

  int totalHeight = y + infoScroll;
  infoScrollMax = max(0, totalHeight - SCREEN_HEIGHT);

  if (infoScrollMax > 0) {
    int barWidth = 3;
    int barX = SCREEN_WIDTH - barWidth;
    int barHeight = (SCREEN_HEIGHT * SCREEN_HEIGHT) / totalHeight;
    int barPos = (infoScroll * (SCREEN_HEIGHT - barHeight - 1)) / infoScrollMax;
    display.fillRect(barX, barPos, barWidth, barHeight, SSD1306_WHITE);
  }

  display.display();

  // restore default for other screens
  display.setFont();
  display.setTextSize(1);
}


//mode 4
void displayBlankMode() {
  handleButtonInterrupt();

  // ----- Read sensor first -----
  float deg;
  if (!readAS5600Deg2(deg)) return;

  // ----- Control math -----
  float err = signedDiff(TARGET_DEG, deg);
  float ae = fabs(err);
  bool balanced = ae <= TOL;

  int desiredDir = 0;
  if (!balanced) desiredDir = (err > 0) ? +1 : -1;

  // ----- Base interval from zones -----
  unsigned long moveInterval = SETTLE_MS;
  if (ae <= SUPER_SLOW_DEG) {
    moveInterval = SUPER_SLOW_MS;
  } else if (ae <= SLOW_ZONE_DEG) {
    moveInterval = SETTLE_MS * (unsigned long)SLOW_MULT;
  }

  unsigned long now = millis();

  // ----- Pause BEFORE reversing direction + start ramp AFTER pause -----
  if (desiredDir != 0 && lastMoveDir != 0 && desiredDir != lastMoveDir) {
    if (pauseUntilMs == 0) {
      pauseUntilMs = now + DIR_PAUSE_MS;
      rampUntilMs = pauseUntilMs + RAMP_MS;  // ✅ ramp starts after pause ends
    }
  }

  bool inDirPause = (pauseUntilMs != 0 && now < pauseUntilMs);
  if (pauseUntilMs != 0 && now >= pauseUntilMs) pauseUntilMs = 0;

  // ✅ Soft-start ramp after direction change: prevents max-speed "kick"
  if (rampUntilMs != 0) {
    if (now < rampUntilMs) {
      // time into ramp (0..1)
      unsigned long rampStart = (rampUntilMs > RAMP_MS) ? (rampUntilMs - RAMP_MS) : 0;
      float t = (now <= rampStart) ? 0.0f : (float)(now - rampStart) / (float)RAMP_MS;
      t = constrain(t, 0.0f, 1.0f);

      // multiplier goes from RAMP_MULT -> 1
      float mult = (float)RAMP_MULT - ((float)(RAMP_MULT - 1) * t);

      // slow down by multiplier (bigger interval = slower)
      moveInterval = (unsigned long)(moveInterval * mult);
    } else {
      rampUntilMs = 0;  // ramp finished
    }
  }

  // Optional: smaller steps during ramp (extra anti-oscillation)
  int stepNow = SERVO_STEP;
  if (rampUntilMs != 0) stepNow = 1;

  // ---- Servo control ----
  if (!balanced && !inDirPause) {
    if (now - lastMoveMs >= moveInterval) {
      servoPos += desiredDir * stepNow;
      servoPos = constrain(servoPos, SERVO_MIN, SERVO_MAX);
      myServo.write(servoPos);

      lastMoveMs = now;
      lastMoveDir = desiredDir;
    }
  }

  float cur = readAS5600Deg1();

  // ----- OLED UI -----
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Back arrow
  drawBackArrow(blankSelect == 0);

  // Title
  const String title = "Auto Balance";
  display.setCursor(getCenteredX(title, 1), 0);
  display.println(title);

  display.setCursor(0, 14);
  display.print("Deg: ");
  display.println(deg - ((int)lroundf(TARGET_DEG) - 180), 2);

  display.setCursor(0, 28);
  display.print("Err: ");
  display.println(err, 2);

  display.setCursor(0, 42);
  display.print("Servo:");
  display.print(servoPos);
  display.print(" Cur:");
  display.println(cur, 2);

  display.setCursor(0, 54);
  if (balanced) display.println("BALANCED");
  else if (inDirPause) display.println("DIR PAUSE");
  else if (rampUntilMs != 0) display.println("RAMPING");
  else if (ae <= SUPER_SLOW_DEG) display.println("SUPER SLOW");
  else if (ae <= SLOW_ZONE_DEG) display.println("SLOW ZONE");
  else display.println("ADJUSTING");

  display.display();

  // ----- Serial debug -----
  Serial.print("deg=");
  Serial.print(deg, 2);
  Serial.print(" err=");
  Serial.print(err, 2);
  Serial.print(" servo=");
  Serial.print(servoPos);
  Serial.print(" interval=");
  Serial.print(moveInterval);
  Serial.print("ms state=");
  if (balanced) Serial.print("BAL");
  else if (inDirPause) Serial.print("DIR_PAUSE");
  else if (rampUntilMs != 0) Serial.print("RAMP");
  else if (ae <= SUPER_SLOW_DEG) Serial.print("SUPER_SLOW");
  else if (ae <= SLOW_ZONE_DEG) Serial.print("SLOW");
  else Serial.print("MOVE");
  Serial.println();

  delay(10);
}
float signedDiff(float target, float current) {
  float d = wrap360(target) - wrap360(current);
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  return d;  // -180 .. +180
}
bool readAS5600Deg2(float& degOut) {
  const int N = 200;  // more samples = smoother (slower)
  uint32_t sum = 0;

  for (int i = 0; i < N; i++) {
    sum += analogRead(AS5600_OUT_PIN);  // 0..4095 (12-bit)
    delayMicroseconds(100);
  }

  float adc = sum / (float)N;            // averaged ADC
  float deg = (adc / 4095.0f) * 360.0f;  // map to degrees

  // Optional invert if your mechanism is reversed
  if (invertDirection) deg = 360.0f - deg;

  degOut = wrap360(deg);
  return true;  // ADC read always “works”
}




//servo and random numb gen
int getSmartRandomGameIndex() {
  int weights[MAX_SERVO_POS] = { 0 };

  // ---- collect valid indices (non-nan) ----
  int validIdx[MAX_SERVO_POS];
  int validCount = 0;

  for (int i = 0; i < servoPosCount; i++) {
    if (servoPositions[i] != SERVO_UNUSED) {
      validIdx[validCount++] = i;
    }
  }

  // No valid angles at all
  if (validCount == 0) return -1;

  // Exactly one valid angle -> ALWAYS pick it
  if (validCount == 1) {
    int only = validIdx[0];
    servoUsageGame[only]++;
    lastGameIndex = only;
    return only;
  }

  // ---- normal weighted pick (2+ valid) ----
  int totalWeight = 0;

  for (int i = 0; i < servoPosCount; i++) {
    if (servoPositions[i] == SERVO_UNUSED) {
      weights[i] = 0;
      continue;
    }

    // base weight: less used = more likely
    int w = max(1, 20 - servoUsageGame[i]);

    // don't repeat last pick (only matters because validCount >= 2 here)
    if (i == lastGameIndex) w = 0;

    weights[i] = w;
    totalWeight += w;
  }

  // Safety: if somehow all weights became 0, just pick any valid
  if (totalWeight <= 0) {
    int pick = validIdx[random(0, validCount)];
    servoUsageGame[pick]++;
    lastGameIndex = pick;
    return pick;
  }

  int roll = random(0, totalWeight);
  int cumulative = 0;

  for (int i = 0; i < servoPosCount; i++) {
    if (weights[i] == 0) continue;
    cumulative += weights[i];
    if (roll < cumulative) {
      servoUsageGame[i]++;
      lastGameIndex = i;
      return i;
    }
  }

  // Final fallback
  int pick = validIdx[random(0, validCount)];
  servoUsageGame[pick]++;
  lastGameIndex = pick;
  return pick;
}

int getSmartRandomEduIndex() {
  int weights[EDU_POS_COUNT];
  int totalWeight = 0;

  for (int i = 0; i < EDU_POS_COUNT; i++) {
    int w = max(1, 20 - eduUsage[i]);

    // avoid repeat
    if (EDU_POS_COUNT > 1 && i == eduLastIndex) w = 0;

    weights[i] = w;
    totalWeight += w;
  }

  // fallback: pick any index except last
  if (totalWeight <= 0) {
    int pick = random(0, EDU_POS_COUNT);
    if (EDU_POS_COUNT > 1) {
      while (pick == eduLastIndex) pick = random(0, EDU_POS_COUNT);
    }
    eduUsage[pick]++;
    eduLastIndex = pick;
    return pick;
  }

  int roll = random(0, totalWeight);
  int cumulative = 0;

  for (int i = 0; i < EDU_POS_COUNT; i++) {
    if (weights[i] == 0) continue;
    cumulative += weights[i];
    if (roll < cumulative) {
      eduUsage[i]++;
      eduLastIndex = i;
      return i;
    }
  }

  // final safety (also avoid repeat)
  int pick = random(0, EDU_POS_COUNT);
  if (EDU_POS_COUNT > 1) {
    while (pick == eduLastIndex) pick = random(0, EDU_POS_COUNT);
  }
  eduUsage[pick]++;
  eduLastIndex = pick;
  return pick;
}

void moveServoRandomEducation() {

  int idx = getSmartRandomEduIndex();

  eduLastIndex = idx;  

  // Random question each round
  currentQuestion = (EduQuestionType)random(0, 5);

  int target = eduAngles[idx];
  gotoAS5600Target((float)target);

  active = true;
  holding = false;
}

void moveServoRandom() {
  for (int tries = 0; tries < 20; tries++) {
    int idx = getSmartRandomGameIndex();
    int targetAS = (idx < 0) ? SERVO_UNUSED : servoPositions[idx];
    if (targetAS == SERVO_UNUSED) continue;
    gotoAS5600Target((float)targetAS);

    Serial.print("Normal/Infinite idx=");
    Serial.print(idx);
    Serial.print(" AS target=");
    Serial.println(targetAS);
    return;
  }
  stopHold();
  smoothMove(servoCenterAngle);
}
void smoothMove(int targetAngle) {
  int durationMs = 100;
  targetAngle = constrain(targetAngle, 0, 180);

  int startAngle = constrain(lastAngle, 0, 180);
  if (startAngle == targetAngle) {
    servoPos = targetAngle;  // keep in sync
    lastAngle = targetAngle;
    return;
  }

  const int frameMs = 20;
  int steps = max(1, durationMs / frameMs);

  for (int i = 1; i <= steps; i++) {
    handleButtonInterrupt();
    float t = (float)i / (float)steps;
    float eased = 0.5f - 0.5f * cosf(t * PI);
    int pos = (int)roundf(startAngle + (targetAngle - startAngle) * eased);

    myServo.write(pos);
    servoPos = pos;  // ✅ keep sync for hold/takeup
    delay(frameMs);
    yield();
  }

  myServo.write(targetAngle);
  servoPos = targetAngle;  // ✅
  lastAngle = targetAngle;
}
void fastMove(int ang) {
  ang = constrain(ang, 0, 180);
  myServo.write(ang);

  servoPos = ang;   // ✅ CRITICAL: keep servoPos in sync
  lastAngle = ang;  // keep smoothMove correct later
}






//settings
void drawSettingsIcon(bool selected) {
  int x = SCREEN_WIDTH - 17;
  int y = SCREEN_HEIGHT - 17;

  if (selected) {
    display.fillRect(x, y, 16, 16, SSD1306_WHITE);
    display.drawBitmap(x, y, settingselected, 16, 16, SSD1306_BLACK);
  } else {
    display.fillRect(x, y, 16, 16, SSD1306_WHITE);
    display.drawBitmap(x, y, settingsIcon, 16, 16, SSD1306_BLACK);
  }
}
void displaySettingsMenu() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Title
  display.setCursor(getCenteredX("SETTINGS", 1), 0);
  display.println("SETTINGS");

  // Back arrow
  drawBackArrow(settingSelect == 0);

  int startY = 16;
  int visibleRows = 4;

  // Total selectable items (10 angles + Reset All)
  int totalItems = servoPosCount + 2;  // +1 = Reset All

  for (int i = 0; i < visibleRows; i++) {
    int idx = settingsScroll + i;
    if (idx >= totalItems) break;

    int y = startY + i * 10;

    // Highlight selector
    if (settingSelect == idx + 1) {
      display.setCursor(5, y);
      display.print(">");
    }

    display.setCursor(15, y);

    // ---- Servo angle rows ----
    // ---- Servo angle rows ----
    if (idx < servoPosCount) {
      display.print("Angle ");
      display.print(idx + 1);
      display.print(": ");

      int shownVal = servoPositions[idx];

      // if editing this row, show the temp value instead
      if (editingServoAngle && idx == editIdx) shownVal = editTemp;

      if (shownVal == SERVO_UNUSED) display.println("nan");
      else display.println(shownVal);
    } else if (idx == servoPosCount) {
      display.print("Target: ");
      display.println(TARGET_DEG, 1);  // 1 dp (change to 0 if you want)
    }

    // ---- Reset All row ----
    else {
      display.println("Reset All");
    }
  }

  display.display();
}
void saveServoPositions() {
  prefs.begin("settings", false);

  // Always save 10 slots to keep size consistent
  prefs.putInt("servoCount", MAX_SERVO_POS);
  prefs.putBytes("servoPos", servoPositions, MAX_SERVO_POS * sizeof(int));
  prefs.putFloat("targetDeg", TARGET_DEG);



  prefs.end();

  Serial.println("Saved servo positions (10 slots)");
}
void loadServoPositions() {
  prefs.begin("settings", false);

  int storedCount = prefs.getInt("servoCount", 0);
  size_t storedSize = prefs.getBytesLength("servoPos");

  // Default everything to unused first
  for (int i = 0; i < MAX_SERVO_POS; i++) {
    servoPositions[i] = SERVO_UNUSED;
  }

  // Accept only if it matches exactly 10 slots
  if (storedCount == MAX_SERVO_POS && storedSize == (MAX_SERVO_POS * sizeof(int))) {

    prefs.getBytes("servoPos", servoPositions, storedSize);
    Serial.println("Servo positions loaded (10 slots)");

  } else {
    Serial.println("No valid saved data → using defaults");

    int defaults[MAX_SERVO_POS] = {
      92, 164, 125, 68, 47, 30, 17,
      SERVO_UNUSED, SERVO_UNUSED, SERVO_UNUSED
    };

    memcpy(servoPositions, defaults, MAX_SERVO_POS * sizeof(int));
  }
  TARGET_DEG = prefs.getFloat("targetDeg", 41.0f);


  // safety clamp
  TARGET_DEG = constrain(TARGET_DEG, 0.0f, 359.0f);


  prefs.end();

  // Always 10 slots in menu
  servoPosCount = MAX_SERVO_POS;
}
void previewSelectedSettingAngle() {
  if (!inSettingsMenu) return;
  if (settingSelect < 1 || settingSelect > servoPosCount) return;

  int idx = settingSelect - 1;
  int targetAS = servoPositions[idx];

  if (targetAS == SERVO_UNUSED) {
    stopHold();
    smoothMove(servoCenterAngle);
    return;
  }

  gotoAS5600Target((float)targetAS);
}





void gotoAS5600Target(float targetDeg) {
  targetDeg = wrap360(targetDeg);
  active = true;
  holding = false;
  targetSensorDeg = targetDeg;

  uint32_t start = millis();
  uint32_t lastStepMs = 0;
  int lastDir = 0;
  uint32_t pauseUntil = 0;

  const uint32_t TIMEOUT_MS = 1800;   // was 5000
  const uint32_t FLIP_PAUSE_MS = 80;  // was 300

  while (millis() - start < TIMEOUT_MS) {
    handleButtonInterrupt();
    float cur = readAS5600Deg1();

    // if I2C read failed, don't "freeze" here
    if (isnan(cur)) {
      yield();
      continue;
    }

    float err = shortestError(targetDeg, cur);
    float ae = fabsf(err);

    if (ae <= HOLD_TOL_IN) break;

    int dir = (err > 0) ? +1 : -1;
    if (invertFeedbackErr) dir = -dir;

    // shorter pause on direction change
    if (lastDir != 0 && dir != lastDir && pauseUntil == 0) {
      pauseUntil = millis() + FLIP_PAUSE_MS;
    }
    if (pauseUntil != 0) {
      if (millis() < pauseUntil) {
        yield();
        continue;
      }
      pauseUntil = 0;
    }

    // more aggressive when far
    uint32_t interval;
    int step;
    if (ae > 60) {
      interval = 15;
      step = 4;
    } else if (ae > 25) {
      interval = 20;
      step = 3;
    } else if (ae > 10) {
      interval = 35;
      step = 2;
    } else {
      interval = 70;
      step = 1;
    }

    if (millis() - lastStepMs >= interval) {
      servoWriteSafe(servoPos + dir * step);
      lastStepMs = millis();
      lastDir = dir;
    }

    yield();
  }

  float cur2 = readAS5600Deg1();
  Serial.print("[AS GOTO] target=");
  Serial.print(targetDeg, 2);
  Serial.print(" cur=");
  Serial.print(cur2, 2);
  Serial.print(" err=");
  Serial.println(shortestError(targetDeg, cur2), 2);
}

uint16_t readAS5600Raw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);  // 0x0C
  if (Wire.endTransmission(false) != 0) return 0xFFFF;

  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return 0xFFFF;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  // 12-bit raw angle
  return ((uint16_t)(msb & 0x0F) << 8) | lsb;
}

float readAS5600Deg1() {  //i2c
  uint16_t raw = readAS5600Raw();
  if (raw == 0xFFFF) return NAN;  // fail

  float deg = (raw * 360.0f) / 4096.0f;
  return wrap360(deg);
}

bool inWindowDeg(int a, int target, int tol) {
  int d = abs(a - target);
  d = min(d, 360 - d);
  return d <= tol;
}
void handleAS5600Trigger() {
  if (!irArmed) return;
  int deg = readAS5600Deg();
  if (deg < 0) return;
  static unsigned long inRangeSince = 0;
  if (millis() - armStart > 1500) {
    irArmed = false;
    inRangeSince = 0;
    Serial.println("AS5600 timeout → returning to waiting");
    return;
  }
  if (inWindowDeg(deg, (int)lroundf(TARGET_DEG), TRIGGER_TOL)) {
    if (inRangeSince == 0) inRangeSince = millis();
    if (millis() - inRangeSince >= STABLE_MS) {
      irArmed = false;
      inRangeSince = 0;
      Serial.println("AS5600 trigger → moving to next level");
      moveServoRandom();
      moveCount++;
      lastMoveTime = millis();
      if (menuIndex == 1 && moveCount >= 10) {
        totalTime = millis() - startTime - pausedTime;
        detecting = false;
        showResult = true;
      }
      if (menuIndex == 2) { maxDelay = max((unsigned long)(maxDelay * difficultyFactor), minDelay); }
    }
  } else {
    inRangeSince = 0;
  }
}

void autoBalanceNearOnly() {
  handleButtonInterrupt();

  float cur = readAS5600Deg();
  float err = shortestError(TARGET_DEG, cur);
  float ae = fabsf(err);

  // ✅ REMOVE this line (caller already gated assist mode)
  // if (ae > assist) return;

  // deadband
  if (ae <= TOL1 + 0.5f) return;

  int desiredDir = (err > 0) ? +1 : -1;
  unsigned long now = millis();

  if (desiredDir != 0 && lastMoveDir != 0 && desiredDir != lastMoveDir) {
    if (pauseUntilMs == 0) pauseUntilMs = now + DIR_PAUSE_MS;
  }

  bool inDirPause = (pauseUntilMs != 0 && now < pauseUntilMs);
  if (pauseUntilMs != 0 && now >= pauseUntilMs) pauseUntilMs = 0;

  const unsigned long moveInterval = SUPER_SLOW_MS;

  if (!inDirPause && (now - lastMoveMs >= moveInterval)) {
    servoPos += desiredDir * SERVO_STEP;
    servoPos = constrain(servoPos, SERVO_MIN, SERVO_MAX);
    myServo.write(servoPos);

    lastMoveMs = now;
    lastMoveDir = desiredDir;
  }
}



void holdLoopStep() {
  if (!active) return;

  // ---- pick ONE "gate" sensor for deciding assist vs hold ----
  float cur_gate = readAS5600Deg();  // OUT filtered
  float err_gate = shortestError(TARGET_DEG, cur_gate);
  float aerr_gate = fabsf(err_gate);

  uint32_t now = millis();

  // ---- hysteresis + debounce ----
  if (!inAssistMode) {
    if (aerr_gate <= ASSIST_ENTER) {
      if (modeChangeAt == 0) modeChangeAt = now;
      if (now - modeChangeAt >= MODE_DEBOUNCE_MS) {
        inAssistMode = true;
        modeChangeAt = 0;
      }
    } else {
      modeChangeAt = 0;
    }
  } else {
    if (aerr_gate >= ASSIST_EXIT) {
      if (modeChangeAt == 0) modeChangeAt = now;
      if (now - modeChangeAt >= MODE_DEBOUNCE_MS) {
        inAssistMode = false;
        modeChangeAt = 0;
      }
    } else {
      modeChangeAt = 0;
    }
  }

  // ---------- HOLD sensor (I2C) ----------
  float cur_hold = readAS5600Deg1();  // I2C
  float err_hold = shortestError(targetSensorDeg, cur_hold);
  float aerr_hold = fabsf(err_hold);

  // ---------- Serial debug (rate-limited) ----------
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 150) {
    lastPrint = now;

    Serial.print("[HOLD] GateCur=");
    Serial.print(cur_gate, 2);
    Serial.print(" GateErr=");
    Serial.print(err_gate, 2);
    Serial.print(" | Mode=");
    Serial.print(inAssistMode ? "ASSIST" : "HOLD");

    Serial.print(" || I2CCur=");
    Serial.print(cur_hold, 2);
    Serial.print(" Target=");
    Serial.print(targetSensorDeg, 2);
    Serial.print(" I2CErr=");
    Serial.print(err_hold, 2);

    Serial.print(" | Servo=");
    Serial.print(servoPos);

    Serial.print(" | holding=");
    Serial.println(holding ? "1" : "0");
  }

  // ---------- Assist mode ----------
  if (inAssistMode) {
    holding = false;
    autoBalanceNearOnly();  // gentle near-target
    return;
  }

  // ---------- HOLD logic ----------
  if (holding) {
    if (aerr_hold < HOLD_TOL_OUT) return;
    holding = false;
  }

  if (!holding && aerr_hold <= HOLD_TOL_IN) {
    holding = true;
    return;
  }

  int dir = (err_hold > 0) ? +1 : -1;
  if (invertFeedbackErr) dir = -dir;

  int step = (aerr_hold > 10.0f) ? HOLD_STEP_FAR : HOLD_STEP_NEAR;
  servoWriteSafe(servoPos + dir * step);
}

float wrap360(float x) {
  while (x < 0.0f) x += 360.0f;
  while (x >= 360.0f) x -= 360.0f;
  return x;
}
void stopHold() {
  active = false;
  holding = false;
}


static float shortestError(float target, float current) {
  float e = target - current;
  while (e > 180.0f) e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return e;
}
// ----- helper: median of 5 -----
static int median5(int a, int b, int c, int d, int e) {
  int v[5] = { a, b, c, d, e };
  // simple sort
  for (int i = 0; i < 5; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (v[j] < v[i]) {
        int t = v[i];
        v[i] = v[j];
        v[j] = t;
      }
    }
  }
  return v[2];
}


static float emaAngle(float prev, float cur, float alpha) {
  if (isnan(prev)) return cur;

  float diff = cur - prev;
  // wrap diff to [-180, +180]
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;

  float blended = prev + alpha * diff;
  return wrap360(blended);
}

static float rawToDeg(float raw) {
  float r = raw;
  if (r < adcMin) r = adcMin;
  if (r > adcMax) r = adcMax;

  float norm = (r - adcMin) / (adcMax - adcMin);  // 0..1
  float deg = norm * 360.0f;
  return wrap360(deg);
}

static float readAdcTrimmedMean() {
  int vals[NSAMPLES];

  for (int i = 0; i < NSAMPLES; i++) {
    vals[i] = analogRead(AS5600_OUT_PIN);
    delayMicroseconds(800);  // small gap between samples
  }

  // simple sort (NSAMPLES is small)
  for (int i = 0; i < NSAMPLES - 1; i++) {
    for (int j = i + 1; j < NSAMPLES; j++) {
      if (vals[j] < vals[i]) {
        int t = vals[i];
        vals[i] = vals[j];
        vals[j] = t;
      }
    }
  }

  long sum = 0;
  int count = 0;
  for (int i = TRIM; i < NSAMPLES - TRIM; i++) {
    sum += vals[i];
    count++;
  }
  return (float)sum / (float)count;
}

float readAS5600Deg() {  //out
  float adc = readAdcTrimmedMean();
  float deg = rawToDeg(adc);

  // EMA on circular angle
  emaDeg = emaAngle(emaDeg, deg, EMA_ALPHA);

  // Optional deadband on output (also circular shortest diff)
  float out = emaDeg;
  if (!isnan(lastOutDeg) && DEADBAND_DEG > 0) {
    float diff = out - lastOutDeg;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    if (fabsf(diff) < DEADBAND_DEG) {
      out = lastOutDeg;
    } else {
      lastOutDeg = out;
    }
  } else {
    lastOutDeg = out;
  }

  return out;
}


int signi(int x) {
  return (x > 0) ? +1 : (x < 0) ? -1
                                : 0;
}
void gotoServoAndLockTarget(int targetServo) {
  targetServo = constrain(targetServo, 0, 180);

  if (targetServo == servoPos) {
    targetSensorDeg = readAS5600Deg1();
    return;
  }

  int rawDir = signi(targetServo - servoPos);
  int dir = rawDir;
  if (invertServoDir) dir = -dir;

  float before = readAS5600Deg1();

  static int lastDir = 0;  // ✅ FIXED
  bool dirFlipped = (lastDir != 0 && dir != lastDir);
  lastDir = dir;

  if (dirFlipped) {
    int overshoot = constrain(targetServo + dir * OVERSHOOT_DEG, 0, 180);
    servoWriteSafe(overshoot);
    delay(SETTLE_MS);
  }

  smoothMove(targetServo);
  delay(SETTLE_MS);

  float after = readAS5600Deg1();
  float moved = fabsf(after - before);
  moved = min(moved, 360.0f - moved);

  int takeup = 0;
  while (moved < MOVE_DEADBAND_DEG && takeup < MAX_TAKEUP_STEPS) {
    servoWriteSafe(servoPos + dir * 1);
    delay(SETTLE_MS);

    float now = readAS5600Deg1();
    float dm = fabsf(now - before);
    dm = min(dm, 360.0f - dm);
    moved = dm;
    takeup++;
  }

  servoWriteSafe(targetServo);
  delay(SETTLE_MS);

  targetSensorDeg = readAS5600Deg1();
}
static int getCurASInt() {
  float cur = readAS5600Deg1();  // I2C angle 0..360
  if (!isfinite(cur)) return 0;
  int v = (int)lroundf(cur);
  if (v < 0) v = 0;
  if (v > 359) v = 359;
  return v;
}
void servoWriteSafe(int pos) {
  pos = constrain(pos, SERVO_MIN, SERVO_MAX);
  servoPos = pos;
  lastAngle = pos;  // ✅ keep smoothMove in sync
  myServo.write(pos);
}
