#include <SPI.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <avr/pgmspace.h>
#include <Encoder.h>

// --- pins ---

#define CS_PIN 10
#define DC_PIN 9
#define RST_PIN 8

// #define ENC_CLK 7
// #define ENC_DT A5
// #define ENC_SW 12

// --- persistence store ---

#define EEPROM_VERSION 1

struct Persist {
  byte version;
  unsigned long circuitLoopMs;
  //bool calibrateAtStartup;
  // unsigned long snoozingMinutes;
  // long stationCenterOffset;
};

// --- display driver ---
U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, 10, 9, 8);

// -------- pins --------
const int IR_PIN = A0;
const int IR_THRESHOLD = 650;

const int in1Pin = 5;
const int in2Pin = 6;
//const int in1Pin = 7;
//const int in2Pin = A2;

const int CLK_PIN = 2;  // INT0
const int DT_PIN = 3;   // INT1
const int buttonPin = 12;
long lastPos = 0;

// const int RED_PIN = 2;
// const int YEL_PIN = 3;
const int RED_PIN = 7;
const int YEL_PIN = A5;
const int GRN_PIN = 4;

const int STN1_PIN = A1;
const int STN2_PIN = A2;
const int STN3_PIN = A3;
const int STN4_PIN = A4;

// -------- tuning ---------
const int MAX_SPEED = 255;
const int DOCKING_SPEED = 165;
const int RAMP_STEP = 10;
const float MAX_MPH = 74.0;

// ----- station stop -------
unsigned long circuitLoopMs = 0;

long trailingEdgeOffset = 1990;
long stationCenterOffset = 780;

// ------- station ---------
bool sensorEnabled = true;
bool calibrateAtStartup = true;
volatile bool calibrating = false;
bool hasCalibrated = false;
bool stationArmed = false;
unsigned long stationTick = 0;
unsigned long snoozingMinutes = 5;  // 20 and Never

// ----- dip behavior -----
const int DIP_SPEED = MAX_SPEED * 3.6 / 10;       // 25MPH
const int DIP_SPEED_FAST = MAX_SPEED * 4.9 / 10;  // 35MPH
const unsigned long DIP_TIME = 3600;              // ms per dip

// -------- display --------
bool isMPH = true;
bool lastDirection = true;
int globalCurrentSpeed = 0;
volatile bool abortRoute = false;
volatile bool restartRequested = false;
uint8_t currentRouteIndex = 0;

char line1[64] = "STATUS";
char line2[64] = "0";
char line3[64] = "READY";

enum StationState {
  CRUISING,
  ARRIVING,
  AT_STATION,
  DEPARTING,
  COOL_DOWN
};

StationState currentStationState = CRUISING;
unsigned long stateStartTime = 0;
bool manualLocked = false;
bool trippedDirection = false;

int currentSpeedNumber(int pwm) {
  if (isMPH) {
    return speedToMph(pwm);
  }
  return speedToKph(pwm);
}

char* perHourName() {
  if (isMPH) {
    return "MPH";
  }
  return "KPH";
}

int speedToMph(int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  return (pwm * MAX_MPH) / MAX_SPEED;
}

int speedToKph(int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  return (pwm * MAX_MPH * 161) / (MAX_SPEED * 100);
}

Encoder speedKnob(CLK_PIN, DT_PIN);

volatile int targetSpeed = 0;
volatile bool targetDirty = false;
enum ControlMode { AUTO,
                   MANUAL };
ControlMode mode = AUTO;
unsigned long lastManualInput = 0;

void readEncoderStep() {
  long pos = speedKnob.read() / 4;

  // --- KNOB ENTERS MANUAL IF AUTO ---
  if (pos != lastPos) {
    lastManualInput = millis();

    if (mode == AUTO) {
      mode = MANUAL;
      restartRequested = true;
      abortRoute = true;
      stationArmed = false;
      Serial.println("REQUEST MANUAL (knob)");
    }

    lastPos = pos;
  }

  // --- BUTTON TOGGLES LOCK ---
  static unsigned long lastPress = 0;

  if (digitalRead(buttonPin) == LOW) {
    if (millis() - lastPress > 250) {
      lastPress = millis();

      // If in manual, toggle lock state
      if (mode == MANUAL) {
        if (manualLocked) {
          manualLocked = false;
          Serial.println("MANUAL LOCK OFF");
        } else {
          manualLocked = true;
          Serial.println("MANUAL LOCK ON");
        }
      }
    }
  }
}
static long delta = 0;

void manualControlLoop() {
  Serial.println("ENTER MANUAL");
  snprintf(line3, sizeof(line3), "MANUAL CONTROL");

  static unsigned long snoozeTime = 0;

  while (true) {
    updateStationLights();
    long pos = speedKnob.read() / 4;

    if (globalCurrentSpeed == 0) {
      if (millis() - snoozeTime < (snoozingMinutes * 60 * 1000)) {
        if (delta < 0) {
          if (globalCurrentSpeed == 0) {
            signalRed();
          } else {
            signalYellow();
          }
        } else if (delta > 0) {
          signalGreen();
        } else {
          signalRed();
        }
      } else {
        signalOff();
        stationLightsOff();
      }
    }

    if (currentStationState == CRUISING) {
      if (globalCurrentSpeed == 0) {
        setStationState(AT_STATION);
      } else if (delta < 0 && globalCurrentSpeed < DOCKING_SPEED) {
        setStationState(ARRIVING);
      }
    } else if (currentStationState == AT_STATION) {
      if (delta > 0 && globalCurrentSpeed > 0) {
        setStationState(DEPARTING);
      }
    }

    if (pos != lastPos) {
      long oldPos = lastPos;
      lastPos = pos;
      int prevSpeed = globalCurrentSpeed;
      delta = oldPos - pos;
      int magnitude = abs(delta);

      int step = 2;
      if (magnitude > 1) step = 11;
      if (magnitude > 2) step = 17;
      if (magnitude > 3) step = 26;
      if (magnitude > 4) step = 31;
      if (magnitude > 5) step = 38;

      if (delta != 0) {
        globalCurrentSpeed += (delta > 0 ? step : -step);
      }

      globalCurrentSpeed = constrain(globalCurrentSpeed, 0, 255);

      if (globalCurrentSpeed == 0 && prevSpeed > 0) {
        lastDirection = !lastDirection;
        snoozeTime = millis();
      }

      if (globalCurrentSpeed > 0) {
        snoozeTime = 0;
      }

      writeMotor(lastDirection, globalCurrentSpeed);
      snprintf(line2, sizeof(line2), "%d %s", speedToMph(globalCurrentSpeed), perHourName());
      draw();
    }

    static unsigned long lastPress = 0;
    if (digitalRead(buttonPin) == LOW) {
      if (millis() - lastPress > 250) {
        lastPress = millis();
        snprintf(line3, sizeof(line3), "EXIT MANUAL");
        draw();
        Serial.println("EXIT MANUAL");
        break;
      }
    }
  }
  mode = AUTO;
  restartRequested = true;
  abortRoute = false;
  stationArmed = false;
}

// -------- go! --------
bool currentDirection = true;

void go(bool forward, int speed, unsigned long runTime, int dipCount) {
  Serial.println("GO!");
  if (abortRoute) return;

  unsigned long pauseTime = random(6, 20);
  setDirection(forward);
  rampSpeed(speed);

  if (dipCount > 0) {
    unsigned long segment = (runTime * 1000) / (dipCount + 1);
    for (int i = 0; i < dipCount; i++) {
      if (abortRoute) return;
      Serial.print("🟢 FAST LEG ⏱ ");
      Serial.print(segment / 1000);
      Serial.println("s");
      const char* msg;
      unsigned long legStartTime = millis();
      while (millis() - legStartTime < segment) {
        if (abortRoute) return;

        snprintf(line3, sizeof(line3), "%s %ds", "FAST LEG", segment / 1000);
        draw();
        updateStationLights();
        readEncoderStep();
      }
      rampSpeed(random(DIP_SPEED, DIP_SPEED_FAST));
      Serial.print("🟡 SLOW LEG ⏱ ");
      Serial.print(DIP_TIME / 1000);
      Serial.println("s");
      unsigned long dipStartTime = millis();
      while (millis() - dipStartTime < DIP_TIME) {
        if (abortRoute) return;

        snprintf(line3, sizeof(line3), "%s %ds", "SLOW LEG", DIP_TIME / 1000);
        draw();
        updateStationLights();
        readEncoderStep();
      }
      rampSpeed(random(speed * 0.85, speed));
    }
    Serial.print("🟢 FAST LEG ⏱ ");
    Serial.print(segment / 1000);
    Serial.println("s");
    unsigned long segmentStartTime = millis();
    while (millis() - segmentStartTime < segment) {
      if (abortRoute) return;

      snprintf(line3, sizeof(line3), "%s %ds", "FAST LEG", segment / 1000);
      draw();
      updateStationLights();
      readEncoderStep();
    }
  } else {
    Serial.print("🟢 ONLY LEG ⏱ ");
    Serial.print(runTime);
    Serial.println("s");
    unsigned long onlyStartTime = millis();
    while (millis() - onlyStartTime < runTime * 1000) {
      if (abortRoute) return;

      snprintf(line3, sizeof(line3), "%s %ds", "ONLY LEG", runTime);
      draw();
      updateStationLights();
      readEncoderStep();
    }
  }
  Serial.print("🛑 STOP ⏱ ");
  Serial.print(pauseTime);
  Serial.println("s");
  readEncoderStep();

  rampSpeed(0);
  stateStartTime = millis();
  unsigned long pauseMs = pauseTime * 1000;
  while (millis() - stateStartTime < pauseMs) {
    if (abortRoute) return;
    snprintf(line3, sizeof(line3), "%s %ds", "AT STATION", pauseTime);
    draw();
    updateStationLights();
    readEncoderStep();
  }
  setStationState(DEPARTING);
  updateStationLights();
  unsigned long start = millis();
  while (millis() - start < 4000) {
    if (abortRoute) return;
    updateStationLights();
    snprintf(line3, sizeof(line3), "%s", "NOW BOARDING");
    draw();
    readEncoderStep();
  }
}

// -------- signal --------
void signalOff() {
  digitalWrite(YEL_PIN, LOW);
  digitalWrite(GRN_PIN, LOW);
  digitalWrite(RED_PIN, LOW);
}

void signalRed() {
  digitalWrite(YEL_PIN, LOW);
  digitalWrite(GRN_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);
}

void signalYellow() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GRN_PIN, LOW);
  digitalWrite(YEL_PIN, HIGH);
}

void signalGreen() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);
  digitalWrite(GRN_PIN, HIGH);
}

void updateTafficSignal(int speed, bool rampUp) {
  if (speed == 0) {
    signalRed();
    return;
  }
  if (rampUp) {
    signalGreen();
  } else {
    signalYellow();
  }
}

// -------- lights --------

const unsigned long ARRIVE_BLINK_MS = 3000;
const unsigned long DEPART_BLINK_MS = 4000;
const unsigned long HOLD_AFTER_LEAVE = 3000;
const unsigned long FADE_MS = 4000;

void stationLightsOff() {
  digitalWrite(STN1_PIN, LOW);
  digitalWrite(STN2_PIN, LOW);
  digitalWrite(STN3_PIN, LOW);
  digitalWrite(STN4_PIN, LOW);
}

void stationLightsOn() {
  digitalWrite(STN1_PIN, HIGH);
  digitalWrite(STN2_PIN, HIGH);
  digitalWrite(STN3_PIN, HIGH);
  digitalWrite(STN4_PIN, HIGH);
}

void fadeToBlackMs(unsigned long ms) {
  static unsigned long start = 0;
  static int phase = 0;

  if (phase == 0) {
    start = millis();
    phase = 1;
  }

  unsigned long step = ms / 4;
  unsigned long elapsed = millis() - start;

  if (phase == 1 && elapsed >= step) {
    digitalWrite(STN1_PIN, LOW);
    phase = 2;
  }
  if (phase == 2 && elapsed >= step * 2) {
    digitalWrite(STN2_PIN, LOW);
    phase = 3;
  }
  if (phase == 3 && elapsed >= step * 3) {
    digitalWrite(STN3_PIN, LOW);
    phase = 4;
  }
  if (phase == 4 && elapsed >= step * 6) {
    digitalWrite(STN4_PIN, LOW);
    phase = 0;  // finished
  }
}

void alternateBlink(unsigned long now) {
  static unsigned long lastToggle = 0;
  static bool phase = false;

  if (now - lastToggle >= 250) {
    lastToggle = now;
    phase = !phase;
    //Serial.print("BLINKING! ");

    if (phase) {
      digitalWrite(STN1_PIN, LOW);
      digitalWrite(STN2_PIN, HIGH);
      digitalWrite(STN3_PIN, HIGH);
      digitalWrite(STN4_PIN, HIGH);
    } else {
      digitalWrite(STN1_PIN, HIGH);
      digitalWrite(STN2_PIN, HIGH);
      digitalWrite(STN3_PIN, HIGH);
      digitalWrite(STN4_PIN, LOW);
    }
  }
}

void setStationState(StationState s) {
  if (currentStationState == s) return;
  currentStationState = s;
  stateStartTime = millis();
}

void updateStationLights() {
  unsigned long elapsed = millis() - stateStartTime;

  switch (currentStationState) {
    case ARRIVING:
      alternateBlink(millis());
      if (elapsed >= ARRIVE_BLINK_MS)
        setStationState(AT_STATION);
      break;

    case DEPARTING:
      alternateBlink(millis());
      if (elapsed >= DEPART_BLINK_MS)
        setStationState(COOL_DOWN);
      break;

    case COOL_DOWN:
      static bool fading = false;

      if (!fading) {
        stationLightsOn();
        if (elapsed >= HOLD_AFTER_LEAVE) {
          fading = true;
          stateStartTime = millis();
        }
      } else {
        fadeToBlackMs(FADE_MS);
        if (millis() - stateStartTime >= FADE_MS) {
          fading = false;
          setStationState(CRUISING);
        }
      }
      break;
    case AT_STATION:
      stationLightsOn();  // Solid lights while stopped
      break;

    case CRUISING:
      // All station pins LOW
      digitalWrite(STN1_PIN, LOW);
      digitalWrite(STN2_PIN, LOW);
      digitalWrite(STN3_PIN, LOW);
      digitalWrite(STN4_PIN, LOW);
      break;
  }
}

// -------- motor --------
void writeMotor(bool forward, int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  globalCurrentSpeed = pwm;
  currentDirection = forward;

  if (forward) {
    digitalWrite(in2Pin, LOW);  // This stays Digital (The Ground)
    analogWrite(in1Pin, pwm);   // This uses PWM (The Speed)
  } else {
    digitalWrite(in1Pin, LOW);  // This stays Digital (The Ground)
    analogWrite(in2Pin, pwm);   // This uses PWM (The Speed)
  }
}

// -------- direction --------
void setDirection(bool forward) {
  lastDirection = forward;
}

// -------- ramp --------
void rampSpeed(int target) {
  static int current = 0;
  if (current != globalCurrentSpeed) current = globalCurrentSpeed;

  static bool lastSensorState = false;
  static bool dockedThisStop = false;
  int start = current;
  int delta = abs(target - start);
  bool rampUp = target > current;
  trippedDirection = (globalCurrentSpeed > 0);

  if (target == 0) stationArmed = false;

  if (target != 0) dockedThisStop = false;

  if (target == current) {
    globalCurrentSpeed = current;
    return;
  }

  updateTafficSignal((rampUp ? 1 : current), rampUp);

  while (current != target) {
    if (abortRoute) return;
    if (target == 0 && current < (DOCKING_SPEED - 10)) {
      setStationState(ARRIVING);
    }
    if (sensorEnabled && target == 0 && !dockedThisStop) {
      if (current <= DOCKING_SPEED) {
        Serial.println("WAITING FOR STATION EDGE");
        snprintf(line3, sizeof(line3), "%s", "BRAKE TO HALT");
        draw();
        while (!stationArmed) {
          if (abortRoute) return;
          int v = analogRead(IR_PIN);
          if (v < IR_THRESHOLD) {
            stationArmed = true;
            stationTick = millis();
          }
          updateStationLights();
          readEncoderStep();
        }

        unsigned long waitMs = calculateStationPause(lastDirection);
        unsigned long startWait = millis();
        while (millis() - startWait < waitMs) {
          if (abortRoute) return;
          updateStationLights();
          draw();
          readEncoderStep();
        }
        setStationState(AT_STATION);
        dockedThisStop = true;
        Serial.println("DOCKING INITIATED");
      }
    }
    // ---- S-CURVE RAMP ----
    int progressed = abs(current - start);
    float phase = (delta == 0) ? 1.0 : (float)progressed / delta;
    int step = max(1, (int)(RAMP_STEP * (0.5 + 1.5 * phase * (1 - phase))));
    if (rampUp) {
      current += step;
      if (current > target) current = target;
    } else {
      current -= step;
      if (current < target) current = target;
    }
    // ---- OUTPUT ----
    globalCurrentSpeed = current;
    if (target == 0) {
      updateTafficSignal(current, rampUp);
    } else {
      snprintf(line3, sizeof(line3), "%s %d %s",
               rampUp ? "RAMP TO" : "DOWN TO",
               speedToMph(target),
               perHourName());
    }
    if (targetDirty) {
      targetDirty = false;
      target = targetSpeed;
      start = current;
      delta = abs(target - start);
      rampUp = target > current;
    }
    writeMotor(lastDirection, current);
    snprintf(line2, sizeof(line2), "%d %s", speedToMph(current), perHourName());
    draw();
    updateStationLights();
    readEncoderStep();
  }
  updateTafficSignal(current, rampUp);
}

// -------- calibrate --------
// Calibration function
void calibrateTrain() {
  unsigned long lapFwd = 0;
  snprintf(line1, sizeof(line1), "CALIBRATION");
  draw();
  unsigned long lapRev = measureLap(false);

  Persist p;
  p.version = EEPROM_VERSION;
  p.circuitLoopMs = lapRev;
  //p.calibrateAtStartup = calibrateAtStartup;
  // p.snoozingMinutes = snoozingMinutes;

  EEPROM.put(0, p);  // Write to EEPROM

  // Log results
  Serial.print("REV Loop Time: ");
  Serial.println(p.circuitLoopMs);
}

void loadFromEEPROM() {
  Persist p;
  EEPROM.get(0, p);

  if (p.version == EEPROM_VERSION) {
    circuitLoopMs = p.circuitLoopMs;
    //calibrateAtStartup = p.calibrateAtStartup;
    // snoozingMinutes = p.snoozingMinutes;
  }
}

unsigned long calculateStationPause(bool forward) {
  if (forward) {
    return stationCenterOffset;
  } else {
    return ((circuitLoopMs * 0.5) + trailingEdgeOffset - stationCenterOffset);
  }
}
// Function to measure lap time
unsigned long measureLap(bool forward) {
  unsigned long start = 0;
  int read = 0;
  snprintf(line3, sizeof(line3), "RAMP");
  draw();

  setDirection(forward);
  for (int s = 0; s <= DOCKING_SPEED; s += 6) {
    globalCurrentSpeed = s;
    snprintf(line2, sizeof(line2), "%d %s", speedToMph(globalCurrentSpeed), perHourName());

    draw();
    writeMotor(forward, s);
  }
  bool lastState = analogRead(IR_PIN) < IR_THRESHOLD;
  snprintf(line3, sizeof(line3), "TIMING");
  draw();

  while (true) {
    if (abortRoute) {
      return 0;
    }
    read = analogRead(IR_PIN);
    bool state = read < IR_THRESHOLD;
    if (!lastState && state) break;
    lastState = state;
  }
  snprintf(line3, sizeof(line3), "MARK");
  draw();

  delay(1000);

  start = millis();
  while (true) {
    read = analogRead(IR_PIN);
    bool state = read < IR_THRESHOLD;
    if (lastState && !state) break;
    lastState = state;
  }

  snprintf(line3, sizeof(line3), "SET");
  draw();

  unsigned long lap = millis() - start;
  for (int s = DOCKING_SPEED; s >= 0; s -= 6) {
    snprintf(line2, sizeof(line2), "%d %s", speedToMph(globalCurrentSpeed), perHourName());

    draw();
    writeMotor(forward, s);
    globalCurrentSpeed = s;
  }
  writeMotor(forward, 0);
  globalCurrentSpeed = 0;
  
  return lap;
}

// Save calibration values to EEPROM
void saveToEEPROM() {
  Persist p;
  p.version = EEPROM_VERSION;
  p.circuitLoopMs = circuitLoopMs;
  //p.calibrateAtStartup = calibrateAtStartup;
  // p.stationCenterOffset = stationCenterOffset;

  EEPROM.put(0, p);  // Write the calibration data to EEPROM
}

const char l0[] PROGMEM = "Pennsylvania Line";
const char l1[] PROGMEM = "Hogwarts Express";
const char l2[] PROGMEM = "California Zephyr";
const char l3[] PROGMEM = "Reading Railroad";
const char l4[] PROGMEM = "The Polar Express";
const char l5[] PROGMEM = "Union Pacific R.R.";
const char l6[] PROGMEM = "The Orient Express";
const char l7[] PROGMEM = "Broadway Limited";
const char l8[] PROGMEM = "The Silver Streak";
const char l9[] PROGMEM = "The B&O Railroad";
const char l10[] PROGMEM = "The Flying Rocket";
const char l11[] PROGMEM = "Grand Central Line";
const char l12[] PROGMEM = "Flying Scotsman";
const char l13[] PROGMEM = "Cannonball Express";
const char l14[] PROGMEM = "The Blue Comet";
const char l15[] PROGMEM = "Taking Pelham 123";
const char l16[] PROGMEM = "Vanderbilt Central";
const char l17[] PROGMEM = "Broadway Local";
const char l18[] PROGMEM = "The Circle Line";
const char l19[] PROGMEM = "Empire State Exp";
const char l20[] PROGMEM = "The Great Ghan";
const char l21[] PROGMEM = "Hudson River Ltd";
const char l22[] PROGMEM = "20th Century Ltd";
const char l23[] PROGMEM = "Thomas & Friends";

const char l26[] PROGMEM = "Snowpiercer";
const char l27[] PROGMEM = "Trans-Siberian";
const char l28[] PROGMEM = "The Starlight Exp";

enum Schedule {
  HIGH_FREQ,
  PEAK,
  OFF_PEAK
};

enum Equipment {
  BULLET,
  SHUTTLE,
  FREIGHT
};

enum Service {
  NONSTOP,
  LIMITED,
  UNPREDICTABLE
};

enum Range {
  LOCAL,
  SHORT_RUN,
  LONG_HAUL
};

const char* const ROUTES[] PROGMEM = {
  l0, l1, l2, l3, l4, l5, l6, l7, l8, l9,
  l10, l11, l12, l13, l14, l15, l16, l17, l18, l19,
  l20, l21, l22, l23
};
const int TITLE_COUNT = sizeof(ROUTES) / sizeof(ROUTES[0]);

struct RouteProfile {
  uint8_t titleId;
  uint8_t schedule;
  uint8_t equipment;
  uint8_t service;
  uint8_t range;
};

const RouteProfile ROUTE_DEFAULTS[] PROGMEM = {
  { 0, PEAK, SHUTTLE, LIMITED, SHORT_RUN },            // Pennsylvania Line
  { 1, HIGH_FREQ, BULLET, NONSTOP, LONG_HAUL },        // Hogwarts Express
  { 2, PEAK, BULLET, LIMITED, LONG_HAUL },             // California Zephyr
  { 3, HIGH_FREQ, FREIGHT, LIMITED, LONG_HAUL },       // Reading Railroad
  { 4, OFF_PEAK, SHUTTLE, UNPREDICTABLE, LONG_HAUL },  // The Polar Express
  { 5, PEAK, FREIGHT, LIMITED, LONG_HAUL },            // Union Pacific R.R.
  { 6, OFF_PEAK, BULLET, NONSTOP, LONG_HAUL },         // The Orient Express
  { 7, OFF_PEAK, BULLET, LIMITED, LONG_HAUL },         // Broadway Limited
  { 8, HIGH_FREQ, BULLET, UNPREDICTABLE, SHORT_RUN },  // The Silver Streak
  { 9, PEAK, FREIGHT, LIMITED, SHORT_RUN },            // The B&O Railroad
  { 10, PEAK, BULLET, NONSTOP, SHORT_RUN },            // The Flying Rocket
  { 11, HIGH_FREQ, SHUTTLE, LIMITED, LOCAL },          // Grand Central Line
  { 12, OFF_PEAK, BULLET, NONSTOP, LONG_HAUL },        // Flying Scotsman
  { 13, PEAK, BULLET, UNPREDICTABLE, SHORT_RUN },      // Cannonball Express
  { 14, PEAK, SHUTTLE, LIMITED, SHORT_RUN },           // The Blue Comet
  { 15, HIGH_FREQ, BULLET, NONSTOP, LOCAL },           // Taking Pelham 123
  { 16, PEAK, SHUTTLE, LIMITED, LONG_HAUL },           // Vanderbilt Central
  { 17, PEAK, BULLET, NONSTOP, LONG_HAUL },            // Broadway Local
  { 18, HIGH_FREQ, SHUTTLE, UNPREDICTABLE, LOCAL },    // The Circle Line
  { 19, PEAK, BULLET, NONSTOP, LONG_HAUL },            // Empire State Exp
  { 20, HIGH_FREQ, SHUTTLE, LIMITED, LONG_HAUL },      // The Great Ghan
  { 21, PEAK, SHUTTLE, LIMITED, LONG_HAUL },           // Hudson River Ltd
  { 22, PEAK, BULLET, NONSTOP, LONG_HAUL },            // 20th Century Ltd
  { 23, HIGH_FREQ, SHUTTLE, UNPREDICTABLE, LOCAL }     // Thomas & Friends
};

const int USER_ROUTES[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 };
const int ROUTE_COUNT = sizeof(USER_ROUTES) / sizeof(USER_ROUTES[0]);

struct OperatingState {
  uint8_t lineId;
  uint8_t schedule;
  uint8_t equipment;
  uint8_t service;
  uint8_t range;
};

RouteProfile currentRoute;

// -------- setup --------

bool detectSensor() {
  unsigned long start = millis();

  while (millis() - start < 500) {
    int v = analogRead(IR_PIN);
    if (abs(v) > 10) return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Serial Established.");

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  Serial.println("Motor Controller Pin Modes Set");

  pinMode(buttonPin, INPUT_PULLUP);
  Serial.println("Control Knob Setup.");

  pinMode(STN1_PIN, OUTPUT);
  pinMode(STN2_PIN, OUTPUT);
  pinMode(STN3_PIN, OUTPUT);
  pinMode(STN4_PIN, OUTPUT);
  Serial.println("Station Lights Setup.");

  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  pinMode(GRN_PIN, OUTPUT);
  Serial.println("Traffic Lights Setup.");

  loadFromEEPROM();
  Serial.println("EPROM Loaded.");

  u8g2.begin();
  u8g2.clearBuffer();
  Serial.println("Display Driver Initialized.");

  Serial.println("Splash.");
  splashScreen();
  Serial.println("Detect Sensor.");

  pinMode(IR_PIN, INPUT);
  sensorEnabled = detectSensor();
  if (sensorEnabled) {
    Serial.println("IR Sensor Detected.");
  } else {
    Serial.println("IR Sensor Disabled.");
  }

  Serial.println("BOOT");
}

void splashScreen() {
  const char* msg = "HELLO";
  u8g2.setFont(u8g2_font_helvB24_tr);
  u8g2.firstPage();
  do {
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 64);

    u8g2.setDrawColor(0);
    u8g2.drawStr(10, 38, msg);

  } while (u8g2.nextPage());
  delay(1000);
  u8g2.clearDisplay();
  delay(120);
}

// -------- draw --------
void toUpper(char* s) {
  for (; *s; s++) {
    if (*s >= 'a' && *s <= 'z') *s -= 32;
  }
}

void draw() {
  int color = 0;
  int backColor = 1;

  u8g2.firstPage();
  do {
    u8g2.clearBuffer();

    // ---- TOP CENTERED: TITLE (small, ALL CAPS, long) ----
    toUpper(line1);
    u8g2.setFont(u8g2_font_7x13_tr);
    u8g2.drawStr(
      (128 - u8g2.getStrWidth(line1)) / 2,
      9,
      line1);
    u8g2.setDrawColor(backColor);
    u8g2.drawBox(0, 14, 128, 36);

    u8g2.setDrawColor(color);
    u8g2.setFont(u8g2_font_logisoso32_tn);
    int numWidth = u8g2.getStrWidth(line2);
    int numRightEdge = 69;
    if (currentSpeedNumber(globalCurrentSpeed) >= 100) {
      numRightEdge = 75;
    }
    u8g2.drawStr(
      numRightEdge - numWidth,
      48,
      line2);

    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(55, 48, perHourName());
    const char* statusStr;

    if (globalCurrentSpeed == 0) {
      statusStr = "HALTED";
    } else {
      // Declare these at the top of your function
      const __FlashStringHelper* dirA;
      const __FlashStringHelper* dirB;

      switch (currentRoute.titleId) {
        case 0:  // Pennsylvania Line
          dirA = F("PHILLY");
          dirB = F("PENN STA");
          break;
        case 1:  // Hogwarts Express
          dirA = F("HOGSMEADE");
          dirB = F("LONDON");
          break;
        case 2:  // California Zephyr
          dirA = F("SAN FRAN");
          dirB = F("CHICAGO");
          break;
        case 3:  // Reading Railroad
          dirA = F("PENN STA");
          dirB = F("GRAND CTRL");
          break;
        case 4:   // The Polar Express
        case 21:  // Hudson River Ltd
          dirA = F("NORTHBOUND");
          dirB = F("SOUTHBOUND");
          break;
        case 5:  // Union Pacific R.R.
          dirA = F("PROMONTORY");
          dirB = F("OMAHA");
          break;
        case 6:  // The Orient Express
          dirA = F("ISTANBUL");
          dirB = F("PARIS");
          break;
        case 7:   // Broadway Limited
        case 17:  // Broadway Local
          dirA = F("THE BRONX");
          dirB = F("DOWNTOWN");
          break;
        case 8:  // The Silver Streak
          dirA = F("LOS ANG");
          dirB = F("CHICAGO");
          break;
        case 9:  // The B&O Railroad
          dirA = F("WASH DC");
          dirB = F("JERSEY CT");
          break;
        case 10:  // The Flying Rocket
          dirA = F("ROCK ISL");
          dirB = F("CHICAGO");
          break;
        case 11:  // Grand Central Line
          dirA = F("NEW HAVEN");
          dirB = F("GRAND CTRL");
          break;
        case 12:  // Flying Scotsman
          dirA = F("EDINBURGH");
          dirB = F("LONDON");
          break;
        case 13:  // Cannonball Express
          dirA = F("ILLINOIS");
          dirB = F("NEW ORL");
          break;
        case 14:  // The Blue Comet
          dirA = F("JERSEY CT");
          dirB = F("ATLANTIC CY");
          break;
        case 15:  // Taking Pelham 123
          dirA = F("PELHAM BAY");
          dirB = F("SOUTH FERRY");
          break;
        case 16:  // Vanderbilt Central
          dirA = F("UPSTATE");
          dirB = F("GRAND CTRL");
          break;
        case 18:  // The Circle Line
          dirA = F("CLOCKWISE");
          dirB = F("COUNTER CW");
          break;
        case 19:  // Empire State Exp
        case 22:  // 20th Century Ltd
          dirA = F("CHICAGO");
          dirB = F("NEW YORK");
          break;
        case 20:  // The Great Ghan
          dirA = F("ADELAIDE");
          dirB = F("DARWIN");
          break;
        case 23:  // Thomas & Friends
          dirA = F("KNAPFORD");
          dirB = F("FARQUHAR");
          break;
        case 24:  // The Bullet Train
          dirA = F("TOKYO");
          dirB = F("OSAKA");
          break;
        case 25:  // Chattanooga Choo
          dirA = F("TENNESSEE");
          dirB = F("PENN STA");
          break;
        case 26:  // Snowpiercer
          dirA = F("THE ENGINE");
          dirB = F("THE TAIL");
          break;

        default:  // Handle generic equipment types
          if (currentRoute.equipment == FREIGHT) {
            dirA = F("HEAVY HAUL");
            dirB = F("RETURN RUN");
          } else if (currentRoute.equipment == SHUTTLE) {
            dirA = F("CROSSTOWN");
            dirB = F("INTERURBAN");
          } else if (currentRoute.service == UNPREDICTABLE) {
            dirA = F("INBOUND");
            dirB = F("OUTBOUND");
          } else {
            dirA = F("UPTOWN");
            dirB = F("DOWNTOWN");
          }
          break;
      }

      char routeBuffer[20];  // 20 chars is plenty for "SOUTHBOUND"
      if (lastDirection) {
        strncpy_P(routeBuffer, (PGM_P)dirA, sizeof(routeBuffer) - 1);
      } else {
        strncpy_P(routeBuffer, (PGM_P)dirB, sizeof(routeBuffer) - 1);
      }
      routeBuffer[sizeof(routeBuffer) - 1] = '\0';
      statusStr = routeBuffer;
    }

    u8g2.setFont(u8g2_font_7x13_tr);
    u8g2.drawStr(
      57,
      26,
      statusStr);

    u8g2.setDrawColor(backColor);
    u8g2.setFont(u8g2_font_9x15_tr);
    u8g2.drawStr(
      (130 - u8g2.getStrWidth(line3)) / 2,
      64,
      line3);

  } while (u8g2.nextPage());
}

// -------- loop --------

void runRoutes() {
  for (uint8_t i = 0; i < ROUTE_COUNT; i++) {
    if (abortRoute) return;
    runRoute(USER_ROUTES[i]);
  }
}

void showTitleByTitleId(uint8_t titleId) {
  if (titleId >= TITLE_COUNT) return;
  strcpy_P(line1, (char*)pgm_read_word(&(ROUTES[titleId])));
  draw();
}

void runRoute(uint8_t index) {
  memcpy_P(&currentRoute, &ROUTE_DEFAULTS[index], sizeof(RouteProfile));
  currentRouteIndex = index;
  showTitleByTitleId(currentRoute.titleId);
  // -------- Schedule → legs --------
  uint8_t legs;
  switch (currentRoute.schedule) {
    case HIGH_FREQ: legs = random(4, 6); break;
    case PEAK: legs = random(2, 4); break;
    case OFF_PEAK: legs = 1; break;
  }

  // -------- Equipment → speed --------
  uint16_t baseSpeed;
  switch (currentRoute.equipment) {
    case BULLET: baseSpeed = MAX_SPEED; break;
    case SHUTTLE: baseSpeed = MAX_SPEED * 0.98; break;
    case FREIGHT: baseSpeed = MAX_SPEED * 0.95; break;
  }

  // -------- Range → duration --------
  uint16_t runTime;
  switch (currentRoute.range) {
    case LOCAL: runTime = random(6, 12); break;
    case SHORT_RUN: runTime = random(12, 20); break;
    case LONG_HAUL: runTime = random(20, 30); break;
  }

  // -------- Service → dips --------
  uint8_t dips;
  switch (currentRoute.service) {
    case NONSTOP: dips = 0; break;
    case LIMITED: dips = random(1, 3); break;
    case UNPREDICTABLE: dips = random(0, 4); break;
  }

  // -------- Execute --------
  for (uint8_t i = 0; i < legs; i++) {
    if (abortRoute) return;
    baseSpeed = baseSpeed * random(95, 105) / 100;
    go(currentDirection, baseSpeed, runTime, dips);
    currentDirection = !currentDirection;
  }
}

void loop() {

  if (abortRoute) {
    abortRoute = false;
    manualControlLoop();
    return;
  }

  if (restartRequested) {
    restartRequested = false;
    rampSpeed(MAX_SPEED);
    runRoute(currentRouteIndex);
    return;
  }

  if (!hasCalibrated && calibrateAtStartup && sensorEnabled) {
    calibrateTrain();
    hasCalibrated = true;
    Serial.println("Train Calibration Complete.");
    return;
  }

  runRoutes();
}