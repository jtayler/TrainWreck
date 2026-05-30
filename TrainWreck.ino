#include <SPI.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <avr/pgmspace.h>
#include <Encoder.h>

// --- pins ---
#define CS_PIN 10
#define DC_PIN 9
#define RST_PIN 8

// --- persistence store ---
#define EEPROM_VERSION 15

struct Persist {
  byte version;
  unsigned long lapFwd;
  unsigned long lapRev;
  unsigned long trainMs;
  uint8_t lastRouteIndex;
};

// --- display driver ---
U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, 10, 9, 8);

// -------- pins --------
const int IR_PIN = A0;
const int in1Pin = 5;
const int in2Pin = 6;
const int CLK_PIN = 2;  // INT0
const int DT_PIN = 3;   // INT1
const int buttonPin = 12;
const int RED_PIN = 7;
const int YEL_PIN = A5;
const int GRN_PIN = 4;

const int STN1_PIN = A1;
const int STN2_PIN = A2;
const int STN3_PIN = A3;
const int STN4_PIN = A4;

// --- startup & station ---
long stationDist = 1 * 60;
bool calibrateAtStartup = true;
bool testStationMode = false;

// -------- tuning ---------
const int MAX_SPEED = 255;
const int DOCKING_SPEED = 165;
const int RAMP_STEP = 10;
const int RAMP_MINUTES = 120;
const float MAX_MPH = 72;

// ------- calibration ---------
unsigned long snoozingMinutes = 20;
bool sensorEnabled = false;
volatile bool calibrating = false;
bool hasCalibrated = false;
bool stationArmed = false;
unsigned long stationTick = 0;
long lastPos = 0;

// ----- dip behavior -----
const unsigned long DIP_TIME = 9600;

// -------- calibrate --------
unsigned long storedLapFwd = 0;
unsigned long storedLapRev = 0;
unsigned long irDipDurationMs = 0;

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
  int max = maxSpeed();

  if (pwm < 0) pwm = 0;
  if (pwm > max) pwm = max;

  return ((long)pwm * MAX_MPH) / max;
}

int speedToKph(int pwm) {
  pwm = constrain(pwm, 0, maxSpeed());
  return (pwm * MAX_MPH * 161) / (maxSpeed() * 100);
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
        } else {
          manualLocked = true;
        }
      }
    }
  }
}
static long delta = 0;

void manualControlLoop() {
  snprintf(line3, sizeof(line3), "MANUAL CONTROL");
  draw();

  static unsigned long snoozeTime = 0;
  snoozeTime = millis();  // seed on entry — prevent false snooze trigger
  delta = 0;              // clear stale direction from prior session

  while (true) {
    updateStationLights();
    long pos = speedKnob.read() / 4;

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
        setStationState(CRUISING);
        stationLightsOff();
      }
    }
    if (pos != lastPos) {
      long oldPos = lastPos;
      lastPos = pos;
      int prevSpeed = globalCurrentSpeed;
      delta = oldPos - pos;

      globalCurrentSpeed += delta * 6;
      if (delta < 0 && globalCurrentSpeed < 10) globalCurrentSpeed = 0;

      globalCurrentSpeed = constrain(globalCurrentSpeed, 0, 255);

      if (globalCurrentSpeed == 0 && prevSpeed > 0) {
        lastDirection = !lastDirection;
        snoozeTime = millis();
      }

      if (globalCurrentSpeed > 0) {
        snoozeTime = millis();
      }

      if (globalCurrentSpeed == 0) signalRed();
      else if (delta < 0) signalYellow();
      else signalGreen();

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
        break;
      }
    }
  }
  mode = AUTO;
  restartRequested = true;
  abortRoute = false;
  stationArmed = false;
}

// -------- dip --------

int maxSpeed() {
  return constrain(MAX_SPEED, 128, 255);
}

int dipSpeed() {
  return max(50, (maxSpeed() * 32) / 100);
}

int dipSpeedFast() {
  return max(dipSpeed() + 10, (maxSpeed() * 46) / 100);
}

// -------- go! --------
bool currentDirection = true;

void go(bool forward, int speed, unsigned long runTime, int dipCount) {
  if (abortRoute) return;

  unsigned long pauseTime = random(6, 20);
  setDirection(forward);
  rampSpeed(speed);

  if (dipCount > 0) {
    long jitterPct = random(-20, 21);  // -20% to +20%
    unsigned long segment = (runTime * 1000UL * (100 + jitterPct)) / 100;
    for (int i = 0; i < dipCount; i++) {
      if (abortRoute) return;
      unsigned long legStartTime = millis();
      while (millis() - legStartTime < segment) {
        if (abortRoute) return;
        snprintf(line3, sizeof(line3), "%s %ds", "FAST LEG", segment / 1000);
        draw();
        updateStationLights();
        readEncoderStep();
      }
      rampSpeed(random(dipSpeed(), dipSpeedFast()));
      long dipJitterPct = random(-20, 21);  // -20% to +20%
      unsigned long thisDipTime = (DIP_TIME * (100 + dipJitterPct)) / 100;
      unsigned long dipStartTime = millis();
      while (millis() - dipStartTime < thisDipTime) {
        if (abortRoute) return;
        snprintf(line3, sizeof(line3), "%s %ds", "SLOW LEG", DIP_TIME / 1000);
        draw();
        updateStationLights();
        readEncoderStep();
      }
      rampSpeed(random(speed * 0.85, speed));
    }
    unsigned long segmentStartTime = millis();
    while (millis() - segmentStartTime < segment) {
      if (abortRoute) return;
      snprintf(line3, sizeof(line3), "%s %ds", "FAST LEG", segment / 1000);
      draw();
      updateStationLights();
      readEncoderStep();
    }
  } else {
    unsigned long onlyStartTime = millis();
    while (millis() - onlyStartTime < runTime * 1000) {
      if (abortRoute) return;
      snprintf(line3, sizeof(line3), "%s %ds", "ONLY LEG", runTime);
      draw();
      updateStationLights();
      readEncoderStep();
    }
  }
  readEncoderStep();

  rampSpeed(0);
  stateStartTime = millis();
  unsigned long pauseMs = max(0L, (long)(pauseTime * 1000UL));
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
    snprintf(line3, sizeof(line3), "%s", "NOW DEPARTING");
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
  if (calibrating) return;
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

const unsigned long ARRIVE_BLINK_MS = 4000;
const unsigned long DEPART_BLINK_MS = 12000;
const unsigned long HOLD_AFTER_LEAVE = 8000;
const unsigned long FADE_MS = 6000;

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
  if (calibrating) return;
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
  pwm = constrain(pwm, 0, maxSpeed());
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

// -------- IR sensor --------
bool trainPassingIR(bool reset = false) {
  static int baseline = -1;
  static bool armed = false;
  static bool inDip = false;
  static unsigned long armStartMs = 0;
  static unsigned long dipStartMs = 0;
  static unsigned long lastBaselineUpdate = 0;

  int v = analogRead(IR_PIN);
  unsigned long now = millis();

  if (reset || baseline < 1) {
    baseline = v;
    armed = false;
    inDip = false;
    armStartMs = now;
    lastBaselineUpdate = now;
    return false;
  }

  if (!armed) {
    int p = ((long)(baseline - v) * 100L) / baseline;
    if (p < 8) baseline = (baseline * 7 + v) / 8;
    if (now - armStartMs >= 750) armed = true;
    return false;
  }

  int percent = ((long)(baseline - v) * 100L) / baseline;

  if (!inDip) {
    if (percent >= 12) {
      inDip = true;
      dipStartMs = now;
    } else if (percent < 4 && now - lastBaselineUpdate >= 500) {
      baseline = (baseline * 15 + v) / 16;
      lastBaselineUpdate = now;
    }
  } else {
    if (percent <= 5) {
      irDipDurationMs = now - dipStartMs;
      inDip = false;
      return true;
    }
  }

  return false;
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

  unsigned long rampBlockStart = 0;

  while (current != target) {
    if (abortRoute) return;
    if (target == 0 && current < (DOCKING_SPEED - 10)) {
      setStationState(ARRIVING);
    }
    if (sensorEnabled && target == 0 && !dockedThisStop) {
      if (current <= DOCKING_SPEED) {
        snprintf(line3, sizeof(line3), "%s", "BRAKE TO HALT");
        draw();
        trainPassingIR(true);
        while (!stationArmed) {
          if (abortRoute) return;
          if (trainPassingIR()) {
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
        rampBlockStart = millis();
        start = current;
        delta = current;
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
      // snprintf(line3, sizeof(line3), "BOO!");

      int mph = speedToMph(target);
      
      const char* unit = perHourName();
      char* ramp = rampUp ? "RAMP TO" : "DOWN TO";

      snprintf(line3, sizeof(line3), "%s %d %s",
               ramp,
               mph,
               unit);
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
    // Serial.println(line3);
    draw();
    updateStationLights();
    readEncoderStep();
  }
  updateTafficSignal(current, rampUp);

  if (target == 0 && rampBlockStart > 0) {
    unsigned long lapMs = lastDirection ? storedLapFwd : storedLapRev;
    unsigned long rampBlockMs = (unsigned long)RAMP_MINUTES * lapMs / 720L;
    while (millis() - rampBlockStart < rampBlockMs) {
      if (abortRoute) return;
      updateStationLights();
      readEncoderStep();
    }
  }
}

void calibrateTrain() {
  calibrating = true;
  signalOff();
  stationLightsOff();
  snprintf(line1, sizeof(line1), "CALIBRATE ENGINE");
  draw();
  storedLapFwd = measureLap(true);
  if (abortRoute) {
    calibrating = false;
    return;
  }
  storedLapRev = measureLap(false);
  if (abortRoute) {
    calibrating = false;
    return;
  }

  Persist p;
  p.version = EEPROM_VERSION;
  p.lapFwd = storedLapFwd;
  p.lapRev = storedLapRev;
  p.trainMs = irDipDurationMs;
  p.lastRouteIndex = currentRouteIndex;
  EEPROM.put(0, p);

  // Serial.print("FWD lap ms: "); Serial.println(storedLapFwd);
  // Serial.print("REV lap ms: "); Serial.println(storedLapRev);
  if (storedLapFwd > 0) {
    // Serial.print("Train size: "); Serial.print(irDipDurationMs * 720UL / storedLapFwd); Serial.println(" cmin");
  }
  // Serial.print("FWD coast ms: "); Serial.println(calculateStationPause(true));
  // Serial.print("REV coast ms: "); Serial.println(calculateStationPause(false));
  calculateStationPause(true);
  calculateStationPause(false);
  setStationState(DEPARTING);
  calibrating = false;
}

void loadFromEEPROM() {
  Persist p;
  EEPROM.get(0, p);
  if (p.version == EEPROM_VERSION) {
    storedLapFwd = p.lapFwd;
    storedLapRev = p.lapRev;
    irDipDurationMs = p.trainMs;
    currentRouteIndex = p.lastRouteIndex;
  }
}

unsigned long calculateStationPause(bool forward) {
  long correction = (long)(irDipDurationMs / 2);
  if (forward) {
    if (storedLapFwd == 0) return 0;
    long ms = (long)((unsigned long)stationDist * storedLapFwd / 720UL) - correction;
    // Serial.print("FWD coast="); Serial.println(max(0L, ms));
    return (unsigned long)max(0L, ms);
  } else {
    if (storedLapRev == 0) return 0;
    long ms = (long)((480L - stationDist) * storedLapRev / 720L) - correction;
    // Serial.print("REV coast="); Serial.println(max(0L, ms));
    return (unsigned long)max(0L, ms);
  }
}
// Function to measure lap time
unsigned long measureLap(bool forward) {
  unsigned long start = 0;
  draw();

  setDirection(forward);
  for (int s = 0; s <= DOCKING_SPEED; s += 6) {
    if (abortRoute) {
      writeMotor(forward, 0);
      return 0;
    }
    globalCurrentSpeed = s;
    snprintf(line2, sizeof(line2), "%d %s", speedToMph(globalCurrentSpeed), perHourName());
    draw();
    readEncoderStep();
    writeMotor(forward, s);
  }

  trainPassingIR(true);
  while (!trainPassingIR()) {
    if (abortRoute) return 0;
    readEncoderStep();
  }
  snprintf(line3, sizeof(line3), "MARK");
  draw();

  delay(300);

  start = millis();
  trainPassingIR(true);
  while (!trainPassingIR()) {
    if (abortRoute) return 0;
    readEncoderStep();
  }

  snprintf(line3, sizeof(line3), "SET");
  draw();

  unsigned long lap = millis() - start;

  bool wasSensor = sensorEnabled;
  sensorEnabled = false;
  rampSpeed(0);
  sensorEnabled = wasSensor;
  snprintf(line3, sizeof(line3), "GO!");
  draw();

  return lap;
}


// -------- route strings (PROGMEM) --------
const char rt_pelham[] PROGMEM = "Taking Pelham 123";
const char rt_hogwarts[] PROGMEM = "Hogwarts Express";
const char rt_zephyr[] PROGMEM = "California Zephyr";
const char rt_polar[] PROGMEM = "The Polar Express";
const char rt_orient[] PROGMEM = "The Orient Express";
const char rt_broadway[] PROGMEM = "Broadway Limited";
const char rt_silver[] PROGMEM = "The Silver Streak";
const char rt_scotsman[] PROGMEM = "Flying Scotsman";
const char rt_cannon[] PROGMEM = "Cannonball Express";
const char rt_circle[] PROGMEM = "The Circle Line";
const char rt_empire[] PROGMEM = "Empire State Exp";
const char rt_ghan[] PROGMEM = "The Great Ghan";
const char rt_century[] PROGMEM = "20th Century Ltd";
const char rt_thomas[] PROGMEM = "Thomas & Friends";

const char dir_pelbay[] PROGMEM = "UPTOWN";
const char dir_sferry[] PROGMEM = "DOWNTOWN";
const char dir_hogs[] PROGMEM = "HOGSMEADE";
const char dir_london[] PROGMEM = "LONDON";
const char dir_sanfran[] PROGMEM = "SAN FRAN";
const char dir_chicago[] PROGMEM = "CHICAGO";
const char dir_north[] PROGMEM = "NORTHBOUND";
const char dir_south[] PROGMEM = "SOUTHBOUND";
const char dir_istanbul[] PROGMEM = "ISTANBUL";
const char dir_paris[] PROGMEM = "PARIS";
const char dir_bronx[] PROGMEM = "THE BRONX";
const char dir_dtown[] PROGMEM = "DOWNTOWN";
const char dir_losang[] PROGMEM = "LOS ANG";
const char dir_edinb[] PROGMEM = "EDINBURGH";
const char dir_illinois[] PROGMEM = "ILLINOIS";
const char dir_neworl[] PROGMEM = "N'AWLINS";
const char dir_cw[] PROGMEM = "CLOCKWISE";
const char dir_ccw[] PROGMEM = "COUNTER CW";
const char dir_newyork[] PROGMEM = "NEW YORK";
const char dir_adelaide[] PROGMEM = "ADELAIDE";
const char dir_darwin[] PROGMEM = "DARWIN";
const char dir_knapford[] PROGMEM = "KNAPFORD";
const char dir_farquhar[] PROGMEM = "FARQUHAR";

enum Schedule { HIGH_FREQ,
                PEAK,
                OFF_PEAK };
enum Equipment { BULLET,
                 SHUTTLE,
                 FREIGHT };
enum Service { NONSTOP,
               LIMITED,
               UNPREDICTABLE };
enum Range { LOCAL,
             SHORT_RUN,
             LONG_HAUL };

struct RouteProfile {
  PGM_P title;
  PGM_P dirA;
  PGM_P dirB;
  uint8_t schedule;
  uint8_t equipment;
  uint8_t service;
  uint8_t range;
};

// Each route is self-contained — reorder freely, nothing will break.
// Pelham is first so it runs first (easy test: should show UPTOWN/DOWNTOWN, not Hogsmeade).
const RouteProfile ROUTE_DEFAULTS[] PROGMEM = {
  { rt_pelham, dir_pelbay, dir_sferry, HIGH_FREQ, BULLET, NONSTOP, LOCAL },
  { rt_hogwarts, dir_hogs, dir_london, HIGH_FREQ, BULLET, NONSTOP, LONG_HAUL },
  { rt_zephyr, dir_sanfran, dir_chicago, PEAK, BULLET, LIMITED, LONG_HAUL },
  { rt_polar, dir_north, dir_south, OFF_PEAK, SHUTTLE, UNPREDICTABLE, LONG_HAUL },
  { rt_orient, dir_istanbul, dir_paris, OFF_PEAK, BULLET, NONSTOP, LONG_HAUL },
  { rt_broadway, dir_bronx, dir_dtown, OFF_PEAK, BULLET, LIMITED, LONG_HAUL },
  { rt_silver, dir_losang, dir_chicago, HIGH_FREQ, BULLET, UNPREDICTABLE, SHORT_RUN },
  { rt_scotsman, dir_edinb, dir_london, OFF_PEAK, BULLET, NONSTOP, LONG_HAUL },
  { rt_cannon, dir_illinois, dir_neworl, PEAK, BULLET, UNPREDICTABLE, SHORT_RUN },
  { rt_circle, dir_cw, dir_ccw, HIGH_FREQ, SHUTTLE, UNPREDICTABLE, LOCAL },
  { rt_empire, dir_chicago, dir_newyork, PEAK, BULLET, NONSTOP, LONG_HAUL },
  { rt_ghan, dir_adelaide, dir_darwin, HIGH_FREQ, SHUTTLE, LIMITED, LONG_HAUL },
  { rt_century, dir_chicago, dir_newyork, PEAK, BULLET, NONSTOP, LONG_HAUL },
  { rt_thomas, dir_knapford, dir_farquhar, HIGH_FREQ, SHUTTLE, UNPREDICTABLE, LOCAL },
};
const int ROUTE_COUNT = sizeof(ROUTE_DEFAULTS) / sizeof(ROUTE_DEFAULTS[0]);

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
  // Serial.println("BOOT");

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(STN1_PIN, OUTPUT);
  pinMode(STN2_PIN, OUTPUT);
  pinMode(STN3_PIN, OUTPUT);
  pinMode(STN4_PIN, OUTPUT);

  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  pinMode(GRN_PIN, OUTPUT);

  loadFromEEPROM();
  // Serial.print("stationDist="); Serial.println(stationDist);
  u8g2.begin();
  u8g2.clearBuffer();
  splashScreen();

  randomSeed(analogRead(A7));  //unused pin for rando!

  pinMode(IR_PIN, INPUT);
  sensorEnabled = detectSensor();
  // Serial.println(sensorEnabled ? "IR OK" : "IR OFF");
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
    char routeBuffer[20] = "HALTED";
    if (calibrating) {
      strncpy(routeBuffer, globalCurrentSpeed == 0 ? "" : (lastDirection ? "FORWARD" : "REVERSE"), sizeof(routeBuffer) - 1);
    } else if (globalCurrentSpeed > 0) {
      strncpy_P(routeBuffer, lastDirection ? currentRoute.dirA : currentRoute.dirB, sizeof(routeBuffer) - 1);
      routeBuffer[sizeof(routeBuffer) - 1] = '\0';
    }
    const char* statusStr = routeBuffer;

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
    runRoute((currentRouteIndex + i) % ROUTE_COUNT);
  }
  currentRouteIndex = 0;
}

void runRoute(uint8_t index) {
  memcpy_P(&currentRoute, &ROUTE_DEFAULTS[index], sizeof(RouteProfile));
  currentRouteIndex = index;
  strcpy_P(line1, currentRoute.title);
  draw();
  {
    Persist p;
    EEPROM.get(0, p);
    if (p.version == EEPROM_VERSION && p.lastRouteIndex != index) {
      p.lastRouteIndex = index;
      EEPROM.put(0, p);
    }
  }
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
    case BULLET: baseSpeed = maxSpeed(); break;
    case SHUTTLE: baseSpeed = maxSpeed() * 0.95; break;
    case FREIGHT: baseSpeed = maxSpeed() * 0.85; break;
  }

  // -------- Range → duration --------
  uint16_t runTime;
  switch (currentRoute.range) {
    case LOCAL: runTime = random(6, 12); break;
    case SHORT_RUN: runTime = random(12, 20); break;
    case LONG_HAUL: runTime = random(20, 40); break;
  }

  // -------- Service → dips --------
  uint8_t dips;
  switch (currentRoute.service) {
    case NONSTOP: dips = 0; break;
    case LIMITED: dips = random(1, 4); break;
    case UNPREDICTABLE: dips = random(0, 6); break;
  }

  // -------- Execute --------
  for (uint8_t i = 0; i < legs; i++) {
    if (abortRoute) return;
    baseSpeed = baseSpeed * random(95, 105) / 100;
    go(currentDirection, baseSpeed, runTime, dips);
    currentDirection = !currentDirection;
  }
}

void testStationLoop() {
  memcpy_P(&currentRoute, &ROUTE_DEFAULTS[currentRouteIndex], sizeof(RouteProfile));
  snprintf(line1, sizeof(line1), "STATION TEST");
  int lap = 0;
  while (!abortRoute) {
    lap++;
    setDirection(lastDirection);
    rampSpeed(DOCKING_SPEED);
    unsigned long t = millis();
    while (millis() - t < 1200) {
      if (abortRoute) return;
      snprintf(line3, sizeof(line3), "DEPARTING %d", lap);
      draw();
      updateStationLights();
      readEncoderStep();
    }
    rampSpeed(0);
    setStationState(AT_STATION);
    t = millis();
    while (millis() - t < 3000) {
      if (abortRoute) return;
      snprintf(line3, sizeof(line3), "AT STATION %d", lap);
      draw();
      updateStationLights();
      readEncoderStep();
    }
    setStationState(DEPARTING);
    lastDirection = !lastDirection;
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
    rampSpeed(maxSpeed());
    runRoute(currentRouteIndex);
    return;
  }

  if (!hasCalibrated && calibrateAtStartup && sensorEnabled) {
    calibrateTrain();
    hasCalibrated = true;
    return;
  }

  if (testStationMode) {
    testStationLoop();
    return;
  }

  runRoutes();
}