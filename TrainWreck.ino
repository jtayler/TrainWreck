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
  unsigned long fwdLoopMs;
  unsigned long revLoopMs;
  long stationPositionOffset;
  long stationCenterOffset;
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

Encoder speedKnob(2, 3);
const int buttonPin = 12;

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
const int RAMP_STEP = 10;
const float MAX_MPH = 72.0;
const int DOCKING_SPEED = 165;

// ----- station stop -------
unsigned long fwdLoopMs = 0;
unsigned long revLoopMs = 0;

long stationPositionOffset = 0;
long stationCenterOffset = 0;
long stationOverlapOffset = 800;

// ------- station ---------
bool sensorEnabled = true;
bool calibrateAtStartup = true;
bool stationArmed = false;
unsigned long stationTick = 0;

// ----- dip behavior -----
//const int DIP_SPEED = MAX_SPEED * 3.6 / 10;  // 25MPH
const int DIP_SPEED = MAX_SPEED * 4.9 / 10;  // 35MPH
const unsigned long DIP_TIME = 3600;         // ms per dip

// -------- display --------
bool isMPH = true;
bool lastDirection = true;
int globalCurrentSpeed = 0;
char line1[64] = "STATUS";
char line2[64] = "0";
char line3[64] = "READY";

enum StationState {
  IDLE,
  ARRIVING,
  AT_STATION,
  DEPARTING,
  COOL_DOWN
};

StationState currentStationState = IDLE;
unsigned long stateStartTime = 0;

int speedToMph(int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  return (pwm * MAX_MPH) / MAX_SPEED;
}

int speedToKph(int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  return (pwm * MAX_MPH * 161) / (MAX_SPEED * 100);
}

// -------- go! --------
bool currentDirection = true;

void go(bool forward, int speed, unsigned long runTime, int dipCount) {
  Serial.println("GO!");
  unsigned long pauseTime = random(6, 20);
  setDirection(forward);
  rampSpeed(speed);

  if (dipCount > 0) {
    unsigned long segment = (runTime * 1000) / (dipCount + 1);
    for (int i = 0; i < dipCount; i++) {
      Serial.print("🟢 FAST LEG ⏱ ");
      Serial.print(segment / 1000);
      Serial.println("s");
      const char* msg;
      snprintf(line3, sizeof(line3), "%s %ds", "FAST LEG", segment / 1000);
      unsigned long legStartTime = millis();
      while (millis() - legStartTime < segment) {
        draw();
        updateStationLights();
        //readEncoderStep();
      }
      rampSpeed(DIP_SPEED);
      Serial.print("🟡 SLOW LEG ⏱ ");
      Serial.print(DIP_TIME / 1000);
      Serial.println("s");
      snprintf(line3, sizeof(line3), "%s %ds", "SLOW LEG", DIP_TIME / 1000);
      unsigned long dipStartTime = millis();
      while (millis() - dipStartTime < DIP_TIME) {
        draw();
        updateStationLights();
        //readEncoderStep();
      }
      rampSpeed(random(speed * 0.85, speed));
    }
    Serial.print("🟢 FAST LEG ⏱ ");
    Serial.print(segment / 1000);
    Serial.println("s");
    snprintf(line3, sizeof(line3), "%s %ds", "FAST LEG", segment / 1000);
    unsigned long segmentStartTime = millis();
    while (millis() - segmentStartTime < segment) {
      draw();
      updateStationLights();
      //readEncoderStep();
    }
  } else {
    Serial.print("🟢 ONLY LEG ⏱ ");
    Serial.print(runTime);
    Serial.println("s");
    snprintf(line3, sizeof(line3), "%s %ds", "ONLY LEG", runTime);
    unsigned long onlyStartTime = millis();
    while (millis() - onlyStartTime < runTime * 1000) {
      draw();
      updateStationLights();
      // readEncoderStep();
    }
  }
  Serial.print("🛑 STOP ⏱ ");
  Serial.print(pauseTime);
  Serial.println("s");
  snprintf(line3, sizeof(line3), "%s", "BRAKE TO HALT");
  draw();
  // readEncoderStep();

  rampSpeed(0);
  stateStartTime = millis();
  snprintf(line3, sizeof(line3), "%s %ds", "AT STATION", pauseTime);
  unsigned long pauseMs = pauseTime * 1000;
  while (millis() - stateStartTime < pauseMs) {
    draw();
    updateStationLights();
    // readEncoderStep();
  }

  setStationState(DEPARTING);
  updateStationLights();
  snprintf(line3, sizeof(line3), "%s", "NOW BOARDING");
  unsigned long start = millis();
  while (millis() - start < 4000) {
    updateStationLights();
    draw();
    // readEncoderStep();
  }
}

// -------- signal --------
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
const unsigned long HOLD_AFTER_LEAVE = 4000;
const unsigned long FADE_MS = 5000;

void allOn() {
  //Serial.print("ALL ON! ");
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
        allOn();
        if (elapsed >= HOLD_AFTER_LEAVE) {
          fading = true;
          stateStartTime = millis();
        }
      } else {
        fadeToBlackMs(FADE_MS);
        if (millis() - stateStartTime >= FADE_MS) {
          fading = false;
          setStationState(IDLE);
        }
      }
      break;
    case AT_STATION:
      allOn();  // Solid lights while stopped
      break;

    case IDLE:
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
  static bool lastSensorState = false;
  static bool dockedThisStop = false;
  int start = current;
  int delta = abs(target - start);
  bool rampUp = target > current;

  if (target == 0) stationArmed = false;

  if (target != 0) dockedThisStop = false;

  if (target == current) {
    globalCurrentSpeed = current;
    return;
  }

  updateTafficSignal((rampUp ? 1 : current), rampUp);

  while (current != target) {

    // Start blinking as soon as we enter docking range
    if (target == 0 && current < (DOCKING_SPEED - 10)) {
      setStationState(ARRIVING);
    }

    if (sensorEnabled && target == 0 && !dockedThisStop) {

      if (current <= DOCKING_SPEED) {

        Serial.println("WAITING FOR STATION EDGE");

        while (!stationArmed) {
          int v = analogRead(IR_PIN);
          if (v < IR_THRESHOLD) {
            stationArmed = true;
            stationTick = millis();
          }
          updateStationLights();
          draw();
        }

        unsigned long waitMs = calculateStationPause(lastDirection);

        unsigned long startWait = millis();
        while (millis() - startWait < waitMs) {
          updateStationLights();
          draw();
          // readEncoderStep();
        }

        setStationState(AT_STATION);
        dockedThisStop = true;
        Serial.println("DOCKING COMPLETED");
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

    if (isMPH) {
      snprintf(line2, sizeof(line2), "%d MPH", speedToMph(current));
    } else {
      snprintf(line2, sizeof(line2), "%d KPH", speedToKph(current));
    }

    if (target == 0) {
      updateTafficSignal(current, rampUp);
    } else {
      if (isMPH) {
        snprintf(line3, sizeof(line3), "%s %d MPH",
                 rampUp ? "RAMP TO" : "DOWN TO",
                 speedToMph(target));
      } else {
        snprintf(line3, sizeof(line3), "%s %d",
                 rampUp ? "RAMP TO" : "DOWN TO",
                 speedToKph(target));
      }
    }
    writeMotor(lastDirection, current);
    draw();
    updateStationLights();
    //readEncoderStep();
  }

  updateTafficSignal(current, rampUp);
}

// -------- calibrate --------
// Calibration function
void calibrateTrain() {
  rampSpeed(0);
  // snprintf(line1, sizeof(line1), "CALIBRATE RAMP");
  // unsigned long rampTestStartTime = millis();
  // rampSpeed(MAX_SPEED);
  // unsigned long rampTime = millis() - rampTestStartTime;
  // Serial.print("rampTime: ");
  // Serial.println(rampTime);
  // delay(1000);
  snprintf(line1, sizeof(line1), "CALIBRATE STATION");

  unsigned long lapFwd = 0; //measureLap(true);
  //delay(1000);
  unsigned long lapRev = measureLap(false);

  // Save results
  Persist p;
  p.version = EEPROM_VERSION;
  p.fwdLoopMs = lapFwd;
  p.revLoopMs = lapRev;

  EEPROM.put(0, p);  // Write to EEPROM

  // Log results
  Serial.print("Lap FWD: ");
  Serial.println(lapFwd);
  Serial.print("Lap REV: ");
  Serial.println(lapRev);
  Serial.print("FWD Loop Time: ");
  Serial.println(p.fwdLoopMs);
  Serial.print("REV Loop Time: ");
  Serial.println(p.revLoopMs);
}

void loadFromEEPROM() {
  Persist p;
  EEPROM.get(0, p);

  if (p.version == EEPROM_VERSION) {
    fwdLoopMs = p.fwdLoopMs;
    revLoopMs = p.revLoopMs;
  } else {
    // No saved data, run calibration
    calibrateTrain();
  }
}

unsigned long calculateStationPause(bool forward) {
  if (forward) {
    return stationPositionOffset - stationCenterOffset;
  } else {
    return ((revLoopMs * 0.5) + stationCenterOffset + stationOverlapOffset);
  }
}
// Function to measure lap time
unsigned long measureLap(bool forward) {

  Serial.println("---- CALIBRATION ----");
  Serial.println(forward ? "FWD" : "REV");

  // Set direction and ramp speed
  setDirection(forward);
  rampSpeed(DOCKING_SPEED);  // Assume this will ramp to speed immediately

  // No delay here because the train will be at speed once rampSpeed() finishes

  bool lastState = analogRead(IR_PIN) < IR_THRESHOLD;

  Serial.println("Waiting for falling edge...");

  // wait for transition HIGH -> LOW
  while (true) {
    bool state = analogRead(IR_PIN) < IR_THRESHOLD;
    if (!lastState && state) break;
    lastState = state;
  }

  Serial.println("First edge.");

  // wait for rising edge (clear)
  while (true) {
    bool state = analogRead(IR_PIN) < IR_THRESHOLD;
    if (lastState && !state) break;
    lastState = state;
  }

  Serial.println("Clear.");
  Serial.println(lastState);

  delay(1000);

  unsigned long start = millis();
  Serial.println("Timing...");

  while (true) {
    bool state = analogRead(IR_PIN) < IR_THRESHOLD;
    if (!lastState && state) break;
    lastState = state;
  }

  unsigned long lap = millis() - start;

  Serial.println();
  Serial.println("SECOND block detected.");
  Serial.print("Lap measured: ");
  Serial.println(lap);

  rampSpeed(0);  // Stop the motor after measuring
  Serial.println("---- CALIBRATION END ----");

  return lap;
}

// Save calibration values to EEPROM
void saveToEEPROM() {
  Persist p;
  p.version = EEPROM_VERSION;
  p.fwdLoopMs = fwdLoopMs;
  p.revLoopMs = revLoopMs;
  p.stationPositionOffset = stationPositionOffset;
  p.stationCenterOffset = stationCenterOffset;

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
const char l17[] PROGMEM = "Broadway Limited";
const char l18[] PROGMEM = "The Circle Line";
const char l19[] PROGMEM = "Empire State Exp";
const char l20[] PROGMEM = "The Great Ghan";
const char l21[] PROGMEM = "Hudson River Ltd";
const char l22[] PROGMEM = "20th Century Ltd";
const char l23[] PROGMEM = "Thomas & Friends";

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
  { 0, PEAK, SHUTTLE, LIMITED, SHORT_RUN },             // Pennsylvania Line
  { 1, HIGH_FREQ, BULLET, NONSTOP, LONG_HAUL },         // Hogwarts Express
  { 2, PEAK, BULLET, LIMITED, LONG_HAUL },              // California Zephyr
  { 3, HIGH_FREQ, SHUTTLE, LIMITED, LOCAL },            // Reading Railroad
  { 4, HIGH_FREQ, SHUTTLE, UNPREDICTABLE, LONG_HAUL },  // The Polar Express
  { 5, PEAK, FREIGHT, LIMITED, LONG_HAUL },             // Union Pacific R.R.
  { 6, OFF_PEAK, BULLET, NONSTOP, LONG_HAUL },          // The Orient Express
  { 7, PEAK, BULLET, LIMITED, LONG_HAUL },              // Broadway Limited
  { 8, HIGH_FREQ, BULLET, UNPREDICTABLE, SHORT_RUN },   // The Silver Streak
  { 9, PEAK, FREIGHT, LIMITED, SHORT_RUN },             // The B&O Railroad
  { 10, PEAK, BULLET, NONSTOP, SHORT_RUN },             // The Flying Rocket
  { 11, HIGH_FREQ, SHUTTLE, LIMITED, LOCAL },           // Grand Central Line
  { 12, OFF_PEAK, BULLET, NONSTOP, LONG_HAUL },         // Flying Scotsman
  { 13, PEAK, BULLET, UNPREDICTABLE, SHORT_RUN },       // Cannonball Express
  { 14, PEAK, SHUTTLE, LIMITED, SHORT_RUN },            // The Blue Comet
  { 15, OFF_PEAK, BULLET, NONSTOP, LOCAL },             // Taking Pelham 123
  { 16, PEAK, SHUTTLE, LIMITED, LONG_HAUL },            // Vanderbilt Central
  { 17, PEAK, BULLET, NONSTOP, LONG_HAUL },             // Broadway Limited (duplicate title entry)
  { 18, HIGH_FREQ, SHUTTLE, UNPREDICTABLE, LOCAL },     // The Circle Line
  { 19, PEAK, BULLET, NONSTOP, LONG_HAUL },             // Empire State Exp
  { 20, HIGH_FREQ, SHUTTLE, LIMITED, LONG_HAUL },       // The Great Ghan
  { 21, PEAK, SHUTTLE, LIMITED, LONG_HAUL },            // Hudson River Ltd
  { 22, PEAK, BULLET, NONSTOP, LONG_HAUL },             // 20th Century Ltd
  { 23, HIGH_FREQ, SHUTTLE, UNPREDICTABLE, LOCAL }      // Thomas & Friends
};

const int USER_ROUTES[] = { 15, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 };
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
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Serial Established.");

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  Serial.println("Motor Controller Pin Modes Set");

  u8g2.begin();
  u8g2.clearBuffer();
  Serial.println("Display Driver Initialized.");

  pinMode(IR_PIN, INPUT);
  Serial.println("IR Sensor Connected.");

  loadFromEEPROM();
  Serial.println("EPROM Loaded.");

  if (calibrateAtStartup && sensorEnabled) {
    snprintf(line1, sizeof(line1), "CALIBRATING");
    draw();
    calibrateTrain();
    Serial.println("Train Calibration Complete.");
  }

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

  Serial.println("BOOT");
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
    u8g2.setFont(u8g2_font_logisoso32_tn);
    u8g2.drawBox(0, 14, 128, 36);

    u8g2.setDrawColor(color);
    u8g2.setFont(u8g2_font_logisoso32_tn);
    int numWidth = u8g2.getStrWidth(line2);
    int numRightEdge = 69;
    if (line2 >= 100) {
      //numRightEdge = 76;
    }
    u8g2.drawStr(
      numRightEdge - numWidth,
      48,
      line2);

    u8g2.setFont(u8g2_font_ncenB18_tr);
    if (isMPH) {
      u8g2.drawStr(56, 48, "MPH");
    } else {
      u8g2.drawStr(56, 48, "KPH");
    }
    const char* statusStr;

    if (globalCurrentSpeed == 0) {
      statusStr = "HALTED";
    } else {
      const char* dirA = "UPTOWN";
      const char* dirB = "DOWNTOWN";

      if (currentRoute.titleId == 1) {  // Hogwarts Express
        dirA = "HOGSMEADE";
        dirB = "LONDON";
      } else if (currentRoute.titleId == 15) {  // Pelham
        dirA = "UPTOWN";
        dirB = "DOWNTOWN";
      } else if (currentRoute.titleId == 18) {  // The Circle Line
        dirA = "CLOCKWISE";
        dirB = "COUNTER CW";
      } else if (currentRoute.titleId == 0 || currentRoute.titleId == 6 || currentRoute.titleId == 2) {  // Penn, Orient, Zephyr
        dirA = "EASTBOUND";
        dirB = "WESTBOUND";
      } else if (currentRoute.titleId == 4 || currentRoute.titleId == 21) {  // Polar Express, Hudson
        dirA = "NORTHBOUND";
        dirB = "SOUTHBOUND";
      } else if (currentRoute.equipment == FREIGHT) {
        dirA = "HEAVY HAUL";
        dirB = "RETURN RUN";
      } else if (currentRoute.equipment == SHUTTLE) {
        dirA = "CROSSTOWN";
        dirB = "INTERURBAN";
      } else if (currentRoute.service == UNPREDICTABLE) {
        dirA = "INBOUND";
        dirB = "OUTBOUND";
      }

      statusStr = lastDirection ? dirA : dirB;
    }

    u8g2.setFont(u8g2_font_7x13_tr);
    u8g2.drawStr(
      58,
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
    case SHUTTLE: baseSpeed = MAX_SPEED * 0.95; break;
    case FREIGHT: baseSpeed = MAX_SPEED * 0.90; break;
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
    case LIMITED: dips = random(1, 3); break;
    case UNPREDICTABLE: dips = random(0, 5); break;
  }

  // -------- Execute --------
  for (uint8_t i = 0; i < legs; i++) {
    // Add slight realism variation (±5%)
    baseSpeed = baseSpeed * random(95, 105) / 100;
    go(currentDirection, baseSpeed, runTime, dips);
    currentDirection = !currentDirection;
  }
}

void loop() {
  runRoutes();
}
