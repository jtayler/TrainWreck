#include <SPI.h>
#include <EEPROM.h>
#include <U8g2lib.h>

#define CS_PIN 10
#define DC_PIN 9
#define RST_PIN 8

// --- persistence store ---

#define EEPROM_VERSION 1

struct Persist {
  byte version;
  unsigned long fwdLoopMs;
  unsigned long revLoopMs;
  long fwdOffsetMs;
  long revOffsetMs;
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

const int RED_PIN = 2;
const int YEL_PIN = 3;
const int GRN_PIN = 4;

const int STN1_PIN = A1;
const int STN2_PIN = A2;
const int STN3_PIN = A3;
const int STN4_PIN = A4;

// -------- tuning ---------
const int MAX_SPEED = 170;
const int RAMP_STEP = 5;
const int RAMP_DELAY = 1;
const int MIN_SPEED = 0;
const float MAX_MPH = 72.0;
const int DOCKING_SPEED = 58;

// ----- station stop -------
unsigned long fwdLoopMs = 0;
unsigned long revLoopMs = 0;

long fwdOffsetMs = 20;
long revOffsetMs = 1000;

// ------- station ---------
bool sensorEnabled = true;
bool stationArmed = false;
unsigned long stationTick = 0;

// ----- dip behavior -----
const int DIP_SPEED = MAX_SPEED * 3.6 / 10; // 25MPH
const unsigned long DIP_TIME = 3600; // ms per dip

// -------- display --------
bool lastDirection = true;
int globalCurrentSpeed = 0;
char line1[64] = "STATUS";
char line2[64] = "0 MPH";
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

// -------- go! --------
void go(bool forward, int speed, unsigned long runTime, int dipCount) {
  Serial.println("GO!");
  //Serial.print(runTime);
  //Serial.println("s");

  unsigned long pauseTime = random(6, 20);

  setDirection(forward);
  rampSpeed(random(speed * 0.95, speed));

  if (dipCount > 0) {
    unsigned long segment = (runTime * 1000) / (dipCount + 1);
    for (int i = 0; i < dipCount; i++) {
      Serial.print("🟢 FAST LEG ⏱ ");
      Serial.print(segment / 1000);
      Serial.println("s");
      const char * msg;
      snprintf(line3, sizeof(line3), "%s %ds", "FAST LEG", segment / 1000);
      draw();
      delay(segment);

      rampSpeed(DIP_SPEED);
      Serial.print("🟡 SLOW LEG ⏱ ");
      Serial.print(DIP_TIME / 1000);
      Serial.println("s");
      snprintf(line3, sizeof(line3), "%s %ds", "SLOW LEG", DIP_TIME / 1000);
      draw();
      delay(DIP_TIME);

      rampSpeed(random(speed * 0.85, speed));
    }
    Serial.print("🟢 FAST LEG ⏱ ");
    Serial.print(segment / 1000);
    Serial.println("s");
    snprintf(line3, sizeof(line3), "%s %ds", "FAST LEG", segment / 1000);
    draw();
    delay(segment);

  } else {
    Serial.print("🟢 ONLY LEG ⏱ ");
    Serial.print(runTime);
    Serial.println("s");
    snprintf(line3, sizeof(line3), "%s %ds", "ONLY LEG", runTime);
    draw();
    delay(runTime * 1000);

  }
  Serial.print("🛑 STOP ⏱ ");
  Serial.print(pauseTime);
  Serial.println("s");
  snprintf(line3, sizeof(line3), "%s", "BRAKE TO HALT");
  draw();

  rampSpeed(0);
  stateStartTime = millis();
  unsigned long pauseMs = pauseTime * 1000;

  snprintf(line3, sizeof(line3), "%s %ds", "AT STATION", pauseTime);

  while (millis() - stateStartTime < pauseMs) {
    draw();
    updateStationLights();
  }

  setStationState(DEPARTING);
  updateStationLights();
  snprintf(line3, sizeof(line3), "%s", "NOW BOARDING");
  draw();
  unsigned long start = millis();
  while (millis() - start < 4000) {
    updateStationLights();
    draw(); // optional but keeps sync feel
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

void updateSignal(int speed, bool rampUp) {
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

const unsigned long ARRIVE_BLINK_MS   = 4000;
const unsigned long DEPART_BLINK_MS   = 4000;
const unsigned long HOLD_AFTER_LEAVE  = 3000;
const unsigned long FADE_MS           = 3000;

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
    digitalWrite(STN3_PIN, LOW);
    phase = 2;
  }
  if (phase == 2 && elapsed >= step * 2) {
    digitalWrite(STN1_PIN, LOW);
    phase = 3;
  }
  if (phase == 3 && elapsed >= step * 3) {
    digitalWrite(STN4_PIN, LOW);
    phase = 4;
  }
  if (phase == 4 && elapsed >= step * 4) {
    digitalWrite(STN2_PIN, LOW);
    phase = 0; // finished
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
  break;  case AT_STATION:
    allOn(); // Solid lights while stopped
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

  if (forward) {
    digitalWrite(in2Pin, LOW); // This stays Digital (The Ground)
    analogWrite(in1Pin, pwm); // This uses PWM (The Speed)
  } else {
    digitalWrite(in1Pin, LOW); // This stays Digital (The Ground)
    analogWrite(in2Pin, pwm); // This uses PWM (The Speed)
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
  if (target == 0) stationArmed = false;

  if (target != 0) dockedThisStop = false;
  if (current == 0 && target > 0)
    current = MIN_SPEED;

  if (target == current) {
    globalCurrentSpeed = current;
    return;
  }

  // if (current == 0 && target > 0) {
  //   writeMotor(lastDirection, 255);
  //   delay(1);
  // }

  int start = current;
  int delta = abs(target - start);
  bool rampUp = target > current;

  updateSignal((rampUp ? 1 : current), rampUp);

  while (current != target) {
    if (sensorEnabled && target == 0 && current < 100) {
      setStationState(ARRIVING);
    }
    updateStationLights();

    // ---- DOCKING LOGIC ----
    if (sensorEnabled && target == 0 && !dockedThisStop && current < DOCKING_SPEED) {
      Serial.println("HARD WAIT FOR SENSOR EDGE");
      updateStationLights();
      while (stationArmed == false) {
        int v = analogRead(IR_PIN);
        if (v < IR_THRESHOLD) {
          stationTick = millis();
          Serial.print(v);
          Serial.println(" TICK LOCKED");
          stationArmed = true;
          break;
        }
        updateStationLights();
      }


      if (sensorEnabled && target == 0 && !dockedThisStop) {

        // wait for tick once
        while (!stationArmed) {
          updateStationLights();
          int v = analogRead(IR_PIN);
          if (v < IR_THRESHOLD) {
            stationArmed = true;
            stationTick = millis();
            Serial.print(v);
            Serial.println(" TICK LOCKED");
          }
        }

unsigned long waitMs = calculateStationPause(lastDirection);

Serial.print("WAIT ");
Serial.println(waitMs);

unsigned long start = millis();

while (millis() - start < waitMs) {
    updateStationLights();   // keep blinking
    draw();                  // keep UI alive
}

        setStationState(AT_STATION);
        updateStationLights();

        dockedThisStop = true; // <-- prevent re-run
        Serial.println("DOCK COMPLETE");
      }

    }

    // ---- S-CURVE RAMP ----
    int progressed = abs(current - start);
    float phase = (delta == 0) ? 1.0 : (float) progressed / delta;
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

    snprintf(line2, sizeof(line2), "%d MPH", speedToMph(current));

    if (target == 0)
      updateSignal(current, rampUp);

    //snprintf(line3, sizeof(line3), "DOWN TO STOP");
    else
      snprintf(line3, sizeof(line3), "%s %d MPH",
        rampUp ? "RAMP TO" : "DOWN TO",
        speedToMph(target));

    writeMotor(lastDirection, current);
    draw();
    delay(RAMP_DELAY);
  }

  updateSignal(current, rampUp);
}

// -------- calibrate --------
// Calibration function
void calibrateTrain() {
  snprintf(line1, sizeof(line1), "CALIBRATE STATION");
  draw();
  rampSpeed(MAX_SPEED);

  // Measure FWD and REV times
  unsigned long lapFwd = measureLap(true);
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

  snprintf(line1, sizeof(line1), "DONE");
  draw();
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
    return fwdOffsetMs;
  } else {
    return ((revLoopMs * 0.51) + revOffsetMs) - fwdOffsetMs;
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
  p.fwdOffsetMs = fwdOffsetMs;
  p.revOffsetMs = revOffsetMs;

  EEPROM.put(0, p);  // Write the calibration data to EEPROM
}

// -------- routes --------
void pelhamRail() {
  snprintf(line1, sizeof(line1), "%s", "Taking Pelham 123");
  draw();
  bool dir = true;
  for (int i = 0; i < 4; i++) {
    go(dir, MAX_SPEED, 5, 0);
    dir = !dir;
  }
}

void readingRailroad() {
  snprintf(line1, sizeof(line1), "%s", "Reading Railroad");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED, 20, 0);
    dir = !dir;
  }
}

void grandCentral() {
  snprintf(line1, sizeof(line1), "%s", "Grand Central Line");
  draw();

  bool dir = true;
  int spd = random(MAX_SPEED * 0.75, MAX_SPEED);

  for (int i = 0; i < 4; i++) {
    dir = !dir;
    go(dir, spd, 40, 4);
  }
}

void hudsonLine() {
  snprintf(line1, sizeof(line1), "%s", "Hudson Limited");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED - 10, 20, 1);
    dir = !dir;
  }
}

void pennLine() {
  snprintf(line1, sizeof(line1), "%s", "Pennsylvania Line");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED - 10, 20, 1);
    dir = !dir;
  }
}

void vanderbiltCentral() {
  snprintf(line1, sizeof(line1), "%s", "Vanderbilt Central");
  draw();

  bool dir = true;

  for (int i = 0; i < 4; i++) {
    go(dir, MAX_SPEED - 4, 20, 1);
    dir = !dir;
  }
}

void bAndO() {
  snprintf(line1, sizeof(line1), "%s", "The B&O Railroad");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED, 16, 0);
    dir = !dir;
  }
}

void circleOfStops() {
  snprintf(line1, sizeof(line1), "%s", "The Circle Line");
  draw();

  bool dir = true;
  int spd = random(MAX_SPEED * 0.75, MAX_SPEED);

  for (int i = 0; i < 8; i++) {
    go(dir, spd, 16, 1);
    dir = !dir;
  }
}

void orientExpress() {
  snprintf(line1, sizeof(line1), "%s", "The Orient Express");
  draw();

  bool dir = true;
  int spd = random(MAX_SPEED * 0.75, MAX_SPEED);

  for (int i = 0; i < 4; i++) {
    go(dir, spd, 16, 1);
    dir = !dir;
  }
}

void jessTrain() {
  snprintf(line1, sizeof(line1), "%s", "Rio-Jess Express");
  draw();

  for (int i = 0; i < 4; i++) {
    bool dir = (i % 2);
    int spd = MAX_SPEED;
    go(dir, spd,
      random(20, 60),
      random(2, 5));
  }
}

void longTrainRunning() {
  snprintf(line1, sizeof(line1), "%s", "Long Train Running");
  draw();

  int spd = random(MAX_SPEED * 0.85, MAX_SPEED);

  for (int i = 0; i < 2; i++) {
    go(true, spd, 43, 4);
    go(false, spd, 43, 4);
  }
}

void gentleWander() {
  snprintf(line1, sizeof(line1), "%s", "Union Pacific R.R.");
  draw();

  for (int i = 0; i < 15; i++) {
    bool dir = (i % 2);
    int spd = random(MAX_SPEED * 0.65, MAX_SPEED * 0.75);
    int dips = random(5, 9);
    go(dir, spd,
      random(80, 105),
      dips);
  }
}

void silverStreak() {
  snprintf(line1, sizeof(line1), "%s", "The Silver Streak");
  draw();

  for (int i = 0; i < 4; i++) {
    bool dir = (i % 2);
    go(dir, random(MAX_SPEED * 0.85, MAX_SPEED),
      20,
      0);
  }
}

// -------- setup --------
void setup() {
  Serial.begin(115200);

  Serial.println("Serial Established.");

  Serial.println("Motor Controller Pin Modes");
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);

  Serial.println("Initialize Display Driver.");
  u8g2.begin();
  u8g2.clearBuffer();

  Serial.println("Calibrate Train.");
  snprintf(line1, sizeof(line1), "CALIBRATING");
  draw();

  // Load the saved calibration settings from EEPROM
  loadFromEEPROM();

  // Always run calibration on startup
  calibrateTrain();

  // Display finished setup message
  snprintf(line1, sizeof(line1), "DONEKSI");
  draw();

  Serial.println("Station Light Setup.");
  pinMode(STN1_PIN, OUTPUT);
  pinMode(STN2_PIN, OUTPUT);
  pinMode(STN3_PIN, OUTPUT);
  pinMode(STN4_PIN, OUTPUT);

  Serial.println("Traffic Light Setup.");
  pinMode(RED_PIN, OUTPUT);
  pinMode(YEL_PIN, OUTPUT);
  pinMode(GRN_PIN, OUTPUT);

  Serial.println("IR Sensor Setup.");
  pinMode(IR_PIN, INPUT);

  Serial.println("BOOT");
}

// -------- draw --------
void toUpper(char * s) {
  for (;* s; s++) {
    if ( * s >= 'a' && * s <= 'z') * s -= 32;
  }
}

void draw() {
  u8g2.firstPage();
  do {
    u8g2.clearBuffer();

    // ---- TOP CENTERED: TITLE (small, ALL CAPS, long) ----
    toUpper(line1);
    u8g2.setFont(u8g2_font_7x13_tr);
    u8g2.drawStr(
      (128 - u8g2.getStrWidth(line1)) / 2,
      9,
      line1
    );
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_logisoso32_tn);
    u8g2.drawBox(0, 14, 128, 36);

    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_logisoso32_tn);
    int numWidth = u8g2.getStrWidth(line2);
    int numRightEdge = 69;
    u8g2.drawStr(
      numRightEdge - numWidth,
      48,
      line2
    );

    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(55, 48, "MPH");
    const char * statusStr;
    if (globalCurrentSpeed == 0) {
      statusStr = "HALTED";
    } else {
      statusStr = lastDirection ? "UPTOWN" : "DOWNTOWN";
    }
    u8g2.setFont(u8g2_font_7x13_tr);
    u8g2.drawStr(
      58,
      26,
      statusStr
    );

    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_9x15_tr);
    u8g2.drawStr(
      (130 - u8g2.getStrWidth(line3)) / 2,
      64,
      line3
    );

  } while (u8g2.nextPage());
}

// -------- loop --------
void loop() {
  Serial.println("LOOP START");
  pelhamRail();
  vanderbiltCentral();
  gentleWander();
  pennLine();
  hudsonLine();
  grandCentral();
  readingRailroad();
  silverStreak();
  bAndO();
  jessTrain();
  orientExpress();
  circleOfStops();
  longTrainRunning();
}