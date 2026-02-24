#include <SPI.h>

#include <U8g2lib.h>

#define CS_PIN 10
#define DC_PIN 9
#define RST_PIN 8

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
const int MAX_SPEED = 160;
const int RAMP_STEP = 5;
const int RAMP_DELAY = 1;
const int MIN_SPEED = 0;
const float MAX_MPH = 72.0;

// ------- station ---------
unsigned long MS_FWD = 290;
unsigned long MS_REV = 2650;
bool sensorEnabled = true;
bool stationArmed = false;
unsigned long stationTick = 0;

// ----- dip behavior -----
const int DIP_SPEED = MAX_SPEED * 3.6 / 10; // 25MPH
const unsigned long DIP_TIME = 4000; // ms per dip

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
void go(bool forward, int speed, unsigned long runTime, unsigned long pauseTime, int dipCount) {
  Serial.println("GO!");
  //Serial.print(runTime);
  //Serial.println("s");

  setDirection(forward);
  rampSpeed(speed);

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

      rampSpeed(speed);
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
  snprintf(line3, sizeof(line3), "%s %ds", "FULL STOP", pauseTime);
  draw();

  rampSpeed(0);
  setStationState(AT_STATION);
  stateStartTime = millis();
  unsigned long pauseMs = pauseTime * 1000;

  snprintf(line3, sizeof(line3), "%s %ds", "AT STATION", pauseTime);

  while (millis() - stateStartTime < pauseMs) {
    draw();
    updateStationLights();
  }

  setStationState(DEPARTING);
  updateStationLights();
  snprintf(line3, sizeof(line3), "%s", "NOW DEPARTING");
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
  //delay(300);

  digitalWrite(RED_PIN, HIGH);
}

void signalYellow() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GRN_PIN, LOW);
  //delay(300);

  digitalWrite(YEL_PIN, HIGH);
}

void signalGreen() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);
  //delay(300);

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
const unsigned long HOLD_AFTER_LEAVE  = 2000;
const unsigned long FADE_MS           = 2000;

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

  int start = current;
  int delta = abs(target - start);
  bool rampUp = target > current;

  updateSignal((rampUp ? 1 : current), rampUp);

  while (current != target) {
    updateStationLights();

    // ---- DOCKING LOGIC ----
    if (sensorEnabled && target == 0 && !dockedThisStop && current < 55) {
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
        if (current < 40) {
          setStationState(ARRIVING);
        }
        updateStationLights();
      }


      if (sensorEnabled && target == 0 && !dockedThisStop) {

        // wait for tick once
        while (!stationArmed) {
          int v = analogRead(IR_PIN);
          if (v < IR_THRESHOLD) {
            stationArmed = true;
            stationTick = millis();
            Serial.print(v);
            Serial.println(" TICK LOCKED");
          }
        }

unsigned long waitMs = lastDirection ? MS_FWD : MS_REV;

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

// -------- routes --------
void pelhamRail() {
  snprintf(line1, sizeof(line1), "%s", "Taking Pelham 123");
  draw();

  // go(true, MAX_SPEED, 5, 1, 0); 
  // go(false, MAX_SPEED, 5, 1, 0); 
  // go(true, MAX_SPEED, 5, 1, 0); 
  // go(false, MAX_SPEED, 5, 1, 0); 
  // go(true, MAX_SPEED, 5, 1, 0); 
  // go(false, MAX_SPEED, 5, 1, 0); 
  // go(true, MAX_SPEED, 5, 1, 0); 
  // go(false, MAX_SPEED, 5, 1, 0); 
  go(true, MAX_SPEED, 5, 10, 0);
  go(false, MAX_SPEED, 5, 10, 0);
}

void readingRailroad() {
  snprintf(line1, sizeof(line1), "%s", "Reading Railroad");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED, 20, 12, 0);
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
    go(dir, spd, 40, 6, 4);
  }
}

void hudsonLine() {
  snprintf(line1, sizeof(line1), "%s", "Hudson Limited");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED - 10, 20, 12, 1);
    dir = !dir;
  }
}

void pennLine() {
  snprintf(line1, sizeof(line1), "%s", "Pennsylvania Line");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED - 10, 20, 18, 1);
    dir = !dir;
  }
}

void vanderbiltCentral() {
  snprintf(line1, sizeof(line1), "%s", "Vanderbilt Central");
  draw();

  bool dir = true;

  for (int i = 0; i < 4; i++) {
    go(dir, MAX_SPEED - 4, 20, 12, 1);
    dir = !dir;
  }
}

void bAndO() {
  snprintf(line1, sizeof(line1), "%s", "The B&O Railroad");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED, 16, 16, 0);
    dir = !dir;
  }
}

void circleOfStops() {
  snprintf(line1, sizeof(line1), "%s", "The Circle Line");
  draw();

  bool dir = true;
  int spd = random(MAX_SPEED * 0.75, MAX_SPEED);

  for (int i = 0; i < 8; i++) {
    go(dir, spd, 16, 24, 1);
    dir = !dir;
  }
}

void orientExpress() {
  snprintf(line1, sizeof(line1), "%s", "The Orient Express");
  draw();

  bool dir = true;
  int spd = random(MAX_SPEED * 0.75, MAX_SPEED);

  for (int i = 0; i < 4; i++) {
    go(dir, spd, 16, 16, 1);
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
      25,
      random(2, 5));
  }
}

void longTrainRunning() {
  snprintf(line1, sizeof(line1), "%s", "Long Train Running");
  draw();

  int spd = random(MAX_SPEED * 0.85, MAX_SPEED);

  for (int i = 0; i < 2; i++) {
    go(true, spd, 43, 12, 4);
    go(false, spd, 43, 12, 4);
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
      random(16, 36),
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
      18,
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
  circleOfStops();
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
  longTrainRunning();
}