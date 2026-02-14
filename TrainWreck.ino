#include <SPI.h>
#include <U8g2lib.h>

#define CS_PIN   10
#define DC_PIN   9
#define RST_PIN  8

//U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, CS_PIN, DC_PIN, RST_PIN);
U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, 10, 9, 8);
// -------- pins --------

const int IR_PIN = A0;
const int IR_THRESHOLD = 420;

const int in1Pin = 5; 
const int in2Pin = 6; 

const int RED_PIN = 2;
const int YEL_PIN = 3;
const int GRN_PIN = 4;

// -------- tuning --------
const int MAX_SPEED   = 180; 
const int RAMP_STEP   = 5;
const int RAMP_DELAY  = 5;
const int MIN_SPEED   = 20;  
const float MAX_MPH = 72.0;

unsigned long MS_FWD = 180;   
unsigned long MS_REV = 200;   
bool sensorEnabled = false;

bool stationArmed = false;
unsigned long stationTick = 0;

// -------- dip behavior --------
const int DIP_SPEED = MAX_SPEED * 4 / 9;
const unsigned long DIP_TIME = 3600; // ms per dip

// -------- forward declaration --------
int speedToMph(int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  return (pwm * MAX_MPH) / MAX_SPEED;
}

// -------- display --------

bool lastDirection = true;
// Global slots for  3 lines of text
char line1[64] = "STATUS";
char line2[64] = "0 MPH";
char line3[64] = "READY";

// -------- go! --------

void go(bool forward,
        int speed,
        unsigned long runTime,
        unsigned long pauseTime,
        int dipCount = 0);

void go(bool forward, int speed, unsigned long runTime, unsigned long pauseTime, int dipCount) {
  Serial.print("🟢 LOOP ⏱ ");
  Serial.print(runTime);
  Serial.println("s");

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
      draw();
      delay(segment);

      rampSpeed(DIP_SPEED);
      Serial.print("🟡 SLOW LEG ⏱ ");
      Serial.print(DIP_TIME/1000);
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
  rampSpeed(0);
  Serial.print("🛑 STOP ⏱ ");
  Serial.print(pauseTime);
  Serial.println("s");
  snprintf(line3, sizeof(line3), "%s %ds", "FULL STOP", pauseTime);
  draw();
  delay(pauseTime * 1000);

}

// -------- signal --------

void signalRed() {
  //LED A
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YEL_PIN, LOW);
  digitalWrite(GRN_PIN, LOW);
}

void signalYellow() {
  //LED B
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, HIGH);
  digitalWrite(GRN_PIN, LOW);
}

void signalGreen() {
  //LED C
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

// -------- motor --------

void writeMotor(bool forward, int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);

  if (forward) {
    analogWrite(in1Pin, pwm);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    analogWrite(in2Pin, pwm);
  }
}

// -------- direction --------

void setDirection(bool forward) {
  lastDirection = forward;
}

// -------- ramp --------

void rampSpeed(int target) {
  static int current = 0;

  if (target != 0 && abs(target) < MIN_SPEED) target = 0;
  target = constrain(target, 0, MAX_SPEED);
  if (target == current) return;

  int start = current;
  int delta = abs(target - start);
  bool rampUp = target > current;

  Serial.print(target > current ? "🔼 RAMP " : "🔽 RAMP ");
  Serial.print(speedToMph(target), 1);

  updateSignal((rampUp ? 1 : current), rampUp);

  while (current != target) {

    if (sensorEnabled && target == 0) {
      int v = analogRead(IR_PIN);

      if (!stationArmed && v > IR_THRESHOLD) {
        stationArmed = true;
        stationTick = millis();
//        Serial.println(" 🚉 TICK ", stationTick);
      }

      if (stationArmed) {
        unsigned long waitMs = (lastDirection > 0) ? MS_FWD : MS_REV;
        if (millis() - stationTick < waitMs) {
          writeMotor(lastDirection, current);
          draw();
          delay(RAMP_DELAY);
          continue;
        }
        stationArmed = false;
      }
    }

    int progressed = abs(current - start);
    float phase = (delta == 0) ? 1.0 : (float)progressed / delta;
    int step = max(1, (int)(RAMP_STEP * (0.5 + 1.5 * phase * (1 - phase))));

    current += (current < target) ? step : -step;

    if ((start < target && current > target) || (start > target && current < target)) {
      current = target;
    }

    snprintf(line2, sizeof(line2), "%d MPH", speedToMph(current));
    if (target == 0) {
      snprintf(line3, sizeof(line3), "%s STOP", rampUp ? "RAMP TO" : "DOWN TO");
    } else {
      snprintf(line3, sizeof(line3), "%s %d MPH", rampUp ? "RAMP TO" : "DOWN TO", speedToMph(target));
    }

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

  bool dir = true;
  int spd = random(36, MAX_SPEED + 1);

  for (int i = 0; i < 2; i++) {
    go(dir, spd, 20, 6, 0); 
    dir = !dir;
  }
}

void readingRailroad() {
  snprintf(line1, sizeof(line1), "%s", "Reading Railroad");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED, 20, 6, 0); 
    dir = !dir;
  }
}

void grandCentral() {
  snprintf(line1, sizeof(line1), "%s", "Grand Central Line");
  draw();

  bool dir = true;
  int spd = random(35, MAX_SPEED);

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
    go(dir, MAX_SPEED - 10, 20, 6, 1); 
    dir = !dir;
  }
}

void pennLine() {
  snprintf(line1, sizeof(line1), "%s", "Pennsylvania Line");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED - 10, 20, 6, 1); 
    dir = !dir;
  }
}

void vanderbiltCentral() {
  snprintf(line1, sizeof(line1), "%s", "Vanderbilt Central");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED - 4, 20, 6, 1); 
    dir = !dir;
  }
}

void bAndO() {
  snprintf(line1, sizeof(line1), "%s", "The B&O Railroad");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED, 16, 8, 0); 
    dir = !dir;
  }
}

void circleOfStops() {
  snprintf(line1, sizeof(line1), "%s", "The Circle Line");
  draw();

  bool dir = true;
  int spd = random(34, MAX_SPEED + 1);

  for (int i = 0; i < 8; i++) {
    go(dir, spd, 16, 8, 1); 
    dir = !dir;
  }
}

void orientExpress() {
  snprintf(line1, sizeof(line1), "%s", "The Orient Express");
  draw();

  bool dir = true;
  int spd = random(40, MAX_SPEED + 1);

  for (int i = 0; i < 4; i++) {
    go(dir, spd, 16, 8, 1); 
    dir = !dir;
  }
}

void jessicaLovesTrains() {
  snprintf(line1, sizeof(line1), "%s", "The Jessica Line");
  draw();

  for (int i = 0; i < 4; i++) {
    bool dir = (i % 2);
    int spd = MAX_SPEED;
    go(dir, spd,
       random(20, 60),
       6,
       random(2, 5));
  }
}

void longTrainRunning() {
  snprintf(line1, sizeof(line1), "%s", "Long Train Running");
  draw();

  int spd = random(MAX_SPEED - 10, MAX_SPEED + 1);

  for (int i = 0; i < 2; i++) {
    go(true,  spd, 43, 6, 4);
    go(false, spd, 43, 6, 4);
  }
}

void gentleWander() {
  snprintf(line1, sizeof(line1), "%s", "Union Pacific");
  draw();

  for (int i = 0; i < 5; i++) {
    bool dir = (i % 2);
    int spd = random(28, MAX_SPEED - 5);
    int dips = random(4, 9);
    go(dir, spd,
       random(80, 105),
       random(2, 6),
       dips);
  }
}

void silverStreak() {
  snprintf(line1, sizeof(line1), "%s", "The Silver Streak");
  draw();

  for (int i = 0; i < 4; i++) {
    bool dir = (i % 2);
    go(dir, random(38, MAX_SPEED),
       10,
       6,
       0);
  }
}

// -------- setup --------
void setup() {
  Serial.begin(115200);

  Serial.println("Serial Established.");

  Serial.println("Set pin modes early...");
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);

  u8g2.begin(); 
  u8g2.clearBuffer();   

  Serial.println("Display initialized.");

  Serial.println("BOOT");
}

// -------- draw --------

void toUpper(char* s) {
  for (; *s; s++) {
    if (*s >= 'a' && *s <= 'z') *s -= 32;
  }
}

void draw() {
  u8g2.firstPage();
  do {
    u8g2.clearBuffer();
    toUpper(line1);
    // ---- TOP: TITLE (small, ALL CAPS, long) ----
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
    // u8g2.setFont(u8g2_font_logisoso32_tn);
    // u8g2.drawStr(10, 48, line2);

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
    const char* dir = lastDirection ? "FORWARD" : "REVERSE";
    u8g2.setFont(u8g2_font_7x13_tr);
    u8g2.drawStr(
      58,
      26,
      dir
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
  pennLine();
  hudsonLine();
  grandCentral();
  readingRailroad();
  silverStreak();
  bAndO();
  jessicaLovesTrains();
  orientExpress();
  longTrainRunning();
  circleOfStops();
  gentleWander();
  Serial.println("LOOP END");
}

