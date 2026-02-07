#include <SPI.h>
#include <U8g2lib.h>

#define CS_PIN   10
#define DC_PIN   9
#define RST_PIN  8

//U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, CS_PIN, DC_PIN, RST_PIN);
U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, 10, 9, 8);
// -------- pins --------
const int in1Pin = 5; 
const int in2Pin = 6; 

const int RED_PIN = 2;
const int YEL_PIN = 3;
const int GRN_PIN = 4;

// -------- tuning --------
const int MAX_SPEED   = 42;   // safer ceiling for 12V
const int RAMP_STEP   = 1;
const int RAMP_DELAY  = 100;    // ms
const int MIN_SPEED   = 8;  
const float MAX_MPH = 72.0;   // calibrate once, then trust it

// -------- dip behavior --------
const int DIP_SPEED = MAX_SPEED * 4 / 9;  // ~44%
const unsigned long DIP_TIME = 3600;     // ms per dip

// -------- forward declaration --------
int speedToMph(int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  return (pwm * MAX_MPH) / MAX_SPEED;   // integer math
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
  setDirection(forward);
  
  Serial.print("🟢 LOOP ⏱ ");
  Serial.print(runTime);
  Serial.println("s");
  // snprintf(line3, sizeof(line3), "%s %ds", "LOOP", runTime);
  // draw();

  rampSpeed(speed);

  // Handle speed "dips" during the run (simulates slowing for curves/stations)
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
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(YEL_PIN, LOW);
  digitalWrite(GRN_PIN, LOW);
}

void signalGreen() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, LOW);
  digitalWrite(GRN_PIN, HIGH);
}

void signalYellow() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YEL_PIN, HIGH);
  digitalWrite(GRN_PIN, LOW);
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

  // normalize the REQUEST
  if (target != 0 && abs(target) < MIN_SPEED) target = 0;
  target = constrain(target, 0, MAX_SPEED);

  if (target == current) return;

  int start = current;
  int delta = abs(target - start);

  Serial.print(target > current ? "🔼 RAMP " : "🔽 RAMP ");
  Serial.print(speedToMph(target), 1);
  
  updateSignal(current, (target > current));

  while (current != target) {
    int progressed = abs(current - start);
    float phase = (float)progressed / delta;
    int step = max(1, (int)(RAMP_STEP * (0.5 + 1.5 * phase * (1 - phase))));
    current += (current < target) ? step : -step;
    if ((start < target && current > target) ||
        (start > target && current < target)) {
      current = target;
    }
    snprintf(line2, sizeof(line2), "%d MPH", speedToMph(current));
    const char* action = (target > current) ? "RAMP TO" : "DOWN TO";
    if (target == 0) {
      snprintf(line3, sizeof(line3), "%s STOP", action);
    } else {
      snprintf(line3, sizeof(line3), "%s %dMPH", action, speedToMph(target));
    }
    writeMotor(lastDirection, current);
    draw();
    delay(RAMP_DELAY);
  }
}

// -------- routes --------

void bAndO() {
  snprintf(line1, sizeof(line1), "%s", "The B&O Railroad");
  draw();

  bool dir = true;

  for (int i = 0; i < 2; i++) {
    go(dir, MAX_SPEED, 16, 8, 0);  // one slow dip
    dir = !dir;
  }
}

void circleOfStops() {
  snprintf(line1, sizeof(line1), "%s", "The Circle Line");
  draw();

  bool dir = true;
  int spd = random(40, MAX_SPEED + 1);

  for (int i = 0; i < 8; i++) {
    go(dir, spd, 16, 8, 1);  // one slow dip
    dir = !dir;
  }
}

void orientExpress() {
  snprintf(line1, sizeof(line1), "%s", "The Orient Express");
  draw();

  bool dir = true;
  int spd = random(40, MAX_SPEED + 1);

  for (int i = 0; i < 4; i++) {
    go(dir, spd, 16, 8, 1);  // one slow dip
    dir = !dir;
  }
}

void jessicaLovesTrains() {
  snprintf(line1, sizeof(line1), "%s", "Jessica Central");
  draw();

  for (int i = 0; i < 4; i++) {
    bool dir = (i % 2);
    int spd = MAX_SPEED;
    go(dir, spd,
       random(20, 60),
       6,
       random(2, 4));
  }
}

void longTrainRunning() {
  snprintf(line1, sizeof(line1), "%s", "New York Central");
  draw();

  int spd = random(MAX_SPEED - 20, MAX_SPEED + 1);

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
    int spd = random(20, MAX_SPEED - 10);
    int dips = random(4, 9);
    go(dir, spd,
       random(45, 95),
       random(2, 6),
       dips);
  }
}

void silverStreak() {
  snprintf(line1, sizeof(line1), "%s", "The Silver Streak");
  draw();

  for (int i = 0; i < 4; i++) {
    bool dir = (i % 2);
    go(dir, random(40, MAX_SPEED),
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
    u8g2.setFont(u8g2_font_logisoso32_tn);
    u8g2.drawStr(10, 48, line2);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(55, 48, "MPH");

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
  bAndO();
  jessicaLovesTrains();
  silverStreak();
  orientExpress();
  longTrainRunning();
  circleOfStops();
  gentleWander();
  circleOfStops();
  orientExpress();
  jessicaLovesTrains();
  gentleWander();
  Serial.println("LOOP END");
}

