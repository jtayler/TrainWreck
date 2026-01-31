#include <SPI.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMono9pt7b.h>

#define CS_PIN   3
#define DC_PIN   8
#define RST_PIN  5
#define BUSY_PIN 4

GxEPD2_3C<GxEPD2_290_C90c, 16> display(
  GxEPD2_290_C90c(CS_PIN, DC_PIN, RST_PIN, BUSY_PIN)
);

// -------- pins --------
const int in1Pin = 9;   // PWM
const int in2Pin = 10;  // PWM

// -------- tuning --------
const int MAX_SPEED   = 50;   // safer ceiling for 12V
const int RAMP_STEP   = 1;
const int RAMP_DELAY  = 180;    // ms
const int MIN_SPEED   = 8;  
const float MAX_MPH = 68.0;   // calibrate once, then trust it

// -------- dip behavior --------
const int DIP_SPEED = MAX_SPEED * 4 / 9;  // ~44%
const unsigned long DIP_TIME = 3600;     // ms per dip

// -------- forward declaration --------
float speedToMph(int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  return (pwm / (float)MAX_SPEED) * MAX_MPH;
}

bool lastDirection = true;

void go(bool forward,
        int speed,
        unsigned long runTime,
        unsigned long pauseTime,
        int dipCount = 0);

// -------- helpers --------

int lastDrawnMPH = -1;
bool displayBusy = false;
unsigned long lastDrawTime = 0;
const unsigned long DISPLAY_MIN_INTERVAL = 15000; // 15s

void updateMPH(float mph) {
  if (displayBusy) return;

  int rounded = (int)(mph + 0.5);
  if (rounded == lastDrawnMPH) return;

  displayBusy = true;
  lastDrawnMPH = rounded;

  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMono9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(10, 30);
    display.print("MPH");
    display.setCursor(10, 65);
    display.print(rounded);
  } while (display.nextPage());

  displayBusy = false;
}

void go(bool forward, int speed, unsigned long runTime, unsigned long pauseTime, int dipCount) {
  setDirection(forward);
  
  // Ramp up
      Serial.print("🟢 LOOP ⏱ ");
      Serial.print(runTime);
      Serial.println("s");
      rampSpeed(speed);
      //updateMPH(speedToMph(speed));

  // Handle speed "dips" during the run (simulates slowing for curves/stations)
  if (dipCount > 0) {
    unsigned long segment = (runTime * 1000) / (dipCount + 1);
    for (int i = 0; i < dipCount; i++) {
      Serial.print("🟢 FAST LEG ⏱ ");
      Serial.print(segment / 1000);
      Serial.println("s");
      delay(segment);
      rampSpeed(DIP_SPEED);
      Serial.print("🟡 SLOW LEG ⏱ ");
      Serial.print(DIP_TIME/1000);
      Serial.println("s");
      delay(DIP_TIME);
      rampSpeed(speed);
    }
      Serial.print("🟢 FAST LEG ⏱ ");
      Serial.print(segment / 1000);
      Serial.println("s");
    delay(segment);

  } else {
      Serial.print("🟢 ONLY LEG ⏱ ");
      Serial.print(runTime);
      Serial.println("s");
    delay(runTime * 1000);
  }

  rampSpeed(0);
  Serial.print("🛑 STOP ⏱ ");
  Serial.print(pauseTime);
  Serial.println("s");
  delay(pauseTime * 1000);
}

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

void setDirection(bool forward) {
  lastDirection = forward;
}

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
  Serial.println(" MPH");

  while (current != target) {
    int progressed = abs(current - start);
    float phase = (float)progressed / delta;

    int step = max(1, (int)(RAMP_STEP * (0.5 + 1.5 * phase * (1 - phase))));

    current += (current < target) ? step : -step;

    if ((start < target && current > target) ||
        (start > target && current < target)) {
      current = target;
    }

    writeMotor(lastDirection, current);
    delay(RAMP_DELAY);
  }
}

void circleOfStops() {
  Serial.println("🔁 Circle Of Stops");
  bool dir = true;
  int spd = random(40, MAX_SPEED + 1);

  for (int i = 0; i < 6; i++) {
    go(dir, spd, 16, 8, 1);  // one slow dip
    dir = !dir;
  }
}

void longTrainRunning() {
  Serial.println("🔁 Long Train Running");
  int spd = random(40, MAX_SPEED + 1);

  for (int i = 0; i < 3; i++) {
    go(true,  spd, 26, 6, 2);
    go(false, spd, 14, 6, 2);
  }
}

void gentleWander() {
  Serial.println("🔁 Gentle Wander");
  for (int i = 0; i < 5; i++) {
    bool dir = random(0, 2);
    int spd = random(40, MAX_SPEED + 1);
    int dips = random(1, 3);   // 0–2 dips
    go(dir, spd,
       random(18, 14),
       random(14, 7),
       dips);
  }
}

// -------- setup --------
void setup() {
  Serial.begin(115200);
  delay(1);
  
  randomSeed(analogRead(A0));

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  display.init(115200, false, 300, true);
  display.setFullWindow();

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  Serial.println(MCUSR, HEX);
  MCUSR = 0;

  //digitalWrite(dirPin, LOW);

  Serial.println("BOOT");
}

// -------- loop --------

void loop() {
  Serial.println("");
  Serial.println("LOOP START");

// digitalWrite(dirPin, LOW);
// analogWrite(speedPin, 80);
// delay(10000);
// analogWrite(speedPin, 0);
// delay(2000);

  circleOfStops();
  longTrainRunning();
  gentleWander();

  Serial.println("LOOP END");
}

