// -------- pins --------
const int speedPin = 9;   // PWMA  (speed / PWM)
const int dirPin   = 8;   // AIN1  (direction)
const int dirPin2  = 7;   // AIN2  (direction)
const int stbyPin  = 6;   // STBY  (enable)

// -------- tuning --------
const int MAX_SPEED   = 130;   // safer ceiling for 12V
const int RAMP_STEP   = 2;
const int RAMP_DELAY  = 50;    // ms
const int MIN_SPEED   = MAX_SPEED * 2 / 10;  // ~20%

// -------- dip behavior --------
const int DIP_SPEED = MAX_SPEED * 4 / 9;  // ~44%
const unsigned long DIP_TIME = 2500;     // ms per dip

// -------- forward declaration --------
void go(bool forward,
        int speed,
        unsigned long runTime,
        unsigned long pauseTime,
        int dipCount = 0);

// -------- helpers --------
void setDirection(bool forward) {
  if (forward) {
    digitalWrite(dirPin, HIGH);
    digitalWrite(dirPin2, LOW);
  } else {
    digitalWrite(dirPin, LOW);
    digitalWrite(dirPin2, HIGH);
  }
}

void rampSpeed(int target) {
  static int current = 0;
  target = constrain(target, target == 0 ? 0 : MIN_SPEED, MAX_SPEED);

  if (target == current) return;

  int start = current;
  int delta = abs(target - start);

  Serial.print(target > current ? "S-RAMP UP → " : "S-RAMP DOWN → ");
  Serial.println(target);

  while (current != target) {
    int progressed = abs(current - start);
    float phase = (float)progressed / delta;   // 0.0 → 1.0

    // S-curve: gentle start, faster middle, gentle end
    int step = max(1, (int)(RAMP_STEP * (0.5 + 1.5 * phase * (1 - phase))));

    current += (current < target) ? step : -step;

    // clamp to target
    if ((current < target && current > target) ||
        (current > target && current < target)) {
      current = target;
    }

    analogWrite(speedPin, current);
    delay(RAMP_DELAY);
  }

  if (current == 0) Serial.println("STOPPED");
  else if (current == MAX_SPEED) Serial.println("MAX SPEED");
  else {
    Serial.print("AT SPEED ");
    Serial.println(current);
  }
}
// -------- core primitive --------
void go(bool forward,
        int speed,
        unsigned long runTime,
        unsigned long pauseTime,
        int dipCount) {

  speed = constrain(speed, 0, MAX_SPEED);

  Serial.print("GO ");
  Serial.print(forward ? "Forward" : "Reverse");
  Serial.print(" | speed ");
  Serial.print(speed);
  Serial.print(" | dips ");
  Serial.println(dipCount);

  setDirection(forward);
  rampSpeed(speed);

  if (dipCount > 0) {
    unsigned long slice = runTime / (dipCount + 1) * 3;

    for (int i = 0; i < dipCount; i++) {
      delay(slice);
      rampSpeed(DIP_SPEED);
      delay(DIP_TIME);
      rampSpeed(speed);
    }
    delay(slice);
  } else {
    delay(runTime);
  }

  rampSpeed(0);
  delay(pauseTime);
}

// -------- behaviors --------
void theLongRun() {
  Serial.println("THE LONG RUN (seasoning)");

  const unsigned long ONE_HOUR = 60UL * 60UL * 1000UL;
  const int SEASON_SPEED = MAX_SPEED * 9 / 10; // ~70%

  // Forward 45 min
  setDirection(true);
  rampSpeed(SEASON_SPEED);
  delay(ONE_HOUR / 2);

  // Reverse 45 min
  setDirection(false);
  rampSpeed(SEASON_SPEED);
  delay(ONE_HOUR / 2);

  rampSpeed(0);
  Serial.println("THE LONG RUN COMPLETE");
}

void circleOfStops() {
  Serial.println("Circle Of Stops");
  bool dir = true;

  for (int i = 0; i < 6; i++) {
    go(dir, MAX_SPEED, 6000, 5000, 1);  // one slow dip
    dir = !dir;
  }
}

void longTrainRunning() {
  Serial.println("Long Train Running");
  for (int i = 0; i < 3; i++) {
    go(true,  150, 20000, 6000, 2);
    go(false, 150, 20000, 6000, 2);
  }
}

void gentleWander() {
  Serial.println("Gentle Wander");
  for (int i = 0; i < 5; i++) {
    bool dir = random(0, 2);
    int spd  = MAX_SPEED;
    int dips = random(1, 3);   // 0–2 dips
    go(dir, spd,
       random(8000, 14000),
       random(4000, 7000),
       dips);
  }
}

// -------- setup --------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(speedPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  digitalWrite(stbyPin, HIGH); // enable driver

  Serial.println("BOOT");
}

// -------- loop --------
void loop() {
  Serial.println("LOOP START");

  theLongRun();
  circleOfStops();
  longTrainRunning();
  gentleWander();

  Serial.println("LOOP END");
}

