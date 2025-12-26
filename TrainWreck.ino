// -------- pins --------
const int speedPin = 9;   // PWMA  (speed / PWM)
const int dirPin   = 8;   // AIN1  (direction)
const int dirPin2  = 7;   // AIN2  (direction)
const int stbyPin  = 6;   // STBY  (enable)

// -------- tuning --------
const int MAX_SPEED   = 130;   // safer ceiling for 12V
const int RAMP_STEP   = 1;
const int RAMP_DELAY  = 60;    // ms
const int MIN_SPEED   = 10;

// -------- dip behavior --------
const unsigned long DIP_TIME = 4500;     // ms per dip

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
if ((start < target && current > target) ||
    (start > target && current < target)) {
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
      int dipSpeed = speed * 4 / 10; // 40% of current speed
      rampSpeed(dipSpeed);
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

  const unsigned long MINUTES = 20UL * 60UL * 1000UL; // 20 mins
  const int SEASON_SPEED = MAX_SPEED * 8 / 10; // ~80%

  // Reverse 
  setDirection(false);
  rampSpeed(SEASON_SPEED);
  delay(MINUTES / 2);

  // Forward 
  setDirection(true);
  rampSpeed(SEASON_SPEED);
  delay(MINUTES / 2);

  rampSpeed(0);
  Serial.println("THE LONG RUN COMPLETE");
}

void circleOfStops() {
  Serial.println("Circle Of Stops");
  bool direction = true;

  int speed = random(80, MAX_SPEED);
  for (int i = 0; i < 6; i++) {
    go(direction, speed, 6000, 5000, 1);  // one slow dip
    direction = !direction;
  }
}

void longTrainRunning() {
  Serial.println("Long Train Running");
  for (int i = 0; i < 3; i++) {
    go(true,  MAX_SPEED, 20000, 6000, 2);
    go(false, MAX_SPEED, 20000, 6000, 2);
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
static bool didLongRun = false;

void loop() {
  Serial.println("LOOP START");

  if (!didLongRun) {
    Serial.println("STARTING LONG RUN");
    theLongRun();
    didLongRun = true;
  }

  Serial.println("CYCLES BEGIN");

  for (int i = 0; i < 5; i++) {
    Serial.print("CYCLE ");
    Serial.println(i + 1);

    circleOfStops();
    longTrainRunning();
    gentleWander();
  }

  Serial.println("CYCLES COMPLETE — RESTING");
  
  const unsigned long LONG_REST = 20UL * 60UL * 1000UL; // 20 minutes
  delay(LONG_REST);

  Serial.println();
}

