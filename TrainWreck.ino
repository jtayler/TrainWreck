// -------- pins --------
const int in1Pin = 9;   // PWM
const int in2Pin = 10;  // PWM

// -------- tuning --------
const int MAX_SPEED   = 80;   // safer ceiling for 12V
const int RAMP_STEP   = 1;
const int RAMP_DELAY  = 80;    // ms
const int MIN_SPEED   = 8;  
const float MAX_MPH = 68.0;   // calibrate once, then trust it

// -------- dip behavior --------
const int DIP_SPEED = MAX_SPEED * 3 / 9;  // ~44%
const unsigned long DIP_TIME = 2500;     // ms per dip

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

void go(bool forward, int speed, unsigned long runTime, unsigned long pauseTime, int dipCount) {
  setDirection(forward);
  
  // Ramp up
  rampSpeed(speed);

  // Handle speed "dips" during the run (simulates slowing for curves/stations)
  if (dipCount > 0) {
    unsigned long segment = (runTime * 1000) / (dipCount + 1);
    for (int i = 0; i < dipCount; i++) {
      delay(segment);
      rampSpeed(DIP_SPEED);
      delay(DIP_TIME);
      rampSpeed(speed);
    }
    delay(segment);
  } else {
    delay(runTime * 1000);
  }

  // Ramp down and pause
  rampSpeed(0);
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

  if (target != 0) target = max(target, MIN_SPEED);
  target = min(target, MAX_SPEED);

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
  Serial.println("🔁 Circle Of Stops");
  bool dir = true;
  int spd = random(40, MAX_SPEED + 1);

  for (int i = 0; i < 6; i++) {
    go(dir, spd, 6, 5, 1);  // one slow dip
    dir = !dir;
  }
}

void longTrainRunning() {
  Serial.println("🔁 Long Train Running");
  int spd = random(100, MAX_SPEED + 1);

  for (int i = 0; i < 3; i++) {
    go(true,  spd, 20, 6, 2);
    go(false, spd, 20, 6, 2);
  }
}

void gentleWander() {
  Serial.println("🔁 Gentle Wander");
  for (int i = 0; i < 5; i++) {
    bool dir = random(0, 2);
    int spd = random(100, MAX_SPEED + 1);
    int dips = random(1, 3);   // 0–2 dips
    go(dir, spd,
       random(8, 14),
       random(4, 7),
       dips);
  }
}

// -------- setup --------
void setup() {
  Serial.begin(115200);
  delay(1);
  
  randomSeed(analogRead(A0));

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

