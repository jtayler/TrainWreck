// -------- pins --------
const int speedPin = 9;   // PWMA  (speed / PWM)
const int dirPin   = 8;   // AIN1  (direction)
const int dirPin2  = 7;   // AIN2  (direction)
const int stbyPin  = 6;   // STBY  (enable)

// -------- tuning --------
const int MAX_SPEED   = 130;   // safer ceiling for 12V
const int RAMP_STEP   = 2;
const int RAMP_DELAY  = 50;    // ms
const int MIN_SPEED   = 10;  
const float MAX_MPH = 63.0;   // calibrate once, then trust it

// -------- dip behavior --------
const int DIP_SPEED = MAX_SPEED * 4 / 9;  // ~44%
const unsigned long DIP_TIME = 2500;     // ms per dip

// -------- forward declaration --------
float speedToMph(int pwm);

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

    // move toward target
    current += (current < target) ? step : -step;

    // ✅ PROPER CLAMP — did we pass the target?
    if ((start < target && current > target) ||
        (start > target && current < target)) {
      current = target;
    }

    analogWrite(speedPin, current);
    delay(RAMP_DELAY);
  }
}
// -------- core primitive --------
void go(bool forward,
        int speed,
        unsigned long runTime,
        unsigned long pauseTime,
        int dipCount) {

  speed = constrain(speed, 0, MAX_SPEED);

Serial.print("🟢 GO ⏱ ");
Serial.print(runTime);
Serial.print("s");

Serial.print(forward ? " ▶️ FWD " : " ◀️ REV ");
Serial.print(speedToMph(speed), 1);
Serial.print(" MPH ");

if (dipCount > 0) {
  Serial.print(" Dips: ");
  Serial.print(dipCount);
}

Serial.println();

  setDirection(forward);
  rampSpeed(speed);

  if (dipCount > 0) {
    unsigned long slice = (runTime * 1000UL) / (dipCount + 1);

    for (int i = 0; i < dipCount; i++) {
      delay(slice);
      rampSpeed(DIP_SPEED);
      delay(DIP_TIME);
      rampSpeed(speed);
    }
    delay(slice);
  } else {
    delay(runTime * 1000);
  }

  rampSpeed(0);
  Serial.print("🛑 STOP ⏱ ");
  Serial.print(pauseTime);
  Serial.println("s");

  delay(pauseTime * 1000);
}

float speedToMph(int pwm) {
  pwm = constrain(pwm, 0, MAX_SPEED);
  return (pwm / (float)MAX_SPEED) * MAX_MPH;
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
  int spd = random(100, MAX_SPEED + 1);

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

  pinMode(speedPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  digitalWrite(stbyPin, HIGH); // enable driver

  Serial.println("BOOT");
}

// -------- loop --------
void loop() {
  Serial.println("");
  Serial.println("LOOP START");

  //theLongRun();
  circleOfStops();
  longTrainRunning();
  gentleWander();

  Serial.println("LOOP END");
}

