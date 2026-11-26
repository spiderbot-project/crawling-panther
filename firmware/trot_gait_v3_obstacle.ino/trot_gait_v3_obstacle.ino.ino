// crawling-panther: trot gait v3
//   * walks the learned gait toward the sensor end (old rear legs R2/L2 are the front)
//   * HC-SR04 on A3 (trig) and A2 (echo): when something is closer than OBSTACLE_CM the robot
//     backs up BACK_CYCLES cycles, turns left about 90 degrees and walks on
//
// Serial monitor (115200 baud, newline):
//   g walk   s stop   d print distance   l left-turn test   z / x steer more right / left
#include <Servo.h>

enum { R1, R2, R3, R4, L1, L2, L3, L4, N };

const int pins[N]    = {2,   6,   5,   7,  8,   10, 9,  11};
const int neutral[N] = {80, 110, 170, 30, 110, 42, 70, 145};   // calibrated angles
const int sign[N]    = {1,   1,  -1,   1, -1,  -1,  1,  -1};   // mounting direction

const int HIP_SWING_R  = 21;    // degrees
const int HIP_SWING_L  = 21;
//                                R1 R2 R3  R4  L1 L2 L3  L4      (only the knees R3, R4, L3, L4 are used)
const int kneeLift[N]         = {0, 0, 45, 45, 0, 0, 45, 45};
int trim = 0;                   // drift trim: robot veers left -> press z. Copy the final value here.

const int INTERP_STEPS = 22;    // sub-steps per movement
const int STEP_DELAY   = 8;     // ms between sub-steps
const int FORWARD      = -1;    // old rear legs (R2, L2) are the front

const uint8_t TRIG_PIN = A3;    // if you never get a reading, swap these two
const uint8_t ECHO_PIN = A2;
const int OBSTACLE_CM    = 10;  // turn when closer than this (an HC-SR04 cannot read below ~2 cm)
const int NO_ECHO        = 999; // nothing heard
const int BACK_CYCLES    = 2;   // how far to back off before turning
const int TURN_CYCLES_90 = 4;   // gait cycles for a ~90 degree left turn: tune with the 'l' key

Servo servos[N];
int cur[N], target[N];
bool walking = true;
int sideDirR = 1, sideDirL = 1;  // -1 = this side's hips swing the other way (turning)
int walkDir = 1;                 // -1 while backing up

// Move all servos smoothly from cur[] to target[]
void apply() {
  int start[N];
  for (int i = 0; i < N; i++) start[i] = cur[i];
  for (int s = 1; s <= INTERP_STEPS; s++) {
    for (int i = 0; i < N; i++) {
      cur[i] = start[i] + (long)(target[i] - start[i]) * s / INTERP_STEPS;
      servos[i].write(cur[i]);
    }
    delay(STEP_DELAY);
  }
}

// d = +1 swings the leg toward the sensor, d = -1 the other way
void hip(int i, int d) {
  bool right = (i == R1 || i == R2);
  int swing = right ? HIP_SWING_R + trim : HIP_SWING_L - trim;
  if (swing < 0) swing = 0;
  int side = right ? sideDirR : sideDirL;
  target[i] = neutral[i] + sign[i] * swing * d * FORWARD * side * walkDir;
}

void knee(int i, bool lifted) {
  target[i] = neutral[i] + (lifted ? sign[i] * kneeLift[i] : 0);
}

void halfCycleA() {
  knee(R3, true); knee(L4, true);
  hip(R1, +1); hip(L2, +1); hip(R2, -1); hip(L1, -1);
  apply();
  knee(R3, false); knee(L4, false); apply();
}

void halfCycleB() {
  knee(R4, true); knee(L3, true);
  hip(R2, +1); hip(L1, +1); hip(R1, -1); hip(L2, -1);
  apply();
  knee(R4, false); knee(L3, false); apply();
}

// ---------- distance ----------
int readOnce() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long t = pulseIn(ECHO_PIN, HIGH, 25000UL);   // 25 ms is about 4 m
  delay(10);                                            // let the echo die out
  if (t == 0) return NO_ECHO;
  return (int)(t / 58);                                 // microseconds to cm
}

bool blocked() {                                        // two close readings in a row
  return readOnce() < OBSTACLE_CM && readOnce() < OBSTACLE_CM;
}

// ---------- avoiding ----------
void backUp(int cycles) {
  walkDir = -1;
  for (int c = 0; c < cycles; c++) { halfCycleA(); halfCycleB(); }
  walkDir = 1;
}

// The robot's front is now the old rear, so its LEFT side carries the R servos:
// turning left = R servos swing backward, L servos swing forward.
void turnLeft(int cycles) {
  sideDirR = -1; sideDirL = 1;
  for (int c = 0; c < cycles; c++) { halfCycleA(); halfCycleB(); }
  sideDirR = sideDirL = 1;
}

void avoidObstacle() {
  Serial.println(F("Obstacle: backing up, turning left"));
  backUp(BACK_CYCLES);
  turnLeft(TURN_CYCLES_90);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  for (int i = 0; i < N; i++) {
    cur[i] = target[i] = neutral[i];
    servos[i].write(cur[i]);
    servos[i].attach(pins[i]);
    delay(500);                 // start servos one by one (protects the regulator)
  }
  Serial.println(F("g walk  s stop  d distance  l left turn  z / x steer"));
  delay(2000);                  // wait 2 s before walking
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') walking = true;
    else if (c == 's') walking = false;
    else if (c == 'd') { Serial.print(F("distance: ")); Serial.println(readOnce()); }
    else if (c == 'l') turnLeft(TURN_CYCLES_90);
    else if (c == 'z') { trim = min(trim + 1, 20);  Serial.print(F("trim ")); Serial.println(trim); }
    else if (c == 'x') { trim = max(trim - 1, -20); Serial.print(F("trim ")); Serial.println(trim); }
  }
}

void loop() {
  handleSerial();
  if (!walking) return;
  halfCycleA();
  if (blocked()) { avoidObstacle(); return; }
  halfCycleB();
  if (blocked()) { avoidObstacle(); return; }
}