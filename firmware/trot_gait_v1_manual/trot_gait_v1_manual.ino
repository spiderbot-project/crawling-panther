#include <Servo.h>

enum { R1, R2, R3, R4, L1, L2, L3, L4, N };

const int pins[N]    = {4,   6,   5,   7,  8,   10, 9,  11};
const int neutral[N] = {85, 110, 170, 30, 110, 40, 70, 145};  // calibrated angles
const int sign[N]    = {1,   1,  -1,   1, -1,  -1,  1,  -1};  // mounting direction

const int HIP_SWING     = 20;
const int KNEE_LIFT     = 45;
const int STEPS         = 5;    // steps forward, then backward
const int INTERP_STEPS  = 20;   // sub-steps per movement
const int STEP_DELAY    = 12;   // ms between sub-steps

// If the robot walks backward instead of forward, swap these two values
const int FORWARD  =  1;
const int BACKWARD = -1;

Servo servos[N];
int cur[N];      // current angles
int target[N];   // target angles
int moveDir = FORWARD;

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

void hip(int i, int d) {
  target[i] = neutral[i] + sign[i] * HIP_SWING * d * moveDir;
}

void knee(int i, bool lifted) {
  target[i] = neutral[i] + (lifted ? sign[i] * KNEE_LIFT : 0);
}

// One full step: both diagonal groups move once
void gaitStep() {
  // Group R1+L2
  knee(R3, true); knee(L4, true); apply();                              // lift
  hip(R1, +1); hip(L2, +1); hip(R2, -1); hip(L1, -1); apply();           // swing
  knee(R3, false); knee(L4, false); apply();                           // lower

  // Group R2+L1
  knee(R4, true); knee(L3, true); apply();
  hip(R2, +1); hip(L1, +1); hip(R1, -1); hip(L2, -1); apply();
  knee(R4, false); knee(L3, false); apply();
}

void rest() {
  for (int i = 0; i < N; i++) target[i] = neutral[i];
  apply();
}

void setup() {
  for (int i = 0; i < N; i++) {
    cur[i] = target[i] = neutral[i];
    servos[i].write(cur[i]);
    servos[i].attach(pins[i]);
    delay(500);   // start servos one by one (protects the regulator)
  }
  delay(2000);    // wait 2 s before walking
}

void loop() {
  moveDir = FORWARD;
  for (int k = 0; k < STEPS; k++) gaitStep();
  rest();
  delay(1000);

  moveDir = BACKWARD;
  for (int k = 0; k < STEPS; k++) gaitStep();
  rest();
  delay(1000);
}
