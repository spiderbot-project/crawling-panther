// crawling-panther: trot gait v2 (parameters learned in simulation), walks forward only.
// Same structure as v1; differences:
//   * left and right hips swing by different amounts (found by simulation)
//   * knee lift and hip swing happen together, then the knees come down (4 moves per cycle)
//   * slower than the simulation on purpose (STEP_DELAY)
#include <Servo.h>

enum { R1, R2, R3, R4, L1, L2, L3, L4, N };

const int pins[N]    = {4,   6,   5,   7,  8,   10, 9,  11};
const int neutral[N] = {85, 110, 170, 30, 110, 40, 70, 145};  // calibrated angles
const int sign[N]    = {1,   1,  -1,   1, -1,  -1,  1,  -1};  // mounting direction

// Learned in simulation (simulation/best_gait_v2.json): right 17.4, left 24.8, knee 45
const int HIP_SWING_R  = 17;
const int HIP_SWING_L  = 25;
const int KNEE_LIFT    = 45;

const int INTERP_STEPS = 22;    // sub-steps per movement (same as simulation)
const int STEP_DELAY   = 8;     // ms between sub-steps (simulation is ~4 ms; 8 ms = about half speed)

// If the robot walks backward instead of forward, change to -1
const int FORWARD = 1;

Servo servos[N];
int cur[N];      // current angles
int target[N];   // target angles

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

// d = +1 swings the leg forward, d = -1 backward
void hip(int i, int d) {
  int swing = (i == R1 || i == R2) ? HIP_SWING_R : HIP_SWING_L;
  target[i] = neutral[i] + sign[i] * swing * d * FORWARD;
}

void knee(int i, bool lifted) {
  target[i] = neutral[i] + (lifted ? sign[i] * KNEE_LIFT : 0);
}

// One full cycle: both diagonal groups move once
void gaitCycle() {
  // Group R1+L2: lift knees R3, L4 while R1, L2 swing forward and R2, L1 push back
  knee(R3, true); knee(L4, true);
  hip(R1, +1); hip(L2, +1); hip(R2, -1); hip(L1, -1);
  apply();
  knee(R3, false); knee(L4, false); apply();

  // Group R2+L1: lift knees R4, L3 while R2, L1 swing forward and R1, L2 push back
  knee(R4, true); knee(L3, true);
  hip(R2, +1); hip(L1, +1); hip(R1, -1); hip(L2, -1);
  apply();
  knee(R4, false); knee(L3, false); apply();
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
  gaitCycle();    // walks forward until powered off
}
