#include <Servo.h>

#define NUM_SERVOS 3

Servo servos[NUM_SERVOS];

// Pins of servos A, B, C
int servoPins[NUM_SERVOS] = {3, 5, 6};

// Current angles (initialize to 90 for safety)
int currentAngle[NUM_SERVOS] = {90, 90, 90};

void setup() {
  Serial.begin(115200);

  // Attach all servos once
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(currentAngle[i]);
  }

  Serial.println("Arduino ready");
}

void loop() {
  // ===== TEST COMMAND =====
  // Format: S,a1,a2,a3,time_ms
  // Example:
  String cmd = "S,10,10,10,500\n";
  parseCommand(cmd);

  delay(2000); // just to avoid spamming
}

// ================= COMMAND PARSER =================
void parseCommand(String cmd) {

  Serial.print("RX: ");
  Serial.println(cmd);

  // Torque ON / OFF
  if (cmd.startsWith("T")) {
    int id, state;
    sscanf(cmd.c_str(), "T,%d,%d", &id, &state);
    torqueSet(id, state);
  }

  // Single servo move
  else if (cmd.startsWith("M")) {
    int id, angle, time_ms;
    sscanf(cmd.c_str(), "M,%d,%d,%d", &id, &angle, &time_ms);
    moveServoTimed(id, angle, time_ms);
  }

  // ===== SYNC MOVE =====
  else if (cmd.startsWith("S")) {
    int a1, a2, a3, time_ms;
    sscanf(cmd.c_str(), "S,%d,%d,%d,%d", &a1, &a2, &a3, &time_ms);

    Serial.print("SYNC: ");
    Serial.print(a1); Serial.print(", ");
    Serial.print(a2); Serial.print(", ");
    Serial.print(a3); Serial.print(", ");
    Serial.println(time_ms);

    moveAllServosTimed(a1, a2, a3, time_ms);
  }
}

// ================= TORQUE =================
void torqueSet(int id, int state) {
  if (id < 1 || id > NUM_SERVOS) return;
  int idx = id - 1;

  if (state == 1) {
    servos[idx].attach(servoPins[idx]);
    servos[idx].write(currentAngle[idx]);
  } else {
    servos[idx].detach();
  }
}

// ================= SINGLE SERVO MOVE =================
void moveServoTimed(int id, int target, int time_ms) {
  if (id < 1 || id > NUM_SERVOS) return;
  int idx = id - 1;

  int start = currentAngle[idx];
  int steps = abs(target - start);
  if (steps == 0) return;

  int delay_ms = time_ms / steps;

  for (int i = 0; i <= steps; i++) {
    int angle = start + (target > start ? i : -i);
    servos[idx].write(angle);
    delay(delay_ms);
  }

  currentAngle[idx] = target;
}

// ================= SYNC MOVE =================
void moveAllServosTimed(int a1, int a2, int a3, int time_ms) {

  int targets[NUM_SERVOS] = {a1, a2, a3};

  int maxSteps = 0;
  for (int i = 0; i < NUM_SERVOS; i++) {
    int steps = abs(targets[i] - currentAngle[i]);
    if (steps > maxSteps) maxSteps = steps;
  }

  if (maxSteps == 0) return;

  int delay_ms = time_ms / maxSteps;

  for (int step = 0; step <= maxSteps; step++) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      int diff = targets[i] - currentAngle[i];
      if (abs(diff) >= step) {
        int dir = (diff > 0) ? 1 : -1;
        servos[i].write(currentAngle[i] + dir * step);
      }
    }
    delay(delay_ms);
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = targets[i];
  }
}
