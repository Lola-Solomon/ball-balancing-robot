#include <Servo.h>

#define NUM_SERVOS 3

Servo servos[NUM_SERVOS];
int servoPins[NUM_SERVOS] = {3, 5, 6};   // A, B, C
int currentAngle[NUM_SERVOS] = {0, 0, 0};

void setup() {
  Serial.begin(115200);

  // Attach all servos once
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(currentAngle[i]);
  }
}

void loop() {
  // String cmd = "S,{10},{10},{10},{0.5}\n";
  // parseCommand(cmd);
  if (Serial.available()) {
    // String cmd = "S,{15},{15},{15},{0.5}\n"
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }
}

void parseCommand(String cmd) {
  Serial.print("RX: ");
  Serial.println(cmd);

  // Torque ON / OFF
  if (cmd.startsWith("T")) {
    int id, state;
    sscanf(cmd.c_str(), "T,%d,%d", &id, &state);
    torqueSet(id, state);
  }

  // Single move (legacy)
  else if (cmd.startsWith("M")) {
    int id, angle, time_ms;
    sscanf(cmd.c_str(), "M,%d,%f,%d", &id, &angle, &time_ms);
    moveServoTimed(id, angle, time_ms);
  }

  // ===== SYNC MOVE =====
  else if (cmd.startsWith("S")) {
    int a1, a2, a3, time_ms;
    sscanf(cmd.c_str(), "S,%f,%f,%f,%d", &a1, &a2, &a3, &time_ms);

    Serial.print("SYNC: ");
    Serial.print(a1); Serial.print(", ");
    Serial.print(a2); Serial.print(", ");
    Serial.print(a3); Serial.print(", ");
    Serial.println(time_ms);

    moveAllServosTimed(a1, a2, a3, time_ms);
  }
}

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

// ======= SYNC MOVE IMPLEMENTATION =======
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
