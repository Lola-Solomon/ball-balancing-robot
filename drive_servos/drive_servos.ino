#include <Servo.h>

#define NUM_SERVOS 3

Servo servos[NUM_SERVOS];
int servoPins[NUM_SERVOS] = {3, 5, 6};  // PWM pins //a , b , c
int currentAngle[NUM_SERVOS] = {90, 90, 90};

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }
}

void parseCommand(String cmd) {
  if (cmd.startsWith("T")) {
    int id, state;
    sscanf(cmd.c_str(), "T,%d,%d", &id, &state);
    torqueSet(id, state);
  }

  if (cmd.startsWith("M")) {
    int id, angle, time_ms;
    sscanf(cmd.c_str(), "M,%d,%d,%d", &id, &angle, &time_ms);
    moveServoTimed(id, angle, time_ms);
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
