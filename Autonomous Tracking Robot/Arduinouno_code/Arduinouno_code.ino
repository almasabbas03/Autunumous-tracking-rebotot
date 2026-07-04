#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial espSerial(A0, A1);

#define TRIG_PIN          2
#define ECHO_PIN          3
#define SERVO_PIN         4
#define ENA               5
#define ENB               6
#define IN1               10
#define IN2               9
#define IN3               8
#define IN4               7

#define OBSTACLE_DISTANCE 10
#define STOP_DISTANCE     12
#define DETECT_RANGE      30

Servo myServo;

char          lastSignal    = 'S';
unsigned long lastCmdTime   = 0;
bool          colorEverSeen = false;
bool          targetStopped = false;
bool          avoiding      = false;

// ── distance helper ──────────────────────────────────────
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 30000);
  if (d == 0) return 999;
  return d * 0.034 / 2;
}

// ── motor helpers ─────────────────────────────────────────
void moveForward(int spd) {
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward(int spd) {
  analogWrite(ENA, spd);  analogWrite(ENB, spd);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void turnLeft(int spd) {
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight(int spd) {
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

// SHARP STEER — 40% slow side
void steerLeft(int spd) {
  analogWrite(ENA, (int)(spd * 0.40));  // left motor 40% — sharper
  analogWrite(ENB, spd);                // right motor full
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void steerRight(int spd) {
  analogWrite(ENA, spd);                // left motor full
  analogWrite(ENB, (int)(spd * 0.40)); // right motor 40% — sharper
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);    analogWrite(ENB, 0);
}

// ── servo scan ────────────────────────────────────────────
long scanAt(int angle) {
  myServo.write(angle);
  delay(600);
  long d = getDistance();
  Serial.print("Scan@"); Serial.print(angle);
  Serial.print("=");     Serial.println(d);
  return d;
}

// ── obstacle avoid ────────────────────────────────────────
void avoidObstacle() {
  avoiding = true;
  stopMotors();
  delay(300);
  Serial.println("=== AVOID START ===");

  long leftDist  = scanAt(160);
  long rightDist = scanAt(20);
  myServo.write(90);
  delay(400);

  Serial.print("LEFT=");   Serial.print(leftDist);
  Serial.print(" RIGHT="); Serial.println(rightDist);

  if (leftDist <= OBSTACLE_DISTANCE && rightDist <= OBSTACLE_DISTANCE) {
    Serial.println("BOTH BLOCKED — reversing");
    moveBackward(220); delay(800); stopMotors(); delay(200);
    turnRight(250);    delay(900); stopMotors(); delay(200);  // faster sharp turn
    moveForward(220);  delay(600); stopMotors(); delay(200);

  } else if (leftDist >= rightDist) {
    Serial.println("GO LEFT");
    turnLeft(250);     delay(500); stopMotors(); delay(200);  // faster sharp turn
    moveForward(220);  delay(700); stopMotors(); delay(200);

    myServo.write(90); delay(400);
    long frontCheck = getDistance();
    Serial.print("FrontCheck="); Serial.println(frontCheck);

    if (frontCheck > OBSTACLE_DISTANCE) {
      moveForward(220); delay(600); stopMotors(); delay(200);
    }

    turnRight(250);    delay(500); stopMotors(); delay(200);
    moveForward(220);  delay(600); stopMotors(); delay(200);
    turnRight(250);    delay(500); stopMotors(); delay(200);
    moveForward(220);  delay(700); stopMotors(); delay(200);
    turnLeft(250);     delay(500); stopMotors(); delay(200);

  } else {
    Serial.println("GO RIGHT");
    turnRight(250);    delay(500); stopMotors(); delay(200);  // faster sharp turn
    moveForward(220);  delay(700); stopMotors(); delay(200);

    myServo.write(90); delay(400);
    long frontCheck = getDistance();
    Serial.print("FrontCheck="); Serial.println(frontCheck);

    if (frontCheck > OBSTACLE_DISTANCE) {
      moveForward(220); delay(600); stopMotors(); delay(200);
    }

    turnLeft(250);     delay(500); stopMotors(); delay(200);
    moveForward(220);  delay(600); stopMotors(); delay(200);
    turnLeft(250);     delay(500); stopMotors(); delay(200);
    moveForward(220);  delay(700); stopMotors(); delay(200);
    turnRight(250);    delay(500); stopMotors(); delay(200);
  }

  stopMotors();
  Serial.println("=== AVOID DONE — waiting for color ===");

  myServo.write(90);
  delay(300);

  unsigned long waitStart = millis();
  while (millis() - waitStart < 4000) {
    if (espSerial.available() >= 1) {
      char cmd = espSerial.read();
      if (cmd == 'F' || cmd == 'L' || cmd == 'R') {
        lastSignal    = cmd;
        lastCmdTime   = millis();
        colorEverSeen = true;
        Serial.print("COLOR REDETECTED: "); Serial.println(cmd);
        break;
      }
    }
    delay(50);
  }

  avoiding      = false;
  targetStopped = false;
  lastCmdTime   = millis();
  Serial.println("=== RESUMING ===");
}

// ── setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90);
  delay(1000);
  Serial.println("Robot Ready!");
}

// ── loop ──────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  while (espSerial.available() >= 1) {
    char cmd = espSerial.read();
    if (cmd != 'F' && cmd != 'L' && cmd != 'R' && cmd != 'S') continue;

    Serial.print("CMD: "); Serial.println(cmd);

    if (!colorEverSeen && cmd != 'S') {
      colorEverSeen = true;
      Serial.println("COLOR FIRST SEEN");
    }

    lastSignal  = cmd;
    lastCmdTime = now;
  }

  if (!colorEverSeen) { stopMotors(); delay(80); return; }

  if (now - lastCmdTime > 1500) {
    lastSignal = 'S';
  }

  myServo.write(90);
  long frontDist = getDistance();
  Serial.print("Front: "); Serial.println(frontDist);

  if (lastSignal == 'F') {

    if (frontDist <= OBSTACLE_DISTANCE) {
      Serial.println("OBSTACLE DETECTED!");
      avoidObstacle();
      return;
    }

    if (frontDist <= STOP_DISTANCE) {
      if (!targetStopped) {
        targetStopped = true;
        Serial.println("12cm — STOPPED");
      }
      stopMotors();
      delay(80);
      return;
    }

    if (targetStopped && frontDist > DETECT_RANGE) {
      targetStopped = false;
      Serial.println("TARGET AWAY — RESUME");
    }

    if (targetStopped) {
      stopMotors();
      delay(80);
      return;
    }

    moveForward(230);  // speed 200 se 230

  } else if (lastSignal == 'L') {
    steerLeft(230);    // speed 200 se 230
  } else if (lastSignal == 'R') {
    steerRight(230);   // speed 200 se 230
  } else {
    targetStopped = false;
    stopMotors();
  }

  delay(80);
}