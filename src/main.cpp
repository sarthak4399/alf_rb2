#include <Arduino.h>

#define MaxSpeed 180      // max speed of the robot
#define BaseSpeed 180     // this is the speed at which the motors should spin when the robot is perfectly on the line
#define SensorCount  8    // number of sensors used

#define ena  5
#define in1  6
#define in2  7
#define enb  10
#define in3  8
#define in4  9
#define enc  11
#define in5  12
#define in6  13
#define end  3
#define in7  4
#define in8  2

#define tir 4

int P;
int I;
int D;
int motorSpeedFL; // Front Left
int motorSpeedFR; // Front Right
int motorSpeedRL; // Rear Left
int motorSpeedRR; // Rear Right

// Sensor pins
const uint8_t SensorPins[SensorCount] = {11, 10, 9, 8, 7, 6, 5, 4};

int lastError = 0;
unsigned int Sensor[8];

double Kp = 0.21;
double Ki = 0;
double Kd = 0;

double Ku = 0;
double Pu = 0;

bool tuning = true;
unsigned long lastTime;
double lastInput;
double outputSum, lastOutput;

int botPosition() {
  int Position = 0;
  int sensor = 0;
  for (int i = 0; i < SensorCount; i++) {
    Position += i * 1000 * digitalRead(SensorPins[i]);
    sensor += digitalRead(SensorPins[i]);
  }
  if (sensor != 0)
    Position /= sensor;
  else
    Position = -1; // no line detected
  return Position;
}

void move(int motor, int Speed, int Direction) {
  boolean inPin1 = HIGH;
  boolean inPin2 = LOW;

  if (Direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  } else {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 0) {
    digitalWrite(in1, inPin1);
    digitalWrite(in2, inPin2);
    analogWrite(ena, Speed);
  } else if (motor == 1) {
    digitalWrite(in3, inPin1);
    digitalWrite(in4, inPin2);
    analogWrite(enb, Speed);
  } else if (motor == 2) {
    digitalWrite(in5, inPin1);
    digitalWrite(in6, inPin2);
    analogWrite(enc, Speed);
  } else if (motor == 3) {
    digitalWrite(in7, inPin1);
    digitalWrite(in8, inPin2);
    analogWrite(end, Speed);
  }
}

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < SensorCount; i++) {
    pinMode(SensorPins[i], INPUT);
  }
  pinMode(tir, INPUT);

  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(enc, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(end, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  delay(2000); // wait for 2s to position the bot before entering the main loop
  move(0, 100, 1);
  move(1, 100, 1);
  move(2, 100, 1);
  move(3, 100, 1);
  delay(800);
}

void loop() {
  if (tuning) {
    // Step 1: Determine Ku and Pu
    double input = botPosition();
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);

    double error = input - lastInput;
    outputSum += Kp * error;

    double output = outputSum;

    // Apply the output to the motors
    move(0, output, 1);
    move(1, output, 1);
    move(2, output, 1);
    move(3, output, 1);

    if (timeChange >= 5000) { // Let's assume we give it 5 seconds to stabilize
      // Check if the system is oscillating
      if (abs(output - lastOutput) < 5) {
        Ku = Kp;
        Pu = (double)timeChange;
        tuning = false;

        // Calculate PID constants using Ziegler-Nichols method
        Kp = 0.6 * Ku;
        Ki = 2 * Kp / Pu;
        Kd = Kp * Pu / 8;

        Serial.print("Ku: ");
        Serial.println(Ku);
        Serial.print("Pu: ");
        Serial.println(Pu);
        Serial.print("Kp: ");
        Serial.println(Kp);
        Serial.print("Ki: ");
        Serial.println(Ki);
        Serial.print("Kd: ");
        Serial.println(Kd);
      }
    }

    lastTime = now;
    lastInput = input;
    lastOutput = output;
  } else {
    // Normal PID control
    int Position = botPosition();

    if (
      (digitalRead(SensorPins[0]) == 1) &&
      (digitalRead(SensorPins[1]) == 1) &&
      (digitalRead(SensorPins[2]) == 1) &&
      (digitalRead(SensorPins[3]) == 1) &&
      (digitalRead(SensorPins[4]) == 1) &&
      (digitalRead(SensorPins[5]) == 1) &&
      (digitalRead(SensorPins[6]) == 1) &&
      (digitalRead(SensorPins[7]) == 1) &&
      (digitalRead(tir) == 1)
    ) {
      // stop the bot at once
      move(0, 0, 0);
      move(1, 0, 0);
      move(2, 0, 0);
      move(3, 0, 0);
      Serial.print("stop");
      delay(10000);
      move(0, 100, 1);
      move(1, 100, 1);
      move(2, 100, 1);
      move(3, 100, 1);
      return;
    }

    if (
      (digitalRead(SensorPins[0]) == 1) &&
      (digitalRead(SensorPins[1]) == 1) &&
      (digitalRead(SensorPins[2]) == 1) &&
      (digitalRead(SensorPins[3]) == 1) &&
      (digitalRead(SensorPins[4]) == 1) &&
      (digitalRead(SensorPins[5]) == 0) &&
      (digitalRead(SensorPins[6]) == 0) &&
      (digitalRead(SensorPins[7]) == 0) &&
      (digitalRead(tir) == 1)
    ) {
      // stop the bot at once
      move(0, 0, 0);
      move(1, 0, 0);
      move(2, 0, 0);
      move(3, 0, 0);

      delay(1000);

      while (Position > 1000) {
        Position = botPosition();
        //Serial.println(Position);
        move(0, 100, 1);
        move(1, 100, 1);
        move(2, 100, 0);
        move(3, 100, 0);
        // delay(2000);
      }
      return;
    }

    if (Position == -1) {
      if (motorSpeedFR > motorSpeedFL) {
        motorSpeedFL = 0;
        motorSpeedRL = 0;
        move(1, motorSpeedFR, 1);
        move(0, motorSpeedFL, 1);
        move(3, motorSpeedRR, 1);
        move(2, motorSpeedRL, 1);
        return;
      }
      if (motorSpeedFR < motorSpeedFL) {
        motorSpeedFR = 0;
        motorSpeedRR = 0;
        move(1, motorSpeedFR, 1);
        move(0, motorSpeedFL, 1);
        move(3, motorSpeedRR, 1);
        move(2, motorSpeedRL, 1);
        return;
      }
    }

    int error = Position - 3500;
    P = error;
    I = I + error;
    D = error - lastError;
    int motorSpeedDiff = Kp * P + Kd * D + Ki * I;
    lastError = error;

    motorSpeedFR = BaseSpeed + motorSpeedDiff;
    motorSpeedFL = BaseSpeed - motorSpeedDiff;
    motorSpeedRR = BaseSpeed + motorSpeedDiff;
    motorSpeedRL = BaseSpeed - motorSpeedDiff;

    if (motorSpeedFR > MaxSpeed) motorSpeedFR = MaxSpeed; // prevent the motor from going beyond max speed
    if (motorSpeedFL > MaxSpeed) motorSpeedFL = MaxSpeed; // prevent the motor from going beyond max speed
    if (motorSpeedRR > MaxSpeed) motorSpeedRR = MaxSpeed; // prevent the motor from going beyond max speed
    if (motorSpeedRL > MaxSpeed) motorSpeedRL = MaxSpeed; // prevent the motor from going beyond max speed
    if (motorSpeedFR < 0) motorSpeedFR = 0;
    if (motorSpeedFL < 0) motorSpeedFL = 0;
    if (motorSpeedRR < 0) motorSpeedRR = 0;
    if (motorSpeedRL < 0) motorSpeedRL = 0;

    move(0, motorSpeedFL, 1); // Front left wheel
    move(1, motorSpeedFR, 1); // Front right wheel
    move(2, motorSpeedRL, 1); // Rear left wheel
    move(3, motorSpeedRR, 1); // Rear right wheel
  }
}
