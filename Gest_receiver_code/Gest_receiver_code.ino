#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // CONNECT BT RX PIN TO ARDUINO 11 PIN | CONNECT BT TX PIN TO ARDUINO 10 PIN
char tiltDirection;
int speedValue;
int motorInput1 = 5;
int motorInput2 = 6;
int motorInput3 = 3;
int motorInput4 = 9;

void setup() {
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
  Serial.begin(38400);      // Serial communication is activated at 38400 baud/s.
  BTSerial.begin(9600);    // HC-05 default speed in AT command mode
}

void loop() {
  if (BTSerial.available()) {
    tiltDirection = BTSerial.read();
    speedValue = BTSerial.read(); // Read speed value

    if (tiltDirection == 'F') {
      Serial.println("Forward");
      forward(speedValue);
    } else if (tiltDirection == 'B') {
      Serial.println("Reverse");
      reverse(speedValue);
    } else if (tiltDirection == 'R') {
      Serial.println("Right");
      right(speedValue);
    } else if (tiltDirection == 'L') {
      Serial.println("Left");
      left(speedValue);
    } else if (tiltDirection == 'S') {
      Serial.println("Stop");
      stopCar();
    }
  }
}

void forward(int speed) {
  analogWrite(motorInput1, 0);
  analogWrite(motorInput2, speed);
  analogWrite(motorInput3, 0);
  analogWrite(motorInput4, speed);
}

void reverse(int speed) {
  analogWrite(motorInput1, speed);
  analogWrite(motorInput2, 0);
  analogWrite(motorInput3, speed);
  analogWrite(motorInput4, 0);
}

void right(int speed) {
  analogWrite(motorInput1, 0);
  analogWrite(motorInput2, speed);
  analogWrite(motorInput3, speed);
  analogWrite(motorInput4, 0);
}

void left(int speed) {
  analogWrite(motorInput1, speed);
  analogWrite(motorInput2, 0);
  analogWrite(motorInput3, 0);
  analogWrite(motorInput4, speed);
}

void stopCar() {
  analogWrite(motorInput1, 0);
  analogWrite(motorInput2, 0);
  analogWrite(motorInput3, 0);
  analogWrite(motorInput4, 0);
}