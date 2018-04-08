#include <Servo.h>

#include "PinChangeInterrupt.h"

Servo myservo;

int ir_pin = A0;
int led_pin = 13;
int debug = 1;
int start_bit = 2000;
int bin_1 = 1000;
int bin_0 = 400;
volatile int key = -1;

void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(ir_pin, INPUT);
  digitalWrite(led_pin, LOW);
  Serial.begin(9600);
  myservo.attach(3);
  attachPCINT(digitalPinToPCINT(A0), getIRKey, CHANGE);
}

void loop() {
  if (key == 2)
  {
    myservo.write(90);
  }
  else if (key == 3)
  {
    myservo.write(180);
  }
}


int getIRKey() {
  noInterrupts();
  int data[2];
  digitalWrite(led_pin, HIGH);
  while (pulseIn(ir_pin, LOW) < 2200)
  {

  }
  data[0] = pulseIn(ir_pin, LOW);
  data[1] = pulseIn(ir_pin, LOW);
  digitalWrite(led_pin, LOW);

  for (int i = 0; i < 2; i++)
  {
    if (data[i] > bin_1) {
      data[i] = 1;
    }  else {
      if (data[i] > bin_0) {
        data[i] = 0;
      } else {
        data[i] = 2;
      }
    }
  }

  int result = 0;
  int seed = 1;
  for (int i = 1; i >= 0; i--) {
    if (data[i] == 1) {
      result += seed;
    }
    seed = seed * 2;
  }
  key = result;

  if (key != -1) {
  //  Serial.println(key);
  }
  interrupts();
}
