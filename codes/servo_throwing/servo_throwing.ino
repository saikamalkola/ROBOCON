#include <Servo.h>
Servo myservo;

uint8_t ldr_pin = A0;
int threshold = 850;

int prev_reading = 0, reading = 0;
int prev_mode = 0, mode = 0;

void setup() {
  pinMode(ldr_pin, INPUT);
  Serial.begin(9600);
  myservo.attach(3);
}

void loop() {
  check_ldr();
  if (mode != prev_mode)
  {
    if (!mode)
    {
      myservo.write(90);
    }
    else
    {
      myservo.write(180);
    }
  }
  prev_mode = mode;
}

void check_ldr()
{
  reading = analogRead(A0);
  if (reading >= threshold)
  {
    reading = 1;
  }
  else
  {
    reading = 0;
  }
  if (prev_reading != reading && reading == 1)
  {
    mode = !mode;
  }
  prev_reading = reading;
}

