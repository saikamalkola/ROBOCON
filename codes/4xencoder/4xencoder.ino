#include "PinChangeInterrupt.h"

int outputA[4] = {A10, A9, A12, A15};
int outputB[4] = {A11, A8, A13, A14};
int velocity[4] = {0};
float counter[4] = {0};
int present_state[4];
int prev_state[4];
unsigned long int present_ms = 0, last_ms = 0, delta_t = 0;

volatile int lastEncoded[4] = {0}, sum[4] = {0}, encoded[4] = {0}, MSB[4] = {0}, LSB[4] = {0};
volatile long encoderValue[4] = {0};


void setup() {
  for (int i = 0; i < 4; i++)
  {
    pinMode (outputA[i], INPUT_PULLUP);
    pinMode (outputB[i], INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(outputA[i]), encode, CHANGE);
    attachPCINT(digitalPinToPCINT(outputA[i]), encode, CHANGE);
  }
  Serial.begin (115200);
  // Reads the initial state of the outputA
}
void loop() {
  for(int i = 0; i < 4; i++)
  {
Serial.print(encoded[i]);
Serial.print(" ");
  }
  Serial.println(" ");
}

void encode()
{

  for (int i = 0; i < 4; i++)
  {
    MSB[i] = digitalRead(outputA[i]); //MSB = most significant bit
    LSB[i] = digitalRead(outputB[i]); //LSB = least significant bit

    encoded[i] = (MSB[i] << 1) | LSB[i]; //converting the 2 pin value to single number
    sum[i]  = (lastEncoded[i] << 2) | encoded[i]; //adding it to the previous encoded value

    if (sum[i] == 0b1101 || sum[i] == 0b0100 || sum[i] == 0b0010 || sum[i] == 0b1011) encoderValue[i] ++;
    if (sum[i] == 0b1110 || sum[i] == 0b0111 || sum[i] == 0b0001 || sum[i] == 0b1000) encoderValue[i] --;

    lastEncoded[i] = encoded[i]; //store this value for next time
  }
}

