#include "PinChangeInterrupt.h"

int outputA[4] = {A9, A11, A13, A14};
int outputB[4] = {A8, A10, A12, A15};
int velocity[4] = {0};
float counter[4] = {0};
int present_state[4];
int prev_state[4];
unsigned long int present_ms = 0, last_ms = 0, delta_t = 0;
void setup() {
  for (int i = 0; i < 4; i++)
  {
    pinMode (outputA[i], INPUT_PULLUP);
    pinMode (outputB[i], INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(outputA[i]), encode, CHANGE);
    prev_state[i] = digitalRead(outputA[i]);
  }
  Serial.begin (115200);
  // Reads the initial state of the outputA
}
void loop() {
    for(int i = 0; i < 4; i++)
  {
Serial.print(counter[i]);
Serial.print(" ");
  }
  Serial.println(" ");
}

void encode()
{
  for (int i = 0; i < 4; i++)
  {
    present_state[i] = digitalRead(outputA[i]); // Reads the "current" state of the outputA
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    if (present_state[i] != prev_state[i])
    {
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if (digitalRead(outputB[i]) != present_state[i]) {
        counter[i] ++;
      } else {
        counter[i] --;
      }
    }
    prev_state[i] = present_state[i]; // Updates the previous state of the outputA with the current state
  }
}

