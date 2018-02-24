#include <PinChangeInterrupt.h>
#include <TimerOne.h>
volatile uint8_t shift_1 = 0, shift_2 = 0, temp1 = 0;
volatile uint16_t temp, prev_data = 0, data = 0;
volatile boolean present_bit = 0, last_bit = 0;
volatile float counter = 0;
uint8_t encoder_pin[10] = {A8, A9, A10, A11, A12, A13, A14, A15, 51, 50};
//MSB to LSB

void setup() {
  // put your setup code here, to run once:
  init_encoder();
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  encode();
}

void init_encoder() {
  for (int i = 0; i < 10; i++)
  {
    pinMode (encoder_pin[i], INPUT_PULLUP);
  }
//  Timer1.initialize(1000);
//  Timer1.attachInterrupt(encode);
}

void encode()
{
  data = 0;
  data = PINK;
  for (int i = 8; i < 10; i++)
  { 
    data = data << 1;
    data |= digitalRead(encoder_pin[i]);
  }
  data = (~data) & 0b1111111111;
  data = gray2binary(data);
  if(prev_data != data)
  {
    if(prev_data == 0 && data == 1023)
    {
      counter--;
    }
    else if(prev_data == 1023 && data == 0)
    {
      counter++;
    }
    else if(prev_data < data)
    {
      counter++;
    }
    else if(prev_data > data)
    {
      counter--;
    }
      Serial.print(data);
  Serial.print(" ");
  Serial.println(counter);
  }
  prev_data = data;

}

uint16_t gray2binary(uint16_t num)
{
  uint16_t bin_data = 0, bit_msk = 0b1000000000;
  boolean present_bit = 0, last_bit  = 0;
  for (int i = 0; i < 10; i++)
  {
    present_bit = (num & bit_msk);
    //Serial.print(present_bit);
    present_bit = present_bit ^ last_bit;
    if (present_bit == 1)
    {
      bin_data |= bit_msk;
    }
    last_bit = present_bit;
    bit_msk = bit_msk >> 1;
  }
  //Serial.println(" ");
  return bin_data;
}

