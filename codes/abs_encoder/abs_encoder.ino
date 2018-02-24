#include <TimerOne.h>
#include <SPI.h>

volatile uint8_t shift_1 = 0, shift_2 = 0, temp1 = 0;
volatile uint16_t temp, prev_data = 0, data = 0;
volatile boolean present_bit = 0, last_bit = 0;
void setup() {
  Serial.begin(115200);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  Timer1.initialize(1000);
  Timer1.attachInterrupt(encoder);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

}

void loop() {

}

void encoder()
{
  PORTD &= ~(0x80);
  PORTD |= (0x80);
  //  digitalWrite(7, LOW);
  //  digitalWrite(7, HIGH);
  data = 0;
  shift_2 = SPI.transfer(0x00);
  shift_1 = SPI.transfer(0x00);
  data |= (~(shift_2)) & (0xFF);
  data |= (((~(shift_1)) & 0x03) << 8);
  if (prev_data != data)
  { //    print_bin(gray2binary(data));
    Serial.println(gray2binary(data));
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

void print_bin(uint16_t val)
{
  temp = val;
  for (int i = 0; i < 10; i++)
  {
    boolean bit_val = temp & 0b1000000000;
    temp <<= 1;
    Serial.print(bit_val);
  }
  Serial.println("");
}

