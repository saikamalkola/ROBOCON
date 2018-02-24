#include <TimerOne.h>
#include <SPI.h>

uint8_t pwm_pin = 5;
uint8_t dir_pin = 6;

uint8_t load_pin = 7;
volatile uint8_t shift_1 = 0, shift_2 = 0, temp1 = 0;
volatile uint16_t temp, previous_data = 0, data = 0;
volatile boolean present_bit = 0, last_bit = 0;

float velocity = 0, counter = 0;

unsigned long int present_ms = 0, previous_ms = 0, last_ms = 0, delta_t = 0;

void setup() {
  Serial.begin(115200);
  init_encoder();
  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
}

void loop() {
  //  get_velocity();
  //  motor(50);
  Serial.println(String(data) + " " + String(velocity) + " " + String(counter));
  motor(255);
  if (data >= 870 && data <= 910)
  {
    analogWrite(pwm_pin, -50);
    delay(1000);
    analogWrite(pwm_pin, 0);
    while (1)
    {
      if (Serial.available() > 0)
      {
        if (Serial.read() == '1')
        {
          break;
        }
      }
    }
  }
}

void motor(int vel)
{
  if (vel >= 0)
  {
    digitalWrite(dir_pin, LOW);
    analogWrite(pwm_pin, vel);
  }
  else
  {
    digitalWrite(dir_pin, HIGH);
    analogWrite(pwm_pin, -vel);
  }
}
void get_velocity() {
  // Serial.print("Actual Speed: ");
  delta_t = millis() - previous_ms;
  previous_ms = millis();
  velocity = ((float)(data - previous_data) * 1000) / (delta_t * 1024); // rps
  previous_data = data;
}

void encoder()
{
  PORTD &= ~(0x80); //Load Pin Low
  PORTD |= (0x80);  //Load Pin High
  //  digitalWrite(7, LOW);
  //  digitalWrite(7, HIGH);
  data = 0;
  shift_2 = SPI.transfer(0x00);
  shift_1 = SPI.transfer(0x00);
  data |= (~(shift_2)) & (0xFF);
  data |= (((~(shift_1)) & 0x03) << 8);
  data = gray2binary(data);
  //  if (prev_data != data)
  //  { //    print_bin(gray2binary(data));
  //    if (prev_data > data)
  //    {
  //      counter++;
  //    }
  //    else
  //    {
  //      counter--;
  //    }
  //  }
  //  prev_data = data;
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

void init_encoder()
{
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  Timer1.initialize(100);
  Timer1.attachInterrupt(encoder);
  pinMode(load_pin, OUTPUT);
  digitalWrite(load_pin, HIGH);
}

