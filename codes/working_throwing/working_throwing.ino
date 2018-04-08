#include <TimerOne.h>
#include <SPI.h>

int ir_pin = 4;
int debug = 1;
int start_bit = 2400;
int bin_1 = 1200;
int bin_0 = 600;
int dataOut = 0;
int guardTime = 300;

int positions[2] = {220, 900};
int max_speed = 40;
uint8_t pwm_pin = 5;
uint8_t dir_pin = 6;

uint8_t load_pin = 7;
volatile uint8_t shift_1 = 0, shift_2 = 0, temp1 = 0;
volatile uint16_t temp, prev_data = 0, data = 0;
volatile boolean present_bit = 0, last_bit = 0;

float error = 0, last_error = 0, set_point = 0;
float Kp = 0.09, Ki = 0.35, Kd = 26;
float P = 0, I = 0, D = 0, PID = 0;
float velocity = 0, counter = 0;

float set_position = 500;
unsigned long int present_ms = 0, previous_ms = 0, last_ms = 0, delta_t = 0;
boolean flag = 1;
boolean throw_flag = 0;
boolean pick_flag = 0;
int prev_inst = 0, inst = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ir_pin, OUTPUT);
  digitalWrite(ir_pin, LOW);
  init_encoder();
  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
}

void loop() {
  if(inst == 2)
  {
  set_position = 750;
  position_pid();
  }
  else if(inst == 3)
  {
  set_position = 160;
  position_pid();    
  }
  parse_serial();
  if (prev_inst != inst && (inst == 2 || inst == 3))
  {
    if (inst == 2)
    {
      grip_inst();
    }
    if (inst == 3)
    {
      throw_inst();
    }
  }
  prev_inst = inst;
}

void parse_serial()
{
  if (Serial.available() > 0)
  {
    int temp = 0;
    temp = Serial.parseInt();
    if (temp == 2 || temp == 3)
    {
      inst = temp;
    }
    else
    {
      return;
    }
    //    if (inst == 2)
    //    {
    //      pick_flag = 1;
    //      throw_flag = 0;
    //    }
    //    else if (inst == 3)
    //    {
    //      throw_flag = 1;
    //    }
    Serial.flush();
  }
}
void grip_inst()
{
  last_ms = millis();
  while (1)
  {
    set_position = 160;
    position_pid();
    //Serial.println("GRIPPING");
    if (millis() - last_ms > 2500)
    {
      break;
    }
  }
  tone(ir_pin, 100, 300);
  last_ms = millis();
  while (1)
  {
    set_position = 160;
    position_pid();
    //Serial.println("GRIPPING");
    if (millis() - last_ms > 2500)
    {
      break;
    }
  }
  last_ms = millis();
  while (1)
  {
    // Serial.println("Throwing Initial");
    set_position = 750;
    position_pid();
    if (millis() - last_ms > 5000)
    {
      break;
    }
  }
  Serial.println("2");
}

void throw_inst()
{
  motor(255);
  while (1)
  {
    //390 430
    //Serial.println("Throwing");
    if (data >= 415  && data <= 440)
    {
      tone(ir_pin, 300, 300);
      break;
    }
  }
  last_ms = millis();
  //Serial.println("Thrown");
  motor(0);
  last_ms = millis();
  while (1)
  {
    set_position = 160;
    position_pid();
    //Serial.println("GRIPPING");
    if (millis() - last_ms > 2500)
    {
      break;
    }
  }
  Serial.println("3");
}

void position_pid()
{
  float curr = (float)data;
  error = set_position - curr;
  if (set_position > curr)
  {
    if (abs(set_position - curr) < abs(-1024 + set_position - curr))
    {
      error = set_position - curr;
    }
    else
    {
      error = -1024 + set_position - curr;
    }
  }
  else
  {
    if (abs(set_position - curr) < abs(1024 + set_position - curr))
    {
      error = set_position - curr;
    }
    else
    {
      error = 1024 + set_position - curr;
    }
  }

  P = error;
  D = error - last_error;
  I = error + last_error;
  PID = Kp * P + Ki * I + Kd * D;

  if (abs(PID) > max_speed)
  {
    if (PID > 0)
    {
      PID = max_speed;
    }
    else
    {
      PID = -1 * max_speed;
    }
  }
  //  Serial.println((String)data + " " + (String)error + " " + (String)(-1*PID));

  last_error = error;
  motor(-PID);
}

void motor(int vel)
{
  if (vel >= 0)
  {
    digitalWrite(dir_pin, HIGH);
    analogWrite(pwm_pin, vel);
  }
  else
  {
    digitalWrite(dir_pin, LOW);
    analogWrite(pwm_pin, -vel);
  }
}

//void get_velocity() {
//  // Serial.print("Actual Speed: ");
//  delta_t = millis() - previous_ms;
//  previous_ms = millis();
//  velocity = ((float)(data - previous_data) * 1000) / (delta_t * 1024); // rps
//  previous_data = data;
//}

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
  if (prev_data != data)
  {
    if (prev_data >= 923 && prev_data <= 1023 && data >= 0 && data <= 123)
    {
      counter = counter + 1024 - prev_data + data;
    }
    else if (data >= 923 && data <= 1023 && prev_data >= 0 && prev_data <= 123)
    {
      counter = counter - (1024 - data + prev_data);
    }
    else if (prev_data < data)
    {
      counter = counter + (data - prev_data);
    }
    else if (prev_data > data)
    {
      counter = counter - (-data + prev_data);
    }
 //   Serial.println(data);
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



void init_encoder()
{
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  Timer1.initialize(1000);
  Timer1.attachInterrupt(encoder);
  pinMode(load_pin, OUTPUT);
  digitalWrite(load_pin, HIGH);
}

