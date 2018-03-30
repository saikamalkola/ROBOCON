#include <TimerOne.h>
#include <SPI.h>

volatile uint8_t shift_1 = 0, shift_2 = 0, temp1 = 0;
volatile uint16_t temp, data = 0, prev_data = 0, ctl_data = 0, rack_data = 0;
volatile int Vx = 512, Vy = 512, W = 512;
volatile uint8_t fast_button = 2, slow_button = 5;
volatile boolean button_status[18] = {0};

unsigned long last_ms = 0, update_ms = 10;
boolean line_mode = 0;
boolean speed_mode = 0;
boolean push_mode = 0;
boolean us_mode = 0;
boolean lift_mode = 0;

void setup() {
  Serial.begin(9600);
  init_SPI();
  pinMode(fast_button, INPUT_PULLUP);
  pinMode(slow_button, INPUT_PULLUP);
  last_ms = millis();
}

void loop() {
  //print_bin(data);

  set_line_mode();
  set_us_mode();
  set_speed_mode();
  set_push_mode();
  set_lift_mode();
  if (millis() - last_ms > update_ms)
  {
    last_ms = millis();
    send_data();
  }
}
void print_data()
{
  Serial.print("Speed");
  Serial.print(speed_mode);
  Serial.print(" ");

  Serial.print("line");
  Serial.print(line_mode);
  Serial.print(" ");

  Serial.print("lift");
  Serial.print(lift_mode);
  Serial.print(" ");

  Serial.print("push");
  Serial.print(push_mode);
  Serial.print(" ");

  Serial.print("us");
  Serial.print(us_mode);
  Serial.println(" ");
}

void send_data()
{
  Serial.print("#");
  Serial.print(us_mode);
  Serial.print("#");
  Serial.print(line_mode);
  Serial.print("#");
  Serial.print(speed_mode);
  Serial.print("#");
  Serial.print(lift_mode);
  Serial.print("#");
  Serial.print(push_mode);
  Serial.print("#");
  Serial.print(rack_data);
  Serial.print("#");
  Serial.print(Vx);
  Serial.print("#");
  Serial.print(Vy);
  Serial.print("#");
  Serial.print(W);
  Serial.println("#");
}
void set_speed_mode()
{
  if (button_status[17] == 1 && button_status[16] == 0)
  {
    speed_mode = 1;
  }
  else if (button_status[17] == 0 && button_status[16] == 1)
  {

    speed_mode = 0;
  }
}
void set_line_mode()
{
  if (button_status[0] == 1 && button_status[1] == 0)
  {
    line_mode = 1;
  }
  else if (button_status[0] == 0 && button_status[1] == 1)
  {

    line_mode = 0;
  }
}
void set_us_mode()
{
  if (button_status[2] == 1 && button_status[3] == 0)
  {
    us_mode = 1;
  }
  else if (button_status[2] == 0 && button_status[3] == 1)
  {
    us_mode = 0;
  }
}

void set_push_mode()
{
  if (button_status[7] == 1 && button_status[6] == 0)
  {
    push_mode = 1;
  }
  else if (button_status[7] == 0 && button_status[6] == 1)
  {
    push_mode = 0;
  }
}
void set_lift_mode()
{
  if (button_status[5] == 1 && button_status[4] == 0)
  {
    lift_mode = 1;
  }
  else if (button_status[5] == 0 && button_status[4] == 1)
  {

    lift_mode = 0;
  }
}
void init_SPI()
{
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  Timer1.initialize(1000);
  Timer1.attachInterrupt(read_data);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
}
void read_data()
{
  digitalWrite(4, LOW);
  digitalWrite(4, HIGH);
  data = 0;
  shift_2 = SPI.transfer(0x00);
  shift_1 = SPI.transfer(0x00);
  rack_data = shift_1;
  data |= (~(shift_2)) & (0xFF);
  data |= ((~(shift_1)) << 8);
  temp = data;
  rack_data = (~(shift_2)) & (0xFF);;
  for (int i = 0; i < 16; i++)
  {
    boolean bit_val = temp & 0b1000000000000000;
    temp <<= 1;
    button_status[i] = bit_val;
  }
  button_status[16] = digitalRead(fast_button);
  button_status[17] = digitalRead(slow_button);
  Vx = analogRead(A2);
  Vy = analogRead(A3);
  W = analogRead(A1);
}

void print_bin(uint16_t val)
{
  temp = val;
  for (int i = 0; i < 16; i++)
  {
    boolean bit_val = temp & 0b1000000000000000;
    temp <<= 1;
    Serial.print(bit_val);
  }
  Serial.print(digitalRead(2));
  Serial.print(digitalRead(5));
  Serial.print(" ");
  Serial.print(analogRead(A0));
  Serial.print(" ");
  Serial.print(analogRead(A1));
  Serial.print(" ");
  Serial.print(analogRead(A2));
  Serial.print(" ");
  Serial.println(analogRead(A3));
}

