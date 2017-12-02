/*
  C 0x43       Calibration             0x00
  L 0x4C       Mode (Dark On/Light On) 0x01(Dark on), 0x00 (light on)
  T 0x54       Line Threshold          0x00 - 0x07
  O 0x4F       Offset                  0x00 - 0x32
  J 0x4A       Junction Width          0x01 - 0x08
  A 0x41       UART Address            0x00 - 0xFF
  B 0x42       LCD Backlight           0x00 - 0x0A
  S 0x53       LCD Contrast            0x00 - 0xFF
  R 0x52       UART Baudrate           0x00 - 0x05
  D 0x44       UART Data Output Mode   0x00 - 0x03
*/
#define max_speed 20
#define DELAY 50

//Sensor Pins
uint8_t ser_enable[4] = {32, 34, 36, 38};
uint8_t Jpulse[4] = {31, 33, 35, 37};

//Motor Pins
uint8_t motor_pin[4] = {9, 12, 3, 6};
uint8_t dir_pin[4] = {8, 11, 2, 5};
uint8_t brake_pin[4] = {10, 13, 4, 7};

/*
  Pin mappings to motors
  0 - front left motor
  1 - front right motor
  2 - rear left motor
  3 - rear right motor
*/

char address = 0x00;
//Sensor Data
int sensor_data[4] = {0, 0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  init_motors();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void follow_forward(int left_speed, int right_speed)
{
  //curved trajectory
  set_motor(0, left_speed);
  set_motor(2, left_speed);
  set_motor(1, right_speed);
  set_motor(3, right_speed);
}
void follow_backward(int left_speed, int right_speed)
{
  set_motor(0, -1 * left_speed);
  set_motor(2, -1 * left_speed);
  set_motor(1, -1 * right_speed);
  set_motor(3, -1 * right_speed);
}

void follow_left(int left_speed, int right_speed)
{
  set_motor(0, left_speed);
  set_motor(1, -1 * right_speed);
  set_motor(2, -1 * right_speed);
  set_motor(3, left_speed);
}

void follow_right(int left_speed, int right_speed)
{
  set_motor(0, -1 * left_speed);
  set_motor(1, right_speed);
  set_motor(2, right_speed);
  set_motor(3, -1 * left_speed);
}

void set_motor(uint8_t index, int motor_speed)
{
  if (motor_speed > 0)
  {
    //clock wise direction
    digitalWrite(brake_pin[index], LOW);
    digitalWrite(dir_pin[index], LOW);
    delayMicroseconds(500);
    analogWrite(motor_pin[index], motor_speed);
  }
  else if (motor_speed < 0)
  {
    //anti clock wise direction
    digitalWrite(brake_pin[index], LOW);
    digitalWrite(dir_pin[index], HIGH);
    delayMicroseconds(500);
    analogWrite(motor_pin[index], -1 * motor_speed);
  }
  else
  {
    //brake condition
    digitalWrite(brake_pin[index], HIGH);
  }
}

void init_motors()
{
  for (int i = 0; i < 4; i++)
  {
    pinMode(motor_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT);
    pinMode(brake_pin[i], OUTPUT);
    digitalWrite(brake_pin[i], LOW);
  }
}
