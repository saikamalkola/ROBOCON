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
int left   = 0, right   = 0;

void motors()
{

  if (dir == 0)
  {
    set_motor(0, left);
    set_motor(1, right);
    set_motor(2, left);
    set_motor(3, right);
  }
  if (dir == 1)
  {
    set_motor(0, -1 * right);
    set_motor(1, -1 * right);
    set_motor(2, -1 * left);
    set_motor(3, -1 * left);
  }
  if (dir == 2)
  {
    /*
      0 - ACW
      2 - CW
    */
    set_motor(0, -1 * right);
    set_motor(1, left);
    set_motor(2,  left);
    set_motor(3, -1 * right);
  }
  if (dir == 3)
  {
    set_motor(0,left);
    set_motor(1, -1 * right);
    set_motor(2, -1 * right);
    set_motor(3, left);
  }
}

void set_motor(uint8_t index, int motor_speed)
{
  if (index % 2 == 0)
    motor_speed = -1 * motor_speed;
  if (motor_speed >= 0)
  {
    //clock wise direction
    digitalWrite(brake_pin[index], LOW);
    digitalWrite(dir_pin[index], LOW);
    analogWrite(motor_pin[index], motor_speed);
  }
  else if (motor_speed < 0)
  {
    //anti clock wise direction
    digitalWrite(brake_pin[index], LOW);
    digitalWrite(dir_pin[index], HIGH);
    analogWrite(motor_pin[index], -1 * motor_speed);
  }
}

void brake(uint8_t index)
{
  digitalWrite(brake_pin[index], HIGH);
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
