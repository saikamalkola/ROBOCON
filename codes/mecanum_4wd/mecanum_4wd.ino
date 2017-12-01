
#define max_speed 50

uint8_t motor_pin[4] = {9, 12, 3, 6};
uint8_t dir_pin[4] = {8,11,2,5};
uint8_t brake_pin[4] = {10,13,4,7};
/*
  Pin mappings to motors

  0 - front left motor
  1 - front right motor
  2 - rear left motor
  3 - rear right motor
*/

char sel = 0;
boolean flag = 0;
void setup() {
  //Declaring Motor pins as OUTPUTS
  init_motors();
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    sel = Serial.read();
    flag = 0;
  }
  switch (sel)
  {
    case '8':
      {
        forward();
        if (flag == 0)
        {
          Serial.println("Forward");
          flag = 1;
        }
      }
      break;
    case '2':
      {
        backward();
        if (flag == 0)
        {
          Serial.println("backward");
          flag = 1;
        }
      }
      break;
    case '7':
      {
        forward_left();
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Forward Left");
        }
      }
      break;
    case '9':
      {
        forward_right();
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Forward Right");
        }
      }
      break;
    case '1':
      {
        backward_left();
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Backward Left");
        }
      }
      break;
    case '3':
      {
        backward_right();
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Backward Right");
        }
      }
      break;
    case '4':
      {
        slide_left();
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Slide Left");
        }
      }
      break;
    case '6':
      {
        slide_right();
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Slide Right");
        }
      }
      break;
    case 'a':
      {
        turn_left();
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Turn Left");
        }
      }
      break;
    case 'd':
      {
        turn_right();
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Turn Right");
        }
      }
      break;
    case 'e':
      {
        motors(100, -50);
        if (flag == 0)
        {
          flag = 1;
          Serial.println("curve right");
        }
      }
      break;
    case 'q':
      {
        motors(-50, 100);
        if (flag == 0)
        {
          flag = 1;
          Serial.println("Curve left");
        }
      }
      break;
    default:
      motors(0, 0);
      if (flag == 0)
      {
        flag = 1;
        Serial.println("Stop");
      }
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

void forward()
{
  //all motors in clockwise direction
  for (int i = 0; i < 4; i++)
  {
    set_motor(i, max_speed);
  }
}

void backward()
{
  //all motors in anti clockwise direction
  for (int i = 0; i < 4; i++)
  {
    set_motor(i, -1 * max_speed);
  }
}

void slide_left()
{
  /*
    front left motor clock wise - 0
    front right motor anti clock wise - 1
    rear left motor anti clock wise - 2
    rear right motor clock wise - 3
  */
  set_motor(0, max_speed);
  set_motor(1, -1 * max_speed);
  set_motor(2, -1 * max_speed);
  set_motor(3, max_speed);
}

void slide_right()
{
  /*
    front left motor anti clock wise - 0
    front right motor clock wise - 1
    rear left motor clock wise - 2
    rear right motor anti clock wise - 3
  */
  set_motor(0, -1 * max_speed);
  set_motor(1, max_speed);
  set_motor(2, max_speed);
  set_motor(3, -1 * max_speed);
}

void turn_left()
{
  /*
    front left motor anti clock wise - 0
    front right motor clock wise - 1
    rear left motor anti clock wise - 2
    rear right motor clock wise - 3
  */
  set_motor(0, -1 * max_speed);
  set_motor(1, max_speed);
  set_motor(2, -1 * max_speed);
  set_motor(3, max_speed);
}

void turn_right()
{
  /*
    front left motor clock wise - 0
    front right motor anti clock wise - 1
    rear left motor clock wise - 2
    rear right motor anti clock wise - 3
  */
  set_motor(0, max_speed);
  set_motor(1, -1 * max_speed);
  set_motor(2, max_speed);
  set_motor(3, -1 * max_speed);
}

void forward_left()
{
  /*
    front left motor clock wise - 0
    front right motor stop - 1
    rear left motor stop - 2
    rear right motor clock wise - 3
  */
  set_motor(0, max_speed);
  set_motor(1, 0);
  set_motor(2, 0);
  set_motor(3, max_speed);
}

void backward_right()
{
  /*
    front left motor anti clock wise - 0
    front right motor stop - 1
    rear left motor stop - 2
    rear right motor anti clock wise - 3
  */
  set_motor(0, -1 * max_speed);
  set_motor(1, 0);
  set_motor(2, 0);
  set_motor(3, -1 * max_speed);
}

void forward_right()
{
  /*
    front left motor stop - 0
    front right motor clock wise - 1
    rear left motor clock wise - 2
    rear right motor stop - 3
  */
  set_motor(0, 0);
  set_motor(1, max_speed);
  set_motor(2, max_speed);
  set_motor(3, 0);
}

void backward_left()
{
  /*
    front left motor stop - 0
    front right motor anti clock wise - 1
    rear left motor anti clock wise - 2
    rear right motor stop - 3
  */
  set_motor(0, 0);
  set_motor(1, -1 * max_speed);
  set_motor(2, -1 * max_speed);
  set_motor(3, 0);
}

void motors(int left_speed, int right_speed)
{
  //curved trajectory
  set_motor(0, left_speed);
  set_motor(2, left_speed);
  set_motor(1, right_speed);
  set_motor(3, right_speed);
}

void set_motor(uint8_t index, int motor_speed)
{
  if (motor_speed > 0)
  {
    //clock wise direction
    digitalWrite(dir_pin[index], LOW);
    analogWrite(motor_pin[index], motor_speed);
  }
  else if (motor_speed < 0)
  {
    //anti clock wise direction
    digitalWrite(dir_pin[index], HIGH);
    analogWrite(motor_pin[index], -1 * motor_speed);
  }
  else
  {
    //brake condition
    digitalWrite(brake_pin[index], HIGH);
  }
}

