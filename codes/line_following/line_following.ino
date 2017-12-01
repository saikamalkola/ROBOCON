#define max_speed 20

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

uint16_t front_sensor = 0;
uint16_t back_sensor = 0;
uint16_t left_sensor = 0;
uint16_t right_sensor = 0;

float Kp = 0, Kd = 0, Ki = 0;
float error = 0;
float P = 0, I = 0, D = 0;
float last_error = 0;
float correction = 0;
uint16_t set_position = 35;

void setup() {
  //Declaring Motor pins as OUTPUTS
  init_motors();
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
 /* for(int i = 0; i < 4; i++)
  {
    digitalWrite(brake_pin[i], LOW);
    digitalWrite(dir_pin[i], LOW);
    analogWrite(motor_pin[i],max_speed);
  }
  */
  follow_forward(20,20);
}

void read_sensors()
{

}
void PID()
{
  P = error;
  I = I + error;
  D = error - last_error;
  correction = (Kp * P) + (Ki * I) + (Kd * D);
  last_error = error;
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
