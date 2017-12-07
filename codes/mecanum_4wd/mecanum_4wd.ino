#define max_speed 150
#define DELAY 50

float J[4][3] = {{13.1579, -13.1579, -8.4211}, {13.1579, 13.1579, 8.4211}, {13.1579, 13.1579, -8.4211}, {13.1579, -13.1579, 8.4211}};
/*
  Conversion Matrix
  13.1579 -13.1579 -8.4211
  13.1579  13.1579  8.4211
  13.1579  13.1579 -8.4211
  13.1579 -13.1579  8.4211
*/
float Vsp[3] = {0, 0, 0};
float base_speed = 0.5; //in m/s
//Vmax = 3.725
//Vx,Vy,W
float max_div = 255.0, max_rpm = 468.0;
float conv_factor = max_div / max_rpm;
float pi = 3.1416;
float w[4] = {0, 0, 0, 0};
/*
  0 - FL
  1 - FR
  2 - BL
  3 - BR
*/
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
char sel = 0;
boolean flag = 0;
void setup() {
  Serial.begin(115200); //For debugging on Serial Monitor
  Serial.flush();
  init_motors();
}

void loop() {
    Vsp[0] = 0.85;
    Vsp[1] = 0;
    Vsp[2] = 0;
        matrix_mult();
    motors(w);
    delay(1000);
        Vsp[0] = -0.85;
    Vsp[1] = 0;
    Vsp[2] = 0;
        matrix_mult();
    motors(w);
    delay(1000);
 /* for(int k = 0; k < 2; k++)
  {
  for (int i = 0; i < 20; i++)
  {
    Vsp[0] = i * 0.05 * pow(-1,k);
    Vsp[1] = 0;
    Vsp[2] = 0;
    matrix_mult();
    motors(w);
    delay(100);
  }
  for (int i = 19; i >=0; i--)
  {
    Vsp[0] = i * 0.05 * pow(-1,k);
    Vsp[1] = 0;
    Vsp[2] = 0;
    matrix_mult();
    motors(w);
    delay(100);
  }
  }
  for(int k = 0; k < 2; k++)
  {
  for (int i = 0; i < 20; i++)
  {
    Vsp[0] = 0;
    Vsp[1] = i * 0.05 * pow(-1,k);
    Vsp[2] = 0;
    matrix_mult();
    motors(w);
    delay(100);
  }
  for (int i = 19; i >=0; i--)
  {
    Vsp[0] = 0;
    Vsp[1] = i * 0.05 * pow(-1,k);
    Vsp[2] = 0;
    matrix_mult();
    motors(w);
    delay(100);
  }
  }
  
  for(int k = 0; k < 2; k++)
  {
  for (int i = 0; i < 20; i++)
  {
    Vsp[0] = 0;
    Vsp[1] = 0;
    Vsp[2] = i * 0.05 * pow(-1,k);
    matrix_mult();
    motors(w);
    delay(100);
  }
  for (int i = 19; i >=0; i--)
  {
    Vsp[0] = 0;
    Vsp[1] = 0;
    Vsp[2] = i * 0.05 * pow(-1,k);
    matrix_mult();
    motors(w);
    delay(100);
  }
  }*/
}

void motors(float motor_speed[4])
{
  for (int i = 0; i < 4; i++)
  {
    set_motor(i, motor_speed[i]);
  }
}

void matrix_mult()
{
  float sum = 0;
  for (int i = 0; i < 4; i++)
  {
    sum  = 0;
    for (int j = 0; j < 3; j++)
    {
      sum += (J[i][j] * Vsp[j]);
    }
    w[i] = sum;
    w[i] = w[i] * 60 / (2 * pi); //rps to RpM
    w[i] = w[i] * conv_factor; //RpM to Arduino PWM value*/
    if (abs(w[i]) > 255)
    {
      if (w[i] > 0)
      {
        w[i] = 255;
      }
      else
      {
        w[i] = -255;
      }
    }
    //Serial.print(String(w[i]) + " ");
  }
  // Serial.println("");
}



void set_motor(uint8_t index, int motor_speed)
{
  if (index % 2 == 0)
    motor_speed = -1 * motor_speed;
  if (motor_speed > 0)
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
