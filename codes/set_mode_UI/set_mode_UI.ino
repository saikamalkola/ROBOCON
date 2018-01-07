#include "PinChangeInterrupt.h"

#define max_speed 0.8
#define follow_speed 0.8
#define DELAY 50

int Emax = 20, Emin = -20;
int dir = 1;
/*
  0   - Stop
  1   - Front
  -1  - Back
  2   - Left
  -2  - Right
*/

float base_speed =  follow_speed;
float J[4][3] = {{13.1579, -13.1579, -8.4211}, {13.1579, 13.1579, 8.4211}, {13.1579, 13.1579, -8.4211}, {13.1579, -13.1579, 8.4211}};
/*
  Conversion Matrix
  13.1579 -13.1579 -8.4211
  13.1579  13.1579  8.4211
  13.1579  13.1579 -8.4211
  13.1579 -13.1579  8.4211
*/
float Vsp[3] = {0, 0, -0.5};
//Vmax = 3.725
//Vx,Vy,W
float max_div = 255.0, max_rpm = 468.0;
float conv_factor[4] = {0.1334, 0.3667, 0.8, 2.6};
float pi = 3.1416;
float w[4] = {0, 0, 0, 0};
/*
  0 - FL
  1 - FR
  2 - BL
  3 - BR
*/

//Sensor Pins
uint8_t ser_enable[4] = {22, 30, 38, 46};   //F B L R
uint8_t Jpulse[4] = {24, 32, 40, 48};       //F B L R

//Motor Pins
uint8_t motor_pin[4] = {6, 3, 12, 9};
uint8_t dir_pin[4] = {5, 2, 11, 8};
uint8_t brake_pin[4] = {7, 4, 13, 10};
/*
  Pin mappings to motors
  0 - front left motor
  1 - front right motor
  2 - rear left motor
  3 - rear right motor
*/
//Sensor Data
char address = 0x00;
char sensor_addr[4] = {0x01, 0x02, 0x03, 0x04};
int sensor_data[4] = {0, 0, 0, 0};
boolean jun_data[4] = {0, 0, 0, 0};

//Sensor Settings
char contrast = 100; //0 - 255
char brightness = 0x04; //0 to 10
char junction_width = 0x08; //0x02 to 0x08
char threshold = 0x04; //0 to 7
char line_mode = 0x00; //Light on Dark Background
char UART_Mode = 0x02; //byte of analog Value

/*
  Left Line Following
  Kp and Kd 0.045
*/
//PID Variables
//PID Variables
float Kp[3] = {0.045, 0.045 , 0.09}, Kd[3] = {0.045, 0.045, 0.045}, Ki[3] = {0};
float P[3] = {0, 0}, I[3] = {0, 0}, D[3] = {0, 0};
float PID[3] = {0, 0};
float error[3] = {0, 0};
float last_error[3] = {0, 0};
float set_position = 35;

/*
  Index
  0     -     Linear Velocity PID
  1     -     Angular Velocity PID
*/

//Wheel PID Variables

float max_vel = 400;
float wheel_Kp[4] = {0, 3, 0,0} , wheel_Kd[4] = {0, 60, 0, 0};
float wheel_Ki[4] = {0,0.06,0,0};
float wheel_P[4] = {0, 0, 0, 0}, wheel_I[4] = {0, 0, 0, 0}, wheel_D[4] = {0, 0, 0, 0};

//Anti WindUp
float Imax = 10000/3;
float Imin = -10000/3;
float int_thresh = 30;
float wheel_error[4] = {0, 0, 0, 0};
float wheel_last_error[4] = {0, 0, 0, 0};

float wheel_set_point[4] = {0, 100, 0, 0};
float wheel_correction[4] = {0, 0, 0, 0};


//encoder variables
int outputA[4] = {50, A9, A12, A15};
int outputB[4] = {A11, A8, A13, A14};
float velocity[4] = {0};
float counter[4] = {0};
int present_state[4];
int prev_state[4];

unsigned long int present_ms = 0, previous_ms = 0, delta_t = 0;

int N = 50;

float readings[4][50];      // the readings from the analog input
int readIndex[4] = {0};              // the index of the current reading
float summation[4] = {0};                  // the running total
float average[4] = {0};                // the average

//UI Variables
int pid_sel = 0, log_sel = 0, sel_log_pid = 0, sel_line_pid = 0, sel_wheel_pid  = 0;
String cmd = "";
String response = "";
float data = 0;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200); //For debugging on Serial Monitor
  Serial.flush();
  Serial1.begin(9600);
  init_motors();
  init_sensors();
  init_encoders();
  init_buff();
  //calibrate();
}

void loop() {
  dir = 0;
  //  read_sensors();
  //  cal_error();
  //  cal_PID();
  //  set_Vsp();
  //  matrix_mult();
  if(Serial.available() > 0)
  wheel_Kd[1] = Serial.parseFloat();
  cal_wheel_error();
  cal_wheel_PID();
  motors(w);
  telemetry();
}

void set_Vsp()
{
  switch (abs(dir))
  {
    case 1:
      if (dir > 0)
      {
        Vsp[0] = base_speed;
        Vsp[1] = -1 * PID[0];
        Vsp[2] = -1 * PID[2];
      }
      else
      {
        Vsp[0] = -1 * base_speed;
        Vsp[1] = -1 * PID[0];
        Vsp[2] = -1 * PID[2];
      }
      break;
    case 2:
      if (dir > 0)
      {
        Vsp[0] = PID[1];
        Vsp[1] =  base_speed;
        Vsp[2] = -1 * PID[2];
      }
      else
      {
        Vsp[0] = PID[1];
        Vsp[1] = -1 * base_speed;
        Vsp[2] = -1 * PID[2];
      }
      break;
    default:
      Vsp[0] = 0;
      Vsp[1] = 0;
      Vsp[2] = 0;
  }
  if (dir == 0)
  {
    base_speed = 0;
    Vsp[0] = PID[1];
    Vsp[1] = -1 * PID[0];
    Vsp[2] = -1 * PID[2];
  }
  else {
    base_speed = follow_speed;
  }
}

void cal_error()
{
  float temp = 0;
  temp = (sensor_data[0] - set_position) - (sensor_data[1] - set_position);
  error[0] = temp / 2;
  temp = (sensor_data[2] - set_position) - (sensor_data[3] - set_position);
  error[1] = temp / 2;
  if (abs(dir) == 1 || abs(dir) == 0)
  {
    temp = (sensor_data[0] - set_position) + (sensor_data[1] - set_position);
    error[2] = temp / 2;
  }
  if (abs(dir) == 2)
  {
    temp = (sensor_data[2] - set_position) + (sensor_data[3] - set_position);
    error[2] = temp / 2;
  }
  //Serial.print("linear X Error: " + String(error[0]) + " ");
  //Serial.print("linear Y Error: " + String(error[1]) + " ");
  //Serial.print("Angular Error: " + String(error[2]) + " ");
}

void cal_PID()
{
  for (int i = 0; i < 3; i++)
  {
    P[i] = error[i];
    D[i] = error[i] - last_error[i];
    if (error[i] > Emax)
    {
      I[i] = I[i];
    }
    else if (error[i] < Emin)
    {
      I[i] = I[i];
    }
    else if (error[i] <= Emax && error[i] > Emin)
    {
      I[i] = I[i] + error[i];
    }
    PID[i] = (Kp[i] * P[i]) + (Kd[i] * D[i]) + (Ki[i] * I[i]);
    last_error[i] = error[i];
    if (abs(PID[i]) > max_speed)
    {
      if (PID[i] > 0)
        PID[i] = max_speed;
      else if (PID[i] < 0)
      {
        PID[i] = -1 * max_speed;
      }
    }
  }

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
    w[i] = w[i] * conv_factor[i]; //RpM to Arduino PWM value*/
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
  }
}


void read_sensors()
{
  int temp = 0;
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(ser_enable[i], LOW);  // Set Serial3EN to LOW to request UART data
    while (Serial3.available() <= 0);  // Wait for data to be available
    temp = Serial3.read();
    if (temp <= 70)
      sensor_data[i] = temp;
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }
  for (int i = 0; i < 4; i++)  {
    jun_data[i] = digitalRead(Jpulse[i]);
  }

}

void calibrate()
{
  for (int i = 0; i < 4; i++)
  {
    address = sensor_addr[i];
    send_command('C', 0x00);
    delay(DELAY);
  }
}
void set_sensor_settings()
{
  // Clear internal junction count of LSA08
  send_command('X', 0x00);
  delay(DELAY);
  // Setting LCD contrast
  send_command('S', contrast);
  delay(DELAY);
  // Setting LCD backlight
  send_command('B', brightness);
  delay(DELAY);
  // Setting junction width
  send_command('J', junction_width);
  delay(DELAY);
  //  send_command('T', threshold);
  //  delay(DELAY);
  // Setting line mode
  send_command('L', line_mode);
  delay(DELAY);
  // Setting UART ouput Mode
  send_command('D', UART_Mode);
  delay(DELAY);
}

void init_sensors()
{
  Serial3.begin(9600); // Start Serial3 communication
  Serial3.flush();   // Clear Serial3 buffer
  for (int i = 0; i < 4; i++)
  {
    // pinMode(Jpulse[i], INPUT);
    pinMode(ser_enable[i], OUTPUT);
    digitalWrite(ser_enable[i], HIGH);
  }
  delay(DELAY);
  for (int i = 0; i < 4; i++)
  {
    address = sensor_addr[i];
    set_sensor_settings();
  }
}

void send_command(char command, char data) {

  char checksum = address + command + data;

  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(data);
  Serial3.write(checksum);

}
void set_motor(uint8_t index, int motor_speed)
{
  if (index % 2 == 0)
    motor_speed = -1 * motor_speed;
  if (abs(motor_speed) < 5)
    motor_speed = 0;
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
