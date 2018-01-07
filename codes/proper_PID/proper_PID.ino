#include "PinChangeInterrupt.h"

#define max_speed 0.8
#define DELAY 50

int dir = 1;
/*
  0   - Stop
  1   - Front
  -1  - Back
  2   - Left
  -2  - Right
*/

float follow_speed = 0.8;
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
uint8_t ser_enable[4] = {22, 30, 38, 46}; //F B L R
uint8_t Jpulse[4] = {24, 32, 40, 48};    //F B L R

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
boolean junction_data[4] = {0, 0, 0, 0};

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
float Kp[3] = {0.045, 0.045 , 0.045}, Kd[3] = {0.045, 0.045, 0.045}, Ki[3] = {0, 0};
float P[3] = {0, 0}, I[3] = {0, 0}, D[3] = {0, 0};
float correction[3] = {0, 0};
float error[3] = {0, 0};
float last_error[3] = {0, 0};
float set_position = 35;

/*
  Index
  0     -     Linear Velocity PID
  1     -     Angular Velocity PID
*/

//Wheel PID Variables

float max_vel = 200;
float wheel_Kp[4] = {0.5, 0.5, 0.5, 0.5} , wheel_Kd[4] = {0, 0.1, 0, 0};
float wheel_Ki[4] = {0};
float wheel_P[4] = {0, 0, 0, 0}, wheel_I[4] = {0, 0, 0, 0}, wheel_D[4] = {0, 0, 0, 0};

float int_thresh = 30;
float wheel_error[4] = {0, 0, 0, 0};
float wheel_last_error[4] = {0, 0, 0, 0};

float wheel_set_point[4] = {0, 100, 0, 0};
float wheel_correction[4] = {0, 0, 0, 0};

//UI Variables
int pid_sel = 0, log_sel = 0, sel_log_pid = 0, sel_line_pid = 0, sel_wheel_pid  = 0;
String cmd = "";
String response = "";
float data = 0;

//encoder variables
int outputA[4] = {50, A9, A12, A15};
int outputB[4] = {A11, A8, A13, A14};
float velocity[4] = {0};
float counter[4] = {0};
int present_state[4];
int prev_state[4];

unsigned long int present_ms = 0, previous_ms = 0, delta_t = 0;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.flush();
  Serial1.flush();
  init_motors();
  init_sensors();
  init_encoders();
  //calibrate();
}

void loop() {
  //read_sensors();
  //dir = 0;
  //cal_wheel_vel();
  cal_wheel_error();
  cal_wheel_PID();
  motors(w);
  bluetooth_handler();
  telemetry();
}


