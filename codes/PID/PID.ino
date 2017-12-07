#define max_speed 0.5
#define DELAY 50

float J[4][3] = {{13.1579, -13.1579, -8.4211}, {13.1579, 13.1579, 8.4211}, {13.1579, 13.1579, -8.4211}, {13.1579, -13.1579, 8.4211}};
/*
  Conversion Matrix
  13.1579 -13.1579 -8.4211
  13.1579  13.1579  8.4211
  13.1579  13.1579 -8.4211
  13.1579 -13.1579  8.4211
*/
float Vsp[3] = {0.5, 0, 0};
float base_speed = 0.4; //in m/s
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

//Sensor Pins
uint8_t ser_enable[4] = {46, 38, 30, 22};   //F B L R
uint8_t Jpulse[4] = {48, 40, 32, 24};       //F B L R

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

//PID Variables
float Kp[2] = {0.005, 0.005}, Kd[2] = {0, 0}, Ki[2] = {0, 0};
float P[2] = {0, 0}, I[2] = {0, 0}, D[2] = {0, 0};
float PID[2] = {0, 0};
float error[2] = {0, 0};
float last_error[2] = {0, 0};
float set_position = 35;

/*
  Index
  0     -     Linear Velocity PID
  1     -     Angular Velocity PID
*/

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200); //For debugging on Serial Monitor
  Serial.flush();
  init_motors();
  init_sensors();
  calibrate();
}

void loop() {
  read_sensors();
  cal_error();
  cal_PID();
  set_Vsp();
  matrix_mult();
  motors(w);
}

void set_Vsp()
{
  Vsp[0] = base_speed;
  Vsp[1] = PID[0];
  Vsp[2] = PID[1];
}
void cal_error()
{
  float temp = 0;
  temp = (sensor_data[0] - set_position) - (sensor_data[1] - set_position);
  error[0] = temp / 2;
  temp = (sensor_data[0] - set_position) + (sensor_data[1] - set_position);
  error[1] = temp;
  Serial.print("linear Error: " + String(error[0]) + " ");
  Serial.println("Angular Error: " + String(error[1]));
}

void cal_PID()
{
  for (int i = 0; i < 2; i++)
  {
    P[i] = error[i];
    I[i] = I[i] + error[i];
    D[i] = error[i] - last_error[i];

    PID[i] = (Kp[i] * P[i]) + (Kd[i] * D[i]) + (Ki[i] * I[i]);
    last_error[i] = error[i];
    if(PID[i] > max_speed)
      PID[i] = max_speed;
    Serial.print(String(PID[i]) + " ");
  }
  Serial.println("");
  
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


void read_sensors()
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(ser_enable[i], LOW);  // Set Serial3EN to LOW to request UART data
    while (Serial3.available() <= 0);  // Wait for data to be available
    sensor_data[i] = Serial3.read();    // Read incoming data and store in variable positionVal
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }

  for (int i = 0; i < 4; i++)
  {
    jun_data[i] = digitalRead(Jpulse[i]);
  }
  // Serial.print("Sensor Data : ");
  for (int i = 0; i < 4; i++)
  {
    //  Serial.print(String(sensor_data[i]) + " ");
  }
  //Serial.print("Junction Data : ");
  for (int i = 0; i < 4; i++)
  {
    //  Serial.print(String(jun_data[i]) + " ");
  }
  //Serial.println("");
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
  //send_command('T', threshold);
  //delay(DELAY);
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
