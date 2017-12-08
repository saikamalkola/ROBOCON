#define max_speed 50
#define DELAY 50

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
float Kp[2] = {1, 1}, Kd[2] = {0, 0}, Ki[2] = {0, 0};
float P[2] = {0, 0}, I[2] = {0, 0}, D[2] = {0, 0};
float PID[2] = {0, 0};
float error[2] = {0, 0};
float last_error[2] = {0, 0};
float set_position = 35;
int left[2] = {0, 0}, right[2] = {0, 0};
/*
  Index
  0     -    Front PID
  1     -    Back PID
*/
boolean dir = 0;
/*
  0 - Follow Straight
  1 - Follow Sideways
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
  motors();
}

void cal_error()
{
  error[0] = sensor_data[dir] - set_position;
  error[1] = set_position - sensor_data[dir + 2];
  Serial.print("Front Error: " + String(error[0]) + " ");
  Serial.println("Back Error: " + String(error[1]));
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
    left[i] = max_speed + PID[i];
    right[i] = max_speed - PID[i];
    if (left[i] > max_speed)
      left[i] = max_speed;
    if (right[i] > max_speed)
      right[i] = max_speed;
    Serial.print(String(PID[i]) + " ");
  }
  Serial.println("");

}
void motors()
{

  if (dir == 0)
  {
    set_motor(0, left[0]);
    set_motor(1, right[0]);
    set_motor(2, left[1]);
    set_motor(3, right[1]);
  }
  if (dir == 1)
  {
    set_motor(0, -1 * right[0]);
    set_motor(1, right[0]);
    set_motor(2, left[0]);
    set_motor(3, -1 * left[1]);
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
    if (temp > 70)
      sensor_data[i] = sensor_data[i];    // Read incoming data and store in variable positionVal
    else
      sensor_data[i] = temp;
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }
    for (int i = 0; i < 4; i++)
  {
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
