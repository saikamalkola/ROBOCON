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
uint8_t ser_enable[4] = {46, 38, 30, 22};
uint8_t Jpulse[4] = {48, 40, 32, 24};

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

//Sensor Settings
char contrast = 90; //0 - 255
char brightness = 0x04; //0 to 10
char junction_width = 0x08; //0x02 to 0x08
char threshold = 0x04; //0 to 7
char line_mode = 0x00; //Light on Dark Background
char UART_Mode = 0x02; //byte of analog Value

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //For debugging on Serial Monitor
  Serial.flush();  
  init_motors();
  init_sensors();
  calibrate();  
}

void loop() {
  // put your main code here, to run repeatedly:
  double present = millis();
  read_sensors();
  if(sensor_data[0] < 70 && sensor_data[1] < 70)
  follow_forward(sensor_data[0],sensor_data[1]);
  else
  follow_forward(20,20);
  Serial.println(String(millis() - present));
}


void read_sensors()
{
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(ser_enable[i], LOW);  // Set Serial3EN to LOW to request UART data
    while (Serial3.available() <= 0);  // Wait for data to be available
    sensor_data[i] = Serial3.read();    // Read incoming data and store in variable positionVal
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }
  for (int i = 0; i < 4; i++)
  {
    Serial.print(String(sensor_data[i]) + " ");
  }
  // Serial.println("");
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
  send_command('T', threshold);
  delay(DELAY);
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
