#define max_speed 0.8
#define follow_speed 0.8
#define DELAY 50

//Indication Variables
uint8_t led_pin = 25;
uint8_t buzzer_pin = 27;

//Controller Variables
int control_data[9] = {0};

float max_vel = 400;
float base_speed =  follow_speed;
float J[3][3] = {{ -4.386, 7.6, 2.193}, { -4.386, -7.6, 2.193}, {8.772, 0, 2.193}};
/*
  Conversion Matrix
  -4.386  7.6 2.193
  -4.386 -7.6 2.193
   8.772  0   2.193
*/
float Vsp[3] = { 0, 0, 0};
//Vmax = 3.725
//Vx,Vy,W
float max_div = 255.0, max_rpm = 468.0;
float conv_factor[4] = {0.1334, 0.3667, 0.8, 2.6};
float pi = 3.1416;
float w[3] = {0, 0, 0};

//Sensor Pins
uint8_t ser_enable[2] = {30, 38};   //F B
uint8_t Jpulse[2] = {32, 40};       //F B

//Motor Pins
uint8_t motor_pin[3] = {3, 5, 7};
uint8_t dir_pin[3] = {2, 4, 6};
/*
  Pin mappings to motors
  0 - front right motor
  1 - front lefr motor
  2 - rear motor
*/
//Sensor Data
char address = 0x00;
char sensor_addr[2] = {0x01, 0x02};
int sensor_data[2] = {0, 0};
boolean jun_data[2] = {0, 0};

//Sensor Settings
char contrast = 100; //0 - 255
char brightness = 0x04; //0 to 10
char junction_width = 0x08; //0x02 to 0x08
char threshold = 0x04; //0 to 7
char line_option = 0x00; //Light on Dark Background
char UART_Mode = 0x02; //byte of analog Value

/*
  Left Line Following
  Kp and Kd 0.045
*/
//PID Variables
//PID Variables
float Kp[2] = {0.032, 0.032}, Kd[2] = {0.012, 0.012}, Ki[2] = {0, 0};
float P[2] = {0, 0}, I[3] = {0, 0}, D[3] = {0, 0};
float PID[2] = {0, 0};
float error[2] = {0, 0};
float last_error[2] = {0, 0};
float set_position = 35;

/*
  Index
  0     -     Linear Velocity PID
  1     -     Angular Velocity PID
*/
//distance PID Varibles
float dist_Kp = 0.025, dist_Kd = 0, dist_Ki = 0;
float dist_P = 0, dist_I = 0, dist_D = 0;
float dist_error = 0;
float dist_last_error = 0;
float dist_PID = 0;
float dist_set_position = 300;

unsigned long int present_ms = 0, previous_ms = 0, last_ms = 0, delta_t = 0;

boolean line_mode = 0;
boolean speed_mode = 0;
boolean push_mode = 0, prev_push_mode = 0;
boolean us_mode = 0, prev_us_mode = 0;
boolean lift_mode = 0, prev_lift_mode = 0;
int rack_data = 0;
float damp = 0.001;

float Vx = 512, Vy = 512, W = 512;

char buff[32] = {0};

//Ultrasonic Variables
unsigned int MSByteDist = 0;
unsigned int LSByteDist = 0;
unsigned int mmDist = 0, distance = 0;

String response = "";
void setup()
{
  Serial.begin(115200);
  Serial.flush();
  Serial2.begin(9600);
  init_indicators();
  init_motors();
  init_sensors();
  //calibrate();
}

void loop()
{
  read_serial();
  read_sensors();
  if (line_mode == 0)
  {
    digitalWrite(led_pin, LOW);
  }
  else
  {
    digitalWrite(led_pin, HIGH);
  }
  if (prev_push_mode != push_mode && push_mode == 1)
  {
    tone(buzzer_pin, 292, 200);
    Serial.flush();

  }
  else if (prev_push_mode != push_mode && push_mode == 0)
  {
    Serial.flush();
    tone(buzzer_pin, 292, 400);
  }
  if (prev_lift_mode != lift_mode && lift_mode == 1)
  {
    tone(buzzer_pin, 100, 200);
    Serial.flush();

  }
  else if (prev_lift_mode != lift_mode && lift_mode == 0)
  {
    Serial.flush();
    tone(buzzer_pin, 100, 400);
  }
  if (prev_us_mode != us_mode && us_mode == 1)
  {
    tone(buzzer_pin, 500, 200);
    Serial.flush();

  }
  else if (prev_us_mode != us_mode && us_mode == 0)
  {
    Serial.flush();
    tone(buzzer_pin, 500, 400);
  }
  prev_us_mode = us_mode;
  prev_push_mode = push_mode;
  prev_lift_mode = lift_mode;
  actuate();
  //  Serial.print(sensor_data[0]);
  //  Serial.print(" ");
  //  Serial.print(sensor_data[1]);
  //  Serial.println(" ");
}

void init_indicators()
{
  pinMode(led_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
}

void cal_dist_PID()
{
  dist_P =  dist_error;
  dist_D =  dist_error -  dist_last_error;

  dist_PID = ( dist_Kp *  dist_P) + ( dist_Kd *  dist_D) + ( dist_Ki *  dist_I);
  dist_last_error =  dist_error;
  if (abs( dist_PID) > max_speed)
  {
    if ( dist_PID > 0)
    {
      dist_PID = max_speed;
    }
    else if ( dist_PID < 0)
    {
      dist_PID = -1 * max_speed;
    }
  }
}

void cal_dist_error()
{
  dist_error = dist_set_position - distance;
}

void read_dist()
{
  Serial2.flush();
  Serial2.write(0x55);
  delay(1);
  if (Serial2.available() >= 2)
  {
    MSByteDist = Serial2.read();
    LSByteDist = Serial2.read();
    mmDist = MSByteDist * 256 + LSByteDist;
  }
  if ((mmDist > 1) && (mmDist < 100000))
  {
    distance = mmDist;
  }
  Serial2.flush();
}

void read_serial()
{
  if (Serial.available() > 0)
  {
    response = Serial.readStringUntil('\n');
    parse_response();
    Serial.flush();
  }
}

void actuate()
{
  if (line_mode == 1)
  {
    cal_error();
    cal_PID();
    if (us_mode)
    {
      read_dist();
      //Serial.println(distance);
      cal_dist_error();
      cal_dist_PID();
    }
  }
  set_Vsp();
  //    Vsp[0] = (Vx - 512) * damp;
  //    Vsp[1] = (Vy - 512) * damp;
  //    Vsp[2] = (-W + 512) * damp;
  matrix_mult();
  motors(w);
}

void parse_response()
{
  int l = response.length(), k = 0;
  int limits[100] = {0};
  String temp = " ";
  if (response[0] != '#')
  {
    return;
  }
  for (int i = 0; i < l ; i++)
  {
    if (response[i] == '#')
    {
      limits[k] = i + 1;
      k++;
    }
  }
  for (int i = 0; i < (k - 1) ; i++)
  {
    temp = (response.substring(limits[i], limits[i + 1] - 1));
    control_data[i] = temp.toInt();
  }
  us_mode = control_data[0];
  line_mode = control_data[1];
  speed_mode = control_data[2];
  lift_mode = control_data[3];
  push_mode = control_data[4];
  rack_data = control_data[5];
  Vy = (float)control_data[6];
  Vx = (float)control_data[7];
  W = (float)control_data[8];
  if (speed_mode == 1)
  {
    damp = 0.008;
  }
  else
  {
    damp = 0.004;
  }
}

void set_Vsp()
{
  if (us_mode == 0)
  {
    if (line_mode == 1)
    {
      Vsp[0] = 1 * PID[0];
      Vsp[1] = (-Vy + 512) * damp;
      Vsp[2] = -1 * PID[1];
    }
    else
    {
      Vsp[0] = (-Vx + 512) * damp;
      Vsp[1] = (-Vy + 512) * damp;
      Vsp[2] = (W -  512) * damp;
    }
  }
  else
  {
    if (line_mode == 1)
    {
      Vsp[0] = 1 * PID[0];
      Vsp[1] = -dist_PID;;
      Vsp[2] = -1 * PID[1];
    }
    else
    {
      Vsp[0] = (-Vx + 512) * damp;
      Vsp[1] = (-Vy + 512) * damp;
      Vsp[2] = (W - 512) * damp;
    }
  }
}

void cal_error()
{
  float temp = 0;
  temp = (sensor_data[0] - set_position) - (sensor_data[1] - set_position);
  error[0] = temp / 2;
  temp = (sensor_data[0] - set_position) + (sensor_data[1] - set_position);
  error[1] = temp / 2;
}

void cal_PID()
{
  for (int i = 0; i < 2; i++)
  {
    P[i] = error[i];
    D[i] = error[i] - last_error[i];

    PID[i] = (Kp[i] * P[i]) + (Kd[i] * D[i]);
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

void matrix_mult()
{
  float sum = 0;
  for (int i = 0; i < 3; i++)
  {
    sum  = 0;
    for (int j = 0; j < 3; j++)
    {
      sum += (J[i][j] * Vsp[j]);
    }
    w[i] = sum;
    w[i] = w[i] * 60 / (2 * pi); //rps to RpM
    if (abs(w[i]) > max_vel)
    {
      if (w[i] > 0)
      {
        w[i] = max_vel;
      }
      else
      {
        w[i] = -1 * max_vel;
      }
    }
    //    int sign = 0;
    //    if (w[i] >= 0)
    //    {
    //      sign = 1;
    //    }
    //    else
    //    {
    //      sign = -1;
    //    }
    //    if (abs(w[i]) <= 300)
    //    {
    //      w[i] = w[i] * conv_factor[0];
    //    }
    //    if (abs(w[i]) > 300 && abs(w[i]) <= 390)
    //    {
    //      w[i] = (w[i] * conv_factor[1]) - sign * 70;
    //    }
    //    if (abs(w[i]) > 390 && abs(w[i]) <= 420)
    //    {
    //      w[i] = (w[i] * conv_factor[2]) - sign * 239;
    //    }
    //    if (abs(w[i]) > 420 && abs(w[i]) < 480)
    //    {
    //      w[i] = (w[i] * conv_factor[3]) - sign * 995;
    //    }
  }
}

void read_sensors()
{
  int temp = 35;
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(ser_enable[i], LOW);  // Set Serial3EN to LOW to request UART data
    while (Serial3.available() <= 0);  // Wait for data to be available
    temp = Serial3.read();
    if (temp <= 70)
      sensor_data[i] = temp;
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }
  for (int i = 0; i < 2; i++)  {
    jun_data[i] = digitalRead(Jpulse[i]);
  }
}

void calibrate()
{
  for (int i = 0; i < 2; i++)
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
  //   Setting line mode
  send_command('L', line_option);
  delay(DELAY);
  // Setting UART ouput Mode
  send_command('D', UART_Mode);
  delay(DELAY);
}

void init_sensors()
{
  Serial3.begin(9600); // Start Serial3 communication
  Serial3.flush();   // Clear Serial3 buffer
  for (int i = 0; i < 2; i++)
  {
    pinMode(Jpulse[i], INPUT);
    pinMode(ser_enable[i], OUTPUT);
    digitalWrite(ser_enable[i], HIGH);
  }
  delay(DELAY);
  for (int i = 0; i < 2; i++)
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

void motors(float motor_speed[3])
{
  for (int i = 0; i < 3; i++)
  {
    set_motor(i, motor_speed[i]);
  }
}

void set_motor(uint8_t index, int motor_speed)
{
  if (abs(motor_speed) < 5)
    motor_speed = 0;
  if (abs(motor_speed) > 255)
  {
    if (motor_speed > 0)
      motor_speed = 255;
    else if (motor_speed < 0)
    {
      motor_speed = -255;
    }
  }
  if (motor_speed >= 0)
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
}

void init_motors()
{
  for (int i = 0; i < 3; i++)
  {
    pinMode(motor_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT);
  }
}
