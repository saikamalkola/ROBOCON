float max_speed = 50;

uint8_t push_dir = 3;
uint8_t lift_dir = 6;

uint8_t motor_pin = 10;
uint8_t dir_pin = 11;

boolean line_mode = 0;
boolean speed_mode = 0;
boolean push_mode = 0;
boolean us_mode = 0;
boolean lift_mode = 0;
int rack_data = 0;
float damp = max_speed / 512;

float Vx = 512, Vy = 512, W = 512;

String response = "";
int control_data[9] = {0};

void setup()
{
  init_sols();
  init_motors();
  Serial.begin(115200);
}

void loop() // run over and over
{
  read_serial();
  update_sols();
  if (line_mode)
  {
    digitalWrite(13,HIGH);
    motor(W);
  }
  else
  {
    digitalWrite(13,LOW);
    motor(0);
  }
}

void motor(int motor_speed)
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
    digitalWrite(dir_pin, LOW);
    analogWrite(motor_pin, motor_speed);
  }
  else if (motor_speed < 0)
  {
    //anti clock wise direction
    digitalWrite(dir_pin, HIGH);
    analogWrite(motor_pin, -1 * motor_speed);
  }
}

void update_sols()
{
  if (!push_mode)
  {
    digitalWrite(push_dir, HIGH);
  }
  else
  {
    digitalWrite(push_dir, LOW);
  }
  if (!lift_mode)
  {
    digitalWrite(lift_dir, HIGH);
  }
  else
  {
    digitalWrite(lift_dir, LOW);
  }
}

void read_serial()
{
  if (Serial.available() > 0)
  {
    response = Serial.readStringUntil('\n');
    parse_response();
  }
}

void parse_response()
{
  int l = response.length(), k = 0;
  int limits[100] = {0};
  String temp = " ";
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
  //  Serial.print(control_data[i]);
  //  Serial.print(" ");
  }
  //Serial.println("");
  us_mode = control_data[0];
  line_mode = control_data[1];
  speed_mode = control_data[2];
  lift_mode = control_data[3];
  push_mode = control_data[4];
  rack_data = control_data[5];
  Vy = (float)control_data[6];
  Vx = (float)control_data[7];
  W = (float)control_data[8];
  W = (W -  512) * damp;
}

void init_motors()
{
  pinMode(motor_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
}

void init_sols()
{pinMode(13, OUTPUT);
  pinMode(push_dir, OUTPUT);
  pinMode(lift_dir, OUTPUT);
}
