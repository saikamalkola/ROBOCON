#include <SoftwareSerial.h>

#define rxPin 2
#define txPin 3

uint8_t push_pwm = 6;
uint8_t push_dir = 5;

uint8_t lift_pwm = 10;
uint8_t lift_dir = 9;


SoftwareSerial mySerial(rxPin, txPin); // RX, Tx
int rack_positions[5] = { -600, -300, 0, 300, 600};
boolean line_mode = 0;
boolean speed_mode = 0;
boolean push_mode = 0;
boolean us_mode = 0;
boolean lift_mode = 0;
int rack_data = 0;
float damp = 0.001;

float Vx = 512, Vy = 512, W = 512;

String response = "";
int control_data[9] = {0};

void setup()
{
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(push_pwm, OUTPUT);
  pinMode(push_dir, OUTPUT);
  pinMode(lift_pwm, OUTPUT);
  pinMode(lift_dir, OUTPUT);
  Serial.begin(115200);
  mySerial.begin(9600);
  delay(250);
  set_servo_settings();
}

void loop() // run over and over
{
  if (Serial.available() > 0)
  {
    response = Serial.readStringUntil('\n');
    parse_response();
  }
//  Serial.println(rack_data);
//  if (rack_data == 128)
//  {
//    mySerial.println("M100");
//    mySerial.println("R100");
//    delay(1000);
//  }
//  if (rack_data == 8)
//  {
//    mySerial.println("M-100");
//    mySerial.println("R-100");
//    delay(1000);
//  }

  if (!push_mode)
  {
    digitalWrite(push_dir, HIGH);
    digitalWrite(push_pwm, LOW);
  }
  else
  {
    digitalWrite(push_dir, HIGH);
    digitalWrite(push_pwm, HIGH);
  }
  if (!lift_mode)
  {
    digitalWrite(lift_dir, HIGH);
    digitalWrite(lift_pwm, LOW);
  }
  else
  {
    digitalWrite(lift_dir, LOW);
    digitalWrite(lift_pwm, HIGH);
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
    Serial.print(control_data[i]);
    Serial.print(" ");
  }
  Serial.println("");
  us_mode = control_data[3];
  line_mode = control_data[4];
  speed_mode = control_data[2];
  lift_mode = control_data[0];
  push_mode = control_data[1];
  rack_data = control_data[5];
  Vy = (float)control_data[6];
  Vx = (float)control_data[7];
  W = (float)control_data[8];
}

void set_rack_pos(int pos)
{
  String cmd = "R";
  cmd = cmd + (String)pos;
  mySerial.println(cmd);
}

void set_servo_settings()
{
  mySerial.println("M10");
  delay(250);
  Serial.println("P0");    //Set current encoder position as 0
  delay(2000);
}

