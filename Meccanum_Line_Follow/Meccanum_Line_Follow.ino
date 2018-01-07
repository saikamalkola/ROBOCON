void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); //For debugging on Serial Monitor
  Serial.flush();
  init_motors();
  init_sensors();
  init_encoders();
  //  calibrate();
}

float wheel_vel[4];
int sensor_data[4];
boolean junction_data[4];
// Front, Back, Left, Right
float w[4] = {0, 0, 0, 0};
int instructions[] = {1, 2, -2, 1, 2, -2, -1};
int instruction = instructions[0];
int n = 0, N = 7;//size(instructions)/2;
/*
  0   - Junction
  1   - Follow Front
  -1  - Follow Back
  2   - Follow Left
  -2  - Follow Right
*/
long iter = 0;
boolean at_junc = LOW;

//float wheel_vel_temp[4] = {10 , 20, 30, 100};
void loop() {
//  read_sensors(sensor_data, junction_data);
//  instruction = 2;
 // get_wheel_vel(sensor_data, wheel_vel, instruction);
 cal_wheel_error();
 cal_wheel_PID();
  // float wi[4] = {50,50,50,50};
  motors(w);
}

