

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200); //For debugging on Serial Monitor
  Serial.flush();
  init_motors();
  init_sensors();
  calibrate();
}

float wheel_vel[4];
int sensor_data[4];
boolean junction_data[4];
// Front, Back, Left, Right
int instruction = 0;
/*
  0   - Junction
  1   - Follow Front
  -1  - Follow Back
  2   - Follow Left
  -2  - Follow Right
*/

void loop() {
  read_sensors(sensor_data, junction_data);
  get_wheel_vel(sensor_data, wheel_vel, instruction);
  motors(wheel_vel);
}







