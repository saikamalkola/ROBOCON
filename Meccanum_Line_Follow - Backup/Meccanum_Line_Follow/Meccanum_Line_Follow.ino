void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); //For debugging on Serial Monitor
  Serial.flush();
  init_motors();
  init_sensors();
//  calibrate();
}

float wheel_vel[4];
int sensor_data[4];
boolean junction_data[4];
// Front, Back, Left, Right
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

void loop() {
  read_sensors(sensor_data, junction_data);

  get_wheel_vel(sensor_data, wheel_vel, instruction);
  motors(wheel_vel);
  
  if(((instructions[n] == 1   && junction_data[0] == 1) || 
      (instructions[n] == -1  && junction_data[1] == 1) || 
      (instructions[n] == 2   && junction_data[2] == 1) || 
      (instructions[n] == -2  && junction_data[3] == 1)    ) && at_junc == LOW){
    reinit_sensor(instructions[n]);
    instruction = 0;
    at_junc = HIGH;
  }
  if(at_junc == HIGH)
    iter++;
  if(at_junc == HIGH && iter >= 150){
    at_junc = LOW;
    iter = 0;
    n++;
    if (n<N){
      instruction = instructions[n];
    }
  }
  Serial.println(String(iter)+ " ");
}

