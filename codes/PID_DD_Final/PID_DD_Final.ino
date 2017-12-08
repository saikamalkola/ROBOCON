#define max_speed 50
#define DELAY 50

boolean dir = 0;
/*
  FBLR
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




