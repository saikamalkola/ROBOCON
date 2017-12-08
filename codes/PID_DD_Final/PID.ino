//PID Variables
float Kp   = 0, Kd   = 0, Ki   = 0;
float P   = 0, I   = 0, D   = 0;
float PID   = 0;
float error   = 0;
float last_error   = 0;
float set_position = 35;
/*
  Index
  0     -    Front PID
  1     -    Back PID
*/

void cal_error()
{
  error = sensor_data[dir] - set_position;
  Serial.println("Error: " + String(error) + " ");
}

void cal_PID()
{
  P  = error ;
  I  = I  + error ;
  D  = error  - last_error ;

  PID  = (Kp  * P ) + (Kd  * D ) + (Ki  * I );
  last_error  = error ;
  left  = max_speed + PID ;
  right  = max_speed - PID ;
  if (left > max_speed)
    left = max_speed;
  if (right > max_speed)
    right = max_speed;
}
