
float max_vel = 400;
float wheel_Kp[4] = {0.5, 0.1, 0.5, 0.5} , wheel_Kd[4] = {0.1, 0, 0.1, 0.1};
float wheel_Ki[4] = {0};//{0.1, 0.1, 0.1, 0.1};
float wheel_P[4] = {0, 0, 0, 0}, wheel_I[4] = {0, 0, 0, 0}, wheel_D[4] = {0, 0, 0, 0};

float int_thresh = 30;
float wheel_error[4] = {0, 0, 0, 0};
float wheel_last_error[4] = {0, 0, 0, 0};

float wheel_set_point[4] = {0, 100, 0, 0};
float wheel_correction[4] = {0, 0, 0, 0};

void cal_wheel_error()
{
  get_velocity();
  for (int i = 0; i < 4; i++)
  {
    //wheel_set_point[i] = w[i];
    wheel_error[i] = wheel_set_point[i] - velocity[i];
  }
}

void cal_wheel_PID()
{
  for (int i = 0; i < 4; i++) {
    wheel_P[i] = wheel_error[i];
    if(wheel_error[i] < int_thresh)
    {
      wheel_I[i] = wheel_I[i] + wheel_error[i];
    }
    wheel_D[i] = wheel_error[i] - wheel_last_error[i];
    wheel_correction[i] = (wheel_Kp[i] * wheel_P[i]) + (wheel_Kd[i] * wheel_D[i]) + (wheel_Ki[i] * wheel_I[i]);
    wheel_last_error[i] = wheel_error[i];
    w[i] = wheel_set_point[i] + wheel_correction[i]; //rpm
    if (abs(w[i]) > max_vel)
    {
     if(w[i] > 0)
     {
      w[i] = max_vel;
     }
     else
     {
      w[i] = -1 * max_vel;
     }
    }
    w[i] = w[i] * conv_factor;

    Serial.print(String(wheel_set_point[i]) + "\t");
  }
  Serial.println("");
}

