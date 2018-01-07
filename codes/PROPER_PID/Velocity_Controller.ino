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
    if (wheel_error[i] < int_thresh)
    {
      wheel_I[i] = wheel_I[i] + wheel_error[i];
    }
    wheel_D[i] = wheel_error[i] - wheel_last_error[i];
    wheel_correction[i] = (wheel_Kp[i] * wheel_P[i]) + (wheel_Kd[i] * wheel_D[i]) + (wheel_Ki[i] * wheel_I[i]);
    wheel_last_error[i] = wheel_error[i];

    w[i] = wheel_set_point[i] + wheel_correction[i]; //rpm

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
    w[i] = w[i] * 255/468;

//    if (abs(w[i]) <= 300)
//    {
//      w[i] = w[i] * conv_factor[0];
//    }
//    if (abs(w[i]) > 300 && abs(w[i]) <= 390)
//    {
//      w[i] = w[i] * conv_factor[1];
//    }
//    if (abs(w[i]) > 390 && abs(w[i]) <= 420)
//    {
//      w[i] = w[i] * conv_factor[2];
//    }
//    if (abs(w[i]) > 420 && abs(w[i]) < 480)
//    {
//      w[i] = w[i] * conv_factor[3];
//    }
  }
}

