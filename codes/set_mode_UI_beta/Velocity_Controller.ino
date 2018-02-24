void cal_wheel_error()
{
  get_velocity();
  cal_avg();
  for (int i = 0; i < 4; i++)
    velocity[i] = average[i];

  for (int i = 0; i < 4; i++)
  {
    wheel_set_point[i] = w_average[i];
    wheel_error[i] = wheel_set_point[i] - velocity[i];
  }
}

void cal_wheel_PID()
{
  for (int i = 0; i < 4; i++) {
    wheel_P[i] = wheel_error[i];
    if ((wheel_I[i] + wheel_error[i]) > Imax)
    {
      wheel_I[i] = Imax;
    }
    else if ((wheel_I[i] + wheel_error[i]) < Imin)
    {
      wheel_I[i] = Imin;
    }
    else
    {
      wheel_I[i] = (wheel_I[i] + wheel_error[i]) ;
    }
    //wheel_I[i] = wheel_I[i] + wheel_error[i];
    wheel_D[i] = wheel_error[i] - wheel_last_error[i];
    wheel_correction[i] = (wheel_Kp[i] * wheel_P[i]) + (wheel_Kd[i] * wheel_D[i]) + (wheel_Ki[i] * wheel_I[i]);
    wheel_last_error[i] = wheel_error[i];

    w[i] =  wheel_correction[i]; //rpm

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
    int sign = 0;
    if (w[i] >= 0)
    {
      sign = 1;
    }
    else
    {
      sign = -1;
    }
    if (abs(w[i]) <= 300)
    {
      w[i] = w[i] * conv_factor[0];
    }
    if (abs(w[i]) > 300 && abs(w[i]) <= 390)
    {
      w[i] = (w[i] * conv_factor[1]) - sign * 70;
    }
    if (abs(w[i]) > 390 && abs(w[i]) <= 420)
    {
      w[i] = (w[i] * conv_factor[2]) - sign * 239;
    }
    if (abs(w[i]) > 420 && abs(w[i]) < 480)
    {
      w[i] = (w[i] * conv_factor[3]) - sign * 995;
    }
  }
}

