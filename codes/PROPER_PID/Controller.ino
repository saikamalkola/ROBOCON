
void matrix_mult()
{
  float sum = 0;
  for (int i = 0; i < 4; i++)
  {
    sum  = 0;
    for (int j = 0; j < 3; j++)
    {
      sum += (J[i][j] * Vsp[j]);
    }
    w[i] = sum;
    w[i] = w[i] * 60 / (2 * pi); //rps to RpM
    if (abs(w[i]) <= 300)
    {
      w[i] = w[i] * conv_factor[0];
    }
    if (abs(w[i]) > 300 && abs(w[i]) <= 390)
    {
      w[i] = w[i] * conv_factor[1];
    }
    if (abs(w[i]) > 390 && abs(w[i]) <= 420)
    {
      w[i] = w[i] * conv_factor[2];
    }
    if (abs(w[i]) > 420 && abs(w[i]) < 480)
    {
      w[i] = w[i] * conv_factor[3];
    }
    if (abs(w[i]) > 255)
    {
      if (w[i] > 0)
      {
        w[i] = 255;
      }
      else
      {
        w[i] = -255;
      }
    }
  }
}

void cal_error()
{
  float temp = 0;
  temp = (sensor_data[0] - set_position) - (sensor_data[1] - set_position);
  error[0] = temp / 2;
  temp = (sensor_data[2] - set_position) - (sensor_data[3] - set_position);
  error[1] = temp / 2;
  if (abs(dir) == 1 || abs(dir) == 0)
  {
    temp = (sensor_data[0] - set_position) + (sensor_data[1] - set_position);
    error[2] = temp / 2;
  }
  if (abs(dir) == 2)
  {
    temp = (sensor_data[2] - set_position) + (sensor_data[3] - set_position);
    error[2] = temp / 2;
  }
  Serial.print("linear X Error: " + String(error[0]) + " ");
  Serial.print("linear Y Error: " + String(error[1]) + " ");
  Serial.print("Angular Error: " + String(error[2]) + " ");
}


void cal_PID()
{
  for (int i = 0; i < 3; i++)
  {
    P[i] = error[i];
    I[i] = I[i] + error[i];
    D[i] = error[i] - last_error[i];

    correction[i] = (Kp[i] * P[i]) + (Kd[i] * D[i]) + (Ki[i] * I[i]);
    last_error[i] = error[i];
    if (correction[i] > max_speed)
      correction[i] = max_speed;
  }
}

void set_Vsp()
{

  switch (abs(dir))
  {
    case 1:
      if (dir > 0)
      {
        Vsp[0] = base_speed;
        Vsp[1] = -1 * correction[0];
        Vsp[2] = -1 * correction[2];
      }
      else
      {
        Vsp[0] = -1 * base_speed;
        Vsp[1] = -1 * correction[0];
        Vsp[2] = -1 * correction[2];
      }
      break;
    case 2:
      if (dir > 0)
      {
        Vsp[0] = correction[1];
        Vsp[1] =  base_speed;
        Vsp[2] = -1 * correction[2];
      }
      else
      {
        Vsp[0] = correction[1];
        Vsp[1] = -1 * base_speed;
        Vsp[2] = -1 * correction[2];
      }
      break;
    default:
      Vsp[0] = 0;
      Vsp[1] = 0;
      Vsp[2] = 0;
  }
  if (dir == 0)
  {
    base_speed = 0;
    Vsp[0] = correction[1];
    Vsp[1] = -1 * correction[0];
    Vsp[2] = -1 * correction[2];
  }
  else {
    base_speed = follow_speed;
  }
}

void cal_wheel_vel()
{
  cal_error();
  cal_PID();
  set_Vsp();
  matrix_mult();
}


