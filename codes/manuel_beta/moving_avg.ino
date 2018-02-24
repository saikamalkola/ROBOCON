void init_buff() {
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < N; j++)
    {
      readings[i][j] = 0;
      w_readings[i][j] = 0;
      ardW_readings[i][j] = 0;
    }
  }
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < N; j++)
    {
      Vsp_readings[i][j] = 0;
    }
  }
}

void cal_avg() {
  // subtract the last reading:
  for (int i = 0; i < 3; i++)
  {
    summation[i] = summation[i] - readings[i][readIndex[i]];
    w_summation[i] = w_summation[i] - w_readings[i][w_readIndex[i]];
    // read from the sensor:
    w_readings[i][w_readIndex[i]] = w[i];
    readings[i][readIndex[i]] = velocity[i];
    // add the reading to the total:
    summation[i] = summation[i] + readings[i][readIndex[i]];
    w_summation[i] = w_summation[i] + w_readings[i][w_readIndex[i]];
    // advance to the next position in the array:
    w_readIndex[i] = w_readIndex[i] + 1;
    readIndex[i] = readIndex[i] + 1;

    // if we're at the end of the array...
    if (readIndex[i] >= N) {
      readIndex[i] = 0;
    }
    if (w_readIndex[i] >= w_N) {
      w_readIndex[i] = 0;
    }
    average[i] = summation[i] / N;
    w_average[i] = w_summation[i] / w_N;
  }
}


void cal_ardW_avg() {
  // subtract the last reading:
  for (int i = 0; i < 3; i++)
  {
    ardW_summation[i] = ardW_summation[i] - ardW_readings[i][ardW_readIndex[i]];
    ardW_readings[i][ardW_readIndex[i]] = w[i];
    ardW_summation[i] = ardW_summation[i] + ardW_readings[i][ardW_readIndex[i]];
    ardW_readIndex[i] = ardW_readIndex[i] + 1;
    if (ardW_readIndex[i] >= ardW_N) {
      ardW_readIndex[i] = 0;
    }
    ardW_average[i] = ardW_summation[i] / ardW_N;
  }
}


void cal_Vsp_average()
{
  for (int i = 0; i < 3; i++)
  {
    Vsp_summation[i] = Vsp_summation[i] - Vsp_readings[i][Vsp_readIndex[i]];

    Vsp_readings[i][Vsp_readIndex[i]] = Vsp[i];

    Vsp_summation[i] = Vsp_summation[i] + Vsp_readings[i][Vsp_readIndex[i]];

    Vsp_readIndex[i] = Vsp_readIndex[i] + 1;

    if (Vsp_readIndex[i] >= Vsp_N) {
      Vsp_readIndex[i] = 0;
    }
    Vsp_average[i] = Vsp_summation[i] / Vsp_N;
  }
}

