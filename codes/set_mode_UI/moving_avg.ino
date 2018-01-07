//
//void init_buff() {
//    for (int i = 0; i < N; i++)
//    {
//      readings[i] = 0;
//    }
//}
//
//void cal_avg() {
//  // subtract the last reading:
//  summation = summation - readings[readIndex];
//  // read from the sensor:
//  readings[readIndex] = velocity[1];
//  // add the reading to the total:
//  summation = summation + readings[readIndex];
//  // advance to the next position in the array:
//  readIndex= readIndex + 1;
//
//  // if we're at the end of the array...
//  if (readIndex>= N) {
//    readIndex = 0;
//  }
//  average = summation / N;
//
//}


void init_buff() {
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < N; j++)
      readings[i][j] = 0;
  }
}

void cal_avg() {
  // subtract the last reading:
  for(int i = 0; i < 4; i++)
  {
  summation[i] = summation[i] - readings[i][readIndex[i]];
  // read from the sensor:
  readings[i][readIndex[i]] = velocity[i];
  // add the reading to the total:
  summation[i] = summation[i] + readings[i][readIndex[i]];
  // advance to the next position in the array:
  readIndex[i] = readIndex[i] + 1;

  // if we're at the end of the array...
  if (readIndex[i] >= N) {
    readIndex[i] = 0;
  }
  average[i] = summation[i] / N;
  }
}
