/*
 * Meccanum System
 * 
 */
#define max_speed 0.6
#define min_speed 0
#define pi 3.1416
#define follow_speed 0.55 //0.6 //in m/s
//Vmax = 3.725

float base_speed = 0;
float max_div = 255.0, max_rpm = 468.0;
float conv_factor = max_div / max_rpm;

float J[4][3] = {{13.1579, -13.1579, -8.4211}, {13.1579, 13.1579, 8.4211}, {13.1579, 13.1579, -8.4211}, {13.1579, -13.1579, 8.4211}};

/*
  Conversion Matrix
  13.1579 -13.1579 -8.4211
  13.1579  13.1579  8.4211
  13.1579  13.1579 -8.4211
  13.1579 -13.1579  8.4211
*/

float Vx = 0, Vy = 0, W = 0;

float w[4] = {0, 0, 0, 0};
/*
  0 - FL
  1 - FR
  2 - BL
  3 - BR
*/

//PID Variables
int dir = 1;
/*
  0   - Stop
  1   - Front
  -1  - Back
  2   - Left
  -2  - Right
*/
float Kp[3] = {0.02, 0.02 , 0.028} , Kd[3] = {0.03, 0.03 , 0.03}, Ki[3] = {0, 0};                     // Kp[3] = {0.045, 0.045 , 0.047}, Kd[3] = {0.045, 0.045 , 0.045}
float Kp_junc[3] = {0.0235, 0.0235 , 0.0255} , Kd_junc[3] = {0.03, 0.03 , 0.03}, Ki_junc[3] = {0, 0}; // Kp_junc[3] = {0.035, 0.035 , 0.045}, Kd_junc[3] = {0.05, 0.05 , 0.05}
float P[3] = {0, 0, 0}, I[3] = {0, 0, 0}, D[3] = {0, 0, 0};
float disp_X_error = 0, disp_Y_error = 0, W_error = 0;
float disp_X_correction = 0, disp_Y_correction = 0, W_correction = 0;
float last_error[3] = {0, 0, 0};
float set_position = 35;

void matrix_mult(){
  float sum = 0;
  float Vsp[4] = {Vx, Vy, W};
  for (int i = 0; i < 4; i++){
    sum  = 0;
    for (int j = 0; j < 3; j++)
      sum += (J[i][j] * Vsp[j]);
    w[i] = sum;
    w[i] = w[i] * 60 / (2 * pi); //rps to RpM
    w[i] = w[i] * conv_factor; //RpM to Arduino PWM value*/
    if (abs(w[i]) > 255)
      if (w[i] > 0)
        w[i] = 255;
      else
        w[i] = -255;
    //Serial.print(String(w[i]) + " ");
  }
  // Serial.println("");
}


/*
 * PID Controller
 */

void cal_error(int sensor_data[4]){
  float temp = 0;
  temp = (sensor_data[0] - set_position) - (sensor_data[1] - set_position);
  disp_X_error = temp / 2;
  temp = (sensor_data[2] - set_position) - (sensor_data[3] - set_position);
  disp_Y_error = temp / 2;
  temp = (sensor_data[0] - set_position) + (sensor_data[1] - set_position);
  
  if (abs(dir) == 1 || abs(dir) == 0){
    temp = (sensor_data[0] - set_position) + (sensor_data[1] - set_position);
    W_error = temp / 2;
  }
  if (abs(dir) == 2){
    temp = (sensor_data[2] - set_position) + (sensor_data[3] - set_position);
    W_error = temp / 2;
  }
//  Serial.print("Displacement X Error: " + String(disp_X_error) + " ");
//  Serial.print("Displacement Y Error: " + String(disp_Y_error) + " ");
//  Serial.print("Angular Error: " + String(W_error) + " ");
}

void cal_PID(){
  float error[3] = {disp_X_error, disp_Y_error, W_error};
  float correction[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++){
//    if(error[i] < error_threshold){
//      error[i] = 0;
//      continue;
//    }
    P[i] = error[i];
    I[i] = I[i] + error[i];
    D[i] = error[i] - last_error[i];
    if(dir == 0)
      correction[i] = (Kp_junc[i] * P[i]) + (Kd_junc[i] * D[i]) + (Ki_junc[i] * I[i]);
    else
      correction[i] = (Kp[i] * P[i]) + (Kd[i] * D[i]) + (Ki[i] * I[i]);
    last_error[i] = error[i];
    if (correction[i] > max_speed)
      correction[i] = max_speed;
//    else if (correction[i] < min_speed)
//      correction[i] = 0;
//    Serial.print(String(correction[i]) + " ");
  }
  disp_Y_correction = correction[0];
  disp_X_correction = correction[1];
  W_correction = correction[2];
//  Serial.println("");
}

void set_Vsp()
{
//  Vx = -0.6;
//  Vy = 0;
//  W = 0;
//  /*
  switch (abs(dir))
  {
    case 1:
      if (dir > 0)
      {
        Vx = base_speed ;//- 0.15;
        Vy = -1 * disp_Y_correction;
        W = -1 * W_correction;
      }
      else
      {
        Vx = -1 * (base_speed);// - 0.15);
        Vy = -1 * disp_Y_correction;
        W = -1 * W_correction;
      }
      break;
    case 2:
      if (dir > 0)
      {
        Vx = disp_X_correction;
        Vy =  base_speed;
        W = -1 * W_correction;
      }
      else
      {
        Vx = disp_X_correction;
        Vy = -1 * (base_speed);// + 0.3);
        W = -1 * W_correction;
      }
      break;
    default:
      Vx = 0;
      Vy = 0;
      W = 0;
  }
  if (dir == 0)
  {
    base_speed = 0;
    Vx = disp_X_correction;
    Vy = -1 * disp_Y_correction;
    W = -1 * W_correction;
  }
  else {
    base_speed = follow_speed;
  }
//  */
}


// Dynamic Direction
void get_wheel_vel(int sensor_data[4], float vel[4], int instruction){
  dir = instruction;
  cal_error(sensor_data);
  cal_PID();
  set_Vsp();
  
  matrix_mult();
//  vel = w;
  for (int i = 0; i < 4; i++){
    vel[i] = w[i];
//    Serial.print(String(w[i])+ " ");
  }
}
