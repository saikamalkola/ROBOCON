float wheel_Kp[4] = {0.5, 0.5, 0.5, 0.5} , wheel_Kd[4] = {0.1, 0.1, 0.1, 0.1}, wheel_Ki[4] = {0};
float Kp[3] = {0.01, 0.01 , 0.014} , Kd[3] = {0.03, 0.03 , 0.03}, Ki[3] = {0, 0};

float wheel_P[4] = {0, 0, 0, 0}, wheel_I[4] = {0, 0, 0, 0}, wheel_D[4] = {0, 0, 0, 0};
float P[3] = {0, 0, 0}, I[3] = {0, 0, 0}, D[3] = {0, 0, 0};

float correction[3] = {0};
float wheel_correction[4] = {0, 0, 0, 0};

int follow_speed = 0;
int sensor_data[4] = {0, 0, 0, 0};
boolean junction_data[4] = {0, 0, 0, 0};

float velocity[4] = {0};
float w[4] = {0, 0, 0, 0};

//UI Variables
int pid_sel = 0, log_sel = 0, sel_log_pid = 0, sel_line_pid = 0, sel_wheel_pid  = 0;
String cmd = "";
String response = "";
float data = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  if (Serial1.available())
  {
    response = Serial1.readStringUntil("\n");
    parse_response();
  }
  telemetry_data();
  log_data();
  //Serial.println("plot_line_pid " + String(random(300)));
}

void telemetry_data()
{
  correction[0] = random(300);
  correction[1] = random(300);
  for(int i = 0; i < 4; i++)
  wheel_correction[i] = random(300);
    float linepid_vy = correction[0];
  float linepid_w = correction[1];
  float vy_sp = 0;
  float w_sp = 0;
  float fl_pid = wheel_correction[0];
  float fr_pid = wheel_correction[1];
  float bl_pid = wheel_correction[2];
  float br_pid = wheel_correction[3];
  float fl_sp = w[0];
  float fr_sp = w[1];
  float bl_sp = w[2];
  float br_sp = w[3];

  char linepid_vy_text[30];
  char linepid_w_text[30];
  char vy_sp_text[30];
  char w_sp_text[30];
  char fl_pid_text[30];
  char fr_pid_text[30];
  char bl_pid_text[30];
  char br_pid_text[30];
  char fl_sp_text[30];
  char fr_sp_text[30];
  char bl_sp_text[30];
  char br_sp_text[30];

  dtostrf(linepid_vy, 10, 10, linepid_vy_text);
  dtostrf(linepid_w, 10, 10, linepid_w_text);
  dtostrf(vy_sp, 10, 10, vy_sp_text);
  dtostrf(w_sp, 10, 10, w_sp_text);
  dtostrf(fl_pid, 10, 10, fl_pid_text);
  dtostrf(fr_pid, 10, 10, fr_pid_text);
  dtostrf(bl_pid, 10, 10, bl_pid_text);
  dtostrf(br_pid, 10, 10, br_pid_text);
  dtostrf(fl_sp, 10, 10, fl_sp_text);
  dtostrf(fr_sp, 10, 10, fr_sp_text);
  dtostrf(bl_sp, 10, 10, bl_sp_text);
  dtostrf(br_sp, 10, 10, br_sp_text);

  char text[372];
  snprintf(text, 372, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", linepid_vy_text, linepid_w_text, fl_pid_text, fr_pid_text, bl_pid_text, br_pid_text, vy_sp_text, w_sp_text, fl_sp_text, fr_sp_text, bl_sp_text, br_sp_text);
  Serial.println(text);

  delay(1);
}
void parse_response()
{
  int l = response.length(), k = 0;
  int limits[100];
  for (int i = 0; i < l - 1 ; i++)
  {
    if (response[i] == ' ')
    {
      limits[k] = i + 1;
      k++;
    }
  }
  String temp = response.substring(0, limits[0] - 1);
  cmd = temp;
  temp = response.substring(limits[0], limits[1] - 1);
  data = temp.toFloat();
  //Serial.println("Command :" + cmd + "\tdata : " + String(data));
  action();

}

void action()
{
  if (cmd == "pid_sel")
  {
    pid_sel = int(data);
  }
  if (cmd == "Kp")
  {
    if (pid_sel < 3)
    {
      Kp[pid_sel] = data;
    }
    else
    {
      wheel_Kp[pid_sel - 3] = data;
    }
  }
  if (cmd == "Ki")
  {
    if (pid_sel < 3)
    {
      Ki[pid_sel] = data;
    }
    else
    {
      wheel_Ki[pid_sel - 3] = data;
    }
  }
  if (cmd == "Kd")
  {
    if (pid_sel < 3)
    {
      Kd[pid_sel] = data;
    }
    else
    {
      wheel_Kd[pid_sel - 3] = data;
    }
  }
  if (cmd == "log")
  {
    log_sel = int(data);
  }
  if (cmd == "sel_log_pid")
  {
    sel_log_pid = int(data);
  }
  if (cmd == "speed")
  {
    follow_speed = (data);
  }

}

void log_data()
{
  switch (log_sel)
  {
    case 0:
      {
        print_sensor_data();
      }
      break;
    case 1:
      {
        print_motor_data();
      }
      break;
    case 2:
      {
        print_pid_data();
      }
      break;
    case 3:
      {
        start_plotting();
      }
      break;
    default:
      {
        //do nothing
      }
  }
}

void print_sensor_data()
{
  Serial1.print("SD: ");
  for (int i = 0; i < 4; i++)  {
    Serial1.print(String(sensor_data[i]) + " ");
  }
  Serial1.print(" JD: ");
  for (int i = 0; i < 4; i++)  {
    Serial1.print(String(junction_data[i]) + " ");
  }
  Serial1.println("");
}

void print_motor_data()
{
  Serial1.print("DV: ");
  for (int i = 0; i < 4; i++)  {
    Serial1.print(String(w[i]) + " ");
  }
  Serial1.print(" AV: ");
  for (int i = 0; i < 4; i++)  {
    Serial1.print(String(velocity[i]) + " ");
  }
  Serial1.println("");
}

void print_pid_data()
{
  if (sel_log_pid < 3)
  {
    Serial1.print("Kp: " + String(Kp[sel_log_pid]) + " ");
    Serial1.print("Ki: " + String(Ki[sel_log_pid]) + " ");
    Serial1.println("Kd: " + String(Kd[sel_log_pid]) + " ");
  }
  else
  {
    Serial1.print("WKp: " + String(wheel_Kp[sel_log_pid - 3]) + " ");
    Serial1.print("WKi: " + String(wheel_Ki[sel_log_pid - 3]) + " ");
    Serial1.println("WKd: " + String(wheel_Kd[sel_log_pid - 3]) + " ");
  }
}

void start_plotting()
{
  Serial1.print("plot_line_pid " + String(correction[sel_line_pid]) + "\n");
  Serial1.print("plot_wheel_pid " + String(wheel_correction[sel_wheel_pid]) + "\n");
}

