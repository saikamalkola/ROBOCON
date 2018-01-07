
void telemetry() {
  float fl_pid = velocity[0];
  float fr_pid = velocity[1];
  float bl_pid = velocity[2];
  float br_pid = velocity[3];
  float fl_sp = wheel_set_point[0];
  float fr_sp = wheel_set_point[1];
  float bl_sp = wheel_set_point[2];
  float br_sp = wheel_set_point[3];

  char fl_pid_text[30];
  char fr_pid_text[30];
  char bl_pid_text[30];
  char br_pid_text[30];
  char fl_sp_text[30];
  char fr_sp_text[30];
  char bl_sp_text[30];
  char br_sp_text[30];

  dtostrf(fl_pid, 10, 10, fl_pid_text);
  dtostrf(fr_pid, 10, 10, fr_pid_text);
  dtostrf(bl_pid, 10, 10, bl_pid_text);
  dtostrf(br_pid, 10, 10, br_pid_text);
  dtostrf(fl_sp, 10, 10, fl_sp_text);
  dtostrf(fr_sp, 10, 10, fr_sp_text);
  dtostrf(bl_sp, 10, 10, bl_sp_text);
  dtostrf(br_sp, 10, 10, br_sp_text);

  char text[248];
  snprintf(text, 248, "%s,%s,%s,%s,%s,%s,%s,%s", fl_pid_text, fr_pid_text, bl_pid_text, br_pid_text, fl_sp_text, fr_sp_text, bl_sp_text, br_sp_text);
  Serial.println(text);
}

