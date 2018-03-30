//
//  void telemetry() {
// // float fl_pid = wheel_I[1];
//  float fl_pid = velocity[0];
//  float fr_pid = velocity[1];
//  float bl_pid = velocity[2];
//  float br_pid = velocity[3];
//  float fl_sp = wheel_set_point[0];
//  float fr_sp = wheel_set_point[1];
//  float bl_sp = wheel_set_point[2];
//  float br_sp = wheel_set_point[3];
//
//  char fl_pid_text[30];
//  char fr_pid_text[30];
//  char bl_pid_text[30];
//  char br_pid_text[30];
//  char fl_sp_text[30];
//  char fr_sp_text[30];
//  char bl_sp_text[30];
//  char br_sp_text[30];
//
//  dtostrf(fl_pid, 10, 10, fl_pid_text);
//  dtostrf(fr_pid, 10, 10, fr_pid_text);
//  dtostrf(bl_pid, 10, 10, bl_pid_text);
//  dtostrf(br_pid, 10, 10, br_pid_text);
//  dtostrf(fl_sp, 10, 10, fl_sp_text);
//  dtostrf(fr_sp, 10, 10, fr_sp_text);
//  dtostrf(bl_sp, 10, 10, bl_sp_text);
//  dtostrf(br_sp, 10, 10, br_sp_text);
//
//  char text[248];
//  snprintf(text, 248, "%s,%s,%s,%s,%s,%s,%s,%s", fl_pid_text, fr_pid_text, bl_pid_text, br_pid_text, fl_sp_text, fr_sp_text, bl_sp_text, br_sp_text);
//  Serial.println(text);
//  }
//

//
//void telemetry() {
//  float vx_pid = PID[0];
//  float vy_pid = base_speed;
//  float w_pid = PID[1];
//  float vx_sp = 0;
//  float vy_sp = 0;
//  float w_sp = 0;
//
//  switch (abs(dir))
//  {
//    case 1:
//      if (dir > 0)
//      {
//        vx_sp = 0;
//        vy_sp = base_speed;
//        w_sp = 0;
//      }
//      else
//      {
//        vx_sp = 0;
//        vy_sp = -1 * base_speed;
//        w_sp = 0;
//      }
//      break;
//    default:
//      Vsp[0] = 0;
//      Vsp[1] = 0;
//      Vsp[2] = 0;
//  }
//
//  char vx_pid_text[30];
//  char vy_pid_text[30];
//  char w_pid_text[30];
//  char vx_sp_text[30];
//  char vy_sp_text[30];
//  char w_sp_text[30];
//
//  dtostrf(vx_pid, 10, 10, vx_pid_text);
//  dtostrf(vy_pid, 10, 10, vy_pid_text);
//  dtostrf(w_pid, 10, 10, w_pid_text);
//  dtostrf(vx_sp, 10, 10, vx_sp_text);
//  dtostrf(vy_sp, 10, 10, vy_sp_text);
//  dtostrf(w_sp, 10, 10, w_sp_text);
//
//  char text[186];
//  snprintf(text, 186, "%s,%s,%s,%s,%s,%s", vx_pid_text, vy_pid_text, w_pid_text, vx_sp_text, vy_sp_text, w_sp_text);
//  Serial.println(text);
//}
//


