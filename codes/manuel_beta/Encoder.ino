//void init_encoders() {
//  for (int i = 0; i < 3; i++)
//  {
//    pinMode (outputA[i], INPUT_PULLUP);
//    pinMode (outputB[i], INPUT_PULLUP);
//    attachPCINT(digitalPinToPCINT(outputA[i]), encode, CHANGE);
//    prev_state[i] = digitalRead(outputA[i]);
//  }
//}
//
//void get_velocity() {
// // Serial.print("Actual Speed: ");
//  delta_t = millis() - previous_ms;
//  previous_ms = millis();
//  for (int i = 0; i < 3; i++)
//  {
//    velocity[i] = counter[i] * 1000 / (delta_t * 270); // rps
//    velocity[i] = velocity[i] * 60; // rpm
//  counter[i] = 0;
//  }
//}
//
//void encode()
//{
//  for (int i = 0; i < 3; i++)
//  {
//    present_state[i] = digitalRead(outputA[i]); // Reads the "current" state of the outputA
//    // If the previous and the current state of the outputA are different, that means a Pulse has occured
//    if (present_state[i] != prev_state[i])
//    {
//      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//      if (digitalRead(outputB[i]) != present_state[i]) {
//        counter[i] ++;
//      } else {
//        counter[i] --;
//      }
//    }
//    prev_state[i] = present_state[i]; // Updates the previous state of the outputA with the current state
//  }
//}
