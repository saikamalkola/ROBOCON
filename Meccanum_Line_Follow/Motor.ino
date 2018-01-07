/*
  Pin mappings to motors
  0 - front left motor
  1 - front right motor
  2 - rear left motor
  3 - rear right motor
*/

//Motor Pins
uint8_t motor_pin[4] = {6, 3, 12, 9};
uint8_t dir_pin[4] = {5, 2, 11, 8};
uint8_t brake_pin[4] = {7, 4, 13, 10};

void init_motors(){
  for (int i = 0; i < 4; i++){
    pinMode(motor_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT);
    pinMode(brake_pin[i], OUTPUT);
    digitalWrite(brake_pin[i], LOW);
  }
  for (int i = 0; i < 4; i++)
    set_motor(i, 0);
}

void motors(float motor_speed[4]){
//  Serial.println();
  for (int i = 0; i < 4; i++){
    set_motor(i, motor_speed[i]);
//    Serial.print(String(motor_speed[i])+ " ");
  }
//  Serial.println();
}

void set_motor(uint8_t index, int motor_speed){
  if (index % 2 == 0)
    motor_speed = -1 * motor_speed;
  if (abs(motor_speed) < 5)
    motor_speed = 0;
  if (motor_speed >= 0){
    //clock wise direction
    digitalWrite(brake_pin[index], LOW);
    digitalWrite(dir_pin[index], LOW);
    analogWrite(motor_pin[index], motor_speed);
  }
  else{
    //anti clock wise direction
    digitalWrite(brake_pin[index], LOW);
    digitalWrite(dir_pin[index], HIGH);
    analogWrite(motor_pin[index], -1 * motor_speed);
  }
}
