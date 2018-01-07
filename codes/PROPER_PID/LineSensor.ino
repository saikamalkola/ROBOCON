
void read_sensors()
{
  int temp = 0;
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(ser_enable[i], LOW);  // Set Serial3EN to LOW to request UART data
    while (Serial3.available() <= 0);  // Wait for data to be available
    temp = Serial3.read();
    if (temp <= 70)
      sensor_data[i] = temp;
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }
  for (int i = 0; i < 4; i++)  {
    junction_data[i] = digitalRead(Jpulse[i]);
  }

}

void calibrate()
{
  for (int i = 0; i < 4; i++)
  {
    address = sensor_addr[i];
    send_command('C', 0x00);
    delay(DELAY);
  }
}
void set_sensor_settings()
{
  // Clear internal junction count of LSA08
  send_command('X', 0x00);
  delay(DELAY);
  // Setting LCD contrast
  send_command('S', contrast);
  delay(DELAY);
  // Setting LCD backlight
  send_command('B', brightness);
  delay(DELAY);
  // Setting junction width
  send_command('J', junction_width);
  delay(DELAY);
  //send_command('T', threshold);
  //delay(DELAY);
  // Setting line mode
  send_command('L', line_mode);
  delay(DELAY);
  // Setting UART ouput Mode
  send_command('D', UART_Mode);
  delay(DELAY);
}

void init_sensors()
{
  Serial3.begin(9600); // Start Serial3 communication
  Serial3.flush();   // Clear Serial3 buffer
  for (int i = 0; i < 4; i++)
  {
    // pinMode(Jpulse[i], INPUT);
    pinMode(ser_enable[i], OUTPUT);
    digitalWrite(ser_enable[i], HIGH);
  }
  delay(DELAY);
  for (int i = 0; i < 4; i++)
  {
    address = sensor_addr[i];
    set_sensor_settings();
  }
}

void send_command(char command, char data) {

  char checksum = address + command + data;

  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(data);
  Serial3.write(checksum);
}

void reinit_sensor(){
  int junction_pull = 25; // 0 to 35
  switch(dir){
    case 1:
      sensor_data[2] = 35;//+junction_pull;//70
      sensor_data[3] = 35;//-junction_pull;//0
      break;
    case 2:
      sensor_data[0] = 35-junction_pull;
      sensor_data[1] = 35+junction_pull;
     break;
    case -2:
      sensor_data[0] = 35+junction_pull;
      sensor_data[1] = 35-junction_pull;
    break; 
    case -1:
      sensor_data[2] = 35;//-junction_pull;
      sensor_data[3] = 35;//+junction_pull;
    break;    
  }
}
