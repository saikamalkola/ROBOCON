#define DELAY 50

//Sensor Pins
uint8_t ser_enable[4] = {22, 30, 38, 46};   //F B L R
uint8_t Jpulse[4] = {24, 32, 40, 48};       //F B L R

//Sensor Data
char address = 0x00;
char sensor_addr[4] = {0x01, 0x02, 0x03, 0x04};
int sensor_val[4] = {0, 0, 0, 0};
//boolean jun_data[4] = {0, 0, 0, 0};

//Sensor Settings
char contrast = 100; //0 - 255
char brightness = 0x04; //0 to 10
char junction_width = 0x08; //0x02 to 0x08
char threshold = 0x04; //0 to 7
char line_mode = 0x00; //Light on Dark Background
char UART_Mode = 0x02; //byte of analog Value

void init_sensors(){
  Serial3.begin(9600); // Start Serial3 communication
  Serial3.flush();   // Clear Serial3 buffer
  for (int i = 0; i < 4; i++){
    // pinMode(Jpulse[i], INPUT);
    pinMode(ser_enable[i], OUTPUT);
    digitalWrite(ser_enable[i], HIGH);
  }
  delay(DELAY);
  for (int i = 0; i < 4; i++){
    address = sensor_addr[i];
    set_sensor_settings();
  }
}

void set_sensor_settings(){
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

void calibrate(){
  for (int i = 0; i < 4; i++){
    address = sensor_addr[i];
    send_command('C', 0x00);
    delay(DELAY);
  }
}

void read_sensors(int sensor_data[4], boolean junction_data[4]){
  int temp = 0;
  for (int i = 0; i < 4; i++){
    digitalWrite(ser_enable[i], LOW);  // Set Serial3EN to LOW to request UART data
    while (Serial3.available() <= 0);  // Wait for data to be available
    temp = Serial3.read();
    if (temp <= 70)
      sensor_val[i] = temp;
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }
  for (int i = 0; i < 4; i++)  {
    sensor_data[i] = sensor_val[i];
    junction_data[i] = digitalRead(Jpulse[i]);
  }
  
}

void send_command(char command, char data) {

  char checksum = address + command + data;

  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(data);
  Serial3.write(checksum);

}

