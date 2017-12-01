char address = 0x02;    // UART address as 0x01

void setup() {
  Serial1.begin(9600);   // Start Serial1 communication
  Serial1.flush();   // Clear Serial1 buffer

}

void loop() {

  char command, data;

  // Clear internal junction count of LSA08
  command = 'X';
  data = 0x00;
  sendCommand(command,data);
delay(1000);
  // Setting LCD contrast to 90
  command = 'S';
  data = 90;
  sendCommand(command,data);
delay(1000);
  // Setting LCD backlight to level 5
  command = 'B';
  data = 0x04;
  sendCommand(command,data);
delay(1000);
  // Setting junction width to 6
  command = 'J';
  data = 0x08;
  sendCommand(command,data);
delay(1000);
  // Setting threshold value to 5
  command = 'T';
  data = 0x04;
  sendCommand(command,data);
delay(1000);
  // Setting line mode to Dark-On
  command = 'L';
  data = 0x00;
  sendCommand(command,data);
delay(1000);
  // Setting UART ouput to mode 1
  command = 'D';
  data = 0x02;
  sendCommand(command,data);
delay(1000);
  // Start calibration
  command = 'C';
  data = 0x00;
  sendCommand(command,data);

address = 0x01;



  // Clear internal junction count of LSA08
  command = 'X';
  data = 0x00;
  sendCommand(command,data);
delay(1000);
  // Setting LCD contrast to 90
  command = 'S';
  data = 90;
  sendCommand(command,data);
delay(1000);
  // Setting LCD backlight to level 5
  command = 'B';
  data = 0x04;
  sendCommand(command,data);
delay(1000);
  // Setting junction width to 6
  command = 'J';
  data = 0x08;
  sendCommand(command,data);
delay(1000);
  // Setting threshold value to 5
  command = 'T';
  data = 0x04;
  sendCommand(command,data);
delay(1000);
  // Setting line mode to Dark-On
  command = 'L';
  data = 0x00;
  sendCommand(command,data);
delay(1000);
  // Setting UART ouput to mode 1
  command = 'D';
  data = 0x02;
  sendCommand(command,data);
delay(1000);
  // Start calibration
  command = 'C';
  data = 0x00;
  sendCommand(command,data);
  while(1);   // Stay here to prevent infinite loop

}

/*
 * Function to send command to LSA08 in 4 continuous bytes.
 * LSA08 will reply with "OK" for every successful command sent.
 * However, reading the reply is optional, and thus not showing here.
 */
void sendCommand(char command, char data) {
  
  char checksum = address + command + data;
  
  Serial1.write(address);
  Serial1.write(command);
  Serial1.write(data);
  Serial1.write(checksum);

}
