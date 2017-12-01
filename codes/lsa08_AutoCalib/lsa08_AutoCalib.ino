/*
  C 0x43       Calibration             0x00
  L 0x4C       Mode (Dark On/Light On) 0x01(Dark on), 0x00 (light on)
  T 0x54       Line Threshold          0x00 - 0x07
  O 0x4F       Offset                  0x00 - 0x32
  J 0x4A       Junction Width          0x01 - 0x08
  A 0x41       UART Address            0x00 - 0xFF
  B 0x42       LCD Backlight           0x00 - 0x0A
  S 0x53       LCD Contrast            0x00 - 0xFF
  R 0x52       UART Baudrate           0x00 - 0x05
  D 0x44       UART Data Output Mode   0x00 - 0x03
*/
#define DELAY 50

char address = 0x00;
uint8_t ser_enable[4] = {32, 34, 36, 38};
//Sensor Data
int sensor_data[4] = {0, 0, 0, 0};
/*
  0 - Front
  1 - Left
  2 - Back
  3 - Right

  All Sensors on Serial1
*/
char sensor_addr[4] = {0x01, 0x02, 0x03, 0x04};

//Sensor Settings
char contrast = 90; //0 - 255
char brightness = 0x04; //0 to 10
char junction_width = 0x08; //0x02 to 0x08
char threshold = 0x04; //0 to 7
char line_mode = 0x00; //Light on Dark Background
char UART_Mode = 0x02; //byte of analog Value

void setup() {
  // put your setup code here, to run once:
  init_sensors();
  Serial.begin(115200); //For debugging on Serial Monitor
  Serial.flush();
  for (int i = 0; i < 4; i++)
  {
    address = sensor_addr[i];
    set_sensor_settings();
  }
  calibrate();
}

void loop() {
  // put your main code here, to run repeatedly:
  double present = micros();
read_sensors();
 Serial.println(String(micros() - present));
}

void read_sensors()
{
  for (int i = 0; i < 3; i = i + 2)
  {
    digitalWrite(ser_enable[i], LOW);  // Set Serial1EN to LOW to request UART data
    while (Serial1.available() <= 0);  // Wait for data to be available
    sensor_data[i] = Serial1.read();    // Read incoming data and store in variable positionVal
    digitalWrite(ser_enable[i], HIGH);   // Stop requesting for UART data
  }
  for(int i = 0; i < 4; i++)
  {
    Serial.print(String(sensor_data[i]) + " ");
  }
 // Serial.println("");
}

void calibrate()
{
  for (int i = 0; i < 4; i++)
  {
    address = sensor_addr[i];
    send_command('C', 0x00);
    delay(500);
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
  send_command('T', threshold);
  delay(DELAY);
  // Setting line mode
  send_command('L', line_mode);
  delay(DELAY);
  // Setting UART ouput Mode
  send_command('D', UART_Mode);
  delay(DELAY);
}
void init_sensors()
{
  Serial1.begin(9600); // Start Serial1 communication
  Serial1.flush();   // Clear Serial1 buffer
  for (int i = 0; i < 4; i++)
  {
    pinMode(ser_enable[i], OUTPUT);
    digitalWrite(ser_enable[i], HIGH);
  }
}

void send_command(char command, char data) {

  char checksum = address + command + data;

  Serial1.write(address);
  Serial1.write(command);
  Serial1.write(data);
  Serial1.write(checksum);

}
