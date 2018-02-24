const byte Serial3En1 = 38;    // Connect UART output enable of LSA08 to pin 2
const byte Serial3En2 = 46;    // Connect UART output enable of LSA08 to pin 2


void setup() {
  pinMode(Serial3En1, OUTPUT);  // Setting Serial3En as digital output pin
  pinMode(Serial3En2, OUTPUT);  // Setting Serial3En as digital output pin


  // Setting initial condition of Serial3En pin to HIGH



  // Begin Serial3 communication with baudrate 9600
  Serial.begin(115200);
  Serial3.begin(9600);

}


void loop() {
  double present = micros();
  digitalWrite(Serial3En1, LOW);  // Set Serial3EN to LOW to request UART data
  while (Serial3.available() <= 0);  // Wait for data to be available
  int positionVal = Serial3.read();    // Read incoming data and store in variable positionVal
  digitalWrite(Serial3En1, HIGH);   // Stop requesting for UART data

  Serial.print(String(positionVal) + " ");
  digitalWrite(Serial3En2, LOW);  // Set Serial3EN to LOW to request UART data
  while (Serial3.available() <= 0);  // Wait for data to be available
  positionVal = Serial3.read();    // Read incoming data and store in variable positionVal
  digitalWrite(Serial3En2, HIGH);   // Stop requesting for UART data

  Serial.print(String(positionVal) + " ");
  Serial.println(String(micros() - present));
}
