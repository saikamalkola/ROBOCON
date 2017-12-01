const byte Serial1En1 = 32;    // Connect UART output enable of LSA08 to pin 2
const byte Serial1En2 = 36;    // Connect UART output enable of LSA08 to pin 2


void setup() {
  pinMode(Serial1En1, OUTPUT);  // Setting Serial1En as digital output pin
  pinMode(Serial1En2, OUTPUT);  // Setting Serial1En as digital output pin


  // Setting initial condition of Serial1En pin to HIGH



  // Begin Serial1 communication with baudrate 9600
  Serial.begin(115200);
  Serial1.begin(9600);

}


void loop() {
  double present = micros();
  digitalWrite(Serial1En1, LOW);  // Set Serial1EN to LOW to request UART data
  while (Serial1.available() <= 0);  // Wait for data to be available
  int positionVal = Serial1.read();    // Read incoming data and store in variable positionVal
  digitalWrite(Serial1En1, HIGH);   // Stop requesting for UART data

  Serial.print(String(positionVal) + " ");
  digitalWrite(Serial1En2, LOW);  // Set Serial1EN to LOW to request UART data
  while (Serial1.available() <= 0);  // Wait for data to be available
  positionVal = Serial1.read();    // Read incoming data and store in variable positionVal
  digitalWrite(Serial1En2, HIGH);   // Stop requesting for UART data

  Serial.print(String(positionVal) + " ");
  Serial.println(String(micros() - present));
}
