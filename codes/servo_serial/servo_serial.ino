#include <SoftwareSerial.h>
#define rxPin 2
#define txPin 3

//SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

SoftwareSerial mySerial(rxPin, txPin); // RX, Tx
void setup()
{
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Issue servo Commands");
  mySerial.begin(9600);
  delay(250);
  mySerial.println("M125");
  delay(250);
  Serial.println("Stop and reset P0?");
  mySerial.println("P0");

}
void loop() // run over and over
{
  Serial.println("Go to  G900");
  mySerial.println("G900");
  delay(2000);
    Serial.println("Go to  G900");
  mySerial.println("G-900");
  delay(2000);
}
