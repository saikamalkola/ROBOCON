#include <Servo.h>
Servo myservo1,myservo2,myservo3;  // create servo object to control a servo
              // a maximum of eight servo objects can be created
int pos = 0;    // variable to store the servo position
void setup() {
  myservo1.attach(3); 
}
void loop() {
 myservo1.write(160);
//
// delay(2000);
// myservo1.write(120);
//   delay(2000);
}
