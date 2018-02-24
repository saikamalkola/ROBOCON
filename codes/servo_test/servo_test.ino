#include <Servo.h>
Servo myservo1,myservo2,myservo3;  // create servo object to control a servo
              // a maximum of eight servo objects can be created
int pos = 0;    // variable to store the servo position
void setup() {
  myservo1.attach(8); 
 myservo2.attach(9);  // attaches the servo on pin 9 to the servo object
 myservo3.attach(10); 
}
void loop() {
 myservo1.write(0);
  myservo2.write(0);
   myservo3.write(0);
 delay(1000);
 myservo1.write(180);
  myservo2.write(180);
   myservo3.write(180);
   delay(1000);
}
