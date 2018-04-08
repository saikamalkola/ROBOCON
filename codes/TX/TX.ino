int ir_pin = 4;
int led_pin = 10;
int debug = 1;
int start_bit = 2400;
int bin_1 = 1200;                         
int bin_0 = 600;                            
int dataOut = 0;
int guardTime = 300;


void setup() {
  //pinMode(led_pin, OUTPUT);             //This shows when we're ready to recieve
  pinMode(ir_pin, OUTPUT);
  //digitalWrite(led_pin, LOW);       //not ready yet
  digitalWrite(ir_pin, LOW);        //not ready yet
  Serial.begin(9600);
}

void loop()
{
sendIRKey(5);
delay(2000);
sendIRKey(7);
delay(2000);
}

int sendIRKey(int dataOut) {
  int data[3];
  //digitalWrite(led_pin, HIGH);     //Ok, i'm ready to send
  for (int i = 0; i < 3; i++) {
    data[i] = dataOut >> i & B1; //encode data as '1' or '0'
  }
  oscillationWrite(ir_pin, start_bit);
  digitalWrite(ir_pin, HIGH);
  delayMicroseconds(guardTime);

  for (int i = 2; i >= 0; i--) {
    if (data[i] == 0)
    {
      oscillationWrite(ir_pin, bin_0);
    }
    else
    {
      oscillationWrite(ir_pin, bin_1);
    }
    digitalWrite(ir_pin, HIGH);
    delayMicroseconds(guardTime);
  }
  delay(20);
  return dataOut;
}

void oscillationWrite(int pin, int time) {
  for (int i = 0; i <= time / 26; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(13);
    digitalWrite(pin, LOW);
    delayMicroseconds(13);
  }
}
