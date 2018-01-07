// this code is a crude template
// you will need to edit this

void setup() {
  Serial.begin(115200);
}

// or use this loop if sending floats
void loop() {
  float p = random(300);
  float i = random(300);
  float d = random(300);

  char p_text[30];
  char i_text[30];
  char d_text[30];

  dtostrf(p, 10, 10, p_text);
  dtostrf(i, 10, 10, i_text);
  dtostrf(d, 10, 10, d_text);

  char text[93];
  snprintf(text, 93, "%s,%s,%s", p_text, i_text, d_text);
  Serial.println(text);

  delay(10);
}

