#define PIN_LED 7
int i = 0;
void setup() {
  pinMode(PIN_LED, OUTPUT);

}

void loop() {
  digitalWrite(PIN_LED, 0);
  delay(1000);
  while(i<5){
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
    i = i+1;
  }
  digitalWrite(PIN_LED, 1);
  while(1) {
  }

}
