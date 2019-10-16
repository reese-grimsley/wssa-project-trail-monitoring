#if defined(ARDUINO_SAMD_ZERO)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define trigger 6
#define echo 12

long duration, cm, inches;

void blink(uint32_t ms) {
  digitalWrite(13, HIGH);
  delay(ms);
  digitalWrite(13, LOW);
  delay(ms);
}

void setup() {
  // Setup Serial port
  while (!Serial) blink(500);
  Serial.begin(9600);
  Serial.println("test");
  
  pinMode(13, OUTPUT);

  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH);

  cm = (duration/2) / 29.1;
  inches = (duration/2) / 74;

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  delay(250);
}
