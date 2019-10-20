#if defined(ARDUINO_SAMD_ZERO)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif




void blink(uint32_t ms) {
  digitalWrite(13, HIGH);
  delay(ms);
  digitalWrite(13, LOW);
  delay(ms);
}

void setup() {
  // Setup Serial port
  while (!Serial) blink(250);
  Serial.begin(9600);
  Serial.println("test");


  pinMode(13, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  blink(1000);
  Serial.println("loop");

}
