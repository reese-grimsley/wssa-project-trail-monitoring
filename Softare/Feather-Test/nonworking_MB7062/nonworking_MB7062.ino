#if defined(ARDUINO_SAMD_ZERO)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#include <SoftwareSerial.h>

//#define Serial SerialUSB

void setupMB7062() {
  Serial.println("Setup Ultrasonic sensor");
   
  while (!Serial1);
  Serial1.begin(9600); // Serial1 used for MB76XX ultrasonic sensor
  while (Serial1.available() > 0) {Serial1.read();}
  Serial1.setTimeout(50);

  Serial.println("Ultrasonic sensor setup");
}

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
// No instance of Serial USB? that makes things harder

  setupMB7062();

  pinMode(13, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:

//  blink(1000);
//  Serial.println("loop");

  //read from sensor
  Serial1.readStringUntil('R');
  String data_str = Serial1.readStringUntil('\r');
  if (data_str.length()  > 0) {
    Serial.println(data_str);
    uint32_t sample = 0;
    uint32_t alt_sample = data_str.substring(1).toInt();
    
    for (int i = 0; i < data_str.length(); i++) {
    //    sample += pow(10, i) * data_str.charAt(i).toInt();
      Serial.print((int) data_str.charAt(i)); Serial.print(" ");
    }
    
    Serial.print("Ultrasonic distance in cm: ");
    Serial.println(sample);
  }



}
