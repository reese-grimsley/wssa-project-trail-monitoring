#include <Arduino.h>
//#include "Serial.h"
//Need this in boards:
//https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
//
//    REG_TC4_READREQ |= 0; //example for writing/reading directly from registers REG_$peripheral-name$_$register-name$

#if defined(ARDUINO_SAMD_ZERO)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define trigger 6
#define echo 12

#define FUDGE_FACTOR 5
#define DISTANCE_TRIGGER_THRESHOLD -200
#define DISTANCE_VARIATION_MAX 100

int32_t duration, cm, inches, d_delta, d_new, d_old = 0;
int32_t person_counter = 0;

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
  Serial.println("Serial started");
  Serial.println(DISTANCE_TRIGGER_THRESHOLD);

  pinMode(13, OUTPUT);

  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  d_old = 0;
}

uint32_t readDistanceUS() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(9);
  digitalWrite(trigger, LOW);

//  pinMode(echo, INPUT);
  return pulseIn(echo, HIGH);
}

float convertInches(uint32_t us_duration) {
    return (us_duration/2) / 74; //convert to inches
}

float convertCM(uint32_t us_duration) {
    return (us_duration/2) / 29.1; //convert to centimeters
}

void loop() {

	while(1) {

		blink(1500);
	}

  Serial.println("LOOP BEGIN");

  duration = readDistanceUS();
  d_new = convertInches(duration);

  d_delta = d_new - d_old;

//  Serial.printf("Outer loop - Distance read: %d\t Delta d: %d\r\n", d_new, d_delta);

  if (d_delta < DISTANCE_TRIGGER_THRESHOLD)  { //distance has dropped by a substantial ammount
    //something has entered space of interest, stop for a moment
    static uint16_t loop_count = 0;
    do {
      loop_count++;
      //delay to enforce ~10Hz sample rate in high activity mode
      uint32_t delayVal = int(duration/1000) - FUDGE_FACTOR; //convert to ms leftover to delay
      delay(100-delayVal);

      //get a new reading
      d_old = d_new;
      duration = readDistanceUS();
      d_new = convertInches(duration);

      //compute difference
      d_delta = d_new - d_old;


//      Serial.printf("Inner loop - Distance read: %d\t Delta d: %d\r\n", d_new, d_delta);

    } while (d_delta < DISTANCE_VARIATION_MAX); //consider differing speeds of people; walking, running, stopped
//    Serial.printf("New passerby counter; stayed in loop for %d samples\r\n", loop_count);
    person_counter++;


  }
  else {
    //anything to do otherwise?
    Serial.println("Didn't take loop");
  }

  d_old = d_new;

//  Serial.printf("People counted: %d\r\n", person_counter);

  delay(1000); //

}
