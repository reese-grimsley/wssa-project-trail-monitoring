#if defined(ARDUINO_SAMD_ZERO)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define trigger 6
#define echo 12

#define FUDGE_FACTOR 5
#define DISTANCE_TRIGGER 200
#define DISTANCE_VARIATION_MAX 100

uint32_t duration, cm, inches, d_delta, d_new, d_old = 0;
uint32_t person_counter;

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

  duration = readDistanceUS();
  d_new = convertInches(duration);

  Serial.println(inches);

  d_delta = d_new - d_old;
  if (d_delta < - DISTANCE_TRIGGER)  { //distance has dropped by a substantial ammount
    //something has entered space of interest, stop for a moment
    uint16_t loop_count = 0;
    do {
      loop_count++;
      //delay to enforce ~10Hz sample rate in high activity mode
      uint32_t delayVal = int(duration/1000) + FUDGE_FACTOR; //convert to ms leftover to delay
      delay(100-delayVal);
  
      //get a new reading
      d_old = d_new;
      duration = readDistanceUS();
      d_new = convertInches(duration);
  
      //compute difference
      d_delta = d_new - d_old;
      
    } while (d_delta < DISTANCE_VARIATION_MAX); //consider differing speeds of people; walking, running, stopped
    Serial.printf("New passerby counter; stayed in loop for %d samples", loop_count);
    person_counter++;

  }
  else {
    //anything to do otherwise?
  }

    
  delay(1000); // 
  d_old = d_new;
}
