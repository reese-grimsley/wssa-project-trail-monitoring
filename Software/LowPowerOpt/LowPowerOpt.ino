 //Need this in boards:
//https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
//
//    REG_TC4_READREQ |= 0; //example for writing/reading directly from registers REG_$peripheral-name$_$register-name$

#if defined(ARDUINO_SAMD_ZERO)
#define Serial Serial1_PORT_USBVIRTUAL
#endif

#define DEBUG_PRINT

//pin defines
#define trigger 16
#define ECHO_RISE 10
#define ECHO_FALL 11
#define LED 13

//define thresholds for finite state machine transitions
#define FUDGE_FACTOR 5
#define DISTANCE_THRESHOLD_SLOW -25 //In inches. negative means distance became shorter between samples
#define DISTANCE_THRESHOLD_FAST -25	 
#define DISTANCE_THRESHOLD_LEAVE 25
#define FAST_COUNT_MAX 200 // If sampling fast for too long, do a state reset
#define FSM_RESET_DELAY_MS 5000

//clock and timer defines
#define CPU_HZ 48000000 //CPU runs at 48MHz
#define TIMER_TRIGGER_PRESCALE 1 // don't have to use this, but 32 is the max value before we sacrifice resolution
#define TIMER_ECHO_PRESCALE 1024
#define FAST_SAMPLE_DELAY CPU_HZ/TIMER_TRIGGER_PRESCALE/10
#define SLOW_SAMPLE_DELAY CPU_HZ/TIMER_TRIGGER_PRESCALE
#define US_THRESHOLD CPU_HZ*50/1000 //~50ms

enum State {
  SLOW, FAST_BASE, FAST_1, FAST_2
};

int32_t duration, d_delta, d_new, d_old = 0;
float inches, cm, distance, former_distance = 0;
int32_t person_counter = 0;

int32_t timer_value = 0;
uint8_t fast_sample_flag = 0;

volatile uint32_t newUS=0, oldUS=0;
volatile int32_t diffUS;

void riseISR(void) {
//  Serial1.println("r");
  
  oldUS = ((TcCount16*)TC3)->COUNT.reg;
//  Serial1.println(oldUS);

//  digitalWrite(6, HIGH);

}

void fallISR(void) {
  
//  Serial1.println("f");
  newUS = ((TcCount16*)TC3)->COUNT.reg;
//  Serial1.println(newUS);

  
//  digitalWrite(6, LOW);
}

void blink(uint32_t ms) {
  digitalWrite(LED, HIGH);
  delay(ms);
  digitalWrite(LED, LOW);
  delay(ms);
}

/*
 * Puts processor in sleep mode. Requires interrupt to get out, so make sure those are configured if you want to call this function
 */
void idle_state() {

//  Serial1.println("EnS");
  
    // Set sleep mode to deep sleep - 2 may be better.
  PM->SLEEP.reg = PM_SLEEP_IDLE_APB;   //puts proc into idle mode (once __WFI is called). Only 1 and 2 have effect.
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // mask corresponding to deep sleep enable
  //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
//  //Disable USB port (to disconnect correctly from host
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;

  SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk; //Thank god for internet forums: https://forum.arduino.cc/index.php?topic=601522.0

  //Enter sleep mode and wait for interrupt (WFI)
  Serial1.flush(); //Apparently we HAVE to have this for the thing to stay asleep... 
//  delay(1);
  __DSB(); //data sync barrier; all instructions complete before moving past instruction
  __WFI(); //Wait for interrupt (or WFE to wait for event OR interrupt); puts processor in sleep state

//  Serial1.println("ExS");
  
  SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
//  //Re-enable USB (should never get there because no wakeup interrupt is configured)
  
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
}

uint32_t readDistanceUS() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(9);
  digitalWrite(trigger, LOW);

  //  pinMode(echo, INPUT);
  return pulseIn(ECHO_FALL, HIGH);
}

float convertInches(uint32_t us_duration) {
  return ((float) us_duration / 2) / 74; //convert to inches
}

float convertCM(uint32_t us_duration) {
  return ((float) us_duration / 2) / 29.1; //convert to centimeters
}

void setupDistanceTimer() {
  Serial1.println("Configure PWM read TCC peripheral");
  REG_PM_APBCMASK |= PM_APBCMASK_TC3;
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);
  
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3; 
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; //disable timer so we can make changes
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
   
  Serial1.println("Done configuring Timer");
}

void updateSampleRate(uint32_t wait) {
  
	TcCount32* TC = (TcCount32*) TC4;
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; //disable timer so we can make changes
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->COUNT.reg=0; //reset
	TC->CC[0].reg = wait; //set compare register; should actually be the TOP value
	while (TC->STATUS.bit.SYNCBUSY == 1);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void setTriggerTimerCC(uint32_t wait) {
  Serial1.println("Set Timer valu1es");

  uint32_t compareValueWait = wait;
  //uint32_t compareValueWait = (uint32_t) CPU_HZ * 1 / TIMER_TRIGGER_PRESCALE; //set timer for 1s (old)
  uint32_t compareValueTrig = (uint32_t) CPU_HZ * 1E-5 / TIMER_TRIGGER_PRESCALE; //set a 10us pulse using a 48MHz base clock (APB)

  Serial1.println(compareValueWait);
  Serial1.println(compareValueTrig);
  
  TcCount32* TC = (TcCount32*) TC4; //TC3 otherwise; think we need TCC to use waveform output values for pin

//  REG_TC3_COUNT32_COUNT = 16;
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValueWait);
  
  TC->CC[0].reg = compareValueWait; //set compare register; should actually be the TOP value
  TC->CC[1].reg = compareValueTrig; //set compare register
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void setTriggerTimer() {
  TcCount32* TC = (TcCount32*) TC4;

  // Configure pin for peripheral. Use TC4 WO1 and port 16 for PB09, pin 8 on processor, pin 16 Arduino
  PORT->Group[g_APinDescription[16].ulPort].PINCFG[g_APinDescription[16].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[16].ulPort].PMUX[g_APinDescription[16].ulPin >> 1].reg = PORT_PMUX_PMUXO_E; //odd pin, mode E
  
  REG_PM_APBCMASK |= PM_APBCMASK_TC4;
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5);
  
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; //disable timer so we can make changes
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

   TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT32 | TC_CTRLA_WAVEGEN_MPWM | TC_CTRLA_RUNSTDBY;
//   TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT32 | TC_CTRLA_WAVEGEN_NPWM;// | TC_CTRLA_RUNSTDBY;

   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

   setTriggerTimerCC(SLOW_SAMPLE_DELAY);

   TC->CTRLA.reg |= TC_CTRLA_ENABLE;
   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

}

void delaySensing(uint32_t ms) {
  Serial1.print("\\---\r\n\\\r\nFSM RESET\r\n\\\r\n\\---\r\n");
  TcCount32* TC = (TcCount32*) TC4;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  delay(ms);
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

}


enum State state = SLOW; //initial state
enum State former_state = state; //initial state

//(Mealy) finite state machine for detecting a passerby (technically automaton). 
//	Input is the difference in distance between subsequent samples in inches
//	Output is an increment to person_counter
//	Future work may benefit from FIR or IIR digital filter rather than extra FSM states
void fsm(float delta) {
	static uint8_t fast_count; //state variable to allow reset from fast sampling
  former_state = state;
	bool z = false; //mealy state machine
	Serial1.print("S:"); 	Serial1.println(state);
  Serial1.print("Delta D: "); Serial1.println(delta);
	
	switch(state) {
		case SLOW:
      fast_count = 0;
			if (delta < DISTANCE_THRESHOLD_SLOW) {//distance dropped, start sampling faster
				state = FAST_BASE;
				z = true; //Mealy
				updateSampleRate(FAST_SAMPLE_DELAY);
			}
			
			else {
				state = SLOW; // not necessary, but operation is virtually free and helps organization
				z = false;
			}
			
			break;
		case FAST_BASE:
			fast_count++;
     
			if (fast_count >= FAST_COUNT_MAX) {
				state = SLOW;
				delaySensing(FSM_RESET_DELAY_MS);
				updateSampleRate(SLOW_SAMPLE_DELAY);
				z = false;
				
			}
			else if (delta >= DISTANCE_THRESHOLD_LEAVE)  {
				state = FAST_1;
				z = false;
				
			}
			
			else {
				state = FAST_BASE;
				z = false;
				
			}
			
			break;
			
		case FAST_1: 
			fast_count++;
			
			if (delta >= DISTANCE_THRESHOLD_FAST) {
				state = FAST_2;
			
			}				
			else {
				state = FAST_BASE;
				
			}
			
			z = false;
			
			break;
			
		case FAST_2:
			fast_count++;
				
			if (delta >= DISTANCE_THRESHOLD_FAST) {
				state = SLOW;
        updateSampleRate(SLOW_SAMPLE_DELAY);
				
			}
			else {
				state = FAST_BASE;
				
			}
			
			z = false;
			
			break;
			
		default:
      Serial1.println("UNEXPECTED ERROR: STATE");
			break;
	}

  if (state != former_state) {
	  Serial1.print("Transition to state: ");	Serial1.println(state);
  }
	//respond to output variable
	if (z) { 
		Serial1.print("C:");
		Serial1.println(++person_counter);
	}
	
	
}

void setup() {
  // Setup Serial port
#ifdef DEBUG_PRINT
  while (!Serial1) blink(500);
  Serial1.begin(230400);
  Serial1.println("Serial started");
#endif

  //config some pins
  pinMode(6, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(ECHO_RISE, INPUT);
  pinMode(ECHO_FALL, INPUT);

  Serial1.println("Shut off peripherals");
  Serial1.println(REG_PM_APBCMASK, HEX);
  REG_PM_APBCMASK &= ~(PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5); //Serial1 uses SERCOM0
  REG_PM_APBCMASK &= ~(PM_APBCMASK_TC6 | PM_APBCMASK_TC7 | PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2); 
  REG_PM_APBCMASK &= ~(PM_APBCMASK_ADC | PM_APBCMASK_DAC);
  Serial1.println(REG_PM_APBCMASK, HEX);

  Serial1.println("Attach ISRs");
  attachInterrupt(digitalPinToInterrupt(ECHO_RISE), riseISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ECHO_FALL), fallISR, FALLING);

  Serial1.println("Configure Timer");
  setTriggerTimer();
  setupDistanceTimer();

  Serial1.println("Finish setup");
}

static bool isSensorReading = false;   //sensor is taking a measurement right now or not

void loop() {

  idle_state();

  if (digitalRead(ECHO_RISE)) {
//    Serial1.println("R");
    //oldUS = newUS;
//	  isSensorReading = true;
  }
  else {
//    Serial1.println("F\t");
  	isSensorReading = false;
    
    //calc microseconds
    diffUS = (newUS-oldUS);
    diffUS = diffUS < 0 ? diffUS + 65536 : diffUS; //do diff modulo 2^16
    diffUS /= (CPU_HZ / TIMER_ECHO_PRESCALE / 1E6); //convert timer ticks to microseconds
//    Serial1.println(diffUS);

    inches = convertInches(diffUS);
    Serial1.print("Inches:   "); Serial1.println(inches);
    former_distance = distance;
    distance = inches;
  	fsm(distance - former_distance);

    Serial1.print("PEOPLE COUNT:\t\t:"); Serial1.println(person_counter);
  }

}
