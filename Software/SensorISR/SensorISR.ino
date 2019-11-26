 //Need this in boards:
//https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
//
//    REG_TC4_READREQ |= 0; //example for writing/reading directly from registers REG_$peripheral-name$_$register-name$

#if defined(ARDUINO_SAMD_ZERO)
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define DEBUG_PRINT

//pin defines
#define trigger 12
#define echo 6 //TBD
#define LED 13

//define sensor counting things
#define FUDGE_FACTOR 5
#define DISTANCE_TRIGGER_THRESHOLD -200
#define DISTANCE_VARIATION_MAX 100

//clock and timer defines
#define CPU_HZ 48000000 //CPU runs at 48MHz
#define TIMER_TRIGGER_PRESCALE 1 // don't have to use this, but 32 is the max value before we sacrifice resolution
#define FAST_SAMPLE_DELAY CPU_HZ/TIMER_TRIGGER_PRESCALE/10
#define SLOW_SAMPLE_DELAY CPU_HZ/TIMER_TRIGGER_PRESCALE
#define US_THRESHOLD CPU_HZ*50/1000 //~50ms


int32_t duration, cm, inches, d_delta, d_new, d_old = 0;
int32_t person_counter = 0;

int32_t timer_value = 0;
uint8_t fast_sample_flag = 0;

void ultraSonicISR(void) {
  Serial.println("ISR enter");
  timer_value = TCC0->COUNT.reg;
  Serial.print("Timer reads: "); Serial.println(timer_value);
  if (fast_sample_flag){
    if (timer_value > US_THRESHOLD)
      fast_sample_flag = 0;
      setTriggerTimerCC(SLOW_SAMPLE_DELAY);
      person_counter++; //can make more complex; this will cause many duplicates
  }

  else {
    if (timer_value < US_THRESHOLD) {
      fast_sample_flag = 1;
      setTriggerTimerCC(FAST_SAMPLE_DELAY);
    }
  }

  
  Serial.println("ISR end");
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
    // Set sleep mode to deep sleep - 2 may be better.
  PM->SLEEP.reg = 2;   //puts proc into idle mode (once __WFI is called). Only 1 and 2 have effect.
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // mask corresponding to deep sleep enable

//  //Disable USB port (to disconnect correctly from host
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;

  SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk; //Thank god for internet forums: https://forum.arduino.cc/index.php?topic=601522.0

  //Enter sleep mode and wait for interrupt (WFI)
//  delay(50);
  __DSB(); //data sync barrier; all instructions complete before moving past instruction
  __WFI(); //Wait for interrupt (or WFE to wait for event OR interrupt); puts processor in sleep state


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
  return pulseIn(echo, HIGH);
}

float convertInches(uint32_t us_duration) {
  return (us_duration / 2) / 74; //convert to inches
}

float convertCM(uint32_t us_duration) {
  return (us_duration / 2) / 29.1; //convert to centimeters
}

void setupDistanceTimer() {
  Serial.println("Configure PWM read TCC peripheral");
  
  REG_PM_APBCMASK |= PM_APBCMASK_TCC0;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TCC0_GCLK_ID) |
              GCLK_CLKCTRL_CLKEN |
              GCLK_CLKCTRL_GEN(0);
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TCC0->CTRLA.reg = TCC_CTRLA_SWRST;
  while(TCC0->SYNCBUSY.bit.SWRST);

  TCC0->CTRLA.reg |= TCC_CTRLA_CPTEN0 | TCC_CTRLA_RUNSTDBY;
  TCC0->EVCTRL.reg |= TCC_EVCTRL_MCEI0 | TCC_EVCTRL_TCEI0 | TCC_EVCTRL_TCEI1 |
            TCC_EVCTRL_EVACT0_RETRIGGER | TCC_EVCTRL_EVACT1_STOP;

  while(TCC0->SYNCBUSY.reg);
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while(TCC0->SYNCBUSY.reg);

  Serial.println("Done configuring Timer");
}


void setTriggerTimerCC(uint32_t wait) {
  Serial.println("Set Timer values");

  uint32_t compareValueWait = wait;
  //uint32_t compareValueWait = (uint32_t) CPU_HZ * 1 / TIMER_TRIGGER_PRESCALE; //set timer for 1s (old)
  uint32_t compareValueTrig = (uint32_t) CPU_HZ * 1E-5 / TIMER_TRIGGER_PRESCALE; //set a 10us pulse using a 48MHz base clock (APB)

  Serial.println(compareValueWait);
  Serial.println(compareValueTrig);
  
  TcCount32* TC = (TcCount32*) TC4; //TC3 otherwise; think we need TCC to use waveform output values for pin

  REG_TC3_COUNT32_COUNT = 16;
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

   Serial.println("Did we get to the end of TriggerTimer");

   //need to configure pin to be driven by this
}

/* Sense:
   None, Rise, Fall, Both, High, Low
   0x0   0x1   0x2   0x3   0x4   0x5
*/
void config_eic_channel(int ch, int sense, bool filt) {
//  // Config channel

  EIC->CONFIG[ch / 8].reg &= ~(0xF << (4 * (ch % 8))); //reset all bits in channel position
  EIC->CONFIG[ch / 8].reg |= (0xF & ((filt ? 0x8 : 0) | (0x7 & sense))) << (4 * (ch % 8));
  // No wake-up
  EIC->WAKEUP.reg &= ~(1 << ch);
  // Clear interrupt
  EIC->INTENCLR.reg |= 1 << ch;
  // Generate Event
  EIC->EVCTRL.reg |= 1 << ch;
}

void config_eic() {
  PM->APBAMASK.reg |= PM_APBAMASK_EIC;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EIC_GCLK_ID) |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN(0);


  //Configure pin 3, PA02 (so even), arduino 14 as EXTINT[2] (peripheral A)
  PORT->Group[0].PINCFG[14].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[0].PMUX[14>>1].bit.PMUXE = PORT_PMUX_PMUXE_A;
                      
  EIC->CTRL.reg = EIC_CTRL_SWRST;
  while (EIC->CTRL.bit.SWRST && EIC->STATUS.bit.SYNCBUSY);

  // TODO: May need to change channel number (11)?
  // Set to 3 for rising and falling; event system will separate the two later
  //  Setting channel number as 2 for EXTINT2
  config_eic_channel(2, 3, false); 

//  // Do we need another for Falling?
//  config_eic_channel(1, 2, false);

  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY);


  while(1) {
    if (EIC->INTFLAG.reg & 0x4) {
      Serial.println(EIC->INTFLAG.reg);
    }
  }
}

void config_evsys() {
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EVSYS_GCLK_ID_0) |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN(0);
  while (GCLK->STATUS.bit.SYNCBUSY);

  EVSYS->CTRL.bit.SWRST = 1;
  while (EVSYS->CTRL.bit.SWRST);

  // Event receiver
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) | // Set channel n-1
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0); // Match/Capture 1 on TCC0
  
  // Event channel
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0) | // Set channel n
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_11) |
                       EVSYS_CHANNEL_EDGSEL_RISING_EDGE; // Detect both edges
  // Wait channel to be ready
  while (!EVSYS->CHSTATUS.bit.USRRDY0);

  // Event receiver
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(2) | // Set channel n-1
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0); // Match/Capture 1 on TCC0
  
  // Event channel
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(1) | // Set channel n
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                       EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_11) |
                       EVSYS_CHANNEL_EDGSEL_FALLING_EDGE; // Detect both edges
  // Wait channel to be ready
  while (!EVSYS->CHSTATUS.bit.USRRDY1);
  // EVSYS is always enabled
}

void setup() {
  // Setup Serial port
#ifdef DEBUG_PRINT
  while (!Serial) blink(500);
  Serial.begin(9600);
  Serial.println("Serial started");
#endif

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  //pinMode(trigger, OUTPUT);
  //pinMode(echo, INPUT);

  d_old = 0;

  Serial.println("Configure Timer");
  setTriggerTimer();
//  setupDistanceTimer();
//
//  attachInterrupt(digitalPinToInterrupt(14), ultraSonicISR, FALLING);
//
//  Serial.println("Configure EIC, Events, and GPIO");
//  Serial.println(EIC->CTRL.bit.ENABLE);
//  Serial.println(EVSYS->USER.reg);
//
//  config_eic();
//  config_evsys();
//  Serial.println(EIC->CTRL.bit.ENABLE);
//  Serial.println(EVSYS->USER.reg);
//

}

void loop() {
  #ifdef DEBUG_PRINT
  Serial.println("LOOP BEGIN");

  idle_state();

  TcCount32* TC = (TcCount32*) TC4;
  Serial.println(TC->COUNT.reg);

  blink(1000);
  
  #endif



}
