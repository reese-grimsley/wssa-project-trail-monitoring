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

int32_t duration, cm, inches, d_delta, d_new, d_old = 0;
int32_t person_counter = 0;

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

void setupTriggerTimerCC() {
  Serial.println("Set Timer values");
  
  uint32_t compareValueWait = (uint32_t) CPU_HZ * 1 / TIMER_TRIGGER_PRESCALE; //set timer for 100ms
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

  // Configure pin for interrupt. Use TC4 WO1 and port 16 for PB09
  PORT->Group[g_APinDescription[16].ulPort].PINCFG[g_APinDescription[16].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[16].ulPort].PMUX[g_APinDescription[16].ulPin >> 1].reg = PORT_PMUX_PMUXO_E;
  
  REG_PM_APBCMASK |= PM_APBCMASK_TC4;
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5);
  
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; //disable timer so we can make changes
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

   TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT32 | TC_CTRLA_WAVEGEN_MPWM; // | TC_CTRLA_RUNSTDBY;
//   TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT32 | TC_CTRLA_WAVEGEN_NPWM | TC_CTRLA_RUNSTDBY;

   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

   setupTriggerTimerCC();

   TC->CTRLA.reg |= TC_CTRLA_ENABLE;
   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

/* Sense: 
 * None, Rise, Fall, Both, High, Low
 * 0x0   0x1   0x2   0x3   0x4   0x5
 */
void config_eic_channel(int ch, int sense, bool filt) {
  // Config channel
  EIC->CONFIG[ch/8].reg &= ~(0xF << 4*(ch%8));
  EIC->CONFIG[ch/8].reg |= (0xF & ((filt? 0x8 : 0) | (0x7 & sense))) << 4*(ch%8);
  // No wake-up
  EIC->WAKEUP.reg &= ~(1 << ch);  
  // No interrupt
  EIC->INTENCLR.reg |= 1<<ch;
  // Generate Event 
  EIC->EVCTRL.reg |= 1<<ch;
}

void config_eic() {
  PM->APBAMASK.reg |= PM_APBAMASK_EIC;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EIC_GCLK_ID) | 
              GCLK_CLKCTRL_CLKEN | 
              GCLK_CLKCTRL_GEN(0);
  EIC->CTRL.reg = EIC_CTRL_SWRST;
  while(EIC->CTRL.bit.SWRST && EIC->STATUS.bit.SYNCBUSY);

  // TODO: May need to change channel number (11)
  // Set to 1 for Rising Action
  config_eic_channel(11, 1, false);

  // Do we need another for Falling?
  //config_eic_channel(12, 2, false);   
 
  EIC->CTRL.bit.ENABLE = 1;
  while(EIC->STATUS.bit.SYNCBUSY);
}

void config_evsys() {
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EVSYS_GCLK_ID_0) |
              GCLK_CLKCTRL_CLKEN |
              GCLK_CLKCTRL_GEN(0);
  while(GCLK->STATUS.bit.SYNCBUSY);

  EVSYS->CTRL.bit.SWRST = 1;
  while(EVSYS->CTRL.bit.SWRST);

  // Event receiver
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) | // Set channel n-1
              EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_1); // Match/Capture 1 on TCC1
  // Event channel
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0) | // Set channel n
           EVSYS_CHANNEL_PATH_ASYNCHRONOUS | 
                 EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_11) |
                 EVSYS_CHANNEL_EDGSEL_BOTH_EDGES; // Detect both edges  
  // Wait channel to be ready
  while(!EVSYS->CHSTATUS.bit.USRRDY0);
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

  Serial.println(digitalPinHasPWM(8));
  Serial.println(digitalPinHasPWM(12));

  Serial.println("Configure Timer");
  setTriggerTimer();

  Serial.println("Configure EIC, Events, and GPIO");
  Serial.println(EIC->CTRL.bit.ENABLE);
  Serial.println(EVSYS->USER.reg);
  
  config_eic();
  config_evsys();
  Serial.println(EIC->CTRL.bit.ENABLE);
  Serial.println(EVSYS->USER.reg);
}

void loop() {
  #ifdef DEBUG_PRINT
  Serial.println("LOOP BEGIN");

  //idle_state();

  TcCount32* TC = (TcCount32*) TC4;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  Serial.println(TC->CC[1].reg);
  
  #endif

  duration = readDistanceUS();
  d_new = convertInches(duration);

  d_delta = d_new - d_old;

  Serial.printf("Outer loop - Distance read: %d\t Delta d: %d\r\n", d_new, d_delta);

  if (d_delta < DISTANCE_TRIGGER_THRESHOLD)  { //distance has dropped by a substantial ammount
    //something has entered space of interest, stop for a moment
    static uint16_t loop_count = 0;
    do {
      loop_count++;
      //delay to enforce ~10Hz sample rate in high activity mode
      uint32_t delayVal = int(duration / 1000) - FUDGE_FACTOR; //convert to ms leftover to delay

      digitalWrite(13, HIGH);
      delay(100 - delayVal);
      digitalWrite(13, LOW);

      //get a new reading
      d_old = d_new;
      duration = readDistanceUS();
      d_new = convertInches(duration);

      //compute difference
      d_delta = d_new - d_old;

      Serial.printf("Inner loop - Distance read: %d\t Delta d: %d\r\n", d_new, d_delta);

    } while (d_delta < DISTANCE_VARIATION_MAX); //consider differing speeds of people; walking, running, stopped

    Serial.printf("New passerby counter; stayed in loop for %d samples\r\n", loop_count);
    person_counter++;
  }
  else {
    //anything to do otherwise?
    //    Serial.println("Didn't take loop");
  }

  d_old = d_new;

  Serial.printf("People counted: %d\r\n", person_counter);

  blink(1000); //

}