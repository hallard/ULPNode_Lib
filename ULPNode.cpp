// **********************************************************************************
// ULPNode Library
// **********************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/4.0/
//
// For any explanation of ULPNode see 
// https://hallard.me/category/ulpnode/
//
// Written by Charles-Henri Hallard (http://hallard.me)
//
// History : V1.00 2014-07-14 - First release
//           V1.10 2015-03-23 - Changed to use RadioHead Library
//           V1.20 2017-06-13 - Lightify ULPNode original LIB (removed RadioHead)
//           V1.22 2017-06-14 - Removed ULPNode related Hardware (booster/WS2812)
//           V1.25 2017-06-30 - Fixed Thermistor calculation (thx Adafruit)
//           V1.26 2017-07-08 - Added CCS811 support
//           V1.27 2017-07-24 - Added BME280 support
//           V1.27 2019-07-16 - Cleanup before release
//
// All text above must be included in any redistribution.
//
// **********************************************************************************

#include "ULPNode.h"

// Interrupt related stuff, no way to put this member of the class
// sorry, that's dirty, I didn't find another way to do this otherwise
// if you found a way, let me know
volatile uint8_t  _adc_irq_cnt;
void (*_isrWakeCallback)();
void (*_isrSwitchCallback)();
void (*_isrWatchdogCallback)();
void (*_isrSensorCallback)();

// Table of steinhart coefficient depending on NTC Type
// Use this calculator to obtain Steinhart coefficient SH_A SH_B and SH_C
// from RT table of data sheet (I used approx 0C, 25C and 85C point)
// http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
// I did not succedeed to put initialized table flash value into ULPNode class
// just tell my if you know how to do this
const float _ntc_sh_coef[] PROGMEM = {
  // Panasonic ERTJ1VG103FA 10K SMD 0603 1%
  //0.0008171303107, 0.0002641127065, 0.0000001335159187
  // Adafruit https://www.adafruit.com/product/372
  0.001225033452,  0.0002167663117, 0.0000001695746310
} ;


/* ======================================================================
Function: INT1_InterruptHandler 
Purpose : IRQ Handler called when external Interrupt for wake is triggered
Input   : - 
Output  : - 
Comments: once fired this interrupt disable itself, could not declare it 
          member of the class else far below attachinterrupt will produce
          compiler error.
====================================================================== */
void ISR_wakeInterruptHandler(void)
{
  // Call user callback if defined
  if ( _isrWakeCallback )
    _isrWakeCallback();
  
  // disable interrupt
  bitClear(EIMSK, INT1); 
}

/* ======================================================================
Function: PCINT1_vect
Purpose : IRQ Handler called when sensors pins fire an interrupt 
Input   : - 
Output  : - 
Comments: this IRQ is for whole PCINT1 pin A0 to A5, take care if you add
          more IRQ on same pin ports
====================================================================== */
ISR (PCINT1_vect)
{
  // Call user callback if defined
  if ( _isrSensorCallback )
    _isrSensorCallback();
}

/* ======================================================================
Function: PCINT2_vect
Purpose : IRQ Handler called when switch is pressed/released (for wake)
Input   : - 
Output  : - 
Comments: once fired this interrupt disable itself
          this IRQ is for whole PCINT2 pin, take care if you add
          more IRQ on same pin ports
====================================================================== */
ISR (PCINT2_vect)
{
  // Call user callback if defined
  if ( _isrSwitchCallback )
    _isrSwitchCallback();

  // Disable Interrupt and PIN Change for switch
  bitClear(PCICR,PCIE2);
  #ifdef SWITCH_PIN
  bitClear(PCMSK2,SWITCH_PIN);  
  #endif
}

/* ======================================================================
Function: WDT_vect
Purpose : IRQ Handler for watchdog 
Input   : - 
Output  : - 
Comments: once fired this interrupt disable the watchdog
====================================================================== */
ISR (WDT_vect) 
{
  // Call user callback if defined
  if ( _isrWatchdogCallback )
    _isrWatchdogCallback();
} 

/* ======================================================================
Function: ADC_vect
Purpose : IRQ Handler for ADC 
Input   : - 
Output  : - 
Comments: used for measuring 8 samples low power mode, ADC is then in 
          free running mode for 8 samples
====================================================================== */
ISR(ADC_vect)  
{
  // Increment ADC counter
  _adc_irq_cnt++;
}

/* ======================================================================
Class   : ULPNode
Purpose : Constructor
Input   : driver 
Output  : -
Comments: -
====================================================================== */
ULPNode::ULPNode()
{
  // Push button related 
  // state machine tp wait press
  _btn_State = BTN_WAIT_PUSH; 
  _btn_Action = BTN_NONE;  // 
  _btn_LongPress = false; 

  // RGB led can be consuming, set a decent value
  RGBSetBrightness(RGB_DEFAULT_BRIGHTNESS) ;

  // No user IRQ callback defined by default 
  _isrWakeCallback = NULL;
  _isrSwitchCallback = NULL;
  _isrWatchdogCallback = NULL;
  _isrSensorCallback = NULL;

  // Init global node status
  _status = 0;

  #if DEBUG_ULPN > 0 || DEBUG > 0
  _status |= RF_NODE_STATE_DEBUG;
  #endif

  // No application watchdog
  bitClear(MCUSR, WDRF);
  setWatchdog(APP_WATCHDOG_NONE);

}

/* ======================================================================
Function: init
Purpose : configure ULPNode I/O ports 
Input   : -
Output  : last "Reset" cause
Comments: - 
====================================================================== */
uint8_t ULPNode::init()
{

  #if F_CPU == 16000000L
    // Set speed to 8MHz
  // clock_prescale_set(clock_div_2);
  #elif F_CPU == 8000000L
    // Set speed to 8MHz
  // clock_prescale_set(clock_div_1);
  #else
    // Default arduino at 125Khz 16MHz (128 prescaler)
    #error "Wrong F_CPU Value"
  #endif

  // Sensors an RF power configuration Pins to control
  // mosfet to drive power on/off
  #ifdef SENSOR_POWER_PIN
  pinMode(SENSOR_POWER_PIN, OUTPUT); 
  #endif

  #ifdef TPL5110_DONE
    digitalWrite(TPL5110_DONE, LOW);
    pinMode(TPL5110_DONE, OUTPUT);
    digitalWrite(TPL5110_DONE, LOW);
  #endif

  #ifdef RF_POWER_PIN
  pinMode(RF_POWER_PIN, OUTPUT); 
  #endif

  // On board led as output  
  #ifdef LED_ON_BOARD
  pinMode(LED_ON_BOARD, OUTPUT);
  #endif
  
  // On old ULPNode proto boards
  #ifdef OLED_POWER_PIN
  pinMode(OLED_POWER_PIN, OUTPUT);
  #endif
  #ifdef LED_RED
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  #endif
  #ifdef LED_GRN
  pinMode(LED_GRN, OUTPUT);
  digitalWrite(LED_GRN, HIGH);
  #endif
  #ifdef LED_BLU
  pinMode(LED_BLU, OUTPUT);
  digitalWrite(LED_BLU, HIGH);
  #endif

  #ifdef SOLAR_VOLTAGE
  pinMode(SOLAR_VOLTAGE, INPUT); 
  #endif

  // battery measurment pin as input 
  #ifdef BATTERY_PIN
  pinMode(BATTERY_PIN, INPUT); 
  #endif
  //digitalWrite(BATTERY_PIN, LOW); 

  // We need to manage interrupts to wake up by push button
  // Set switch to input with pullup
  #ifdef SWITCH_PIN
    #ifdef SWITCH_PULLUP
      // Use internal pull up, switch connected between GND and input pin
      pinMode(SWITCH_PIN, INPUT_PULLUP); 
    #else
      // This mode need a external pull down and switch connected between 3V3 and input pin
      pinMode(SWITCH_PIN, INPUT); 
    #endif
  #endif

  // ADC Initialization
  // ==================
  InitADCSRA();

  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  // ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  // Initialize ADC and set bandgap to read
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

  // Needed to calculate vbat
  readVcc();

  // Fill up the VBat average table
  readVBatAverage(true);

  return _reset_state;
}

/* ======================================================================
Function: attachWakeInterrupt 
Purpose : define a user callback to call when we're wake up by the booster
Input   : User callback
Output  : - 
Comments: -
====================================================================== */
void ULPNode::attachWakeInterrupt(void (*isr)())
{
  #ifdef WAKE_IRQ
  // register the user's callback with the real ISR
  _isrWakeCallback = isr;   
  #endif
}

/* ======================================================================
Function: attachSwitchInterrupt 
Purpose : define a user callback to call when user press on switch
Input   : User callback
Output  : - 
Comments: -
====================================================================== */
void ULPNode::attachSwitchInterrupt(void (*isr)())
{
  #ifdef SWITCH_PIN
  // register the user's callback with the real ISR
  _isrSwitchCallback = isr;   
  #endif
}

/* ======================================================================
Function: attachWatchdogInterrupt 
Purpose : define a user callback to call when watchdog IRQ occurs
Input   : User callback
Output  : - 
Comments: -
====================================================================== */
void ULPNode::attachWatchdogInterrupt(void (*isr)())
{
  // register the user's callback with the real ISR
  _isrWatchdogCallback = isr;   
}

/* ======================================================================
Function: attachSensorInterrupt 
Purpose : define a user callback to call when sensor IRQ occurs
Input   : User callback
Output  : - 
Comments: -
====================================================================== */
void ULPNode::attachSensorInterrupt(void (*isr)())
{
  // register the user's callback with the real ISR
  _isrSensorCallback = isr;   
}

/* ======================================================================
Function: readADCLowNoise
Purpose : Read Analog Value with reducing noise
Input   : true return the average value, false return only the sum
Output  : average value read
Comments: hard coded to read 8 samples each time
          AD MUX Channel and ADC must have been set before this call
====================================================================== */
uint16_t ULPNode::readADCLowNoise(boolean average)
{
  uint8_t low, high;
  uint16_t sum = 0;
  
  // Start 1st Conversion, but ignore it, can be hazardous
  ADCSRA |= _BV(ADSC); 
  
  // wait for first dummy conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Init our measure counter
  _adc_irq_cnt = 0;

  // We want to have a interrupt when the conversion is done
  ADCSRA |= _BV( ADIE );

  // Loop thru samples
  // 8 samples (we don't take the 1st one)
  do {
    // Enable Noise Reduction Sleep Mode
    set_sleep_mode( SLEEP_MODE_ADC );
    sleep_enable();

    // Wait until conversion is finished 
    do {
      // enabled IRQ before sleeping
      sei();
      sleep_cpu();
      cli();
    }
    // Check is done with interrupts disabled to avoid a race condition
    while (bit_is_set(ADCSRA,ADSC));

    // No more sleeping
    sleep_disable();
    sei();
    
    // read low first
    low  = ADCL;
    high = ADCH;
    
    // Sum the total
    sum += ((high << 8) | low);
  }
  while (_adc_irq_cnt<8);
  
  // No more interrupts needed for this
  // we finished the job
  ADCSRA &= ~ _BV( ADIE );
  
  // Return the average divided by 8 (8 samples)
  return ( average ? sum >> 3 : sum );
}

/* ======================================================================
Function: readVcc
Purpose : Read and Calculate V powered, the Voltage on Arduino VCC pin
Input   : -
Output  : value readed
Comments: ADC Channel input is modified
====================================================================== */
uint16_t ULPNode::readVcc() 
{
  uint16_t value; 

  // Enable ADC (just in case going out of low power)
  power_adc_enable();
  ADCSRA |= _BV(ADEN)  ;

  // Read 1.1V reference against AVcc
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

  // Take care, changing reference from VCC to 1.1V bandgap can take some time, this is due
  // to the fact that the capacitor on aref pin need to discharge
  // or to charge when we're just leaving power down mode
  // power down does not hurt and 15ms strong enough for ADC setup
  sleepQuickWake(WDTO_15MS);  

  // read value
  //value = readADC(true);
  value = readADCLowNoise(true);

  // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  // _vcc = (( 1023L * config.aRef) / _vcc); 

  // Vcc reference in millivolts
  _vcc =  ( 1023L * 1100L /*config.aRef*/) / value ; 
  
  // Operating range of ATMega
  if (_vcc < 1800 ) _vcc = 1800 ;
  if (_vcc > 5500 ) _vcc = 5500 ;
    
  // Vcc in millivolts
  return ( _vcc ); 
}


/* ======================================================================
Function: readVoltage
Purpose : Read voltage on analog pin
Input   : analog pin (A0 to A7)
          R1 and R2 voltage divider if any on entry pîn (10K + 10K)
          if R1 = R2 = 0 assume full range, no voltage divider
Output  : value of volatge in mV (ex: 1500 for 1.5V)
Comments: ADC Channel input is modified, booster should be enabled
====================================================================== */
uint16_t ULPNode::readVoltage(uint8_t analog_pin, uint8_t r1, uint8_t r2) 
{
  unsigned long voltage;
  unsigned long vmax = 3600L;

  // Maximum voltage possible depending on divider
  // assuming board running at 3.6V MAX
  if (r1 && r2) {
    vmax = ( vmax * (r2+r1) ) / r2;
  } 

  // Enable ADC (just in case going out of low power)
  power_adc_enable();
  ADCSRA |= _BV(ADEN)  ;

  // Init ADC and select Channel 
  SetAnalogPinToRead(analog_pin);

  // read average value
  voltage = readADCLowNoise(true);
  
  /*
  Serial.print(F("\nReading Raw Voltage (A"));
  Serial.print(analog_pin-14);
  Serial.print(F(", "));
  Serial.print(r1);
  Serial.print(F("K, "));
  Serial.print(r2);
  Serial.print(F("K) max="));
  Serial.print(vmax);
  Serial.print(F("mV : "));
  Serial.print(voltage);
  Serial.flush();
  */

  // Calculate Voltage knowing VCC value (in mV)  
  voltage = voltage * _vcc / 1024L ;

  //Serial.print(F(" -> calculated "));
  //Serial.print(voltage);
  //Serial.print(F(" vcc="));
  //Serial.print(_vcc);
  
  // voltage divider recalculation
  // multiply first to avoid value truncation
  if (r1 && r2) {
    voltage *= (r2 + r1);
    voltage /= r2 ;
  }

  // Operating range 
  if (voltage < 100  ) voltage = 100 ;
  if (voltage > vmax ) voltage = vmax ;

  //Serial.print(F(" adjusted -->"));
  //Serial.println(voltage);
  //Serial.flush();

  return ( (uint16_t) voltage ) ;
}

/* ======================================================================
Function: readVBat
Purpose : Read VBAT/VIN value 
Input   : -
Output  : value of battery level in mV (ex: 1500 for 1.5V)
Comments: ADC Channel input is modified
====================================================================== */
uint16_t ULPNode::readVBat() 
{
  #ifdef BATTERY_PIN
  _vbat = readVoltage(BATTERY_PIN, VBATT_R1, VBATT_R2) ;
  #else
  _vbat = readVcc();
  #endif
  return ( _vbat );
}

/* ======================================================================
Function: readVBatAverage
Purpose : Read VBAT/VIN value 
Input   : -
Output  : value of battery level in mV (ex: 1500 for 1.5V)
        : Differences with readVBatAverage(), is that this one read VBAT
          but place value in a table, and return the average of the table
          this permit us to have the last 8 values average with more
          precision, using the last 8 sampled valuessleepDeviceWake
Comments: ADC Channel input will be modified
====================================================================== */
uint16_t ULPNode::readVBatAverage(boolean reset_table) 
{
  // No average value from now ?
  // we're called from setup, 1st init
  //if (_vbat_tbl_idx>VBATT_AVG_VALUES)
  if (reset_table) {
    // Read all values except the last one (so 7)
    for (_vbat_tbl_idx=0; _vbat_tbl_idx<VBATT_AVG_VALUES-1 ; _vbat_tbl_idx++ )
      _vbat_avg_tbl[_vbat_tbl_idx]= readVBat();
  }

  // read value and save to the table index 
  _vbat_avg_tbl[_vbat_tbl_idx++] = readVBat();

  /*
  for (uint8_t i=0; i<VBATT_AVG_VALUES; i++)
  {
    Serial.print(F("_vbat_avg_tbl["));
    Serial.print(i);
    Serial.print(F("]="));
    Serial.println(_vbat_avg_tbl[i],DEC);
  }
  Serial.flush();
  */

  //Serial.print(F(" adjusted -->"));
  //Serial.println(_vbat);
  //Serial.flush();

  // circular buffer of values, always VBATT_AVG_VALUES filled
  if (_vbat_tbl_idx>=VBATT_AVG_VALUES)
    _vbat_tbl_idx = 0;

  return (VBatAverage());
}

/* ======================================================================
Function: VBatAverage
Purpose : return VBat Table Average
Input   : -
Output  : value of averaged battery level * 1000 (ex: 1500 for 1.5V)
        : Differences with readVBatAverage(), is that this one just return
          the average value of the table
Comments: -
====================================================================== */
uint16_t ULPNode::VBatAverage() 
{
  uint16_t sum = 0 ; // max 8 values * 5500 we fit in a uint16 !!!

  // Sum all the values
  for (uint8_t count=0; count<VBATT_AVG_VALUES; count++)
    sum += _vbat_avg_tbl[count];
    
  // calculate the average
  sum /= VBATT_AVG_VALUES;

  // we done, return the value
  return sum;
}

/* ======================================================================
Function: lowBat
Purpose : indicate if battery level is LOW
Input   : low batt threshold (default to 900mV is not set)
Output  : 1 if low battery, 0 otherwise
Comments: -
====================================================================== */
uint8_t ULPNode::lowBat(uint16_t threshold) 
{
  // need to be calculated depending on battery type
  // AA/AAA/CR3032/CR2450/CR123, ...
  // To be done !!!!
  // for testing now < 1V = lowbat
  if ( VBatAverage() < threshold ) { 
    _status |= RF_NODE_STATE_LOWBAT ;
  } else {
    _status &= ~RF_NODE_STATE_LOWBAT ;
  }

  return ( _status&RF_NODE_STATE_LOWBAT ? 1:0 );
}

/* ======================================================================
Function: setDevice
Purpose : power on or off LED, RF module, Booster and/or Sensors
Input   : bit field indicator on what device to set on/off
Output  : - 
Comments: - 
====================================================================== */
void ULPNode::setDevice(uint8_t device_state )
{
  // On board LED related stuff
  if (device_state & DEVICE_LED_ON) {
    #ifdef LED_ON_BOARD
    digitalWrite(LED_ON_BOARD, HIGH);
    #endif
  }   
  if (device_state & DEVICE_LED_OFF) {
    #ifdef LED_ON_BOARD
    digitalWrite(LED_ON_BOARD, LOW);
    #endif
  }

  // Sensor and WS2812 RGB led Powering
  if (device_state & DEVICE_SENSORS_ON) {
    #ifdef SENSOR_POWER_PIN
    digitalWrite(SENSOR_POWER_PIN, LOW);
    #endif
    #ifdef OLED_POWER_PIN
    digitalWrite(OLED_POWER_PIN, LOW);
    #endif

    #ifdef CCS811_WAKE_PIN
    pinMode(CCS811_WAKE_PIN, OUTPUT);
    digitalWrite(CCS811_WAKE_PIN, LOW);
    delayMicroseconds(100);
    #endif


    // enable back reading digital input on analog pins
    //DIDR0 = 0x00;
  }

  if (device_state & DEVICE_SENSORS_OFF) {

    #ifdef CCS811_WAKE_PIN
    ccs811_sleep();
    digitalWrite(CCS811_WAKE_PIN, HIGH);
    #endif

    //Disable I2C interface so we can control the SDA and SCL pins directly
    TWCR &= ~(_BV(TWEN)); 

    // disable I2C module this allow us to control
    // SCA/SCL pins and reinitialize the I2C bus at wake up
    TWCR = 0;
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);  

    #ifdef SENSOR_POWER_PIN
    digitalWrite(SENSOR_POWER_PIN, HIGH);
    #endif
    #ifdef OLED_POWER_PIN
    //digitalWrite(OLED_POWER_PIN, HIGH);
    #endif

    // Next TX is scheduled after TX_COMPLETE event.
    #ifdef TPL5110_DONE
    // turn off power from sensors not needed anymore
    // ulpn.setDevice(DEVICE_SENSORS_OFF); 
    pinMode(TPL5110_DONE, OUTPUT);
    for (uint8_t i=0 ; i<5; i++) {
      digitalWrite(TPL5110_DONE, HIGH);
      delayMicroseconds(100);
      digitalWrite(TPL5110_DONE, LOW);
      delayMicroseconds(100);
    }
    #endif

  }

  // RF module Powering
  #ifdef RF_POWER_PIN
  if (device_state & DEVICE_RF_ON)  
    digitalWrite(RF_POWER_PIN, LOW );
  if (device_state & DEVICE_RF_OFF) 
    digitalWrite(RF_POWER_PIN, HIGH);
  #endif
}

/* ======================================================================
Function: RGBSetBrightness
Purpose : set the RGB brighness "255 = 100divider"
Input   : 100 = 100%, 
          50  = 50%,
          00  = OFF 
Output  : - 
====================================================================== */
void ULPNode::RGBSetBrightness( uint8_t rgb_brightness)
{
  _rgb_brightness = rgb_brightness;
}

/* ======================================================================
Function: RGBSetColor
Purpose : prepare the colors for the onboard RGB Led but do nothing else
Input   : r, g, b colors
Output  : - 
Comments: -
====================================================================== */
void ULPNode::RGBSetColor(uint8_t r, uint8_t g, uint8_t b)
{
  _rgb_led.r = r ;
  _rgb_led.g = g ;
  _rgb_led.b = b ;
}

/* ======================================================================
Function: RGBHasSetColor
Purpose : determine is RGB led need to be called to set the internal color
Input   : -
Output  : return true if RGB as one color set  
Comments: -
====================================================================== */
boolean ULPNode::RGBHasSetColor(void)
{
  return ( (_rgb_led.r||_rgb_led.g||_rgb_led.b ) ? true : false) ;
}

/* ======================================================================
Function: RGBShow
Purpose : send the data color to RGB LED
Input   : -
Output  : - 
Comments: This is working only if booster is enabled AND sensor power
          is activated.
          during data transfer, CPU is working at 8MHz
====================================================================== */
uint8_t ULPNode::RGBAdjustBrightness(uint8_t level)
{
  uint16_t value = level; 
  value *= _rgb_brightness;
  value /= 100;
  return  (uint8_t) value;
}

/* ======================================================================
Function: RGBShow
Purpose : send the data color to RGB LED
Input   : -
Output  : - 
Comments: This is working only if booster is enabled AND sensor power
          is activated.
          during data transfer, CPU is working at 8MHz
====================================================================== */
void ULPNode::RGBShow(void)
{
  struct cRGB led; // Create a local instance 

  // copy values and adjust brightness 
  led.r = RGBAdjustBrightness(_rgb_led.r);
  led.g = RGBAdjustBrightness(_rgb_led.g);
  led.b = RGBAdjustBrightness(_rgb_led.b);
  //led.g = _rgb_led.g >> _rgb_brightness;
  //led.b = _rgb_led.b >> _rgb_brightness;

// Atmega328p Lora Node Pro Mini 8MHz 3V3
#if defined (ARDUINO_AVR_MINILORA) || (defined (ARDUINO_AVR_PRO) && (F_CPU == 8000000L))

  // Green is too lighty, divide by 4
  led.g = led.g >> 2;

  // For Common Anode RGB LED else comment these lines
  led.r = 255 - led.r;
  led.g = 255 - led.g;
  led.b = 255 - led.b;
#endif

#ifdef LED_PWM
  // Assume old ULPNode revision with 3 LED on PWN Common Anode
  #ifdef LED_RED
    analogWrite(LED_RED,  led.r );
  #endif
  #ifdef LED_GRN
    analogWrite(LED_GRN,  led.g );
  #endif
  #ifdef LED_BLU
    analogWrite(LED_BLU,  led.b );
  #endif
#else
  #ifdef LED_RED
    digitalWrite(LED_RED, led.r );
  #endif
  #ifdef LED_GRN
    digitalWrite(LED_GRN,  led.g );
  #endif
  #ifdef LED_BLU
    digitalWrite(LED_BLU,  led.b );
  #endif
#endif
}

/* ======================================================================
Function: RGBShow
Purpose : set the color of the onboard RGB Led and show immediatly
Input   : r, g, b colors
Output  : - 
Comments: -
====================================================================== */
void ULPNode::RGBShow(uint8_t r, uint8_t g, uint8_t b)
{
  // Set the led color
  RGBSetColor(r, g, b);

  // send to chip
  RGBShow();
}

/* ======================================================================
Function: RGBBlink
Purpose : blink the RGB Led x times
Input   : number of blink
          r, g, b color
          delay between blink (WDTO_60MS by default)
Output  : - 
Comments: This is working only if booster is enabled AND sensor powered
====================================================================== */
void ULPNode::RGBBlink(uint8_t num, uint8_t r, uint8_t g, uint8_t b, uint8_t dly)
{
  while (num--) {
    RGBShow(r,g,b);

    #ifdef LED_PWM
      long thisdelay = 15 * (1 << dly);
      // Start at 15ms and multiply per 2 each increment
      // seems simple and not too bad
      // WDTO_15MS => 15 ms
      // WDTO_120MS => 120 ms
      // WDTO_2S => 1920 ms
      wdt_reset(); 
      delay(thisdelay);
      RGBShow(0,0,0);
      wdt_reset(); 
      delay(thisdelay);
    #else
      sleepQuickWake(dly);
      RGBShow(0,0,0);
      sleepQuickWake(dly);
    #endif
  }
}


/* ======================================================================
Function: LEDBlink
Purpose : blink the onboard Led x times
Input   : number of blink
          delay between blink (WDTO_60MS by default)
Output  : - 
Comments: -
====================================================================== */
void ULPNode::LEDBlink(uint8_t num, uint8_t dly)
{
  while (num--) {
    setDevice(DEVICE_LED_ON);
    sleepQuickWake(dly);
    setDevice(DEVICE_LED_OFF);
    sleepQuickWake(dly);
  }
}


/* ======================================================================
Function: setSensorsIRQ
Purpose : enable IRQ for sensors 
Input   : sensors irq mode flags (A0, A1 or A3) 
          true to enable pullup
Output  : - 
Comments: Configure IRQ to be able to wake up from sleep
====================================================================== */
void ULPNode::setSensorsIRQ(uint8_t sensors_irq, boolean enable_pullup )
{
  // save current interrupt state
  uint8_t oldSREG = SREG;

  // switch all interrupts off while messing with their settings  
  cli();  

  // Sensors A0, A1, A3 related IRQ
  // ------------------------------
  if ( sensors_irq ) {
    // clear any pending interrupts for theese IRQ
    if ( sensors_irq & IRQ_SENSORS_ENABLE )
      bitSet(PCIFR,PCIF1);

    // A0 input pullup, authorize IRQ pin
    if (sensors_irq & IRQ_SENSOR_A0_ENABLE) {
      pinMode(A0, INPUT_PULLUP);
      if (enable_pullup)
        digitalWrite (A0, 1);  
      PCMSK1 |= _BV(PCINT8); // A0
    }
    // A1 input, authorize IRQ pin
    if (sensors_irq & IRQ_SENSOR_A1_ENABLE) {
      pinMode(A1, INPUT);
      if (enable_pullup)
        digitalWrite (A1, 1);  
      PCMSK1 |= _BV(PCINT9); // A1
    }
    // A3 input pullup, authorize IRQ pin
    if (sensors_irq & IRQ_SENSOR_A3_ENABLE) {
      pinMode(A3, INPUT_PULLUP);
      if (enable_pullup)
        digitalWrite (A3, 1);  
      PCMSK1 |= _BV(PCINT11); // A3
    }

    if ( sensors_irq & IRQ_SENSOR_A0_DISABLE)
      PCMSK1 &= ~_BV(PCINT8); // A0
    if ( sensors_irq & IRQ_SENSOR_A1_DISABLE)
      PCMSK1 &= ~_BV(PCINT9); // A1
    if ( sensors_irq & IRQ_SENSOR_A3_DISABLE)
      PCMSK1 &= ~_BV(PCINT11); // A3

    // Authorize global sensors interrupts if one set
    if ( sensors_irq & IRQ_SENSORS_ENABLE )
      bitSet(PCICR,PCIE1);

    // Disable global sensors interrupts if all three disabled
    if ( (sensors_irq & IRQ_SENSORS_DISABLE) == IRQ_SENSORS_DISABLE ) 
      bitClear(PCICR,PCIE1);
  }

  // restore interrupt previous state
  SREG = oldSREG;  
}
  
/* ======================================================================
Function: setIRQ
Purpose : enable IRQ for Push button and/or Booster
Input   : irq mode flags
Output  : - 
Comments: Configure IRQ to be able to wake up from sleep
====================================================================== */
void ULPNode::setIRQ(uint8_t irq_mode)
{
  // save current interrupt state
  uint8_t oldSREG = SREG;

  // switch all interrupts off while messing with their settings  
  cli();  

  #ifdef WAKE_IRQ
  // Booster related IRQ
  // -------------------
  if ( irq_mode & IRQ_EXTWAKE ) {
    // clear any pending interrupts for Wake
    bitSet(EIFR, WAKE_IRQ);

    // Do we want to enable the booster IRQ ?
    if (irq_mode & IRQ_EXTWAKE_ENABLE) {
      // set INT to trigger on falling edge 
      //bitClear(EICRA, ISC10);    
      //bitSet(  EICRA, ISC11);    

      // enable this IRQ
      //bitSet( EIMSK, WAKE_IRQ); 

      // Dirty Hack for global ISR in a class
      // You can't declare class::ISR_wakeInterruptHandler so above
      // ISR_wakeInterruptHandler is declared as "global", not clean but works
      attachInterrupt(WAKE_IRQ, ISR_wakeInterruptHandler, FALLING);
    }

    // Do we want to disable the booster IRQ ?
    if (irq_mode & IRQ_EXTWAKE_DISABLE) {
      // disable booster interrupt
      // bitClear( EIMSK, WAKE_IRQ); 
      detachInterrupt(WAKE_IRQ);
    }
  }
  #endif

  // Switch push button related IRQ
  // ------------------------------
  #ifdef SWITCH_PIN
  if ( irq_mode & IRQ_SWITCH ) {
    // clear any pending interrupts for switch
    bitSet(PCIFR,PCIF2);

    // Do we want to enable the switch IRQ ?
    if (irq_mode & IRQ_SWITCH_ENABLE ) {
      // Enable Interrupt and PIN Change for switch
      bitSet(PCICR,PCIE2);
      bitSet(PCMSK2,SWITCH_PIN);      
    }

    // Do we want to disable the Switch IRQ ?
    if (irq_mode & IRQ_SWITCH_DISABLE) {
      // Disable Interrupt and PIN Change for this port/bit
      bitClear(PCICR,PCIE2);
      bitClear(PCMSK2,SWITCH_PIN);         
    }
  }
  #endif

  // restore interrupt previous state
  SREG = oldSREG;  
}

/* ======================================================================
Function: sleepQuickWake
Purpose : quick sleep mode for wdt period
Input   : Watchdog duration 
Output  : - 
Comments: nothing is touched on this (ADC, IRQ, whatever ...)
          this is mainly used to do quick pause such as blink LED or
          wait a device to have time to initialize
====================================================================== */
void ULPNode::sleepQuickWake(uint8_t wdt_period)
{
  // save current interrupt state
  uint8_t oldSREG = SREG;

  // switch all interrupts off while messing with their settings  
  // or doing something that can trigger one
  // we will sleep, WE DO NOT WANT to be interrupted before sleeping
  cli();  

  // WDTCSR period msb bit watchdog period is not contiguous 
  // adapt wdt_period to be compatible with  WDTCSR
  wdt_period = ((wdt_period & 0x08)?_BV(WDP3):0x00) | (wdt_period & 0x07);
  
  // allow changes on watchdog config
  // set interrupt mode and period, and do a fresh start
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | wdt_period;    
  wdt_reset(); 

  // Set Low Power to max during sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable(); 
  
  // turn off BOD (sequenced order see datasheet)
  // Don't change anything before we're waked
  MCUCR = _BV(BODS) | _BV(BODSE);
  MCUCR = _BV(BODS); 
  sei();          
  sleep_cpu();   
   
  // ZZZZZZZZZZZ
  
  // Waked !!!
  sleep_disable();

  // allow changes on watchdog config
  // set interrupt mode 
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) ;    

  // enable back application watchdog if any
  if (_watchdog != APP_WATCHDOG_NONE) {
    setWatchdog(_watchdog);
  } else {
  }

  // Restore original Interrupts state
  SREG = oldSREG;
}

/* ======================================================================
Function: sleepDeviceWake
Purpose : prepare sleep mode for total power down and sleep
Input   : what IRQ could wake us
          Watchdog duration if we want it to wake us
Output  : - 
Comments: - 
====================================================================== */
void ULPNode::sleepDeviceWake(uint8_t mode, uint8_t wdt_period)
{
  uint8_t irq_mode = 0;
  
  // save current interrupt state
  uint8_t oldSREG = SREG;

  // switch all interrupts off while messing with their 
  // settings or doing something that can trigger one
  // we will sleep, WE DO NOT WANT to be interrupted before sleeping
  cli();  


  // Does the external device if any need to wake us ?
  irq_mode |= (mode & SLEEP_WAKE_EXT) ? IRQ_EXTWAKE_ENABLE : IRQ_EXTWAKE_DISABLE ;

  // Does the switch push need to wake us ?
  irq_mode |= (mode & SLEEP_WAKE_SWITCH)  ? IRQ_SWITCH_ENABLE : IRQ_SWITCH_DISABLE ;

  // Setting IRQ
  setIRQ(irq_mode);
  
  // Do we want watchdog to wake us with interrupt ?
  if (mode & SLEEP_WAKE_WATCHDOG) {
    // WDTCSR period msb bit watchdog period is not contiguous 
    // adapt wdt_period to be compatible with WDTCSR
    wdt_period = ((wdt_period & 0x08)?_BV(WDP3):0x00) | (wdt_period & 0x07);
    
    // allow changes on watchdog config
    // set interrupt mode and period, and do a fresh start
    WDTCSR = _BV(WDCE) | _BV(WDE) ;
    WDTCSR = _BV(WDIE) | wdt_period;    
    wdt_reset(); 
  } else {
    // we don't want to be waked by the watchdog, so be 
    // sure to disable it by changing the config
    WDTCSR = _BV(WDCE) | _BV(WDE);
    WDTCSR = 0;    
  }

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable(); 
  
  // Sequence timed instruction don't try to optimize this
  // part, even if temptation is high on common sei/sleep_cpu
  if (mode & SLEEP_BOD_OFF) {
    // turn off BOD (sequenced order see datasheet)
    // Don't change anything before we're waked
    MCUCR = _BV(BODS) | _BV(BODSE);
    MCUCR = _BV(BODS); 
    sei();          
    sleep_cpu();   
  } else {
    sei();          
    sleep_cpu();   
  }
   
  // ...........
  // ZZZZZZZZZZZ
  // ...........
  
  // Waked !!!
  sleep_disable();

  // enable back application watchdog if any
  if (_watchdog != APP_WATCHDOG_NONE)
    setWatchdog(_watchdog);
  
  // Restore original Interrupts
  SREG = oldSREG;
}

/* ======================================================================
Function: setWatchdog
Purpose : enable or disable application watchdog
Input   : application watchdog value in
          APP_WATCHDOG_NONE no watchdog
          APP_WATCHDOG_1S to APP_WATCHDOG_8S
Output  : - 
Comments: - 
====================================================================== */
void ULPNode::setWatchdog(uint8_t time_out)
{
  // enable if valid
  wdt_reset();
  if ( time_out == APP_WATCHDOG_1S || time_out == APP_WATCHDOG_2S || 
       time_out == APP_WATCHDOG_4S || time_out == APP_WATCHDOG_8S ) {
    _watchdog = time_out;  
    wdt_enable(_watchdog);
  } else {
    // all other cases, we disable it
    _watchdog = APP_WATCHDOG_NONE;  
    wdt_disable();
  }
}

/* ======================================================================
Function: disableCPUDevices
Purpose : disable Atmel integrated devices (for low power)
Input   : -
Output  : - 
Comments: - 
====================================================================== */
void ULPNode::disableCPUDevices(void)
{
  // Disable ADC 
  ADCSRA &= ~_BV(ADEN)  ; 

  // disable Analog comparator  
  ACSR |= _BV(ACD); 
  
  // Disable digital input buffers on all ADC0-ADC5 pins
  //DIDR0 = 0x3F;    

  // set I2C pin as input no pull up
  // this prevent current draw on I2C pins that
  // completly destroy our low power mode

  //Disable I2C interface so we can control the SDA and SCL pins directly
  TWCR &= ~(_BV(TWEN)); 

  // disable I2C module this allow us to control
  // SCA/SCL pins and reinitialize the I2C bus at wake up
  TWCR = 0;
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);  

  /*
  power_adc_disable();
  power_usart0_disable();
  power_spi_disable();  
  power_twi_disable();
  power_timer0_disable(); 
  power_timer1_disable();
  power_timer2_disable();
  */

  power_all_disable();
}

/* ======================================================================
Function: buttonManageState
Purpose : manage button states (longpress actions/simple press/debounce)
Input   : pin button state (LOW or HIGH) we read just before
Output  : current state machine
Comments: need to be called after push until state machine = BTN_WAIT_PUSH
          code inspired from one button library
====================================================================== */
btn_state_e ULPNode::buttonManageState(uint8_t buttonLevel)
{
  // get current time in msecs
  unsigned long now = millis(); 

  // Implementation of the state machine
  // waiting for menu pin being pressed.
  if (_btn_State == BTN_WAIT_PUSH) { 
    // Putton pushed ?
    if (buttonLevel == BTN_PRESSED) {
      // now wait for release
      _btn_State = BTN_WAIT_RELEASE; 
      // Set starting time
      _btn_StartTime = now; 
    } 

  // we're waiting for button released.
  } else if (_btn_State == BTN_WAIT_RELEASE) { 
    // effectivly released ?
    if (buttonLevel == BTN_RELEASED) {
      // next step is to check debounce 
      _btn_State = BTN_WAIT_DEBOUNCE; 
    // still pressed, is it a Long press that is now starting ?
    } else if ((buttonLevel == BTN_PRESSED) && (now > _btn_StartTime + BTN_LONG_PUSH_DELAY)) {
      // Set long press state
      _btn_LongPress = true;  

      // step to waiting long press release
      _btn_State = BTN_WAIT_LONG_RELEASE; 
    }

  // waiting for being pressed timeout.
  } else if (_btn_State == BTN_WAIT_DEBOUNCE) { 
    // do we got debounce time reached when released ?
    if (now > _btn_StartTime + BTN_DEBOUNCE_DELAY) 
      _btn_Action = BTN_QUICK_PRESS;
    else
      _btn_Action = BTN_BAD_PRESS;

    // restart state machine
    _btn_State = BTN_WAIT_PUSH; 

  // waiting for menu pin being release after long press.
  } else if (_btn_State == BTN_WAIT_LONG_RELEASE) { 
    // are we released the long press ?
    if (buttonLevel == BTN_RELEASED) {
      // we're not anymore in a long press
      _btn_LongPress = false;  

      // be sure to light off the blinking RGB led
      RGBShow(0, 0, 0);

      // We done, restart state machine
      _btn_State = BTN_WAIT_PUSH; 
    } else {
      uint8_t sec_press;

      // for how long we have been pressed (in s)
      sec_press = ((now - _btn_StartTime)/1000L);

      // we're still in a long press
      _btn_LongPress = true;

      // We pressed button more than 7 sec
      if (sec_press >= 7 ) {
        // Led will be off 
        _btn_Action = BTN_TIMEOUT;
      // We pressed button between 6 and 7 sec
      } else if (sec_press >= 6  ) {
        _btn_Action = BTN_PRESSED_67;
        // Prepare LED color
        RGBSetColor(RGB_RED);
      // We pressed button between 5 and 6 sec
      } else if (sec_press >= 5  ) {
        _btn_Action = BTN_PRESSED_56;
        RGBSetColor(RGB_YELLOW);
      // We pressed button between 4 and 5 sec
      } else if (sec_press >= 4  ) {
        _btn_Action = BTN_PRESSED_45;
        RGBSetColor(RGB_CYAN);
      // We pressed button between 3 and 4 sec
      } else if (sec_press >= 3 ) {
        _btn_Action = BTN_PRESSED_34;
        RGBSetColor(RGB_GREEN);
      // We pressed button between 2 and 3 sec
      } else if (sec_press >= 2 ) {
        _btn_Action = BTN_PRESSED_23;
        RGBSetColor(RGB_BLUE);
      // We pressed button between 1 and 2 sec
      } else if (sec_press >= 1 ) {
        _btn_Action = BTN_PRESSED_12;
        RGBSetColor(RGB_PURPLE);
      }

      // manage the fast blinking 
      // 20ms ON / 80ms OFF
      if (millis() % 100 < 10 ) 
        RGBShow();
      else 
        RGBShow(0,0,0);
    }   
  }

  // return the state machine we're in
  return (_btn_State);
} 


/* ======================================================================
Function: powerRadio
Purpose : expose driver method of power or unpower the RF module 
Input   : true to power on false to power off
Output  : true if powered and module found
Comments: -
====================================================================== */
boolean ULPNode::powerRadio(uint8_t power)
{

  // do we need to power up the sensors
  if ( power) {
    uint8_t status_mask = 0;

    // From here and with latest Arduino version we have a problem
    // Arduino SPI library now check if SPI has already been initialized
    // if so, init is not done again and as we changed SS pin and some 
    // others to have full Low Power, we need to enhance back all as it
    // should be done in a real Spi init EACH time.
  
    //SPCR |= _BV(SPE);

    // Warning: if the SS pin ever becomes a LOW INPUT then SPI
    // automatically switches to Slave, so the data direction of
    // the SS pin MUST be kept as OUTPUT.

    // set back CSN pin with pullup (it was input)
    digitalWrite(RF_CSN_PIN, HIGH); 
    // now set it as output high
    pinMode(RF_CSN_PIN, OUTPUT); 
    digitalWrite(RF_CSN_PIN, HIGH); 

    // Power Up Modules SPI 
    power_spi_enable();  

    // Enable back SPI and set as MASTER
    SPCR |= _BV(SPE);
    SPCR |= _BV(MSTR);
  }
  
  // So this is a power off
  if ( !power) {
    // This will configure the radio pins for correct low power mode
    //driver.sleep();

    // We're going to sleep, we've done our job no need to be awake by
    // RF module firing up a IRQ when we're in power down (can cause trouble?)
    //detachInterrupt(digitalPinToInterrupt(RF_IRQ_PIN));

    // Once agin, very important even if we power off the module, because 
    // of pullup, module still powered via SS/IRQ Pin. if we don't do this
    // even if VDD of RFModule set to "float" using mosfet, current is get
    // drawn by other pins pullup (CS or IRQ), so disable pull up
    //pinMode(RF_CSN_PIN, INPUT); 
    //digitalWrite(RF_CSN_PIN, 0); 

    // Disable SPI device
    SPCR &= ~_BV(SPE);
    
    // unpower SPI of Arduino
    power_spi_disable();  

    // unpower RF module
    setDevice(DEVICE_RF_OFF);
  }

  return (true);
}  

/* ======================================================================
Function: i2cScan
Purpose : scan I2C bus and check for known devices compatible with ULPNode
Input   : -
Output  : -
Comments: global status flags updated with devices found
====================================================================== */
uint8_t ULPNode::i2cScan()
{
  byte error, address;
  uint8_t nDevices;

  // Enable I2C (just in case)
  power_twi_enable();
  Wire.begin();

  // Set I2C 100KHz for 4MHz speed
  TWBR = ((4000000L / 100000L) - 16) / 2;

  #if DEBUG_ULPN > 0
  DebuglnF("Scanning I2C bus ...");
  #endif

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      #if DEBUG_ULPN > 0
      DebugF("  0x");
      if (address<16) 
        DebugF("0");
      Debug(address,HEX);
      #endif
      
      #if DEBUG_ULPN > 1
      DebugF(" -> ");
      #endif
      if (address == 0x3c || address == 0x3d) {
        _status |= RF_NODE_STATE_OLED;
        #if DEBUG_ULPN > 1
        DebuglnF("OLED Display");
        #endif
      } else if (address == 0x40) {
        _status |= RF_NODE_STATE_SI7021;
        #if DEBUG_ULPN > 1
        DebuglnF("SI7021/HTU21D Temp/Hum sensor");
        #endif
      } else if (address==0x29 || address==0x39 || address==0x49) {
        _status |= RF_NODE_STATE_TSL2561;
        #if DEBUG_ULPN > 1
        DebuglnF("TSL2561 Light sensor");
        #endif
      } else if (address==0x50) {
        _status |= RF_NODE_STATE_24AA02E64;
        #if DEBUG_ULPN > 1
        DebuglnF("24AA02E64 EEP");
        #endif
        address+=0x07;
      } else if (address==0x5a || address==0x5b) {
        _status |= RF_NODE_STATE_CCS811;
        #if DEBUG_ULPN > 1
        DebuglnF("CCS811 CO sensor");
        #endif
      } else if (address==0x77 || address==0x76) {
        _status |= RF_NODE_STATE_BME280;
        #if DEBUG_ULPN > 1
        DebuglnF("BME280 Pressure sensor");
        #endif
      } else {
        #if DEBUG_ULPN > 1
        DebuglnF(" unknown device");
        #endif
      }
      #if DEBUG_ULPN > 0
      Debugln();
      #endif

      // Found 1 more device
      nDevices++;

    } else if (error==4) {
      #if DEBUG_ULPN > 0
      DebugF("Unknow error at address 0x");
      if (address<16) 
        DebugF("0");
      Debug(address,HEX);
      #endif
    }    
  }

  #if DEBUG_ULPN > 0
  if (nDevices == 0) {
    DebuglnF("No I2C devices found");
  } else {
    DebuglnF("Scan done");
  }
  DebugFlush();
  #endif

  return nDevices;
}

/* ======================================================================
Function: i2c_Init
Purpose : prepare and configure I2C when going out of low power
Input   : -
Output  : -
Comments: set global flags with ULPN devices seen
====================================================================== */
void ULPNode::i2cInit(boolean check_devices)
{
  // Enable I2C (just in case)
  power_twi_enable();

  // Init I2C Bus 
  Wire.begin();

  // Set I2C 100KHz 
  TWBR = ((F_CPU / 100000L) - 16) / 2;

  // need to check device presence ?
  if (check_devices) {
    // Clear status devices
    _status &= ~(RF_NODE_STATE_SI7021 | RF_NODE_STATE_OLED | RF_NODE_STATE_TSL2561 | RF_NODE_STATE_24AA02E64 | RF_NODE_STATE_CCS811) ;

    // Clear wire as a precaution
    while ( Wire.available() ) {
      Wire.read();
    }

    Wire.beginTransmission(ULPN_SI7021_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) 
      _status |= RF_NODE_STATE_SI7021;

    Wire.beginTransmission(ULPN_TSL2561_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) 
      _status |= RF_NODE_STATE_TSL2561;

    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) 
      _status |= RF_NODE_STATE_OLED;

    Wire.beginTransmission(0x3D);
    if (Wire.endTransmission() == 0) 
      _status |= RF_NODE_STATE_OLED;

    Wire.beginTransmission(ULPN_24AA02E64_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) 
      _status |= RF_NODE_STATE_24AA02E64;

    Wire.beginTransmission(ULPN_CCS811_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) 
      _status |= RF_NODE_STATE_CCS811;

    Wire.beginTransmission(ULPN_BME280_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) 
      _status |= RF_NODE_STATE_BME280;
  }
}

/* ======================================================================
Function: getResistorValue
Purpose : calculate resistor value (NTC/CTN/LDR,...)
Input   : Pin of the analog value we need to read (A0..A7)
          the pullup resitor value
Output  : value of the resistor in ohm
Comments: ADC Channel input is modified
          Resistor value to get is connected between Analog Pin and ADC
          and it's pulled up by R1 Resistor as follow
          
          VCC
           |
          | | 
          | |  R1 (Pullup)
          | |
           |
           |----------- Arduino Analog Pin
           |
          | |
          | |  R2 Resistor to Calculate (NTC, LDR, ...)
          | |
           |
          GND
====================================================================== */
uint32_t ULPNode::getResistorValue(uint8_t pin, uint32_t rpullup) 
{
  uint16_t vadc;
  uint32_t resistor;
  
  // Initialize ADC and mux
  SetAnalogPinToRead(pin);

  // read value, return average
  vadc = readADCLowNoise(true);

  #ifdef USE_THERMISTOR
  #if defined (THERMISTOR_PULLUP)
  // If Thermistor is down (connected to GND as pull down)
  resistor =  rpullup * ( ( 1023.0f / vadc ) - 1.0f );
  #elif defined (THERMISTOR_PULLDOWN)
  // If Thermistor is up (connected to VCC as pull up)
  resistor = rpullup / ( ( 1023.0f / vadc ) - 1.0f );
  #else
  #error "please defined how is connected thermistor"
  #endif
  #endif
  
//DebugF("R on ADC Raw A");
//Debug(pin-14, DEC);
//DebugF(" Value = ");
//Debug(vadc);
//DebugF("  => ");
//Debug(resistor);
//DebuglnF(" Ohms");
//DebugFlush();

  return (resistor);
}


/* ======================================================================
Function: getTemperatureWithSteinhart
Purpose : Get temperature from resistor value using Steinhart-Hart equation
Input   : analog pin
          resistor pullup value
Output  : temperature value * 100 rounded
Comments: See https://learn.adafruit.com/thermistor?view=all
====================================================================== */
int16_t ULPNode::getTemperatureWithSteinhart(uint32_t resistor_value) 
{
  float steinhart;

  #ifdef USE_THERMISTOR

  steinhart = ((float) resistor_value) / THERMISTOR_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= THERMISTOR_BCOEFF;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (THERMISTOR_TNOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;   // convert to C
  steinhart *= 100.0;

  // check positive number and do round
  if (steinhart >= 0.0f) {
    steinhart = floor(steinhart + 0.5f);
  } else {
    steinhart = ceil(steinhart - 0.5f);
  }

  #endif 

  // convert cast to int value
  return (static_cast<int16_t>(steinhart));
}

/* ======================================================================
Function: getTemperature
Purpose : return temperature measured with the onboard 10K thermistor
Input   : -
Output  : temperature value * 100 rounded
Comments: -
====================================================================== */
int16_t ULPNode::getTemperature(void) 
{
  // Get thermistor resistor value
  uint32_t resistor;

  #ifdef THERMISTOR_PULLUP 
  // Read thermistor value
  resistor=getResistorValue(THERMISTOR_PIN, THERMISTOR_PULLUP);

  // convert to temperature and save
  _ntc_temperature = getTemperatureWithSteinhart(resistor);
  #else
  _ntc_temperature = 0;
  #endif

  // return value
  return (_ntc_temperature);
}


/* ======================================================================
Function: i2c_readRegister
Purpose : read a register from i2c device
Input   : I2C device address
          register address
          register value filled with read value
Output  : 0 if okay
Comments: -
====================================================================== */
uint8_t ULPNode::i2c_readRegister(uint8_t i2caddr, uint8_t reg, uint8_t * value)
{
  uint8_t err;
  Wire.beginTransmission(i2caddr);
  Wire.write(reg);      
  err = Wire.endTransmission();
  // all was fine ?
  if ( err==0 ) {
    // request 1 byte and have it ?
    if (Wire.requestFrom(i2caddr, 1)==1) {
      // return value
      *value = Wire.read();
      return 0;
    } else {
      return 5; // Other than I2C error code
    }
  }
  return err;
}

/* ======================================================================
Function: i2c_readRegister16
Purpose : read 16 bits register from i2c device
Input   : I2C device address
          register address
          register value filled with read value
Output  : 0 if okay
Comments: -
====================================================================== */
uint8_t ULPNode::i2c_readRegister16(uint8_t i2caddr, uint8_t reg, uint16_t * value)
{
  uint8_t err;
  Wire.beginTransmission(i2caddr);
  Wire.write(reg);      
  err = Wire.endTransmission();
  // all was fine ?
  if ( err==0 ) {
    // request 2 bytes and have it ?
    if (Wire.requestFrom(i2caddr, 2)==2) {
      // return value
      *value  = Wire.read()<<8;
      *value |= Wire.read();
      return 0;
    } else {
      return 5; // Other than I2C error code
    }
  }
  return err;
}

/* ======================================================================
Function: i2c_readRegister16_LE
Purpose : read 16 bits register from i2c device Low Endian format
Input   : I2C device address
          register address
          register value filled with read value
Output  : 0 if okay
Comments: -
====================================================================== */
uint8_t ULPNode::i2c_readRegister16_LE(uint8_t i2caddr, uint8_t reg, uint16_t * value) 
{
  uint16_t temp;
  uint8_t err = i2c_readRegister16(i2caddr, reg, &temp);
  *value = (temp >> 8) | (temp << 8);
  return err;
}

/* ======================================================================
Function: i2c_readRegister16_S
Purpose : read 16 bits signed register from i2c device 
Input   : I2C device address
          register address
          register value filled with read value
Output  : 0 if okay
Comments: -
====================================================================== */
uint8_t ULPNode::i2c_readRegister16_S(uint8_t i2caddr, uint8_t reg, int16_t * value) 
{
  return ( i2c_readRegister16(i2caddr, reg, (uint16_t *) value) );
}

/* ======================================================================
Function: i2c_readRegister16_S_LE
Purpose : read 16 bits signed register from i2c device Low Endian format
Input   : I2C device address
          register address
          register value filled with read value
Output  : 0 if okay
Comments: -
====================================================================== */
uint8_t ULPNode::i2c_readRegister16_S_LE(uint8_t i2caddr, uint8_t reg, int16_t * value) 
{
  return ( i2c_readRegister16_LE(i2caddr, reg, (uint16_t *) value) );
}

/* ======================================================================
Function: i2c_readRegister24
Purpose : read 24 bits  register from i2c device 
Input   : I2C device address
          register address
          register value filled with read value
Output  : 0 if okay
Comments: -
====================================================================== */
uint8_t ULPNode::i2c_readRegister24(uint8_t i2caddr, uint8_t reg, uint32_t * value) 
{
  uint8_t err;
  Wire.beginTransmission(i2caddr);
  Wire.write(reg);      
  err = Wire.endTransmission();
  // all was fine ?
  if ( err==0 ) {
    // request 2 bytes and have it ?
    if (Wire.requestFrom(i2caddr, 3)==3) {
      uint32_t data;
      // return value
      data = Wire.read();
      data <<= 8;
      data |= Wire.read();
      data <<= 8;
      data |= Wire.read();
      *value = data;
      return 0;
    } else {
      return 5; // Other than I2C error code
    }
  }
  return err;
}

/* ======================================================================
Function: i2c_writeRegister
Purpose : write a value to a register from i2c device
Input   : I2C device address
          register address
          register value to write
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::i2c_writeRegister(uint8_t i2caddr, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(reg);
  Wire.write(value); 
  return (Wire.endTransmission()); 
}

/* ======================================================================
Function: i2c_writeValue
Purpose : write a value to i2c device
Input   : I2C device address
          value to write
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::i2c_writeValue(uint8_t i2caddr, uint8_t value)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(value); 
  return (Wire.endTransmission()); 
}

/* ======================================================================
Function: si7021_StartConv
Purpose : return temperature or humidity measured 
Input   : data type ULPN_SI7021_READ_HUM or ULPN_SI7021_READ_TEMP
Output  : 0 if okay
Comments: internal values of temp and rh are set
====================================================================== */
int8_t ULPNode::si7021_StartConv(si7021_e datatype)
{
  long data;
  uint16_t raw ;
  uint8_t checksum,tmp;
  uint8_t error;

  // Request a reading Humidiy, will read temp later
  error = si7021_writeValue(datatype==ULPN_SI7021_READ_HUM?ULPN_SI7021_MEASURE_HUM:ULPN_SI7021_MEASURE_TEMP);
  if ( error ) {
    return error;
  }

  // Comes back in three bytes, data(MSB) / data(LSB) / Checksum
  Wire.requestFrom(ULPN_SI7021_I2C_ADDRESS, 3);

  // Wait for data to become available
  // always use time out in loop to avoid
  // potential lockup (here 90ms (6*15ms))
  tmp = 0;
  while( Wire.available()<3 && tmp++ < 9) {
    sleepQuickWake(WDTO_15MS);
  }

  // read raw value
  raw  = ((uint16_t) Wire.read()) << 8;
  raw |= Wire.read();
  checksum = Wire.read();

  // Check CRC of data received
  if(si7021_checkCRC(raw, checksum) != 0)
   return -1; 

  if (datatype == ULPN_SI7021_READ_HUM) {
    // Convert value to Himidity percent 
    data = ((125 * (long)raw) >> 16) - 6;

    // Datasheet says doing this check
    if (data>100) data = 100;
    if (data<0)   data = 0;

    // save value
    _si_humidity = (uint8_t) data;

  } else {
    // Convert value to Temperature (*100)
    // for 23.45C value will be 2345
    data =  ((17572 * (long)raw) >> 16) - 4685;

    // save value
    _si_temperature = (int16_t) data;
  }

  return 0;
}


/* ======================================================================
Function: si7021_readValues
Purpose : read temperature and humidity from SI7021 sensor
Input   : -
Output  : 0 if okay
Comments: -
====================================================================== */
int8_t ULPNode::si7021_readValues(void)
{
  int8_t error = 0;

  // start humidity conversion
  error |= si7021_StartConv(ULPN_SI7021_READ_HUM);

  // start temperature conversion
  error |= si7021_StartConv(ULPN_SI7021_READ_TEMP);

  return error;
}

/* ======================================================================
Function: reset
Purpose : Soft Reset SI7021 sensor
Input   : -
Output  : 0 if okay
Comments: 
====================================================================== */
int8_t ULPNode::si7021_reset(void) 
{
  uint8_t error = si7021_writeValue(ULPN_SI7021_SOFT_RESET);
  if (error==0) {
    sleepQuickWake(WDTO_60MS);
  }
  return (error); 
}

/* ======================================================================
Function: setResolution
Purpose : Sets the sensor resolution to one of four levels 
Input   : see #define is .h file, default is ULPN_SI7021_RESOLUTION_14T_12RH
Output  : temperature or humidity
Comments: 0 if okay
====================================================================== */
int8_t ULPNode::si7021_setResolution(uint8_t res)
{
  uint8_t reg;
  uint8_t error;

  // Just in case of bad parameter
  res &= ~ULPN_SI7021_RESOLUTION_MASK;

  // Get the current register value
  error = si7021_readRegister(ULPN_SI7021_READ_REG, &reg);
  if ( error == 0) {
    // remove current resolution bits from current 
    reg &= ULPN_SI7021_RESOLUTION_MASK ; 
    // Set the new ones
    error = si7021_writeRegister(ULPN_SI7021_WRITE_REG, reg | res );
  } 
  return error;
}


/* ======================================================================
Function: si7021_checkCRC
Purpose : check the CRC of received data
Input   : value read from sensor
Output  : CRC read from sensor
Comments: 0 if okay
====================================================================== */
uint8_t ULPNode::si7021_checkCRC(uint16_t data, uint8_t check)
{
  uint32_t remainder, divisor;

  //Pad with 8 bits because we have to add in the check value
  remainder = (uint32_t)data << 8; 

  // From: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
  // POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1 : http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
  // 0x988000 is the 0x0131 polynomial shifted to farthest left of three bytes
  divisor = (uint32_t) 0x988000;

  // Add the check value
  remainder |= check; 

  // Operate on only 16 positions of max 24. 
  // The remaining 8 are our remainder and should be zero when we're done.
  for (uint8_t i = 0 ; i < 16 ; i++) {
    //Check if there is a one in the left position
    if( remainder & (uint32_t)1<<(23 - i) ) {
      remainder ^= divisor;
    }

    //Rotate the divisor max 16 times so that we have 8 bits left of a remainder
    divisor >>= 1; 
  }
  return ((uint8_t) remainder);
}

/* ======================================================================
Function: tsl2561_calcLux
Purpose : start a conversion and return calculated lux
Input   : -
Output  : 0 if calculated value ok and updated 
Comments: global lux value is updated
====================================================================== */
int8_t ULPNode::tsl2561_calcLux(void)
{
  unsigned long chScale;
  unsigned long channel0, channel1;
  unsigned long ratio, ratio1;
  unsigned long lux;
  unsigned int b, m;
  uint16_t ch0,ch1;
  uint8_t msb, lsb;
  uint8_t err = 0;

  // Power UP device
  tsl2561_writeRegister(ULPN_TSL2561_CONTROL, ULPN_TSL2561_POWER_UP); 

  // I noticed 1st calculation after power up could be hazardous; so
  // do a 1st dummy reading, with speed integration time, here 13ms
  tsl2561_writeRegister(ULPN_TSL2561_TIMING, ULPN_TSL2561_TIMING_13MS);  
  sleepQuickWake(WDTO_15MS);  

  // 101 ms is the best integration time I found 
  // (13ms sometime got saturation on full light, so I increased it has says datasheet
  // since Watchdog sleep does not have default 101ms (120 or 60)
  // we'll use 120ms watchdog duration
  tsl2561_writeRegister(ULPN_TSL2561_TIMING, ULPN_TSL2561_TIMING_CUSTOM_START);  
  sleepQuickWake(WDTO_120MS);  
  tsl2561_writeRegister(ULPN_TSL2561_TIMING, ULPN_TSL2561_TIMING_CUSTOM_STOP);  // Stop integration

  // don't try to change reading order of LOW/HIGH, it will not work !!!!
  // you must read LOW then HIGH
  err |= tsl2561_readRegister(ULPN_TSL2561_CHANNEL_0L, &lsb);
  err |= tsl2561_readRegister(ULPN_TSL2561_CHANNEL_0H, &msb);
  ch0 = word(msb,lsb);
  err |= tsl2561_readRegister(ULPN_TSL2561_CHANNEL_1L, &lsb);
  err |= tsl2561_readRegister(ULPN_TSL2561_CHANNEL_1H, &msb);
  ch1 = word(msb,lsb);;

  // put the device into power down mode
  tsl2561_writeRegister(ULPN_TSL2561_CONTROL, ULPN_TSL2561_POWER_DOWN); 

  // I2C error ?
  if( err ) {
    return -2; 
  }

  // ch0 out of range, but ch1 not. the lux is not valid in this situation.
  if( ch0/ch1 < 2 && ch0 > 4900) {
    return -1;  
  }

  chScale = ULPN_TSL2561_CHSCALE_TINT_120MS ;
  // gain is 1 so put it to 16X
  chScale <<= 4;
  
  // scale the channel values
  channel0 = (ch0 * chScale) >> ULPN_TSL2561_CH_SCALE;
  channel1 = (ch1 * chScale) >> ULPN_TSL2561_CH_SCALE;

  ratio1 = 0;
  if (channel0!= 0) {
    ratio1 = (channel1 << (ULPN_TSL2561_RATIO_SCALE+1))/channel0;
  }
  
  // round the ratio value
  ratio = (ratio1 + 1) >> 1;

  // ULPNode have T package
  // Adjust constant depending on calculated ratio
  if ((ratio >= 0) && (ratio <= ULPN_TSL2561_K1T)) {
    b=ULPN_TSL2561_B1T; 
    m=ULPN_TSL2561_M1T;
  } else if (ratio <= ULPN_TSL2561_K2T) {
    b=ULPN_TSL2561_B2T; 
    m=ULPN_TSL2561_M2T;
  } else if (ratio <= ULPN_TSL2561_K3T) {
    b=ULPN_TSL2561_B3T; 
    m=ULPN_TSL2561_M3T;
  } else if (ratio <= ULPN_TSL2561_K4T) {
    b=ULPN_TSL2561_B4T; 
    m=ULPN_TSL2561_M4T;
  } else if (ratio <= ULPN_TSL2561_K5T) {
    b=ULPN_TSL2561_B5T; 
    m=ULPN_TSL2561_M5T;
  } else if (ratio <= ULPN_TSL2561_K6T) {
    b=ULPN_TSL2561_B6T; 
    m=ULPN_TSL2561_M6T;
  } else if (ratio <= ULPN_TSL2561_K7T) {
    b=ULPN_TSL2561_B7T; 
    m=ULPN_TSL2561_M7T;
  } else if (ratio > ULPN_TSL2561_K8T) {
    b=ULPN_TSL2561_B8T; 
    m=ULPN_TSL2561_M8T;
  }

  // datasheet formula
  lux=((channel0*b)-(channel1*m));
  
  // do not allow negative lux value
  if(lux<0) {
    lux=0;
  }
  
  // round lsb (2^(LUX_SCALE−1))
  lux += (1<<(ULPN_TSL2561_LUX_SCALE-1));
  
  // strip off fractional portion
  lux >>= ULPN_TSL2561_LUX_SCALE;

  // strip off fractional portion
  _tsl_lux = (uint16_t) (lux);

  return 0;
}



/* ======================================================================
Function: ccs811_begin
Purpose : check and start CCS811
Input   : -
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::ccs811_begin()
{
  uint8_t status;

/*
  // reset sequence from the datasheet
  uint8_t seq[] = {0x11, 0xE5, 0x72, 0x8A};
  
  this->write(CCS811_SW_RESET, seq, 4);

  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t *)seq, 4);
  Wire.endTransmission();



  // from datasheet - up to 70ms on the first Reset after new application download
  sleepQuickWake(WDTO_120MS);  
*/
  // Read Hardware ID
  if ( ccs811_readRegister(ULPN_CSS811_HW_ID, &status) == 0 ) {

    // this is the expected hardware ID
    if( status != 0x81) {
      #if DEBUG_ULPN > 0
      DebugF("CCS811 bad ID=0x");
      Debugln2(status, HEX);
      #endif
      return 1;
    }

    // Read status
    if ( ccs811_readStatus(&status) == 0 ) {
      // APP_VALID should be set
      if ( (status & 0x10) == 0 ) {
        #if DEBUG_ULPN > 0
        DebuglnF("CCS811 no APP_VALID");
        #endif
        return 1; 
      }

      // Start the application
      Wire.beginTransmission(ULPN_CCS811_I2C_ADDRESS);
      Wire.write(ULPN_CSS811_APP_START);
      Wire.endTransmission();

      // Read status
      if ( ccs811_readStatus(&status) == 0 ) {
        // FW_MODE should be set (Firmware is in application mode)
        if ( (status & 0x80) == 0 ) {
          #if DEBUG_ULPN > 0
          DebuglnF("CCS811 boot mode");
          #endif
          return 1;
        }

        // Measurment every seconds, no interrupts
        if ( ccs811_writeRegister(ULPN_CSS811_MEAS_MODE, 0x10) == 0 ) {
          return 0;
        }
      }
    }
  } else {
    #if DEBUG_ULPN > 0
    DebuglnF("CCS811 Error");
    #endif
  }
  return 1;
}


/* ======================================================================
Function: ccs811_readStatus
Purpose : read and check CCS811 status register
Input   : pointer where to store status value
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::ccs811_readStatus(uint8_t * status)
{
  Wire.beginTransmission(ULPN_CCS811_I2C_ADDRESS);
  Wire.write(ULPN_CSS811_STATUS);
  // all was fine ?
  if ( Wire.endTransmission()==0 ) {
    // request 1 byte and have it ?
    if (Wire.requestFrom(ULPN_CCS811_I2C_ADDRESS, 1)==1) {
      // return value
      *status = Wire.read();

      // Check if error on status register
      if ( (*status & 0x01) ==0 ) {
        return 0;
      } else {
        #if DEBUG_ULPN > 0
        uint8_t errorID;
        ccs811_readRegister(ULPN_CSS811_ERROR_ID, &errorID);
        DebugF("ErrorID=");
        Debugln(errorID);
        #endif
      }
    }
  }
  return 1;
}

/* ======================================================================
Function: ccs811_dataReady
Purpose : read status register and check data is ready
Input   : -
Output  : 1 if data is ready
Comments: 
====================================================================== */
uint8_t ULPNode::ccs811_dataReady(void)
{
  uint8_t status;
  if ( ccs811_readStatus(&status) == 0 ) {
    if (status & 0x08 ) {
      return 1;
    }
  }
  return 0;
}

/* ======================================================================
Function: ccs811_sleep
Purpose : put device into sleep mode
Input   : -
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::ccs811_sleep(void)
{
  // Measurment in full idle mode
  return ccs811_writeRegister(ULPN_CSS811_MEAS_MODE, 0x00);
}

/* ======================================================================
Function: getValues
Purpose : read sensor data
Input   : -
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::ccs811_getValues(void)
{
  Wire.beginTransmission(ULPN_CCS811_I2C_ADDRESS);
  Wire.write(ULPN_CSS811_ALG_RESULT_DATA);         
  // all was fine ?
  if ( Wire.endTransmission()==0 ) {
    // request 4 bytes and have it ?
    if ( Wire.requestFrom(ULPN_CCS811_I2C_ADDRESS, 4)==4 ) {
      _co2 = Wire.read() << 8;
      _co2|= Wire.read();
      _tvoc = Wire.read() << 8;
      _tvoc|= Wire.read();

      // return value
      return 0;
    }
  }
  return 1;
}

/* ======================================================================
Function: ccs811_compensate
Purpose : compensate for temperature and relative humidity
Input   : -
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::ccs811_compensate(int16_t _temp, uint8_t _rh)  
{
  /*
  int _temp, _rh;
  if(t>0)
    _temp = (int)t + 0.5;  // this will round off the floating point to the nearest integer value
  else if(t<0) // account for negative temperatures
    _temp = (int)t - 0.5;
  _temp = _temp + 25;  // temperature high byte is stored as T+25°C in the sensor's memory so the value of byte is positive
  _rh = (int)rh + 0.5;  // this will round off the floating point to the nearest integer value
*/

  uint8_t data[4];

  data[0] = _rh << 1;  // shift the binary number to left by 1. This is stored as a 7-bit value
  data[1] = 0;  // most significant fractional bit. Using 0 here - gives us accuracy of +/-1%. Current firmware (2016) only supports fractional increments of 0.5
  data[2] = _temp << 1;
  data[3] = 0;

  Wire.beginTransmission(ULPN_CCS811_I2C_ADDRESS);
  Wire.write(ULPN_CSS811_ENV_DATA);
  Wire.write(data[0]); // 7 bit humidity value
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
  return ( Wire.endTransmission() );
}

/* ======================================================================
Function: bme280_readCoefficients
Purpose : read coefficient from device and set them to RAM
Input   : -
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::bme280_readCoefficients(void)
{
  uint8_t err = 0;
  uint8_t status;

  // be sure NVM data are being copied to image registers 
  do {
    sleepQuickWake(WDTO_15MS);
    err = bme280_readRegister(ULPN_BME280_STAT_REG, &status);
  }
  while ( (err==0) && ( (status & 1) != 0) );

  if (err ) {
    return err | 0x10 ;
  }


  err |= bme280_readRegister16_LE(ULPN_BME280_REGISTER_DIG_T1, &_bme_calib.dig_T1 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_T2, &_bme_calib.dig_T2 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_T3, &_bme_calib.dig_T3 );

  if (err) {
    // Indicate I2C error in Tx reading
    return 0x20 | err;
  }

  err |= bme280_readRegister16_LE(ULPN_BME280_REGISTER_DIG_P1, &_bme_calib.dig_P1 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_P2, &_bme_calib.dig_P2 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_P3, &_bme_calib.dig_P3 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_P4, &_bme_calib.dig_P4 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_P5, &_bme_calib.dig_P5 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_P6, &_bme_calib.dig_P6 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_P7, &_bme_calib.dig_P7 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_P8, &_bme_calib.dig_P8 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_P9, &_bme_calib.dig_P9 );

  if (err) {
    // Indicate I2C error in Px reading
    return 0x30 | err;
  }

  err |= bme280_readRegister(ULPN_BME280_REGISTER_DIG_H1, &_bme_calib.dig_H1 );
  err |= bme280_readRegister16_S_LE(ULPN_BME280_REGISTER_DIG_H2, &_bme_calib.dig_H2 );
  err |= bme280_readRegister(ULPN_BME280_REGISTER_DIG_H3, &_bme_calib.dig_H3 );

  uint8_t msb, lsb;
  err |= bme280_readRegister(ULPN_BME280_REGISTER_DIG_H4+0, &msb );
  err |= bme280_readRegister(ULPN_BME280_REGISTER_DIG_H4+1, &lsb );
  _bme_calib.dig_H4 = (msb << 4) | (lsb & 0x0F);

  err |= bme280_readRegister(ULPN_BME280_REGISTER_DIG_H5+0, &msb );
  err |= bme280_readRegister(ULPN_BME280_REGISTER_DIG_H5+1, &lsb );
  _bme_calib.dig_H5 = (msb >> 4) | (lsb << 4) ;

  err |= bme280_readRegister(ULPN_BME280_REGISTER_DIG_H6, &_bme_calib.dig_H6 );

  if (err) {
    // Indicate I2C error in Hx reading
    err |= 0x40;
  }

  return err;
}

/* ======================================================================
Function: bme280_begin
Purpose : check and start BME280
Input   : -
Output  : 0 if okay
Comments: 
====================================================================== */
uint8_t ULPNode::bme280_begin(void)
{
  uint8_t err;
  uint8_t data;

  // Check CHIP ID
  err = bme280_readRegister(ULPN_BME280_CHIP_ID_REG, &data);
  if (err || data != 0x60 ) {
    return err + 0x50 ;
  }

  // reset the device using soft-reset
  // this makes sure the IIR is off, etc.
  bme280_writeRegister(ULPN_BME280_RST_REG, 0xB6);

   // Ok now we can read coefficients
  err = bme280_readCoefficients();
  if (err) {
    return err;
  }

  // Set the oversampling control words.
  // config will only be writeable in sleep mode, so first insure that.
  bme280_writeRegister(ULPN_BME280_CTRL_MEAS_REG, ULPN_BME280_MODE_SLEEP);
  
  // Set the config word (standby unused in sleep mode)
  data  = ULPN_BME280_STANDBY_MS_1000 << 5;
  data |= ULPN_BME280_FILTER_OFF      << 1;
  bme280_writeRegister(ULPN_BME280_CONFIG_REG, data);
  
  // Set ctrl_hum first, then ctrl_meas to activate ctrl_hum
  data = ULPN_BME280_SAMPLING_X1 ; 
  bme280_writeRegister(ULPN_BME280_CTRL_HUMIDITY_REG, data);
  
  // set ctrl_meas
  // First, set temp oversampling
  data  = ULPN_BME280_SAMPLING_X1 << 5 ;
  // Next, pressure oversampling
  data |= ULPN_BME280_SAMPLING_X1 << 2 ;
  // Last, set mode (forced, do a measure then sleep)
  data |= ULPN_BME280_MODE_FORCED;
  // Load the byte
  bme280_writeRegister(ULPN_BME280_CTRL_MEAS_REG, data);

  return err;
}

/* ======================================================================
Function: bme280_readTemperature
Purpose : read latest temperature measurment
Input   : -
Output  : 0 if okay
Comments: library bme temperature var is set on output
====================================================================== */
uint8_t ULPNode::bme280_readTemperature(void)
{
  int32_t var1, var2;
  uint8_t err;
  uint32_t adc_T;

  err = bme280_readRegister24(ULPN_BME280_TEMPERATURE_REG, (uint32_t *)&adc_T);

  // value in case temp measurement was disabled 
  if (err || adc_T == 0x800000) {
    return err | 0x80;
  }

  adc_T >>= 4;

  var1 = ((((adc_T>>3) - ((int32_t)_bme_calib.dig_T1 <<1))) * 
                         ((int32_t)_bme_calib.dig_T2)) >> 11;

  var2 = (((((adc_T>>4) - ((int32_t)_bme_calib.dig_T1)) *
                          ((adc_T>>4) - ((int32_t)_bme_calib.dig_T1))) >> 12) *
                          ((int32_t)_bme_calib.dig_T3)) >> 14;

  _bme_t_fine = var1 + var2;

  var1 = (_bme_t_fine * 5 + 128) >> 8;
  _bme_temperature = (int16_t) var1;

  return 0;
}


/* ======================================================================
Function: bme280_readPressure
Purpose : read latest temperature measurment
Input   : -
Output  : 0 if okay
Comments: library pressure temperature var is set on output
====================================================================== */
uint8_t ULPNode::bme280_readPressure( void )
{
  // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
  // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
  //int32_t adc_P = ((uint32_t)readRegister(BME280_PRESSURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_PRESSURE_LSB_REG) << 4) | ((readRegister(BME280_PRESSURE_XLSB_REG) >> 4) & 0x0F);
  int64_t var1, var2, p;
  int32_t adc_P;
  uint8_t err ;

  //err = readTemperature(); // must be done first to get t_fine

  err = bme280_readRegister24(ULPN_BME280_PRESSURE_REG, (uint32_t *) &adc_P);
  if (err || adc_P == 0x800000 ) {
    // value in case pressure measurement was disabled
    return err | 0x080; 
  }

  adc_P >>= 4;

  var1 = ((int64_t)_bme_t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bme_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bme_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bme_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bme_calib.dig_P3)>>8) +
  ((var1 * (int64_t)_bme_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bme_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0x070; // avoid exception caused by division by zero
  }

  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bme_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bme_calib.dig_P8) * p) >> 19;

  p =((p + var1 + var2) >> 8) + (((int64_t)_bme_calib.dig_P7)<<4);
  p /= 256;

  // return 10245 => 1024.5
  p /= 10;
  _bme_pressure = (uint16_t) p;

  return 0;
}

/* ======================================================================
Function: bme280_readHumidity
Purpose : read latest humidity measurment
Input   : -
Output  : 0 if okay
Comments: library humidity var is set on output
====================================================================== */
uint8_t ULPNode::bme280_readHumidity( void )
{
  int16_t adc;
  uint8_t err;
  
  //err = readTemperature(); // must be done first to get t_fine
  err= bme280_readRegister16(ULPN_BME280_HUMIDITY_REG, (int16_t *)&adc);
  if ( err || adc == 0x8000) {
    // value in case humidity measurement was disabled
    return err |= 0x80;
  }

  int32_t adc_H = adc;
  int32_t v_x1_u32r;

  v_x1_u32r = (_bme_t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme_calib.dig_H4) << 20) -
              (((int32_t)_bme_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
              (((((((v_x1_u32r * ((int32_t)_bme_calib.dig_H6)) >> 10) *
              (((v_x1_u32r * ((int32_t)_bme_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
              ((int32_t)2097152)) * ((int32_t)_bme_calib.dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
              ((int32_t)_bme_calib.dig_H1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  int32_t h = (v_x1_u32r>>12) / 1024;

  _bme_humidity = (uint8_t) h;

  return 0;
}

bool ULPNode::bme280_readValues(bool wait) 
{
  uint8_t err;
  uint8_t data;

  // we are in sleep mode, the BME sensor goes back to sleep after each
  // measurement and we need to set it to forced mode once at this point, so
  // it will take the next measurement and then return to sleep again.

  // set to forced mode, i.e. "take next measurement"
  err = bme280_readRegister(ULPN_BME280_CTRL_MEAS_REG, &data);

  if (err==0) {
    // remove mode bits
    data &= 0xFC;
    data |= ULPN_BME280_MODE_FORCED;
    err = bme280_writeRegister(ULPN_BME280_CTRL_MEAS_REG, data);

    // wait until measurement has been completed, otherwise we would read
    // the values from the last measurement
    if (wait) {
      do {
        sleepQuickWake(WDTO_15MS);
        err = bme280_readRegister(ULPN_BME280_STAT_REG, &data);
      }
      while ( (err==0) && ( (data & 0x08) != 0) );
    }

    if (err==0) {
      // must be done first to get t_fine
      err |= bme280_readTemperature(); 
      err |= bme280_readHumidity(); 
      err |= bme280_readPressure(); 
    }
  }
  return err;
}

/*
void ULPNode::bme280_showCalib(void) 
{
  DebuglnF("Displaying calibration");
  DebugF("dig_T1, uint16: ");
  Debugln(_bme_calib.dig_T1);
  DebugF("dig_T2, int16: ");
  Debugln(_bme_calib.dig_T2);
  DebugF("dig_T3, int16: ");
  Debugln(_bme_calib.dig_T3);
  
  DebugF("dig_P1, uint16: ");
  Debugln(_bme_calib.dig_P1);
  DebugF("dig_P2, int16: ");
  Debugln(_bme_calib.dig_P2);
  DebugF("dig_P3, int16: ");
  Debugln(_bme_calib.dig_P3);
  DebugF("dig_P4, int16: ");
  Debugln(_bme_calib.dig_P4);
  DebugF("dig_P5, int16: ");
  Debugln(_bme_calib.dig_P5);
  DebugF("dig_P6, int16: ");
  Debugln(_bme_calib.dig_P6);
  DebugF("dig_P7, int16: ");
  Debugln(_bme_calib.dig_P7);
  DebugF("dig_P8, int16: ");
  Debugln(_bme_calib.dig_P8);
  DebugF("dig_P9, int16: ");
  Debugln(_bme_calib.dig_P9);
  
  DebugF("dig_H1, uint8: ");
  Debugln(_bme_calib.dig_H1);
  DebugF("dig_H2, int16: ");
  Debugln(_bme_calib.dig_H2);
  DebugF("dig_H3, uint8: ");
  Debugln(_bme_calib.dig_H3);
  DebugF("dig_H4, int16: ");
  Debugln(_bme_calib.dig_H4);
  DebugF("dig_H5, int16: ");
  Debugln(_bme_calib.dig_H5);
  DebugF("dig_H6, uint8: ");
  Debugln(_bme_calib.dig_H6);
}
*/