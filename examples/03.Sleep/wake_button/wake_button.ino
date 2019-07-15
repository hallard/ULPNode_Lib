// **********************************************************************************
// ULPNode onboard RGB blink waked by voltage controller, booster, switch activation
// **********************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this code but please abide with the CC-BY-SA license:
// Attribution-NonCommercial-ShareAlike 4.0 International License
// http://creativecommons.org/licenses/by-nc-sa/4.0/
//
// For any explanation about ULPNode, see my blog
// https://hallard.me/category/ulpnode/
//
// This program works with the ULPNode board
// see schematic here https://github.com/hallard/ULPNode
//
// Written by Charles-Henri Hallard (https://hallard.me)
//
// History : V1.00 2014-07-14 - First release
//
// All text above must be included in any redistribution.
//
// **********************************************************************************
//
// This example blink the ULPNode onboard LED every time the button is pressed 
// If the board as also RGB Led, this will blink alsp
//
// **********************************************************************************
#include <arduino.h>
#include <Wire.h>
#include <ULPNode.h>

// Remember we can be at 4MHz so serial can't do better than 38400 
// Since Arduino 1.5, now they added the 250KBPS speed great !!!
//#define SERIAL_PORT_SPEED 250000
#define SERIAL_PORT_SPEED 115200

// define an ULPNode object 
ULPNode  ulpn;

// Some counter demo used in IRQ
volatile uint32_t iSwitchCounter = 0 ;
volatile uint8_t  iIrq=0;

/* ======================================================================
Function: switchInterruptHandler
Purpose : IRQ Handler called when switch is pressed/released (for wake)
Input   : - 
Output  : - 
Comments: once fired this interrupt disable itself
====================================================================== */
void switchInterruptHandler(void)
{
  // Inc counter and set flag for main loop
  iSwitchCounter++;
  iIrq |= SLEEP_WAKE_SWITCH;
}

/* ======================================================================
Function: setup
Purpose : try to guess 
Input   : -
Output  : - 
Comments: -
====================================================================== */
void setup()
{
  // Init ULPNode I/O
  ulpn.init();

  // Define IRQ callbacks we need in "user space"
  // Here we want callbacks of 
  // watchdog, wake booster and switch push button push
  ulpn.attachSwitchInterrupt( switchInterruptHandler ); 

  // Give power to sensors (and so WS2812 RGB LED)
  // And disable powering RF Module just in case
  ulpn.setDevice(DEVICE_SENSORS_ON | DEVICE_RF_OFF | DEVICE_LED_OFF);

  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println();
  Serial.println(F("==========================="));
  Serial.println(F("ULPNode wake by button test"));
  Serial.flush();

  // Full brigtness 
  ulpn.RGBSetBrightness(RGB_FULL_BRIGHTNESS);
}

/* ======================================================================
Function: loop
Purpose : main loop
Input   : -
Output  : - 
Comments: -
====================================================================== */
void loop() 
{
  uint32_t SwitchCounter;
  uint8_t  IrqTrigger;
  bool     isError = false;

  // action to be done with button, default none
  btn_action_e SwitchAction = BTN_NONE;
  
  // this is our Last talk before sleeping
  Serial.print(F("Zzzz...")); 
  Serial.flush();
  Serial.end();
  
  // Light off LEDs, remove power from sensors and RF modules
  ulpn.RGBShow(RGB_OFF);
  ulpn.setDevice(DEVICE_LED_OFF | DEVICE_SENSORS_OFF | DEVICE_RF_OFF);

  // Disable all CPU peripherals for low power
  ulpn.disableCPUDevices();

  // go to sleep mode, NO BOD, only push button button
  ulpn.sleepDeviceWake(SLEEP_BOD_OFF | SLEEP_WAKE_SWITCH, 0);
  
  // We've been waked up by a IRQ, disable ours until we done the job
  ulpn.setIRQ(IRQ_SWITCH_DISABLE);

  // Ok now we need to be sure that we never go in a dead loop
  // So we activate watchdog to autoreset the board in case of
  // this, let's say we have 2 seconds max doing our JOB !!!
  #ifdef APP_WATCHDOG_TO
  wdt_enable(APP_WATCHDOG_TO);
  wdt_reset();
  #endif

  // Enable back Arduino core functions working such as delay() or PWM
  power_timer0_enable(); 

  // IRQ are disabled, it's safe to use these vars
  SwitchCounter   = iSwitchCounter;
  IrqTrigger      = iIrq;
 
  // Enable also sensors, RGB LED and onboard LED 
  ulpn.setDevice( DEVICE_SENSORS_ON | DEVICE_LED_ON );

  // Activate back Serial
  power_usart0_enable();
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.print(F("Awake by "));  

  // loop to be sure to grab all pending IRQ
  while(IrqTrigger)
  {
    // Waked by push button
    if (IrqTrigger & SLEEP_WAKE_SWITCH )
    {

      // Ack this IRQ
      IrqTrigger &= ~SLEEP_WAKE_SWITCH;

      Serial.print(F("Switch ")); 
    }


    // Another Wake type ? 
    // Houston we have a problem !!!
    if ( IrqTrigger )
    {
      Serial.print(F("?!?!?! ")); 
      
      // ACK all IRQ, fresh reset
      IrqTrigger = 0;

      isError = true;
    }

    // Display information
    Serial.print(F(" Push counter=")); 
    Serial.print(SwitchCounter); 
    Serial.print(F(" VBat=")); 
    Serial.print(ulpn.VBatAverage()); 
    Serial.println(F(" mV")); 
  }

  Serial.flush();

  // This point is managed by ULPNode library, but need to be user refreshed
  // you don't need to do this each wake up, but twice a day sounds good
  ulpn.readVcc();
  ulpn.readVBatAverage();

  // Ligh on onboard LED
  ulpn.setDevice(DEVICE_LED_ON);

  // Blink RGB Led
  if (isError) {
    ulpn.RGBBlink(1, RGB_RED, WDTO_120MS);
    isError = false;
  } else {
    ulpn.RGBBlink(1, RGB_GREEN, WDTO_120MS);
  }
  ulpn.LEDBlink(1, WDTO_120MS);

  // Ligh off onboard LED
  ulpn.setDevice(DEVICE_LED_OFF);

  cli();
  iIrq=0;
  sei();

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}