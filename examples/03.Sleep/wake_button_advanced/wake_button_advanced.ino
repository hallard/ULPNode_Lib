// **********************************************************************************
// ULPNode advanced switch push button functons
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
// This example blink the ULPNode onboard LED every time button is pushed or watchdog
// wake it. You can totally avoid watchdog wake using feature on push button
// this example is an advanced example on how to manage differents features with
// one one button
//
// Low Power is done with our without watchdog (best Low Power mode), 
// Once waked node light led then go to sleep for 120ms before light it down again
// - Blink blue if waked by watchdog
// - Blink Green if waked by button
// - Blink yellow if push was too short or too long
//
// you can also wake the node and changing action by pushing the switch push button
// the press duration will change action and wake mode
// when you release the button, the actions follow :
// between 0 and 1 second (normal push) led blink green
// longer push, led start blinking changing color each second
//   - released between 1s and 2s (purple) => disable watchdog (only button wake)
//   - released between 2s and 3s (blue)   => wake by watchdog every 2s
//   - released between 3s and 4s (cyan)   => wake by watchdog every 4s
//   - released between 4s and 5s (green)  => wake by watchdog every 8s
//   - released between 5s and 6s (yellow) => do nothing
//   - released between 6s and 7s (red)    => do nothing
//
// **********************************************************************************
#include <arduino.h>
#include <Wire.h>
#include <ULPNode.h>

// Remember we can be at 4MHz so serial can't do better than 38400 
// Since Arduino 1.5, now they added the 250KBPS speed great !!!
//#define SERIAL_PORT_SPEED 250000
#define SERIAL_PORT_SPEED 115200

#define WAKED_BY_ERROR     0
#define WAKED_BY_WATCHDOG  1
#define WAKED_BY_SWITCH    2
#define WAKED_BY_BADSWITCH 3

// Some counter demo used in IRQ
volatile uint32_t iSwitchCounter = 0 ;
volatile uint32_t iWatchdogCounter = 0 ;
volatile uint8_t  iIrq=0;

// define an ULPNode object 
ULPNode  ulpn;

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
Function: watchdogInterruptHandler
Purpose : IRQ Handler called when watchdog IRQ occurs
Input   : - 
Output  : - 
Comments: once fired this interrupt disable the watchdog
====================================================================== */
void watchdogInterruptHandler(void) 
{
  // Inc counter and set flag for main loop
  iWatchdogCounter++;
  iIrq |= SLEEP_WAKE_WATCHDOG ;
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
  // watchdog, and switch push button push
  // We use this to know who waked us
  ulpn.attachSwitchInterrupt( switchInterruptHandler ); 
  ulpn.attachWatchdogInterrupt( watchdogInterruptHandler ); 

  // Give power to sensors (and so WS2812 RGB LED)
  // And disable powering RF Module just in case
  ulpn.setDevice(DEVICE_SENSORS_ON | DEVICE_RF_OFF | DEVICE_LED_OFF);

  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println();
  Serial.println(F("==============================="));
  Serial.println(F("ULPNode advance wake buton test"));
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
  uint32_t WatchdogCounter;
  uint8_t  IrqTrigger;
  uint8_t wakedBy = WAKED_BY_ERROR;

  // Used for this demo
  static uint8_t my_sleep_wdt_period = 0;

  // action to be done with button, default none
  btn_action_e SwitchAction = BTN_NONE;

  // this is our Last talk before sleeping
  Serial.print(F("Zzzz...")); 
  Serial.flush();
  Serial.end();
  
  // Light off LEDs, remove power from sensors and RF modules
  ulpn.RGBShow(RGB_OFF);
  ulpn.setDevice(DEVICE_LED_OFF | DEVICE_SENSORS_OFF);

  // Disable all CPU peripherals for low power
  ulpn.disableCPUDevices();

  // go to sleep mode, NO BOD, disable booster but allow it to wake us such as the switch button
  if (my_sleep_wdt_period==0)
  {
    ulpn.sleepDeviceWake( SLEEP_BOD_OFF | SLEEP_WAKE_SWITCH);
  }
  else 
  {
    ulpn.sleepDeviceWake( SLEEP_BOD_OFF | SLEEP_WAKE_SWITCH | SLEEP_WAKE_WATCHDOG, my_sleep_wdt_period);
  }

  // We've been waked up by a IRQ, disable ours until we done the job
  ulpn.setIRQ(IRQ_SWITCH_DISABLE);

  // Enable back Arduino core functions working such as delay() or PWM
  power_timer0_enable(); 

  // IRQ are disabled, it's safe to use these vars
  SwitchCounter   = iSwitchCounter;
  WatchdogCounter = iWatchdogCounter;
  IrqTrigger      = iIrq;
  
  // Enable also sensors, and onboard LED also
  ulpn.setDevice(DEVICE_SENSORS_ON | DEVICE_LED_ON );

  // Activate back Serial
  power_usart0_enable();
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.print(F("Awake by "));  
  Serial.flush();

  // loop to be sure to grab all pending IRQ
  while(IrqTrigger)
  {
    
    // Waked by push button
    if (IrqTrigger & SLEEP_WAKE_SWITCH )
    {
      // Get switch port state 
      uint8_t button_port = digitalRead(SWITCH_PIN);

      // Ack this IRQ
      IrqTrigger &= ~SLEEP_WAKE_SWITCH;

      Serial.print(F("Switch ")); 

      // Button pressed 
      if (button_port==BTN_PRESSED)
      {
        btn_state_e btn_state;
        Serial.print(F("pushed ")); 
        Serial.flush();

        // we enter into the loop to manage
        // the function that will be done
        // depending on button press duration
        do
        {
          // keep watching the push button:
          btn_state = ulpn.buttonManageState(button_port);

          if (btn_state == BTN_WAIT_LONG_RELEASE) {
            ulpn.setDevice(DEVICE_LED_OFF);
          }

          // read new state button
          button_port = digitalRead(SWITCH_PIN);

          // Pat the dog, this loop can be as long 
          // as button is pressed
          wdt_reset();
        }
        // we loop until button state machine finished
        while (btn_state != BTN_WAIT_PUSH);

        // Get and save action we need to do after button analyze
        SwitchAction = ulpn.buttonAction();
      }
      else
      {
        // should never happen
        Serial.print(F("released ")); 
      }
    }

    // Waked by watchdog
    if (IrqTrigger & SLEEP_WAKE_WATCHDOG )
    {
      // Ack this IRQ
      IrqTrigger &= ~SLEEP_WAKE_WATCHDOG;
      Serial.print(F("Watchdog ")); 
      wakedBy = WAKED_BY_WATCHDOG;
    }
    
    // Another Wake type ? 
    // Houston we have a problem !!!
    if ( IrqTrigger )
    {
      Serial.print(F("?!?!?! ")); 
      
      // ACK all IRQ, fresh reset
      IrqTrigger = 0;
    }

    // Display information
    Serial.print(F(" Switch=")); 
    Serial.print(SwitchCounter); 
    Serial.print(F(" Watchdog=")); 
    Serial.println(WatchdogCounter); 
    Serial.flush();

    // Do the switch press invoke us an action ?
    if (SwitchAction != BTN_NONE)
    {
      wakedBy = WAKED_BY_SWITCH;

      Serial.print(F("Switch Action=")); 
      Serial.print(SwitchAction); 
      Serial.print(F(" =>")); 
      if (SwitchAction==BTN_BAD_PRESS) {
        Serial.println(F("Bad Press")); 
        wakedBy = WAKED_BY_BADSWITCH;
      }
      if (SwitchAction==BTN_QUICK_PRESS) {
        Serial.println(F("Quick Press")); 
      }
      if (SwitchAction==BTN_TIMEOUT) {
        Serial.println(F("Timeout")); 
        wakedBy = WAKED_BY_BADSWITCH;
      }

      if (SwitchAction==BTN_PRESSED_12) 
      {
        Serial.println(F("12 Button wake only")); 
        // Remove watchdog sleep
        my_sleep_wdt_period = 0;
      }
      if (SwitchAction==BTN_PRESSED_23) 
      {
        Serial.println(F("23 Adding Watchdog sleep every 2s")); 
        // Set watchdog sleep
        my_sleep_wdt_period = WDTO_2S;
      }
      if (SwitchAction==BTN_PRESSED_34) 
      {
        Serial.println(F("34 Adding Watchdog sleep every 4s")); 
        // Set watchdog sleep each 4s
        my_sleep_wdt_period = WDTO_4S;
      }
      if (SwitchAction==BTN_PRESSED_45) 
      {
        Serial.println(F("45 Adding Watchdog sleep every 8s")); 
        // Set watchdog sleep each 8s
        my_sleep_wdt_period = WDTO_8S;
      }
      if (SwitchAction==BTN_PRESSED_56) 
        Serial.println(F("56 Nothing")); 
      if (SwitchAction==BTN_PRESSED_67) 
        Serial.println(F("67 Nothing")); 

      Serial.println(); 
      Serial.flush();
    }
  }

  // Add Green LED
  if ( wakedBy == WAKED_BY_SWITCH ) 
  {
    ulpn.RGBBlink(1, RGB_GREEN, WDTO_120MS);
  } 
  else if ( wakedBy == WAKED_BY_BADSWITCH ) 
  {
    ulpn.RGBBlink(1, RGB_YELLOW, WDTO_120MS);
  } 
  else if ( wakedBy == WAKED_BY_WATCHDOG ) 
  {
    ulpn.RGBBlink(1, RGB_BLUE, WDTO_120MS);
  } 
  else 
  {
    ulpn.RGBBlink(1, RGB_RED, WDTO_120MS);
  }

  cli();
  iIrq=0;
  sei();

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}