// **********************************************************************************
// ULPNode Wake watchdog every 15 min
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
// History : V1.00 2019-07-14 - First release
//
// All text above must be included in any redistribution.
//
// **********************************************************************************
//
// This example just go to sleep mode for 8s (maximum watchdog time), then wake do 
// action only every 15 minutess.
// Low Power is done with watchdog enable to wake up the node (not the best Low Power)
//
// **********************************************************************************
#include <arduino.h>
#include <Wire.h>
#include <ULPNode.h>

// Remember we can be at 4MHz so serial can't do better than 38400 
// Since Arduino 1.5, now they added the 250KBPS speed great !!!
//#define SERIAL_PORT_SPEED 250000
#define SERIAL_PORT_SPEED 115200

// Schedule TX every this many seconds (multiple of 8 due to watchdog).
// Takr care of to duty cycle limitations).
// 15 min is 900s but watchdog multiple of 8s so 904 is 113 wake of 8s => 15 min and 4 seconds
#define TX_INTERVAL 904 

// define an ULPNode object 
ULPNode  ulpn;

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

  // Give power to sensors (and so WS2812 RGB LED)
  // And disable powering RF Module just in case
  ulpn.setDevice(DEVICE_SENSORS_ON | DEVICE_RF_OFF);

  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println();
  Serial.println(F("=========================="));
  Serial.println(F("ULPNode Wake Watchdog test"));
  Serial.flush();

  // 50% brigtness 
  ulpn.RGBSetBrightness(RGB_BRIGHTNESS_50);
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
  static uint32_t wdt_count  = TX_INTERVAL / 8 ; // number of WDT wake between transmit

  // Wait n WDT wake to do things
  while (wdt_count) {

    // print counter before sleeping
    // if you do not want to print anythong and conserve low power
    // comment all the lines in the while loop except the one
    // ulpn.sleepDeviceWake(...)
    Serial.print(wdt_count); 
    Serial.print(F(" => ")); 
    Serial.print(wdt_count*8); 
    Serial.print(F(" seconds ")); 

    // this is our Last talk before sleeping
    Serial.print(F(" Zzzz...")); 
    Serial.flush();
    Serial.end();

    // Disable all CPU peripherals for low power
    ulpn.disableCPUDevices();

    // go to sleep mode, NO BOD but allow only watchdog to wake us
    ulpn.sleepDeviceWake(SLEEP_BOD_OFF | SLEEP_WAKE_WATCHDOG, WDTO_8S);

    // Enable back Arduino core functions working such as delay() or PWM
    power_timer0_enable(); 
    power_usart0_enable();

    // Activate back Serial
    Serial.begin(SERIAL_PORT_SPEED);
    Serial.println(F("Awake")); 
    Serial.flush(); 

    // decrement our counter
    wdt_count--;
  }

  // Restart our watchdog count counter
  wdt_count = TX_INTERVAL / 8;

  Serial.println(F("Action time")); 

  // Activate RGB Led
  ulpn.setDevice(DEVICE_SENSORS_ON);

  // Set RGB red 1 blink 
  ulpn.RGBBlink(1, RGB_GREEN, WDTO_120MS);

  // power off RGB Led
  ulpn.setDevice(DEVICE_SENSORS_OFF);

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}