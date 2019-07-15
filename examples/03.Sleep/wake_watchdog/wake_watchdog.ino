// **********************************************************************************
// ULPNode Wake watchdog
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
// This example just go to sleep mode for 4s, then wake by watchdog and blink green
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
  // this is our Last talk before sleeping
  Serial.print(F("Zzzz...")); 
  Serial.flush();
  Serial.end();
  
  // Light off LEDs, remove power from sensors and RF modules
  ulpn.RGBShow(RGB_OFF);
  ulpn.setDevice( DEVICE_SENSORS_OFF | DEVICE_RF_OFF);

  // Disable all CPU peripherals for low power
  ulpn.disableCPUDevices();

  // go to sleep mode, NO BOD but allow only watchdog to wake us
  ulpn.sleepDeviceWake(SLEEP_BOD_OFF | SLEEP_WAKE_WATCHDOG, WDTO_4S);

  // Enable back Arduino core functions working such as delay() or PWM
  power_timer0_enable(); 
  power_usart0_enable();

  // Activate back Serial
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println(F("Awake")); 
  Serial.flush(); 

  // Set RGB red 1 blink 
  ulpn.RGBBlink(1, RGB_GREEN, WDTO_120MS);

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}