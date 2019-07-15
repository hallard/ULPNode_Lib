// **********************************************************************************
// ULPNode Read and Display VCC and VBat values
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
// This example read VCC and VBat every 8 seconds and display values.
// It also shows how to use the debug macro
//
// **********************************************************************************
#include <arduino.h>
#include <Wire.h>

// Debug Level of sketch (another flag is in the library for library only)
// But you need to put this one before include so debug macro will be enabled
// 0 No Debug
// 1 Basic
// 2 Full
#define DEBUG 1

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

  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F("ULPNode Read VCC and VBat test"));
  Serial.flush();
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

  // This point is managed by ULPNode library, but need to be user refreshed
  // you don't need to do this each wake up, but twice a day sounds good
  ulpn.readVcc();
  ulpn.readVBatAverage();

  #if DEBUG > 0
  DebuglnF("Awake");  
  DebugF("Vcc=");
  Debug(ulpn.Vcc());
  DebugF("mV VBat=");
  Debug(ulpn.VBatAverage());
  DebuglnF("mV ");
  DebugFlush();  
  #endif

  // this is our Last talk before sleeping
  Serial.print(F("Zzzz...")); 
  Serial.flush();
  Serial.end();
  
  // Light off LEDs, remove power from sensors and RF modules
  ulpn.setDevice(DEVICE_LED_OFF | DEVICE_SENSORS_OFF | DEVICE_RF_OFF);

  // Disable all CPU peripherals for low power
  ulpn.disableCPUDevices();

  // go to sleep mode, NO BOD, and wake by watchdog only
  ulpn.sleepDeviceWake(SLEEP_BOD_OFF | SLEEP_WAKE_WATCHDOG, WDTO_8S);
  
  // Enable back Arduino core functions working such as delay() or PWM and serial
  power_timer0_enable(); 
  power_usart0_enable();

  // Activate back Serial
  Serial.begin(SERIAL_PORT_SPEED);

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}