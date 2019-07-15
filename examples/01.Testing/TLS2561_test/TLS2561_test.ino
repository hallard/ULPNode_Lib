// **********************************************************************************
// ULPNode TLS2561 Luminosity sensor test
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
// This example just read luminosity sensor every second then 
// blink green if ok and red if not, brightness is adjusted depending on luminosity
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
  Serial.println(F("===================="));
  Serial.println(F("ULPNode TLS2561 test"));
  Serial.flush();

  // Full brigtness (HSV will take care)
  // this make ULPNode librarie to not divide
  // RGB Value to adjust brightness
  ulpn.RGBSetBrightness(RGB_FULL_BRIGHTNESS);

  // Do a I2C scan, this will look for known devices and 
  // set the accordings flags to global status
  ulpn.i2cScan();

  // Read sensor LUX value, it will set the accordings flags
  // to global status if the sensor is foundfound 
  //ulpn.readTLS();

  if (ulpn.status() & RF_NODE_STATE_TSL2561 )
    Serial.println(F("TLS2561 found!"));
  else
    Serial.println(F("Error, TLS2561 not found!"));
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
  uint8_t error;

  // this is our Last talk before sleeping
  Serial.print(F("Zzzz...")); 
  Serial.flush();
  Serial.end();
  
  // Light off LEDs, remove power from sensors and RF modules
  ulpn.RGBShow(0,0,0);
  ulpn.setDevice( DEVICE_SENSORS_OFF | DEVICE_RF_OFF);

  // Disable all CPU peripherals for low power
  ulpn.disableCPUDevices();

  // go to sleep mode, NO BOD, but allow it to wake us such as the switch button
  ulpn.sleepDeviceWake(SLEEP_BOD_OFF | SLEEP_WAKE_WATCHDOG | SLEEP_WAKE_SWITCH, WDTO_1S);

  // We've been waked up by a IRQ, disable ours until we done the job
  ulpn.setIRQ(IRQ_SWITCH_DISABLE);

  // Enable back Arduino core functions working such as delay() or PWM
  power_timer0_enable(); 
  power_usart0_enable();

  // Enable also sensors, RGB LED and onboard LED also
  ulpn.setDevice( DEVICE_SENSORS_ON );

  // Activate back Serial
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println(F("Awake")); 
  Serial.flush();  

  // Restore back I2C features re check devices
  ulpn.i2cInit(true);

  // Read luminosity
  if ( !(error = ulpn.tsl2561_calcLux() ) ) {
    // lowest
    uint8_t brightness=2;

    // > 6000 lux
    if ( ulpn.Lux() >6000 )
      brightness = RGB_FULL_BRIGHTNESS;
    else if ( ulpn.Lux() > 4000 )
      brightness = RGB_BRIGHTNESS_80;
    else if ( ulpn.Lux() > 1000 )
      brightness = RGB_BRIGHTNESS_60;
    else if ( ulpn.Lux() > 500 )
      brightness = RGB_BRIGHTNESS_50;
    else if ( ulpn.Lux() > 100 )
      brightness = RGB_BRIGHTNESS_40;
    else if ( ulpn.Lux() > 30 )
      brightness = RGB_BRIGHTNESS_30;
    else 
      brightness = ulpn.Lux() ? ulpn.Lux() : 1;

    Serial.print(F("Luminosity=")); 
    Serial.print(ulpn.Lux()); 
    Serial.print(F(" Lux => Set Brightness divider to ")); 
    Serial.println(brightness,DEC); 
    Serial.flush();  

    ulpn.RGBSetBrightness(brightness);

    // Set RGB green 1 blink 
    ulpn.RGBBlink(1, RGB_GREEN, WDTO_120MS);
  } else {
    Serial.print(F("TSL2561 ")); 
    if (error) {
      Serial.print(F("Read error:")); 
      Serial.println(error); 
    } else {
      Serial.println(F("Not found!")); 
    }
    Serial.flush();  

    // Set RGB red 2 blink 
    ulpn.RGBBlink(1, RGB_RED, WDTO_120MS);
  }

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}