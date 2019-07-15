// **********************************************************************************
// ULPNode CCS811 Air Quality CO2 and TVOC sensor test
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
// This example just read CO2 and TVOC sensor every second then 
// blink green if ok and red if not,
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
  Serial.println(F("==================="));
  Serial.println(F("ULPNode CCS811 test"));
  Serial.flush();

  // Full brigtness (HSV will take care)
  // this make ULPNode librarie to not divide
  // RGB Value to adjust brightness
  ulpn.RGBSetBrightness(RGB_FULL_BRIGHTNESS);

  // Wait for CCS811 sensor to settle (20 ms after POR)
  ulpn.sleepQuickWake(WDTO_30MS);   

  // Do a I2C scan, this will look for known devices and 
  // set the accordings flags to global status
  ulpn.i2cScan();

  if (ulpn.status() & RF_NODE_STATE_CCS811 ) {
    Serial.println(F("CCS811 found!"));

    if (ulpn.status() & RF_NODE_STATE_SI7021 ) {
      Serial.println(F("SI7021 found, we can do temperature compensation"));

      DebugF("Reset SI7021 ");
      DebugFlush();
      if (ulpn.si7021_reset() == 0) {
        DebuglnF("OK");
      } else {
        DebuglnF("Error");
      }

    }

    Serial.flush();
    if (ulpn.ccs811_begin() == 0) {
      Serial.println(F("CCS811 init done"));
    } else {
      Serial.println(F("CCS811 init error!"));
    }
  }
  else
    Serial.println(F("Error, CCS811 not found!"));
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
  int8_t error;

  // this is our Last talk before sleeping
  Serial.print(F("Zzzz...")); 
  Serial.flush();
  Serial.end();
  
  // Light off LEDs, remove power from sensors and RF modules
  ulpn.RGBShow(0,0,0);
  ulpn.setDevice( DEVICE_SENSORS_OFF | DEVICE_RF_OFF);

  // Disable all CPU peripherals for low power
  ulpn.disableCPUDevices();

  // go to sleep mode, NO BOD but allow it to wake us such as the switch button
  ulpn.sleepDeviceWake(SLEEP_BOD_OFF | SLEEP_WAKE_WATCHDOG | SLEEP_WAKE_SWITCH, WDTO_1S);

  // We've been waked up by a IRQ, disable ours until we done the job
  ulpn.setIRQ(IRQ_SWITCH_DISABLE );

  // Enable back Arduino core functions working such as delay() or PWM
  power_timer0_enable(); 
  power_usart0_enable();

  // Enable also sensors, RGB LED and onboard LED also
  ulpn.setDevice( DEVICE_SENSORS_ON );

  // Activate back Serial
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println(F("Awake")); 
  Serial.flush(); 

  // Activate back I2C stuff, check back device presence
  ulpn.i2cInit(true);   

  // Read sensor values, it will set the accordings flags
  // to global status if the sensor is found or not found 
  if ( !(error=ulpn.ccs811_getValues()) )
  {
    Serial.print(F("  CCS811 CO2=")); 
    Serial.print(ulpn.CO2()); 
    Serial.print(F("  TVOC=")); 
    Serial.print(ulpn.TVOC()); 
    
    if (ulpn.status() & RF_NODE_STATE_SI7021 ) {

      if ( !(error=ulpn.si7021_readValues()) )
      {
        Serial.print(F("  SI7021 Temperature=")); 
        Serial.print(ulpn.SiTemp()/100.0,2); 
        Serial.print(F(" C")); 
        Serial.print(F(" Humidity=")); 
        Serial.print(ulpn.SiTemp()); 
        Serial.println(F(" %")); 
      }
      else
      {
        Serial.print(F(" SI7021 ")); 
        Serial.flush();  
        if (error) {
          Serial.print(F("Read error:")); 
          Serial.println(error); 
        } else {
          Serial.println(F("not found!")); 
        }
      }
    }
    Serial.flush();  

    if ( ulpn.CO2() >1000 ) // > 1000ppm
      ulpn.RGBBlink(1, RGB_RED, WDTO_120MS); 
    else if ( ulpn.CO2() > 500 ) // > 500 ppm
      ulpn.RGBBlink(1, RGB_ORANGE, WDTO_120MS); 
    else  // > 20 C
      ulpn.RGBBlink(1, RGB_GREEN, WDTO_120MS); 
  }
  else
  {
    Serial.println(F("CCS811 ")); 
    Serial.flush();  
    if (error) {
      Serial.print(F("Read error:")); 
      Serial.println(error); 
    } else {
      Serial.println(F("not found!")); 
    }
    Serial.flush();  
    // Set RGB red 2 blink 
    ulpn.RGBBlink(2, RGB_RED, WDTO_120MS);
  }

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}