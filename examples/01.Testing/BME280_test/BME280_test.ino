// **********************************************************************************
// ULPNode BME280 Pressure / Temperature / Humidity sensor test
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
  Serial.println(F("==================="));
  Serial.println(F("ULPNode SI7021 test"));
  Serial.flush();

  // Full brigtness (HSV will take care)
  // this make ULPNode librarie to not divide
  // RGB Value to adjust brightness
  ulpn.RGBSetBrightness(RGB_FULL_BRIGHTNESS);

  // Do a I2C scan, this will look for known devices and 
  // set the accordings flags to global status
  ulpn.i2cScan();

  if (ulpn.status() & RF_NODE_STATE_BME280 ) {
    Serial.println(F("BME280 found!"));

    if (ulpn.bme280_begin() == 0) {
      Serial.println(F("BME280 init done"));
    } else {
      Serial.println(F("BME280 init error!"));
    }
  }
  else
    Serial.println(F("Error, BME280 not found!"));
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
  if ( !(error=ulpn.bme280_readValues()) )
  {
    Serial.print(F("  BME280 Pressure=")); 
    Serial.print(ulpn.BmePress()/10.0,1); 
    Serial.print(F("  Temperature=")); 
    Serial.print(ulpn.BmeTemp()/100.0,2); 
    Serial.print(F(" C")); 
    Serial.print(F(" Humidity=")); 
    Serial.print(ulpn.BmeHum()); 
    Serial.println(F(" %")); 
    Serial.flush();  

    if ( ulpn.BmeTemp() >350 ) // > 35 C
      ulpn.RGBBlink(1, RGB_RED, WDTO_120MS); 
    else if ( ulpn.BmeTemp() > 300 ) // > 30 C
      ulpn.RGBBlink(1, RGB_ORANGE, WDTO_120MS); 
    else if ( ulpn.BmeTemp() > 250 ) // > 25 C
      ulpn.RGBBlink(1, RGB_YELLOW, WDTO_120MS); 
    else if ( ulpn.BmeTemp() > 200 ) // > 20 C
      ulpn.RGBBlink(1, RGB_GREEN, WDTO_120MS); 
    else if ( ulpn.BmeTemp() > 10 ) // > 10 C
      ulpn.RGBBlink(1, RGB_CYAN, WDTO_120MS);
    else if ( ulpn.BmeTemp() > 0 ) // > 0 C
      ulpn.RGBBlink(1, RGB_BLUE, WDTO_120MS);
    else if ( ulpn.BmeTemp() > -50 ) // > -5 C
      ulpn.RGBBlink(1, RGB_PURPLE, WDTO_120MS);
    else // < -5 C
      ulpn.RGBBlink(1, RGB_WHITE, WDTO_120MS);
  }
  else
  {
    Serial.println(F("BME280 ")); 
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