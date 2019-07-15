// **********************************************************************************
// ULPNode I2C device scan test 
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
// This example scan I2C devices every 4 seconds and display type of known devices.
// the RGB led blink magenta the number of devices founds, then go to sleep mode again
//
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
  Serial.println(F("================"));
  Serial.println(F("ULPNode I2C Scan"));
  Serial.flush();

  // Full brigtness (HSV will take care)
  // this make ULPNode librarie to not divide
  // RGB Value to adjust brightness
  ulpn.RGBSetBrightness(RGB_FULL_BRIGHTNESS);

  Wire.begin();
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
  uint8_t ndev ;

  // Do a I2C scan, this will look for known devices and 
  // set the accordings flags to global status
  Serial.print(F("I2C Scanning..."));
  Serial.flush();

  ndev = ulpn.i2cScan();

  // devices found
  if (ndev ) {
    Serial.print(F("found "));
    Serial.print(ndev);
    Serial.println(F(" device(s)"));

    // Check ULPNode status flags with some known devices
    if ( ulpn.status() & RF_NODE_STATE_TSL2561) {
      Serial.println(F("TSL2561"));
    }
    if ( ulpn.status() & RF_NODE_STATE_SI7021) {
      Serial.println(F("SI7021"));
    }
    if ( ulpn.status() & RF_NODE_STATE_OLED) {
      Serial.println(F("OLED"));
    }
    if ( ulpn.status() & RF_NODE_STATE_24AA02E64) {
      Serial.println(F("MCP 24AA02E64"));
    }
    if ( ulpn.status() & RF_NODE_STATE_CCS811) {
      Serial.println(F("CCS811"));
    }
    if ( ulpn.status() & RF_NODE_STATE_BME280) {
      Serial.println(F("BME280"));
    }

    Serial.flush(); 
    // Set magenta blink to number of devices
    ulpn.RGBBlink(ndev, RGB_PURPLE, WDTO_120MS);
  }
  else {
    Serial.println(F("no device found!"));
    Serial.flush(); 
    // one longeur RED blink
    ulpn.RGBBlink(1, RGB_RED, WDTO_250MS);
  }

  Serial.println(F("Sleeping for 4s\r\n"));
  Serial.flush(); 

  // Wait for 4 second
  ulpn.sleepQuickWake(WDTO_4S);  

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}