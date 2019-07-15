// **********************************************************************************
// ULPNode onboard RGB blink led example
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
// This example blink the ULPNode onboard RGB LED every seconds 
// Low Power is done with watchdog enable to wake up the node (not the best Low Power)
// Once waked node light led then go to sleep for 120ms before light it down
// each time blink color is changed to go trought rainbow color
// you can force wake up anythime pushing the on board switch
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

// RGB Led structure working values
struct cRGB myrgb;
static int h = 0 ;  // 360 Rainbow HSV value

/* ======================================================================
Function: SetHSV
Purpose : set RGB led HSV Color 
Input   : hue (0 to 360)
          saturation (0-255)
          brightness value (0-255)
Output  : - 
Comments:  HSV Related function based on 
           <http://www.codeproject.com/miscctrl/CPicker.asp>
           dim_curve idea by Jims
           created 05-01-2010 by kasperkamperman.com
           dim_curve 'lookup table' compensate nonlinearity of human vision.
           saturation and brightness to make 'dimming' look more natural.
           Exponential function used to create values below :
           x from 0 - 255 : y = round(pow( 2.0, x+64/40.0) - 1) 
====================================================================== */
void SetHSV(int hue, byte sat, byte val) 
{
  // For demo only, else values should be located in progmem
  const byte dim_curve[] = {
  0, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
  3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6,
  6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8,
  8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11,
  11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15,
  15, 15, 16, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20,
  20, 20, 21, 21, 22, 22, 22, 23, 23, 24, 24, 25, 25, 25, 26, 26,
  27, 27, 28, 28, 29, 29, 30, 30, 31, 32, 32, 33, 33, 34, 35, 35,
  36, 36, 37, 38, 38, 39, 40, 40, 41, 42, 43, 43, 44, 45, 46, 47,
  48, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
  63, 64, 65, 66, 68, 69, 70, 71, 73, 74, 75, 76, 78, 79, 81, 82,
  83, 85, 86, 88, 90, 91, 93, 94, 96, 98, 99, 101, 103, 105, 107, 109,
  110, 112, 114, 116, 118, 121, 123, 125, 127, 129, 132, 134, 136, 139, 141, 144,
  146, 149, 151, 154, 157, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 190,
  193, 196, 200, 203, 207, 211, 214, 218, 222, 226, 230, 234, 238, 242, 248, 255,
  };

  val = dim_curve[val];
  sat = 255 - dim_curve[255 - sat];

  int base;

  // Acromatic color (gray). Hue doesn't mind.
  if (sat == 0) 
  { 
    myrgb.r = val;
    myrgb.g = val;
    myrgb.b = val;
  }
  else  
  {
    base = ((255 - sat) * val) >> 8;

    switch (hue / 60) 
    {
      case 0:
        myrgb.r = val;
        myrgb.g = (((val - base)*hue) / 60) + base;
        myrgb.b = base;
      break;

      case 1:
        myrgb.r = (((val - base)*(60 - (hue % 60))) / 60) + base;
        myrgb.g = val;
        myrgb.b = base;
      break;

      case 2:
        myrgb.r = base;
        myrgb.g = val;
        myrgb.b = (((val - base)*(hue % 60)) / 60) + base;
      break;

      case 3:
        myrgb.r = base;
        myrgb.g = (((val - base)*(60 - (hue % 60))) / 60) + base;
        myrgb.b = val;
      break;

      case 4:
        myrgb.r = (((val - base)*(hue % 60)) / 60) + base;
        myrgb.g = base;
        myrgb.b = val;
      break;

      case 5:
        myrgb.r = val;
        myrgb.g = base;
        myrgb.b = (((val - base)*(60 - (hue % 60))) / 60) + base;
      break;
    }     
  }
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

  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println();
  Serial.println(F("=============================="));
  Serial.println(F("ULPNode blink on board RGB led"));
  Serial.flush();

  // Full brigtness (HSV will take care)
  // this make ULPNode librarie to not divide
  // RGB Value to adjust brightness
  ulpn.RGBSetBrightness(RGB_FULL_BRIGHTNESS);

  // RGB Led default values
  h = 0;   //stores 0 to 360
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
  ulpn.setDevice( DEVICE_SENSORS_OFF | DEVICE_RF_OFF);
  ulpn.RGBShow(0,0,0);

  // Disable all CPU peripherals for low power
  ulpn.disableCPUDevices();

  // go to sleep mode, NO BOD, but allow it to wake us such as the switch button
  ulpn.sleepDeviceWake(SLEEP_BOD_OFF | SLEEP_WAKE_WATCHDOG | SLEEP_WAKE_SWITCH, WDTO_1S);

  // We've been waked up by a IRQ, disable ours until we done the job
  ulpn.setIRQ(IRQ_SWITCH_DISABLE);

  // Enable back Arduino core functions working such as delay() or PWM
  power_timer0_enable(); 
  power_usart0_enable();

    // Activate back Serial
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println(F("Awake")); 
  Serial.flush();  

  // Calc HSV Value
  SetHSV(h, 255, 127);

  Serial.print(F("Setting color to h="));
  Serial.print(h, DEC);
  Serial.print(F(" => r="));
  Serial.print(myrgb.r, DEC);
  Serial.print(F(" g="));
  Serial.print(myrgb.g, DEC);
  Serial.print(F(" b="));
  Serial.print(myrgb.b, DEC);
  
  Serial.println();
  Serial.flush();

  // Set RGB Led 1 blink for 120ms
  ulpn.RGBBlink(1,myrgb.r,myrgb.g,myrgb.b,WDTO_120MS);
  
  //number of hues we skip in a 360 range  
  h += 15; 
  if(h > 360)
    h %= 360;

  // go back to loop again, 1st thing is going
  // to low power the devices and going to sleep
}