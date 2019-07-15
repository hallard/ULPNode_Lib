// **********************************************************************************
// ULPNode Library include file
// **********************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/4.0/
//
// For any explanation of ULPNode see 
// https://hallard.me/category/ulpnode/
//
// Written by Charles-Henri Hallard (http://hallard.me)
//
// History : V1.00 2014-07-14 - First release
//           V1.10 2015-03-23 - Changed to use RadioHead Library
//           V1.20 2017-06-13 - Lightened ULPNode original LIB (removed RadioHead)
//           V1.22 2017-06-14 - Removed ULPNode related Hardware (booster/WS2812)
//           V1.25 2017-06-30 - Fixed Thermistor calculation (thx Adafruit)
//           V1.26 2017-07-08 - Added CCS811 support
//           V1.27 2017-07-24 - Added BME280 support
//           V1.27 2019-07-16 - Cleanup before release
//
// All text above must be included in any redistribution.
//
// **********************************************************************************
#ifndef ULPNODELIGHT_H
#define ULPNODELIGHT_H

#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <Wire.h>

// Debug Level of this library
// 0 No Debug
// 1 Basic
// 2 Full
#define DEBUG_ULPN 2
#define SERIAL_DEBUG Serial

// I/O pins definition for ULPNode
// should be this one, other are for
// using other boards, hardware may change on 
// other boards, you'll need to adapt
#if defined (__AVR_ATmega328P__)

  #define SERIAL_DEBUG  Serial
  #define EEP_START  0
  #define NSS        SS
  #define RF_CSN_PIN SS

  #ifdef BTN_ACTION
  #define SWITCH_PIN    BTN_ACTION 
  #endif

  // Atmega328p Mini Lora
  #if defined (ARDUINO_AVR_MINILORA) 
    #define TARGET_NAME   "MiniLora"

  // Atmega328p Mini Lora with TPL5110
  #elif defined (ARDUINO_AVR_MINILORA_TPL)
    #define TARGET_NAME   "MiniLora TPL"

  // Atmega328p LoraDuino
  #elif defined (ARDUINO_AVR_LORADUINO)
    #define TARGET_NAME   "LoraDuino"

    // if your button has pullup instead pulldown
    // indicate to management button routine
    #ifdef BTN_ACTION
    #define SWITCH_PULLUP
    #endif

    // I strongly suggest to change resistor divider to 1M/1M
    // default boards will consume too much instead
    #ifdef BAT_ANALOG
    #define BATTERY_PIN BAT_ANALOG
    #define VBATT_R1 10000
    #define VBATT_R2 10000
    #endif

  // Atmega328p LoRa Radio Node
  #elif defined (ARDUINO_AVR_LORARADIONODE)
    #define TARGET_NAME   "LoRa Radio Node"

  // Atmega328p Lora Node Pro Mini 8MHz 3V3
  // CH2i custom Old testing bards
  #elif defined (ARDUINO_AVR_PRO) && (F_CPU == 8000000L) 
    #define TARGET_NAME   "Pro Mini Lora 8MHz"
    #define LORA_DIO0  2
    #define LORA_DIO1  7
    #define LORA_DIO2  8
    #define LORA_RESET A0
    #define SWITCH_PIN 3 
    #define LED_BLU    5
    #define LED_GRN    6
    #define LED_RED    9
    #define LED_PWM

  // Atmega328p Mini Lora
  // CH2i custom Old testing bards
  #elif defined (ARDUINO_AVR_PRO) && (F_CPU == 16000000L)
    #define TARGET_NAME   "Pro Mini Lora 16MHz"
    #define WAKE_PIN      3
    #define WAKE_IRQ      INT1 // Connected to INT1
    #define TPL5110_DONE  A0   // TPL5110 Done Pin
    #define LORA_DIO0  2
    #define LORA_DIO1  6
    #define LORA_DIO2  7
    #define LORA_RESET 9
    #define SWITCH_PIN 5
    #define LED_GRN    A2
    #define LED_RED    A3
    #define THERMISTOR_PIN    A7
    #define THERMISTOR_PULLUP   10000 // Value of the Pullup
    //#define THERMISTOR_PULLDOWN 10000 // Value of the Pulldown
    #define THERMISTOR_NOMINAL  10000 // resistance at 25 degrees C
    #define THERMISTOR_TNOMINAL 25    // temp. for nominal resistance (almost always 25 C)
    #define THERMISTOR_BCOEFF   3950  // The beta coefficient of the thermistor (usually 3000-4000)
  #else 
    #error "Wrong Board type"
  #endif

// Moteino Mega LoRa, Check that could be another board
#elif defined (__AVR_ATmega1284P__ )
  #define SERIAL_DEBUG  Serial
  #define EEP_START  0
  #define NSS        4
  #define RF_CSN_PIN 4
  #define TARGET_NAME "Arduino Mega"
  #define FLASH_CS 23
  #define LORA_DIO0  2
  #define LORA_DIO1  1
  #define LORA_DIO2  0
  #define LORA_RESET LMIC_UNUSED_PIN

// Error
#else
#error "Board Target not yet supported"
#endif 

// Different node internal status flags
#define RF_NODE_STATE_RADIO     0x0001 /* Radio Module OK    */
#define RF_NODE_STATE_RFM69     0x0002 /* RFM69 seen and OK   */
#define RF_NODE_STATE_NRF24     0x0004 /* NRF24 seen and OK   */
#define RF_NODE_STATE_TSL2561   0x0008 /* TSL2561 seen and OK */
#define RF_NODE_STATE_SI7021    0x0010 /* SI7021 seen and OK  */
#define RF_NODE_STATE_OLED      0x0020 /* OLED seen and OK    */
#define RF_NODE_STATE_RFM95     0x0040 /* RFM95 seen and OK */
#define RF_NODE_STATE_24AA02E64 0x0080 /* Microchip 24AA02E64 seen and OK */
#define RF_NODE_STATE_CCS811    0x0100 /* CCS811 seen and OK */
#define RF_NODE_STATE_BME280    0x0200 /* BME280 seen and OK */
#define RF_NODE_STATE_DEBUG     0x2000 /* Node with debug  en */
#define RF_NODE_STATE_BOOST     0x4000 /* DC bosster always on*/
#define RF_NODE_STATE_LOWBAT    0x8000 /* LOW Bat detected    */
#define RF_NODE_STATE_SENSOR    (RF_NODE_STATE_TSL2561|RF_NODE_STATE_SI7021|RF_NODE_STATE_CCS811|RF_NODE_STATE_BME280)

// Constant parameters for sleepDeviceWake function
#define SLEEP_BOD_OFF       0x01
#define SLEEP_ADC_OFF       0x02
#define SLEEP_WAKE_EXT      0x08
#define SLEEP_WAKE_SWITCH   0x10
#define SLEEP_WAKE_SENSORS  0x20
#define SLEEP_WAKE_WATCHDOG 0x40

// Constant parameters for application/Watchdog function
#define APP_WATCHDOG_NONE   0x00
#define APP_WATCHDOG_1S     WDTO_1S
#define APP_WATCHDOG_2S     WDTO_2S
#define APP_WATCHDOG_4S     WDTO_4S
#define APP_WATCHDOG_8S     ((uint8_t) WDTO_8S)

// Constant parameters for setIRQ function
#define IRQ_EXTWAKE_ENABLE    0x01
#define IRQ_EXTWAKE_DISABLE   0x02
#define IRQ_SWITCH_ENABLE     0x04
#define IRQ_SWITCH_DISABLE    0x08
#define IRQ_EXTWAKE           (IRQ_EXTWAKE_ENABLE | IRQ_EXTWAKE_DISABLE)
#define IRQ_SWITCH            (IRQ_SWITCH_ENABLE  | IRQ_SWITCH_DISABLE )


// Constant parameters for setSensorsIRQ function
#define IRQ_SENSORS_NONE      0x00
#define IRQ_SENSOR_A0_ENABLE  0x01
#define IRQ_SENSOR_A1_ENABLE  0x02
#define IRQ_SENSOR_A3_ENABLE  0x04
#define IRQ_SENSOR_A0_DISABLE 0x10
#define IRQ_SENSOR_A1_DISABLE 0x20
#define IRQ_SENSOR_A3_DISABLE 0x40
#define IRQ_SENSORS_ENABLE    (IRQ_SENSOR_A0_ENABLE|IRQ_SENSOR_A1_ENABLE|IRQ_SENSOR_A3_ENABLE) 
#define IRQ_SENSORS_DISABLE   (IRQ_SENSOR_A0_DISABLE|IRQ_SENSOR_A1_DISABLE|IRQ_SENSOR_A3_DISABLE) 


// Constant parameters for setDevice function
#define DEVICE_LED_ON       0x01
#define DEVICE_LED_OFF      0x02
#define DEVICE_SENSORS_ON   0x04
#define DEVICE_SENSORS_OFF  0x08
#define DEVICE_RF_ON        0x10
#define DEVICE_RF_OFF       0x20
#define DEVICE_LED          (DEVICE_LED_ON      | DEVICE_LED_OFF)
#define DEVICE_SENSORS      (DEVICE_SENSORS_ON  | DEVICE_SENSORS_OFF)
#define DEVICE_RF           (DEVICE_RF_ON       | DEVICE_RF_OFF)

// This one is more for user side, like this user can
// decide what's the threshold for his low bat level 
// detection. Take care that this will not change the
// booster works, booster will be forced to always ON only
// if vbat is below BOOSTER_MIN_STARTUP defined above
// Setting both to the same value makes sense, but for 
// example for a 3V cell coin you can set lowbat to 2V 
#define DEFAULT_LOW_BAT_THRESHOLD 900

// Average vbat number of values
#define VBATT_AVG_VALUES 8

// Vbat Resistor divider to calculate
#ifndef VBATT_R1
#define VBATT_R1  10 // 10K
#endif
#ifndef VBATT_R2
#define VBATT_R2  24 // 24K
#endif

// since measure has been divided Resistors we
// inverse to formula to calculate value (*)
//#define VBATT_DIVIDER  ((VBATT_R2+VBATT_R1)/VBATT_R2)  

#define BTN_DEBOUNCE_DELAY  100
#define BTN_LONG_PUSH_DELAY 1000

#ifdef SWITCH_PULLUP
// Button pressed set pin port to LOW
#define BTN_PRESSED   LOW
#define BTN_RELEASED HIGH
#else
// Button pressed set pin port to HIGH
#define BTN_PRESSED  HIGH
#define BTN_RELEASED LOW
#endif

// The button state machine when pressed
typedef enum {
  BTN_WAIT_PUSH,
  BTN_WAIT_RELEASE,
  BTN_WAIT_DEBOUNCE,
  BTN_WAIT_LONG_RELEASE
} 
btn_state_e;

// The actions possibe with different button press
typedef enum {
  BTN_NONE,        // do nothing.
  BTN_BAD_PRESS,   // button pressed lower than debounce time
  BTN_QUICK_PRESS, // button pressed with debounce OK
  BTN_PRESSED_12,  // pressed between 1 and 2 seconds
  BTN_PRESSED_23,  // pressed between 2 and 3 seconds 
  BTN_PRESSED_34,  // pressed between 3 and 4 seconds 
  BTN_PRESSED_45,  // pressed between 4 and 5 seconds 
  BTN_PRESSED_56,  // pressed between 5 and 6 seconds 
  BTN_PRESSED_67,  // pressed between 6 and 7 seconds 
  BTN_TIMEOUT      // Long press timeout
} 
btn_action_e;

// ADC Set input pin macro
// ========================
// Set Arduino Analog entry to read (MUX value to setup ADC channel)
//                                        MUX3 MUX2 MUX1 MUX0;
// Arduino A0 = 14 but mux need to be 0     0    0    0    0
// ....
// Arduino A7 = 21 but mux need to be 7     0    1    1    1

// REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
#define SetAnalogPinToRead(p) ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (p-14) ;

// ADC Initialization macro
// ========================
// clear the ADC prescaler defined by arduino env to enable 
// Fast ADC reading setting the prescaler to 32 (500Khz) 
// 
// See http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
// ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128 Prescaler 
// ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1)             ; // 64 Prescaler
// ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0)             ; // 32 Prescaler
// ADCSRA = _BV(ADEN) | _BV(ADPS2)                          ; // 16 Prescaler
// ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0)             ; // 8 Prescaler
// ADCSRA = _BV(ADEN) | _BV(ADPS1)                          ; // 4 Prescaler
// ADCSRA = _BV(ADEN) | _BV(ADPS2)                          ; // 2 Prescaler

#if F_CPU == 16000000L
  // We're running at 16MHz so divide by 32 to run ADC at 500Khz
  #define InitADCSRA()  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0)
#elif F_CPU == 8000000L
  // We're running at 8MHz so divide by 16 to run ADC at 500Khz
  #define InitADCSRA()  ADCSRA = _BV(ADEN) | _BV(ADPS2) 
#elif F_CPU == 4000000L
  // At 4MHz set prescaler to 8 (500Khz => 32 us per sample)
  #define InitADCSRA()  ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0) 
#else
  // Default arduino at 125Khz 16MHz (128 prescaler)
  #define InitADCSRA()  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0)
#endif

// Replace Serial.print primitive with debug one 
// or empty if debug is not enabled
#if DEBUG_ULPN > 0 || DEBUG > 0
  #define Debug(args...)    SERIAL_DEBUG.print(args)
  #define Debugln(args...)  SERIAL_DEBUG.println(args)
  #define DebugF(s)         SERIAL_DEBUG.print(F(s))
  #define DebuglnF(s)       SERIAL_DEBUG.println(F(s))
  #define Debug2(s,v)       SERIAL_DEBUG.print(s,v)
  #define Debugln2(s,v)     SERIAL_DEBUG.println(s,v)
  #define DebugFlush()      SERIAL_DEBUG.flush()
#else
  #define Debug(args...)      
  #define Debugln(args...)    
  #define DebugF(s)     
  #define DebuglnF(s)   
  #define Debug2(s,v)     
  #define Debugln2(s,v)   
  #define DebugFlush()  
#endif

// ======================================
// On board RGB LED
// ======================================
// some colors constants for calling RGB functions
#define RGB_OFF     0  ,   0,   0 /* OFF    */
#define RGB_GREEN   0,   255,   0 /* Green  */
//#define RGB_CYAN    0,    80, 255 /* Cyan   */
#define RGB_CYAN    0,   128, 255 /* Cyan   */
#define RGB_BLUE    0,     0, 255 /* Blue   */
//#define RGB_PINK    255,   0,  32 /* pink  */
#define RGB_PINK    255,   0,  128 /* pink  */
#define RGB_PURPLE  255,   0, 255 /* purple */
//#define RGB_ORANGE  255,  32,   0 /* Orange */
#define RGB_ORANGE  255, 128,   0 /* Orange */
//#define RGB_YELLOW  255,  80,   0 /* Yellow */
#define RGB_YELLOW  255, 255,   0 /* Yellow */
#define RGB_RED     255,   0,   0 /* Red    */

#define RGB_WHITE   255, 255, 255 /* White  */


// RGB Brightness
// These value are divider (number of shift of the value) used by RGBShow
// example RED = RED >> RGB_BRIGHTNESS_DIV2 
/*
#define RGB_FULL_BRIGHTNESS     0
#define RGB_BRIGHTNESS_DIV2     1 
#define RGB_BRIGHTNESS_DIV4     2 
#define RGB_BRIGHTNESS_DIV8     3
#define RGB_BRIGHTNESS_DIV16    4
#define RGB_BRIGHTNESS_DIV32    5
#define RGB_BRIGHTNESS_DIV64    6
#define RGB_BRIGHTNESS_DIV128   7
// Default we divide brightness by 8
#define RGB_DEFAULT_BRIGHTNESS RGB_FULL_BRIGHTNESS
*/

#define RGB_FULL_BRIGHTNESS  100
#define RGB_BRIGHTNESS_90  90
#define RGB_BRIGHTNESS_80  80
#define RGB_BRIGHTNESS_70  70
#define RGB_BRIGHTNESS_60  60 
#define RGB_BRIGHTNESS_50  50
#define RGB_BRIGHTNESS_40  40
#define RGB_BRIGHTNESS_30  30
#define RGB_BRIGHTNESS_20  20
#define RGB_BRIGHTNESS_10  10
#define RGB_BRIGHTNESS_00  00
// Default we divide brightness by 8
#define RGB_DEFAULT_BRIGHTNESS RGB_FULL_BRIGHTNESS


// Structure of the LED array
struct cRGB { uint8_t r; uint8_t g; uint8_t b; };

// ======================================
// Microchip 24AA02E64 EEP 
// ======================================
#define ULPN_24AA02E64_I2C_ADDRESS 0x50 // I2C address for the sensor
#define ULPN_24AA02E64_MAC_ADDRESS 0xF8 // is the memory address where the read-only MAC value is

// ======================================
// SI7021 sensor 
// ======================================
#define ULPN_SI7021_I2C_ADDRESS    0x40 // I2C address for the sensor
#define ULPN_SI7021_MEASURE_TEMP0  0xE0 // Can be read only after a RH conversion done
#define ULPN_SI7021_MEASURE_TEMP   0xE3 // Default hold
#define ULPN_SI7021_MEASURE_HUM    0xE5 // Default hold
#define ULPN_SI7021_MEASURE_NOHOLD 0x80 // NO HOLD Bit flag
#define ULPN_SI7021_WRITE_REG      0xE6
#define ULPN_SI7021_READ_REG       0xE7
#define ULPN_SI7021_SOFT_RESET     0xFE

// SI7021 Sensor resolution
// default at power up is ULPN_SI7021_RESOLUTION_14T_12RH
#define ULPN_SI7021_RESOLUTION_14T_12RH 0x00 // 12 bits RH / 14 bits Temp
#define ULPN_SI7021_RESOLUTION_13T_10RH 0x80 // 10 bits RH / 13 bits Temp
#define ULPN_SI7021_RESOLUTION_12T_08RH 0x01 //  8 bits RH / 12 bits Temp
#define ULPN_SI7021_RESOLUTION_11T_11RH 0x81 // 11 bits RH / 11 bits Temp

#define ULPN_SI7021_RESOLUTION_MASK 0B01111110

// The type of measure we want to trigger on sensor
typedef enum {
  ULPN_SI7021_READ_TEMP,
  ULPN_SI7021_READ_HUM
} 
si7021_e;


// ======================================
// CCS811 CO sensor
// ======================================
#define ULPN_CCS811_I2C_ADDRESS     0x5a
#define ULPN_CSS811_STATUS          0x00
#define ULPN_CSS811_MEAS_MODE       0x01
#define ULPN_CSS811_ALG_RESULT_DATA 0x02
#define ULPN_CSS811_RAW_DATA        0x03
#define ULPN_CSS811_ENV_DATA        0x05
#define ULPN_CSS811_NTC             0x06
#define ULPN_CSS811_THRESHOLDS      0x10
#define ULPN_CSS811_BASELINE        0x11
#define ULPN_CSS811_HW_ID           0x20
#define ULPN_CSS811_HW_VERSION      0x21
#define ULPN_CSS811_FW_BOOT_VERSION 0x23
#define ULPN_CSS811_FW_APP_VERSION  0x24
#define ULPN_CSS811_ERROR_ID        0xE0
#define ULPN_CSS811_APP_START       0xF4
#define ULPN_CSS811_SW_RESET        0xFF

// ======================================
// TSL2561 luminosity sensor
// ======================================
#define ULPN_TSL2561_I2C_ADDRESS 0x39 // I2C address for the sensor
#define ULPN_TSL2561_CONTROL     0x80
#define ULPN_TSL2561_TIMING      0x81
#define ULPN_TSL2561_INTERRUPT   0x86
#define ULPN_TSL2561_CHANNEL_0L  0x8C
#define ULPN_TSL2561_CHANNEL_0H  0x8D
#define ULPN_TSL2561_CHANNEL_1L  0x8E
#define ULPN_TSL2561_CHANNEL_1H  0x8F

// Control register bits
#define ULPN_TSL2561_POWER_UP   0x03
#define ULPN_TSL2561_POWER_DOWN 0x00

// Timing register bits
#define ULPN_TSL2561_TIMING_13MS         0x00
#define ULPN_TSL2561_TIMING_101MS        0x01
#define ULPN_TSL2561_TIMING_402MS        0x02
#define ULPN_TSL2561_TIMING_CUSTOM_STOP  0x03
#define ULPN_TSL2561_TIMING_CUSTOM_START 0x0B

#define ULPN_TSL2561_LUX_SCALE     14     // scale by 2^14
#define ULPN_TSL2561_RATIO_SCALE   9      // scale ratio by 2^9
#define ULPN_TSL2561_CH_SCALE      10     // scale channel values by 2^10
#define ULPN_TSL2561_CHSCALE_TINT_13MS  0x7517 // 322/11 * 2^CH_SCALE (13ms)
#define ULPN_TSL2561_CHSCALE_TINT_60MS  0x1800 // 322/48 * 2^CH_SCALE (60ms)
#define ULPN_TSL2561_CHSCALE_TINT_101MS 0x0fe7 // 322/81 * 2^CH_SCALE (101ms)
#define ULPN_TSL2561_CHSCALE_TINT_120MS 0x0D6B // 322/96 * 2^CH_SCALE (120ms)

#define ULPN_TSL2561_K1T 0x0040   // 0.125 * 2^RATIO_SCALE
#define ULPN_TSL2561_B1T 0x01f2   // 0.0304 * 2^LUX_SCALE
#define ULPN_TSL2561_M1T 0x01be   // 0.0272 * 2^LUX_SCALE
#define ULPN_TSL2561_K2T 0x0080   // 0.250 * 2^RATIO_SCA
#define ULPN_TSL2561_B2T 0x0214   // 0.0325 * 2^LUX_SCALE
#define ULPN_TSL2561_M2T 0x02d1   // 0.0440 * 2^LUX_SCALE
#define ULPN_TSL2561_K3T 0x00c0   // 0.375 * 2^RATIO_SCALE
#define ULPN_TSL2561_B3T 0x023f   // 0.0351 * 2^LUX_SCALE
#define ULPN_TSL2561_M3T 0x037b   // 0.0544 * 2^LUX_SCALE
#define ULPN_TSL2561_K4T 0x0100   // 0.50 * 2^RATIO_SCALE
#define ULPN_TSL2561_B4T 0x0270   // 0.0381 * 2^LUX_SCALE
#define ULPN_TSL2561_M4T 0x03fe   // 0.0624 * 2^LUX_SCALE
#define ULPN_TSL2561_K5T 0x0138   // 0.61 * 2^RATIO_SCALE
#define ULPN_TSL2561_B5T 0x016f   // 0.0224 * 2^LUX_SCALE
#define ULPN_TSL2561_M5T 0x01fc   // 0.0310 * 2^LUX_SCALE
#define ULPN_TSL2561_K6T 0x019a   // 0.80 * 2^RATIO_SCALE
#define ULPN_TSL2561_B6T 0x00d2   // 0.0128 * 2^LUX_SCALE
#define ULPN_TSL2561_M6T 0x00fb   // 0.0153 * 2^LUX_SCALE
#define ULPN_TSL2561_K7T 0x029a   // 1.3 * 2^RATIO_SCALE
#define ULPN_TSL2561_B7T 0x0018   // 0.00146 * 2^LUX_SCALE
#define ULPN_TSL2561_M7T 0x0012   // 0.00112 * 2^LUX_SCALE
#define ULPN_TSL2561_K8T 0x029a   // 1.3 * 2^RATIO_SCALE
#define ULPN_TSL2561_B8T 0x0000   // 0.000 * 2^LUX_SCALE
#define ULPN_TSL2561_M8T 0x0000   // 0.000 * 2^LUX_SCALE


// ======================================
// BME280 Pressure / Temp / Hum sensor
// ======================================
//#define ULPN_BME280_I2C_ADDRESS          0x77 // I2C address for the sensor
#define ULPN_BME280_I2C_ADDRESS          0x76 // I2C address for the sensor
#define ULPN_BME280_REGISTER_DIG_T1      0x88
#define ULPN_BME280_REGISTER_DIG_T2      0x8A
#define ULPN_BME280_REGISTER_DIG_T3      0x8C
#define ULPN_BME280_REGISTER_DIG_P1      0x8E
#define ULPN_BME280_REGISTER_DIG_P2      0x90
#define ULPN_BME280_REGISTER_DIG_P3      0x92
#define ULPN_BME280_REGISTER_DIG_P4      0x94
#define ULPN_BME280_REGISTER_DIG_P5      0x96
#define ULPN_BME280_REGISTER_DIG_P6      0x98
#define ULPN_BME280_REGISTER_DIG_P7      0x9A
#define ULPN_BME280_REGISTER_DIG_P8      0x9C
#define ULPN_BME280_REGISTER_DIG_P9      0x9E

#define ULPN_BME280_REGISTER_DIG_H1      0xA1
#define ULPN_BME280_CHIP_ID_REG          0xD0 //Chip ID
#define ULPN_BME280_RST_REG              0xE0 //Softreset Reg
#define ULPN_BME280_REGISTER_DIG_H2      0xE1
#define ULPN_BME280_REGISTER_DIG_H3      0xE3
#define ULPN_BME280_REGISTER_DIG_H4      0xE4
#define ULPN_BME280_REGISTER_DIG_H5      0xE5
#define ULPN_BME280_REGISTER_DIG_H6      0xE7
#define ULPN_BME280_CTRL_HUMIDITY_REG    0xF2 //Ctrl Humidity Reg
#define ULPN_BME280_STAT_REG             0xF3 //Status Reg
#define ULPN_BME280_CTRL_MEAS_REG        0xF4 //Ctrl Measure Reg
#define ULPN_BME280_CONFIG_REG           0xF5 //Configuration Reg
#define ULPN_BME280_PRESSURE_REG         0xF7 //Pressure 
#define ULPN_BME280_TEMPERATURE_REG      0xFA //Temperature 
#define ULPN_BME280_HUMIDITY_REG         0xFD //Humidity 

#define ULPN_BME280_SAMPLING_NONE 0b000
#define ULPN_BME280_SAMPLING_X1   0b001
#define ULPN_BME280_SAMPLING_X2   0b010
#define ULPN_BME280_SAMPLING_X4   0b011
#define ULPN_BME280_SAMPLING_X8   0b100
#define ULPN_BME280_SAMPLING_X16  0b101

#define ULPN_BME280_MODE_SLEEP    0b00
#define ULPN_BME280_MODE_FORCED   0b01
#define ULPN_BME280_MODE_NORMAL   0b11

#define ULPN_BME280_FILTER_OFF    0b000
#define ULPN_BME280_FILTER_X2     0b001
#define ULPN_BME280_FILTER_X4     0b010
#define ULPN_BME280_FILTER_X8     0b011
#define ULPN_BME280_FILTER_X16    0b100

#define ULPN_BME280_STANDBY_MS_0_5   0b000
#define ULPN_BME280_STANDBY_MS_10    0b110
#define ULPN_BME280_STANDBY_MS_20    0b111
#define ULPN_BME280_STANDBY_MS_62_5  0b001
#define ULPN_BME280_STANDBY_MS_125   0b010
#define ULPN_BME280_STANDBY_MS_250   0b011
#define ULPN_BME280_STANDBY_MS_500   0b100
#define ULPN_BME280_STANDBY_MS_1000  0b101

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
struct BME280_Calibration {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  uint8_t dig_H6;
};

// ======================================
// ULPNode CLASS
// ======================================
class ULPNode 
{
  public:
             //ULPNode(RHGenericDriver& rh_driver, RHDatagram& rh_datagram);
             ULPNode();
   uint8_t   init();
   uint16_t  readADCLowNoise(boolean average=true);
   uint16_t  readVcc();  
   uint16_t  readVoltage(uint8_t analog_pin, uint8_t r1=0, uint8_t r2=0) ;
   uint16_t  readVBat(); 
   uint16_t  readVBatAverage(boolean reset_table = false); 
   uint16_t  Vcc()  { return _vcc; };
   uint16_t  VBat() { return _vbat; };
   uint16_t  VBatAverage(); 
   uint8_t   lowBat(uint16_t threshold=DEFAULT_LOW_BAT_THRESHOLD);
   uint16_t  status() { return _status; };
   boolean   powerRadio(uint8_t power);
   uint16_t  resetState() { return _reset_state; };
   uint16_t  Lux() { return _tsl_lux; };
   int16_t   Temperature() { return _ntc_temperature; };
   int16_t   SiTemp() { return _si_temperature; };
   uint8_t   SiHum() { return _si_humidity; };
   uint16_t  CO2() { return _co2; };
   uint16_t  TVOC() { return _tvoc; };
   int16_t   BmeTemp() { return _bme_temperature; };
   uint8_t   BmeHum() { return _bme_humidity; };
   uint16_t  BmePress() { return _bme_pressure; };
   void      setDevice(uint8_t device_state=0 );
   void      disableCPUDevices(void); 
   void      setSensorsIRQ(uint8_t sensors_irq,  boolean enable_pullup=true);
   void      setIRQ(uint8_t irq_mode);
   void      RGBSetBrightness( uint8_t _rgb_brightness);
   uint8_t   RGBBrightness() { return _rgb_brightness; }; 
   void      RGBSetColor(uint8_t r=0, uint8_t g=0, uint8_t b=0);
   boolean   RGBHasSetColor(void);
   void      RGBBlink(uint8_t num, uint8_t r, uint8_t g, uint8_t b, uint8_t delay=WDTO_60MS);
   void      LEDBlink(uint8_t num, uint8_t delay=WDTO_60MS);
   void      RGBShow(void);
   void      RGBShow(uint8_t r, uint8_t g, uint8_t b);

   void      setCoreFrequency(uint8_t speed);
   void      setSerialFromRealFrequency(uint16_t bps);
   uint16_t  getCurrentFrequency() { return 16L/_BV(clock_prescale_get()); };

   void      sleepQuickWake(uint8_t wdt_period );
   void      sleepDeviceWake(uint8_t mode, uint8_t wdt_period = 0); 
   void      setWatchdog( uint8_t time_out); 
   uint8_t   getWatchdog(void) { return _watchdog; } ; 

  // User interrupts
   void attachWakeInterrupt(void (*isr)()); // Related to D3 INT1
   void attachSwitchInterrupt(void (*isr)());  // related to PCINT2
   void attachWatchdogInterrupt(void (*isr)());// related to WDT
   void attachSensorInterrupt(void (*isr)());// related to PCINT1

  // handling button events.
   btn_state_e  buttonManageState(uint8_t buttonLevel);
   btn_action_e buttonAction() { return _btn_Action; };

   // I2C Related
   uint8_t i2cScan();
   void    i2cInit(boolean check_devices = false);

   // TSL2561 Luminosity sensor related
   int8_t  tsl2561_calcLux();

   // SI7021 temperature / humidity sensor related
   int8_t  si7021_readValues(void);
   int8_t  si7021_reset(void) ;
   int8_t  si7021_setResolution(uint8_t res);

   // CCS811 CO sensor related
   uint8_t ccs811_begin(void);
   uint8_t ccs811_readStatus(uint8_t * status);
   uint8_t ccs811_dataReady(void);
   uint8_t ccs811_sleep(void);
   uint8_t ccs811_getValues(void);
   uint8_t ccs811_compensate(int16_t temp, uint8_t rh)  ;

   uint8_t bme280_begin(void);
   bool    bme280_readValues(bool wait=true) ;
   void    bme280_showCalib(void) ;

   // Resistor reading
   uint32_t getResistorValue(uint8_t pin, uint32_t rpullup);
   int16_t  getTemperatureAdafruit(uint8_t pin, float rpullup) ;
   int16_t  getTemperature(void); 

  private:
    // Get temperature from Thermistor resistor value
    int16_t getTemperatureWithSteinhart(uint32_t resistor_value) ;
 
    uint8_t RGBAdjustBrightness(uint8_t level);

    // IC2 Common 
    uint8_t i2c_readRegister (uint8_t i2caddr, uint8_t reg, uint8_t * value);
    uint8_t i2c_writeRegister(uint8_t i2caddr, uint8_t reg, uint8_t   value);
    uint8_t i2c_writeValue(uint8_t i2caddr, uint8_t value);
    uint8_t i2c_readRegister16(uint8_t i2caddr, uint8_t reg, uint16_t * value);
    uint8_t i2c_readRegister16_LE(uint8_t i2caddr, uint8_t reg, uint16_t * value) ;
    uint8_t i2c_readRegister16_S(uint8_t i2caddr, uint8_t reg, int16_t * value); 
    uint8_t i2c_readRegister16_S_LE(uint8_t i2caddr, uint8_t reg, int16_t * value) ;
    uint8_t i2c_readRegister24(uint8_t i2caddr, uint8_t reg, uint32_t * value) ;


    // TSL2561 luminosity sensor 
    #define tsl2561_readRegister(r,v)  i2c_readRegister (ULPN_TSL2561_I2C_ADDRESS, r, v)
    #define tsl2561_writeRegister(r,v) i2c_writeRegister(ULPN_TSL2561_I2C_ADDRESS, r, v)

    // CCS811 CO sensor 
    #define ccs811_readRegister(r,v)  i2c_readRegister (ULPN_CCS811_I2C_ADDRESS, r, v)
    #define ccs811_writeRegister(r,v) i2c_writeRegister(ULPN_CCS811_I2C_ADDRESS, r, v)

    // SI7021 temperature / humidity sensor related
    int8_t  si7021_StartConv(si7021_e datatype);
    uint8_t si7021_checkCRC(uint16_t data, uint8_t check);
    #define si7021_readRegister(r,v)  i2c_readRegister (ULPN_SI7021_I2C_ADDRESS, r, v)
    #define si7021_writeRegister(r,v) i2c_writeRegister(ULPN_SI7021_I2C_ADDRESS, r, v)
    #define si7021_writeValue(v)      i2c_writeValue(ULPN_SI7021_I2C_ADDRESS, v)


    // BME280 Pressure sensor 
    uint8_t bme280_readCoefficients(void);
    uint8_t bme280_readTemperature(void);
    uint8_t bme280_readPressure( void );
    uint8_t bme280_readHumidity(void);

    #define bme280_readRegister(r,v)   i2c_readRegister(ULPN_BME280_I2C_ADDRESS, r, v)
    #define bme280_writeRegister(r,v)  i2c_writeRegister(ULPN_BME280_I2C_ADDRESS, r, v)
    #define bme280_readRegister16(r,v) i2c_readRegister16(ULPN_BME280_I2C_ADDRESS, r, v)
    #define bme280_readRegister16_LE(r,v) i2c_readRegister16_LE(ULPN_BME280_I2C_ADDRESS, r, v)
    #define bme280_readRegister16_S(r,v) i2c_readRegister16_S(ULPN_BME280_I2C_ADDRESS, r, v)
    #define bme280_readRegister16_S_LE(r,v) i2c_readRegister16_S_LE(ULPN_BME280_I2C_ADDRESS, r, v)
    #define bme280_readRegister24(r,v) i2c_readRegister24(ULPN_BME280_I2C_ADDRESS, r, v)

    // Global node status
    uint16_t  _status ;

    // what caused our reset
    uint8_t  _reset_state ;

    // Application watchdog value
    uint8_t   _watchdog;

    // These variables manages button feature
    btn_state_e   _btn_State;    // Button management state machine
    btn_action_e  _btn_Action;   // button action after press
    boolean       _btn_LongPress;// indicate a long press on button
    unsigned long _btn_StartTime;// when push started

    // Battery reading related
    uint16_t   _vbat_avg_tbl[VBATT_AVG_VALUES];  // Last vbat measures table
    uint8_t    _vbat_tbl_idx; // table index
    uint16_t   _vbat;  // Last vbat measure
    uint16_t   _vcc;   // Last vcc measure

    struct cRGB _rgb_led;        // RGB Led values
    uint8_t     _rgb_brightness; // RGB Brightness

    uint16_t    _tsl_lux;         // latest lux value read
    int16_t     _ntc_temperature; // latest temperature value read from ntc

    uint8_t     _si_humidity; // latest humidity value read
    int16_t     _si_temperature; // latest temperature value read

    uint16_t    _tvoc;
    uint16_t    _co2;

    // BME280
    int32_t            _bme_t_fine;
    uint8_t            _bme_humidity;     // latest humidity value read
    int16_t            _bme_temperature;  // latest temperature value read
    uint16_t           _bme_pressure;     // latest pressure value read
    struct BME280_Calibration _bme_calib;
};

#endif 



  
  