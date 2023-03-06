/************************************************************
 * In this File everything which is related to the 
 * Hardware Design is defined
 * - I2C Interface
 * - Interupt Pin
 ************************************************************/ 
#ifndef _MYHWCONFIG_H_
#define _MYHWCONFIG_H_

/************************************************************
 * I2C 
 * - 800kHz
 * - GPIO Routing
 ************************************************************/ 
#define I2CSPEED         800000  
#define I2C_SDA            12                          // GPIO of SDA Signal
#define I2C_CLK            15                          // GPIO of CLK Signal


/************************************************************
 * Display
 * - 
 * - 
 ************************************************************/ 
#define TFT_CS   5
#define TFT_RST  22
#define TFT_DC   4
#define TFT_LED  15  

#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_CLK  18

/************************************************************
 * Touch Controller
 * - Touchscreen Calibration
 * - 
 ************************************************************/ 
#define TOUCH_CS 14
#define TOUCH_IRQ 2

#endif // _MYHWCONFIG_H_
