/*
 * STM32_I2C_Display.h
 *
 *  Created on: Nov 15, 2021
 *      Author: CarlosWork
 */

#ifndef INC_STM32_I2C_DISPLAY_H_
#define INC_STM32_I2C_DISPLAY_H_

#include <inttypes.h>
#include "stm32f4xx_hal.h"
#include <string.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Register select bit

void LiquidCrystal_I2C(uint8_t, uint8_t, uint8_t);
void lcdBegin(void);
void lcdClear(void);
void lcdHome(void);
void lcdNoBacklight(void);
void lcdBacklight(void);
void lcdNoDisplay(void);
void lcdDisplay(void);
void lcdNoBlink(void);
void lcdBlink(void);
void lcdNoCursor(void);
void lcdCursor(void);
void lcdScrollDisplayLeft(void);
void lcdScrollDisplayRight(void);
void lcdLeftToRight(void);
void lcdRightToLeft(void);
void lcdAutoscroll(void);
void lcdNoAutoscroll(void);
void lcdCreateChar(uint8_t, uint8_t[]);
void lcdSetCursor(uint8_t, uint8_t);
size_t lcdPrint(const char []);
size_t lcdWrite(uint8_t value);
//uint32_t DWT_Delay_Init(void);

#endif /* INC_STM32_I2C_DISPLAY_H_ */

