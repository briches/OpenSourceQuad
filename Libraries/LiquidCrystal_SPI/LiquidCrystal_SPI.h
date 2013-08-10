#pragma once
#include "Arduino.h"

// commands
#define LCD_CLEARDISPLAY			0x01
#define LCD_RETURNHOME				0x02
#define LCD_ENTRYMODESET			0x04
#define LCD_DISPLAYCONTROL			0x08
#define LCD_CURSORSHIFT				0x10
#define LCD_FUNCTIONSET				0x20
#define LCD_SETCGRAMADDR			0x40
#define LCD_SETDDRAMADDR			0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT				0x00
#define LCD_ENTRYLEFT				0x02
#define LCD_ENTRYSHIFTDISPLAYON		0x01
#define LCD_ENTRYSHIFTDISPLAYOFF	0x00

// flags for display on/off control
#define LCD_DISPLAYON				0x04
#define LCD_DISPLAYOFF				0x00
#define LCD_CURSORON				0x02
#define LCD_CURSOROFF				0x00
#define LCD_BLINKON					0x01
#define LCD_BLINKOFF				0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE				0x08
#define LCD_CURSORMOVE				0x00
#define LCD_MOVERIGHT				0x04
#define LCD_MOVELEFT				0x00

// flags for function set
#define LCD_8BITMODE				0x10
#define LCD_4BITMODE				0x00
#define LCD_2LINE					0x08
#define LCD_1LINE					0x00
#define LCD_5x10DOTS				0x04
#define LCD_5x8DOTS					0x00

#define BL							0x01
#define RS							0x02

class LiquidCrystal_SPI : public Print
{
public:
	LiquidCrystal_SPI(uint8_t csPin);

	void begin(uint8_t columns, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);

	void clear();
	void home();

	void noDisplay();
	void display();
	void noBlink();
	void blink();
	void noCursor();
	void cursor();
	void scrollDisplayLeft();
	void scrollDisplayRight();
	void leftToRight();
	void rightToLeft();
	void autoscroll();
	void noAutoscroll();
	void enableBackLight(bool value);
	
	void createChar(uint8_t index, const uint8_t charMap[]);
	void setCursor(uint8_t column, uint8_t row);
	virtual size_t write(uint8_t byte);
	void command(uint8_t cmd);

private: 
	void initLcd();
	void writeSpi(uint8_t data, uint8_t cmd);
	volatile uint8_t *m_CsPin;
	volatile uint8_t *m_CsPortMode;
	volatile uint8_t m_CsBitMask;
	uint8_t m_InitCmd;
	uint8_t m_DisplayCtl;
	uint8_t m_DisplayMode;
	uint8_t m_Backlight;
};

