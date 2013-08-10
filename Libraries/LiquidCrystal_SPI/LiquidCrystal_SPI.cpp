#include "LiquidCrystal_SPI.h"
#include <SPI.h>

LiquidCrystal_SPI::LiquidCrystal_SPI(uint8_t csPin)
{	
	this->m_CsBitMask = digitalPinToBitMask(csPin);
	uint8_t port = digitalPinToPort(csPin);
	this->m_CsPin = portOutputRegister(port);
	this->m_CsPortMode = portModeRegister(port);
}

void LiquidCrystal_SPI::begin(uint8_t columns, uint8_t rows, uint8_t charsize)
{
	*this->m_CsPortMode |= this->m_CsBitMask;	// Set CS pin as output.
	*this->m_CsPin &= this->m_CsBitMask;		// Set CS pin high.
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV4);
	this->m_Backlight = BL;
	m_InitCmd = LCD_FUNCTIONSET | LCD_8BITMODE | LCD_1LINE | charsize;
	if (rows > 1)
		this->m_InitCmd |= LCD_2LINE;
	this->m_DisplayCtl = LCD_DISPLAYCONTROL | LCD_DISPLAYON;
	this->m_DisplayMode = LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDISPLAYOFF;
	this->initLcd();
}

void LiquidCrystal_SPI::clear()
{
	this->command(LCD_CLEARDISPLAY);
	delay(2);
}

void LiquidCrystal_SPI::home()
{
	this->command(LCD_RETURNHOME);
	delay(2);
}

void LiquidCrystal_SPI::noDisplay()
{
	this->m_DisplayCtl &= ~LCD_DISPLAYON;
	this->command(this->m_DisplayCtl);	
}

void LiquidCrystal_SPI::display()
{
	this->m_DisplayCtl |= LCD_DISPLAYON;
	this->command(this->m_DisplayCtl);	
}

void LiquidCrystal_SPI::noBlink()
{
	this->m_DisplayCtl &= ~LCD_BLINKON;
	this->command(this->m_DisplayCtl);	
}

void LiquidCrystal_SPI::blink()
{
	this->m_DisplayCtl |= LCD_BLINKON;
	this->command(this->m_DisplayCtl);	
}

void LiquidCrystal_SPI::noCursor()
{
	this->m_DisplayCtl &= ~LCD_CURSORON;
	this->command(this->m_DisplayCtl);	
}

void LiquidCrystal_SPI::cursor()
{
	this->m_DisplayCtl |= LCD_CURSORON;
	this->command(this->m_DisplayCtl);	
}

void LiquidCrystal_SPI::scrollDisplayLeft()
{
	this->command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LiquidCrystal_SPI::scrollDisplayRight()
{
	this->command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void LiquidCrystal_SPI::leftToRight()
{
	this->m_DisplayMode |= LCD_ENTRYLEFT;
	this->command(this->m_DisplayMode);	
}

void LiquidCrystal_SPI::rightToLeft()
{
	this->m_DisplayMode &= ~LCD_ENTRYLEFT;
	this->command(this->m_DisplayMode);	
}

void LiquidCrystal_SPI::autoscroll()
{
	this->m_DisplayMode |= LCD_ENTRYSHIFTDISPLAYON;
	this->command(this->m_DisplayMode);	
}

void LiquidCrystal_SPI::noAutoscroll()
{
	this->m_DisplayMode &= ~LCD_ENTRYSHIFTDISPLAYON;
	this->command(this->m_DisplayMode);	
}

void LiquidCrystal_SPI::enableBackLight(bool value)
{
	this->m_Backlight = value ? BL : 0;
	this->writeSpi(0, 0);
}


void LiquidCrystal_SPI::createChar(uint8_t index, const uint8_t charMap[])
{
	index &= 0x7;
	this->command(LCD_SETCGRAMADDR | (index << 3));
	for (uint8_t i = 0; i < 8; i++)
		this->writeSpi(charMap[i], RS);
}

void LiquidCrystal_SPI::setCursor(uint8_t column, uint8_t row)
{
	uint8_t rowAddress[] = { 0x00, 0x40, 0x14, 0x54 };	
	this->command(LCD_SETDDRAMADDR | (rowAddress[row] + column));
}

size_t LiquidCrystal_SPI::write(uint8_t byte)
{
	this->writeSpi(byte, RS);
	return 1;
}

void LiquidCrystal_SPI::writeSpi(uint8_t data, uint8_t cmd)
{
	cmd |= this->m_Backlight;
	SPI.transfer(cmd);
	SPI.transfer(data);
	*this->m_CsPin |= this->m_CsBitMask;	// digitalWrite(CS, HIGH);
	delayMicroseconds(1);
	*this->m_CsPin &= ~this->m_CsBitMask;	// digitalWrite(CS, LOW);	
#if (F_CPU <= 16000000)
	delayMicroseconds(30);
#else
	delayMicroseconds(37);      // commands & data writes need > 37us to complete
#endif
}

void LiquidCrystal_SPI::command(uint8_t cmd)
{
	this->writeSpi(cmd, 0);
}

void LiquidCrystal_SPI::initLcd()
{
	// Initialization sequence
	delay(50);
	this->command(this->m_InitCmd);
	delay(5);
	this->command(this->m_InitCmd);
	delay(1);
	this->command(this->m_InitCmd);
	this->command(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF);	
	this->command(LCD_CLEARDISPLAY);
	delay(2);
	this->command(this->m_DisplayMode);
	// ----------------------------

	this->command(this->m_DisplayCtl);
	this->command(LCD_RETURNHOME);
	delay(2);
}
