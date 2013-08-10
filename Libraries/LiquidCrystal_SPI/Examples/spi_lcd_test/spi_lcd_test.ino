#include <SPI.h>
#include <LiquidCrystal_SPI.h>

// This library is completely compatible with the standard LiquidCrystal library
// which comes with the Arduino IDE. All you have to do is subsitute its declaration
// in your sketch, like so:

#define LCD_CS 10 // The Enable/'latch' pin for the circuit.

LiquidCrystal_SPI lcd(LCD_CS);

void setup()
{
	lcd.begin(16, 2);
}

void loop()
{
	lcdTestCycle();
}

void lcdTestCycle()
{
	lcd.clear();
	lcd.print(F("SPI LCD Backpack"));
	lcd.setCursor(0, 1);
	lcd.print(F("  Instructible  "));
	delay(2000);	
	
	lcd.clear();
	lcd.print(F("Turning display"));
	lcd.setCursor(0, 1);
	lcd.print(F("off."));
	delay(2000);
	lcd.noDisplay();
	delay(2000);	
	lcd.clear();
	lcd.print(F("Turning display"));
	lcd.setCursor(0, 1);
	lcd.print(F("back on."));
	lcd.display();
	delay(2000);

	lcd.clear();
	lcd.print(F("Blinking cursor"));
	lcd.setCursor(0, 1);
	lcd.print(F("on each char"));	
	lcd.setCursor(0, 0);
	lcd.blink();
	for (uint8_t i = 0; i < 32; i++)
	{
		lcd.setCursor(i % 16, i / 16);
		delay(500);
	}
	delay(1000);
	
	lcd.clear();
	lcd.noBlink();
	lcd.print(F("Blinking cursor"));
	lcd.setCursor(0, 1);
	lcd.print(F("off."));	
	delay(2000);
	
	lcd.clear();
	lcd.print(F("Show cursor"));
	lcd.setCursor(0, 1);
	lcd.print(F("position"));	
	lcd.setCursor(0, 0);
	lcd.cursor();
	for (uint8_t i = 0; i < 32; i++)
	{
		lcd.setCursor(i % 16, i / 16);
		delay(500);
	}
	delay(1000);
	
	lcd.clear();
	lcd.noCursor();
	lcd.print(F("Cursor off"));
	delay(2000);
	
	lcd.clear();
	lcd.print(F("Scroll display"));
	lcd.setCursor(0, 1);
	lcd.print(F("to the left"));
	delay(1000);
	for (uint8_t i = 0; i < 8; i++)
	{
		lcd.scrollDisplayLeft();
		delay(500);
	}
	delay(1000);

	lcd.clear();
	lcd.print(F("Scroll display"));
	lcd.setCursor(0, 1);
	lcd.print(F("to the right"));
	delay(1000);
	for (uint8_t i = 0; i < 8; i++)
	{
		lcd.scrollDisplayRight();
		delay(500);
	}
	delay(1000);

	lcd.leftToRight();
	lcd.clear();
	lcd.print(F("Left to right"));
	lcd.setCursor(0, 1);
	lcd.print(F("mode"));
	delay(2000);

	lcd.rightToLeft();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("Right to left"));
	lcd.setCursor(0, 1);
	lcd.print(F("mode"));
	delay(2000);
	lcd.leftToRight();

	char myStr[] = "My string.";
	lcd.clear();
	lcd.print(F("Autoscroll on"));
	delay(1000);
	lcd.autoscroll();
	lcd.setCursor(16, 1);
	for (uint8_t i = 0; i < strlen(myStr); i++)
	{
		lcd.print(myStr[i]);
		delay(500);
	}
	delay(2000);

	lcd.clear();
	lcd.noAutoscroll();
	lcd.setCursor(0, 0);
	lcd.print(F("Autoscroll off"));
	delay(1000);
	lcd.setCursor(0, 1);
	for (uint8_t i = 0; i < strlen(myStr); i++)
	{
		lcd.print(myStr[i]);
		delay(500);
	}
	delay(2000);

	bool bl = true;
	lcd.clear();
	lcd.print(F("Blinking"));
	lcd.setCursor(0, 1);
	lcd.print(F("backlight"));
	delay(1000);
	for (uint8_t i = 0; i < 20; i++)
	{
		bl = !bl;
		lcd.enableBackLight(bl);
		delay(250);
	}
}
