#include "parallel_lcd.h"

void initialiseLcd()
{
  Serial.begin(9600);
  Serial.println(F("TFT LCD test"));
//   lcd.reset();

   uint16_t identifier = lcd.readID();
   if(identifier == 0x9325) 
   {
      Serial.println(F("Found ILI9325 LCD driver"));
   }
   else if(identifier == 0x9328)
   {
      Serial.println(F("Found ILI9328 LCD driver"));
   }
   else if(identifier == 0x4535)
   {
      Serial.println(F("Found LGDP4535 LCD driver"));
   }
   else if(identifier == 0x7575)
   {
      Serial.println(F("Found HX8347G LCD driver"));
   }
   else if(identifier == 0x9341)
   {
      Serial.println(F("Found ILI9341 LCD driver"));
   }
   else if(identifier == 0x7783)
   {
      Serial.println(F("Found ST7781 LCD driver"));
   }
   else if(identifier == 0x8230)
   {
      Serial.println(F("Found UC8230 LCD driver"));  
   }
   else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
   }
   else if(identifier==0x0101)
   {     
      identifier=0x9341;
      Serial.println(F("Found 0x9341 LCD driver"));
   }
   else if(identifier==0x9481)
   {     
      Serial.println(F("Found 0x9481 LCD driver"));
   }
   else if(identifier==0x9486)
   {     
      Serial.println(F("Found 0x9486 LCD driver"));
   }
   else
   {
      Serial.print(F("Unknown LCD driver chip: "));
      Serial.println(identifier, HEX);
      Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
      Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
      Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
      Serial.println(F("If using the breakout board, it should NOT be #defined!"));
      Serial.println(F("Also if using the breakout, double-check that all wiring"));
      Serial.println(F("matches the tutorial."));
      identifier=0x9486;
   }
  
   lcd.begin(identifier);
   Serial.print("TFT size is "); Serial.print(lcd.width()); Serial.print("x"); Serial.println(lcd.height());
}
