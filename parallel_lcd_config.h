#ifndef PARALLEL_LCD_CONFIG_H
#define PARALLEL_LCD_CONFIG_H

   #include <Adafruit_GFX.h>    // Core graphics library
   #include <Adafruit_TFTLCD.h> // Hardware-specific library
   #include <MCUFRIEND_kbv.h>

   // The control pins for the LCD can be assigned to any digital or
   // analog pins...but we'll use the analog pins as this allows us to
   // double up the pins with the touch screen (see the TFT paint example).
   #define LCD_CS A3 // Chip Select goes to Analog 3
   #define LCD_CD A2 // Command/Data goes to Analog 2
   #define LCD_WR A1 // LCD Write goes to Analog 1
   #define LCD_RD A0 // LCD Read goes to Analog 0

   #define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

   // When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
   // For the Arduino Uno, Duemilanove, Diecimila, etc.:
   //   D0 connects to digital pin 8  (Notice these are
   //   D1 connects to digital pin 9   NOT in order!)
   //   D2 connects to digital pin 2
   //   D3 connects to digital pin 3
   //   D4 connects to digital pin 4
   //   D5 connects to digital pin 5
   //   D6 connects to digital pin 6
   //   D7 connects to digital pin 7
   // For the Arduino Mega, use digital pins 22 through 29
   // (on the 2-row header at the end of the board).

   // Assign human-readable names to some common 16-bit color values:
   const int BLACK   = 0x0000;
   const int BLUE    = 0x001F;
   const int RED     = 0xF800;
   const int GREEN   = 0x07E0;
   const int CYAN    = 0x07FF;
   const int MAGENTA = 0xF81F;
   const int YELLOW  = 0xFFE0;
   const int WHITE   = 0xFFFF;

   const int TFT_LCD           =   1;
   const int LCD_WIDTH         = 480;
   const int LCD_HEIGHT        = 320;
   const int LCD_HALF_WIDTH    = 240;
   const int LCD_QUARTER_WIDTH = 120;
   const int LCD_HALF_HEIGHT   = 160;
   const int LCD_TOP_ROW       =  64;
   const int LCD_BOTTOM_ROW    = 224;
   const int LCD_BORDER        =  20;

   const int INHALE_LED    = 21;         // GReen LED - ( LEDG_PIN_VIN on circuito.io )
   const int EXHALE_LED    = 20;         // Red LED - ( LEDR_PIN_VIN on circuito.io )
   const int ROTARY_CLK    = 24;         // Connected to CLK on KY-040 ( ROTARYENCI_PIN_CLK on circuito.io )
   const int ROTARY_DT     = 23;         // Connected to DT on KY-040 ( ROTARYENCI_PIN_D on circuito.io )
   const int SELECT_BUTTON = 22;         // Push switch built into the rotary encoder ( ROTARYENCI_PIN_S1 on circuito.io )
   const int SERVO_PIN     = 25;

   int respRate = 15;
   int cmH20 = 1;
   float iERatio = 100.0;
   int tidal = 100;

   MCUFRIEND_kbv lcd;
   
#endif
