#ifndef I2C_LCD_CONFIG_H
#define I2C_LCD_CONFIG_H
#ifdef I2C_LCD

   #include <LiquidCrystal_I2C.h>

   // Set the LCD address to 0x27 for a 16 chars and 2 line display
   LiquidCrystal_I2C lcd(0x3f, 20, 4);
   
   const int LCD_BORDER     = 0;
   const int LCD_TOP_ROW    = 0;
   const int LCD_BOTTOM_ROW = 1;

   const int INHALE_LED    = 5;             // GReen LED - ( LEDG_PIN_VIN on circuito.io )
   const int EXHALE_LED    = 6;             // Red LED - ( LEDR_PIN_VIN on circuito.io )
   const int ROTARY_CLK    = 2;             // Connected to CLK on KY-040 ( ROTARYENCI_PIN_CLK on circuito.io )
   const int ROTARY_DT     = 3;              // Connected to DT on KY-040 ( ROTARYENCI_PIN_D on circuito.io )
   const int SELECT_BUTTON = 4;          // Push switch built into the rotary encoder ( ROTARYENCI_PIN_S1 on circuito.io )
   const int SERVO_PIN     = 7;
#endif
#endif
