// IMPORTANT: Adafruit_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h FOR SETUP.
//Technical support:goodlcd@163.com

#include "parallel_lcd_config.h"

void setup(void)
  {
  initialiseLcd();
  }

void loop(void) 
{
  updateDisplay();
  delay(1000);delay(1000);delay(1000);delay(1000);delay(1000);
  changeSettings();
  delay(1000);delay(1000);delay(1000);delay(1000);delay(1000);
}


void updateDisplay()
{
    lcd.setRotation(1);
    lcd.fillScreen(BLUE);
    
    lcd.fillRect(0, LCD_HALF_HEIGHT, LCD_WIDTH, 5, WHITE);
    lcd.fillRect(LCD_HALF_WIDTH, 0, 5,LCD_HEIGHT, WHITE);

    lcd.setTextColor(WHITE);
    lcd.setTextSize(4);
  
    lcd.setCursor(LCD_BORDER, LCD_TOP_ROW);
    lcd.print(respRate);
    lcd.setCursor(LCD_QUARTER_WIDTH, LCD_TOP_ROW);
    lcd.print("bpm");

#if 1
    lcd.setCursor((LCD_QUARTER_WIDTH) + (LCD_HALF_WIDTH), LCD_TOP_ROW);
    lcd.println("-");
#else
    lcd.setCursor((LCD_HALF_WIDTH) + LCD_BORDER, LCD_TOP_ROW);
    lcd.print(inspPressure);
    lcd.setCursor((LCD_QUARTER_WIDTH) + (LCD_HALF_WIDTH), LCD_TOP_ROW);
    lcd.print("cmH20");
#endif
    lcd.setCursor(LCD_BORDER, LCD_BOTTOM_ROW);
    lcd.print("1:");
    lcd.print(iERatio);

    lcd.setCursor((LCD_HALF_WIDTH) + LCD_BORDER, LCD_BOTTOM_ROW);
    lcd.println(tidal);
    lcd.setCursor((LCD_QUARTER_WIDTH) + (LCD_HALF_WIDTH), LCD_BOTTOM_ROW);
    lcd.println("ml");
}

void changeSettings() {

//  lcd.clear();
    lcd.fillScreen(BLUE);
    lcd.setTextColor(WHITE);
    lcd.setTextSize(4);

  // We don't actualy update the control parameters directly .. this is done within the control loop at the end of a cycle
  // What we do here is to collect an updated set of parameters, and set a semaphore to indicate to the control loop to pick them up


  // Next section triggers device calibration .. but only for non-production (test code).
//  if (not PRODUCTIION_CODE) {
//    long int timeSelected = millis();                                          // Counts the number of loops that the select button has to be pushed for, before going into calibration mode
//    int selectButtonData = digitalRead(SELECT_BUTTON);
//    if (selectButtonData == LOW) {
      lcd.setCursor(LCD_BORDER, LCD_TOP_ROW);
      lcd.print("Keep button pushed to calibrate");
//    }
//    while ((selectButtonData == LOW) && ((millis() - timeSelected) < ENTER_CALIBRATION )) {
//      selectButtonData = digitalRead(SELECT_BUTTON);
//    }
//    if ((selectButtonData == LOW) && ((millis() - timeSelected) >= ENTER_CALIBRATION )) {
//      calibrate();
//    }
//  }

delay(1000);delay(1000);


  // Serial.println("Respiratory rate");
//  lcd.clear();
  lcd.fillScreen(BLUE);
  lcd.setCursor(LCD_BORDER, LCD_TOP_ROW);
  lcd.print("Respiratory rate:");
//  newRespRate = int(getKnob(RESP_RATE_MIN, RESP_RATE_MAX, respRate, RESP_RATE_STEP, "bpm"));
//
  // Serial.println("Inspiratory Pressure");               // Not implemented in this version
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("Inspiratory Press:");
  // newInspPressure = int(getKnob(INSP_PRESS_MIN, INSP_PRESS_MAX, inspPressure, 1.0, "cmH20"));

delay(1000);delay(1000);

  // Serial.println("I.E. Ratio");
//  lcd.clear();
  lcd.fillScreen(BLUE);
  lcd.setCursor(LCD_BORDER, LCD_TOP_ROW);
//  newIERatio = getKnob(I_E_RATIO_MIN, I_E_RATIO_MAX, iERatio, I_E_RATIO_STEP, "");


  // Serial.println("TIDAL");
//  lcd.clear();
  lcd.fillScreen(BLUE);
  lcd.setCursor(LCD_BORDER, LCD_TOP_ROW);
  lcd.print("TIDAL:");
//  newTidal = int(getKnob(TIDAL_MIN, TIDAL_MAX, tidal, TIDAL_STEP, "ml"));

  // Now signal to the interrup-driven control loop that it can pick up the new control parameters when it is ready to do to (at the start of a cycle)
//  Timer1.stop();                   // Halt the interrupt so that there is no chance of a race condition
 // paramUpdateSemaphore = true;     // Inform the main control loop that it can pick up the new control parameters
//  Timer1.start();                  // Re-start the main control-loop interrupt
delay(1000);delay(1000);

//  lcd.clear();
  lcd.fillScreen(BLUE);
  lcd.setCursor(LCD_BORDER, LCD_TOP_ROW);
  lcd.print("Adjusting ...");

delay(1000);delay(1000);
//  while (paramUpdateSemaphore) {};  // wait until this update has been picked up

  // Finished sending updated parameters to control loop


//  lcd.clear();
  updateDisplay();

//  calcTicksPerCycle(respRate, iERatio );           // Update the breath cycles based on the new settings of RR and I.E. ratio

//  changeTime = millis(); // manage the keyboard de-bounce
}
