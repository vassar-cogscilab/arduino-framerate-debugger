#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

int phase;
int SETUP = 0;
int MEASURE = 1;

int threshold;

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16,2);

  phase = SETUP;
  threshold = 512;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("CURR THRESH:");
  lcd.setCursor(0,1);
  lcd.print("CURR VALUE: ");
}

void loop() {
  uint8_t buttons = lcd.readButtons();
  int a = analogRead(A0);

  // SETUP phase
  // set threshold
  if(phase == SETUP){
    lcd.setCursor(12,0);
    lcd.print("    ");
    lcd.setCursor(12,0);
    lcd.print(threshold);

    lcd.setCursor(12,1);
    lcd.print("    ");
    lcd.setCursor(12,1);
    lcd.print(a);

    if (buttons) {
      if (buttons & BUTTON_UP) {
        threshold++;
        if(threshold > 1023) {
          threshold = 1023;
        }
      }
      if (buttons & BUTTON_DOWN) {
        threshold--;
        if(threshold < 0) {
          threshold = 0;
        }
      }
      if (buttons & BUTTON_SELECT) {
        lcd.clear();
        phase = MEASURE;
      }
    }
  }

  if(phase == MEASURE){
    
  }
}
