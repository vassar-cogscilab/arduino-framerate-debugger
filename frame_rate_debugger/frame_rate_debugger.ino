#include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

int phase;
int SETUP = 0;
int MEASURE = 1;

int screen;
int SCREEN_ON = 0;
int SCREEN_OFF = 1;

int threshold;

// for measuring
int screen_onset_time;

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

void phase_setup() {
  uint8_t buttons = lcd.readButtons();
  int a = analogRead(A0);
  
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
      setup_measure_phase();
      phase = MEASURE;
    }
  }
}

void setup_measure_phase() {
  lcd.setCursor(0,0);
  lcd.print("LAST FRAME: ");
  screen = SCREEN_OFF;
}

void phase_measure() {
  //uint8_t buttons = lcd.readButtons();
  int a = analogRead(A0);
  lcd.setCursor(0,1);
  if(a >= threshold){
    lcd.print("ON ");
  } else {
    lcd.print("OFF");
  }
  if(screen == SCREEN_ON){
    if(a < threshold){
      int diff = millis() - screen_onset_time;
      lcd.setCursor(12,0);
      lcd.print("    ");
      lcd.setCursor(12,0);
      lcd.print(diff);
      screen = SCREEN_OFF;
    }
  } else if(screen == SCREEN_OFF){
    if(a >= threshold){
      screen_onset_time = millis();
      screen = SCREEN_ON;
    }
  }
}

void loop() {
  // SETUP phase
  // set threshold
  if(phase == SETUP){
    phase_setup();
  }

  if(phase == MEASURE){
    phase_measure();
  }
}
