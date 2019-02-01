// include the library code:
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

  // Storage for button status. 
  // Updated in: buttonCheck()
  // Used in: buttonCheck(), modeSwitch()
const byte bUp = 1;
const byte bDown = 2;
const byte bLeft = 3;
const byte bRight = 4;
const byte bSelect = 5;
byte static currButton = 0;

  // Volitile global variables for waveStartISR() and waveEndISR() Interrupt Service Routines (ISRs). ***All variables used in interrupt ISRs must be global and volatile.***
  // Updated in: waveStartISR(), waveEndISR(), waveReset()
  // Used in: waveStartISR(), waveEndISR(), ISRwaveCalc(), phaseMain(), periodMain()
unsigned long volatile waveStartTime = 0;           //Time micros for rising edge
unsigned long volatile waveEndTime = 0;             //Time micros for falling edge
unsigned long volatile waveStartLast = 0;           //Time micros last rising edge
unsigned long volatile wavePeriodLive[5] = {0,0xFFFFFFFF,0,0,0};        //{Period update: current, min, max, total for avg, count for avg}
unsigned long volatile wavePhaseLive[5] = {0,0xFFFFFFFF,0,0,0};         //{Phase update: current, min, max, total for avg, count for avg}
unsigned long volatile waveErrorCount = 0;          //Total number of time mismatch errors detected. 
bool volatile waveStartFlag = false;                //For checking active wave status and error correction.
bool volatile phaseUpdateFlag = false;              //For checking completed phase duration update status.
bool volatile periodUpdateFlag = false;             //For checking completed period duration update status.
bool volatile waveResetFlag = true;                 //For checking recent reset. Suppresses phase data update until second rising edge.   

  // Storage for ISRwave data. 
  // Updated in: ISRwaveCalc(), waveReset()
  // Used in: ISRwaveCalc(), waveReset(), phaseMain(), periodMain(), freqMain()
const byte xPhase =0;
const byte xPeriod =1;
const byte xFreq =2;
const byte xDuty = 3;
const byte xVal =0;
const byte xMin =1;
const byte xMax = 2;
const byte xAvg = 3;                  
    //All current phase, period, freq, and duty data. Initialized in setup() via waveReset(). 
                                                                            //            xVal, xMin, xMax, xAvg
float static ISRwaveData[4][4];                                             //xPhase    {     ,     ,     ,     }
                                                                            //xPeriod   {     ,     ,     ,     }
                                                                            //xFreq     {     ,     ,     ,     }
                                                                            //xDuty     {     ,     ,     ,     }
    //Tracks wave update state in ISRwaveCalc() to update display.
byte static waveStatus = 0;                                                 //0=Extended LOW, 1=Extended HIGH, 2=Recent Phase update

/*
  //Storage for ADCwave data
const byte minADC = 0;
const byte maxADC = 1;
const byte halfADC = 2;
int static ADCwaveData[3] = {1023,0,0};                       //{minADC, maxADC, halfADC}
int static ADCwaveToPWM = 0;                                  //Quarter wave height mapped to PWM value for threshold recommendation.
unsigned long static analogUpdateCount = 0;
*/

  //Storage for current interface mode. 
  //Updated in: modeSwitch()
  //Used in: modeSwitch(), modeLaunch(), phaseMain(), periodMain()
int static currMainMode = 1;                                                        //Store current main mode. 
const byte mainThresh = 0;                                                            //Threshold setting and signal min/max measurement
const byte mainPhase = 1;                                                             //Phase measurement mode
const byte mainPeriod = 2;                                                            //Period measurement mode
const byte mainFreq = 3;                                                              //Frequency measurement mode 
const byte mainDuty = 4;                                                              //Duty cycle measurement mode   
int static currSubMode = 3;                                                         //Store current sub mode. (Value sets boot default)
const byte subMin = 0;                                                                //Display min value
const byte subMax = 1;                                                                //Display max value
const byte subAvg = 2;                                                                //Display average value
const byte subModeTotal = 3;                                                          //Display total successful ISR updates since reset
const byte subModeErrors = 4;                                                         //Display total errors detected in ISRs since reset

  //Tells mode functions to print mode label to reduce unnecessary lcd writes. Must start TRUE
bool static modeSwitchFlag = true;                                                                                                                                        //For reducing unnecessary lcd print cycles. 

  //Threshold global variables
const byte threshOutPin = 6;                        //Threshold PWM output pin
const byte threshInPin = A1;                        //Threshold analog input sense pin
byte static threshOut;                              //Threshold PWM output setting. Initialized in setup(). 

  //Analog wave sense global variables
const byte analogWavePin = A0;                         //Analog wave sense pin


  //Delays for mode changes and status updates
const unsigned int modeSwitchDelay = 150;                    // Min millis between menu changes.
//const int modeSplashDelay = 1300;                   // Max millis to display mode config information on mode switch.
//const long modeSplashMax = 180000;                  // Max millis total run time to allow splash to display on mode switch. 
const int offResetDelay = 10000;                    // Delay millis to hold data on screen before displaying OFF message.
unsigned long static lastModeSwitch = 0;            //Millis since last mode switch. 


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  lcd.begin(16, 2);

  lcd.setCursor(0,0);
  lcd.print("Initializing");

    //Initialize threshold and analog wave sense pins
  pinMode(threshInPin, INPUT);
  pinMode(analogWavePin, INPUT);

    //Initialize threshold output. Delay 3sec to allow capacitor charing value stabilizing.
  pinMode(threshOutPin, OUTPUT);   
  threshOut = 75;
  analogWrite(threshOutPin, threshOut);
  delay(3000);

  
    // declare interrupt pins. Falling state initialized first to eliminate false updates when reset with active signal.
  pinMode(2, INPUT);
  pinMode(3, INPUT); 
  attachInterrupt(digitalPinToInterrupt(2), waveEndISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), waveStartISR, RISING);

    //Set default wave statistic values. Must be included in setup to initialize variables. 
  waveReset();
}


void waveStartISR(){
   // Start wave timing. Rising edge ISR. Digital pin 2
   // Update Start flag and time. Set stop flag to 0 for error reduction. 
   
  waveStartTime = micros();

  if(waveResetFlag == false){                                   //Skip first Period calc after reset to capture full rise to rise time. 
    if( waveStartTime > waveEndTime ){                            //Time comparison for error prevention
      
    wavePeriodLive[0] = (waveStartTime - waveStartLast);                //Update period lenght micros

          //Update period min
    if( wavePeriodLive[0] < wavePeriodLive[1] ){
      wavePeriodLive[1] = wavePeriodLive[0];
    }    
  
        //Update period max
    if( wavePeriodLive[0] > wavePeriodLive[2] ){
      wavePeriodLive[2] = wavePeriodLive[0];
    } 

      //Update running totals for averaging
    wavePeriodLive[3] += wavePeriodLive[0];
    wavePeriodLive[4]++;    

      //Update period comparision time. Set flag to trigger period update in ISRwaveCalc(). 
    waveStartLast = waveStartTime;                     
    periodUpdateFlag = true;
    }
    else{
    waveErrorCount++;                       //Increment error count
    }
  }
  else{
    waveStartLast = waveStartTime;           //Update time for first period calc after reset
  }
  
  waveStartFlag = true;                     //Set flag to be check in waveEndISR(). 
  
}

void waveEndISR(){
  // End wave timing. Falling edge ISR. Digital pin 3
  // Verify start flag and positive time difference. Update total wave time.

  waveEndTime = micros();

    //Update values after error prevention check
  if( (waveStartFlag == true) && (waveEndTime - waveStartTime >= 0) ){          //Check for rising edge update and compare times for error prevention

    wavePhaseLive[0] = (waveEndTime - waveStartTime);           //Update phase length micros  

      //Update phase min
    if( wavePhaseLive[0] < wavePhaseLive[1] ){
      wavePhaseLive[1] = wavePhaseLive[0];
    }    
  
        //Update phase max
    if( wavePhaseLive[0] > wavePhaseLive[2] ){
      wavePhaseLive[2] = wavePhaseLive[0];
    } 

      //Update running totals for averaging
    wavePhaseLive[3] += wavePhaseLive[0];
    wavePhaseLive[4]++;    

      //Set trigger flags
    phaseUpdateFlag = true;         //Tell ISRwaveCalc() to update data
    waveStartFlag = false;          //False until reset on next rising edge for error prevention
    waveResetFlag = false;          //Set false to indicate 
  } 
  else{
    waveStartFlag = false;          //Error detected. Reset rising edge flag until next update.
    waveErrorCount++;               //Increment error count
  }
}


void loop() {
  // put your main code here, to run repeatedly:

  ISRwaveCalc();              //Update all wave values measured from interrupts.
  buttonCheck();              //Check button state
  modeSwitch();               //Update UI if button state changes.  

        
}


void ISRwaveCalc(){
  //Recalculate wave data taken from ISR's if updated

  unsigned long static lastPhaseUpdate = 0;                                       //Time millis since last phase update
  unsigned long waveMicrosCopy[5] = {0,0xFFFFFFFF,0,0,0};                          //For quickly copying current data without conversion

  //Calculate wave phase times and statistics if update detected. 

    //Phase time and status display mode updates 
  if( phaseUpdateFlag == true ){

      //Copy new data and update reset flag. Interrupts disabled to prevent error. 
    noInterrupts();
    for (byte i=0; i<5; i++){
    waveMicrosCopy[i] = wavePhaseLive[i];       //Copy unsigned long to unsigned long to minimize ISR down time. 
    }
    phaseUpdateFlag = false;                   //Reset update flag
    interrupts(); 

      //Convert unsigned long micros to float millis
    for (byte i=0; i<4; i++){
    ISRwaveData[xPhase][i] = waveMicrosCopy[i];                       //Convert unsigned long (val, min, max, average) to float
    ISRwaveData[xPhase][i] = ISRwaveData[xPhase][i] * 0.001;            //Convert float micros to float millis
    }

      //Calculate average millis
    ISRwaveData[xPhase][xAvg] /= waveMicrosCopy[4];                   //Phase avg = total millis / total counts

      //Update status. 
    lastPhaseUpdate = millis();                                //Update time for wave status setting. 
    waveStatus = 2;                                            //Set wave status to active current readout
    phaseUpdateFlag = false;                                   //Clear update flag.
  }
  else{                                                        //Set flags for measurement status messages 

      //Check time since last update. Update wave status if necessary.
    if( (millis() - lastPhaseUpdate) >= offResetDelay ){        
      if ( waveStartFlag == true ){                               //Check if a rising edge has been detected without a falling edge to detect active measurement
        waveStatus = 1;                                           //Set status to MEASURING for extended active high
      }
      else{
        waveStatus = 0;                                           //Set status to OFF for extended low
      }      
    }
  }

 //Calculate wave period times, frequency, duty cycle and statistics if update detected. 

    //Period time, freq, and duty updates
  if( periodUpdateFlag == true ){
    
    noInterrupts();
    for (byte i=0; i<5; i++){
      waveMicrosCopy[i] = wavePeriodLive[i];       //Copy unsigned long to unsigned long to minimize ISR down time. 
    }
    periodUpdateFlag = false;
    interrupts();
    
      //Convert unsigned long micros to float millis
    for (byte i=0; i<4; i++){
      ISRwaveData[xPeriod][i] = waveMicrosCopy[i];                       //Convert unsigned long (val, min, max, average) to float
      ISRwaveData[xPeriod][i] = ISRwaveData[xPeriod][i] * 0.001;             //Convert float micros to float millis
    }

      //Calculate average period millis
    ISRwaveData[xPeriod][xAvg] /= waveMicrosCopy[4];                   //Phase avg = total millis / total counts


      //Update frequency data. Convert period to seconds and calculate frequency. Freq Hz = 1/ (period time in seconds).
    ISRwaveData[xFreq][xVal] = ( 1 / (ISRwaveData[xPeriod][xVal] * 0.001) );          //Current frequency Hz = 1/ (Current period time in seconds).
    ISRwaveData[xFreq][xMin] = ( 1 / (ISRwaveData[xPeriod][xMax] * 0.001) );          //Min frequency Hz = 1/ (Max period time in seconds). Freq and period are inversely related
    ISRwaveData[xFreq][xMax] = ( 1 / (ISRwaveData[xPeriod][xMin] * 0.001) );          //Max frequency Hz = 1/ (Min period time in seconds). Freq and period are inversely related       
    ISRwaveData[xFreq][xAvg] = ( 1 / (ISRwaveData[xPeriod][xAvg] * 0.001) );          //Average frequency Hz = 1/ (Average period time in seconds).


      //Update duty cycle data
    ISRwaveData[xDuty][xVal] = ( (ISRwaveData[xPhase][xVal] / ISRwaveData[xPeriod][xVal]) * 100 );            //Current positive Duty% = Current phase / Current period
    ISRwaveData[xDuty][xAvg] = ( (ISRwaveData[xPhase][xAvg] / ISRwaveData[xPeriod][xAvg]) * 100 );            //Average positive Duty% = Average phase / Average period. Overall averages used to capture general wave nature. 
    if( ISRwaveData[xDuty][xVal] < ISRwaveData[xDuty][xMin] ){                                                //Update Min. Discrete duty cycles must be taken from the same wave. Cannot use overall phase and period min
      ISRwaveData[xDuty][xMin] = ISRwaveData[xDuty][xVal]; 
    } 
    if( ISRwaveData[xDuty][xVal] > ISRwaveData[xDuty][xMax] ){                                                //Update Max. Discrete duty cycles must be taken from the same wave. Cannot use overall phase and period max
      ISRwaveData[xDuty][xMax] = ISRwaveData[xDuty][xVal]; 
    }
    

    periodUpdateFlag = false;                 //Clear update flag.
  }

}

/*   ADCwaveCalc
void ADCwaveCalc(){


  
}
*/

void waveReset(){
  //Reset all wave data to default values
  
  lcd.clear();
  
    //reset live capture values and set reset flag. Disable interrupts to prevent error
  noInterrupts();
     //Reset live capture Ulong data
  for( byte i=0; i<5; i++ ){
    
    if(i == 1){                               //Reset min capture data storage to max possible data value.
      wavePeriodLive[1] = 0xFFFFFFFF;
      wavePhaseLive[1] = 0xFFFFFFFF;
    } 
    else {                                    //Clear all other values
    wavePeriodLive[i] = 0;                    
    wavePhaseLive[i] = 0;
    }
  }

  waveResetFlag = true;                       //Set reset flag to prevent period calc updates until second rising edge.
  waveStartFlag = false;                      //Clear rising edge flag
  phaseUpdateFlag = false;                    //Clear phase update flag
  periodUpdateFlag = false;                   //Clear period update flag
  waveErrorCount = 0;                         //Clear error count

    //Clear external interrupt flags to prevent immediate launch of ISRs if interrupt request triggered while data was being reset.
    //Fixes device reset bug due to recurrsive calls of IRSs after data reset.   
  EIFR = 0x03;                                //Write logical 1 to INTF1 and INTF0 bits of EIFR (External Interrupt Flag Register). Atmega328P datasheet page 55 for details.  
  interrupts();

    //Reset stored float millis data
  for( byte i=0; i<4; i++ ){
    ISRwaveData[i][xVal] = 0.00;
    ISRwaveData[i][xMin] = 3.4028235E+38;        //Reset min capture data storage to max possible data value.
    ISRwaveData[i][xMax] = -3.4028235E+38;       //Reset max capture data storage to min possible data value.
    ISRwaveData[i][xAvg] = 0.00;
  }
  
}

   
void buttonCheck() {
  //return button value as byte variable
  
  uint8_t buttons = lcd.readButtons();
  
  if (buttons != 0){
    if (buttons & BUTTON_UP) {
      currButton = bUp;
    }
    if (buttons & BUTTON_DOWN) {
      currButton = bDown;
    }
    if (buttons & BUTTON_LEFT) {
      currButton = bLeft;
    }
    if (buttons & BUTTON_RIGHT) {
      currButton = bRight; 
    }
    if (buttons & BUTTON_SELECT) {
      currButton = bSelect;
    }
  }
  else {
    currButton = 0; 
  }

}


void modeSwitch(){
  // Loop through modes or reset wave stats with buttons. Clear display after any button press. Maintain currMainMode else. 
    //Button functions: (bRight = Main++), (bLeft = Main--), (bSelect = Reset stats). 
                                                   
  const byte mainModeList[] = {mainThresh, mainPhase, mainPeriod, mainFreq, mainDuty};            //List and order of main modes to cycle with left/right
      
  if( currButton != 0 ){
    if( millis() - lastModeSwitch >= modeSwitchDelay){      //Check if minimum switch delay is met for more controlled switching. 
      switch (currButton){
        case bRight:
              currMainMode++ ;
              if ( currMainMode > (sizeof(mainModeList) - 1) ){
                currMainMode = 0;
              }
              lastModeSwitch = millis();                          //Reset mode switch reference time
              lcd.clear();                                        //Clear display
              modeSwitchFlag = true;                              //Trigger mode label reprint  
              break;
        case bLeft:
              currMainMode-- ;                
              if (currMainMode < 0){
                currMainMode = (sizeof(mainModeList) - 1);
              }
              lastModeSwitch = millis();                          //Reset mode switch reference time
              lcd.clear();                                        //Clear display
              modeSwitchFlag = true;                              //Trigger mode label reprint  
              break;
        case bSelect:
              waveReset();
              lastModeSwitch = millis();                          //Reset mode switch reference time
              lcd.clear();                                        //Clear display
              modeSwitchFlag = true;                              //Trigger mode label reprint  
              break;
      }
    currMainMode = mainModeList[currMainMode];          //Set main mode changes
    }
  }

 
    //Begin function associated with main mode number. Run splash display for set hold time, clear after. 
  switch (currMainMode){
    case mainThresh:
          threshMain();
          break;
    case mainPhase:
          ppfdMain();
          break;
    case mainPeriod:
          ppfdMain();
          break;
    case mainFreq:
          ppfdMain();
          break;
    case mainDuty:
          ppfdMain();
          break;
  }
  return;
}


int subSwitch2(int currSubVal = 0, int maxSubVal = 0, int minSubVal = 0){
  // Loop through sub modes with buttons. Maintain currSub else. Cycle speed increased once if held 
    //Button functions: (bUp = Sub++), (bDown = Sub--) 
    //Takes current sub mode and total number of sub modes from passing function. Returns updated current sub mode. 
  
  
  unsigned int static holdCycles = 0;           //Current count of cycles while button is held
  byte static subValChange = 1;                 //Number to increment/decrement subVal. Increases if hold cycle check passes. 
  byte const cyclesBeforeBoost = 30;            //Number of cycles to count before boosted speed begins. 
  
  
    //Set subValChange based on hold cycle count. Increases value change speed if button is held. 
  if (holdCycles > cyclesBeforeBoost ){
    subValChange = 10;
  }
  else {
    subValChange = 1;
  }

    //Check button state and if minimum switch delay. Update change control variables if button pressed. Reset hold cycle count else. 
  if( currButton != 0 ){
    if( millis() - lastModeSwitch > modeSwitchDelay ){     //Check if minimum time has been met for deboucing. 
      switch (currButton){                                              
        case bUp:                                                       //Increment currSubVal inside valid range
              currSubVal += subValChange;
              if ( currSubVal > maxSubVal ){
                currSubVal = minSubVal;
              }
              break;
        case bDown:                                                     //Decrement currSubVal inside valid range
              currSubVal -= subValChange;
              if ( currSubVal < minSubVal ){
                currSubVal = maxSubVal;
              }
              break;
      }
    lastModeSwitch = millis();                          //Reset mode switch reference time
    modeSwitchFlag = true;                              //Trigger mode label reprint 
    holdCycles++;                                       //Increment hold cycles for speed boost
    }    
  }
  else {                                               //Reset hold cycles if currButton == 0
    holdCycles = 0;
  }
 return currSubVal;                                  //Pass updated value to previous function. 
}


void threshMain(){

  String stCurrPWM;
  String stCurrADC;
  byte stCurrLengthPWM = 0;
  byte stCurrLengthADC = 0;
  byte static stPrevLengthPWM = 0;
  byte static stPrevLengthADC = 0;

    //Print mode label if mode has changed.  Set in modeSwitch().
  if(modeSwitchFlag == true){
  lcd.setCursor(0,0);
  lcd.print("Thresh PWM:");
  lcd.setCursor(0,1);
  lcd.print("Thresh ADC:");
  }

    //Set string values for printing
  stCurrPWM = String(threshOut);
  stCurrADC = String(analogRead(threshInPin));


    //Update current value string length. Clear value display if character length decreased. 
    //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)
  stCurrLengthPWM = stCurrPWM.length();
  stCurrLengthADC = stCurrADC.length();
  if (stCurrLengthPWM < stPrevLengthPWM){
    lcd.setCursor(11,0);
    lcd.print("     ");
  }
  if (stCurrLengthADC < stPrevLengthADC){
    lcd.setCursor(11,1);
    lcd.print("     ");
  }

    //Print string values
  lcd.setCursor(11,0);
  lcd.print(stCurrPWM);
  lcd.setCursor(11,1);
  lcd.print(stCurrADC);

  stPrevLengthPWM = stCurrLengthPWM;
  stPrevLengthADC = stCurrLengthADC;


  threshOut = subSwitch2(threshOut, 255, 0);             //Update threshold setting with Up/Down buttons. max value 250, min value 10

/*
    //Check for threshold setting updates
  if( currButton != 0 ){
    if( millis() - lastModeSwitch >= modeSwitchDelay){      //Check if minimum switch delay is met for more controlled switching. 
      switch (currButton){
        case bUp:
              if (threshOut != 255){
                threshOut++;
              }
              break;
        case bDown:
              if (threshOut != 0){
                threshOut--;
              }
              break;
      }
    analogWrite(threshOutPin, threshOut);             //Set sub mode changes
    lastModeSwitch = millis();
        
    }
  }

*/

  analogWrite(threshOutPin, threshOut);             //Set sub mode changes
  
  
    //Prevent label from reprinting until next mode change. 
  modeSwitchFlag = false;
  return;
}


void ppfdMain(){
  //Phase, Period, Frequency, and Duty modes top print line display settings

    //Store previous and current value string lengths. For clearing field if length reduces. 
  byte static stPrevMainLength = 0;
  String stMainVal;
  byte stMainLength;
  byte static cursorMain;
  byte static deciMain;
  byte static deciSub;
  byte static currModeVal;
  

  
    //Print mode label if mode has changed.  Set in modeSwitch(). reset in sub mode function after full print completed. 
  if(modeSwitchFlag == true){

    switch (currMainMode){
      case mainPhase:
            lcd.setCursor(0,0);
            lcd.print("Phase mS:");
            cursorMain = 9;
            deciMain = 2;
            deciSub = 3;
            currModeVal = xPhase;
            break;
      case mainPeriod:
            lcd.setCursor(0,0);
            lcd.print("Period mS:");
            cursorMain = 10;
            deciMain = 2;
            deciSub = 3;
            currModeVal = xPeriod;
            break;
      case mainFreq:
            lcd.setCursor(0,0);
            lcd.print("Freq. Hz:");
            cursorMain = 9;
            deciMain = 0;
            deciSub = 0;
            currModeVal = xFreq;
            break;
      case mainDuty:
            lcd.setCursor(0,0);
            lcd.print("+Duty %:");
            cursorMain = 8;
            deciMain = 1;
            deciSub = 1;
            currModeVal = xDuty;
            break;
      default:
            lcd.setCursor(0,0);
            lcd.print("Phase mS:");
            cursorMain = 9;
            deciMain = 2;
            deciSub = 3;
            currModeVal = xPhase;
            break;
    }
  }

    //Set main value string based on current waveStatus (set in ISRwaveCalc())
  switch (waveStatus){
    case 0:
          stMainVal = "_LOW_";
          break;
    case 1:
          stMainVal = "-HIGH-";
          break;
    default: 
          stMainVal = String(ISRwaveData[currModeVal][xVal], deciMain);
          break; 
  }


    //Update current value string length. Clear value display if character length decreased. 
    //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)
  stMainLength = stMainVal.length();
  if ( stMainLength < stPrevMainLength ){
    lcd.setCursor(cursorMain,0);
    lcd.print("        ");
  }


    //Print value string
  lcd.setCursor(cursorMain,0);
  lcd.print(stMainVal);

    //Save current string length for comparision on next loop. 
  stPrevMainLength = stMainLength;  


  ppfdSub(currModeVal, deciSub);
  
}


void ppfdSub(byte currModeVal, byte deciSub){
  //Phase, Period, Frequency, and Duty modes bottom print line display settings

  byte static stPrevSubLength = 0;
  String stSubVal;
  byte stSubLength;
  byte static cursorSub;
  unsigned long totalCounts;

  if (currMainMode == mainPhase){
    totalCounts = wavePhaseLive[4];
  }
  else{
    totalCounts = wavePeriodLive[4];
  }

  currSubMode = subSwitch2(currSubMode, 4, 0);
  
    //Print mode label and set following value cursor position variable if mode has changed. 
  if( modeSwitchFlag == true ){
    lcd.setCursor(0,1);
    switch (currSubMode){
      case subMin:
            lcd.print("Min:            ");
            cursorSub = 4;        
            break;
      case subMax:
            lcd.print("Max:            ");
            cursorSub = 4;
            break;
      case subAvg:
            lcd.print("Avg:            ");
            cursorSub = 4;
            break;
      case subModeTotal:
            lcd.print("Total:          ");
            cursorSub = 6;
            break;
      case subModeErrors:
            lcd.print("Errors:         ");
            cursorSub = 7;
            break;  
    }
    
    
  }

    //Set string output value based on current sub mode.
      //String set to "0" for Min and Max if no updates have been completed to prevent string length overflow from max pos/neg float values. 
  switch (currSubMode){
    case subMin:
          if (waveResetFlag == false){
            stSubVal = String(ISRwaveData[currModeVal][xMin], deciSub);
          }
          else{
            stSubVal = "0";
          }
          break;
    case subMax:
          if (waveResetFlag == false){
            stSubVal = String(ISRwaveData[currModeVal][xMax], deciSub);
          }
          else{
            stSubVal = "0";
          }
          break;
    case subAvg:
          stSubVal = String(ISRwaveData[currModeVal][xAvg], deciSub);
          break;
    case subModeTotal:
          stSubVal = String(totalCounts);
          break;
    case subModeErrors:
          stSubVal = String(waveErrorCount);
          break;  
  }


    //Update current value string length. Clear value display if character length decreased. 
    //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)
  stSubLength = stSubVal.length();
  if(stSubLength < stPrevSubLength){
    lcd.setCursor(cursorSub, 1);
    lcd.print("            ");
  }

    //Set cursor to match mode label. Print value. 
  lcd.setCursor(cursorSub, 1);
  lcd.print(stSubVal);

    //Update previous string length for next cycle. 
  stPrevSubLength = stSubLength;
  
    //Prevent label from reprinting until next mode change. 
  modeSwitchFlag = false;                   
  
  return;
  
}

/*
 * 
 * *****Notes: *********
 * Verify min/max for ISRwaveData can be initialized as 0.00 after updating ISRwaveCalc behavior
 * Add analog wave calc values to reset function
 * 
 * 
 * 
 ********** Prototype frame count code: *************
 * 
 *  //Counts for given frame count ± target
 * unsigned long volatile frameCount[9]{0,0,0,0,0,0,0,0,0};       // Count totals: {<f-3, f-3, f-2, f-1, f, f+1, f+2, f+3, >f+3}
 * 
 *  // Length comparisons to target frame count(f). 
 * unsigned long volatile frameLength[8]{0,0,0,0,0,0,0,0};        //{f-3 min, f-3 max, f-2 max, f-1 max, f max, f+1 max, f+2 max, f+3max}  
 * 
 * unsigned int static frameFreq = 60;                    // Store monitor frame rate
 * byte static frameTarget = 1;                           // Store target frame count for comparison
 * 
 * 
 * unsigned long frameSingle = (1/frameFreq)*1000000);    //Convert frameFreq to length in microseconds 
 * unsigned long frameBuffer = (frameSingle * 0.15);      //15% buffer for response time and trigger level tolerance
 * 
 * 
 * 
 *  //Set minimum value for one frame under target. Balanced for worst case with 15% shorter frame and no ISR delay.
 * frameLength[0] = ( frameSingle * (frameTarget - 1) ) - ( frameBuffer);                                                                                                                                                                 
 * 
 * 
 *  // Set maximum value for frames ±1 from target. Balanced for worst case with 15% longer frame and 8uS ISR launch delay. 
 *  
 *  
 * frameLength[1] = ( frameSingle * (frameTarget - 1) ) + frameBuffer + 8;                                                                                
 * frameLength[2] = ( frameSingle * frameTarget )       + frameBuffer + 8;  
 * frameLength[3] = ( frameSingle * (frameTarget + 1) ) + frameBuffer + 8;  
 * 
 * 
 * 
 * if (frameTarget - i) < 0){
 *  frameLength[i] = 0;
 * }else if ( (frameTarget - i) = 0 ){
 *  frameLength[i] = frameSingle - frameBuffer;
 * }else{
 *  frameLength[i] = ( frameSingle * (frameTarget - i) ) + frameBuffer + 8;
 * 
 * if ( i == 
 * 
 * 
 * 
 *  //Increment appropriate frame count. 
 *  //Check target frame conditions first to limit conditional checks if on target. 
 * if ( (wavePeriodLive[0] > frameLength[1]) && (wavePeriodLive[0] <= frameLength[2]) ){      //Between +buffered target-1 and +buffered target
 *  frameCount[2]++;                                                                          //target count++
 * }
 * else if (wavePeriodLive[0] < frameLength[0]){                                             //Less than -buffered target-1
 *  frameCount[0]++;                                                                          // <-1 target count++ 
 * }
 * else if (wavePeriodLive[0] <= frameLength[1]){                                            //Between -buffered target-1 and +buffered target-1
 *  frameCount[1]++;                                                                          // -1 target count++
 * }
 * else if (wavePeriodLive[0] <= frameLength[3]){                                            //Between +buffered target and +buffered target+1
 *  frameCount[3]++;                                                                          // +1 target count++
 * }
 * else {                                                                                   //Greater than +buffered target+1
 *  frameCount[4]++;                                                                          // >+1 target count++
 * }
 * 
 */
