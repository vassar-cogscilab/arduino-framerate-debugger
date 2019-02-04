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

  // Volatile global variables for waveStartISR() and waveEndISR() Interrupt Service Routines (ISRs). ***All variables used in interrupt ISRs must be global and volatile.***
  // Updated in: waveStartISR(), waveEndISR(), waveReset()
  // Used in: waveStartISR(), waveEndISR(), ISRwaveCalc(), phaseMain(), periodMain()
unsigned long volatile waveStartTime = 0;           //Time micros for rising edge
unsigned long volatile waveEndTime = 0;             //Time micros for falling edge
unsigned long volatile waveStartLast = 0;           //Time micros last rising edge
unsigned long volatile wavePeriodLive[5] = {0,0xFFFFFFFF,0,0,0};        //{Period update: current, min, max, total for avg, count for avg}
unsigned long volatile wavePhaseLive[5] = {0,0xFFFFFFFF,0,0,0};         //{Phase update: current, min, max, total for avg, count for avg}
unsigned long volatile waveErrorCount = 0;          //Total number of time mismatch errors detected. 
bool volatile waveStartFlag = false;                //For checking active wave status and error detection.
bool volatile waveEndFlag = false;                  //For error detection
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
byte static waveStatus = 0;                               //0=Extended LOW, 1=Extended HIGH, 2=Recent Phase update
bool static calcUpdateFlag = false;                       //True if ISRwaveData values updated since reset. 
                                                            //Used to prevent min/max float values causing string overflow and program crash. 

/*
  //Storage for ADCwave data
const byte minADC = 0;
const byte maxADC = 1;
const byte halfADC = 2;
int static ADCwaveData[3] = {1023,0,0};                       //{minADC, maxADC, halfADC}
int static ADCwaveToPWM = 0;                                  //Quarter wave height mapped to PWM value for threshold recommendation.
unsigned long static analogUpdateCount = 0;
*/


  //Tells mode functions to print mode label to reduce unnecessary lcd writes. Must start TRUE
bool static modeSwitchFlag = true;                  //For reducing unnecessary lcd print cycles. 

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

/* Notes on interrupt logic and control requirements
 *  
 *  
 * ***Interrupt handling behavior in microcontroller and code*** 
 *  
 * If an interrupt request triggers during normal conditions, the microcontroller will just to the ISR as soon as the present machine code operation has completed and resume the next machine code after ISR is completed. 
 * If an interrupt request triggers while interrupts are disabled (time between "noInterrupts();" and "interrupts();"), the ISR will launch immediately after interrupts are re-enabled. 
 * If multiple interrupt requests are placed while interrupts are disabled, ISR's execute in order of their priority. 
 * Interrupts are disabled automatically while an ISR is running and re-enabled upon completion. 
 * If additional and/or multiple interrupt requests trigger while an ISR is running, newly requested ISRs will run sequentially in priority order until all request are cleared.   
 *  
 * Microcontroller reset request has highest priority (IRQ priority 0). 
 * Falling edge interrupt has higher priority (INT0, IRQ priority 1) than rising edge (INT1, IRQ priority 2). 
 * This will improve phase time accuracy by updating data as quickly as possible and preventing errors in calculation if rising and falling both trigger while interrupts are disabled.  
 *  
 * All global variables used in ISR must be declared volatile. 
 * This tells the compiler they must be stored in and addressed from program DDR RAM (as opposed to a SD RAM processor cache register) at all times because they can be updated at any time. 
 * This also prevents them from being optimized away by the compiler if it does not appear they would be updated within the loop. 
 * Volatile variable >1byte should be addressed "atomically" (interrupts detached or disabled) to prevent error in value if ISR is called while variable is being used. 
 * Volatile variables take longer to address (DDR RAM speed vs SD cache speed), so making a standard data type copy can allow for faster successive calls and further manipulation at the cost of storage and real time accuracy. 
 *  
 * ***ISR reset detection logic.***  
 * 
 * waveReset() resets default values and sets waveResetFlag. 
 *   Interrupts disabled during reset. Interrupt request flags are cleared to prevent immediate launch of ISR if edge detected while interrupts disabled. 
 * 
 * First interrupt after reset: 
 * 
 *  If rising edge: 
 *    waveStartISR fails waveResetFlag conditional check. 
 *    Else condition clears waveResetFlag and sets waveStartFlag allowing waveEndISR to pass conditional checks at next falling edge. 
 *    Wave start time saved for the next period calculation. 
 *  If falling edge:
 *    waveEndISR fails waveResetFlag conditional check. 
 *    Set waveEndFlag to allow waveStartISR to begin updating after waveResetFlag has cleared.  
 * 
 * This prevents an update to period and phase data until after the first rising edge is detected so calculations are not made with old data. 
 * 
 * ***ISR wave calculation and error detection logic. (After waveResetFlag has been cleared) ***
 *   
 * waveStartISR: 
 *   If waveEndFlag check passes:
 *      Update start time for current period and next phase calculations. 
 *      Update period calculations. 
 *      Set waveStartFlag for waveEndISR error check and wave status update in loop.  
 *      Set periodUpdateFlag to trigger further calculation in loop. 
 *      Clear waveEndFlag for waveStartISR error detection. 
 *      Save start time for next period calculation. 
 *   If waveEndFlag fails: 
 *      Error detected. Two rising edges detected without a falling edge. 
 *      Save start time for next period calculation. 
 * waveEndISR:
 *   If waveStartFlag check passes:
 *      Update phase and frame calculations. 
 *      Set waveEndFlag for waveStartISR error check.  
 *      Clear waveStartFlag for waveEndISR error detection. 
 *   If waveStartFlag fails: 
 *      Error detected. Two falling edges detected without a rising edge. 
 * 
 */

void waveStartISR(){
   // Start wave timing for phase and period calculations. Rising edge ISR. Digital pin 2 
   
  waveStartTime = micros();                

    //Check if waveResetFlag has been cleared. If flag still true, only store time for next cycle calculation. 
  if(waveResetFlag == false){              
    
      //Check flag to detect error. Update period times if passes. 
      //Else increment error count. Two rising edges triggered without a falling edge. 
    if(waveEndFlag == true){

        //Update period length micros
      wavePeriodLive[0] = (waveStartTime - waveStartLast);                
  
        //Update period min
      if(wavePeriodLive[0] < wavePeriodLive[1]){
        wavePeriodLive[1] = wavePeriodLive[0];
      }    
    
        //Update period max
      if(wavePeriodLive[0] > wavePeriodLive[2]){
        wavePeriodLive[2] = wavePeriodLive[0];
      } 
  
        //Update running totals for averaging
      wavePeriodLive[3] += wavePeriodLive[0];
      wavePeriodLive[4]++;    
  
        //Update trigger flags and time for next period calculation.   
      waveStartLast = waveStartTime;          //Store time for next period calculation           
      periodUpdateFlag = true;                //Set flag to trigger update in IRSwaveCalc()
      waveEndFlag = false;                    //False until reset on next falling edge for error prevention  
      
    }else{
      waveErrorCount++;                       //Increment error count
      waveStartLast = waveStartTime;          //Update time for next period calculation
    }
  }else{
    waveStartLast = waveStartTime;            //Update time for first period calc after reset
    waveResetFlag = false;                    //Clear reset flag to allow new updates.  
  }
  
  waveStartFlag = true;                       //Set flag to be check in waveEndISR(). 
  
}

void waveEndISR(){
  // End wave timing for phase and frame calculations. Falling edge ISR. Digital pin 3

  waveEndTime = micros();

  if(waveResetFlag == false){

      //Check flag to detect error. Update phase times if passes. 
      //Else increment error count. Two falling edges triggered without a rising edge. 
    if(waveStartFlag == true){

        //Update phase length micros
      wavePhaseLive[0] = (waveEndTime - waveStartTime);             
  
        //Update phase min
      if(wavePhaseLive[0] < wavePhaseLive[1]){
        wavePhaseLive[1] = wavePhaseLive[0];
      }    
    
        //Update phase max
      if(wavePhaseLive[0] > wavePhaseLive[2]){
        wavePhaseLive[2] = wavePhaseLive[0];
      } 
  
        //Update running totals for averaging
      wavePhaseLive[3] += wavePhaseLive[0];
      wavePhaseLive[4]++;    
  
        //Update trigger flags
      phaseUpdateFlag = true;         //Tell ISRwaveCalc() to update data 
      waveStartFlag = false;          //False until reset on next rising edge for error prevention  
    }else{
      waveErrorCount++;               //Increment error count
    }
  }
  
  waveEndFlag = true;                 //Set flag to be check in waveStartISR(). 
  
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

      //Update status and refresh counts. 
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
    

      //Update period refresh count
    calcUpdateFlag = true;                    //Set flag for min/max value display updates
    periodUpdateFlag = false;                 //Clear update flag.
  }

}

/*   ADCwaveCalc
void ADCwaveCalc(){
  unsigned long startTime;
  unsigned int currAnaWave = 0;
  unsigned int minAnaWave = 0xFFFF;
  unsigned int maxAnaWave = 0; 
  const byte sampleMillis = 100; 

  String stCurrMin;
  String stCurrMax;
  byte stCurrMinLength;
  byte stCurrMaxLength;
  byte static stPrevMinLength = 0;
  byte static stPrevMaxLength = 0;

  startTime = millis();
  
    //Sample analog wave and check for button press. 
  while( (currButton != 0) && (millis() - startTime < sampleMillis) ){

    currAnaWave = analogRead(analogWavePin);

    if (currAnaWave > maxAnaWave){
      maxAnaWave = currAnaWave;
    }
    
    if (currAnaWave < minAnaWave){
      minAnaWave = currAnaWave;
    }

    buttonCheck();
  }

    //Respond to mode change and reset requests. 
  if ( (currButton == bLeft) || (currButton == bRight) || (currButton == bSelect) ){
    modeSwitch();
  }

    //Print mode label if mode has changed.  Set in modeSwitch().
  if(modeSwitchFlag == true){
  lcd.setCursor(0,0);
  lcd.print("Wave +Peak:");
  lcd.setCursor(0,1);
  lcd.print("Wave -Peak:");
  }
  

      //Set string values for printing
  stCurrMax = String(maxAnaWave);
  stCurrMin = String(minAnaWave);


    //Update current value string length. Clear value display if character length decreased. 
    //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)
  stCurrMaxLength = stCurrMax.length();
  stCurrMinLength = stCurrMin.length();
  if (stCurrMaxLength < stPrevMaxLength){
    lcd.setCursor(11,0);
    lcd.print("     ");
  }
  if (stCurrMinLength < stPrevMinLength){
    lcd.setCursor(11,1);
    lcd.print("     ");
  }

    //Print string values
  lcd.setCursor(11,0);
  lcd.print(stCurrMax);
  lcd.setCursor(11,1);
  lcd.print(stCurrMin);

  stPrevMaxLength = stCurrMaxLength;
  stPrevMinLength = stCurrMinLength;

    //Prevent label from reprinting until next mode change. 
  modeSwitchFlag = false;    
}
*/

void waveReset(){
  //Reset all wave data to default values
  
  lcd.clear();
  
    //reset live capture values and set reset flag. Disable interrupts to prevent error
  noInterrupts();
  
     //Reset live capture Ulong data. Intentionally verbose to eliminate conditional checks and minimize interrupt down time. 
  wavePhaseLive[0] = 0;                       //Phase current val
  wavePhaseLive[1] = 0xFFFFFFFF;              //Phase min val. Set to max possible to insure update
  wavePhaseLive[2] = 0;                       //Phase max val.
  wavePhaseLive[3] = 0;                       //Phase running total micros for averaging in ISRwaveCalc()
  wavePhaseLive[4] = 0;                       //Phase running total updates for averaging in ISRwaveCalc() and total display in ppfdSub()
  wavePeriodLive[0] = 0;                      //Period current val
  wavePeriodLive[1] = 0xFFFFFFFF;             //Period min val. Set to max possible to insure update
  wavePeriodLive[2] = 0;                      //Period max val.
  wavePeriodLive[3] = 0;                      //Period running total micros for averaging in ISRwaveCalc()
  wavePeriodLive[4] = 0;                      //Period running total updates for averaging in ISRwaveCalc() and total display in ppfdSub()

    //Reset IRS flags and error counts
  waveResetFlag = true;                       //Set reset flag to prevent period calc updates until second rising edge.
  waveStartFlag = false;                      //Clear rising edge flag
  phaseUpdateFlag = false;                    //Clear phase update flag
  periodUpdateFlag = false;                   //Clear period update flag
  waveErrorCount = 0;                         //Clear error count

    //Clear external interrupt flags to prevent immediate launch of ISRs if interrupt request triggered while data was being reset.
    //Fixes device reset bug due to recursive calls of IRSs after data reset.   
  EIFR = 0x03;                                //Write logical 1 to INTF1 and INTF0 bits of EIFR (External Interrupt Flag Register). Atmega328P datasheet page 55 for details.  
  interrupts();

    //Reset stored float millis data
  for( byte i=0; i<4; i++ ){
    ISRwaveData[i][xVal] = 0.00;
    ISRwaveData[i][xMin] = 3.4028235E+38;                   //Reset min capture data storage to max possible data value.
    ISRwaveData[i][xMax] = -3.4028235E+38;                  //Reset max capture data storage to min possible data value.
    ISRwaveData[i][xAvg] = 0.00;
  }
  calcUpdateFlag = false;
  
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


int subSwitch(int currSubVal = 0, int maxSubVal = 0, int minSubVal = 0){
  // Loop through sub modes with buttons. Maintain currSub else. Cycle speed increased once if held 
    //Button functions: (bUp = Sub++), (bDown = Sub--) 
    //Takes current sub mode and total number of sub modes from passing function. Returns updated current sub mode. 
  
  
  unsigned int static holdCycles = 0;           //Current count of cycles while button is held
  byte static subValChange = 1;                 //Number to increment/decrement subVal. Increases if hold cycle check passes. 
  byte const cyclesBeforeBoost = 25;            //Number of cycles to count before boosted speed begins. 
  
  


    //Check button state and update control variables.
  if( currButton != 0 ){

       //Set subValChange based on hold cycle count. Increases value change speed if button is held. 
    if (holdCycles > cyclesBeforeBoost ){
      subValChange = 10;
    }else {
      subValChange = 1;
    }

       //Check if minimum switch delay and update control variables. Reset hold cycle count else.
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
  }else {                                               //Reset hold cycles if currButton == 0
    holdCycles = 0;
  }
  
 return currSubVal;                                     //Pass updated value to previous function. 
}


void threshMain(){

  String stThresh;
  String stPhase;
  byte stCurrThreshLength = 0;
  byte stCurrPhaseLength = 0;
  byte static stPrevThreshLength = 0;
  byte static stPrevPhaseLength = 0;

    //Print mode label if mode has changed.  Set in modeSwitch().
  if(modeSwitchFlag == true){
  lcd.setCursor(0,0);
  lcd.print("Threshold:");
  lcd.setCursor(0,1);
  lcd.print("Phase mS:");
  }

    //Set string values for printing
  stThresh = String(analogRead(threshInPin));
  stPhase = String(ISRwaveData[xPhase][xVal], 2);


    //Update current value string length. Clear value display if character length decreased. 
    //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)
  stCurrThreshLength = stThresh.length();
  stCurrPhaseLength = stPhase.length();
  if (stCurrThreshLength < stPrevThreshLength){
    lcd.setCursor(10,0);
    lcd.print("      ");
  }
  if (stCurrPhaseLength < stPrevPhaseLength){
    lcd.setCursor(9,1);
    lcd.print("       ");
  }

    //Print string values
  lcd.setCursor(10,0);
  lcd.print(stThresh);
  lcd.setCursor(9,1);
  lcd.print(stPhase);

  stPrevThreshLength = stCurrThreshLength;
  stPrevPhaseLength = stCurrPhaseLength;


  threshOut = subSwitch(threshOut, 255, 0);             //Update threshold setting with Up/Down buttons. max value 250, min value 10

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
}


void ppfdMain(byte modeReq = 0){
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

    switch (modeReq){
      case 0:
            lcd.setCursor(0,0);
            lcd.print("Phase mS:");
            cursorMain = 9;
            deciMain = 2;
            deciSub = 3;
            currModeVal = xPhase;
            break;
      case 1:
            lcd.setCursor(0,0);
            lcd.print("Period mS:");
            cursorMain = 10;
            deciMain = 2;
            deciSub = 3;
            currModeVal = xPeriod;
            break;
      case 2:
            lcd.setCursor(0,0);
            lcd.print("Freq. Hz:");
            cursorMain = 9;
            deciMain = 0;
            deciSub = 0;
            currModeVal = xFreq;
            break;
      case 3:
            lcd.setCursor(0,0);
            lcd.print("+Duty %:");
            cursorMain = 8;
            deciMain = 1;
            deciSub = 1;
            currModeVal = xDuty;
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

    //Save current string length for comparison on next loop. 
  stPrevMainLength = stMainLength;  


  ppfdSub(currModeVal, deciSub);
  
}


void ppfdSub(byte currModeVal, byte deciSub){
  //Phase, Period, Frequency, and Duty modes bottom print line display settings

  byte static currSubMode = 0;                                                         //Store current sub mode. (Value sets boot default)
  const byte subMin = 0;                                                                //Display min value
  const byte subMax = 1;                                                                //Display max value
  const byte subAvg = 2;                                                                //Display average value
  const byte subModeTotal = 3;                                                          //Display total successful ISR updates since reset
  const byte subModeErrors = 4;                                                         //Display total errors detected in ISRs since reset

  byte static stPrevSubLength = 0;
  String stSubVal;
  byte stSubLength;
  byte static cursorSub;
  unsigned long sampleCounts;
  unsigned long totalCounts;

  if (currModeVal == xPhase){
    totalCounts = wavePhaseLive[4];
  }
  else{
    totalCounts = wavePeriodLive[4];
  }

  currSubMode = subSwitch(currSubMode, 4, 0);
  
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
          if (calcUpdateFlag == true){
            stSubVal = String(ISRwaveData[currModeVal][xMin], deciSub);       
          } 
          else{
            stSubVal = "0";
          }
          break;
    case subMax:
          if (calcUpdateFlag == true ){
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
  
}


void modeSwitch(){
  // Loop through modes or reset wave stats with buttons. Clear display after any button press. Maintain currMainMode else. 
    //Button functions: (bRight = Main++), (bLeft = Main--), (bSelect = Reset stats). 

    //Mode switch control variables.
  int static currMainMode = 1;                                                          //Store current main mode.
  const byte maxMainVal = 4;                                                            //Total number of modes (zero referenced) 
 
    //Mode list. Values set rotation order. ***Must be zero referenced and sequential*** 
  const byte mainThresh = 0;                                                            //Threshold setting and signal min/max measurement
  const byte mainPhase = 1;                                                             //Phase measurement mode
  const byte mainPeriod = 2;                                                            //Period measurement mode
  const byte mainFreq = 3;                                                              //Frequency measurement mode 
  const byte mainDuty = 4;                                                              //Duty cycle measurement mode

                                                   
  if( currButton != 0 ){
    if( millis() - lastModeSwitch >= modeSwitchDelay){      //Check if minimum switch delay is met for more controlled switching. 
      switch (currButton){
        case bRight:
              currMainMode++ ;
              if ( currMainMode > maxMainVal) {
                currMainMode = 0;
              }
              lastModeSwitch = millis();                          //Reset mode switch reference time
              lcd.clear();                                        //Clear display
              modeSwitchFlag = true;                              //Trigger mode label reprint  
              break;
        case bLeft:
              currMainMode-- ;                
              if (currMainMode < 0){
                currMainMode = maxMainVal;
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
    }
  }

 
    //Begin function associated with main mode number. Run splash display for set hold time, clear after. 
  switch (currMainMode){
    case mainThresh:
          threshMain();
          break;
    case mainPhase:
          ppfdMain(0);
          break;
    case mainPeriod:
          ppfdMain(1);
          break;
    case mainFreq:
          ppfdMain(2);
          break;
    case mainDuty:
          ppfdMain(3);
          break;
  }
  return;
}



/*
 * 
 *  ***Global variables***
 * 
 * unsigned long volatile frameCount[9] = {0,0,0,0,0,0,0,0,0};                 //{>f-3, f-3, f-2, f-1, f, f+1, f+2, f+3, >f+3}
 * long volatile frameTarget[2] = {12500, 20832};                               //target frame length upper and lower limits. Default values for 60Hz frame rate. 
 * long volatile frameUnder[3] = {-4166, -20832, -37498};                       //target -1frame, -2frames, -3frames. lower limits. 
 * long volatile frameOver[3] = {37489, 54164, 70830};                          //target +1frame, +2frames, +3frames. upper limits. 
 * 
 * 
 * int frameRate = 60;
 * int frameCount = 1; 
 * 
 * 
 * 
 * **** insert into top of waveReset() above noInterrupts()
 * 
 * unsigned long frameLength = 1000000/ frameRate         //Set target frame phase length in uS. 1/frameRate = frames/Sec. 1000000/frameRate = frames/uS. 
 * unsigned long frameBuffer = frameLength >> 2           //Buffer is 25% of frame rate. Allows for phase length tolerance from threshold and filter effects. 
 *                                                           //Bitshift right 2 is equivalent to val/4 in unsigned integer types, but much faster. 
 * 
 * unsigned long frameTargetTemp[2];                      //For calculation. Store target frame phase length ±buffer upper and lower limits
 * unsigned long frameUnderTemp[3];                       //For calculation. Store frame phase length buffered lower limits for f-1 through f-3
 * unsigned long frameOverTemp[3];                        //For calculation. Store frame phase length buffered upper limits for f+1 through f+3
 * 
 * frameTargetTemp[0] = (frameLength * frameCount) - frameBuffer;       //For calculation. Set target frame phase length - buffered lower limit
 * frameTargetTemp[1] = (frameLength * frameCount) + frameBuffer;       //For calculation. Set target frame phase length + buffered upper limit
 * 
 * for (int i = 1, i++, i<4){                                          //For calculation. Set frame phase length buffered lower limits for f-1 through f-3
 *   frameUnderTemp[i-1] = frameTargetTemp[0] - (frameLength * i);
 * }
 * 
 * for (int i = 1, i++, i<4){                                          //For calculation. Set frame phase length buffered upper limits for f+1 through f+3
 *   framOverTemp[i-1] = fameTargetTemp[1] + (frameLength * i);
 * }
 *
 * 
 *  *** insert into waveReset() within noInterrupts()
 *  
 *    //Update and reset frame comparison values
 *  frameTarget[0] = frameTargetTemp[0];
 *  frameTarget[1] = frameTargetTemp[1];
 *  frameUnder[0] = frameUnderTemp[0];
 *  frameUnder[1] = frameUnderTemp[1];
 *  frameUnder[2] = frameUnderTemp[2];
 *  frameOver[0] = frameUnderOver[0];
 *  frameOver[1] = frameUnderOver[1];
 *  frameOver[2] = frameUnderOver[2];
 *  frameCount[0]= 0;
 *  frameCount[1]= 0;
 *  frameCount[2]= 0;
 *  frameCount[3]= 0;
 *  frameCount[4]= 0;
 *  frameCount[5]= 0;
 *  frameCount[6]= 0;
 *  frameCount[7]= 0;
 *  frameCount[8]= 0;
 * 
 * 
 *  **** insert into waveEndIRS() ****
 * 
 * 
 * if ( (wavePhaseLive[0] > frameTarget[0]) && (wavePhaseLive[0] < frameTarget[1]) ){             //If phase between upper and lower limits of frame target, target count++
 *   frameCount[4]++;
 * }else if (wavePhaseLive[0] <= frameTarget[0]) {                                                //If phase >= target lower limit, check frameUnder times
 *   if(wavePhaseLive[0] > frameUnder[0]){                                                          //If phase > frame-1 lower limit, frame-1++
 *     frameCount[3]++;
 *   }else if (wavePhaseLive[0] > frameUnder[1]){                                                   //If phase > frame-2 lower limit, frame-2++
 *     frameCount[2]++;
 *   }else if (wavePhaseLive[0] > frameUnder[2]){                                                   //If phase > frame-3 lower limit, frame-3++
 *     frameCount[1]++;
 *   }else{                                                                                         //Else phase must be <= frame-3 lower limit, >frame-3++
 *     frameCount[0]++;
 * }else if (wavePhaseLive[0] >= frameTarget[1]) {                                                //If phase <= target upper limit, check frameUnder times
 *   if(wavePhaseLive[0] < frameOver[0]){                                                           //If phase < frame+1 upper limit, frame-1++
 *     frameCount[5]++;
 *   }else if (wavePhaseLive[0] < frameOver[1]){                                                    //If phase < frame+2 upper limit, frame-2++
 *     frameCount[6]++;
 *   }else if (wavePhaseLive[0] < frameOver[2]){                                                    //If phase < frame+3 upper limit, frame-3++
 *     frameCount[7]++;
 *   }else{                                                                                         //Else phase must be >= frame+3 upper limit, >frame-3++
 *     frameCount[8]++;
 * }
 * 
 * 
 * 
 */
