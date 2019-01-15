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

  // Volitile global variables for waveStart() and waveEnd() Interrupt Service Routines (ISRs). ***All variables used in interrupt ISRs must be global and volatile.***
  // Updated in: waveStart(), waveEnd()
  // Used in: waveStart(), waveEnd(), waveCalc(), phaseMain(), periodMain()
unsigned long volatile waveStartTime = 0;           //Time micros for rising edge
unsigned long volatile waveEndTime = 0;             //Time micros for falling edge
unsigned long volatile wavePhaseMicros = 0;         //Time micros rising edge to falling edge
unsigned long volatile waveStartLast = 0;           //Time micros last rising edge
unsigned long volatile wavePeriodMicros = 0;        //Time micros rising edge to rising edge
unsigned long volatile wavePhaseTotal = 0;          //Total number of detected phase updates
unsigned long volatile wavePeriodTotal = 0;         //Total number of detected period updates
bool volatile waveStartFlag = false;                //For checking active wave status and error correction.
bool volatile phaseUpdateFlag = false;              //For checking completed phase duration update status.
bool volatile periodUpdateFlag = false;             //For checking completed period duration update status.
bool volatile waveResetFlag = true;                 //For checking recent reset. Suppresses phase data update until second rising edge.   

  // Storage for wave data. 
  // Updated in: waveCalc(), waveReset()
  // Used in: waveCalc(), waveReset(), phaseMain(), periodMain(), freqMain()
const byte xPhase =0;
const byte xPeriod =1;
const byte xFreq =2;
const byte xDuty = 3;
const byte xVal =0;
const byte xMin =1;
const byte xMax = 2;
const byte xAvg = 3;                  
    //All current phase, period, freq, and duty data                        //            xVal, xMin, xMax, xAvg
float static waveData[4][4];                                                //xPhase    {     ,     ,     ,     }
                                                                            //xPeriod   {     ,     ,     ,     }
                                                                            //xFreq     {     ,     ,     ,     }
                                                                            //xDuty     {     ,     ,     ,     }
    //Tracks sample counts and Sums for average values.
unsigned long static volatile phaseUpdateCount;                             //Running total of phase updates for calculating average. Also used in waveStart() to check for recent reset. 
float static phaseAvgSum;                                                   //Running total of phase times for calculating average 
unsigned long static periodUpdateCount;                                     //Running total of period updates for calculating average
float static periodAvgSum;                                                  //Running total of period times or calculating average
    //Tracks wave update state in waveCalc() to update display.
byte static waveStatus = 0;                                                 //0=Extended LOW, 1=Extended HIGH, 2=Recent Phase update
                                                                                

  //Storage for current interface mode. 
  //Updated in: modeSwitch()
  //Used in: modeSwitch(), modeLaunch(), phaseMain(), periodMain()
int static currMainMode = 1;                                                        //Store current main mode. 
const byte mainThresh = 0;                                                            //Threshold setting and signal min/max measurment
const byte mainPhase = 1;                                                             //Phase measurment mode
const byte mainPeriod = 2;                                                            //Period measurment mode
const byte mainFreq = 3;                                                              //Frequency and duty measurement mode
int static currSubMode = 3;                                                         //Store current sub mode. (Value sets boot default)
const byte subMin = 0;                                                                //Display Min value
const byte subMax = 1;                                                                //Display Max value
const byte subAvg = 2;                                                                //Display Average value
const byte subModeSampled = 3;                                                        //Display Samples captured in waveCalc().
const byte subModeTotal = 4;                                                          //Display Total updates since start

//Tells mode functions to print mode label to reduce unnecessary lcd writes. Must start TRUE
bool static modeSwitchFlag = true;                                                                                                                                        //For reducing unnecessary lcd print cycles. 


  //Delays for mode changes and status updates
const int modeSwitchDelay = 150;                    // Min millis between menu changes.
const int modeSplashDelay = 1300;                   // Max millis to display mode config information on mode switch.
const long modeSplashMax = 180000;                  // Max millis total run time to allow splash to display on mode switch. 
const int offResetDelay = 10000;                    // Delay millis to hold data on screen before displaying OFF messsage. 


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  lcd.begin(16, 2);

    // declare interrupt pins. Falling state initialized first to eliminate false updates when reset with active signal.
  pinMode(2, INPUT);
  pinMode(3, INPUT); 
  attachInterrupt(digitalPinToInterrupt(2), waveEnd, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), waveStart, RISING);

    //Set default wave statistic values
  waveReset();
}


void loop() {
  // put your main code here, to run repeatedly:

  //waveCalc();                 //Update all wave values. Run between all function changes and print strings. 
  buttonCheck();              //Check button state
  //waveCalc();                 
  modeSwitch();               //Update UI if button state changes.  
  //waveCalc();
  modeLaunch();               //Launch current main and sub mode settings.

        
}


void waveCalcPhase(){
  //Calculate wave times and statistics if update detected. 

  //NOTE:  Run waveCalc() as often as practical to improve captured sample rate and data accuracy. 
    //LCD I2C communication takes the longest and therefore causes the most uncaputred samples. Insert waveCalc() between print strings.  
    //Execution time increases with frequency do to interrupts. True for all functions, but especially waveCalc. Noticably slower above 10kHz. 
    //Min execution: 4uSec (without signal updates) 
    //Max durations: 300uSec @60Hz; 310uSec @144Hz; 320uSec @2kHz; 350uSec @5kHz; 415uSec @10kHz; 675uSec @20kHz; 2015uSec @31kHz. 


  unsigned long static lastPhaseUpdate = 0;                                       //Time millis since last phase update
  interrupts();


    //Phase time and status display mode updates 
  if( phaseUpdateFlag == true ){
    noInterrupts();
    waveData[xPhase][xVal] = wavePhaseMicros;       //convert unsigned long phase micros to float
    interrupts(); 
    lastPhaseUpdate = millis();
    waveData[xPhase][xVal] = waveData[xPhase][xVal]/1000;     //convert micros to millis         

    if (waveData[xPhase][xVal] > 0 ){
      //Update phase min
      if( waveData[xPhase][xVal] < waveData[xPhase][xMin] ){
        waveData[xPhase][xMin] = waveData[xPhase][xVal];
      }
      
        //Update phase max
      if( waveData[xPhase][xVal] > waveData[xPhase][xMax] ){
        waveData[xPhase][xMax] = waveData[xPhase][xVal];
      } 
  
        //Update phase average
      phaseUpdateCount ++;
      phaseAvgSum += waveData[xPhase][xVal];
      waveData[xPhase][xAvg] = phaseAvgSum / phaseUpdateCount;
      
      phaseUpdateFlag = false;
      waveStatus = 2;           
    }
  }
  else{                                                         //Set flags for measurement status messages 
    if( (millis() - lastPhaseUpdate) >= offResetDelay ){        //Check time since last update 
      if ( waveStartFlag == true ){                               //From waveStart()
        waveStatus = 1;
      }
      else{
        waveStatus = 0;
      }      
    }
  }
}

void waveCalcPeriod(){
  interrupts();

    //Period time, freq, and duty updates
  if( periodUpdateFlag == true ){
      noInterrupts();
      waveData[xPeriod][xVal] = wavePeriodMicros;     //convert unsigned long period micros to float
      interrupts();
      waveData[xPeriod][xVal] = waveData[xPeriod][xVal]/1000;   //convert micros to millis
  
      if ( waveData[xPeriod][xVal] > 0 ){
        //Update period min
      if( waveData[xPeriod][xVal] < waveData[xPeriod][xMin] ){
        waveData[xPeriod][xMin] = waveData[xPeriod][xVal];
      }
    
        //Update period max
      if( waveData[xPeriod][xVal] > waveData[xPeriod][xMax] ){
        waveData[xPeriod][xMax] = waveData[xPeriod][xVal];
      }
    
        //Update period average
      periodUpdateCount ++;
      periodAvgSum += waveData[xPeriod][xVal];
      waveData[xPeriod][xAvg] = periodAvgSum / periodUpdateCount;


      
        //Update frequency and duty cycle data
      for (byte i=0; i<4; i++){  
        waveData[xFreq][i] = ( 1 / (waveData[xPeriod][i] / 1000) );         //Convert period to seconds and calculate frequency. Freq Hz = 1/ (period time in seconds). 
        waveData[xDuty][i] = ( (waveData[xPhase][i] / waveData[xPeriod][i]) * 100 );            //positive Duty% = positive phase / period
      }

        //Reset flag until next sample set in waveStart()
      periodUpdateFlag = false;
    }    
  }

}

void waveReset(){
  //Reset all wave data to default values
  
  lcd.clear();
  
    //Reset phase data
  noInterrupts();
    for ( byte i=0; i<4; i++ ){
  waveData[i][xVal] = 0.00;
  waveData[i][xMin] = 3.4028235E+38;
  waveData[i][xMax] = -3.4028235E+38;
  waveData[i][xAvg] = 0.00;
  }
  
  phaseAvgSum = 0;
  phaseUpdateCount = 0;
  periodAvgSum = 0;
  periodUpdateCount = 0;
  interrupts();
}

   
byte buttonCheck() {
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
    //Button functions: (bRight = Main++), (bLeft = Main--), (bUp = Sub++), (bDown = Sub--), (bSelect = Reset stats). 
  
  unsigned long static lastModeSwitch = 0;         //Millis since last mode switch.
                                                        
  const byte mainModeList[] = {mainThresh, mainPhase, mainPeriod, mainFreq};            //List and order of main modes to cycle with left/right
  const byte subModeList[] = {subMin, subMax, subAvg, subModeSampled, subModeTotal};    //List and order of sub modes to cycle with up/down


      
  if( currButton != 0 ){
    if( millis() - lastModeSwitch >= modeSwitchDelay){      //Check if minimum switch delay is met for more controlled switching. 
      switch (currButton){
        case bRight:
              currMainMode++ ;
              if ( currMainMode > (sizeof(mainModeList) - 1) ){
                currMainMode = 0;
              }
              break;
        case bLeft:
              currMainMode-- ;                
              if (currMainMode < 0){
                currMainMode = (sizeof(mainModeList) - 1);
              }
              break;
        case bUp:
              currSubMode++ ;
              if ( currSubMode > (sizeof(subModeList) - 1) ){
                currSubMode = 0;
              }
              break;
        case bDown:
              currSubMode-- ;
              if (currSubMode < 0){
                currSubMode = ( sizeof(subModeList) - 1 );
              }
              break;
        case bSelect:
              waveReset();
              break;
      }
    currMainMode = mainModeList[currMainMode];          //Set main mode changes
    currSubMode = subModeList[currSubMode];             //Set sub mode changes
    lastModeSwitch = millis();                          //Reset mode switch reference time
    lcd.clear();
    modeSwitchFlag = true;  
    }
  }
}


void modeLaunch(){
   //Launch mode set in modeSwitch()

  unsigned long static modeSplashStart = 0;             //Time Millis of new mode start. For brief config display. 
  bool static splashClearFlag = false;                  //Used to trigger screen clear if splash was displayed
  
    //Check for run time. Allow control reminder splash screen to appear if main mode switched
  if ( millis() < modeSplashMax ){

      //Check if main mode has been switched. 
    if( (currButton == bLeft) || (currButton == bRight) ){
      modeSplashStart = millis();                                 //Reset mode spash screen reference time
    }
 
      //Check if splash display time has been met. 
    if ( (millis() - modeSplashStart) <= modeSplashDelay ){
      switch (currMainMode){
        case mainThresh:
              threshSplash();
              break;
        case mainPhase:
              phaseSplash();
              break;
        case mainPeriod:
              periodSplash();
              break;
        case mainFreq:
              freqSplash();
              break;
      }
      
      splashClearFlag = true;
      return;
            
    }
  }

  if( splashClearFlag == true ){
    lcd.clear();
    splashClearFlag = false;
  }
 
    //Begin function associated with main mode number. Run splash display for set hold time, clear after. 
  switch (currMainMode){
    case mainThresh:
          threshMain();
          break;
    case mainPhase:
          phaseMain();
          break;
    case mainPeriod:
          periodMain();
          break;
    case mainFreq:
          freqMain();
          break;
  }

}



void threshSplash(){
  //Splash display for entering Threshold setup mode. 
  
  lcd.setCursor(0,0);
  lcd.print("Threshold Setup");
  //waveCalc();
  lcd.setCursor(0,1);
  lcd.print("Up/Dn-Set thresh");
   
}

void threshMain(){

  lcd.setCursor(0,0);
  lcd.print("Thresh");
  lcd.setCursor(0,1);

  
}


void phaseSplash(){
  //Splash display for entering phase time mode. 
  
  lcd.setCursor(0,0);
  lcd.print("Phase: mSeconds");
  //waveCalc();
  lcd.setCursor(0,1);
  lcd.print("U/D-Stat Sel-RSt"); 
  
}

void phaseMain(){
  //Phase mode top line display controls
  //waveCalc() inserted between all strings. 

    //Store previous and current value string lengths. For clearing field if length reduces. 
  byte static stPrevLength = 0;
  String stCurrVal;
  byte stCurrLength;


    //Print mode label if mode has changed.  Set in modeSwitch(). reset in sub mode function.  
  if(modeSwitchFlag == true){
  lcd.setCursor(0,0);
  lcd.print("Phase:");
  //waveCalc(); 
  }

    //Set main value string based on current waveStatus (set in waveCalc())
  switch (waveStatus){
    case 0:
          stCurrVal = "0.00   OFF";
          break;
    case 1:
          stCurrVal = "MEASURING";
          break;
    default: 
          stCurrVal = String(waveData[xPhase][xVal], 2);
          break; 
  }


    //Update current value string length. Clear value display if character length decreased. 
    //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)
  stCurrLength = stCurrVal.length();
  if ( stCurrLength < stPrevLength ){
    lcd.setCursor(6,0);
    lcd.print("          ");
    //waveCalc();
  }

  lcd.setCursor(6,0);
  lcd.print(stCurrVal);
  //waveCalc();
  
  stPrevLength = stCurrLength;  


  phaseSub();
  
}


void phaseSub(){

  byte static stPrevLength = 0;
  String stCurrVal;
  byte stCurrLength;
  byte static cursorVal;

  
    //Print mode label and set following value cursor postion variable if mode has changed. 
  if( modeSwitchFlag == true ){
    lcd.setCursor(0,1);
    switch (currSubMode){
      case subMin:
            lcd.print("Min:");
            cursorVal = 4;        
            break;
      case subMax:
            lcd.print("Max:");
            cursorVal = 4;
            break;
      case subAvg:
            lcd.print("Avg:");
            cursorVal = 4;
            break;
      case subModeSampled:
            lcd.print("Samples:");
            cursorVal = 8;
            break;
      case subModeTotal:
            lcd.print("Total:");
            cursorVal = 6;
            break;  
    }
    
    //waveCalc();
    
  }

    //Set string output value based on current sub mode.
      //String set to "0" for Min and Max if no updates have been completed to prevent string length overflow from max pos/neg float values. 
  switch (currSubMode){
    case subMin:
          if (waveResetFlag == false){
            stCurrVal = String(waveData[xPhase][xMin], 3);
          }
          else{
            stCurrVal = "0";
          }
          break;
    case subMax:
          if (waveResetFlag == false){
            stCurrVal = String(waveData[xPhase][xMax], 3);
          }
          else{
            stCurrVal = "0";
          }
          break;
    case subAvg:
          stCurrVal = String(waveData[xPhase][xAvg], 3);
          break;
    case subModeSampled:
          stCurrVal = String(phaseUpdateCount);
          break;
    case subModeTotal:
          stCurrVal = String(wavePhaseTotal);
          break;  
  }


    //Update current value string length. Clear value display if character length decreased. 
    //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)
  stCurrLength = stCurrVal.length();
  if(stCurrLength < stPrevLength){
    lcd.setCursor(cursorVal, 1);
    lcd.print("            ");
    //waveCalc();
  }

    //Set cursor to match mode label. Print value. 
  lcd.setCursor(cursorVal, 1);
  lcd.print(stCurrVal);

    //Update previous string length for next cycle. 
  stPrevLength = stCurrLength;
  
    //Prevent label from reprinting until next mode change. 
  modeSwitchFlag = false;                   
  
  return;
  
}


void periodSplash(){
  //Splash display for entering period time mode. 
  
  lcd.setCursor(0,0);
  lcd.print("Period: mSeconds");
  //waveCalc();
  lcd.setCursor(0,1);
  lcd.print("U/D-Stat Sel-RSt");
   
}

void periodMain(){
  //Period mode display controls

    //Stores previous display modes to detect changes and trigger screen clearing. 
  byte static prevMainDisplay = 0;
  byte static prevSubDisplay = 0;

    
    //Detect waveStatus changes. Clear display if change detected. 
  if ( prevMainDisplay != waveStatus ){
    lcd.clear(); 
    //waveCalc();   
  }

    //Main mode output
  lcd.setCursor(0,0);
  lcd.print("Period:");
  //waveCalc();
  switch (waveStatus){
    case 0:
          lcd.print("0.00  OFF");
          break;
    case 1:
          lcd.print("MEASURING");
          break;
    default: 
          lcd.print(waveData[xPeriod][xVal]);
          break; 
  }
  
  prevMainDisplay = waveStatus;
  //waveCalc();                 

  
    //Detect sub mode changes. Clear sub display if change detected. 
  if ( prevSubDisplay != currSubMode ){
    lcd.setCursor(0,1); 
    lcd.print("                ");
    //waveCalc();       
  }

    //Sub mode output
  lcd.setCursor(0,1);
  if( waveData[xPeriod][xVal] > 0 ){
    switch (currSubMode){
      case subMin:
            lcd.print("Min:");
            //waveCalc();
            lcd.print(waveData[xPeriod][xMin], 3);
            break;
      case subMax:
            lcd.print("Max:");
            //waveCalc();
            lcd.print(waveData[xPeriod][xMax], 3);
            break;
      case subAvg:
            lcd.print("Avg:");
            //waveCalc();
            lcd.print(waveData[xPeriod][xAvg], 3);
            break;
      case subModeSampled:
            lcd.print("Samples:");
            //waveCalc();
//            Serial.println(periodAvgSum, 8);
//            Serial.println(periodUpdateCount);
            lcd.print(periodUpdateCount);
            break;
      case subModeTotal:
            lcd.print("Total:");
            //waveCalc();
            lcd.print(wavePeriodTotal);
            break;  
    }
  }

  prevSubDisplay = currSubMode;
  
  return;
  
}


void freqSplash(){
    //Splash display for entering Frequency and Duty Cycle mode.
    
  lcd.setCursor(0,0);
  lcd.print("Frequency: Hz");
  //waveCalc();
  lcd.setCursor(0,1);
  lcd.print("+Duty Cycle: %"); 
        
}

void freqMain(){
  //Frequency mode display controls
  
  lcd.setCursor(0,0);
  lcd.print("Freq:");
  //waveCalc();
  lcd.print(waveData[xFreq][xVal], 0);
  lcd.setCursor(14,0);
  //waveCalc();
  lcd.print("Hz");
  //waveCalc();

  lcd.setCursor(0,1);
  lcd.print("+Dty:");
  //waveCalc();
  lcd.print(waveData[xDuty][xVal], 1);
  //waveCalc();
  lcd.setCursor(15,1);
  lcd.print("%");

  return;
  
}


void waveStart(){
   // Start wave timing. Rising edge ISR. Digital pin 2
   // Update Start flag and time. Set stop flag to 0 for error reduction. 
   
  waveStartTime = micros();

  if(waveResetFlag == false){ 
    if( waveStartTime > waveEndTime ){
    wavePeriodMicros = (waveStartTime - waveStartLast);
    waveStartLast = waveStartTime;
    wavePeriodTotal++;
    periodUpdateFlag = true;
    waveCalcPeriod();
    }
  }
  else{
    periodUpdateFlag = false; 
  }
  
  waveStartFlag = true;
  
}

void waveEnd(){  
  // End wave timing. Falling edge ISR. Digital pin 3
  // Verify start flag and postive time difference. Update total wave time.

  waveEndTime = micros();
  
  if( (waveStartFlag == true) && (waveEndTime - waveStartTime >= 0) ){
      wavePhaseMicros = (waveEndTime - waveStartTime);
      wavePhaseTotal++;
      phaseUpdateFlag = true;
      waveStartFlag = false;
      waveResetFlag = false; 
      waveCalcPhase();
  } 
  else{
    waveStartFlag = false;
  }
}


/*
 * 
 * unsigned long volatile frameLength[6]{0, 1, 2, 3, 4, 5};       // Length comparisons: {<1 frame, 1f, 2f, 3f, 4f, 5f} 
 * unsigned long volatile frameCount[7]{0,0,0,0,0,0,0};           // Count totals: {noise, 1f, 2f, 3f, 4f, 5f, >5f}
 * unsigned int frameFreq = 60;
 * 
 * unsigned long frameSingle;                                     //Length of single frame in microseconds
 * 
 * frameSingle = (1/frameFreq)*1000000);                          //Convert frameFreq to length in microseconds 
 * frameLength[0] = frameSingle *0.9;                             //Set minimum value to count noise/errors. 90% of frameSingle 
 * 
 *  //Set worst case max length of frames with 15% response lag and 8micros IRS error 
 * for(byte i=1; i++; i<6){
 * frameLength[1] = (( (frameSingle * i) * 1.15 ) + 8)            
 * }
 * 
 * 
 * 
 * 
 * 
 * if (wavePeriodMicros < frameLength[0]){
 *  frameCount[0]++;
 * }
 * else if ((wavePeriodMicros <= frameLength[1]){
 *  frameCount[1]++;
 * }
 * else if ((wavePeriodMicros <= frameLength[2]){
 *  frameCount[2]++;
 * }
 * else if ((wavePeriodMicros <= frameLength[3]){
 *  frameCount[3]++;
 * }
 * else if ((wavePeriodMicros <= frameLength[4]){
 *  frameCount[4]++;
 * }
 * else if ((wavePeriodMicros <= frameLength[5]){
 *  frameCount[5]++;
 * }
 * else if ((wavePeriodMicros > frameLength[5]){
 *  frameCount[6]++;
 * }
 * 
 */
