
// include the library code:
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

  //Storage for button status. 
byte static currButton = 0;
const byte bUp = 1;
const byte bDown = 2;
const byte bLeft = 3;
const byte bRight = 4;
const byte bSelect = 5;


  // Volatile global variables for waveStartISR() and waveEndISR() Interrupt Service Routines (ISRs). ***All variables used in interrupt ISRs must be global and volatile.***
unsigned long volatile waveStartTime = 0;           //Time micros for rising edge. Needed for both period and phase calculations. 
unsigned long volatile wavePeriodLive[5] = {0,0xFFFFFFFF,0,0,0};        //Period update from ISR: {current val, min, max, total for avg, count for avg}
unsigned long volatile wavePhaseLive[5] = {0,0xFFFFFFFF,0,0,0};         //Phase update from ISR: {current val, min, max, total for avg, count for avg}
unsigned long volatile waveErrorCount = 0;          //Total number of errors detected. 
bool volatile waveStartFlag = false;                //For checking active wave status and error detection.
bool volatile waveEndFlag = false;                  //For error detection
bool volatile phaseUpdateFlag = false;              //For checking completed phase duration update status.
bool volatile periodUpdateFlag = false;             //For checking completed period duration update status.
bool volatile waveResetFlag = true;                 //For checking recent reset. Suppresses phase data update until second rising edge.   

unsigned long volatile frameCount[9] = {0,0,0,0,0,0,0,0,0};                 //Store current frame counts. {>f-3, f-3, f-2, f-1, f, f+1, f+2, f+3, >f+3}
long volatile frameGoal[2] = {12500, 20832};                                //target frame length upper and lower limits. Default values for 60Hz frame rate with 1 frame target. 
long volatile frameUnder[3] = {-4166, -20832, -37498};                      //target -1frame, -2frames, -3frames. lower limits. 
long volatile frameOver[3] = {37489, 54164, 70830};                         //target +1frame, +2frames, +3frames. upper limits. 


  // Storage for ISRwave data. All current phase, period, freq, and duty data. Initialized in setup() via waveReset(). 
const byte xPhase =0;
const byte xPeriod =1;
const byte xFreq =2;
const byte xDuty = 3;
const byte xVal =0;
const byte xMin =1;
const byte xMax = 2;
const byte xAvg = 3;                                                                                            
float static ISRwaveData[4][4];                                             
								  //            xVal, xMin, xMax, xAvg                                           
								  //xPhase    {     ,     ,     ,     }                                                                          
								  //xPeriod   {     ,     ,     ,     }                                                                          
								  //xFreq     {     ,     ,     ,     }
								  //xDuty     {     ,     ,     ,     }


  
    //Tracks wave update state in ISRwaveCalc() to update display.
byte static waveStatus = 0;                               //0=Extended LOW, 1=Extended HIGH, 2=Recent Phase update
bool static calcUpdateFlag = false;                       //True if ISRwaveData values updated since reset. 
                                                              //Used to prevent min/max float values causing string overflow and program crash. 
                                                            
  //Tells mode functions to print mode label to reduce unnecessary lcd writes. Must start TRUE
bool static modeSwitchFlag = true;                  //For reducing unnecessary lcd print cycles. 

  //Threshold global variables
const byte threshOutPin = 6;                        //Threshold PWM output pin
const byte threshInPin = A1;                        //Threshold analog input sense pin
byte static threshOut;                              //Threshold PWM output setting. Initialized in setup(). 

  //Analog wave sense global variables
const byte analogWavePin = A0;                         //Analog wave sense pin

  //Current frame rate and goal frame number
int frameRate = 60;                         //Store current frame rate settings. Should match refresh rate Hz of monitor under test. 
int frameGoalNum = 1;                       //Store current goal number of frames for a test. 


  //Delays for mode changes and status updates
const unsigned int modeSwitchDelay = 150;             // Min millis between menu changes.
//const int modeSplashDelay = 1300;                   // Max millis to display mode config information on mode switch.
//const long modeSplashMax = 180000;                  // Max millis total run time to allow splash to display on mode switch. 
const int offResetDelay = 10000;                      // Delay millis to hold data on screen before displaying OFF message.
unsigned long static lastModeSwitch = 0;              //Millis since last mode switch. 


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
 * General behavior:
 * If an interrupt request triggers during normal conditions, the program will jump to the ISR (Interrupt Service Routine) as soon as the present operation has completed. 
 * The program will resume at the next operation after the ISR is completed. 
 * Operations are on the assembly and machine code level, not the higher level language written. 
 * This allows for the fewest number of clock cycle possible for an ISR to begin, with usual response times on the order of <10uSec for most platforms and clock rates. (<3uSec for the ATmega328P @16MHz).  
 * 
 * 
 * Conditional behavior:
 * If an interrupt request triggers while interrupts are disabled (time between "noInterrupts();" and "interrupts(); or while another ISR is active"), the ISR will launch immediately after interrupts are re-enabled. 
 * If multiple interrupt requests are placed while interrupts are disabled, ISR's execute in order of their priority. 
 * Interrupts are disabled automatically while an ISR is running and re-enabled upon completion. 
 * If additional and/or multiple interrupt requests trigger while an ISR is running, newly requested ISRs will run sequentially in priority order until all request are cleared.   
 * 
 * Microcontroller reset requests have highest priority (IRQ priority 0). 
 * Falling edge interrupt has higher priority (INT0, IRQ priority 1) than rising edge (INT1, IRQ priority 2). 
 * This will improve phase time accuracy by updating data as quickly as possible and preventing errors in calculation if rising and falling both trigger while interrupts are disabled.  
 * Full list of interrupt vector priorities is available in the microcontroller datasheet.  
 *  
 *  
 * Coding considerations:  
 * "noInterrupts();" and "interrupts();" commands are global, meaning they affect all internal and external interrupt vectors. 
 * "attachInterrupt()" and "detachInterrupt()" will only affect the individual interrupt vector. 
 * Disabling an interrupt only ignores the request to run the ISR. It does no clear the request flag. The ISR will run as soon as the interrupt is enabled regardless of how long it has been since the request. 
 * Request flags can be manually cleared to prevent immediate ISR launch after enabling the interrupt. See the datasheet for details. (used in waveReset() for this program) 
 * 
 * Some software functions built into the microcontroller and Arduino environment also run on interrupts (e.g. Millis(), "delay();" and Serial communications). 
 * Software interrupt functionality may be affected by long or frequent ISR calls. They cannot be used within an ISR or during "noInterrupts();". 
 * The value of "millis();" can be retrieved, but it will not increment while interrupts are disabled. 
 * Global interrupts can be enabled during an ISR, but it should be avoided as it may lead to recursive ISR calls and/or longer ISR run times.  
 *  
 * ISRs should be written with clock cycles for execution in mind. 
 * Division takes significantly longer than addition, subtraction, or multiplication. Floating point math takes longer than integer math. Floating point and division should be avoided in an ISR whenever possible. 
 * Conditional checks should be as simple as possible. Avoid loops. Remember that compound comparisons (i.e. ">=" and "<=" ) have more overhead than direct comparisons (i.e. "==", "!=", ">", "<" ). 
 *  
 * Due to interrupts injecting themselves between lower level assembly instructions, additional considerations must be made for how data is addressed and protected.  
 * This can be important to keep in mind as evaluating a 32bit variable on an 8bit platform will take several operations to complete.  
 * Without controls in place, a value may be altered or have changes overwritten/negated if an ISR writes to it while the main program is using it.
 * Volatile variable >1byte should be addressed "atomically" (interrupts detached or disabled) to prevent error in value if ISR is called while variable is being used by the main program. 
 * (Some volatile value calls for creating a print string in this program are not done atomically as a small inaccuracy for a single print cycle seemed better than disabling interrupts more frequently)
 *  
 * All global variables used in ISR must be declared volatile. 
 * This tells the compiler they must be stored in and addressed from program RAM (as opposed to the processor cache) at all times because they can be updated at any time. 
 * This also prevents them from being optimized away by the compiler if it does not appear they would be addressed or updated within the program. 
 * Volatile variables take longer to address (RAM speed vs cache speed), so making a standard data type local copy can allow for faster successive calls and further manipulation at the cost of storage and real time accuracy. 
 * Local variables within ISR should be declared normally (not volatile) for best performance and scope protection. 
 * 
 * More information on Arduino interrupts can be found at http://www.gammon.com.au/interrupts and the microcontroller datasheet.  
 * ATmega329P datasheet for Arduino Uno (page 49-54):http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 *  
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

void waveStartISR(){      // Start wave timing for phase and period calculations. Rising edge ISR. Digital pin 3 
   

  unsigned long static waveStartLast = 0;         //Store previous start time for period calculation
   
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


void waveEndISR(){      // End wave timing for phase and frame calculations. Falling edge ISR. Digital pin 2
  
  
  long frameLive;
  unsigned long waveEndTime;

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

      //Check if phase length micros can be converted to a valid signed long. 
      //Convert to signed long for frame length comparison and update correct frame count. Needed for proper evaluation against negative values for low frame count goals.
      //Else increment >frame+3 count. Value exceeds all user settable ranges. 
    if( wavePhaseLive[0] < 0x7FFFFFFF ){
      frameLive = wavePhaseLive[0];
  
        //Compare phase length to expected frame lengths and update counts
      if ( (frameLive > frameGoal[0]) && (frameLive < frameGoal[1]) ){             //If phase between upper and lower limits of frame target, target count++
        frameCount[4]++;
      }else if (frameLive <= frameGoal[0]) {                                                //If phase >= target lower limit, check frameUnder times
        if(frameLive > frameUnder[0]){                                                          //If phase > frame-1 lower limit, frame-1++
          frameCount[3]++;
        }else if (frameLive > frameUnder[1]){                                                   //If phase > frame-2 lower limit, frame-2++
          frameCount[2]++;
        }else if (frameLive > frameUnder[2]){                                                   //If phase > frame-3 lower limit, frame-3++
          frameCount[1]++;
        }else{                                                                                         //Else phase must be <= frame-3 lower limit, >frame-3++
          frameCount[0]++;
        }
      }else if (frameLive >= frameGoal[1]) {
      //If phase <= target upper limit, check frameUnder times
        if(frameLive < frameOver[0]){                                                           //If phase < frame+1 upper limit, frame-1++
          frameCount[5]++;
        }else if (frameLive < frameOver[1]){                                                    //If phase < frame+2 upper limit, frame-2++
          frameCount[6]++;
        }else if (frameLive < frameOver[2]){                                                    //If phase < frame+3 upper limit, frame-3++
          frameCount[7]++;
        }else{                                                                                         //Else phase must be >= frame+3 upper limit, >frame-3++
          frameCount[8]++;
        }
      }
    }else{
      frameCount[8]++;
    }
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


void ISRwaveCalc(){     //Recalculate wave data taken from ISR's if updated
  

  unsigned long static lastPhaseUpdate = 0;                                       //Time millis since last phase update
  unsigned long waveMicrosCopy[5] = {0,0xFFFFFFFF,0,0,0};                          //For quickly copying current data without conversion

  //Calculate wave phase times and statistics if update detected. 

    //Phase time and status display mode updates 
  if( phaseUpdateFlag == true ){

      //Copy new data and update reset flag. Interrupts disabled to prevent error. 
    noInterrupts();
      //Copy unsigned long phase micros data for further calculations to minimize ISR down time. Intentionally verbose to eliminate conditional checks and minimize interrupt down time. 
    waveMicrosCopy[0] = wavePhaseLive[0];       
    waveMicrosCopy[1] = wavePhaseLive[1];
    waveMicrosCopy[2] = wavePhaseLive[2];
    waveMicrosCopy[3] = wavePhaseLive[3];
    waveMicrosCopy[4] = wavePhaseLive[4];
    
    phaseUpdateFlag = false;                   //Reset update flag
    interrupts(); 

      //Convert unsigned long micros to float millis
    for (byte i=0; i<4; i++){
    ISRwaveData[xPhase][i] = waveMicrosCopy[i];                         //Convert unsigned long (val, min, max, average) to float
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

      //Copy new data and update reset flag. Interrupts disabled to prevent error. 
    noInterrupts();
      //Copy unsigned long period micros data for further calculations to minimize ISR down time. Intentionally verbose to eliminate conditional checks and minimize interrupt down time. 
    waveMicrosCopy[0] = wavePeriodLive[0]; 
    waveMicrosCopy[1] = wavePeriodLive[1];
    waveMicrosCopy[2] = wavePeriodLive[2];
    waveMicrosCopy[3] = wavePeriodLive[3];
    waveMicrosCopy[4] = wavePeriodLive[4];
      
    periodUpdateFlag = false;
    interrupts();
    
      //Convert unsigned long micros to float millis
    for (byte i=0; i<4; i++){
      ISRwaveData[xPeriod][i] = waveMicrosCopy[i];                       //Convert unsigned long (val, min, max, average) to float
      ISRwaveData[xPeriod][i] = ISRwaveData[xPeriod][i] * 0.001;             //Convert float micros to float millis
    }

      //Calculate average period millis
    ISRwaveData[xPeriod][xAvg] /= waveMicrosCopy[4];                   //Phase avg = total millis / total counts


      //Update frequency data. Freq Hz = (1000 / (period in milliseconds) ). 
    ISRwaveData[xFreq][xVal] = ( 1000 / (ISRwaveData[xPeriod][xVal]) );          //Current frequency Hz = 1000/ (Current period time in milliseconds).
    ISRwaveData[xFreq][xMin] = ( 1000 / (ISRwaveData[xPeriod][xMax]) );          //Min frequency Hz = 1000/ (Max period time in milliseconds). Freq and period are inversely related
    ISRwaveData[xFreq][xMax] = ( 1000 / (ISRwaveData[xPeriod][xMin]) );          //Max frequency Hz = 1000/ (Min period time in milliseconds). Freq and period are inversely related       
    ISRwaveData[xFreq][xAvg] = ( 1000 / (ISRwaveData[xPeriod][xAvg]) );          //Average frequency Hz = 1000/ (Average period time in milliseconds).


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


void waveReset(){     //Update frame count control variables and reset all wave data to default values
  
  
  lcd.clear();

 
  long frameGoalTemp[2];                        //For calculation. Store target frame phase length ±buffer upper and lower limits
  long frameUnderTemp[3];                       //For calculation. Store frame phase length buffered lower limits for f-1 through f-3
  long frameOverTemp[3];                        //For calculation. Store frame phase length buffered upper limits for f+1 through f+3
   
  unsigned long frameLength = 1000000/ frameRate;         //Set target frame phase length in uS. 1/frameRate = frames/Sec. 1000000/frameRate = frames/uS. 
  unsigned long frameBuffer = frameLength >> 2;           //Buffer is 25% of frame rate. Allows for phase length tolerance from threshold and filter effects. 
                                                             //Bitshift right 2 is equivalent to val/4 in unsigned integer types, but much faster. 

    //Calculate new goal frame length min and max range
  frameGoalTemp[0] = (frameLength * frameGoalNum) - frameBuffer;       //For calculation. Set target frame phase length - buffered lower limit
  frameGoalTemp[1] = (frameLength * frameGoalNum) + frameBuffer;       //For calculation. Set target frame phase length + buffered upper limit
  
    //Calculate frame length for given number of undershot frames. Subtract one frame unbuffered frame length per. 
  frameUnderTemp[0] = frameGoalTemp[0] - frameLength;
  frameUnderTemp[1] = frameUnderTemp[0] - frameLength;
  frameUnderTemp[2] = frameUnderTemp[1] - frameLength;

    //Calculate frame length for given number of overshot frames. Add one frame unbuffered frame length per. 
  frameOverTemp[0] = frameGoalTemp[1] + frameLength;
  frameOverTemp[1] = frameOverTemp[0] + frameLength;
  frameOverTemp[2] = frameOverTemp[1] + frameLength;
  
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

    //Update and reset frame comparison values
  frameGoal[0] = frameGoalTemp[0];
  frameGoal[1] = frameGoalTemp[1];
  frameUnder[0] = frameUnderTemp[0];
  frameUnder[1] = frameUnderTemp[1];
  frameUnder[2] = frameUnderTemp[2];
  frameOver[0] = frameOverTemp[0];
  frameOver[1] = frameOverTemp[1];
  frameOver[2] = frameOverTemp[2];
  frameCount[0]= 0;
  frameCount[1]= 0;
  frameCount[2]= 0;
  frameCount[3]= 0;
  frameCount[4]= 0;
  frameCount[5]= 0;
  frameCount[6]= 0;
  frameCount[7]= 0;
  frameCount[8]= 0;
  


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

   
void buttonCheck() {      //Set button value as byte variable
  
  
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


int subSwitch(int currSubVal = 0, int maxSubVal = 0, int minSubVal = 0, byte boostMode = 0){      // Loop through sub modes with buttons. Maintain currSub else. Cycle speed increased once if held 
    //Button functions: (bUp = Sub++), (bDown = Sub--) 
    //Takes current sub mode and total number of sub modes from passing function. Returns updated current sub mode. 
  
  
  unsigned int static holdCycles = 0;           //Current count of cycles while button is held
  byte static subValChange = 1;                 //Number to increment/decrement subVal. Increases if hold cycle check passes. 
  byte const cyclesBeforeBoost = 25;            //Number of cycles to count before boosted speed begins.
  byte const cyclesExtraBoost = 50;             //Number of cycles to count before extra boost applied. 
  
  


    //Check button state and update control variables.
  if( currButton != 0 ){

       //Set subValChange based on hold cycle count and mode parameter. Increases value change speed if button is held. 
    if (holdCycles > cyclesExtraBoost ){
      if (boostMode == 0){
        subValChange = 100;
      }else{
        subValChange = 5;
      }
    }else if (holdCycles > cyclesBeforeBoost ){
      if (boostMode == 0){
        subValChange = 10;
      }else{
        subValChange = 2;
      }
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


void autoThresh(int sigMin = 0, int sigMax = 1023){
  //Calculate and apply an optimal threhsold value based on current signal min and max. Reset wave data after. 

  int sigDiff;              //Signal range difference between min and max 
  int targetAna;            //Target analog value for threshold
  int targetPWM;            //Target PWM value for threshold 
  int sigDivide = 4;        //Value to divide range difference to find target threshold. (For thresh of 1/4 sig range, set to 4. For 1/3, set to 3. Etc. )


    //Calculate and set new threshold value
  sigDiff = sigMax - sigMin;                        //Find signal min/max range difference
  targetAna = (sigDiff / sigDivide) + sigMin;       //Find target thresh analog value. Portion of signal range above signal min. 
  targetPWM = map(targetAna, 0, 1023, 0, 255);      //Find target thresh PWM value. Map analog range to PWM range. 
  threshOut = targetPWM;                            //Set global threshold variable to new target value
  analogWrite(threshOutPin, threshOut);             //Set threshold PWM out to target value.

    //Display confirmation message. Delay to allow threshold value to stabilize. Reset wave data values after. 
      //Minimum time for 99% stabile threshold: 1030mS (Due to RC lowpass filter with 10kΩ and 22uF)
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Auto-calibrating");
  lcd.setCursor(0,1);
  lcd.print("Threshold levels");
  delay(900);
  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("New Thresh:");
  delay(700);
  lcd.setCursor(11,0);
  lcd.print(String( analogRead(threshInPin) )); 
  lcd.setCursor(0,1); 
  lcd.print("Resetting data");
  delay(800);

    //Reset wave data
  waveReset();              

    //Trigger mode label reprint 
  modeSwitchFlag = true;                              
}

void threshMain(){      //Threshold settings and current analog value display 

    //Print control variables
  String stThresh;
  String stPhase;
  byte stCurrThreshLength = 0;
  byte stCurrPhaseLength = 0;
  byte static stPrevThreshLength = 0;
  byte static stPrevPhaseLength = 0;

    //Thresh sample variables. 
  const unsigned int sampleLoops = 60;          //Number for samples to be taken. Must be <64 to prevent possible overflow of threshSampleSum unsigned int value 
  unsigned int threshSampleSum = 0;             //Running total of sample values to be divided by number of loops for averaging
  unsigned int threshSampleAvg = 0;             //Average result of samples to be printed. 


    //Print mode label if mode has changed.  Set in modeSwitch().
  if(modeSwitchFlag == true){
  lcd.setCursor(0,0);
  lcd.print("Threshold:");
  lcd.setCursor(0,1);
  lcd.print("Phase mS:");
  }


    //Take measurements of threshold analog value and produce average. 
  for(unsigned int i= 0; i < sampleLoops; i++){
    threshSampleSum += analogRead(threshInPin);
  }  
  threshSampleAvg = threshSampleSum / sampleLoops;


    //Set string values for printing
  stThresh = String(threshSampleAvg);
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


    //Print string values and update control variables
  lcd.setCursor(10,0);
  lcd.print(stThresh);
  lcd.setCursor(9,1);
  lcd.print(stPhase);
  stPrevThreshLength = stCurrThreshLength;
  stPrevPhaseLength = stCurrPhaseLength;


    //Update and output new PWM threshold setting value. 
  threshOut = subSwitch(threshOut, 255, 0, 1);      //Update threshold setting with Up/Down buttons. max value 250, min value 10. Disable x100 boosted change rate. 
  analogWrite(threshOutPin, threshOut);             //Set sub mode changes
  
  
    //Prevent label from reprinting until next mode change. 
  modeSwitchFlag = false;
}


void frameRateMain(){     //Frame rate settings and current value display for frame count calculations. 
  

  int newFrameRate;
  String stRate;
  byte stCurrRateLength = 0;
  byte stPrevRateLength = 0;

    //Copy current frame rate value for reference. 
  newFrameRate = frameRate;

    //Print labels
  lcd.setCursor(0,0);
  lcd.print("Set F Rate:     ");
  lcd.setCursor(0,1);
  lcd.print("[SELECT] to save");


    //Perform rate adjustments.  
    //While loop with local variables used to prevent accidental changes and to force waveReset after changes accepted. 
    //Mode switches without value changes saved if bLeft or bRight. 
  while( (currButton != bLeft) && (currButton != bRight) ){

      //Set string value for printing
    stRate = String(newFrameRate);
    
      //Update current value string length. Clear value display if character length decreased. 
      //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)     
    stCurrRateLength = stRate.length();
    if(stCurrRateLength < stPrevRateLength){
      lcd.setCursor(11,0);
      lcd.print("     ");
    }

      //Print string value and update for next loop comparison
    lcd.setCursor(11,0);
    lcd.print(stRate);
    stPrevRateLength = stCurrRateLength;

      
      //Update newFrameRate setting with Up/Down buttons. max value 20,000Hz, min value 1Hz
      //Min and max numbers selected to be within devices error free frequency range. 
    newFrameRate = subSwitch(newFrameRate, 20000, 1);       

      //Check for save request. 
    if(currButton == bSelect){
      frameRate = newFrameRate;         //Save local loop variable value to global control variable. 
      waveReset();                      //Run waveReset to recalculate values and clear data
      break;    
    }

      //Update button control value and keep background wave data current. 
    buttonCheck();
    ISRwaveCalc();
  }
}


void frameGoalMain(){     //Frame count goal number settings and current value display for frame count calculations. 

  int newFrameGoal;
  String stGoal;
  byte stCurrGoalLength = 0;
  byte stPrevGoalLength = 0;

    //Copy current frame rate value for reference. 
  newFrameGoal = frameGoalNum;

    //Print labels
  lcd.setCursor(0,0);
  lcd.print("Set Goal F#:    ");
  lcd.setCursor(0,1);
  lcd.print("[SELECT] to save");


    //Perform rate adjustments.  
    //While loop with local variables used to prevent accidental changes and to force waveReset after changes accepted. 
    //Mode switches without value changes saved if bLeft or bRight. 
  while( (currButton != bLeft) && (currButton != bRight) ){

      //Set string value for printing
    stGoal = String(newFrameGoal);
    
      //Update current value string length. Clear value display if character length decreased. 
      //(Without this, a value change from "10" to "9" would display as "90" due to LCD leaving characters on if not addressed)     
    stCurrGoalLength = stGoal.length();
    if(stCurrGoalLength < stPrevGoalLength){
      lcd.setCursor(12,0);
      lcd.print("    ");
    }

      //Print string value and update for next loop comparison
    lcd.setCursor(12,0);
    lcd.print(stGoal);
    stPrevGoalLength = stCurrGoalLength;

      
      //Update newFrameRate setting with Up/Down buttons. max value 2,000 frames, min value 1 frame
      //Min and max numbers selected to keep possible values for frameGoal, frameOver, and frameUnder inside signed long valid range. 
    newFrameGoal = subSwitch(newFrameGoal, 2000, 1);       

      //Check for save request. 
    if(currButton == bSelect){
      frameGoalNum = newFrameGoal;      //Save local loop variable value to global control variable. 
      waveReset();                      //Run waveReset to recalculate values and clear data
      break;    
    }

      //Update button control value and keep background wave data current. 
    buttonCheck();
    ISRwaveCalc();
  }
}


void frameCountMain(){      //Display setting for frame count results


  byte static currSubMode = 4;                                                          //Store current sub mode. (Value sets boot default)
  const byte frUnderMoreThan3 = 0;                                                      //Display number of frames below goal -3
  const byte frUnder3 = 1;                                                              //Display number of frames equal to goal -3
  const byte frUnder2 = 2;                                                              //Display number of frames equal to goal -2
  const byte frUnder1 = 3;                                                              //Display number of frames equal to goal -1
  const byte frGoal = 4;                                                                //Display number of frames equal to goal
  const byte frOver1 = 5;                                                               //Display number of frames equal to goal +1
  const byte frOver2 = 6;                                                               //Display number of frames equal to goal +2
  const byte frOver3 = 7;                                                               //Display number of frames equal to goal +3
  const byte frOverMoreThan3 = 8;                                                       //Display number of frames above goal +3 

  byte static cursorSub;                                                                //Cursor position to print selected value. 

                                                       

    //Print mode label and current frame goal number if mode has changed.  Flag set in modeSwitch().
  if(modeSwitchFlag == true){
  lcd.setCursor(0,0);
  lcd.print("Frame# Goal:    ");
  lcd.setCursor(12,0);
  lcd.print(frameGoalNum);
  }

    //Update selected mode.
  currSubMode = subSwitch(currSubMode, 8, 0);

    //Print mode label if mode has changed. 
  if( modeSwitchFlag == true ){
    lcd.setCursor(0,1);
    switch (currSubMode){
      case frUnderMoreThan3:
            lcd.print("<Goal-3:        ");
            cursorSub = 8;     
            break;
      case frUnder3:
            lcd.print("Goal-3:         ");
            cursorSub = 7;
            break;
      case frUnder2:
            lcd.print("Goal-2:         ");
            cursorSub = 7;
            break;
      case frUnder1:
            lcd.print("Goal-1:         ");
            cursorSub = 7;
            break;
      case frGoal:
            lcd.print("=Goal:          ");
            cursorSub = 6;
            break;
      case frOver1:
            lcd.print("Goal+1:         ");
            cursorSub = 7;
            break;  
      case frOver2:
            lcd.print("Goal+2:         ");
            cursorSub = 7;
            break;
      case frOver3:
            lcd.print("Goal+3:         ");
            cursorSub = 7;
            break;
      case frOverMoreThan3:
            lcd.print(">Goal+3:        ");
            cursorSub = 8;
            break;
    }
  }


    //Print current count for selected mode
  lcd.setCursor(cursorSub, 1);
  lcd.print(frameCount[currSubMode]);


    //Prevent label from reprinting until next mode change. 
  modeSwitchFlag = false;

}


void ppfdMain(byte modeReq = 0){      //Phase, Period, Frequency, and Duty modes top print line display settings
  
  

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
            deciMain = 1;
            deciSub = 2;
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


void ppfdSub(byte currModeVal, byte deciSub){     //Phase, Period, Frequency, and Duty modes bottom print line display settings
  

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
  unsigned long totalCounts;


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
          if (currModeVal == xPhase){
            totalCounts = wavePhaseLive[4];
          }else{
            totalCounts = wavePeriodLive[4];
          }
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


void analogWaveMain(){      //Measure and display analog wave min and max analogRead values. 
  //Displays values after a series of sample loops with a button check between each to maintain user experience. 

    //Sample and measurement control variables
  unsigned int currAnaRead = 0;                   //Current analog read of signal value
  unsigned int currAnaMin = 0xFFFF;               //Current minimum value. Default to max possible value until updated from sample loop
  unsigned int currAnaMax = 0;                    //Current maximum value. Default to min possible value until updated from sample loop
  unsigned int static prevAnaMin = 100;           //Prevous min value. Stores completed sample results. Default to ideal values until updated.
  unsigned int static prevAnaMax = 900;           //Prevous max value. Stores completed sample results. Default to ideal values until updated.
  unsigned long startTime;                        //Time Millis() of sample loop start
  byte loopCount = 0;                             //Current number of completed sample loops
  const byte sampleMillis = 50;                   //Length in millis for loop to run. 
  const byte sampleLoops = 6;                     //Total number of sample loops to run. 

    //Printing control variables
  String stCurrMin;                               //For printing current min value.
  String stCurrMax;                               //For printing current max value. 
  byte stCurrMinLength;                           //Length of current min value string. For clearing the display if necessary. 
  byte stCurrMaxLength;                           //Length of current max value string. For clearing the display if necessary. 
  byte static stPrevMinLength = 0;                //Length of previous min value string. For comparing to current length.  
  byte static stPrevMaxLength = 0;                //Length of previous max value string. For comparing to current length. 


      //Print mode label if mode has changed.  Set in modeSwitch().
  if(modeSwitchFlag == true){
  lcd.setCursor(0,0);
  lcd.print("Wave +Peak:");
  lcd.setCursor(0,1);
  lcd.print("Wave -Peak:");
  }


    //Sample analog wave (sampleLoops) times for (sampleMillis) duration. Check for button press between loops.
    //Averages 445 times per 50ms. Testing performed with no input to cause interrupt ISR requests. Specs will be negatively impacted by interrupt delays.
    //Single sample loop specs: 50mS samples width @ 8.9kHz sample rate is ideal for sampling signals from 20Hz to 890Hz. (based on continuous wave input at 10 samples per period minimum)
    //Overall sample specs: Suitable for signals from 3.4Hz to 2.2kHz. (Based on 6 loops and continuous wave input at 4 samples per period minimum)
  while( (currButton == 0) && (loopCount < sampleLoops) ){

    startTime = millis();                                   //Update start time of loop
      
    while( millis() - startTime < sampleMillis ){           //Sample analog wave and update min/max values
      currAnaRead = analogRead(analogWavePin);  
      if (currAnaRead > currAnaMax){
        currAnaMax = currAnaRead;
      }else if (currAnaRead < currAnaMin){
        currAnaMin = currAnaRead;
      }
    }

    loopCount++;                                            //Increment loop count
    buttonCheck();                                          //Check for button press to exit and respond to requests. 
  }

    //Check if cycle loop completed and update values. Prevents update errors for autoThresh. 
  if(loopCount == sampleLoops){
        //Set string values for printing  
    stCurrMax = String(currAnaMax);
    stCurrMin = String(currAnaMin);
  
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
    prevAnaMin = currAnaMin;
    prevAnaMax = currAnaMax; 
  }


    //Prevent label from reprinting until next mode change. 
  modeSwitchFlag = false;    

      //Run autothresh if select is pressed. Use most recent completed min/max values. 
  if(currButton == bSelect){
    autoThresh(prevAnaMin, prevAnaMax); 
  }

}



void modeSwitch(){      // Loop through modes or reset wave stats with buttons.  Maintain currMainMode else. 
  //Button functions: (bRight = Main++), (bLeft = Main--), (bSelect = Reset stats). Clear display after any accepted input.

    //Mode switch control variables.
  int static currMainMode = 0;                                                          //Store current main mode. Initial value is starting mode after reboot. 
  
 
    //Mode list. Values set rotation order. ***Must be zero referenced and sequential*** 
  const byte mainAnaWave = 0;                                                            //Threshold settings and signal min/max measurement
  const byte mainFrameRate = 1;                                                         //Frame rate settings
  const byte mainFrameGoal = 2;                                                         //Frame count goal settings
  const byte mainFrameCount = 3;                                                        //Frame counts vs goal measurement mode. 
  const byte mainPhase = 4;                                                             //Phase measurement mode
  const byte mainPeriod = 5;                                                            //Period measurement mode
  const byte mainFreq = 6;                                                              //Frequency measurement mode 
  const byte mainDuty = 7;                                                              //Duty cycle measurement mode
  const byte mainThresh =8;                                                            //Analog wave measurement mode. 
  const byte maxMainVal = 8;                                                            //Total number of modes (below mode list count, zero ) 
                                                   
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
    case mainFrameCount:
          frameCountMain();
          break;
    case mainFrameRate:
          frameRateMain();
          break;
    case mainFrameGoal:
          frameGoalMain();
          break;
    case mainAnaWave:
          analogWaveMain();
          break;
    default:
          lcd.setCursor(0,0);
          lcd.print("Mode set error");
  }
}
