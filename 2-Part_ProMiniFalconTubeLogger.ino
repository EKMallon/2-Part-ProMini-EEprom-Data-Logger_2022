// Cave Pearl Project 2-Part logger code by Edward Mallon - modified for e360 course at N.U.
// https://thecavepearlproject.org/2022/03/09/powering-a-promini-logger-for-one-year-on-a-coin-cell/
/*
This program supports an ongoing series of DIY 'Classroom Logger' tutorials from the Cave Pearl Project. 
The goal is to provide a starting point for self-built student projects in environmental monitoring courses.
This low power 2-module iteration runs from a CR2032 coin cell and uses EEprom memory to store sensor readings. 
Data download & logger control are managed through the IDE's serial monitor window at 500000 baud. 
The logger WILL NOT START until those serial handshakes are completed with a UART connection.
The most important rule to follow when adding new sensors is that code can only accept 1, 2, 4, 8 or 16 bytes per record.
These 'powers of 2' fit in the I2C buffer AND divide evenly into the EEproms hardware page size to prevent page wrap arounds.
*/

#include <Wire.h>       // I2C bus coms library: RTC, EEprom & Sensors
#include <EEPROM.h>     // note: requires default promini bootloader (ie NOT optiboot)
#include <avr/power.h>  // for shutting down peripherals to lower runtime current
#include <avr/sleep.h>  // used in readBattery() function
#include <LowPower.h>   // for interval & battery recovery sleeps

// LOGGER OPERATING PARAMETERS:  Adjust the following variables to suit your build!
//---------------------------------------------------------------------------------
const char loggerConfiguration[] PROGMEM = "1112159,John Smiths e360 logger,4k,1.086vref,NTC[7]&LDR[6]NOref104cap,LED_r9_b10_g11_gnd12";
const char deploymentDetails[] PROGMEM = "Description of the current deployment";
                                              // Populate the variables above with information about your logger

int32_t InternalReferenceConstant = 1126400;  // default = 1126400L = 1100mV internal vref * 1024
                                              // gets changed manually in setup via serial menu input option OR
                                              // adding/subtracting 400 from the constant raises/lowers the 'calculated' result from readBattery() by ~1 millivolt,
                                              // simply read the rail with a DVM while running on UART power and change the constant until the calculation is accurate

uint8_t SampleIntervalMinutes = 15;   // Allowed values: 1,2,3,5,10,15,20,30 or 60 - must divide equally into 60!
uint8_t SampleIntervalSeconds = 0;    // minutes must be zero for intervalseconds, used for rapid burn tests only
                                      // NOTE: Make sure your sensor readings don't take longer than your sample interval!
                                      // If you over-run your next alarm you will have to wait 24hours for next wakeup

bool ECHO_TO_SERIAL = false;      // ONLY enable this 'true' for debugging when tethered to USB! enables multiple print statements throughout the code  // also starts the run with no interval sync delay so timestamps are misaligned

#define LED_r9_b10_g11_gnd12      // since we added an extra indicator LED Red LED on D13 gets used if #define LED_r9_b10_g11_gnd12 is commented out

// Create SENSOR definitions HERE to match your loggers configuration: 
// Use these to enable the necessary additions with #ifdef & #endif statements throughout the program

#define LogLowestBattery        // 2-bytes: saves LowestBattery recorded during operation
//#define Si7051_Address 0x40   // 2-bytes: NOTE the si7051 is generally only used for NTC calibrations

// VARIABLES below this point stay the SAME on all machines:
//---------------------------------------------------------------------------
// information printed with the boilerplate
#define fileNAMEonly (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) //from: https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
const char compileDate[] PROGMEM = __DATE__;  //  built-in function in C++ makes text string: Jun 29 2023
const char compileTime[] PROGMEM = __TIME__;  //  built-in function in C++ makes text string: 10:04:18

uint8_t bytesPerRecord = 0;       // gets changed at the beginning of setup to match #defined sensors. MUST divide evenly into EEprom Page buffer AND fit inside I2C buffer
uint16_t EEmemoryPointr = 0;      // a counter that advances through the EEprom memory locations by bytesPerRecord at each pass through the main loop
#define EEpromI2Caddr 0x57        // SET the eeprom address/bytes manually to match your hardware // Run a bus scanner to check where your eeproms are https://github.com/RobTillaart/MultiSpeedI2CScanner
#define EEbytesOfStorage 4096     // Must also set this to the correct value for your storage eeprom
                                  // AT24c256 (red YL-90 module) is 32768 @0x50 =
                                  // OR // AT24c512 (via chip swap) is 65536 bytes @0x50

//defines & variables for ADC & readbattery() function
//------------------------------------------------------------------------------
uint16_t CurrentBattery = 0;
uint16_t LowestBattery = 5764;                                  
uint16_t systemShutdownVoltage = 2795; // MUST be > BrownOutDetect default of 2775mv (which is also the EEprom voltage limit)
byte default_ADCSRA,default_ADMUX;     // stores default ADC controll register settings for peripheral shut down
byte set_ADCSRA_2readRailVoltage, set_ADMUX_2readRailVoltage; // stores custom settings for readbattery() via 1.1 internal band gap reference
volatile uint8_t adc_interrupt_counter;// incremented in readADCLowNoise ISR to calculate average of multiple ADC readings

//defines & variables for DS3231 RTC
//------------------------------------------------------------------------------
#define rtcAlarmInputPin 2          // DS3231's SQW output is connected to interrupt0 pin D2 on the ProMini
#define DS3231_ADDRESS     0x68     // this is the I2C bus address of our RTC chip
#define DS3231_STATUS_REG  0x0F     // reflects status of internal operations
#define DS3231_CONTROL_REG 0x0E     // enables or disables clock functions
#define DS3231_TMP_UP_REG  0x11     // temperature registers (upper byte 0x11 & lower 0x12) gets updated every 64sec

uint32_t loggerStartTime;           // uint32_t is large enough to hold the 10-digit unixtime number
char CycleTimeStamp[] = "0000/00/00,00:00"; //16 character array to store human readble time (without seconds)
uint8_t t_second,t_minute,t_hour,t_day,t_month; // current time variables populated by calling RTC_DS3231_getTime()
uint8_t Alarmday,Alarmhour,Alarmminute,Alarmsecond,AlarmSelectBits; // calculated variables for setting next alarm
uint16_t t_year;                    //current year //note: yOff = raw year to which you need to add 2000
float rtc_TEMP_degC = 0.0;
int16_t rtc_TEMP_integer = 0; 
volatile boolean rtc_INT0_Flag = false;  // volatile because it's changed in an ISR // rtc_d2_alarm_ISR() sets this boolean flag=true when RTC alarm wakes the logger

// temporary 'buffer' variables only used during calculations
//------------------------------------------------------------------------------
bool booleanBuffer;             // boolean for functions that return a true/false or 1/0
uint8_t byteBuffer1 = 0;        // 1-byte (8 bit) type = unsigned number from 0 to 255
uint8_t byteBuffer2 = 0;        // note: uint8_t is the same as byte variable type
int16_t integerBuffer = 9999;   // 2-byte from -32,768 to 32,767
uint16_t uint16_Buffer= 9999;   // 2-byte from 0 to 65535
//int32_t int32_Buffer = 9999;    // 4-byte from -2,147,483,648 to 2,147,483,647
uint32_t uint32_Buffer= 9999;   // 4-byte from 0 to 4,294,967,295
float floatBuffer = 9999.9;     // for float calculations

//--------------------------------------------------------------------------------
// variables for sensors attached: Wrapped in #ifdef / #endif statements
//--------------------------------------------------------------------------------

#ifdef Si7051_Address 
//------------------------------------------------------------------------------
  // we are not using a library - init and read functions for si7051 at end of program
  uint16_t TEMP_si7051=0;   //NOTE sensor output overruns this uint16_t at 40C!
#endif


//======================================================================================================================
//======================================================================================================================
//======================================================================================================================
//*   *   *   *   *   *   *   *   *   *   *   SETUP   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   * 
//======================================================================================================================
//======================================================================================================================
//======================================================================================================================
// NOTE: problems in setup usually call the error() function which shuts down the logger

void setup () {

//=========================================================================================
// Adjust the bytesPerRecord variable to match the #bytes saved at each sampling interval
//=========================================================================================

  #ifdef LogLowestBattery
    bytesPerRecord +=2;   // two byte integer
  #endif
  #ifdef Si7051_Address
    bytesPerRecord +=2;   // two-byte integer  
  #endif
  
//====================================================================================================
// General Startup housekeeping: Set UNUSED digital pins to a known state at startup to reduce current & noise
//====================================================================================================
    digitalWrite(13, LOW); pinMode(13, INPUT);  // turn of D13 onboard red LED by setting D13 to INPUT & LOW
  #ifdef LED_r9_b10_g11_gnd12   // we will use INPUT & PULLUP resistor to PIP the leds to reduce current
    for (int i = 9; i <=12; i++) { digitalWrite(i, LOW);  pinMode(i, INPUT); }
    pinMode(12, OUTPUT); //the common ground line on our RGB led must OUTPUT to allow current
  #endif
                                   
  // set UNUSED digital pins to LOW & OUTPUT so EMI noise does not toggle the pin & draw current
    for (int i = 3; i <=8; i++) { digitalWrite(i, LOW);  pinMode(i, OUTPUT); } //Note: if an interrupt source connected to D3 then the pin must be reset to INPUT

  //Disable UNUSED Peripherals to save power - restart them later when needed - At 3V @ 25°C, the ADC consumes ~87µA so we disable it to save power
    SPCR = 0; power_spi_disable(); // stop the peripheral clock with SPCR = 0 BEFORE calling power_spi_disable(); -same applies to the ADC
    power_timer1_disable(); power_timer2_disable(); // NOTE: DON'T mess with timer0! - other peripherals like the I2C bus require Timer0 operating
    bitSet(ACSR,ACD);             // Disables the analog comparator on AIN0(PD6) and AIN1(PD7)by setting the ACD bit (bit 7) of the ACSR register to one. analog comparator draws ~51µA.

 //============================================================================================
// ADC Configuration: default & modified control register settings saved into storage variables
//============================================================================================
// A3..A0 - make sure pullups are OFF so they dont interefere with ADC readings // ignore A4/5 because I2C bus has hardware pullups (on RTC module)
   digitalWrite(A0,LOW);digitalWrite(A1,LOW);digitalWrite(A3,LOW);
   DIDR0 = 0x0F;          // diconnects the DIGITAL inputs sharing analog pins 0..3 (but NOT on 4&5 which are used by I2C as digital pins) //Once disabled, a digitalRead on those pins will always return zero.
                          // Digital input circuits can 'leak' a relatively high amount of current if the analog input is approximately half-Vcc 

  analogReference(DEFAULT); analogRead(A3);         // sets the ADC channel to A3 input pin
  default_ADCSRA = ADCSRA; default_ADMUX = ADMUX;   // Saves the DEFAULT ADC control registers into byte variables so we can restore those ADC control settings later

  // Set the ADC system clock prescalar to 32 so it operates at 2x the normal speed note: readings at 2x are nearly identical to 1x speed readings
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,0);bitWrite(ADCSRA,ADPS0,1); // https://www.arduino.cc/reference/en/language/functions/bits-and-bytes/bitwrite/
  set_ADCSRA_2readRailVoltage = ADCSRA;    // store the modified ADCSRA register values for use in readBattery() function & while saving eeprom data

  // modify ADC settings to reading the battery/rail voltage using the internal 1.1v reference inside the 328p chip 
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0); // from https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  set_ADMUX_2readRailVoltage = ADMUX;      // store modified ADMUX register values in a variable for use in readBattery()

  ADMUX = default_ADMUX;            //restore the default
  ADCSRA = 0; power_adc_disable();  //turn off the ADC to save power At 3V @ 25°C, the ADC consumes ~87µA
  
//========================================================================================
// Configure the DS3231 Real Time Clock control registers for coincell powered operation
//========================================================================================
  Wire.begin();     // Start the I2C bus // enables internal 30-50k pull-up resistors on SDA & SCL by default
  
  // i2c_setRegisterBit function requires: (deviceAddress, registerAddress, bitPosition, 1 or 0)
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_STATUS_REG, 3, 0);  // disable the 32khz output  pg14-17 of datasheet  // This does not reduce the sleep current but can't run because we cut VCC
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 6, 1); // 0Eh Bit 6 (Battery power ALARM Enable) - MUST set to 1 for wake-up alarms when running from the coincell bkup battery
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 7, 0); // Enable Oscillator (EOSC).  This bit is clear (logic 0) when power is first applied.  EOSC = 0 IS REQUIRED with OUR LOGGER DESIGN:
                                                                // when EOSC (bit7) is 0, the RTC oscillator continues running during battery powered operation. Otherwise it would stop.

  RTC_DS3231_turnOffBothAlarms();                               // stops RTC from holding the D2 interrupt line low if system reset just occured 
  digitalWrite(2, LOW);  pinMode(2, INPUT);                     // D2 INPUT & D2 pullup off because it is not requried with 4k7 hardware pullups on the RTC module
  bitSet(EIFR,INTF0); bitSet(EIFR,INTF1);                       // clears any previous trigger-flags inside the 328p processor for interrupt 0 (D2) &  interrupt 1 (D3)

//============================================================================================================
// Check Previous run Parameters stored in 328p eeprom [ update happens only on 1st run of a brand new logger]
//============================================================================================================
  EEPROM.get(4,InternalReferenceConstant); //in this case .get is reading 4 consecutive bytes into the long (32bit) integer InternalReferenceConstant
  if((InternalReferenceConstant<1000000) || (InternalReferenceConstant>1228800)){ //if value stored in eeprom is outside normal operating parameters
    InternalReferenceConstant=1126400;      // then re-sets it to the default 1126400
  EEPROM.put(4,InternalReferenceConstant);} // and store that default back in the eeprom
  
  SampleIntervalMinutes = EEPROM.read(8); // retrieve 'previous' sampling interval data stored in the CPU's internal 1024 bytes of eeprom space
  SampleIntervalSeconds = EEPROM.read(9); // these numbers will be random the first time the logger is run because the EEprom memory locations are empty
  if((SampleIntervalMinutes>60) || (SampleIntervalSeconds>30))  //if values read from eeprom are outside allowed maximums then reset to 'safe' default values
    { SampleIntervalMinutes=15;SampleIntervalSeconds=0;
      EEPROM.update(8,SampleIntervalMinutes); 
      EEPROM.update(9,SampleIntervalSeconds); }  // .update is the same as .put, except that it only writes the data if there is a change - eeprom has a limited # of write cycles

//===============================================================================================
// Logger Configuration Menu via Serial Monitor  [loops for 8 minutes or until START is selected]
//===============================================================================================
                                      // NOTE: Opening the serial monitor always "restarts" the Arduino
  Serial.begin(500000);               // 500000 baud is max possible with 8Mhz processor http://wormfood.net/avrbaudcalc.php
  
  setup_sendboilerplate2serialMonitor(); // send currently running code fileNAME, deployment details to monitor
//----------------------------------------------------------------------------------------

  setup_displayStartMenu();           // see this function & linked sub functions after end of MAIN loop
//----------------------------------------------------------------------------------------

// check if ECHO is on & confirm with user that's OK
  if(ECHO_TO_SERIAL){                 //confirmation check so we don't waste battery power
  Serial.println(F("Serial Output ON! - NOT compatible w bat. powered operation. Disable? y/n"));
                                      //DO NOT ENABLE ECHO_TO_SERIAL during battery powered logger operation or system wastes a lot of power waiting for a serial handshake that never arrives
  booleanBuffer = true; byteBuffer1 = 0;
    while (booleanBuffer) {
        if (Serial.available()) {byteBuffer1 = Serial.read();} //.read captures only character at a time from the serial monitor window
        
        switch (byteBuffer1) {
          case 'y': 
            ECHO_TO_SERIAL = !ECHO_TO_SERIAL; booleanBuffer = false; break;
          case 'n': 
            booleanBuffer = false; break;
          default: break;
              } // terminates switch case
        }  // terminates while(booleanBuffer)
   } // terminates if(ECHO_TO_SERIAL)
   
  Serial.print(F("Serial "));Serial.println(ECHO_TO_SERIAL ? "ON" : "OFF");  // ? here is short-form of if..else statement which checks the boolean and prints first text if true, second option if false
  Serial.println();

//Final confirmation check before erasing EEprom & starting the logger
  Serial.println(F("Type 'start' (& enter) to clear the EEprom & begin logging"));
  Serial.println(F("Any other input will shut down the logger"));
  Serial.println(F("PROCEEDING at **this** point will ERASE ALL previous DATA!"));
//========================================================================================
  Serial.println();

  String command="";                            // any variables delcared in the setup function get deleted at end of setup function
  boolean goFlagReceived = false;               // so these variables will 'disapear' when we reach the main loop
  Serial.setTimeout(100000);                    // need to set timeout or .readStringUntil would wait for input forever...
  unsigned long startMillis = millis();

do { command = Serial.readStringUntil('\n');    // read serial monitor data into the string until carridge return = \n character

    // here we are using if statements to check the input instead of the switch / case method used above, goFlagReceived only becomes 'true' with valid input
    if(command == "start"){ 
      Serial.print(F("Erasing EEprom: Stay on UART power until done"));
      //------------------------------------------------------------------------------
                
      for (int memoryLocation=0; memoryLocation<EEbytesOfStorage; memoryLocation+=16){  // loop writes 16-bytes at a time into the I2C buffer
          Wire.beginTransmission(EEpromI2Caddr);
          Wire.write(highByte(memoryLocation));       // sends only the HiByte of the 2-byte integer address
          Wire.write(lowByte(memoryLocation));        // send only the LowByte of the address
          for (byte k=0; k<16; k++) { Wire.write(0);} // we are loading the I2C buffer with 16 'zeros' each time
          Wire.endTransmission();                     // only at this command does the I2C bus transmission actually occcur

          // here we use modulo to print a progress dots in the serial monitor as the memory erase proceeds // also generates a return when memoryLocation=0
          if ((memoryLocation % 64) == 0){Serial.print(F("."));} // progress bar dot every 32 memory locations // %(modulo) = reminder after division 
          if ((memoryLocation % 4096) == 0){Serial.println();}   // serial printing takes approximately 10 seconds divided by the baud rate, per character
          
          // then poll the eeprom to see if its ready for the next bytes to be written:
          do{ Wire.beginTransmission(EEpromI2Caddr); }while (Wire.endTransmission() != 0x00);
          // endTransmission returns ZERO for successfully ACKnowledgement ONLY when EEprom is ready for more data
          
      } //terminates: for (int memoryLocation=0; memoryLocation<EEbytesOfStorage; memoryLocation+=16){
     
          Serial.println(); goFlagReceived=true;
          
    } else if(command == "test")    // 'test' is a HIDDEN option not on the displayed menu
           {goFlagReceived=true;}   // 'test' simply skips erasing eeprom: I only use this to make debugging faster
 
  if(goFlagReceived) break;         // breaks out of the while loop when goFlagReceived=true;
 
}while ((millis() - startMillis) < 200000); // terminates the do { command = Serial.readStringUntil - runs for 200 seconds before loop times out with goFlagReceived=false which leads to logger shutdown

if (!goFlagReceived){   // if goflag=false then the loop timed out so shut down the logger
      Serial.println(F("Timeout with NO command recieved -> logger shutting down"));Serial.flush();
      error();          // shut down the logger
}


//========================================================================================
Serial.print(F("Initializing sensors "));
//========================================================================================
// this is where you could configure your control registers & usually take a first reading if its a sensor
// usualy wrapped with #ifdef Sensor_Address  ... #endif statements
// for example: 

CurrentBattery = readBattery(); 

#ifdef Si7051_Address             // using functions at the end of this program
  initSI7051();  // see function at the end of this program
  Serial.println(F("Si7051 started"));Serial.flush();
#endif

//========================================================================================
Serial.println(F("& Starting the logger:"));Serial.flush();

//========================================================================================
// FINAL STARTUP PROCEDURE: is to DELAY START of the logger until 1st sampling alarm is in sync
// otherwise you might get a "clipped interval" at the first hour rollover
//========================================================================================
// ALSO saves logger startup time to 328p internal EEprom WHILE tethered to UART for power
// the saved startup time is used later when setup_sendSensorData2Serial reconstructs TimeStamps during download
//========================================================================================

  RTC_DS3231_getTime();    // this function populates the global variables t_day, t_hour, etc.
 
if(ECHO_TO_SERIAL){        // if ECHO is on you are in debug mode so the regular time sync delay is skipped when sample interval is minutes
//----------------------------------------------------------------------------------------------------------------------------------------
 
    uint32_t timeCalcVariable = RTC_DS3231_unixtime(); //returns date/time as single unixtime long integer
    Serial.println(F("ECHO_TO_SERIAL is ON: Sync delay disabled."));Serial.println();Serial.flush();    
    
    integerBuffer=0;
      if(SampleIntervalMinutes==0){     // we will add brief delay for seconds-only intervals
         if(SampleIntervalSeconds>1){
            integerBuffer = SampleIntervalSeconds - (t_second % SampleIntervalSeconds); // % = modulo = remainder after division
            delay(integerBuffer*1000);  // delay (the remainder) seconds until sample interval divides evenly into current time
         } else {                       // if sample interval seconds is 1 we delay for 3 seconds because remainder calculation fails
          integerBuffer = 3;
          delay(3000);
          }
      }
    loggerStartTime = timeCalcVariable + integerBuffer;   //= RTC_DS3231_unixtime();
    EEPROM.put(0,loggerStartTime); // our time-index value gets stored at memory location 0 in the internal eeprom  .put handles all 4 bytes of the long integer

}else{ 
  // if ECHO_TO_SERIAL is false we are not debugging so delay logger startup until our sampling interval divides evenly into current time
  //---------------------------------------------------------------------------------------------------------------------------------------------------- 

  Alarmday = t_day; Alarmhour = t_hour; Alarmsecond = 0; 
  // Alarmminute = gets calculated and we check for rollovers
      
  if(SampleIntervalMinutes==0){     // sub-minute intervals used for rapid testing, SampleIntervalMinutes must be 0 for sub-minute intervals
        Alarmminute=t_minute+2;     // the delay becomes remainder of current minute + 1 minute
  } else {  //  SampleIntervalMinutes>0  (the normal default)
        Alarmminute=((t_minute/SampleIntervalMinutes)*SampleIntervalMinutes) + (2*SampleIntervalMinutes); 
        // last aligned time + 1 sample interval = NEXT alignment time,  PLUS  one 'extra' SampleInterval added
  }     // end if(SampleIntervalMinutes==0)

  if (Alarmminute > 59) { // checking for Alarmminute roll-over //can result in start delays clipped due to sminutes =0
     Alarmminute = 0; Alarmhour = Alarmhour + 1; //this line does not work with SampleIntervalMinutes =60
        if (Alarmhour > 23) {
          Alarmhour = 0; 
          Alarmday = Alarmday + 1;  // but setAlarm1Simple has no days input?
                                    // what about month rollovers?
        }
   }

// NOW set the 1st -ALIGNED- wakeup alarm time:
  RTC_DS3231_setAlarm1Simple(Alarmhour, Alarmminute);
  RTC_DS3231_turnOnAlarm(1);                // alarm will break the logger out of flashing RED&BLUE light sync delay that follows
  noInterrupts ();                          // make sure we don't get interrupted before we sleep
  bitSet(EIFR,INTF0);                       // clear any previous flags for interrupt 0 (D2) see https://gammon.com.au/interrupts
  rtc_INT0_Flag=false;                      // Flag gets set TRUE only inside rtc_d2_alarm_ISR ISR
  attachInterrupt(0,rtc_d2_alarm_ISR, LOW); // RTC SQW alarms LOW and is connected to pin D2 which is interupt channel 0
  interrupts ();
//---------------------------------------------------------------------------------------------- 
// calculate and save the unix time for the alarm we just programmed // this is the same calculation used in function: RTC_DS3231_unixtime():
// but we are doing it here because UART is still connected to supply power for the save - internal eeprom saving draws alot of current (8mA for EEprom + 5mA for ProMini)
  uint16_Buffer = date2days(t_year, t_month, Alarmday); // the days calculation
  uint32_Buffer = time2long(uint16_Buffer, Alarmhour, Alarmminute, Alarmsecond);
  uint32_Buffer += 946684800;       // add # seconds from 1970 to 2000 which is the delta between Unixtime start & our RTC's internal time start
  loggerStartTime = uint32_Buffer;  // this will be the unixtime when we wake AFTER the sync delay
  EEPROM.put(0,loggerStartTime);    // store this so it can be used reconstructing timestamps in the setup_sendSensorData2Serial() function in future
//------------------------------------------------------------------------------  
  Serial.println(F("Red ( & Blue ) LEDs will now flash @1sec until the logger takes 1st reading")); Serial.flush();
  
  if(!ECHO_TO_SERIAL){          // if it's not being used, shut down the UART peripheral now to save power
    Serial.println(F("Disconnect UART now - NO additional messages will be sent over serial.")); Serial.flush();
    power_usart0_disable();     // we waited until this point because the startup input menu requires serial input via the UART
    digitalWrite(0, LOW);  pinMode(0, OUTPUT); // digital pins 0(RX) and 1(TX) are connected to the UART peripheral inside the 328p chip
    digitalWrite(1, LOW);  pinMode(1, OUTPUT); // Connecting anything to these pins may interfere with serial communication, including causing failed uploads
  }
//------------------------------------------------------------------------------
// FIRST sampling wakeup alarm is already set But instead of simply going to sleep we will use LowPower.powerDown SLEEP_1S
// to wake the logger once per second to toggle the LEDs ON/Off so user can tell we are in the sync-delay period before logging starts
  #ifdef LED_r9_b10_g11_gnd12   // RED & BLUE leds start in opposite states so they ALTERNATE when toggled by the PIN register
      pinMode(10,INPUT_PULLUP); // D10 [Blue] LED INPUT & PULLUP ON
      pinMode(13,INPUT);        // D13 onboard red LED INPUT with PULLUP OFF  
  #endif

  do{     // see a ProMini pin map to understand why we are using PINB here for the LED controls https://images.theengineeringprojects.com/image/webp/2018/06/introduction-to-arduino-pro-mini-2.png.webp?ssl=1
        #ifdef LED_r9_b10_g11_gnd12
          PINB = B00100100;               // setting any bits in the 328p PIN control registers to 1 TOGGLES the PULLUP RESISTOR on the associated pins
        #else
          PINB = B00100000;               // this toggles ONLY the D13 led // ~50uA to light RED onboard led through D13s internal pullup resistor
        #endif 
        LowPower.powerDown(SLEEP_1S, ADC_ON, BOD_OFF);   //ADC_ON preserves the existing ADC state - if its already off it stays off
    }while(!rtc_INT0_Flag);               // flag variable only becomes 'true' when RTC alarm triggers execution of rtc_d2_alarm_ISR
    
      pinMode(13,INPUT);                  // D13 [red] indicator LED PULLUP OFF   // alt:  bitClear(PORTB,5); would also do this job
      pinMode(10,INPUT);                  // D10 [Blue] LED PULLUP OFF            // alt:  bitClear(PORTB,2);

  RTC_DS3231_turnOffBothAlarms();         // Note: detachInterrupt(0); was already done inside the rtc_d2_alarm_ISR 

  } //terminates if(ECHO_TO_SERIAL){

//==========================================================================================
//==========================================================================================
}  // terminator for void setup()


//==========================================================================================
//==========================================================================================
//========================== START OF MAIN loop()===========================================
//==========================================================================================
//==========================================================================================
//================================================== START OF MAIN loop()===================
//==========================================================================================
//==========================================================================================

void loop(){

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  *  *  *  *  Set the next RTC wakeup alarm  *  *  *  *  *  *
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
   RTC_DS3231_getTime();
   LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // short sleeps for Cr2032 battery recovery after EVERY I2C exchange
   
//calculate the time for your next RTC alarm:
//------------------------------------------------------------------------------
    Alarmday = t_day;            // Alarmday = now.day(); //with #include <RTClib.h>
    Alarmhour = t_hour;          // Alarmhour = now.hour(); //with #include <RTClib.h>
    Alarmminute = t_minute + SampleIntervalMinutes; //Alarmminute = now.minute()+SampleIntervalMinutes; 
    Alarmsecond = t_second + SampleIntervalSeconds; //only used for special testing & debugging runs - usually gets ignored

// Check for TIME ROLLOVERS: THEN SET the next RTC alarm
//------------------------------------------------------------------------------
if (SampleIntervalMinutes > 0)  //when our interval alarm is in minutes
    {
      if (Alarmminute > 59) {         //error catch - if alarmminute=60 or greater the interrupt will never trigger
        Alarmminute = 0; Alarmhour = Alarmhour + 1;
        if (Alarmhour > 23) {
          Alarmhour = 0; 
          LowestBattery = CurrentBattery;  // reset prevents an accidental low read from permanently affecting lobat
        }
      }  //terminator for if (Alarmminute > 59) rollover catching

      RTC_DS3231_setAlarm1Simple(Alarmhour, Alarmminute);
    }  //for if (SampleIntervalMinutes > 0)

else    //to get sub-minute alarms use the full setA1time function
    
    {  // for testing & debug I sometimes want the alarms more frequent than 1 per minute.
      if (Alarmsecond >59){
        Alarmsecond =0; Alarmminute = Alarmminute+1;  
        if (Alarmminute > 59) 
        {                               //  error catch - if alarmminute>=60 the interrupt never triggers due to rollover!
          Alarmminute =0; Alarmhour = Alarmhour+1; 
          if (Alarmhour > 23) {         //  uhoh a day rollover, but we dont know the month..so we dont know the next day number?
            Alarmhour =0;
            LowestBattery = CurrentBattery;  // reset prevents an accidental low read from permanently affecting lobat
          }
        }
      }
      
      //function expects: (byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmSelectBits, bool A1Dy, bool A1h12, bool A1PM)
      AlarmSelectBits=0b00001000;       // ALRM1_SET bits and ALRM2_SET are 0b1000 and 0b0111 respectively.
      RTC_DS3231_setA1Time(Alarmday, Alarmhour, Alarmminute, Alarmsecond, AlarmSelectBits, false, false, false);
      LowPower.powerDown(SLEEP_120MS, ADC_ON, BOD_OFF);  // RTC memory register WRITING time & battery recovery time
} // terminator for second else case of if (SampleIntervalMinutes > 0) 
    
  RTC_DS3231_turnOnAlarm(1);
  if(ECHO_TO_SERIAL){
      sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);
      Serial.println(); Serial.print(F(">Logger woke at: ")); Serial.print(CycleTimeStamp); // sprintf ref:  http://www.perlmonks.org/?node_id=20519
      Serial.print(F(" Alarm Set:")); Serial.print(t_hour, DEC);Serial.print(F(":"));Serial.print(t_minute, DEC);Serial.print(F(":"));Serial.print(t_second, DEC);
      Serial.print(F(" Next Read in: ")); Serial.print(SampleIntervalMinutes);Serial.print(F("m "));
      Serial.print(SampleIntervalSeconds);Serial.println(F("s")); Serial.flush();
    }

  CurrentBattery = readBattery(); // Note: SLEEP_15MS embedded in readBattery function
    if(ECHO_TO_SERIAL){           // readBat does not usually set LObat which generally happens during high current drain EEsave at end of main loop
      Serial.print(F(", Current Bat[mV]: "));Serial.print(CurrentBattery);
    }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// *  *  *  *  READ your sensors here -AFTER- wake-up alarm set!  *  *  *  *  *
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#ifdef Si7051_Address
//------------------------------------------------------------------------------
    TEMP_si7051 = readSI7051();  // see functions at end of this program
      #ifdef Si7051_Address
        Serial.print(F(", SI7051 temp: "));Serial.print(((175.26*TEMP_si7051)/65536.0)-46.85 ,3);Serial.flush();//print 3 decimals
      #endif
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);  //  battery recovery time
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// HEARTBEAT pip of the LED at the end of the senor readings // draws 30-50uA
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
// also provides some battery recovery time before data is saved to EEprom

    #ifdef LED_r9_b10_g11_gnd12
      pinMode(11,INPUT_PULLUP);
    #else  //use the red led on D13
      pinMode(13,INPUT_PULLUP);
    #endif
      LowPower.powerDown(SLEEP_120MS, ADC_ON, BOD_OFF);  
    pinMode(11,INPUT); // D11 [Green] LED pullup Off
    pinMode(13,INPUT); // pin13 indicator LED pullup Off


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  *  *  *  SAVE NEW SENSOR READINGS into EEprom & READ battery after *  *  *  *
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// the number of bytes you transfer here must match the number in bytesPerRecord
// AND bytes written to per cycle MUST divide evenly into the eeproms pagesize
// So each cycle can only add 1,2,4,8 or 16 bytes - not 3 , not 5, not 7 etc. 
// or you bork the 'powers of 2' math required by page boundaries in the EEprom
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  the general pattern when sending bytes to store in an I2C eeprom:

//  Wire.beginTransmission(EEpromAddressonI2Cbus);  // first byte in I2C buffer
//  Wire.write(memoryAddress >> 8);                 // MSB is second byte
//  Wire.write(memoryeeAddress & 0b11111111);       // LSB is third byte
//    Wire.write(byte);       // adds 1st byte of SAVED data to buffer
//    Wire.write(byte);       // adds 2nd byte of SAVED data to buffer
//   -multiple wire.writes-   // CONTINUE adding to 16 DATA bytes per record -divide the sensor variables up into individual bytes for sending
//  Wire.endTransmission();   // Only when this command executes does the data get sent over the I2C bus

  // beginTransmission() and write() are slightly misleading terms, they do NOT send commands/packets to the I2C device. 
  // They are simply queuing commands, which means they are adding bytes to an internal buffer in the Wire/TWI library. 
  // This internal buffer is not sent to the I2C device on the bus until end.Transmission() is called 
  // estimate about 100us per byte at 100khz bus = 0.7milliseconds for 3(adr)+4(payload) bytes
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// Setup ADC to read the coincell voltage DURING the EEprom data save:
  power_adc_enable();
    ADMUX = set_ADMUX_2readRailVoltage; ADCSRA = set_ADCSRA_2readRailVoltage; 
    bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,1);   // 128 prescalar =67 kHz, = slower than normal ~208uS/ADC readings 
    bitSet(ADCSRA,ADSC);                                  // trigger a 1st throw-away ADC reading to engauge the Aref capacitor //1st read takes 20 ADC clock cycles instead of usual 13
  
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);        // Aref Rise time can take 5-10 milliseconds after re starting the ADC so 15ms of ADC_ON powerDown sleep works ok!

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Wire.beginTransmission(EEpromI2Caddr);  // Starts filling the I2C transmission buffer:
  Wire.write(highByte(EEmemoryPointr));   // send the HighByte of the address
  Wire.write(lowByte(EEmemoryPointr));    // send the LowByte of the address    // Note: EEmemoryPointr gets advanced at the end of the main loop  
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// NOW load the sensor variables - one byte at a time - into the I2C transmission buffer with Wire.write statements
// ORDER listed here MUST EXACTLY MATCH the order of the bytes retrieved in setup_sendSensorData2Serial function
// ALSO NOTE we are using ZEROs as our END OF FILE marker to stop the download ( this is why we filled the eeprom with zeros at startup)
// So we must implement a 'zero trap' on first byte of each record and bump it to '1' if it actualy was zero - this occasionally causes a data error

#ifdef LogLowestBattery                       // INDEX compression converts battery reading to # less than 255 which can be stored in one byte eeprom memory location
//----------------------------------------------------------------------------------------------------
  byteBuffer1 = lowByte(LowestBattery);
  if(byteBuffer1<1){byteBuffer1=1;}           // First data saved in record must have Zero Trap to preserve zero EOF indicators in EEprom
        Wire.write(byteBuffer1);              // first byte added to I2C buffer
  byteBuffer2 = highByte(LowestBattery);
        Wire.write(byteBuffer2);              // 2nd byte of data added to I2C buffer
#endif //LogLowestBattery

#ifdef Si7051_Address
//------------------------------------------------------------------------------
  byteBuffer1 = lowByte(TEMP_si7051);   //NOTE TEMP_si7051 overruns this uint16_t if temps >40C!
  //if(byteBuffer1==0){byteBuffer1=1;}   // note the zero trap is only necessary here if the si7051 is the first/only sensor data being saved
        Wire.write(byteBuffer1);
  byteBuffer2 =  highByte(TEMP_si7051); 
        Wire.write(byteBuffer2);
#endif 

// ---------------- add more sensor data here as required -----------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    Wire.endTransmission(); // ONLY AT THIS POINT do the bytes accumulated in the I2C buffer actually get sent over the wires
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Then the EEPROM enters an internally-timed write cycle to memory which takes ~3-10ms
// 4k AT24c32 write draws ~10mA for about 10ms @3mA, but newer eeproms can take only only 5ms @3mA
// the coincell battery experiences a significant voltage droop from its internal resistance under this load
// so we read the battery voltage during this load event for a true Li battery reading
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC));  // this is another throw away ADC reading
    bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC));  uint16_Buffer=ADC; //ADC a macro command that combines the peripherals two output registers into one integer
    ADMUX = default_ADMUX; ADCSRA = 0; power_adc_disable(); // restore defaults & turn off the ADC peripheral

    LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF); // now shut down the processor and let the battery recover from the EEprom save event 
    // NOTE .powerDown ONLY works with the 4K eeproms -larger eeproms require the bus to keep running with: LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_ON); but this FREEZES the 4k eeproms

  LowestBattery = InternalReferenceConstant / uint16_Buffer;   
    if(ECHO_TO_SERIAL){
      Serial.print(F(", Lowest Bat[mV]: "));Serial.println(LowestBattery);Serial.flush();
      } 
  if (LowestBattery <= systemShutdownVoltage){
  error(); // shutdown down the logger
  } 

  EEmemoryPointr += bytesPerRecord;
  if( EEmemoryPointr >= EEbytesOfStorage){
      // then the eeprom memory is full!
      error();  // so shutdown down the logger
      }

  sleepNwait4RTCalarm(); // sleep till the next alarm

//======== END OF MAIN loop()===============================================================
//==========================================================================================
//==========================================================================================
   }     // terminator for MAIN loop()========================================
//==========================================================================================
//==========================================================================================


//==========================================================================================
//==========================================================================================
//   *  *   *  *  *  *  * Processor SLEEPING function  *  *  *  *  *  *  *  *  *
//==========================================================================================
//==========================================================================================
void sleepNwait4RTCalarm() {         //NOTE all existing pin states are preserved during sleep

  pinMode(2, INPUT);  //D2 pullup off - not needed with hardware pullups on RTC module
  noInterrupts();
  bitSet(EIFR,INTF0); // clears interrupt 0's flag bit before attachInterrupt(0,isr,xxxx)
  attachInterrupt(0,rtc_d2_alarm_ISR,LOW); //RTC alarm connected to pin D2 // LOW assures it will always respond if the RTC alarm is asserted
  interrupts();
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF); // ADC_ON simply preserves whatever the current ADC status is (in our case it's already OFF...)
  
  //HERE AFTER WAKING  // note that detachInterrupt(0); happened inside the ISR
    
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_STATUS_REG);
  Wire.write(0); // clearing the entire status register turns Off (both) RTC alarms though technically only the last two bits need to be set
  Wire.endTransmission();
  rtc_INT0_Flag = false;  // clear the flag we use to indicate the RTC alarm occurred
  LowPower.powerDown(SLEEP_120MS, ADC_ON, BOD_OFF); // coincell voltage recovery time from Wakup & I2C transaction
  
}  //terminator for sleepNwait4RTCalarm

void rtc_d2_alarm_ISR() {   // this function gets called by attachInterrupt above
  detachInterrupt(0);       // detaching inside the ISR itself makes sure it only triggers ONE time
  rtc_INT0_Flag = true;     // this flag only necessary with interrupt generating sensors on D3
}

//==========================================================================================
//==========================================================================================
//  *  *  *  *  *  *  *  *  *  FUNCTIONS called during Setup()  *  *  *  *  *  *  *  *  *  * 
//==========================================================================================
//==========================================================================================
void setup_sendboilerplate2serialMonitor(){      
//-----------------------------------------------------------------------------------------
//NOTE:(__FlashStringHelper*) is needed to print variables is stored in PROGMEM instead of regular memory
    Serial.print(F("CodeBuild: ")); Serial.print(fileNAMEonly);         // or use Serial.println((__FlashStringHelper*)codebuild); //for the entire path + filename
    Serial.print(F(" Compiled: "));Serial.print((__FlashStringHelper*)compileDate);
    Serial.print(F(" @ ")); Serial.println((__FlashStringHelper*)compileTime);
    Serial.print(F("Hardware: ")); Serial.println((__FlashStringHelper*)loggerConfiguration);
    Serial.print(F("Last Deployment: ")); Serial.println((__FlashStringHelper*)deploymentDetails); 
    Serial.flush();
}

void setup_printMenuOptions(){  // note: setup_sendboilerplate2serialMonitor(); runs once on startup before this
//-----------------------------------------------------------------------------------------

  Serial.println();
  RTC_DS3231_getTime();     // reads current clock time  and display it via CycleTimeStamp
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);  //combines numeric time variables to a human readable text string
  Serial.print(F("Current Logger time:"));Serial.print(CycleTimeStamp);Serial.print(F(":"));Serial.print(t_second); //seconds separate because usually value is zero
  EEPROM.get(4,InternalReferenceConstant);
  Serial.print(F(", VREF const.="));Serial.print(InternalReferenceConstant);Serial.println(F(" (default:1126400)"));
  if(ECHO_TO_SERIAL){
  Serial.print(F("SERIAL output ON"));
  }else{
  Serial.print(F("SERIAL output OFF"));
  } 
  SampleIntervalMinutes = EEPROM.read(8);
  SampleIntervalSeconds = EEPROM.read(9);
  Serial.print(F(", Every "));Serial.print(SampleIntervalMinutes);Serial.print(F("m "));Serial.print(SampleIntervalSeconds);Serial.print(F("s"));
  Serial.print(F(" logs: "));
    #ifdef LogLowestBattery
      Serial.print(F("LoBat.[mv], "));
    #endif
    #ifdef Si7051_Address
      Serial.print(F("SI7051 T°C, "));
    #endif
    Serial.println();
    
    Serial.println();
    Serial.println(F("Select one of the following options:"));
    Serial.println(F("  [1] DOWNLOAD Data"));
    Serial.println(F("  [2] Set CLOCK        [3] Set INTERVAL"));
    Serial.println(F("  [4] Toggle SERIAL    [5] Change VREF"));
    Serial.println(F("  [6] START logging"));
    Serial.println(); Serial.flush();
}   //terminates setup_printMenuOptions

void setup_displayStartMenu() {       
//-----------------------------------------------------------------------------------------

    while (Serial.available() > 0){byteBuffer1 = Serial.read();}  // this just clears out any residual data in serial send buffer before starting our menu
  
    Serial.setTimeout(1000);      // 1000 milliseconds is the default timeout for the Serial.read(); command
    uint8_t inByte=0;
    boolean wait4input = true;
    boolean displayMenuAgain = true;
    uint32_Buffer = millis();  //Beginning of time-out period must be unsigned long variable

  do{ inByte=0;
    if (displayMenuAgain) { 
      setup_printMenuOptions(); displayMenuAgain=false;}

    if (Serial.available()) { 
      inByte = Serial.parseInt(); } //from https://forum.arduino.cc/t/simple-serial-menu-without-a-library/669556
    
    switch (inByte) {  //NOTE: switch can also accept 'letter inputs' with single quotes: case 'Z':
            case 1:
              setup_sendSensorData2Serial(true);  displayMenuAgain=true;  break;
            case 2:
              setup_setRTCtime(); Serial.setTimeout(1000); 
              displayMenuAgain=true;  break;
            case 3:
              setup_changeSampleInterval(); Serial.setTimeout(1000); 
              displayMenuAgain=true;  break;
            case 4:
              ECHO_TO_SERIAL = !ECHO_TO_SERIAL;
              displayMenuAgain=true;  break;
            case 5:
              updateVref(); Serial.setTimeout(1000);
              displayMenuAgain=true;  break;
            case 6:
              wait4input=false;   break;      // sends you back to Setup where displayStartMenu was first called
            //case '7':  //a hidden debugging option not displayed in the startmenu
            //  setup_sendSensorData2Serial(false);  displayMenuAgain=true;  break;
              
            default:    // Check milliseconds elapsed & send logger into shutdown if we've waited too long
              if ((millis() - uint32_Buffer) > 480000) { // start menu has an 480000 = 8 minute timeout
              Serial.println(F("Startmenu Timed out with NO command recieved?"));
              Serial.println(F("Logger is shutting down...")); Serial.flush(); error(); 
              }
              break;
         }                  //  terminates switch-case cascade
      }while(wait4input);   //  the do-while loop keeps checking for input if wait4input=true;
  return;
}

void updateVref(){  //default =1126400L = 1100mV * 1024 
//-----------------------------------------------------------------------------------------
do {
    Serial.println(F("Input a new Vref constant between 1000000 and 1228800:")); //1,126,400L = default for 1100mV * 1024
    Serial.setTimeout(100000);  //parseInt will normally “time out” after default set point is 1 second (1000 milliseconds).
    InternalReferenceConstant = Serial.parseInt(); //parseInt() actually returns a long
    }while((InternalReferenceConstant<1000000) || (InternalReferenceConstant>1228800)); // if condition fails & you have to re-enter the number
   Serial.print(F("Vref set to: ")); Serial.println(InternalReferenceConstant);
   EEPROM.put(4,InternalReferenceConstant); // every time you run the logger it will retrieve the interval from the previous run 
   return;
} // terminates updateVref

void setup_changeSampleInterval(){
//-----------------------------------------------------------------------------------------
do {
    Serial.println();Serial.println(F("Input a sampling interval of 1,2,5,15,30,60 or [0] minutes:"));
    byteBuffer1 = 0;
    Serial.setTimeout(100000); SampleIntervalMinutes = Serial.parseInt(); 
    byteBuffer1 = SampleIntervalMinutes ? 60 % SampleIntervalMinutes : 0; // ERROR check: input must be valid divisor of 60 OR zero
    }while((byteBuffer1 !=0) || (SampleIntervalMinutes>60));              // or while condition fails & you have to re-enter the number
    
    Serial.print(F("  Sample Interval set to: ")); Serial.print(SampleIntervalMinutes);Serial.println(F(" min"));
    
    if (SampleIntervalMinutes>0){ SampleIntervalSeconds = 0; }
    // sub minute sampling ONLY for rapid burn tests so this is a HIDDEN OPTION for rapid debugging/testing (it's not displayed on the menu)
    // IF you enter a zero for minutes, then the logger assumes you want an interval in seconds:
    if (SampleIntervalMinutes==0){ 
        do {
         Serial.println(F("Enter sub-minute interval as 1,2,3,5,10,15, or 30 seconds: (sampling time may over-run the alarm!)"));
          byteBuffer2 =0;
         SampleIntervalSeconds = Serial.parseInt();
          byteBuffer2 = SampleIntervalSeconds ? 30 % SampleIntervalSeconds : 0; // ERROR check: input must be valid divisor of 30 OR zero
        }while ((byteBuffer2 !=0) || (SampleIntervalSeconds>30));               // or while condition fails & you have to re-enter the number
      Serial.print(F("  Sub-minute Sample Interval: ")); Serial.print(SampleIntervalSeconds);Serial.println(F(" seconds"));
      }

    if (SampleIntervalMinutes==0 && SampleIntervalSeconds==0){
      Serial.println(F("Invalid Entry: 15 min DEFAULT interval being SET"));Serial.flush();
      SampleIntervalMinutes = 15;SampleIntervalSeconds = 0;
      }
    EEPROM.update(8,SampleIntervalMinutes); // newly entered values are now stored in the 329p's internal eeprom
    EEPROM.update(9,SampleIntervalSeconds); // every time you run the logger it will retrieve the interval from the previous run 
    
 return;
}//setup_changeSampleInterval

void setup_setRTCtime(){
//-----------------------------------------------------------------------------------------
  Serial.println(F("Enter current date/time with digits as indicated:"));  // note Serial.parseInt will ONLY ACCEPT NUMBERS from the serial window!
  Serial.setTimeout(100000); 
  Serial.print(F("YYYY:"));     t_year = Serial.parseInt();      Serial.println(t_year);
  Serial.print(F("MM:"));       t_month = Serial.parseInt();     Serial.println(t_month);
  Serial.print(F("DD:"));       t_day = Serial.parseInt();       Serial.println(t_day);
  Serial.print(F("(24hour) HH:")); t_hour = Serial.parseInt();   Serial.println(t_hour);
  Serial.print(F("MM:"));       t_minute = Serial.parseInt();    Serial.println(t_minute);
  Serial.print(F("SS:"));       t_second = Serial.parseInt();    Serial.println(t_second);

  if (t_month==0 && t_day==0){  // this is a very crude error catch //this needs to be further developed
    Serial.println(F("Not valid input to set RTC time!"));
    return;                    //shut down the logger - user will need to re-open the serial window to restart the logger
    } else {
    RTC_DS3231_setTime(); delay(15);      // give the RTC register memory write a little time
    i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,7,0); //clear the OSF flag after time is set
    } //terminates if (t_month==0 && t_day==0){
}

void setup_sendSensorData2Serial(boolean convertDataFlag){ // called at startup via serial window
//===============================================================================================
// NOTE: the ORDER of BYTES/SENSORS listed in setup_sendSensorData2Serial MUST EXACTLY MATCH
// the order you ADDED those SENSOR READINGS when loading the EEprom write buffer in the main loop

// header information
  setup_sendboilerplate2serialMonitor(); // a description of the deployment should be part of the data output
  Serial.println(F("Convert Unixtime to Excel with:  =CELLref/(24*60*60) + DATE(1970,1,1)"));
  Serial.print(F("UnixTime,"));
  
#ifdef LogLowestBattery
  Serial.print(F("LoBat.[mv],"));
#endif
#ifdef Si7051_Address
  Serial.print(F("SI7051 T°C,"));
#endif
 Serial.println();Serial.flush();

//starting time value was stored in first four bytes of the 328p internal eeprom:
    uint32_t unix_timeStamp;
    EEPROM.get(0,unix_timeStamp); // the loggerStartTime saved at previous logger startup
    SampleIntervalMinutes = EEPROM.read(8);
    SampleIntervalSeconds = EEPROM.read(9);
    
    uint16_t secondsPerSampleInterval;
    if (SampleIntervalMinutes==0){  // sub-minute alarms for accelerated run testing
        secondsPerSampleInterval = SampleIntervalSeconds;
        }else{  //  normal minute based alarms:
        secondsPerSampleInterval = 60UL*SampleIntervalMinutes;
        }
    
    uint32_t EEmemoryPointr = 0;      // a counter that advances through the EEprom Memory in bytesPerRecord increments
    uint32_t RecordMemoryPointer= 0;  // note uint32_t only because oveshoot uint16_t on larger memory systems
    

  do{    // this big do-while loop readback must EXACTLY MATCH the data saving pattern in our main loop:
  //---------------------------------------------------------------------------------------------------------- 
   
  RecordMemoryPointer = EEmemoryPointr;   // a pointer for the location of each byte within a given record
  
  //if the first byte readback process is ZERO then we've reached our end of data marker
  byteBuffer1 = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer); 
  if(byteBuffer1==0 & convertDataFlag){break;}  // this breaks us out of the do-while readback loop

  if (!convertDataFlag){    // output raw bytes exactly as read from eeprom [with no timestamp] // this is ONLY used for debugging
        for (uint8_t j = 0; j < bytesPerRecord; j++) {
        byteBuffer1 = i2c_eeprom_read_byte(EEpromI2Caddr,(RecordMemoryPointer+j));
        Serial.print(byteBuffer1); Serial.print(F(","));   // outputs raw bytes as read from eeprom:
        }     
  } else { //if convertDataFlag is true then eeprom variable bytes get re-constituted back to human readable numbers

  Serial.print(unix_timeStamp);Serial.print(",");
  unix_timeStamp += secondsPerSampleInterval; //increment unix timestamp for the NEXT record after printing

  // order of sensors & bytes listed here must EXACLTY MATCH the order in which you loaded the bytes into the eeprom in the main loop

#ifdef LogLowestBattery   // stored as two data bytes, low byte first
      byteBuffer1 = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);RecordMemoryPointer++;    //Low byte was saved first
      integerBuffer = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);RecordMemoryPointer++;  //Hi byte is next
      integerBuffer = integerBuffer<<8 | byteBuffer1; // this combines the two separate eeprom bytes back into an integer variable
      Serial.print(integerBuffer);Serial.print(F(","));
#endif

#ifdef Si7051_Address 
        byteBuffer1 = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);RecordMemoryPointer++;//low byte
        TEMP_si7051 = i2c_eeprom_read_byte(EEpromI2Caddr,RecordMemoryPointer);RecordMemoryPointer++;//hi byte
        TEMP_si7051 = (TEMP_si7051 << 8) | byteBuffer1;
        Serial.print(((175.26*TEMP_si7051)/65536.0)-46.85,3);Serial.print(F(",")); 
        //integer converted to celcius (3 decimals output)  
        //or Serial.print(TEMP_si7051); to print raw integer 
#endif // #ifdef Si7051_Address

          } //terminator for if(convertDataFlag)

  Serial.println();
  EEmemoryPointr += bytesPerRecord;
  
  } while(EEmemoryPointr <= EEbytesOfStorage); // terminates the big do-while readback loop
  //---------------------------------------------------------------------------------------

  Serial.flush();
}  // terminates setup_sendSensorData2Serial() function


//==========================================================================================
//==========================================================================================
//   *  *   *  *  *  *  * FUNCTIONS called by the Main loop()  *  *  *  *  *  *  *  *  *  *
//==========================================================================================
//==========================================================================================

// ======================================================================================
//   *  *   *  *  *  *  * I2C SENSOR & MEMORY REGISTER FUNCTIONS  *  *  *  *  *  *  *  * 
// ======================================================================================
// see: https://thecavepearlproject.org/2017/11/03/configuring-i2c-sensors-with-arduino/

bool i2c_getRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition) 
//-----------------------------------------------------------------------------------------
{
  byte registerByte;
  registerByte = i2c_readRegisterByte(deviceAddress, registerAddress);
  return ((registerByte >> bitPosition) & 0b00000001);
}

byte i2c_setRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, bool state) 
//-----------------------------------------------------------------------------------------
{
  byte registerByte;
  byte result;
  registerByte = i2c_readRegisterByte(deviceAddress, registerAddress);  //load the existing register contents
  if (state) {    // when state = 1
    registerByte |= (1 << bitPosition);  // bitPosition of registerByte now = 1
  }
  else {         // when state = 0
    registerByte &= ~(1 << bitPosition); // bitPosition of registerByte now = 0
  }
  result = i2c_writeRegisterByte(deviceAddress, registerAddress, registerByte);
  return result;    // result =0 if the writing the new data to the registry went ok
}

byte i2c_readRegisterByte(uint8_t deviceAddress, uint8_t registerAddress)
//-----------------------------------------------------------------------------------------
{
  byte registerData;
  Wire.beginTransmission(deviceAddress); //set destination target
  Wire.write(registerAddress); //assumes register address is <255  - this is not the case for all sensors
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);
  registerData = Wire.read();
  return registerData;
}

byte i2c_writeRegisterByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t newRegisterByte){
//-----------------------------------------------------------------------------------------
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(newRegisterByte);
  byte result = Wire.endTransmission();

  if (result > 0)   //error checking
  {
    if(ECHO_TO_SERIAL){   //NOTE: only call halt on error if in debug mode!
      Serial.print(F("FAIL in I2C register write! Result code: "));
      Serial.println(result); Serial.flush();
      error();
    }
  }
  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);  // some sensors need this settling time after a register change?
  return result;
}

uint8_t i2c_eeprom_read_byte(uint8_t deviceAddress, uint16_t memoryAddress ) {  // called by void setup_sendSensorData2Serial
//-----------------------------------------------------------------------------------------
// unlike i2c_readRegisterByte this function takes a TWO-byte memory address

  uint8_t rdata = 0xFF;
  Wire.beginTransmission(deviceAddress);
  Wire.write(highByte(memoryAddress)); // Address High Byte
  Wire.write(lowByte(memoryAddress));  // Address Low Byte
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress,(uint8_t)1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

// ======================================================================================
//   *  *   *  *  *  *  * BATTERY MONITORING function  *  *  *  *  *  *  *  *  *  *  *
// ======================================================================================
// see: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// long video exlainer: https://www.youtube.com/watch?v=G6dvDgCOyqk&ab_channel=JulianIlett
// for info on sleeping the ADC during readings see:  https://www.gammon.com.au/adc
  
uint16_t readBattery(){    // reads 1.1vref as input against VCC as reference voltage
  power_all_disable();  power_adc_enable(); 
  ADMUX = set_ADMUX_2readRailVoltage;   ADCSRA = set_ADCSRA_2readRailVoltage;
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,0); // 64 prescalar @ 8MHz/64 sets(default)125 kHz ADC clock 
                                                        // typical ADC read takes 13 ADC clock cycles, so default speed is about 9615 Hz (or 0.104 milliseconds per reading).
  bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC));  // triggers a 1st THROW AWAY READING to engage AREF capacitor
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);      // leaves ADC_ON: ~110µA so aref cap charges up during this sleep
    
  uint16_Buffer=0; adc_interrupt_counter = 0;   // reset our accumulator variables
  bitSet(ACSR,ADIF);                  // clears any previous ADC interrupt flags
  bitSet(ADCSRA,ADIE);                // tells ADC to generate processor interrupts to wake the processor when a new reading is ready
  set_sleep_mode( SLEEP_MODE_ADC );   // Enable ADC Noise Reduction Sleep Mode
    do{
          do{ sleep_mode();           // sleep_mode macro combines sleep enable & disable with sleep_cpu command     // Note: sleep_disable(); not needed with sleep_mode();    
          }while (bit_is_set(ADCSRA,ADSC)); // ADC resets ADSC bit to zero ONLY when conversion is finished, otherwise bit is 1 while ADC is reading
            uint16_Buffer += ADC;
    }while (adc_interrupt_counter<4); // uint16_Buffer accumulates the sum of 4 ADC readings

  uint16_t railvoltage = InternalReferenceConstant / (uint16_Buffer>>2); // convert average ADC reading into railvoltage in mV // bitshift >>2 same as divide by 4

  bitClear(ADCSRA,ADIE);              // turn off the ADC interrupts
  bitSet(ACSR,ADIF);                  // clears any ADC interrupt flags in the processor
  ADMUX = default_ADMUX;              // restore defaults
  ADCSRA = 0; power_adc_disable();    // turn off ADC
  // re-enable the other peripherals we turned off with power_all_disable();
  power_timer0_enable(); power_twi_enable();
  if(ECHO_TO_SERIAL){power_usart0_enable();}

  if (railvoltage < LowestBattery) {LowestBattery = railvoltage;}
  
  if (railvoltage < systemShutdownVoltage){
        if(ECHO_TO_SERIAL){
          Serial.print(railvoltage); Serial.println(F(" battery voltage too low!")); Serial.flush();
        }
      error();} // this shuts down the logger
         
  return railvoltage; 
}  // terminator for readBattery()

ISR (ADC_vect){ adc_interrupt_counter++;}  // called by the readBattery() FUNCTION above
// the ADC_vect ISR executes when the ADC generates interrupts - increments the adc_interrupt_counter variable with each new reading

// ======================================================================================
//   *  *   *  *  *  *  *  *  *  *  ERROR HANDLER   *  *  *  *  *  *  *  *  *  *  *  *  *
// ======================================================================================
void error() {

if(ECHO_TO_SERIAL){
   Serial.print(F("Shutting Down: LowBattery @")); Serial.println(LowestBattery); Serial.flush();
}
  
  power_twi_enable();
  RTC_DS3231_turnOffBothAlarms(); //before we disable I2C
  bitSet(EIFR,INTF0);     // clear flag for interrupt 0  see: https://gammon.com.au/interrupts
  bitSet(EIFR,INTF1);     // clear flag for interrupt 1

  pinMode(13, OUTPUT);         // = D13 OUTPUT
  for (byte CNTR = 0; CNTR < 253; CNTR++) {   // FLASH red indicator LED to indicate error state
    PINB = B00100000;                         // ^ TOGGLES D13 LED pullup resistor ON/Off
    LowPower.powerDown(SLEEP_250MS, ADC_ON, BOD_OFF);
  }
  
  bitSet(ACSR,ACD);       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0;   // Disable ADC & SPI // only use PRR after disabling the peripheral clocks, otherwise the ADC is "frozen" in an active state drawing power
  power_all_disable();    // Turn all the internal peripherals off - must come after disabling ADC

 // FLOAT all digital pins so no current can leak after shutdown
    for (int i = 0; i <=13; i++) { 
    pinMode(i, INPUT);  digitalWrite(i, LOW);  
    }
    
  LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
}

//==========================================================================================
//==========================================================================================
//   *  *   *  *  Functions used for RTC control  *  *  *  *  *  *  *  *  *  *  *  *  *
//==========================================================================================
//==========================================================================================
// modified from the original JeeLab's fantastic real time clock library that was released into the public domain 
// =========================================================================================

void RTC_DS3231_getTime(){
//------------------------
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDRESS, 7);
  t_second = bcd2bin(Wire.read() & 0x7F);
  t_minute = bcd2bin(Wire.read());
  t_hour = bcd2bin(Wire.read());
  Wire.read();
  t_day = bcd2bin(Wire.read());
  t_month = bcd2bin(Wire.read());
  t_year = bcd2bin(Wire.read()) + 2000;
  return;
}

void RTC_DS3231_setTime(){
//------------------------
   Wire.beginTransmission(DS3231_ADDRESS);
   Wire.write((byte)0);
   Wire.write(bin2bcd(t_second));
   Wire.write(bin2bcd(t_minute));
   Wire.write(bin2bcd(t_hour));
   Wire.write(bin2bcd(0));
   Wire.write(bin2bcd(t_day));
   Wire.write(bin2bcd(t_month));
   Wire.write(bin2bcd(t_year - 2000));
   //Wire.write(0); //what is the last one for?
   Wire.endTransmission();
  }

void RTC_DS3231_setAlarm1Simple(byte hour, byte minute) {
//-----------------------------------------------------------
  RTC_DS3231_setA1Time(0, hour, minute, 00, 0b00001000, false, false, false);
}

void RTC_DS3231_setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmSelectBits, bool A1Dy, bool A1h12, bool A1PM) {
//-----------------------------------------------------------------------------------------------------------------------------------
//  Sets the alarm-1 date and time on the DS3231, using A1* information
  byte temp_buffer;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x07);    // A1 starts at 07h
  // Send A1 second and A1M1
  Wire.write(bin2bcd(A1Second) | ((AlarmSelectBits & 0b00000001) << 7));
  // Send A1 Minute and A1M2
  Wire.write(bin2bcd(A1Minute) | ((AlarmSelectBits & 0b00000010) << 6));
  // Figure out A1 hour
  if (A1h12) {
    // Start by converting existing time to h12 if it was given in 24h.
    if (A1Hour > 12) {
      // well, then, this obviously isn't a h12 time, is it?
      A1Hour = A1Hour - 12;
      A1PM = true;
    }
    if (A1PM) {
      // Afternoon
      // Convert the hour to BCD and add appropriate flags.
      temp_buffer = bin2bcd(A1Hour) | 0b01100000;
    } else {
      // Morning
      // Convert the hour to BCD and add appropriate flags.
      temp_buffer = bin2bcd(A1Hour) | 0b01000000;
    }
  } else {
    // Now for 24h
    temp_buffer = bin2bcd(A1Hour);
  }
  temp_buffer = temp_buffer | ((AlarmSelectBits & 0b00000100) << 5);
  // A1 hour is figured out, send it
  Wire.write(temp_buffer);
  // Figure out A1 day/date and A1M4
  temp_buffer = ((AlarmSelectBits & 0b00001000) << 4) | bin2bcd(A1Day);
  if (A1Dy) {
    // Set A1 Day/Date flag (Otherwise it's zero)
    temp_buffer = temp_buffer | 0b01000000;
  }
  Wire.write(temp_buffer);
  Wire.endTransmission();
}

void RTC_DS3231_turnOnAlarm(byte Alarm) {
//---------------------------------------
// turns on alarm number "Alarm". Defaults to 2 if Alarm is not 1.
  byte temp_buffer = i2c_readRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG);
  // modify control byte
  if (Alarm == 1) {
    temp_buffer = temp_buffer | 0b00000101;  //bitwise OR //either or both
  } else {
    temp_buffer = temp_buffer | 0b00000110; //Defaults to enable 2 if Alarm is not 1
  }
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG,temp_buffer);
}

void RTC_DS3231_turnOffBothAlarms() {  // from http://forum.arduino.cc/index.php?topic=109062.0
//-----------------------------------
  byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG);
  byteBuffer1 &= B11111100; //with '&=' only the 0's affect the target byte // with '|=' only the 1's will set
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG,byteBuffer1);
  rtc_INT0_Flag = false; //clear the flag we use to indicate the RTC alarm occurred
}

static uint8_t bcd2bin (uint8_t val) {
  return val - 6 * (val >> 4);
}
static uint8_t bin2bcd (uint8_t val) {
  return val + 6 * (val / 10);
}

const uint8_t days_in_month [12] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
//#define SECONDS_FROM_1970_TO_2000 946684800
uint32_t RTC_DS3231_unixtime() {
  uint32_t t;
  uint16_t days = date2days(t_year, t_month, t_day);
  t = time2long(days, t_hour, t_minute, t_second);
  t += 946684800;  // add # seconds from 1970 to 2000 which we took away with y -= 2000
  return t;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
  return ((days * 24L + h) * 60 + m) * 60 + s;
}

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
  if (y >= 2000)
    y -= 2000;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(days_in_month + i - 1);
  if (m > 2 && (y % 4 == 0)) // modulo checks (if is LeapYear) add extra day
    ++days;
  return days + 365 * y + (y + 3) / 4 - 1;
}

// Other RTC refs:
// also see https://github.com/sleemanj/DS3231_Simple/blob/master/DS3231_Simple.cpp for an alt library
// also see https://github.com/mizraith/RTClib
// or https://github.com/akafugu/ds_rtc_lib for more DS3231 specific libs
// https://github.com/MajicDesigns/MD_DS3231/blob/master/src/MD_DS3231.cpp
// https://github.com/JChristensen/DS3232RTC wrapper for the time.h lib
// alternate unixtime calculation: https://github.com/rodan/ds3231
// http://tronixstuff.com/2014/12/01/tutorial-using-ds1307-and-ds3231-real-time-clock-modules-with-arduino/


// ===========================================================================================================
// ============================================================================================================
//   *  *   *  *  *  *  *  *  *  *  SENSOR SPECIFIC FUNCTIONS  *  *  *  *  *  *  *  *  *  *  *  *  *
// ============================================================================================================
// ===========================================================================================================

// ============================================================================================================
// ============================================================================================================
// SI7051 TEMPERATURE SENSOR - here as an 'example' but we will not use this sensor in the course
// ============================================================================================================
// ============================================================================================================
// code from https://github.com/closedcube/ClosedCube_Si7051_Arduino/blob/master/src/ClosedCube_Si7051.cpp
// datasheet at : http://www.silabs.com/Support%20Documents/TechnicalDocs/Si7050-1-3-4-5-A20.pdf  

#if defined(Si7051_Address)   //then include these functions
//--------------------------------------------------------
void initSI7051()
//--------------------------------------------------------
{
  byteBuffer1=0; 
  if(ECHO_TO_SERIAL){
  Serial.println(F("INIT: SI7051 sensor..."));
  Serial.flush(); 
  }
  
  Wire.beginTransmission(Si7051_Address);
  Wire.write(0xE6); //settings regester
  Wire.write(0x0);  //bit 0 and bit 7 to zero sets highest 14 bit resolution on sensor
  byteBuffer1 = Wire.endTransmission();
  if ( byteBuffer1 != 0) {
        if(ECHO_TO_SERIAL){  //if echo is on, we are in debug mode, and errors force a halt.
        Serial.print (F("FAIL Initial control reg write:si7051"));
        }
    error();
    byteBuffer1=0;
  }
}
//--------------------------------------------------------
int readSI7051() 
//--------------------------------------------------------
//Conversion time: 14-bit temperature  10 ms
//Temperature conversion in progress: 120 μA
//peak current during I2C operations 4.0 mA

{
  byteBuffer1=0; 
  Wire.beginTransmission(Si7051_Address);
  Wire.write(0xF3);
  Wire.write(Si7051_Address);   //why is this repeated three times?
  //Wire.write(SI7051_ADDRESS);  
  //Wire.write(SI7051_ADDRESS);
  byteBuffer1 = Wire.endTransmission();
    if ( byteBuffer1 != 0) {
      if(ECHO_TO_SERIAL){  //if echo is on, we are in debug mode, and errors force a halt.
      Serial.println(F("FAIL request data:si7051")); 
      error();
      }
    byteBuffer1=0;
  } 
  //delay(10);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); // some of my sensors need settling time after a register change.
  
  Wire.requestFrom(Si7051_Address, 2);
  byte msb = Wire.read();
  byte lsb = Wire.read();
  uint16_t val= msb << 8 | lsb;
  return val; //to calculate TEMP_degC =(175.26*val) / 65536 - 46.85;
}
// ============================================================================================================
// ============================================================================================================
#endif // end of code for SI7051 sensor
// ============================================================================================================
// ============================================================================================================
