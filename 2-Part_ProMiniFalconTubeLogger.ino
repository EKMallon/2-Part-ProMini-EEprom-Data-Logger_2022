// Cave Pearl Project 2-Part logger code by Edward Mallon ===========

// The build tutorial for this logger can be found at:
// https://thecavepearlproject.org/2022/03/09/powering-a-promini-logger-for-one-year-on-a-coin-cell/

/*
This program is supports of an ongoing series of  DIY 'Classroom Logger' tutorials 
from the Cave Pearl Project. The idea is to provide a starting point for self-built 
student projects in environmental monitoring courses.

This 'low power' 2-module iteration runs the logger from a CR2032 coin cell and uses 
EEprom memory to store sensor readings. This necessarily involves several power 
optimization steps which add complexity to the base code as compared to previous 
versions, but hopefully everyone can read through the code and understand 
what is happening from the comments. Data download & logger control are managed 
through the IDE's serial monitor window at 250000 baud. The logger WILL NOT START 
taking readings until those serial handshakes are completed via the UART connection.

Note that all the readings are initially buffered in opdDataBuffer[16] & sensorDataBuffer[16] arrays
so there won't be any data in the EEprom until those ram buffers get transfered to eeprom memory. 
With the 1-byte RTCtemp only default configuration that you will have to wait 16* sampleInterval before that happens. 
The default 4k eeprom on the rtc module stores 4096 of those readings so takes ~2.8 days at 1min interval before it is full

The most important rule to follow when adding new sensors is buffers can only handle byte additions in quantity of 1, 2, 4, or 8
Any odd numbers other than one and you end up with page boundary issues in the EEprom (page sizes are powers of 2)

Note: This script will also run on the 3-module "Modules & Jumper Wires" 
loggers described in the original Sensors paper: http://www.mdpi.com/1424-8220/18/2/530 
*/

#include <Wire.h> 
#include <EEPROM.h>     // note: requires default promini bootloader (ie NOT optiboot)
#include <avr/power.h>  // for peripheral shutdown to lower runtime current
#include <avr/sleep.h>
#include <LowPower.h>   // https://github.com/rocketscream/Low-Power

//==========================================
// LOGGER OPERATING PARAMETERS:  
//==========================================

// use deploymentDetails[] to record unit numbers,or anthing else relevant to a given build/run
// eg: "RTC ONLY,1123875L,1.098v,1.9uA";
// "1127315L,NTC 64k65536@0x57,siTHERMcalibration,1.101v,1.46uA,10Kref[D6]/NTC[D7]/CDS[D9]-104cap,RGBledA0-3";

const char deploymentDetails[] PROGMEM = "RTC ONLY,1126400L,1.098v,1.9uA";
#define InternalReferenceConstant 1126400L //1126400L = default = 1100mV * 1024 
// use a DVM to read the actual rail voltage & add or subtract 400 from the constant 
// to raise/lower calculated output from readBattery() by ~1 millivolt

//#define ECHO_TO_SERIAL  // DO NOT ENABLE ECHO_TO_SERIAL during battery powered logger operation or system wastes a lot of power waiting for a serial response that never arrives
                        // It prints readings to screen for USB tethered debugging & also starts the run with no sample interval sync delay

#define SampleIntervalMinutes 1   
// Allowed values: 1,2,3,5,10,15,20 or 30, - a number equally into 60!
// Make sure your sensor readings don't take longer than your sample interval
// If you pass your alarm time you will have to wait 24hours for the next sensor reading

boolean BatteryReadingEveryCycle = true;  // default: false to save LOWbattery reading only once per day at midnight (when battery is cold)
// set BatteryReadingEveryCycle=true for frequent reading of battery voltage during debug & testing
// this forces the 'Once per Day' data to be saved at EVERY SampleInterval & checks coincell battery each time opdDataBuffer is transferred to EEprom
// the time between these 'forced' battery checks = [sizeof(opdDataBuffer)/opdBytesPerRecord]*SampleInterval

//uncomment the defines to match your attached sensors:
//=====================================================
#define RTC_ONLY
//#define BMP280_ON                   // provided here as 'type example' to show steps required to add other sensors to your logger
//#define DigitalPinReadAnalogTherm   // can be combined with DigitalPinReadCDScell
//#define DigitalPinReadCDScell       // CDS cell support is 'embedded' inside readNTCthermistor() so DigitalPinReadAnalogTherm must also be enabled

#ifdef DigitalPinReadAnalogTherm
const char NTCcal[] PROGMEM = "No cal for this unit yet...";
 // usually something like this after calibration      (and yes, the RTC temp can be calibrated too!)
 // const char NTCcal[] PROGMEM = "1127315L,Cal202202:,A=,0.001246979,B=,0.000216248,C=,0.0000001537444523,R(25°C)=9834.81Ω,β=3850.83K,RTCy=0.9744x -0.4904";
#endif

//change the eeprom settings manually to match your hardware configuration:
//once-per-day readings are just lowest battery reading per day & RTC temp at midnight
#define opdEEpromI2Caddr 0x57       // default: 0x57 4096 4k EEprom on RTC module
#define opdEEbytesOfStorage 4096    // AT24c256 YL-90 (red module)32768@0x50 // OR // 64k AT24c512(via chip swap on RTC)65536@0x57
//#define opdBytesPerRecord         // Change this manually if you edit #of bytes stored in opdDataBuffer (at midnight rollover)
uint8_t opdDataBuffer[16];          // [16] is max unless you increase the I2C buffers
int8_t opdArrayPointer= sizeof(opdDataBuffer); // opd Pointer increments backwards & must be able to hold negative value without overflow
uint32_t opdEEprMemPointer = opdEEbytesOfStorage - sizeof(opdDataBuffer); //increments backwards

//for sensor readings // these can be the same as the OPD eeprom
#define sensorEEpromI2Caddr 0x57    //as the two bufferes are transfered into eeprom memory from 'opposite ends'
#define sensorEEbytesOfStorage 4096
uint8_t sensorDataBuffer[16];
uint8_t sensorArrayPointer = 0;

#ifdef RTC_ONLY
  #define sensorBytesPerRecord 1
#endif

#if defined(BMP280_ON)
  #define sensorBytesPerRecord 4
#endif

#if defined(DigitalPinReadAnalogTherm) && defined(DigitalPinReadCDScell)
  #define sensorBytesPerRecord 4
#endif

// when reading ONLY the NTC but NOT the CDS cell:
#if defined(DigitalPinReadAnalogTherm) && !defined(DigitalPinReadCDScell)
   #define sensorBytesPerRecord 2
#endif

#if defined(RTC_ONLY)
  #define opdBytesPerRecord 1         // Change this manually if you edit #of bytes stored in opdDataBuffer (at midnight rollover)
#else
  #define opdBytesPerRecord 2 
#endif

uint16_t systemShutdownVoltage = 2750; // MUST be > BOD default of 2700mv (also EEprom limit)
byte default_ADCSRA,default_ADMUX;     // stores default register settings
byte set_ADCSRA, set_ADMUX;            // custom settings for readbattery() via 1.1 internal band gap reference
volatile uint8_t adc_irq_cnt;          // used in readADCLowNoise ISR to allow averaging
uint16_t currentBatteryRead = 0;
uint16_t LowestBattery = 5764;         // 1-byte encoding range to 4080 mv above the BIAS value of 1700 (so range = 1700-5780mv)
                                       // the smallest delta resolvable with internal Aref trick is 11 millivolts
                                       // but that is further reduced to 16mV per bit by 1-byte indexed encoding

//DS3231 RTC variables ====================================================
#define DS3231_ADDRESS     0x68      //=104 dec
#define DS3231_STATUS_REG  0x0F
#define DS3231_CONTROL_REG 0x0E
#define DS3231_TMP_UP_REG  0x11
#define rtc_INT0_Pin 2   //DS3231's SQW connected to pin D2 on the arduino
char CycleTimeStamp[] = "0000/00/00,00:00"; //16 characters without seconds!
uint8_t t_second;   //= sec;  used in RTC_DS3231_getTime()
uint8_t t_minute;   //= min;
uint8_t t_hour;     //= hour;
uint8_t t_day;      //= day;
uint8_t t_month;    //= month;
uint16_t t_year;    //= year //yOff = raw year; //need to add 2000
uint8_t Alarmday = 1;
uint8_t Alarmhour = 1;
uint8_t Alarmminute = 1;
uint8_t Alarmsecond = 1; 
volatile boolean rtc_INT0_Flag = false;  // volatile because it's changed in an ISR
boolean midnightRollover = false;        // default false on start
int rtc_TEMP_Raw = 0; 
float rtc_TEMP_degC = 0.0;

union {                       // for Unix Time Index storage & retrieval
  uint32_t cyleTimeStart;     // 0-4,294,967,295 large enough for unixtime
  uint8_t EE_byteArray[4];    // Arduino: char = int8_t but byte = uint8_t
} utime;

// temporary variables for calculations =============================
byte byteBuffer1 = 0;          // for functions that return a byte
byte byteBuffer2 = 0;          // A uint8_t data type is basically the same as byte in Arduino
int integerBuffer = 9999;      // for temp-swapping ADC readings  // range of -32,768 to 32,767
float floatBuffer = 9999.9;    // for temp. float calculations

// info printed with the boilerplate IF ECHO is on ========================
#define fileNAMEonly (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
// from: https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
const char compileDate[] PROGMEM = __DATE__; 
const char compileTime[] PROGMEM = __TIME__;
 
#ifdef BMP280_ON          //================================================
#include <BMP280_DEV.h>   // from    https://github.com/MartinL1/BMP280_DEV
BMP280_DEV bmp280;        // Create a BMP280_DEV library object. called ‘bmp280’
float bmp280_temp,  bmp280_pressure,  bmp280_altitude;    // 3 float variables for output
#endif  //BMP280_ON       // ================================================

#ifdef DigitalPinReadAnalogTherm   //========================================
  #define referencePullupResistance 10000 //an ARBITRARY value (regardless of the actual 10k reference value) simply shifts the thermistor calibration!
  uint16_t resistanceof10kNTC;     // max of 65535 limits our ability to measure 10kNTC resistance below zero C
  volatile boolean triggered;
  volatile uint16_t timer1CounterValue;
#endif

#ifdef DigitalPinReadCDScell    //============================================
  uint16_t resistanceofCDScell; // 65535 limits so we test the value of the CDScell with discharge before measuring
#endif

//======================================================================================================================
//  *  *   *   *   *   *   SETUP   *   *   *   *   *
//======================================================================================================================
// NOTE: problems in setup call error() routine which halts the system!

void setup () {

  // configure ADC on 168/328 boards to read the rail voltage using the internal 1.1 aref
  default_ADCSRA = ADCSRA; default_ADMUX = ADMUX; // in case we need to restore defaults later
  bitSet(ACSR,ACD); // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  DIDR0 = 0x0F;     // Disables the digital input on analog 0..3   (analog 4/5 used for I2C!)
  analogReference(INTERNAL);analogRead(A6);// engauge aref cap @ 1.1v
  //bitWrite(ADCSRA,ADPS2,0);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,1); // 8 ADC prescalar
  //bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,0);bitWrite(ADCSRA,ADPS0,0); // 16 ADC prescalar  = 4x normal speed
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,0);bitWrite(ADCSRA,ADPS0,1);   // 32 ADC prescalar doubles the speed of the ADC
  //bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,0);   // 64 (default) prescalar @ 8MHz/64 = 125 kHz
  //bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,1);   // 128 (default) prescalar @ 16mhz
  
  set_ADCSRA = ADCSRA; // store new register values for use in readBattery() function
  // REFS1 REFS0  --> 0 1, AVcc internal ref. //MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
  set_ADMUX = ADMUX;   // store new register values for use in readBattery()

  //Promini takes ~2 sec for the bootloader so rail caps should be charged(?) but first reading can be low (?)
  currentBatteryRead = readBattery(); // to catch low battery restarts triggers shutdown if < systemShutdownVoltage

// ==============disableUnusedPeripherals ==========================
  ADCSRA = 0; power_adc_disable();
  SPCR = 0; power_spi_disable();
  power_timer1_disable();
  power_timer2_disable();
  //DON'T mess with timer0 unless you know what you are doing!

  unusedPins2InputPullup();
    
  Wire.begin();  // enables internal 30-50k pull-up resistors on SDA & SCL by default
  TWBR = 2;      // for ~400 kHz I2C bus @ 8MHz CPU
  // TWBR = 32; = default 100 kHz I2C bus @ 8MHz CPU

  Serial.begin(250000);  // Opening the serial monitor "resets" the Arduino
  // max at 8Mhz http://wormfood.net/avrbaudcalc.php

#ifdef BMP280_ON
//Sensor initialization in Setup{} - AFTER wire.begin(); starts the I2C bus 
bmp280.begin(BMP280_I2C_ALT_ADDR);    // ALT_ADDR for sensors at 0x76
bmp280.setPresOversampling(OVERSAMPLING_X2);
bmp280.setTempOversampling (OVERSAMPLING_X2);
bmp280.setSeaLevelPressure (1013.25f);    // default value for altitude calculations
bmp280.setIIRFilter(IIR_FILTER_OFF); 
#endif


  RTC_DS3231_turnOffBothAlarms();   //stops RTC from holding the D2 interrupt line low if system reset just occured 
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_STATUS_REG, 3, 0);  // disable the 32khz output  pg14-17 of datasheet  // This does not reduce the sleep current
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 6, 1); // 0Eh Bit 6 (Battery power ALARM Enable) - MUST set this to 1 for wake-up alarms when running from the coincell
  //pinMode(rtc_INT0_Pin, INPUT);   // not needed with hardware pullup on RTC module
  RTC_DS3231_getTime();
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);  
  
send_serial_boilerplate();
printMenuOptions();

String command;                     //declared here so it gets deleted at end of setup
boolean goFlagReceived = false;     //deleted at end of setup
Serial.setTimeout(15000);           //so input loop doesn't run forever
//see https://forum.arduino.cc/t/calling-functions-in-the-serial-monitor/618958/3

unsigned long startMicros = micros();
do { 
command = Serial.readStringUntil('\n'); // read string until newline character
   
    if(command == "new")
    {
          Serial.println(F("Downloading: Sensor Data:"));
          sendSensorData2Serial(true);  //false converts data into human readable form
          sendOPDreadings2Serial(true);
          startMicros = micros(); //resets the while loop for another 100 seconds
          printMenuOptions();goFlagReceived=false;
    }
    else
    if(command == "raw")   //only used for debugging
    {
          Serial.println(F("Downloading: 'Raw' bytes from EEprom:"));
          //note: does not include OPD data unless they are in the same memory space
          sendSensorData2Serial(false);
          startMicros = micros(); //resets the while loop for another 100 seconds
          printMenuOptions();goFlagReceived=false;
    }
    else
    if(command == "start")   // a second 'start' will be required to launch the run
    {goFlagReceived=true;}
    else
    if(command == "test")    // hidden option only used for rapid debugging/testing (not displayed on menu)
    {goFlagReceived=true;}

 if (goFlagReceived) break; //breaks out of the while loop
 
}while ((micros() - startMicros) < 100000); // 100 seconds to respond?
// terminates: do-while timeout loop

if (!goFlagReceived){   // if goflag=false then shut down the logger
Serial.println(F("Timeout: NO valid command received -> shutting down"));Serial.flush();
error(); //shut down the logger
}


//=========== Set RTC time (if needed )===================
Serial.print(F("Current RTC time = "));Serial.println(CycleTimeStamp);

bool DS3231_lostpower = i2c_readRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG) >> 7;
//Oscillator Stop Flag (OSF). A logic 1 in bit7 indicates that the oscillator is stopped or was stopped for some period due to power loss 
if (DS3231_lostpower){
  Serial.println(F("RTC Power-loss detected!"));
}

if (t_year<2020 || DS3231_lostpower){
  Serial.println(F("Enter current date/time with digits as indicated:"));

Serial.print(F("YYYY:"));     t_year = Serial.parseInt();      Serial.println(t_year);
Serial.print(F("MM:"));       t_month = Serial.parseInt();     Serial.println(t_month);
Serial.print(F("DD:"));       t_day = Serial.parseInt();       Serial.println(t_day);
Serial.print(F("(24hour) HH:")); t_hour = Serial.parseInt();Serial.println(t_hour);
Serial.print(F("MM:"));       t_minute = Serial.parseInt();    Serial.println(t_minute);
Serial.print(F("SS:"));       t_second = Serial.parseInt();    Serial.println(t_second);

if (t_month==0 && t_day==0){  //if no values were input
Serial.println(F("No valid input to set RTC time!"));
error(); //shut down the logger
} else {
RTC_DS3231_setTime(); delay(15); // give the eeprom a little time
RTC_DS3231_getTime();            // DS3231 Vbat = 2.3v min
i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,7,0);  //clear the OSF flag after time has been set
sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);  
Serial.print(F("RTC NOW SET to: "));Serial.println(CycleTimeStamp);
i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,7,0);//clear the OSF flag after time is set
}

} //terminator for Seting RTC time

//=========== FINAL check before erasing eeprom & starting next run ===================
Serial.println(F("----------------NOTE--------------------"));
Serial.println(F("Are you SURE? Starting a new run will ERASE ALL DATA on the logger."));
Serial.println(F("(type 'start' & enter) if you want to proceed:"));
Serial.println();

goFlagReceived=false;startMicros = micros();
do {  
    command = Serial.readStringUntil('\n'); // read string until meet newline character
    if(command == "start")
    {
        Serial.println(F("Erasing 328p internal EEprom"));Serial.flush();
        for (int i = 0; i < 1024; i++){  EEPROM.update(i,0); } 
        //EEPROM.update only 'writes' a location if it is being changed (to preserve eeprom lifespan)   
  
        Serial.println(F("Erasing OPD memory...(this may take some time for large EEproms)"));Serial.flush();
        for (int j=0; j<sizeof(opdDataBuffer); j++) {opdDataBuffer[j] = 0;} //opddatabuffer now contains zeros
        
        if(opdEEbytesOfStorage==4096){TWBR=32;}else{TWBR=2;}   // 100kHz I2C bus only needed for 4k AT24c32
        //Note: we erase the entire external eeprom here to avoid refresh problems with EEproms: http://mientki.ruhosting.nl/data_www/pic/libs_hw/eeprom_problems.html 
        
        uint32_t CurrentMemAddress=0;
          
        for (int i=0; i<(opdEEbytesOfStorage/sizeof(opdDataBuffer)); i++){  // loop in 16 byte page writes, ExtEEmemoryPointer=0; when declared
          Wire.beginTransmission(opdEEpromI2Caddr);
          Wire.write((byte)(CurrentMemAddress >> 8));   // send the MSB of the address
          Wire.write((byte)(CurrentMemAddress & 0xFF)); // send the LSB of the address
              for (int k=0; k<sizeof(opdDataBuffer); k++) {
                Wire.write((byte)opdDataBuffer[k]);
              } 
          Wire.endTransmission();//this is where the send actually happens
          CurrentMemAddress = CurrentMemAddress + sizeof(opdDataBuffer);
          if ((CurrentMemAddress % 64) == 0){Serial.print(F("."));} //progress bar
          if ((CurrentMemAddress % 2048) == 0){Serial.println();}   //return every 32 dots
          do  // poll the eeprom to see when next bytes can be written:
          { Wire.beginTransmission(opdEEpromI2Caddr); }
          while (Wire.endTransmission() != 0x00);
        }// terminates  for (int i=0; i<(opdEEbytesOfStorage/sizeof(opdDataBuffer)); i++){ 

     for (int j=0; j<sizeof(sensorDataBuffer); j++) {sensorDataBuffer[j] = 0;} //sensor databuffer now contains 0s
     if (sensorEEpromI2Caddr != opdEEpromI2Caddr) { //this erasure would be redundant if sensor & opd are stored in same eeprom 

        if(sensorEEbytesOfStorage==4096){TWBR=32;}else{TWBR=2;}   // 100kHz I2C bus only needed for 4k AT24c32 
        
        Serial.println(F("Erasing Sensor Data EEprom..."));Serial.flush();
          CurrentMemAddress=0;
          for (int i=0; i<(sensorEEbytesOfStorage/sizeof(sensorDataBuffer)); i++){  // loop in 16 byte page writes, ExtEEmemoryPointer=0; when declared
          Wire.beginTransmission(sensorEEpromI2Caddr);
          Wire.write((byte)(CurrentMemAddress >> 8));   // send the MSB of the address
          Wire.write((byte)(CurrentMemAddress & 0xFF)); // send the LSB of the address
              for (int k=0; k<sizeof(sensorDataBuffer); k++) {
                Wire.write((byte)sensorDataBuffer[k]);
              } 
          Wire.endTransmission();//this is where the send actually happens
          CurrentMemAddress = CurrentMemAddress + sizeof(sensorDataBuffer);
          if ((CurrentMemAddress % 64) == 0){Serial.print(F("."));} //progress bar
          if ((CurrentMemAddress % 2048) == 0){Serial.println();}   //return every 32 dots
          //serial print takes about 10 seconds divided by the baud rate, per character. = 1/10ms per char at 57000

          do  //we are still powered by uart: poll the eeprom to see when next bytes can be written:
          { Wire.beginTransmission(sensorEEpromI2Caddr); }
          while (Wire.endTransmission() != 0x00);
          
          } //terminates for (int i=0; i<(sensorEEbytesOfStorage/sizeof(sensorDataBuffer)); i++){
      } // terminates if (sensorEEpromI2Caddr != opdEEpromI2Caddr) 
     
          Serial.println();Serial.println(F("EEprom(s) now loaded with zero values"));
          Serial.println();Serial.flush();
          startMicros = micros();
          goFlagReceived=true;
    }
    else
    if(command == "test")    //note this option is not in the options menu
    {goFlagReceived=true;}   //it is only used for rapid debugging/testing
 
  if (goFlagReceived) break; //breaks out of the while loop
 
}while ((micros() - startMicros) < 100000); // 100 seconds to respond before timeout

if (!goFlagReceived){   // if goflag=false then shut down the logger
Serial.println(F("Timeout w NO command recieved -> logger shutting down"));Serial.flush();
      error();         //shut down the logger
}
  
Serial.println(F("Starting the Data logger...")); Serial.flush();
  
//============================================================================
//Delay start of logger until alarm is in sync with sampling interval
//============================================================================
//otherwise you get a "clipped interval" at the first hour rollover
//AND save index values (for later Unixtime reconsruct) to 328p internal 1k eeprom WHILE still tethered to UART power

  RTC_DS3231_getTime();
  uint32_t timeCalcVariable = RTC_DS3231_unixtime();
  utime.cyleTimeStart = ((timeCalcVariable/86400UL)*86400UL)+86400UL; //calculates next midnight rollover
  EEPROM.put(1020,utime.EE_byteArray); // saves this time-index value for OncePerDay readings to internal 1K eeprom
  // uses location at the END of memory to leave first few bytes for other information (later)
  
#ifdef ECHO_TO_SERIAL                     //no waiting for time sync delay if ECHO is on
  utime.cyleTimeStart = timeCalcVariable; //= RTC_DS3231_unixtime();
  EEPROM.put(1016,utime.EE_byteArray);    // save this sensor reading time-index value to intEEprom location 0 // 3msec blocking
  Serial.println(F("Sync delay disabled when ECHO_TO_SERIAL enabled"));
  Serial.println();Serial.flush();        // In debug mode you don't want to wait for a 5-30 min startup
  
#else   //sleep logger till time is sync'd with 1st sampling interval

  int syncdelay=1; //default 1minute delay if sample interval seconds being used
  Alarmday = t_day;Alarmhour = t_hour;Alarmminute = t_minute;Alarmsecond = 0;
  if(SampleIntervalMinutes!=0){
  syncdelay=Alarmminute % SampleIntervalMinutes;  // 7 % 10 = 7 because 7 / 10 < 1, e.g. 10 does not fit even once in seven. So the entire value of 7 becomes the remainder.
  syncdelay=SampleIntervalMinutes-syncdelay;      // when SampleIntervalMinutes is 1, syncdelay is 1, other cases variable
  }
  Alarmminute = Alarmminute + syncdelay;
  if (Alarmminute > 59) {                // check for roll-over
     Alarmminute = 0; Alarmhour = Alarmhour + 1; 
        if (Alarmhour > 23) { Alarmhour = 0;} // check for roll-over
  }
  RTC_DS3231_setA1Time(Alarmday,Alarmhour,Alarmminute,Alarmsecond,0b00001000,false,false,false);   //or RTC.setAlarm1Simple(Alarmhour, Alarmminute);
  RTC_DS3231_turnOnAlarm(1);

  utime.cyleTimeStart = timeCalcVariable +(60*syncdelay); //previous utime reading plus how many seconds you waited to sync
  EEPROM.put(1016,utime.EE_byteArray);  // save this 4-byte sensor read UnixTime index {via union} to intEEprom location 0

  Serial.println(F("RED d13 LED will now flash slowly until the 1st sensor read is taken."));
  Serial.flush();
  Serial.println(F("Disconnect from the UART now - NO additional messages will be sent."));
  Serial.flush();
  power_usart0_disable(); //~40uA

   noInterrupts ();         // make sure we don't get interrupted before we sleep
   bitSet(EIFR,INTF0);      // clear flag for interrupt 0 (D2) see https://gammon.com.au/interrupts
   bitSet(EIFR,INTF1);      // clear flag for interrupt 1 (D3)
   attachInterrupt(0,rtcAlarmTrigger, LOW); //RTC SQW alarms LOW connected to pin D2
   interrupts ();           // interrupts allowed now
   rtc_INT0_Flag=false;
     do{
     PORTB  = PORTB ^ 0b00100000;    // toggles D13 LED pullup //~50uA to light led through pullup resistor
     LowPower.powerDown(SLEEP_1S, ADC_ON, BOD_OFF);
     }while(!rtc_INT0_Flag);         // flag only becomes true if RTC alarm causes D2's ISR rtcAlarmTrigger to run
   
  //detachInterrupt(0); done in the interrupt
  RTC_DS3231_turnOffBothAlarms();
#endif  //#ifdef ECHO_TO_SERIAL 

LowestBattery =5764;                 //1st readbattery in setup is often low because refcap not fully charged
currentBatteryRead = readBattery();

TWBR = 2;          // I2C bus at 400khz - works on DS3231  // (1msec) Tlow while 1.3 microseconds is the official standard
bitClear(PORTB,5); // pin13 LED indicator LED PULLUP OFF
sensorArrayPointer = 0;

//====================================================================================================
}  // terminator for void setup()
//=====================================================================================================
//========================================================================================================
//========================================================================================================
//========================================================================================================
// ========================================================================================================
//      *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
//========================================================================================================
//========================================================================================================
//========================================================================================================
// NOTE: problems during main loop call error()routine that shuts down the system

void loop (){

//===========================================================================================
//this loop wraps the "cycle of samples" repeating until the eeprom memory is full:
//===========================================================================================
for (uint16_t forwardEEmemPointr = 0; forwardEEmemPointr < (sensorEEbytesOfStorage -sensorBytesPerRecord); forwardEEmemPointr += sensorBytesPerRecord) { 

// if BOTH sensor & opd buffers are saving to the same eeprom...
   if (sensorEEpromI2Caddr == opdEEpromI2Caddr) {  // AND their memory pointers overlap the eeprom is full: shutdown logger   
      if (forwardEEmemPointr > opdEEprMemPointer){break;} 
   }
  
   RTC_DS3231_getTime();
   LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // recovery after each I2C exchange to reduce CR2032 battery droop

//calculate the time for your next RTC alarm:
    Alarmday = t_day;            // Alarmday = now.day(); //with #include <RTClib.h>
    Alarmhour = t_hour;          // Alarmhour = now.hour(); //with #include <RTClib.h>
    Alarmminute = t_minute + SampleIntervalMinutes; //Alarmminute = now.minute()+SampleIntervalMinutes; //with #include <RTClib.h>
    Alarmsecond = 0;

// Check for RTC TIME ROLLOVERS: THEN SET the next RTC alarm and go back to sleep
//============================================================================

  if (Alarmminute > 59) {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
        Alarmminute = 0; Alarmhour = Alarmhour + 1;
        if (Alarmhour > 23) {
          Alarmhour = 0; midnightRollover = true;
        }
      }  //terminator for if (Alarmminute > 59) rollover catching

  RTC_DS3231_setAlarm1Simple(Alarmhour, Alarmminute);
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  
  RTC_DS3231_turnOnAlarm(1);
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);

  //RTC_DS3231_forceTempConversion(); // does it actually matter to you if temp 64 seconds old?
  //LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);

#ifdef ECHO_TO_SERIAL
    Serial.print(F("A1 Set:"));
    Serial.print(t_hour, DEC);Serial.print(F(":"));Serial.print(t_minute, DEC);Serial.print(F(":"));Serial.print(t_second, DEC);
    Serial.print(F(" wake in:"));
    Serial.print(SampleIntervalMinutes);Serial.println(F("m"));
    Serial.flush();
#endif

//Read RTC temperature //========================================================================== 
rtc_TEMP_degC = RTC_DS3231_getTemp();             //+10 shifts 1-byte encoding of RTC temp to range -12.5C to 51C
floatBuffer = (rtc_TEMP_degC + 10.0) / 0.0625;    //converts the RTC float temperature into a 'RAW' 3 digit integer compatible with 12-bit temps sensors
rtc_TEMP_Raw = (int)(floatBuffer);                //this integer takes less space in eeprom than the float

#ifdef BMP280_ON  //========================================================================== 
bmp280.startForcedConversion();                  // time needed here depends on oversampling settings
bitClear(DDRB,5);bitSet(PORTB,5);                // D13 indicator LED adds ~50uA current
LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_OFF); // uses watchdog timer
bitClear(PORTB,5);                               // D13 PULLUP OFF
bmp280.getCurrentMeasurements(bmp280_temp,bmp280_pressure,bmp280_altitude);
#endif

#ifdef DigitalPinReadAnalogTherm  //==========================================================================
readNTCthermistor(); 
#endif 

//===================================================================================================
//====================================================================================================
// This is for serial output for debugging only  - comment out ECHO_TO_SERIAL skips this
//====================================================================================================
//====================================================================================================
#ifdef ECHO_TO_SERIAL
    Serial.print(F(" (Forward) EEprom pointer: "));Serial.print(forwardEEmemPointr);  
    Serial.print(F(" , (Backward) EEprom pointer: "));Serial.println(opdEEprMemPointer); 
    sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute); // sprintf ref  http://www.perlmonks.org/?node_id=20519
    Serial.print(CycleTimeStamp);

  #ifdef BMP280_ON
    Serial.print(F(", BMP280 Temp:"));Serial.print(bmp280_temp,2);
    Serial.print(F(", BMP280 Pressure:"));Serial.println(bmp280_pressure,1);
  #endif
    Serial.print(F(", RTC TempC: "));Serial.print(rtc_TEMP_degC,2);
    Serial.print(F(", LOW Bat(mV)= "));Serial.println(LowestBattery);
    Serial.println();
    Serial.flush();
#endif  //ENDIF FOR ECHO TO SERIAL

  // heartbeat pip of the LED at the end of the senor readings // adds ~50uA current
  // provides a battery rest/buffer cap recharge before moving on to the eeprom writes
  bitClear(DDRB,5);bitSet(PORTB,5);//= pinMode(d13,INPUT_PULLUP);
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  bitClear(PORTB,5);               //pin13 indicator LED pullup Off
    
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// check if sensorDataBuffer was filled on previous cycle
//     -> if so move the old data to EEprom before before adding new readings
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

if (sensorArrayPointer == sizeof(sensorDataBuffer)) { 
  Write_i2c_eeprom_array(sensorEEpromI2Caddr,(forwardEEmemPointr - sizeof(sensorDataBuffer)),sensorDataBuffer,sizeof(sensorDataBuffer));
  sensorArrayPointer = 0; // readings buffered in sizeof(sensorDataBuffer)so need to shift forwardEEmemPointr BACK before save to EEp
   
  #ifdef ECHO_TO_SERIAL
    Serial.println(F("Sensor Data has been moved to EEprom"));Serial.println();Serial.flush();
  #endif
  
  } 

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Now Pass NEW sensor Values to sensorDataBuffer
// NOTE: bytes added per cycle MUST divide evenly into sizeof(sensorDataBuffer)
// AND: the number of bytes you save must match the #define sensorBytesPerRecord
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// in each cycle you can add 1, 2, 4 or 8 bytes to sensorDataBuffer - not 3 , not 5, not 7 etc. 
// or you mess up the powers of 2 math & eeprom page boundaries
  
#ifdef RTC_ONLY  //===========================================================
integerBuffer = round((rtc_TEMP_Raw+40)/4);  // NOTE: this 1-byte encoding is restricted to range:
if(integerBuffer>255){integerBuffer=255;}    // temps above 51.5 clipped
if(integerBuffer<1){integerBuffer=1;}        // min value of 1 to preserve EOFcheck, this sets lower cutoff to -12.25C!
sensorDataBuffer[sensorArrayPointer] = integerBuffer;
sensorArrayPointer = sensorArrayPointer+1;
#endif //RTC_ONLY

#ifdef BMP280_ON    //===========================================================
floatBuffer = bmp280_temp*100.00;
integerBuffer = (int)floatBuffer;  byteBuffer1 = lowByte(integerBuffer);
if(byteBuffer1==0){byteBuffer1=1;} //to preserve zero check for EOF later

sensorDataBuffer[sensorArrayPointer] = byteBuffer1;
  sensorArrayPointer = sensorArrayPointer+1;
sensorDataBuffer[sensorArrayPointer] = highByte(integerBuffer);
  sensorArrayPointer = sensorArrayPointer+1;
floatBuffer = bmp280_pressure*10.0;
integerBuffer = (int)floatBuffer;
sensorDataBuffer[sensorArrayPointer] = lowByte(integerBuffer);
  sensorArrayPointer = sensorArrayPointer+1;
sensorDataBuffer[sensorArrayPointer] = highByte(integerBuffer);
  sensorArrayPointer = sensorArrayPointer+1;
#endif

#ifdef DigitalPinReadAnalogTherm  //===========================================================

byteBuffer1= lowByte(resistanceof10kNTC);
if(byteBuffer1==0){byteBuffer1=1;} //to preserve zero EOF indicator in 'empty' EEprom space
sensorDataBuffer[sensorArrayPointer] = byteBuffer1;
  sensorArrayPointer = sensorArrayPointer+1;
sensorDataBuffer[sensorArrayPointer] = highByte(resistanceof10kNTC);
  sensorArrayPointer = sensorArrayPointer+1;

  #ifdef DigitalPinReadCDScell  // embedded inside NTC function
  sensorDataBuffer[sensorArrayPointer] = lowByte(resistanceofCDScell);
  sensorArrayPointer = sensorArrayPointer+1;
  sensorDataBuffer[sensorArrayPointer] = highByte(resistanceofCDScell);
  sensorArrayPointer = sensorArrayPointer+1;
  #endif

#endif

//=================== oncePerDayEvents=======================
  //in each pass you can add 1, 2, 4 or 8 bytes to opdDataBuffer - not 3 , not 5, not 7 etc or you mess up the powers-of-2 math

if(BatteryReadingEveryCycle){   // only used for rapid testing
     midnightRollover=true;     // forces new coin cell voltage reading every opd buffersave
}
// OPD data gets stored IN REVERSE ORDER to share same EEprom storing sensorDataBuffer
// NOTE: data bytes added to the buffer per cycle MUST DIVIDE EVENLY into sizeof(opdDataBuffer)
// AND: the (number) of bytes you store must match #define opdBytesPerRecord (number) at start of program

if (midnightRollover) {    //normally set true only on the midnight hour rollover but can be forced by BatteryReadingEveryCycle

  opdArrayPointer=opdArrayPointer-1;             // incrementing backwards here for stack/heap method
  integerBuffer = round((LowestBattery-1700)/16);
  if(integerBuffer<1){integerBuffer=1;}          // to preserve END of DATA check
  opdDataBuffer[opdArrayPointer] = integerBuffer;

#ifndef RTC_ONLY
  opdArrayPointer=opdArrayPointer-1;
  integerBuffer = round((rtc_TEMP_Raw+40)/4);    // this 1-byte encoding is restricted to temp. range 51.5 to -12.25 degC
  if(integerBuffer>255){integerBuffer=255;}      // temps above 51.5 clipped
  if(integerBuffer<0){integerBuffer=0;}          // sets lower cutoff to -12.25C!
  opdDataBuffer[opdArrayPointer] = integerBuffer;
#endif

if (opdArrayPointer ==0) {    // the Buffer is full - so transfer that data to the eeprom

    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
    Write_i2c_eeprom_array(opdEEpromI2Caddr,opdEEprMemPointer,opdDataBuffer,sizeof(opdDataBuffer)); 

    opdArrayPointer = sizeof(opdDataBuffer);     // resets to 'far' end because this pointer increments backward
    if (opdEEprMemPointer >= sizeof(opdDataBuffer)) {
    opdEEprMemPointer-= sizeof(opdDataBuffer);   // Pointer jumps 'in advance' to where the NEXT EEprom save can occur
    } else {
     error();  //when opdEEprMemPointer goes negative the opd eeprom is full - shutdown
    }
    //opdEEprMemPointer gets checked against forwardEEmemPointr to flag when eeprom is full -IF- both are stored in the same eeprom
    #ifdef ECHO_TO_SERIAL
    Serial.println(F("OncePerDay data moved to EEprom"));Serial.println();Serial.flush();
    #endif
  }         //terminates if (opdArrayPointer <= 0)  
  midnightRollover=false;
}

sleepNwait4RTC(); // sleep till the next alarm

//================================================================================================
} //TERMINATOR for "cycle of samples" LOOP 
//================================================================================================

// if you've reached this point then the external eeprom memory storing sensor readings is now full!
   error(); //so logger goes into shutdown down

//==========================================================================================
//==========================================================================================
}  //============= END OF MAIN LOOP=========================================================
//==========================================================================================
//==========================================END OF MAIN LOOP================================
//==========================================================================================
//==========================================================================================


//======================================================
void sleepNwait4RTC() { 
//======================================================  

  unusedPins2InputPullup(); //pin states preserved through sleep

  noInterrupts();
  bitSet(EIFR,INTF0);                         // use before attachInterrupt(1,isr,xxxx) to clear interrupt 1 flag
  attachInterrupt(0,rtcAlarmTrigger,FALLING); //RTC alarm connected to pin D2
  interrupts ();
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF); // ADC_ON preserves current ADC status(it's already OFF...)
  //HERE AFTER WAKING  // powerDown & powerSave add 16,000 clock cycles (~2msec) to wakeup time at ~250uA
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_STATUS_REG);
  Wire.write(0);                // turn Off (both) alarms
  Wire.endTransmission();

  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  rtc_INT0_Flag = false; //clear the flag we use to indicate the RTC alarm occurred
     
  #ifdef DigitalPinReadAnalogTherm  // ground & drain 104(100nF) sampling cap through D8s 300Ω // 100nF) =>5RC is 0.15ms
  // moved from readNTCthermistor() to provide discharge time while processor RTC readings
  PORTB &=B11111110; DDRB |=B00000001; PORTB &=B11111110; //LOW-OUTPUT-LOW
  #endif
    
}  //terminator for sleepNwait4RTC

void rtcAlarmTrigger() { 
  detachInterrupt(0);
  rtc_INT0_Flag = true; //this flag used in SETUP
}

//============================================================================
void unusedPins2InputPullup(){
//============================================================================

//unused pins to input-pullup     // &= causes zeros to set
  DDRB &= B11000000; // pins 13..8 set to zero for input
  DDRD &= B00000011; // pins 7..2 set as inputs
  DDRC &= B11110000; // unused A3..A0 set as inputs
 
//enable pullups during runtime   // with |= only the 1' set
  PORTB |= B00011111; // 12..9 pullups on, NOT for Xtal, Xtal or D13 LED 
  PORTD |= B11111100; // 7..2 pullups on, avoids D1&D0 for serial coms
  PORTC |= B00001111; // pins A3..A2 // leave these low if you add an RGB led to A0-A3

#ifdef DigitalPinReadAnalogTherm
  PORTB &= B11111100; // 9-8 pullups OFF // if using ICU on D8 to read NTC/CDS sensors
  PORTD &= B00111111; // 7-6 pullups OFF // otherwise we charge the sampling capacitor
  #endif

#ifndef ECHO_TO_SERIAL   // don't mess with USART pins in ECHO_TO_SERIAL debug mode
  DDRD &= B11111100;     // d1&0 set as inputs
  PORTD |= B00000011 ;   // d1&0 pullup turned on
  #endif
}


//============================================================================
void sendSensorData2Serial(boolean convertDataFlag){ //only runs at startup with serial connection
//============================================================================

Serial.print(F("UnixTime,"));

#ifdef RTC_ONLY
  Serial.println(F("RTCtemp"));
#endif
#ifdef DigitalPinReadAnalogTherm
  Serial.print(F("NTC,"));
#endif
#ifdef DigitalPinReadCDScell
  Serial.println(F("CDScell"));
#endif
#ifdef BMP280_ON
  Serial.println(F("BMP T°C,Pr."));
#endif

    //unixtime index value stored in penultimate four bytes of internal eeprom
    EEPROM.get(1016,utime.EE_byteArray);           //note bytearray unioned w utime.cyleTimeStart
    uint32_t unix_timeStamp = utime.cyleTimeStart; // ULong to 4,294,967,295
    int32_t RecordCounter = 0;
    
for (uint16_t i = 0; i < (sensorEEbytesOfStorage-sensorBytesPerRecord); i+=sensorBytesPerRecord) { //increment by # of bytes PER RECORD in eeprom
          
      byteBuffer1 = i2c_eeprom_read_byte(sensorEEpromI2Caddr,i); //zeros first byte read are EOF / end of data markers
      if(byteBuffer1==0 & convertDataFlag){break;} //convertDataFlag sends every memory location even if zeros

if (convertDataFlag){  //UnixTime is 'reconstructed' from start time & record number 
       unix_timeStamp = utime.cyleTimeStart + (uint32_t)(60UL*RecordCounter*SampleIntervalMinutes); //used if you are using more than one byte per record        }
       Serial.print(unix_timeStamp);Serial.print(",");
     }

#ifdef RTC_ONLY                           //RTC temperature record: low side cutoff at 1 for minimum reading of -12.25C
   integerBuffer=byteBuffer1;             // read previously for EOF check
   if(convertDataFlag){                   // restore our 'raw' RTCtemp data format from single bytes stored in EEprom to tempC
     integerBuffer=(integerBuffer*4)-40; 
     floatBuffer  =(integerBuffer*0.0625)-10.0;
     Serial.print(floatBuffer,2);
     }else{                               // we are printing RAW data as store in eeprom
     Serial.print(integerBuffer);
     }     //if(convertDataFlag)
#endif  //#ifdef RTC_ONLY 

#ifdef DigitalPinReadAnalogTherm          //resistanceof10kNTC is only stored as raw two data bytes

   if(convertDataFlag){ 
      //byteBuffer1 = lowbyte was read previously for end of data check
      resistanceof10kNTC = i2c_eeprom_read_byte(sensorEEpromI2Caddr,(i+1));
      resistanceof10kNTC = (resistanceof10kNTC << 8) | byteBuffer1; //reconstitutes the rawNTC reading
      Serial.print(resistanceof10kNTC);Serial.print(F(","));

        #ifdef DigitalPinReadCDScell            //embedded inside readNTC function
          byteBuffer1 = i2c_eeprom_read_byte(sensorEEpromI2Caddr,(i+2));         //low byte
          resistanceofCDScell = i2c_eeprom_read_byte(sensorEEpromI2Caddr,(i+3)); //hi byte
          resistanceofCDScell = (resistanceofCDScell << 8) | byteBuffer1;
          Serial.print(resistanceofCDScell);    
        #endif
    
    }else{ //if !convertDataFlag  // must match the number of bytes exactly
      
    Serial.print(byteBuffer1);Serial.print(F(","));
    Serial.print(i2c_eeprom_read_byte(sensorEEpromI2Caddr,(i+1)));Serial.print(F(","));
        #ifdef DigitalPinReadCDScell
         Serial.print(i2c_eeprom_read_byte(sensorEEpromI2Caddr,(i+2)));Serial.print(F(","));
         Serial.print(i2c_eeprom_read_byte(sensorEEpromI2Caddr,(i+3)));Serial.print(F(","));
        #endif
        
    } //if(convertDataFlag)
    
#endif // #ifdef DigitalPinReadAnalogTherm

#ifdef BMP280_ON
      // byteBuffer1 read previously for EOF check
      integerBuffer = i2c_eeprom_read_byte(sensorEEpromI2Caddr,i+1);
      integerBuffer = integerBuffer <<8 | byteBuffer1;
      Serial.print((float)(integerBuffer)/100.0,2);Serial.print(",");
      byteBuffer1=i2c_eeprom_read_byte(sensorEEpromI2Caddr,i+2);
      integerBuffer=i2c_eeprom_read_byte(sensorEEpromI2Caddr,i+3);
      integerBuffer = integerBuffer <<8 | byteBuffer1;
      Serial.print((float)(integerBuffer)/10.0,1);
#endif
      
       Serial.println();Serial.flush();
       RecordCounter++;    //this counter us used for Unixtime reconstruction
   }                       // terminator: for (int i = 16; i < sensorEEbytesOfStorage; i++)
   
}  // terminator: void sendSensorData2Serial()

//=================================================================================
void sendOPDreadings2Serial(boolean convertDataFlag) {     // data saved when midnight rollover flags true
//=================================================================================
      
   uint32_t unix_timeStamp;    // to 4,294,967,295 
   uint32_t RecordCounter = 0; // uint16_t good to 65535 - only need uint32_t for eeproms larger than 64k
   Serial.println(F("UnixTime,LOWbat,RTCtemp")); //change this to suit the data save in oncePerDayEvents()
   
   #ifdef BatteryReadingEveryCycle  //using the same TimeStamp index as the sensor records
   EEPROM.get(1016,utime.EE_byteArray);
   #else
   //the normal onceperday 'midnight' time index value stored in 2nd to last four bytes of the int.EEprom
   EEPROM.get(1020,utime.EE_byteArray); // filling utime.EE_byteArray populates utime.cyleTimeStart via union
   #endif

   for (uint16_t i = (opdEEbytesOfStorage-1) ; i >= opdBytesPerRecord; i-=opdBytesPerRecord) { //this loop increments backwards 

       integerBuffer = i2c_eeprom_read_byte(opdEEpromI2Caddr,i); // byte will not be zero with battery reads above 1700mv
       if(integerBuffer==0 & convertDataFlag){break;}   //stops sending if zero is found

     #ifdef BatteryReadingEveryCycle  //then use the same TimeStamp calc as the sensor records
     
     if (convertDataFlag){                 //UnixTime is 'reconstructed' from start time & record number 
       unix_timeStamp = utime.cyleTimeStart + (uint32_t)(60UL*RecordCounter*SampleIntervalMinutes); //used if you are using more than one byte per record        }
       Serial.print(unix_timeStamp);Serial.print(",");
     }

     #else       //86400 seconds is time increment for once per day events
     unix_timeStamp = utime.cyleTimeStart + (86400UL * RecordCounter);  
     Serial.print(unix_timeStamp);Serial.print(","); //UL forces long calc
     
     #endif 
     
     if(convertDataFlag){  //then restore LOWEST Battery value from one byte encoded in eeprom
       integerBuffer =(integerBuffer*16)+1700; //4096mv range at 16mv resolution
       }
     Serial.print(integerBuffer);

#ifndef RTC_ONLY      //no need for RTC temp in opd record for RTC only config

    integerBuffer=i2c_eeprom_read_byte(opdEEpromI2Caddr,i-1);
         
   if(convertDataFlag){  // convert  to human readable temp
     // restore our 'raw' RTCtemp data format from single bytes stored in EEprom to tempC
     // NOTE: this 1-byte encoding method FAILS above 51.5 degrees and below -12.25C degrees celcius
     integerBuffer=(integerBuffer*4)-40; floatBuffer =(integerBuffer*0.0625)-10.0;
     Serial.print(",");Serial.print(floatBuffer,2);
     }else{ // we are printing RAW data as store in eeprom
     Serial.print(",");Serial.print(integerBuffer); //the raw RTC temp data from the eeprom
     } 
#endif
     
     RecordCounter++;Serial.println();Serial.flush();
  }        // terminator: for (uint32_t i = (opdEEbytesOfStorage-1) ; i >= 0; i=i-2)
}          // terminator: sendOPDreadings2Serial()

//=================================================
//----------Voltage monitoring functions ----------
//=================================================
// see http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// also https://code.google.com/p/tinkerit/wiki/SecretVoltmeter

uint16_t readBattery() 
{
  power_adc_enable(); 
  ADCSRA = set_ADCSRA; // start ADC clock
  ADMUX = set_ADMUX;   // to 1.1vref & internal channel
  // throw-away readings to engage Mux & let reference capacitor stabilize
  readADCLowNoise(true);LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);readADCLowNoise(true);
  
  int16_t value = readADCLowNoise(true);
  ADCSRA =0; power_adc_disable();
  
  uint16_t result = InternalReferenceConstant / value;  //scale Rail voltage in mV
  if (result < LowestBattery) {LowestBattery = result;}
  
  if (result < systemShutdownVoltage){
      #ifdef ECHO_TO_SERIAL
      Serial.print(result); Serial.println(F("Battery < shutdown voltage!")); Serial.flush();
      #endif
     error();
    } 
    
  return result; 
}  // terminator for getRailVoltage()

//======================================================================================
// ADMUX Channel must be set BEFORE calling this function!
// from: Sleep during reading at https://www.gammon.com.au/adc
//======================================================================================
uint16_t readADCLowNoise(bool average) { 
  uint8_t low, high; uint16_t sum =0;
  adc_irq_cnt = 0;      // counts ADC samples gathered
  bitSet(ADCSRA,ADIE);  // generate interrupt when the ADC conversion is done
  
  do{
    set_sleep_mode( SLEEP_MODE_ADC );     // Enable Noise Reduction Sleep Mode
    sleep_enable();

      do{
        interrupts(); sleep_cpu(); noInterrupts(); 
        }while (bit_is_set(ADCSRA,ADSC)); // the ADC clears the bit when conversion is complete 
        // Checking conversion status has to be done with interrupts disabled to avoid a race condition
      
      sleep_disable();            // No more sleeping
      interrupts();
      low  = ADCL;high = ADCH;    // read low first
      sum += ((high << 8) | low);
    }while (adc_irq_cnt<4);       // Hard coded to read 4 samples
  //}while (adc_irq_cnt<2);       // 2 samples
  //}while (adc_irq_cnt<8);       // 8 samples
      
  bitClear(ADCSRA,ADIE);          // turn off ADC interrupts
  return ( average ? sum >> 2 : sum );   // divide averageby 4
  //return ( average ? sum >> 1 : sum ); // divide by 2
  //return ( average ? sum >> 3 : sum ); // divide by 8
}

// ADC complete ISR
ISR (ADC_vect)
  { adc_irq_cnt++;} 

//================================
void send_serial_boilerplate()
//================================
{
  Serial.print(F("CodeBuild: ")); Serial.println(fileNAMEonly); //filename only
  //Serial.println((__FlashStringHelper*)codebuild); //for the entire path + filename
  Serial.print(F("Compiled:  "));Serial.print((__FlashStringHelper*)compileDate);Serial.print(F(" at: ")); Serial.println((__FlashStringHelper*)compileTime);
  Serial.print(F("Details:   ")); 
  Serial.println((__FlashStringHelper*)deploymentDetails);
  #ifdef DigitalPinReadAnalogTherm
  Serial.println((__FlashStringHelper*)NTCcal);
  #endif
  Serial.println(F("Select “newline” OPTION in Serial Monitor Window:"));
  Serial.flush();
}

//================================
void printMenuOptions(){
//================================
Serial.println();
Serial.println(F(" Select action -> please type:"));
Serial.println(F("  ['new' & enter] to Download sensor data from logger to Serial"));
Serial.println(F("  ['raw' & enter] to Download ALL eeprom memory as RAW byte values"));
Serial.println(F("  ['start' & enter] to ERASE the EEprom(s) & start a new logging run"));
Serial.println();Serial.flush();//clears all serial coms
}

// ==========================================================================================
// I2C SENSOR MEMORY REGISTER FUNCTIONS    // these FUNCTIONS assume <255 register addresses
//===========================================================================================
// see: https://thecavepearlproject.org/2017/11/03/configuring-i2c-sensors-with-arduino/

byte i2c_readRegisterByte(uint8_t deviceAddress, uint8_t registerAddress)
{
  byte registerData;
  Wire.beginTransmission(deviceAddress); //set destination target
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);
  registerData = Wire.read();
  return registerData;
}

byte i2c_writeRegisterByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t newRegisterByte)
{
  byte result;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(newRegisterByte);
  result = Wire.endTransmission();

  if (result > 0)   //error checking
  {
#ifdef ECHO_TO_SERIAL   //NOTE: only call halt on error if in debug mode!
    Serial.print(F("FAIL in I2C register write! Result code: "));
    Serial.println(result); Serial.flush();
    error();
#endif
  }
  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);  // some sensors need this settling time after a register change
  return result;
}

byte i2c_setRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, bool state) {
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

bool i2c_getRegisterBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition) {
  byte registerByte;
  registerByte = i2c_readRegisterByte(deviceAddress, registerAddress);
  return ((registerByte >> bitPosition) & 0b00000001);
}

//========================================================================================
// READ & WRITE data to the I2C EEprom
//========================================================================================
// The Arduino Wire library DEFAULT only has a 32-BYTE buffer which limits data payload to 30 bytes MAX

byte i2c_eeprom_read_byte( uint8_t deviceaddress, uint16_t eeaddress ) {
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((byte)(eeaddress >> 8));    // Address High Byte
  Wire.write((byte)(eeaddress & 0xFF));  // Address Low Byte
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

// NOTE: this function uses the EEprom current to load the coin cell for an accurate voltage reading
// midnightRollover gets 'forced' every opd EEprom save by BatteryReadingEveryCycle

void Write_i2c_eeprom_array( uint8_t deviceAddress, uint16_t registerAddress_16bit,byte *arrayPointer,uint8_t numOfBytes){ 

int OLDadcReading;

// lowering I2C bus speed for slower eeproms but test your eeproms! - on most units the 4K works fine at 400KHz
if (midnightRollover){   
  if(opdEEbytesOfStorage==4096){TWBR=32;}else{TWBR=2;}
} else {
  if(sensorEEbytesOfStorage==4096){TWBR=32;}else{TWBR=2;} 
}

    // 4k AT24c32 writes for 10ms @3mA(max), but newer eeproms can take only only 5ms @3mA
    Wire.beginTransmission(deviceAddress);
    Wire.write((byte)((registerAddress_16bit) >> 8));   // send the MSB of the address
    Wire.write((byte)((registerAddress_16bit) & 0xFF)); // send the LSB of the address
       for (int i=0; i<numOfBytes; i++) {
          Wire.write((byte)arrayPointer[i]);
       }      
    Wire.endTransmission();

// WARNING: Be careful about what you do immediately AFTER changing the CPU speed with CLKPR!
// some peripherals need a couple of clock cycles before they can be addressed after CLKPR or THEY WILL HANG

if (midnightRollover){
    noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_8;interrupts();// slow processor to 1MHZ for low current       
    power_adc_enable();
    ADCSRA = set_ADCSRA; ADMUX = set_ADMUX;// ADC needs time ~1ms(?) for 1.1v on AREF cap to stabilize
    bitSet(ADCSRA,ADSC);                   // throw away reading to engage AREF capacitor
    
//==Timer2 SLEEP_MODE_IDLE delay for 1.56 millseconds so voltage on AREF cap stabilizes ===============
    integerBuffer = 0;
    power_timer0_disable();             // at 1mHZ the shortest T0 overflow time is 8msec so not useful here
    power_timer2_enable();              // Timer2 ovrflows lets us create SHORT sleeps at 1MHz
    set_sleep_mode (SLEEP_MODE_IDLE);   // IDLE current typically ~1mA for 8Mhz Pro Mini // IDLE wakes takes 6 clock cycles
    noInterrupts(); sleep_enable();     // Timer generates an overflow interrupt which wakes the CPU
    TCCR2A = 0x00;                      // Wave Form Generation Mode 0: Normal Mode, OC2A disconnected
    TCCR2B = 0x00; bitSet(TCCR2B,CS20); // set prescaler to 1 // or could use: bitClear(TCCR2B,CS22);bitClear(TCCR2B,CS21);bitSet(TCCR2B,CS20);
    bitSet(TIMSK2,TOIE2);               // enables Timer/Counter2 Overflow interrupt
    TCNT2=0;                            // reset T2 counter to zero
    interrupts();  
        do{
        sleep_cpu(); integerBuffer++;   // ADD 6 clock ticks to wake from IDLE //~ 0.261 msec per T2 overflow at 1mhz 
        }while (integerBuffer<7);       // 100 overflows =~2msec at 8Mhz BUT only ~8 overflows at 1MHZ
    power_timer2_disable();
//========================================end of Timer2 sleep-delay for ~1.56msec======================

//============ now we read the CR2032 battery voltage  ================================================
    bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC)); // 0.422 msec per ADC reading at PS32 & 1MHZ
    byteBuffer1=ADCL; byteBuffer2=ADCH; 
    integerBuffer =((byteBuffer2 << 8)|byteBuffer1);     // initial reading
      
    set_sleep_mode(SLEEP_MODE_ADC);   // Noise Reduction Sleep Mode, typically ~1.1mA current @1mhz
    bitSet(ADCSRA,ADIE);              // generate interrupt when the ADC conversion is done
                                      // sleep_enable(); // we left it enabled...
      do{  
           OLDadcReading = integerBuffer;              // to catch the lowest battery voltage we keep reading until voltage starts to rise again
           bitSet(ADCSRA,ADSC);                        // start a new ADC reading
             do{
             interrupts();sleep_cpu();noInterrupts();  // 13+6 clocks for ADC reading take ~0.8millseconds [@1mhz with PS128]
             }while(bit_is_set(ADCSRA,ADSC));          // the ADC clears ADSC flag when conversion complete 
           byteBuffer1=ADCL; byteBuffer2=ADCH;         // low byte first
           integerBuffer = ((byteBuffer2 << 8) | byteBuffer1);       
      }while (integerBuffer<OLDadcReading);
         
} // if (midnightRollover)

  noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); // ALWAYS return to 8MHz before sleep
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);                // battery recovery time from EEprom load
                                                                   // sleep_disable(); taken care of in LowPower.powerDown
if (midnightRollover){
   bitClear(ADCSRA,ADIE); ADCSRA=0; power_adc_disable();       // turn off ADC & interupt generation
   LowestBattery = InternalReferenceConstant / OLDadcReading;  // LONG calc forced by ref constant = 630 cpu clocks =do this after 8MHz
   power_timer0_enable();  // T0 needed for I2C bus comms
} // if (midnightRollover)

   TWBR=2; //back to 400KHz I2C bus for RTC
} // terminates Write_i2c_eeprom_array

// TIMER2 interrupt service routine
ISR(TIMER2_OVF_vect){           //must be here or you hang the system at the first T2 overflow...
}

//================================
#ifdef DigitalPinReadAnalogTherm
//=================================
// Based on Nick Gammon's frequency counter at https://www.gammon.com.au/forum/?id=11504
// Adapted by Edward Mallon for ratiometric reading of resistive sensors with ICU on D8. For a details see: 
// https://thecavepearlproject.org/2019/03/25/using-arduinos-input-capture-unit-for-high-resolution-sensor-readings/

//=========================================================================================
void readNTCthermistor() {          //uses all global variables
//=========================================================================================
// NOTE: referencePullupResistance is arbitrarily #defined as 10000 ohms in calibration & calculations
// NOTE: all unused peripherals turned off already - only timer0, I2C, and possibly usart0 left on by default
// so power_all_disable is redundant here: but keeping it here so function is atomic - PRR only applies in non-sleep & idle modes. 
// this function should be run @ the fastest clock speed your system supports

noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //this function must run a max clock speed

uint32_t elapsedTimeReff=0; 
uint32_t elapsedTimeSensor=0;

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // must come after disabling ADC
  power_timer1_enable();                  // need Timer1 to clock the change on the D8 pin

  #ifdef DigitalPinReadCDScell
  bitClear(DDRB,1);bitClear(PORTB,1);     // CDS connection floats so it does not affect NTC reading
  #endif
  DDRD &= B00111111;PORTD &= B00111111;   //same as pinMode(D7,INPUT); &  pinMode(D6,INPUT); 

// initial charging of the cap through the small 300ohm resistor on D8
// this cycles the capacitor to bring it to a 'standardised residual level'
set_sleep_mode (SLEEP_MODE_IDLE);// this mode leaves Timer1 running @ <1mA draw
prepareForInterrupts(); // sets triggered=false
noInterrupts ();sleep_enable();
bitSet(DDRB,0);bitSet(PORTB,0);            //pinMode(8,OUTPUT);//digitalWrite(8,HIGH);

do{
  interrupts(); sleep_cpu(); noInterrupts(); 
  }while(!triggered); // nothing is recorded during this initial capacitor charge up (takes about 1msec through 36k pullup)

interrupts ();
//sleep_disable(); // not needed as ISR (TIMER1_CAPT_vect) ends with sleep_disable();

// sampling cap is now at its HIGH trigger point -> now discharge the cap through 300Ω on D8
// NOTE: 15MS is overkill:  5T with 300ohm&105(1uF) is 1.5 msec, with 300Ω&104(100nF) 5RC is only 0.15ms
bitClear(PORTB,0);                               // digitalWrite(8,LOW); 
bitClear(DDRB,5);bitSet(PORTB,5);                // pinMode(d13,INPUT_PULLUP);  // pips the LED  //optional
LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // + extra 2msec for osc start! //ADC_ON leaves the already sleeping ACD alone as is
bitClear(PORTB,5);                                                              // D13 indicator LED pullup Off
bitClear(DDRB,0);                                // pinMode(8,INPUT);  // ICU ready to listen

//========== read 10k reference resistor RISING on D6 ===========
set_sleep_mode(SLEEP_MODE_IDLE);
prepareForInterrupts();
noInterrupts(); sleep_enable();
TCNT1 = 0;                    // reset Timer 1 counter value to 0
bitSet(DDRD,6);               // D6 OUTPUT
bitSet(PORTD,6);              // D6 HIGH -> now charging the cap through 10k ref
  do{
  interrupts(); sleep_cpu(); noInterrupts();
  }while(!triggered);         // trapped here till TIMER1_CAPT_vect changes triggered to true

elapsedTimeReff = timer1CounterValue;
interrupts ();                // cant use I2C bus or lowpower without interrupts...
bitClear(DDRD,6);             // D6 to INPUT ->stops the capacitor discharge
bitClear(PORTD,6);            // D6 to LOW -> stops charging through 10k reference

//===========================================================
// discharge the capacitor through 300Ω on D8
bitSet(DDRB,0);bitClear(PORTB,0); //D8 OUTPUT LOW
LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
bitClear(DDRB,0);                 //D8 INPUT 

//=== read the NTC thermistor RISING on D7 ===========
set_sleep_mode (SLEEP_MODE_IDLE); 
prepareForInterrupts(); 
noInterrupts();sleep_enable();
TCNT1 = 0;                     // reset Timer 1
bitSet(DDRD,7);                // Pin D7 OUTPUT
bitSet(PORTD,7);               // Pin D7 HIGH -> now charging the cap through NTC thermistor
  do{
  interrupts(); sleep_cpu(); noInterrupts();
  }while(!triggered);

elapsedTimeSensor= timer1CounterValue;
interrupts ();                 //cant use I2C bus or lowpower without interrupts...
bitClear(DDRD,7);              //D7 to INPUT
bitClear(PORTD,7);             //D7 to LOW

resistanceof10kNTC=(elapsedTimeSensor * referencePullupResistance) / elapsedTimeReff;
//Note: With 105 capacitor must cast to higher bits
//resistanceof10kNTC=((uint64_t)elapsedTimeSensor * (uint64_t)referencePullupResistance) / elapsedTimeReff;

#ifdef ECHO_TO_SERIAL
integerBuffer = elapsedTimeSensor;      //preserve variable for sending to serial later
#endif

#ifdef DigitalPinReadCDScell  //======= if enabled by define at start of program =============

// discharge the capacitor through 300Ω on D8
bitSet(DDRB,0);bitClear(PORTB,0); //D8 OUTPUT LOW
LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
bitClear(DDRB,0);                 //D8 INPUT 

//=== read the CDS cell RISING on D8 with power through D9 ===========
  set_sleep_mode (SLEEP_MODE_IDLE);
  prepareForInterrupts();
  noInterrupts(); sleep_enable();
  TCNT1 = 0;
  bitSet(DDRB,1);            //D9 OUTPUT 
  bitSet(PORTB,1);           //D9 HIGH
     do{
     interrupts(); sleep_cpu(); noInterrupts();
     }while(!triggered);
   
  elapsedTimeSensor= timer1CounterValue;
  interrupts(); 
  sleep_disable();          // redundant here...
  bitClear(DDRB,1);         // D9 INPUT
  bitClear(PORTB,1);        // D9 LOW // stops capacitor charging
  
  resistanceofCDScell =(elapsedTimeSensor * referencePullupResistance) / elapsedTimeReff;

#endif  //#ifdef DigitalPinReadCDScell ==============================

//now simply leave D8 LOW to dishcarge the capacitor
bitSet(DDRB,0);             // D8 OUTPUT & LOW 
bitClear(PORTB,0);          // pin will go to input later in sleepNwait

//cleanup after Dpin readings ============================
  power_timer1_disable();    // no longer need Timer1
  power_timer0_enable();     //MUST RE-ENABLE for any delay statements,etc
  power_twi_enable();        //also cant use I2C bus without interrupts enabled

#ifdef ECHO_TO_SERIAL
   power_usart0_enable();   
   Serial.print(F("10kNTC ohms: "));Serial.print(resistanceof10kNTC);
   Serial.print(F("   Raw Ref:"));Serial.print(elapsedTimeReff);Serial.print(F(", "));
   Serial.print(F("Raw Sensor:"));Serial.println(integerBuffer);
      #ifdef DigitalPinReadCDScell 
      Serial.print(F("CDScell ohms: "));Serial.print(resistanceofCDScell);
      Serial.print(F("   Raw Ref:"));Serial.print(elapsedTimeReff);Serial.print(F(", "));
      Serial.print(F("Raw Sensor:"));Serial.println(elapsedTimeSensor);
      #endif
   Serial.flush(); 
#endif
}  // terminates void readNTCthermistor()

// based on Nick Gammon's frequency counter at https://www.gammon.com.au/forum/?id=11504
void prepareForInterrupts() {
    noInterrupts ();       // protected code
    triggered = false;     // re-arm for do-while loop
    TCCR1A = 0;            // set entire TCCR1A register to 0
    TCCR1B = 0;            // same for TCCR1B
    TCNT1 = 0;             // initialize counter value to 0
    bitSet(TIMSK1,TOIE1);  // interrupt on Timer 1 overflow
    bitSet(TIMSK1,ICIE1);  // enable input capture
    bitSet(TCCR1B,CS10);   // No prescaling on Timer1 // set prescaler to 1x system clock (F_CPU)
    bitSet(TCCR1B,ICNC1);  // Activates Input Capture Noise Canceler=filter function requires four successive equal valued samples of the ICP1 pin for changing its output. 
    bitSet(TCCR1B,ICES1);  // ICES1: = 1 for trigger on RISING edge on pin D8
    TIFR1 = bit (ICF1) | bit (TOV1);  // clear flags so we don't get a bogus interrupt
    interrupts ();
}  // end of prepareForInterrupts()

// a 10k resistor only with 104 capacitor counts ~6000 ticks @8MHz - so should never reach T1 overflow in normal temp ranges
ISR (TIMER1_CAPT_vect){  
    if (triggered){return;}        // error catch for overflow or multiple triggers(?)
    timer1CounterValue = ICR1;     // transfer value held in Input Capture register
    triggered = true;
    bitClear(TIMSK1,ICIE1);        // disable input capture unit
    bitClear(TIMSK1,TOIE1);        // also disable interrupts on Timer 1 overflow
    sleep_disable();
}  // end of TIMER1_CAPT_vect

// NOTE when powering from a coincell you don't want the sensor readings to take too long
// so here we throttle the system here to only ONE overflow as an error catch with v.large resistors (like CDS cells in the dark)
ISR (TIMER1_OVF_vect) {          // timer1 overflows (every 65536 system clock ticks = 122 times a second @8MHz)
    triggered = true;
    timer1CounterValue = 65534;  // sometimes I modify these slightly while debugging...
    sleep_disable();
}

//================================================
#endif   //end of #ifdef DigitalPinReadAnalogTherm
//================================================


// =================================================================================
// RTC functions
// =================================================================================
// unixtime functions modified from https://github.com/MrAlvin/RTClib 
// which is a fork JeeLab's fantastic real time clock library for Arduino
// Extracted those functions here because Alvins version has the SAME RTClib name as 
// ADAFRUITS fork & this was causing library manager update issues & confusing students
// =================================================================================

void RTC_DS3231_turnOffBothAlarms() {     // from http://forum.arduino.cc/index.php?topic=109062.0
  byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG);
  byteBuffer1 &= B11111100; //with &= the 0's will set // with |= only the 1's set
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG,byteBuffer1);
  rtc_INT0_Flag = false; //clear the flag we use to indicate the RTC alarm occurred
}

// Unfortunately a user-initiated temperature conversion does NOT change the internal 64-second update cycle
void RTC_DS3231_forceTempConversion() {
    byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG); //Bit 2: Busy (BSY).
    if (!(byteBuffer1 & B00000100)) {  // all non-zero results interpreted as 'true' by if statement
     byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG); //byteBuffer1 = byteBuffer1 | B00100000;  
     bitSet(byteBuffer1,5);            //Control Register (0Eh) Bit 5: (CONV). Setting this bit to 1 forces the TCXO algorithm
     i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG,byteBuffer1);
     }

    do{   // this can take up to 200 milliseconds!  so sleep until the conversion is complete
      LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_OFF); //ADC_ON leaves ADC settings as they already were
      byteBuffer1=i2c_readRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG);
    }while(byteBuffer1 & B00000100); //Status Reg(0Fh) Bit2: Busy (BSY) = 1 when temperature sensor conversion is happening
}

float RTC_DS3231_getTemp()          // from http://forum.arduino.cc/index.php?topic=22301.0
{
  byte tMSB, tLSB;
  float temp3231;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_TMP_UP_REG); //temp registers (upper byte 0x11 & lower 0x12) get updated automatically every 64s
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDRESS, 2);
  if (Wire.available()) {
    tMSB = Wire.read(); //2's complement int portion - If tMSB bit7 is a 1 then the temperature is negative
    tLSB = Wire.read(); //fraction portion
    temp3231 = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0; // Allows for readings below freezing - Thanks to Coding Badly
    //temp3231 = (temp3231 * 1.8) + 32.0; // to Convert Celcius to Fahrenheit
  }
  else {
    temp3231 = 0.0;  //got no data
  }
  return temp3231;
}

static uint8_t bcd2bin (uint8_t val) {
  return val - 6 * (val >> 4);
}
static uint8_t bin2bcd (uint8_t val) {
  return val + 6 * (val / 10);
}

void RTC_DS3231_getTime()
{
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

void RTC_DS3231_setTime()
  {
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
  RTC_DS3231_setA1Time(0, hour, minute, 00, 0b00001000, false, false, false);
}

void RTC_DS3231_setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM) {
  //  Sets the alarm-1 date and time on the DS3231, using A1* information
  byte temp_buffer;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0x07);    // A1 starts at 07h
  // Send A1 second and A1M1
  Wire.write(bin2bcd(A1Second) | ((AlarmBits & 0b00000001) << 7));
  // Send A1 Minute and A1M2
  Wire.write(bin2bcd(A1Minute) | ((AlarmBits & 0b00000010) << 6));
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
  temp_buffer = temp_buffer | ((AlarmBits & 0b00000100) << 5);
  // A1 hour is figured out, send it
  Wire.write(temp_buffer);
  // Figure out A1 day/date and A1M4
  temp_buffer = ((AlarmBits & 0b00001000) << 4) | bin2bcd(A1Day);
  if (A1Dy) {
    // Set A1 Day/Date flag (Otherwise it's zero)
    temp_buffer = temp_buffer | 0b01000000;
  }
  Wire.write(temp_buffer);
  Wire.endTransmission();
}

void RTC_DS3231_turnOnAlarm(byte Alarm) {
  // turns on alarm number "Alarm". Defaults to 2 if Alarm is not 1.
  //byte temp_buffer = RTC_DS3231_readControlByte(0);
  byte temp_buffer = i2c_readRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG);
  // modify control byte
  if (Alarm == 1) {
    temp_buffer = temp_buffer | 0b00000101;  //bitwise OR //either or both
  } else {
    temp_buffer = temp_buffer | 0b00000110;
  }
  //RTC_DS3231_writeControlByte(temp_buffer, 0);
  i2c_writeRegisterByte(DS3231_ADDRESS,DS3231_CONTROL_REG,temp_buffer);
}

// unixtime from https://github.com/MrAlvin/RTClib
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

//===================================================
// ERROR HANDLER      "Houston we have a problem..."
//===================================================
void error() {
 
#ifdef ECHO_TO_SERIAL
   noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //8mhz
   Serial.print(F("Current Battery @"));
   Serial.print(currentBatteryRead); Serial.flush();
#endif
  
  power_twi_enable();
  RTC_DS3231_turnOffBothAlarms(); //before we disable I2C
  noInterrupts ();
  EIFR = bit (INTF0);  // clear flag for interrupt 0  see: https://gammon.com.au/interrupts
  EIFR = bit (INTF1);  // clear flag for interrupt 1
  interrupts (); 
 
  for (int CNTR = 0; CNTR < 240; CNTR++) {   // FLASH red indicator LED to indicate error state
    PORTB  = PORTB ^ 0b00100000;             // ^ TOGGLES D13 LED pullup resistor ON/Off
    LowPower.powerDown(SLEEP_250MS, ADC_ON, BOD_OFF);
  }
  PORTB &= B11011111; // pin13 pullup resistor OFF
  
  bitSet(ACSR,ACD); // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0; // Disable ADC & SPI   //only use the PRR after setting ADCSRA to zero, otherwise the ADC is "frozen" in an active state.
  power_all_disable(); // Turn everything else off - must come after disabling ADC.
  //wdt_disable();     // wdt is off by default

  //ALL pins to INPUT PULLUP
  DDRB &= B11100000; // pins 12..8 set to zero for input
  DDRD &= B00000000; // pins 7..0 set as inputs - d1&0 included?
  DDRC &= B11000000; // unused A5..A0 set as inputs 
  //enable pullups 
  PORTB |= B00011111 ; // 12..8 pullups on, but NOT D13 LED
  PORTD |= B11111111 ; // 7..0 pullups on 
  PORTC |= B00001111 ; // pins A3..A0  // A5/4 has 4k7 hardware pullups

  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF);  //use ADC_ON because we alreay have ADC turned off!
}
