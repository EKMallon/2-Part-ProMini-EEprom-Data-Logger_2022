// Cave Pearl Project 2-Part logger code by Edward Mallon
// The build tutorial for this logger can be found at:
// https://thecavepearlproject.org/2022/03/09/powering-a-promini-logger-for-one-year-on-a-coin-cell/

// with a companion video tutorial at:
// https://www.youtube.com/watch?v=58ps9fUyY0Q&t=0s&ab_channel=EdwardMallon

/*
This program supports an ongoing series of DIY 'Classroom Logger' tutorials 
from the Cave Pearl Project. The goal is to provide a starting point 
for self-built student projects in environmental monitoring courses.
This 'low power' 2-module iteration runs the logger from a CR2032 coin cell and uses 
EEprom memory to store sensor readings. This necessarily involves several power 
optimization steps which add complexity to the base code as compared to previous 
versions, but hopefully everyone can read through the code and understand 
what is happening from the comments. Data download & logger control are managed 
through the IDE's serial monitor window at 500000 baud. The logger WILL NOT START 
taking readings until those serial handshakes are completed via a UART connection.
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
#include <LowPower.h>   // library from https://github.com/rocketscream/Low-Power

//===============================================================
// LOGGER OPERATING PARAMETERS:  Adjust these to suit your build!
//===============================================================

//#define ECHO_TO_SERIAL  // prints readings to screen for USB tethered debugging
// also starts the run with no sample interval sync delay so timestamps are not aligned
// DO NOT ENABLE ECHO_TO_SERIAL DURING BATTERY POWERED OPERATION or system wastes a lot of power waiting for a serial response that never arrives

// Populate deploymentDetails[] with information about your logger. For example:
const char deploymentDetails[] PROGMEM = "1085872L,10Kref[D6]/NTC[D7]/CDS[D9]-104cap,RGB a0-3,Built:20220216";

#define InternalReferenceConstant 1085872L //1126400L is default value! = 1100mV internal vref * 1024 
// adding/subtracting 400 from the constant raises/lowers the 'calculated' result from
// readBattery() by ~1 millivolt, simply read the rail with a DVM while running on UART power
// change the constant, reload & restart with ECHO_TO_SERIAL on until the voltage is correct

boolean SaveBatteryEveryCycle = true; // default=false to save battery log only OncePerDay at midnight
// with false it takes 8 days before the readings in opdDataBuffer[16] gets saved to eeprom!
// so you get no battery data on runs shorter than that with SaveBatteryEveryCycle = false;
// set SaveBatteryEveryCycle=true to record battery voltage more often during debug & testing
// true forces the battery record every time a full sensorDataBuffer[] gets transfered to eeprom
// but this also uses a significant amount of memory - limiting your runtime 

// Enable these to match your sensor configuration: 
// NOTE: total sensor BYTES must be even divisior of #bytes in sensorDataBuffer[16];
#define RTC_Temperature     // 1-byte: 0.25C resolution temp. using the RTC's internal temperature sensor
//#define ReadNTC_6ref7ntc  // 2-bytes: can be combined with ReadLDR_onD9 or stand alone
//#define ReadLDR_onD9      // 2-bytes: can be combined with ReadNTC_6ref7ntc or stand alone
//#define BMP280_Address 0x76 // 4bytes: a 'code example' to show steps required to add other sensors to your logger
//#define bh1750_Address 0x23

boolean blinkLED = true;    // enables/disables LED pips except those in setup & error functions
//#define LED_GndGB_A0-A2   // A0=gnd & Green=A1, blue=A2 if you added an LED as shown in the build video
// NOTE: default Red LED on D13 pips if LED_GndGB_A0-A2 remains commented out

#ifdef ReadNTC_6ref7ntc
const char NTCcal[] PROGMEM = "No cal for this unit yet...";
 // usually something like this after calibration (and yes, the RTC temp can be calibrated too!)
 // typically something like: "unit 1127315L,Cal202202:,A=,0.001246979,B=,0.000216248,C=,0.0000001537444523,R(25°C)=9834.81Ω,β=3850.83K,RTCy=0.9744x -0.4904";
#endif

// MUST SET the eeprom address/bytes manually to match your hardware config!
// Run a bus scanner to check where your eeproms are https://github.com/RobTillaart/MultiSpeedI2CScanner
#define opdEEpromI2Caddr 0x57     // default: 0x57 for 4k EEprom on RTC module with all address pins pulled high
#define opdEEbytesOfStorage 4096  // default: 4096 bytes for 4k EEprom
#define sensorEEpromI2Caddr 0x57
#define sensorEEbytesOfStorage 4096
// OPD & SENSOR can be set to different physical eeproms or the same eeprom
// AT24c256 YL-90(red module) is 32768 @0x50 // OR // 64k AT24c512(via chip swap) is 65536 bytes
// The two buffers are saved into memory from 'opposite ends'
// like stack vs heap in SRAM:  https://learn.adafruit.com/memories-of-an-arduino/optimizing-sram

//===========================================================================
// most variables below this point stay the same from one machine to the next
//===========================================================================

// NOTE: intervals get set in setup via Serial input
uint8_t SampleIntervalMinutes = 15;  // Allowed values: 1,2,3,5,10,15,20,30 or 60 - must divide equally into 60!
uint8_t SampleIntervalSeconds = 0;   // used for rapid burn tests only, minutes must be zero for intervalseconds
// Make sure your sensor readings don't take longer than your sample interval!
// If you over-run your next alarm you will have to wait 24hours for next wakeup

int8_t opdDataBuffer[16];         // [16] is max unless you increase the I2C buffers
int8_t opdArrayPointer= sizeof(opdDataBuffer);    // this Pointer increments backwards & must be able to hold negative value without overflow
uint8_t opdBytesPerRecord = 2;    // saves Highest & Lowest battery read per cycle as 1 byte each
uint32_t opdEEprMemPointer = opdEEbytesOfStorage; // this Pointer increments backwards

uint8_t sensorDataBuffer[16];
uint8_t sensorArrayPointer = 0;   // this pointer increments forward and requires no negative
uint8_t sensorBytesPerRecord = 0; // this gets changed in SETUP 
                                  // to match different sensor combinations
uint16_t systemShutdownVoltage = 2785; // MUST be > BOD default of 2700mv (also EEprom limit)
byte default_ADCSRA,default_ADMUX;     // stores default register settings
byte set_ADCSRA, set_ADMUX;            // custom settings for readbattery() via 1.1 internal band gap reference
volatile uint8_t adc_irq_cnt;          // used in readADCLowNoise ISR to allow averaging
uint16_t currentBatteryRead = 0;
uint16_t LowestBattery = 5764;
uint16_t HighestBattery = 0; 
boolean newBatteryReadReady = false;

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
uint8_t AlarmBits;
volatile boolean rtc_INT0_Flag = false;  // volatile because it's changed in an ISR
boolean midnightRollover = false; 
int rtc_TEMP_Raw = 0; 
float rtc_TEMP_degC = 0.0;
union {                       // for Unixtime INDEX storage & retrieval from eeproms
  uint32_t cyleTimeStart;     // 0-4,294,967,295 large enough for unixtime
  uint8_t EE_byteArray[4];    // Arduino: char = int8_t but byte = uint8_t
} utime;

// temporary variables for used during calculations =============================
bool bitFlag;                  // for fuctions that return a bit state
uint8_t byteBuffer1 = 0;       // byte (8 bit) type = unsigned number from 0 to 255
uint8_t byteBuffer2 = 0;       // uint8_t data type same as byte in Arduino
int16_t integerBuffer = 9999;      //2-byte  from -32,768 to 32,767
int32_t longInt32_Buffer = 9999;  //4-byte  from -2,147,483,648 to 2,147,483,647
uint16_t uint16_Buffer;           //2-byte from 0-65535
uint32_t uint32_Buffer;
float floatBuffer = 9999.9;    // for various float calculations

// info printed with the boilerplate IF ECHO is on ========================
#define fileNAMEonly (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
// from: https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
const char compileDate[] PROGMEM = __DATE__; 
const char compileTime[] PROGMEM = __TIME__;
 
#ifdef BMP280_Address          //================================================
#include <BMP280_DEV.h>   // from    https://github.com/MartinL1/BMP280_DEV
BMP280_DEV bmp280;        // Create a BMP280_DEV library object. called ‘bmp280’
float Bmp280_Temp_degC,  Bmp280_Pr_mBar,  Bmp280_altitude;    // 3 float variables for output
#endif  //BMP280_Address       // ================================================

#ifdef bh1750_Address     //=============================================
#include <hp_BH1750.h>      //from  https://github.com/Starmbi/hp_BH1750  
//this lib returns the sensor to sleep automatically after each read & supports auto-ranging.
hp_BH1750 bh1750;        // Instantiate a BH1750FVI library object
float lux_BH1750;        // a variable to receive the sensor reading
//int32_t BH1750_NewReading = 10000;
//int32_t BH1750_OldReading = 10000;
union {
  uint32_t lux_Integer;     //0-4,294,967,295
  uint8_t lux_byteArray[4]; //0-255 each byte
} BH1750sensor;
//Note: 'unsigned long' is Little Endian on ProMini
//BH1750sensor.lux_Integer = 0x5CE3B975
//BH1750sensor.lux_byteArray[0] = 0x75, BH1750sensor.lux_byteArray[1] = 0xB9, and so on.
//ALSO arrays are ZERO indexed so 4 element array is indexed with 0-3
//ALT method without union: https://forum.arduino.cc/t/split-long-into-bytes-for-transmitting-over-ic2-solved/90004
#endif  // bh1750_Address ================================================


//====================================================
#if defined(ReadNTC_6ref7ntc) || defined(ReadLDR_onD9)
//====================================================
#define referenceResistorValue 10000UL  //an ARBITRARY value (the 10k ref resistors true value does not matter)
volatile boolean triggered;          
volatile uint16_t timer1CounterValue;
//  volatiles needed for all digital pin reading of resistance
//  prepareForInterrupts(), ISR (TIMER1_OVF_vect), ISR (TIMER1_CAPT_vect),ReadD6riseTimeOnD8
#endif

#ifdef ReadNTC_6ref7ntc   //========================================
  uint32_t NTC_NewReading;     // max of 65535 limits our ability to measure 10kNTC resistance below zero C
#endif
#ifdef ReadLDR_onD9    //============================================
  int32_t LDR_NewReading;     // 65535 limits so we test the value of the CDScell with discharge before measuring
#endif

//======================================================================================================================
//  *  *   *   *   *   *   SETUP   *   *   *   *   *
//======================================================================================================================
// NOTE: problems in setup call error() routine which halts the system!

void setup () {
  bitClear(DDRB,5);bitClear(PORTB,5); // D13 input&low 
  // led pips ALWAYS use INPUT PULLUP resistor to reduce current load

  #ifdef RTC_Temperature
    sensorBytesPerRecord++;   // encoded to 1-Byte
  #endif
  #ifdef ReadNTC_6ref7ntc 
    sensorBytesPerRecord+= 2; //  two byte integer
  #endif
  #ifdef ReadLDR_onD9
    sensorBytesPerRecord+= 2; //  two byte integer
  #endif
  #ifdef BMP280_Address
    sensorBytesPerRecord+= 4; //  sensor also provides temp
  #endif
  #ifdef bh1750_Address
    sensorBytesPerRecord+= 4; //  four byte LONG integer
  #endif

  // configure ADC on 168/328 boards to read the rail voltage using the internal 1.1 aref
  default_ADCSRA = ADCSRA; default_ADMUX = ADMUX; // in case we need to restore defaults later
  bitSet(ACSR,ACD); // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  DIDR0 = 0x0F;     // Disables the digital input on analog 0..3   (analog 4/5 used for I2C!)
  analogReference(INTERNAL);analogRead(A6);   // engauge aref cap @ 1.1v
  // An ADC read takes 13 ADC clock cycles(though 1st one takes 20), so default is about 9615 Hz (or 0.1 milliseconds per).
  //bitWrite(ADCSRA,ADPS2,0);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,1); // 8 ADC prescalar  = 8x normal speed
  //bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,0);bitWrite(ADCSRA,ADPS0,0); // 16 ADC prescalar  = 4x normal speed
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,0);bitWrite(ADCSRA,ADPS0,1);   // 32 ADC prescalar = 2x normal the speed of the ADC
  //bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,0);   // 64 (default) prescalar @ 8MHz/64 = 125 kHz, =~104uS/ADC reading
  
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

  #ifdef LED_GndGB_A0-A2
  DDRC &= B11110001;  // unused A3..A1 set as inputs
  PORTC &= B1110000;  // A3..A0 LOW = pullups OFF
  DDRC |=B00000001;   // A0 set OUTPUT =the GND line for the RGB led
  #endif
  
// Digital input buffers can draw a relatively high amount of current if the input is close to half-Vcc
// Turning the digital buffer off is available only for analog pins. When you turn the ADC clock off 
// (when you set PRADC bit in PRR register) the DIDR0 register is no longer accessible.
  //PORTC &= B11000000; // A5..A0 pullups OFF (I2C will take control of A5/4 at wire.begin)
  //DDRC &= B11000000;  // unused A5..A0 set as inputs
  //bitSet (DIDR0, ADC0D);  // disable digital buffer on A0
  //bitSet (DIDR0, ADC1D);  // disable digital buffer on A1
  //bitSet (DIDR0, ADC2D);  // disable digital buffer on A2 - because we have repurposed this SCREW TERMINAL port for I2C A4 DATA
  //bitSet (DIDR0, ADC3D);  // disable digital buffer on A3 - because we have repurposed ST port for I2C A5 SCLOCK
  //DIDR0 = 0x0F;//  disables the digital inputs on analog 0..3 (NOT analog 4&5 being used by I2C!)   
  //Once disabled, a digitalRead on those pins will always return zero.
    
  Wire.begin();  // enables internal 30-50k pull-up resistors on SDA & SCL by default
  TWBR = 2;      // for ~400 kHz I2C bus @ 8MHz CPU
                 // TWBR = 32; = default 100 kHz I2C bus @ 8MHz CPU

  Serial.begin(500000);   // Opening the serial monitor "resets" the Arduino
                          // max at 8Mhz http://wormfood.net/avrbaudcalc.php

  RTC_DS3231_turnOffBothAlarms();   //stops RTC from holding the D2 interrupt line low if system reset just occured 
  DDRD &= B11111011;                // D2 set as input
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_STATUS_REG, 3, 0);  // disable the 32khz output  pg14-17 of datasheet  // This does not reduce the sleep current
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 6, 1); // 0Eh Bit 6 (Battery power ALARM Enable) - MUST set to 1 for wake-up alarms when running from the coincell bkup battery
  i2c_setRegisterBit(DS3231_ADDRESS, DS3231_CONTROL_REG, 7, 0); // Enable Oscillator (EOSC).  This bit is clear (logic 0) when power is first applied.
  // when EOSC is 0, the oscillator continues running during battery operation. The RTC doesn't change the EOSC bit on its own.
  // When EOSC set to logic 1, the oscillator is stopped when DS3231 switches to VBAT.
  
  RTC_DS3231_getTime();
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);  

//===================================================================
//=========== Main Download Data Menu  ==============================
//===================================================================

  send_serial_boilerplate(); printMenuOptions();

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
          sendOPDreadings2Serial();
          startMicros = micros(); //resets the while loop for another 100 seconds
          printMenuOptions();goFlagReceived=false;
    }
    else
    if(command == "raw")   //only used for debugging
    {
          Serial.println(F("Downloading: 'Raw' bytes from EEprom:"));
          //NOTE: raw download does not include OPD data unless they are in the same eeprom
          sendSensorData2Serial(false);
          startMicros = micros(); //resets the while loop for another 100 seconds
          printMenuOptions();goFlagReceived=false;
    }
    else
    if(command == "start")   // a second 'start' will be required to launch the run
    {goFlagReceived=true;}
    else
    if(command == "test")    // HIDDEN OPTION only used for rapid debugging/testing (not displayed on menu)
    {goFlagReceived=true;}

 if (goFlagReceived) break; //breaks out of the while loop
 
}while ((micros() - startMicros) < 100000); // 100 seconds to respond?
// terminates: do-while timeout loop

if (!goFlagReceived){   // if goflag=false then shut down the logger
Serial.println(F("Timeout: NO valid command received -> shutting down"));Serial.flush();
error(); //shut down the logger
}


//===================================================================
//=========== Set RTC time (if needed )==============================
//===================================================================
RTC_DS3231_getTime(); sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);  
Serial.print(F("Current RTC time = "));Serial.print(CycleTimeStamp);Serial.print(F(":"));Serial.print(t_second);
Serial.println(F(" Reset the time? (y/n) & enter"));
goFlagReceived = false; startMicros = micros();
  do { 
    command = Serial.readStringUntil('\n'); // read string until return character
    if(command == "y")   // a second 'start' will be required later to start logging
      {goFlagReceived=true; t_year=0;         //forces clock update
    }
    else
    if(command == "n")
      {goFlagReceived=true;}
    if (goFlagReceived) break;                //breaks out of the while loop
  }while ((micros() - startMicros) < 60000);  // 60 seconds to respond?

bool DS3231_lostpower = i2c_readRegisterByte(DS3231_ADDRESS,DS3231_STATUS_REG) >> 7;
  if (DS3231_lostpower){
    Serial.println(F("RTC Power-loss detected!"));
      //Oscillator Stop Flag (OSF). A logic 1 in bit7 indicates that the oscillator is stopped or was stopped for some period due to power loss 
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
  sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);  
  Serial.print(F("RTC NOW SET to: "));Serial.println(CycleTimeStamp);
  i2c_setRegisterBit(DS3231_ADDRESS,DS3231_STATUS_REG,7,0);//clear the OSF flag after time is set
  }
} //terminator for Seting RTC time

//===================================================================
//========= Set sampling interval via serial (if needed ) ===========
//===================================================================
// do I need to make these into two byte integers instead of uint8_t
// for calculations later in send serial?
SampleIntervalMinutes = EEPROM.read(1008); // uint8_t
SampleIntervalSeconds = EEPROM.read(1009); // uint8_t
Serial.print(F("Current sampling interval is "));
Serial.print(SampleIntervalMinutes);Serial.print(F(" min. "));
Serial.print(SampleIntervalSeconds);Serial.println(F(" sec. "));
Serial.println(F("Reset the interval? (y/n) & enter"));
goFlagReceived = false; startMicros = micros(); //String command; already declared
  do { 
    command = Serial.readStringUntil('\n'); // read string until return character
    if(command == "y")   // a second 'start' will be required to launch the run
      {goFlagReceived=true; t_year=0; //forces clock update
    }
    else
    if(command == "n")    // hidden option only used for rapid debugging/testing (not displayed on menu)
      {goFlagReceived=true;}
    if (goFlagReceived) break; //breaks out of the while loop
  }while ((micros() - startMicros) < 60000); // 60 seconds to respond?

  // force reset if existing interval settings are not valid:
  // however X % 0 undefined but zero values are possible (and valid)
  // So we need a conditional using ? shorthand for if(bolean) then, else [all non zeros =true]
  // of first term is >0, byteBuffer gets set to 60%sim, else it gets set to 0
  byteBuffer1 = SampleIntervalMinutes ? 60 % SampleIntervalMinutes : 0;
  byteBuffer2 = SampleIntervalSeconds ? 30 % SampleIntervalSeconds : 0;
  
  if ((byteBuffer1 !=0) || (byteBuffer2 !=0)){
      command = "y"; // if EITHER number is not an even divisor of 60  OR  zero
      }   
  if ( SampleIntervalMinutes>60 || SampleIntervalSeconds>60){
      command = "y";
      }
  if ( SampleIntervalMinutes==0 && SampleIntervalSeconds==0){
      command = "y";
      }
      
  if (command == "y"){
    do {
    Serial.println(F("Input a sampling interval as 1,2,5,15,30 or 60 minutes:"));
    byteBuffer1 = 0;
    SampleIntervalMinutes = Serial.parseInt();// not doing a complete error check here
    byteBuffer1 = SampleIntervalMinutes ? 60 % SampleIntervalMinutes : 0;
    }while((byteBuffer1 !=0) || (SampleIntervalMinutes>60));  // must be valid divisor of 60 OR zero
    Serial.print(F("  Sample Interval set to: ")); Serial.print(SampleIntervalMinutes);Serial.println(F(" min"));
    if (SampleIntervalMinutes>0){SampleIntervalSeconds = 0;}
    
      // I use sub minute sampling ONLY for rapid burn tests so this is a
      // HIDDEN OPTION for rapid debugging/testing (so it's not displayed on the menu)
      if (SampleIntervalMinutes==0){ 
        do {
         Serial.println(F("Enter interval as 5,10,15,20,or 30 seconds:"));
          byteBuffer2 =0;
         SampleIntervalSeconds = Serial.parseInt();
          byteBuffer2 = SampleIntervalSeconds ? 30 % SampleIntervalSeconds : 0;
        }while ((byteBuffer2 !=0) || (SampleIntervalSeconds>30)); // must be valid divisor of 60 OR zero
      Serial.print(F("  Sub-minute Sample Interval: ")); Serial.print(SampleIntervalSeconds);Serial.println(F(" seconds"));
      }

    if (SampleIntervalMinutes==0 && SampleIntervalSeconds==0){
      Serial.println(F("Invalid Sampling Interval entered: 15 min DEFAULT being SET"));Serial.flush();
      SampleIntervalMinutes = 15;SampleIntervalSeconds = 0;
      }
    EEPROM.update(1008,SampleIntervalMinutes);
    EEPROM.update(1009,SampleIntervalSeconds);
} //if (command == "y"){

//===================================================================
//========= Serial Input: brief deployment description ==============
//===================================================================
Serial.println(F("Type in a brief description of the deployment (& your name)"));
Serial.println(F("followed by [Enter] key  (255 characters MAX)  [Timeout in 100 sec]"));

command =""; goFlagReceived = false;
//Serial.setTimeout(30000);  //is this needed again?
byteBuffer1=0;//retry counter limits to 3x
startMicros = micros();
do { 
    command = Serial.readStringUntil('\n'); // read string until newline character
    integerBuffer = command.length();       // length does not inlude the nul terminator
    
    if(integerBuffer>255)
    {
          Serial.println(F("Input too long: 255 characters max"));
          command ="";//deletes all characters in the string
          Serial.println(F("Try again..."));
          startMicros = micros(); //resets the while loop for another 100 seconds
          goFlagReceived=false;byteBuffer1++;
    }
    else if(integerBuffer<2){           //you can't just hit the enter key
    Serial.println(F("No description recieved? Try again."));
          startMicros = micros(); //resets the while loop for another 100 seconds
          goFlagReceived=false; byteBuffer1++;
    }
    else
    {goFlagReceived=true;}
    if (byteBuffer1>=4) break;    //limits # of re-tries before timeout shutdown
    if (goFlagReceived) break;    //breaks out of the while loop
}while ((micros() - startMicros) < 100000); // 100 seconds to respond?

if (!goFlagReceived){   // if goflag=false then shut down the logger
Serial.println(F("NO valid information received -> shutting down"));Serial.flush(); error(); // shutdown
}

// erase previously saved description         // note first 512 spaces already use for boilerplate/calibration data
for (uint16_t h = 512; h < 767; h++){         // update does not write unless new content is different
    EEPROM.update(h,32);                      // fill memory with ascii32 = blank space character
    if ((h % 16) == 0){Serial.print(F("."));} // progress indicator dots
    delay(4);                                 // needs 3.3 msec per byte/char write
}
// save new description to EEprom
for (uint16_t i = 0; i < command.length(); i++){
    EEPROM.update(i+512,command.charAt(i));
    if ((i % 16) == 0){Serial.print(F("."));}
    delay(4);
}  
// remaining 256 bytes of internal eeprom used for 
// STARTING index values for time & initial sensor values captured at runtime

Serial.println();Serial.print(F("Deployment details saved: "));
Serial.println(command);Serial.println();Serial.flush();

//=========== FINAL check before erasing eeprom & starting next run ===================
  Serial.println(F("PROCEEDING at *this* point will ERASE ALL previous DATA!"));
  Serial.println(F("Type 'start'(&enter) to clear the EEprom & begin logging"));
  Serial.println(F("(or re-start serial monitor to retrieve the old data)"));
  Serial.println();

goFlagReceived=false;startMicros = micros();
do {  
    command = Serial.readStringUntil('\n'); // read string until meet newline character
    if(command == "start")
    {
        //Serial.println(F("Erasing 328p internal EEprom"));Serial.flush();
        //for (int i = 0; i < 1024; i++){  EEPROM.update(i,0); } 
        //EEPROM.update only 'writes' a location if it is being changed (to preserve eeprom lifespan)   
  
        Serial.println(F("Erasing EEprom: Stay on UART power until done. (up to 1min w multiple EEproms)"));Serial.flush();
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
          if ((CurrentMemAddress % 4096) == 0){Serial.println();}   //return every 32 dots
          do  // poll the eeprom to see when next bytes can be written:
          { Wire.beginTransmission(opdEEpromI2Caddr); }
          while (Wire.endTransmission() != 0x00);
        }// terminates  for (int i=0; i<(opdEEbytesOfStorage/sizeof(opdDataBuffer)); i++){ 

      for (int j=0; j<sizeof(sensorDataBuffer); j++) {sensorDataBuffer[j] = 0;} //sensor databuffer now contains 0s
      
      if (sensorEEpromI2Caddr != opdEEpromI2Caddr) { //skip if sensor & opd are in same eeprom 
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
          if ((CurrentMemAddress % 4096) == 0){Serial.println();}   //return every 32 dots
          //serial print takes about 10 seconds divided by the baud rate, per character. = 1/10ms per char at 57000

          do  //we are still powered by uart: poll the eeprom to see when next bytes can be written:
          { Wire.beginTransmission(sensorEEpromI2Caddr); }
          while (Wire.endTransmission() != 0x00);
          
          } //terminates for (int i=0; i<(sensorEEbytesOfStorage/sizeof(sensorDataBuffer)); i++){
      } // terminates if (sensorEEpromI2Caddr != opdEEpromI2Caddr) 
     
          Serial.println();Serial.print(F("EEprom loaded with zeros: "));
          Serial.flush();
          goFlagReceived=true;
    }
    else
    if(command == "test")    //note this option is not in the options menu
    {goFlagReceived=true;}   //skips erasing eeprom: only used for rapid debugging/testing
 
  if (goFlagReceived) break; //breaks out of the while loop
 
}while ((micros() - startMicros) < 100000); // 100 seconds to respond before timeout

if (!goFlagReceived){   // if goflag=false then shut down the logger
Serial.println(F("Timeout w NO command recieved -> logger shutting down"));Serial.flush();
      error();         //shut down the logger
}

//===================================================================
//===================================================================
// initialize sensors  - AFTER wire.begin(); starts the I2C bus
//===================================================================
//===================================================================

#ifdef BMP280_Address
bmp280.begin(BMP280_Address); // or bmp280.begin(BMP280_I2C_ALT_ADDR); for sensors at 0x76
//Options are OVERSAMPLING_SKIP, _X1, _X2, _X4, _X8, _X16 // pg 15 datasheet One millibar = 100 Pa
bmp280.setPresOversampling(OVERSAMPLING_X16);
//×2 Low power              17 bit / 1.31 Pa     7.5-8.7ms    3-4µA @ 1 Hz forced mode
//×4 Standard resolution    18 bit / 0.66 Pa     11.5-13.3ms  7-10µA @ 1 Hz forced mode
//×8 High resolution        19 bit / 0.33 Pa     19.5-22.5ms  12-18µA @ 1 Hz forced mode
//×16 Ultra high resolution 20 bit / 0.16 Pa     37.5-43.2ms  24-37µA @ 1 Hz forced mode
bmp280.setTempOversampling (OVERSAMPLING_X2); // NOTE: NO benefit to Pr accuracy beyond TempOS X2
//×2 Low power              17 bit / 0.0025 °C
//×4 Standard resolution    18 bit / 0.0012 °C
//×8 High resolution        19 bit / 0.0006 °C
//×16 Ultra high resolution 20 bit / 0.0003 °C
bmp280.setSeaLevelPressure (1013.25f);  // default value for altitude calculations
bmp280.setIIRFilter(IIR_FILTER_OFF); 
bmp280.startForcedConversion(); // time needed here depends on oversampling settings
LowPower.powerDown(SLEEP_60MS, ADC_ON, BOD_OFF); // long enough for max rez.
bmp280.getCurrentMeasurements(Bmp280_Temp_degC,Bmp280_Pr_mBar,Bmp280_altitude);
Serial.println(F("BMP280 started"));Serial.flush();
#endif //BMP280_Address

#ifdef bh1750_Address
  Serial.println(F("BH1750 light sensor MUST be exposed to >150 lux @ startup")); 
  Serial.println(F("for self-cal or it may freeze randomly. 15 sec = minimum interval."));Serial.println();Serial.flush();
  bh1750.begin(bh1750_Address); // set address & initialize
  bh1750.calibrateTiming();     // NOTE: ambient light must be at least 140 lux when this runs!
  bh1750.start (BH1750_QUALITY_LOW, BH1750_MTREG_LOW); // Quality LOW = fastest readings
  // H-Resolution Mode Measurement Time 120-180 ms depending on light levels
  // L-Resolution Mode Measurement Time 16-24 msec //LOW MTreg:31  resolution lux:7.4, 121557 is highest lux
#endif //bh1750_Address

Serial.println(F("Starting the Data logger..."));Serial.flush();
  
//===================================================================
//Delay start of logger until 1st sampling alarm is in sync with hour
//===================================================================
//otherwise you get a "clipped interval" at the first hour rollover
//AND save index values (for later Unixtime reconsruct) to 328p internal 1k eeprom WHILE still tethered to UART power

  RTC_DS3231_getTime();
  uint32_t timeCalcVariable = RTC_DS3231_unixtime();

  //save time-index for OncePerDay readings @ end of 328P INTERNAL 1K eeprom
  //calculate next midnight rollover -assumes integer (truncating) division
  utime.cyleTimeStart = ((timeCalcVariable/86400UL)*86400UL)+86400UL; 
  EEPROM.put(1020,utime.EE_byteArray); // blocks processor for 4bytes x 3msec = 12 milliseconds
  // LowPower.powerDown(SLEEP_15MS,ADC_ON,BOD_OFF); // Not needed - still on UART power
  
#ifdef ECHO_TO_SERIAL    // no sync delay if ECHO is on
    Serial.println(F("ECHO_TO_SERIAL is ON: Sync delay disabled."));
//NOTE: without the sync delay your timestamps will be ODD & you will occasionally get a bad start bc of over-running the next alarm
    Serial.println();Serial.flush(); // In debug mode dont wait for SampleIntervalMinutes to pass before startup
    integerBuffer=0;
      if(SampleIntervalMinutes==0){ //add brief delay for seconds-only intervals used in testing
        integerBuffer = SampleIntervalSeconds - (t_second % SampleIntervalSeconds);
      delay(integerBuffer*1000);
      }
    utime.cyleTimeStart = timeCalcVariable + integerBuffer; //= RTC_DS3231_unixtime();
    EEPROM.put(1016,utime.EE_byteArray); // sensor reads time-index value '1 step before end' locations 

#else // if NO ECHO_TO_SERIAL then sleep delay till time is sync'd with our sampling interval

  Alarmday = t_day; Alarmhour = t_hour; Alarmsecond = 0; // Alarmminute = gets calculated;
      
  if(SampleIntervalMinutes==0){ // sub-minute intervals only for rapid testing
    Alarmminute=t_minute+2;  // delay becomes remainder of current minute + 1 minute
  } else {    //  SampleIntervalMinutes>0  (normal default)
    Alarmminute=((t_minute/SampleIntervalMinutes)*SampleIntervalMinutes) + (2*SampleIntervalMinutes); 
    // last alignment + 1 sample interval = NEXT alignment time  + one more SampleIntervalMinutes
  } // terminates if(SampleIntervalMinutes==0)

  if (Alarmminute > 59) { // checking for Alarmminute roll-over //can result in start delays clipped due to sminutes =0
     Alarmminute = 0; Alarmhour = Alarmhour + 1; //this line does not work with SampleIntervalMinutes =60
        if (Alarmhour > 23) {
          Alarmhour = 0; 
          Alarmday = Alarmday + 1;  // but setAlarm1Simple has no days input?
                                    // what about month rollovers?
        }
   }

  // same calculation in RTC_DS3231_unixtime();
  uint16_Buffer = date2days(t_year, t_month, Alarmday); // the days calculation
  uint32_Buffer = time2long(uint16_Buffer, Alarmhour, Alarmminute, Alarmsecond);
  uint32_Buffer += 946684800;  // add # seconds from 1970 to 2000 which we took away with y -= 2000 in date2days
  utime.cyleTimeStart = uint32_Buffer;  // this will be the unixtime when we wake after the delay
  EEPROM.put(1016,utime.EE_byteArray);  // still tethered to USB at this point so uart supplies power 
                                        // LowPower.powerDown(SLEEP_15MS,ADC_ON,BOD_OFF);   
  RTC_DS3231_setAlarm1Simple(Alarmhour, Alarmminute);
  RTC_DS3231_turnOnAlarm(1); // will break logger out of flashing RED&BLUE light sync delay
  // or // AlarmBits=0b00001000; // RTC_DS3231_setA1Time(Alarmday,Alarmhour,Alarmminute,Alarmsecond,AlarmBits,false,false,false);   

  Serial.println(F("RED (&Blue)LED will now flash @1sec until the 1st reading."));
  Serial.println(F("Disconnect from serial UART now - NO additional messages."));Serial.flush();

//LED's in setup are not controled by if(blinkLED) bc they MUST BE OBSERVED to confirm OK startup
  bitClear(DDRB,5);bitClear(PORTB,5); // D13 [default] RED led to INPUT & LOW
  #ifdef LED_GndGB_A0-A2  // RED & BLUE leds start in opposite states so ALTERNATE when toggled
    PORTC&= B11111110; DDRC|=B00000001; // A0 LOW (0) & OUTPUT (1) // LED ground
    bitClear(DDRC,2);bitSet(PORTC,2);   // A2 [Blue] LED INPUT & PULLUP ON
  //red & blue leds configured to ALTERNATE while waiting for clock sync delay
  #endif
  
   noInterrupts ();         // make sure we don't get interrupted before we sleep
   bitSet(EIFR,INTF0); bitSet(EIFR,INTF1);  // clear flag for interrupt 0 (D2) &  interrupt 1 (D3) see https://gammon.com.au/interrupts
   attachInterrupt(0,rtcAlarmTrigger, LOW); //RTC SQW alarms LOW connected to pin D2
   interrupts (); rtc_INT0_Flag=false;      // Flag gets set inside ISR rtcAlarmTrigger
     do{
        #ifdef LED_GndGB_A0-A2 // some rebuilds use Red on RGB @d13 requiring LED ground pin on A0
          PINC = B00000100;// toggles A2 [Blue] LED PULLUP with pinc write ~30uA
        #endif 
        PORTB  = PORTB ^ 0b00100000; // toggles D13 LED pullup  ~50uA to light led through internal pullup resistor
        LowPower.powerDown(SLEEP_1S, ADC_ON, BOD_OFF); //ADC already off, ADC_ON preserves this state
      }while(!rtc_INT0_Flag);// flag only true if RTC alarm triggers D2 ISR: rtcAlarmTrigger

    bitClear(PORTB,5);  // pin13 indicator LED PULLUP OFF
    #ifdef LED_GndGB_A0-A2
    bitClear(PORTC,2);  // A1 [Blue] LED PULLUP OFF
    #endif
  RTC_DS3231_turnOffBothAlarms(); //detachInterrupt(0); was done in the interrupt 
#endif  // #ifdef ECHO_TO_SERIAL 

LowestBattery =5764; //1st readbattery in setup is often low because refcap not fully charged
currentBatteryRead = readBattery(); LowestBattery = currentBatteryRead;
TWBR = 2; // I2C bus to 400khz - OK for DS3231 but must slow down for 4k eeprom
// slightly out of spec: (1msec) Tlow while 1.3 microseconds is the official standard
//====================================================================================================
}  // terminator for void setup()
//=====================================================================================================

//========================================================================================================
//========================================================================================================
//========================================================================================================
//========================================================================================================
//      *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
//========================================================================================================
//========================================================================================================
//========================================================================================================
void loop (){

//===========================================================================================
//this for-loop wraps the "cycle of samples" repeating until the eeprom memory is full:
//===========================================================================================
for (uint16_t forwardEEmemPointr = 0; forwardEEmemPointr < (sensorEEbytesOfStorage -sensorBytesPerRecord); forwardEEmemPointr += sensorBytesPerRecord) { 

// if BOTH sensor & opd buffers are saving to the same eeprom...
   if (sensorEEpromI2Caddr == opdEEpromI2Caddr) {    // AND their memory pointers overlap the eeprom is full -> shutdown   
      if ((forwardEEmemPointr + sizeof(sensorDataBuffer)) >= opdEEprMemPointer){break;}
      // this leaves a buffer of 'zeros' between the two data streams being saved in the eeprom as 'stop' codes for sendSensorData2Serial & sendOPDreadings2Serial
      // note sizeof(sensorDataBuffer) & sizeof(opdDataBuffer) are usually the same size (within wire library limits)
   }
  
   RTC_DS3231_getTime();
   LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // battery recovery time after each I2C exchange

//calculate the time for your next RTC alarm:
//===========================================
    Alarmday = t_day;            // Alarmday = now.day(); //with #include <RTClib.h>
    Alarmhour = t_hour;          // Alarmhour = now.hour(); //with #include <RTClib.h>
    Alarmminute = t_minute + SampleIntervalMinutes; //Alarmminute = now.minute()+SampleIntervalMinutes; //with #include <RTClib.h>
    Alarmsecond = t_second + SampleIntervalSeconds; //only used for special testing & debugging runs - usually gets ignored

// Check for RTC TIME ROLLOVERS: THEN SET the next RTC alarm
//==========================================================
if (SampleIntervalMinutes > 0) //then our alarm is in (SampleInterval) minutes
    {
      if (Alarmminute > 59) {  //error catch - if alarmminute=60 or greater the interrupt never trigger
        Alarmminute = 0; Alarmhour = Alarmhour + 1;
        if (Alarmhour > 23) {
          Alarmhour = 0; 
            if(!SaveBatteryEveryCycle){   // skip the usual midnight reading IF you are already
              midnightRollover=true;      // forcing new battery readings at every sensordata save into the eeprom 
            }
        }
      }  //terminator for if (Alarmminute > 59) rollover catching

      RTC_DS3231_setAlarm1Simple(Alarmhour, Alarmminute);
    }  //terminator for if (SampleIntervalMinutes > 0)

else  //to get sub-minute alarms use the full setA1time function
    
    {  // for testing & debug I sometimes want the alarms more frequent than 1 per minute.
      if (Alarmsecond >59){
        Alarmsecond =0; Alarmminute = Alarmminute+1;  
        if (Alarmminute > 59) 
        {  //error catch - if alarmminute>=60 the interrupt never triggers due to rollover!
          Alarmminute =0; Alarmhour = Alarmhour+1; 
          if (Alarmhour > 23) { //uhoh a day rollover, but we dont know the month..so we dont know the next day number?
            Alarmhour =0;
            if(!SaveBatteryEveryCycle){   // skip the usual midnight read IF you are already forcing new voltage readings
              midnightRollover=true;      // at every sensordata save into the eeprom 
            }  
          }
        }
      }
      
      //function expects: (byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM)
      AlarmBits=0b00001000;       // ALRM1_SET bits and ALRM2_SET are 0b1000 and 0b0111 respectively.
      RTC_DS3231_setA1Time(Alarmday, Alarmhour, Alarmminute, Alarmsecond, AlarmBits, false, false, false);
      LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
    } // terminator for second else case of if (SampleIntervalMinutes > 0) 
    
  RTC_DS3231_turnOnAlarm(1);
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  currentBatteryRead = readBattery(); // SLEEP_15MS embedded
  //readBat resets HIbat, but NOT LObat which only happens during EEsave

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
LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);

#if defined(ReadLDR_onD9) || defined(ReadNTC_6ref7ntc)
ConditionCapacitorOnD8(); // charge cycles the cap through D8 to standardize condition
  //AND floats all pins with resistor/sensors connected to that common capacitor 
  //ConditionCap only needs to be called ONCE before other ICU resistor readings
 uint32_Buffer = ReadD6riseTimeOnD8();    //elapsedTime of 10k ohm resistor
#endif //many sleep_IDLE events (~1mA) during digital pin reads

// essentially: =(elapsedTimeSensor * referenceResistorValue) / elapsedTimeReff;
  //NOTE calculation works OK with uint32_t variables HOWEVER
  //With 105 capacitors must change functions to handle T1 overflows & cast this calculation to higher bits
  //NTC_NewReading=((uint64_t)elapsedTimeSensor * (uint64_t)referenceResistorValue) / elapsedTimeReff;

#ifdef ReadNTC_6ref7ntc   //==============================================
 NTC_NewReading = ReadD7riseTimeOnD8();   //elapsedTimeSensor uint32_t
 NTC_NewReading = (referenceResistorValue * (uint32_t)NTC_NewReading) / uint32_Buffer;
    #ifdef ECHO_TO_SERIAL
    Serial.print(F(" NTC ohms: "));Serial.println(NTC_NewReading); Serial.flush();
    #endif  
#endif // #ifdef ReadNTC_6ref7ntc

#ifdef ReadLDR_onD9       //==============================================
 LDR_NewReading = ReadD9riseTimeOnD8(); //elapsedTime Sensor
 LDR_NewReading = (referenceResistorValue * (uint32_t)LDR_NewReading) / uint32_Buffer; // max: 655,350,000
    #ifdef ECHO_TO_SERIAL
    Serial.print(F(" LDR ohms: "));Serial.println(LDR_NewReading); 
    Serial.flush();
    #endif  
#endif //#ifdef ReadLDR_onD9

#ifdef BMP280_Address  //========================================================================== 
bmp280.startForcedConversion();                  // time needed here depends on oversampling settings
bitClear(DDRB,5);bitSet(PORTB,5);                // D13 indicator LED adds ~50uA current
LowPower.powerDown(SLEEP_60MS, ADC_ON, BOD_OFF); // uses watchdog timer
bitClear(PORTB,5);                               // D13 PULLUP OFF
bmp280.getCurrentMeasurements(Bmp280_Temp_degC,Bmp280_Pr_mBar,Bmp280_altitude);
#endif //BMP280_Address

#ifdef bh1750_Address //=========================================================================== 
bh1750.start (); //triggers a new sensor reading
do{              //sleep till new reading is ready
LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);    // uses watchdog timer
}while(bh1750.hasValue() == false); 
// L-Resolution Mode Measurement Time 16-24 msec 
// H-Resolution Mode Measurement Time 120-180 ms depending on light levels
// LOW MTreg:31  resolution lux:7.4, 121557 is highest lux
lux_BH1750 = bh1750.getLux();                    // request the reading 
if(lux_BH1750<1.0){lux_BH1750=1.0;}              // zero catch for EOF check later which uses zero value
BH1750sensor.lux_Integer =uint32_t(lux_BH1750);  // reading can reach 120,000
#endif //bh1750_Address

//===================================================================================================
//====================================================================================================
// This is for serial output for debugging only  - comment out ECHO_TO_SERIAL skips this
//====================================================================================================
//====================================================================================================
#ifdef ECHO_TO_SERIAL
    Serial.print(F("(Forward) EEprom pointer: "));Serial.print(forwardEEmemPointr);  
    Serial.print(F(" , (Backward) EEprom pointer: "));Serial.println(opdEEprMemPointer); 
    sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", t_year, t_month, t_day, t_hour, t_minute);
    Serial.print(CycleTimeStamp); // sprintf ref  http://www.perlmonks.org/?node_id=20519
    Serial.print(F(", RTC TempC: "));Serial.print(rtc_TEMP_degC,2);
    Serial.print(F(", LOW Bat(mV)= "));Serial.println(LowestBattery);

  #ifdef BMP280_Address
    Serial.print(F(", BMP280 Temp:"));Serial.print(Bmp280_Temp_degC,2);
    Serial.print(F(", BMP280 Pressure:"));Serial.println(Bmp280_Pr_mBar,1);
  #endif

  #ifdef bh1750_Address
    Serial.print(F(", BH1750 Lux: "));Serial.print(BH1750sensor.lux_Integer);
  #endif
  
    Serial.println();
    Serial.flush();
#endif  //ENDIF FOR ECHO TO SERIAL

// heartbeat pip of the LED at the end of the senor readings // 30-50uA
  if(blinkLED){
    #ifdef LED_GndGB_A0-A2
      bitClear(DDRC,1);bitSet(PORTC,1); //GreenLED = pinMode(A1,INPUT_PULLUP);
    #else  //default red led on D13
      bitClear(DDRB,5);bitSet(PORTB,5); //same as pinMode(d13,INPUT_PULLUP);
    #endif
  LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_OFF);
    bitClear(PORTC,1); // A1 indicator LED pullup Off
    bitClear(PORTB,5); // pin13 indicator LED pullup Off
  }//if(blinkLED)
    
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ADD NEW SENSOR READINGS to sensorDataBuffer
// NOTE: bytes added per cycle MUST divide evenly into sizeof(sensorDataBuffer)
// AND: the number of bytes you save must match the #define sensorBytesPerRecord
// ORDER of sensors listed here must match that in sendSensorData2Serial!
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// in each cycle you can add 1, 2, 4 or 8 bytes to sensorDataBuffer - not 3 , not 5, not 7 etc. 
// or you mess up the powers of 2 math & eeprom page boundaries
// ALSO we are using zero as our end of file markers....so implement traps on first byte
  
#ifdef RTC_Temperature  //===========================================================
integerBuffer = round((rtc_TEMP_Raw+40)/4);  // NOTE: this 1-byte encoding is restricted to range:
if(integerBuffer>255){integerBuffer=255;}    // temps above 51.5 clipped
if(integerBuffer<1){integerBuffer=1;}        // min value of 1 to preserve EOFcheck, this sets lower cutoff to -12.25C!
sensorDataBuffer[sensorArrayPointer] = integerBuffer;
sensorArrayPointer = sensorArrayPointer+1;
#endif //RTC_Temperature

#ifdef ReadNTC_6ref7ntc  //=========================================================
byteBuffer1= lowByte(NTC_NewReading);
if(byteBuffer1==0){byteBuffer1=1;} //to preserve zero EOF indicator in 'empty' EEprom space
sensorDataBuffer[sensorArrayPointer] = byteBuffer1;
  sensorArrayPointer = sensorArrayPointer+1;
sensorDataBuffer[sensorArrayPointer] = highByte(NTC_NewReading);
  sensorArrayPointer = sensorArrayPointer+1;
#endif

#ifdef ReadLDR_onD9     //===========================================================
  sensorDataBuffer[sensorArrayPointer] = lowByte(LDR_NewReading);
  sensorArrayPointer = sensorArrayPointer+1;
  sensorDataBuffer[sensorArrayPointer] = highByte(LDR_NewReading);
  sensorArrayPointer = sensorArrayPointer+1;
#endif

#ifdef BMP280_Address    //===========================================================
floatBuffer = Bmp280_Temp_degC*100.00;
integerBuffer = (int)floatBuffer;  
byteBuffer1 = lowByte(integerBuffer);
if(byteBuffer1==0){byteBuffer1=1;} //zero trap for zero/EOF check in sendSensorData2Serial
  sensorDataBuffer[sensorArrayPointer] = byteBuffer1;
sensorArrayPointer = sensorArrayPointer+1;
  sensorDataBuffer[sensorArrayPointer] = highByte(integerBuffer);
sensorArrayPointer = sensorArrayPointer+1;
  
floatBuffer = Bmp280_Pr_mBar*10.0;
integerBuffer = (int)floatBuffer;
sensorDataBuffer[sensorArrayPointer] = lowByte(integerBuffer);
  sensorArrayPointer = sensorArrayPointer+1;
sensorDataBuffer[sensorArrayPointer] = highByte(integerBuffer);
  sensorArrayPointer = sensorArrayPointer+1;
#endif //BMP280_Address

#ifdef bh1750_Address  //===========================================================
//4-byte union encoding Note: 'unsigned long' is Little Endian on arduino
//if BH1750sensor.lux_Integer = 0x5CE3B975 then BH1750sensor.lux_byteArray[0] = 0x75, long_num.bytes[1] = 0xB9, and so on.
if(BH1750sensor.lux_byteArray[0]==0){BH1750sensor.lux_byteArray[0]=1;} 
//zero trap preserves EOF zero check in sendSensorData2Serial()

sensorDataBuffer[sensorArrayPointer] = BH1750sensor.lux_byteArray[0];sensorArrayPointer++;
sensorDataBuffer[sensorArrayPointer] = BH1750sensor.lux_byteArray[1];sensorArrayPointer++;
sensorDataBuffer[sensorArrayPointer] = BH1750sensor.lux_byteArray[2];sensorArrayPointer++;
sensorDataBuffer[sensorArrayPointer] = BH1750sensor.lux_byteArray[3];sensorArrayPointer++;
#endif //bh1750_Address

//=================== oncePerDayEvents=======================
// OPD data gets stored IN REVERSE ORDER to share same EEprom storing sensorDataBuffer
// NOTE: data bytes added to the buffer per cycle MUST DIVIDE EVENLY into sizeof(opdDataBuffer)
// AND: the (number) of bytes you store in OPD must match #define opdBytesPerRecord at start of program

if ((SaveBatteryEveryCycle && newBatteryReadReady) || (midnightRollover)){

  // Note: adding 'half the power of 2' (8 in this case) forces 'round-up' after shift
  // https://forum.arduino.cc/t/fast-divide-of-signed-number-using-shift/563478/13 

  // 1-byte encoding range to 4080 mv above the BIAS value of 1700 (so range = 1700-5780mv)
  // the smallest delta resolvable with internal Aref trick is 11 millivolts
  // but that is further reduced to 16mV per bit by the 1-byte indexed encoding

  integerBuffer = (LowestBattery-1700)>>4;         // >>4 same as /16
  if(integerBuffer<1){integerBuffer=1;} // Zero Trap to preserve END of DATA during sendOPDreadings2Serial
  opdArrayPointer=opdArrayPointer-1;               // incrementing backwards here for stack/heap method
  opdDataBuffer[opdArrayPointer] = integerBuffer;  // battery reading stored in OPD buffer array
  opdEEprMemPointer=opdEEprMemPointer-1;           // decrement eeprom pointer for every byte added to opdDataBuffer[opdArrayPointer]

  integerBuffer = (HighestBattery-1700)>>4;
  opdArrayPointer=opdArrayPointer-1;               // incrementing backwards here for stack/heap method
  opdDataBuffer[opdArrayPointer] = integerBuffer;  // battery reading stored in OPD buffer array
  opdEEprMemPointer=opdEEprMemPointer-1;           // decrement eeprom pointer for every byte added to opdDataBuffer[opdArrayPointer]

  if (SaveBatteryEveryCycle){
     newBatteryReadReady=false;  // flag sets True only when SENSORdata is written to eeprom
  }
  if (midnightRollover){ 
     midnightRollover=false; newBatteryReadReady=false;
     HighestBattery = 0;  // forces reset @ next readbattery
     LowestBattery = 5700;// forces reset @ next EEsave event
                          // Note: Hi/Lo resets once per day
  }

  if (opdArrayPointer ==0) { // the Buffer is full - so transfer that data to the eeprom

  //WAIT FOR BATTERY RECOVERY B4 EEsave events
      byteBuffer1 = 0; integerBuffer = currentBatteryRead; // taken at the beginning of *this* sampling cycle
      do{ // SLEEP_15MS embedded in readBattery
        currentBatteryRead = readBattery(); 
       byteBuffer1++; 
        if(byteBuffer1>8){break;}   // limits max cycles to 9 x15ms = ~135ms
      }while(currentBatteryRead<integerBuffer);

    Write_i2c_eeprom_array(opdEEpromI2Caddr,opdEEprMemPointer,opdDataBuffer,sizeof(opdDataBuffer)); 
    opdArrayPointer = sizeof(opdDataBuffer); //NOTE ArrayPtr decremented -1 before 1st use - resets to 'high' value because pointer increments backward 
    #ifdef ECHO_TO_SERIAL
    Serial.println(F("OPD data moved to EEprom"));Serial.println();Serial.flush();
    #endif
    
    //if opdEEprMemPointer reaches zero then opd eeprom is full - shutdown
    if (opdEEprMemPointer == 0){error();}   
    //opdEEprMemPointer also gets checked against forwardEEmemPointr in main loop to flag when eeprom is full -IF- both are stored in the same eeprom
  } //terminates if(opdArrayPointer==0) 
}//terminates if ((SaveBatteryEveryCycle && newBatteryReadReady) || (midnightRollover)){

//===============================================================================
// check if sensorDataBuffer was filled on previous cycle
// -> if so move the old data to EEprom before before adding new readings
//===============================================================================
// OPD save can only occur on the following cycle so dual eeprom events dont occur at the same time
if (sensorArrayPointer == sizeof(sensorDataBuffer)) { 

//WAIT FOR BATTERY RECOVERY B4 EEsave events
      byteBuffer1 = 0; integerBuffer = currentBatteryRead; // taken at the beginning of *this* sampling cycle
      do{ // SLEEP_15MS embedded in readBattery
       currentBatteryRead = readBattery(); //15msec sleep embedded in readbat
       byteBuffer1++; 
       if(byteBuffer1>8){break;} // limits max cycles to 9 x15ms = ~135ms
        }while(currentBatteryRead<integerBuffer);
 
  Write_i2c_eeprom_array(sensorEEpromI2Caddr,((forwardEEmemPointr+sensorBytesPerRecord) - sizeof(sensorDataBuffer)),sensorDataBuffer,sizeof(sensorDataBuffer));
  sensorArrayPointer = 0; // readings buffered in sizeof(sensorDataBuffer)so need to shift forwardEEmemPointr BACK before save to EEp
  //battery voltage is checked every time Write_i2c_eeprom_array gets called
  //setting newBatteryReadReady=true; allows OPD data entry on FOLLOWING cycle
  #ifdef ECHO_TO_SERIAL
    Serial.println(F("Sensor data moved to EEprom"));Serial.println();Serial.flush();
  #endif
}// if (sensorArrayPointer == sizeof(sensorDataBuffer))

if (LowestBattery <= systemShutdownVoltage){
  error();
  } 

sleepNwait4RTC(); // sleep till the next alarm

//================================================================================================
} //TERMINATOR for "cycle of samples" LOOP 
//================================================================================================

// if you've reached this point then the external eeprom memory storing sensor readings is now full!
   error(); //so logger goes into shutdown down

//==========================================================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================
}//======================== END OF MAIN LOOP================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================
//==========================================================================================

//======================================================
void sleepNwait4RTC() { 
//======================================================  

    //unusedPins2InputPullup(); //OR
  unusedPins2OutputLow();  //pin states preserved through sleep
  bitClear(DDRD,2);   //D2 input
  bitClear(PORTD,2);  //D2 pullup off - not needed with hardware pullup on RTC module
  noInterrupts();
  bitSet(EIFR,INTF0); // clear interrupt flag before attachInterrupt(1,isr,xxxx)
  attachInterrupt(0,rtcAlarmTrigger,FALLING); //RTC alarm connected to pin D2
  interrupts ();
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF); // ADC_ON preserves current ADC status(it's already OFF...)
  //HERE AFTER WAKING  // powerDown & powerSave add 16,000 clock cycles (~2msec) to wakeup time at ~250uA
  
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_STATUS_REG);
  Wire.write(0);  // turns Off (both) alarms
  Wire.endTransmission();
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
  rtc_INT0_Flag = false;  //clear the flag we use to indicate the RTC alarm occurred
    
  unusedPins2OutputLow(); 
  
}  //terminator for sleepNwait4RTC

void rtcAlarmTrigger() { 
  detachInterrupt(0);
  rtc_INT0_Flag = true;   //flag used in SETUP
}

//============================================================================
void unusedPins2OutputLow(){
//============================================================================

// does not affect D2 alarm interupt line is INPUT

//SET all pins low      // with &= causes zeros to set
  PORTB &= B11000000;   // 13..8 LOW / pullups OFF(0)
  PORTD &= B00001111;   // 7..4 LOW / pullups OFF 
  PORTC &= B1110000;    // A3..A0 LOW / pullups OFF   A4/5=I2C bus

//SET pins OUTPUT       // with |= only the ones set
  DDRB |=B00011111;     // D12-8 OUTPUT(1) //d13 stays input
  DDRD |=B11110000;     // D7-4 OUTPUT          

//leave D2 alone for RTC D2 pulled up by rtc module
//SKIP: DDRC |=B00001111; // A3..A0 OUTPUT // A4/5=I2C bus

//if no interrupt source connected on D3:
  PORTD &= B11110111;     // 3 LOW
  DDRD |=B00001000;       // 3 OUTPUT

#ifndef ECHO_TO_SERIAL   // don't change UART pins in ECHO_TO_SERIAL debug mode
  PORTD &= B11111100;    // 1-0 LOW
  DDRD  |= B00000011;    // 1-0 OUTPUT
#endif
}

//============================================================================
void sendSensorData2Serial(boolean convertDataFlag){ //only runs at startup with serial connection
//============================================================================
// NOTE: the ORDER of sensors listed in sendSensorData2Serial MUST MATCH
// the order in which you ADDED those NEW SENSOR READINGS to sensorDataBuffer 

  SampleIntervalMinutes = EEPROM.read(1008); // uint8_t
  SampleIntervalSeconds = EEPROM.read(1009); // uint8_t //not implemented here
  // note you may get a timestamp calculation error on readback if the internal eeprom 
  // has just been 'freshly erased' as this also deletes the previously set sampling interval

Serial.print(F("UnixTime,"));
#ifdef RTC_Temperature
  Serial.print(F("RTC T°C,"));
#endif
#ifdef ReadNTC_6ref7ntc
  Serial.print(F("NTC[Ω],"));
#endif
#ifdef ReadLDR_onD9
  Serial.print(F("LDR[Ω],"));
#endif
#ifdef BMP280_Address
  Serial.print(F("BMP[T°C],Pr[mbar],"));
#endif
#ifdef bh1750_Address
  Serial.print(F("BH1750lux,"));
#endif
 Serial.println();Serial.flush();

    //unixtime index value stored in penultimate four bytes of internal eeprom
    EEPROM.get(1016,utime.EE_byteArray); // note bytearray unioned w utime.cyleTimeStart
    uint32_t unix_timeStamp = utime.cyleTimeStart;
    uint16_t RecMemoryPointer;
    
    uint16_t secondsPerSampleInterval;
    if (SampleIntervalMinutes==0){  // sub-minute alarms for accelerated run testing
        secondsPerSampleInterval = SampleIntervalSeconds;
        }else{  //  normal minute based alarms:
        secondsPerSampleInterval = 60UL*SampleIntervalMinutes;
        }
        
for (uint32_t i = 0; i <= (sensorEEbytesOfStorage-sensorBytesPerRecord); i+=sensorBytesPerRecord) { //increment by # of bytes PER RECORD in eeprom

  RecMemoryPointer=i;
  byteBuffer1 = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer); //zeros first byte read are EOF / end of data markers
  if(byteBuffer1==0 & convertDataFlag){break;}

  if (!convertDataFlag){ //NOTE: !convertDataFlag flushes the entire eeprom contents including OPD data at end
        for (uint8_t j = 0; j < sensorBytesPerRecord; j++) {
        byteBuffer1 = i2c_eeprom_read_byte(sensorEEpromI2Caddr,(RecMemoryPointer+j));
        Serial.print(byteBuffer1); Serial.print(F(",")); // output raw bytes as read from eeprom:
        }     
  } else { //convertDataFlag is true then readback eeprom bytes & convert to human readable sensor records

  Serial.print(unix_timeStamp);Serial.print(",");
  unix_timeStamp += secondsPerSampleInterval; //increment for the NEXT record after sending

// order here must match the order in which you loaded the sensor array
#ifdef RTC_Temperature                      // RTC temperature record: low side cutoff at 1 for minimum reading of -12.25C
     integerBuffer = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;           // read previously for EOF check
     integerBuffer=(integerBuffer*4)-40; 
     floatBuffer  =(integerBuffer*0.0625)-10.0;
     Serial.print(floatBuffer,2);Serial.print(F(",")); 
#endif  //#ifdef RTC_Temperature 

#ifdef ReadNTC_6ref7ntc  //NTC_NewReading is only stored as raw two data bytes, low byte first
      byteBuffer1 = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;//low byte
      NTC_NewReading = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;//hi byte
      NTC_NewReading = (NTC_NewReading << 8) | byteBuffer1; //reconstitutes the rawNTC reading
      Serial.print(NTC_NewReading);Serial.print(F(","));
      
#endif // #ifdef ReadNTC_6ref7ntc

#ifdef ReadLDR_onD9  //embedded inside readNTCthermistor() function
      byteBuffer1 = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;//low byte
      LDR_NewReading = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;//hi byte
      LDR_NewReading = (LDR_NewReading << 8) | byteBuffer1;
      Serial.print(LDR_NewReading);Serial.print(F(","));    
#endif
      
#ifdef BMP280_Address
      // Bmp280_Temp_degC, low byte first
      byteBuffer1 = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;//low byte
      integerBuffer = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;//hi byte
      integerBuffer = integerBuffer <<8 | byteBuffer1;
      Serial.print((float)(integerBuffer)/100.0,2);Serial.print(",");
      // Bmp280_Pr_mBar, low byte first
      byteBuffer1 = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;//low byte
      integerBuffer = i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;//hi byte
      integerBuffer = integerBuffer <<8 | byteBuffer1;
      Serial.print((float)(integerBuffer)/10.0,1);
#endif

#ifdef bh1750_Address  // 4 byte Long Integer concatenated via union
BH1750sensor.lux_byteArray[0]= i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;
BH1750sensor.lux_byteArray[1]= i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;
BH1750sensor.lux_byteArray[2]= i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;
BH1750sensor.lux_byteArray[3]= i2c_eeprom_read_byte(sensorEEpromI2Caddr,RecMemoryPointer);RecMemoryPointer++;
Serial.print(BH1750sensor.lux_Integer);Serial.print(F(","));
#endif

          } //if(convertDataFlag)
      Serial.println();
      } // terminator: for (int i = 16; i < sensorEEbytesOfStorage; i++)
  Serial.flush();
}  // terminator: void sendSensorData2Serial()

//=================================================================================
void sendOPDreadings2Serial() {     // data saved when midnight rollover flags true
//=================================================================================

   uint32_t unix_timeStamp;
   Serial.print(F("UnixTime,LObat,HIbat[mV],")); //change this to suit the data save in oncePerDayEvents()  
   if (SaveBatteryEveryCycle){  //using the same TimeStamp index as the sensor records
   Serial.print(F("(Saves Bat Every Cycle)"));
   }
   Serial.println();Serial.flush();

   if (SaveBatteryEveryCycle){
   EEPROM.get(1016,utime.EE_byteArray); // the same start TimeStamp as the sensor records
   // HOWEVER the first battery read does not happen until AFTER sensorArray is full & written to eeprom (1 full cycle)
   }else{ //otherwise normal onceperday 'midnight' time index value stored in 2nd to last four bytes of 328p internal EEprom
   EEPROM.get(1020,utime.EE_byteArray); // filling utime.EE_byteArray populates utime.cyleTimeStart (via union)
   } // note utime.cyleTimeStart is unioned with utime.EE_byteArray

    uint32_t secondsBtwEachOPDrecord;
    if (SaveBatteryEveryCycle){    //change the TimeStamp calc to the same one used for sensor records   
        if (SampleIntervalMinutes==0){  // short second level alarms  // TimeStart from  EEPROM.get(1016 same as samples
          // note: -1 sample interval offset - the 1st sensor record occurs at zero time  after no intervals
              secondsBtwEachOPDrecord = SampleIntervalSeconds * (sizeof(sensorDataBuffer)/sensorBytesPerRecord); //= time to fill & write sensor buffer
              unix_timeStamp = (utime.cyleTimeStart-SampleIntervalSeconds) + secondsBtwEachOPDrecord;
            } else {                    // we are using normal minute - based alarms:
              secondsBtwEachOPDrecord = 60UL*SampleIntervalMinutes * (sizeof(sensorDataBuffer)/sensorBytesPerRecord); 
              unix_timeStamp = (utime.cyleTimeStart-(60UL*SampleIntervalMinutes)) + secondsBtwEachOPDrecord;
            }  //SampleIntervalMinutes==0  
        // note: -1 sample interval offset:
        // an OPD record gets generated ONLY when newBatteryReadReady - which happens every sensorbuffer EEsave
        // the first OPD battery read does not happen until AFTER sensorArray has filled (1 cycle)
        // AND OPD doesnt get buffered till the FOLLOWING cycle (so backtrack one sample interval)
        // this is because OPD comes before save sensor data avoiding double write events 
                
       }else{   //OPD saved on midnight rollovers ONLY : 86400 seconds is time increment for once per day events
            unix_timeStamp = utime.cyleTimeStart;   // from  EEPROM.get(1020, =not same as sample start time
            secondsBtwEachOPDrecord = 86400;        //long multiplication ~72 clock cycles
       }//if (SaveBatteryEveryCycle)
       
  //note uint16_t can't hold >65535 // loop increments backwards 
  for (int32_t i = (opdEEbytesOfStorage-1) ; i >= opdBytesPerRecord; i-=opdBytesPerRecord) {

  byteBuffer1 = i2c_eeprom_read_byte(opdEEpromI2Caddr,i);  // normal battery reading can not return zero
  if(byteBuffer1==0){break;} // stop if zero = EOF is found

  Serial.print(unix_timeStamp);Serial.print(","); //note that timestamps do not print with 'raw' option selected 
  unix_timeStamp = unix_timeStamp + secondsBtwEachOPDrecord; 

// LOW battery reading encoded in 1st byte of record in eeprom
    uint16_Buffer = i2c_eeprom_read_byte(opdEEpromI2Caddr,i);
    uint16_Buffer =(uint16_Buffer<<4)+1700; // <<4 same as *16
    Serial.print(uint16_Buffer);Serial.print(F(",")); // battery reading stored in 1st byte
// HIGH battery reading in 2nd //16mv resolution
    uint16_Buffer = i2c_eeprom_read_byte(opdEEpromI2Caddr,i-1);
    uint16_Buffer =(uint16_Buffer<<4)+1700;
    Serial.print(uint16_Buffer);Serial.print(F(",")); // battery reading stored in 1st byte

  Serial.println();
  } // terminator: for (int32_t i = (opdEEbytesOfStorage-1) ; i >= opdBytesPerRecord;
Serial.flush();
} // terminator: sendOPDreadings2Serial()

//=================================================
//----------Voltage monitoring functions ----------
//=================================================
// see http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// also https://code.google.com/p/tinkerit/wiki/SecretVoltmeter

uint16_t readBattery() 
{
  power_adc_enable(); 
  ADCSRA = set_ADCSRA; // start ADC clock
  ADMUX = set_ADMUX;   // set to 1.1vref & internal channel
  // for 'normal' readings set ADC clock back to default speed
  bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,1);bitWrite(ADCSRA,ADPS0,0); // 64 (default) prescalar @ 8MHz/64 = 125 kHz
  bitSet(ADCSRA,ADSC); while(bit_is_set(ADCSRA,ADSC)); // THROW AWAY READING & sleep for AREF stabilization
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); //leaves ADC_ON: ~110µA during this sleep
  
  int16_t value = readADCLowNoise(true);
  ADCSRA =0; power_adc_disable();
  
  uint16_t result = InternalReferenceConstant / value;  //scale Rail voltage in mV
  if (result < LowestBattery) {LowestBattery = result;}
  if (result > HighestBattery) {HighestBattery = result;} //this is an 'unloaded'reading
  
  if (result < systemShutdownVoltage){
      #ifdef ECHO_TO_SERIAL
      Serial.print(result); Serial.println(F("Battery < shutdown voltage!")); Serial.flush();
      #endif
      error();} 
         
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
  #ifdef ReadNTC_6ref7ntc
  Serial.println((__FlashStringHelper*)NTCcal);
  #endif
  //Serial.println(F("Select “newline” OPTION in Serial Monitor Window:"));
  Serial.flush();

  char charbuffer; // retrieve 255 character deployment description stored internal eeprom at startup
  // retrieve 255 character description stored internal eeprom at startup
  for (uint16_t k = 512; k < 767; k++) {
     charbuffer = EEPROM.read(k);
     Serial.print(charbuffer);
  }
  Serial.println();Serial.flush();
}

//================================
void printMenuOptions(){
//================================
Serial.println(F(" Select action -> type:"));
Serial.println(F("  ['new' & enter] to Download sensor data (up to 3 minutes w 128k)"));
Serial.println(F("  ['raw' & enter] to Download eeprom as RAW bytes [w no timestamp]"));
Serial.println(F("  ['start' & enter] to set up a new logging run"));
Serial.println();Serial.flush();
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

uint8_t i2c_eeprom_read_byte(uint8_t deviceaddress, uint16_t eeaddress ) {
  uint8_t rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((uint8_t)(eeaddress >> 8));    // Address High Byte
  Wire.write((uint8_t)(eeaddress & 0xFF));  // Address Low Byte
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,(uint8_t)1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

// NOTE: this function uses the EEprom current to load the coin cell during voltage read
void Write_i2c_eeprom_array( uint8_t deviceAddress, uint16_t registerAddress_16bit,byte *arrayPointer,uint8_t numOfBytes){ 

// lowering I2C bus speed for slower 4k eeproms but test your eeproms - although many work fine at faster bus speeds.
    if (deviceAddress==opdEEpromI2Caddr){   
       if(opdEEbytesOfStorage==4096){TWBR=32;}
    }
    if (deviceAddress==sensorEEpromI2Caddr){ 
      if(sensorEEbytesOfStorage==4096){TWBR=32;}
    }
    
// THROW AWAY ADC reading to engage the AREF cap, first read takes 20 ADC clock cycles
// 1st reading after re-enable adc is usually a HIGH SPIKE bc aref voltage has not yet risen
    power_adc_enable();   // Aref rise takes ~1msec, while it takes 5msec or more for Aref to fall.
    ADCSRA = set_ADCSRA; ADMUX = set_ADMUX; // NOTE: prescalar was chosen in SETUP!
    bitSet(ADCSRA,ADSC);  // trigger ADC reading
 
// 4k AT24c32 writes for 10ms @3mA, but newer eeproms can take only only 5ms @3mA
    Wire.beginTransmission(deviceAddress);
    Wire.write((byte)((registerAddress_16bit) >> 8));   // send the MSB of the address
    Wire.write((byte)((registerAddress_16bit) & 0xFF)); // send the LSB of the address
       for (int i=0; i<numOfBytes; i++) {
          Wire.write((byte)arrayPointer[i]);
       }      
    Wire.endTransmission();

// slow processor to 1MHZ for low current
    noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_8;interrupts();       
    // NOTE: At 1Mhz system clock, each ADC read takes 8x longer than the default 0.1mSec @ 8MHz (with default 64 prescalar)
    // WARNING: Be careful about what you do immediately AFTER changing the CPU speed with CLKPR!
    // some peripherals need a couple of clock cycles before they can be addressed after CLKPR or THEY WILL HANG
    power_timer0_disable(); power_twi_disable(); //more current reduction

// Measuring the internal 1.1v using Vcc as the reference with the int ref voltage as the INPUT
// INVERSE relationship! as rail voltage falls the ADC reading increases.
// An ADC read takes 13 ADC clock cycles, so default is about 9615 Hz (or 0.104 milliseconds per reading).
// current: bitWrite(ADCSRA,ADPS2,1);bitWrite(ADCSRA,ADPS1,0);bitWrite(ADCSRA,ADPS0,1); // 32 ADC prescalar = 2x normal the speed of the ADC
// clock is 8x slower, and adc is 2x faster:  result is 416 uS per ADC reading
// we will ignore the first 8 readings to give the AREF cap 3.328msec to stabilize 

  uint16_t NEWadcReading = 0; uint16_t LOWESTpoint = 0; uint8_t loopCounter = 0;
  bitSet(ADCSRA,ADIE);  // generate interrupt when the ADC conversion is done
  set_sleep_mode(SLEEP_MODE_ADC); //stops CPU to lower current draw but allows ADC peripheral
     
     do{ //ADC clock *2 & System clock/8 = each read takes 0.416 msec
            bitSet(ADCSRA,ADSC);  //trigger a new reading
            sleep_enable(); // Enable Noise Reduction Sleep Mode
                do{ 
                    interrupts();   // always use sei immediately followed by sleep_cpu 
                    sleep_cpu();    // + 6 clock cycles to wake from SLEEP_MODE_ADC
                    noInterrupts(); // Check conversion status done w interrupts disabled to avoid race condition
                  }while (bit_is_set(ADCSRA,ADSC)); // the ADC clears the bit when conversion is complete 
            sleep_disable();
            interrupts(); 
            loopCounter++;
              if(loopCounter>8){ // ignore the first 8 readings = 3.328msec Aref Rise time
                byteBuffer1= ADCL; byteBuffer2 = ADCH;// read low first
                NEWadcReading= ((byteBuffer2 << 8)|byteBuffer1); //yes relationship is inverse!
                if(NEWadcReading>LOWESTpoint){LOWESTpoint=NEWadcReading;}  
              }
       }while(loopCounter<24); // number of ADC reads should match your expected eeprom writing time
       // use 24x416uS for 4K because 10msec needed for save by slow 4K eeproms
       // but faster 64k eeproms only need about 6mSec for EE saving limit could be 14

  bitClear(ADCSRA,ADIE);          // turn off ADC interrupts
  ADCSRA=0; power_adc_disable();  // turn off ADC
    noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); // return to 8MHz before sleep
    LowPower.powerDown(SLEEP_120MS, ADC_ON, BOD_OFF);                // battery recovery time                                                                 // sleep_disable(); taken care of in LowPower.powerDown
  power_timer0_enable();   // Timer0 required for I2C, delay, serial print, etc
  power_twi_enable();      // enable I2C bus
  
  currentBatteryRead = InternalReferenceConstant / LOWESTpoint; // LONG calc forced by ref constant = 630 cpu clocks! do this after 8MHz
  if (currentBatteryRead < LowestBattery) {LowestBattery = currentBatteryRead;}
  TWBR=2; //back to 400KHz bus for RTC - only 4k eeprom needs slower I2C bus
  newBatteryReadReady=true; // this flag only gets used if SaveBatteryEveryCycle = true;
} // terminates Write_i2c_eeprom_array


#if defined(ReadLDR_onD9) || defined(ReadNTC_6ref7ntc)
//=============================================
// ATOMIC versions of each pin read with D8 ICU
//=============================================
// Based on Nick Gammon's frequency counter at https://www.gammon.com.au/forum/?id=11504
// Adapted by Edward Mallon for ratiometric reading of resistive sensors with ICU on D8. For a details see 
// https://thecavepearlproject.org/2019/03/25/using-arduinos-input-capture-unit-for-high-resolution-sensor-readings/
// NewSensorReading =(elapsedTimeSensor * referenceResistorValue) / elapsedTimeReff;
// this calculation works with [104] caps & uint32_t variables HOWEVER
// With 105 capacitors must change read functions & cast this calculation to higher bit depth
// NTC_NewReading=((uint64_t)elapsedTimeSensor * (uint64_t)referenceResistorValue) / elapsedTimeReff;

void ConditionCapacitorOnD8(){
  // Charge & discharge [104] cap through 300ohm resistor on D8
  // cycles the capacitor to bring it to a 'standardised' residual level/condition
  //AND floats all pins with resistor/sensors connected to that common capacitor 

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // must come after disabling ADC
  power_timer1_enable();                  // need Timer1 to clock the change on the D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(DDRB,1);bitClear(PORTB,1);     //D9 [LDR] to INPUT(0) & LOW(0) 
  DDRD &= B00011111;PORTD &= B00011111;   //D6 [ref],D7[NTC],D8[300Ω] to INPUT(0) & LOW(0)
  bitSet(DDRB,0); //pinMode(8,OUTPUT)

  set_sleep_mode(SLEEP_MODE_IDLE);// this mode leaves Timer1 running @<1mA draw but wakes in 6 clock cycles
  prepareForInterrupts(); // sets triggered=false
  noInterrupts ();  sleep_enable();
  bitSet(PORTB,0); //digitalWrite(8,HIGH);
  do{
    interrupts(); sleep_cpu(); noInterrupts(); 
    }while(!triggered); // nothing is recorded during this initial capacitor charge up (takes about 1msec through 36k pullup)
  interrupts ();
  // sleep_disable(); inside ISR (TIMER1_CAPT_vect)
  
  // sampling cap is now at its HIGH trigger point -> now discharge the cap through 300Ω on D8
  bitClear(PORTB,0);  // digitalWrite(8,LOW); 
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); // + extra 2msec for osc start! //ADC_ON leaves the already sleeping ACD alone as is
  // NOTE: 15MS is overkill:  5T with 300ohm&105(1uF) is 1.5 msec, with 300Ω&104(100nF) 5RC is only 0.15ms 
  bitClear(DDRB,0);  // pinMode(8,INPUT); 
  // could insert the timer2 delays here from void Write_i2c_eeprom_array 
  // to make this dischage faster? but that works in sleep mode IDLE?

    //re-enable timers
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
    #ifdef ECHO_TO_SERIAL
      power_usart0_enable();   
    #endif
    
  }
  
uint16_t ReadD6riseTimeOnD8(){
//=========================================================================================
// 10k ref D6, NTC  on D7, 300Ω on D8, {CDS cell on D9}
//=========================================================================================
  noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //this function must run a max clock speed

  //uint16_t elapsedTime=0;

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // must come after disabling ADC
  power_timer1_enable();                  // need Timer1 to clock the change on the D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(DDRB,1);bitClear(PORTB,1);     //D9 [LDR] to INPUT(0) & LOW(0) 
  DDRD &= B00011111;PORTD &= B00011111;   //D6 [ref],D7[NTC],D8[300Ω] to INPUT(0) & LOW(0)

//========== read resistor RISING on D6 ===========
  set_sleep_mode(SLEEP_MODE_IDLE);
  prepareForInterrupts();
  noInterrupts(); sleep_enable();
  TCNT1 = 0;                    // reset Timer 1 counter value to 0
  bitSet(DDRD,6);               // D6 OUTPUT
  bitSet(PORTD,6);              // D6 HIGH -> now charging the cap through 10k ref
  do{
  interrupts(); sleep_cpu(); noInterrupts();
  }while(!triggered);         // trapped here till TIMER1_CAPT_vect changes triggered to true

  //elapsedTime = timer1CounterValue; // capped in ISR (TIMER1_OVF_vect) at 65534
  interrupts ();                // can't use I2C bus or powerDown without interrupts...
  // sleep_disable() inside ISR (TIMER1_CAPT_vect)
  
    bitClear(DDRD,6); bitClear(PORTD,6); // D6 INPUT & LOW -> stops the capacitor charge
//  discharge the capacitor through 300Ω on D8
    bitSet(DDRB,0);bitClear(PORTB,0); //D8 OUTPUT LOW
      LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
    bitClear(DDRB,0);                 //D8 INPUT 
    
//re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
    #ifdef ECHO_TO_SERIAL
      power_usart0_enable();   
      Serial.print(F(" D6: "));Serial.print(timer1CounterValue);Serial.flush(); 
    #endif

return timer1CounterValue;
} // terminates ReadD6riseTime 

uint16_t ReadD7riseTimeOnD8(){
//=========================================================================================
// 10k ref D6, NTC  on D7, 300Ω on D8, {CDS cell on D9}
//=========================================================================================
  noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //this function must run a max clock speed

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // must come after disabling ADC
  power_timer1_enable();                  // need Timer1 to clock the change on the D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(DDRB,1);bitClear(PORTB,1);     //D9 [LDR] to INPUT(0) & LOW(0) 
  DDRD &= B00011111;PORTD &= B00011111;   //D6 [ref],D7[NTC],D8[300Ω] to INPUT(0) & LOW(0)
  
//=== read the NTC thermistor(?) RISING on D7 ===========
  set_sleep_mode (SLEEP_MODE_IDLE); 
  prepareForInterrupts(); 
  noInterrupts(); sleep_enable(); TCNT1 = 0; // reset Timer 1
  bitSet(DDRD,7); bitSet(PORTD,7);  // Pin D7 OUTPUT & HIGH  //now charging the cap through NTC thermistor
    do{
    interrupts(); sleep_cpu(); noInterrupts();
    }while(!triggered);
    interrupts ();    // cant use I2C bus without interrupts!
    // sleep_disable() inside ISR (TIMER1_CAPT_vect)
  
    bitClear(DDRD,7);bitClear(PORTD,7);   //  D7 to INPUT & LOW
    bitSet(DDRB,0);bitClear(PORTB,0);     //  D8 OUTPUT LOW // discharge the cap through D8
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF);
    bitClear(DDRB,0);                     //  D8 INPUT 

//re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
    #ifdef ECHO_TO_SERIAL
      power_usart0_enable();   
      Serial.print(F(" D7: "));Serial.print(timer1CounterValue);Serial.flush(); 
    #endif
    
  return timer1CounterValue;
} // terminates uint32_t ReadD7riseTime 

uint16_t ReadD9riseTimeOnD8(){
  noInterrupts();CLKPR=bit(CLKPCE);CLKPR=clock_div_1;interrupts(); //run @ max clock speed
  //uint16_t elapsedTime=0;

  bitSet(ACSR,ACD);                       // Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one.
  ADCSRA = 0; SPCR = 0;                   // Disable ADC & SPI
  power_all_disable();                    // AFTER disabling ADC!
  power_timer1_enable();                  // ICU uses Timer1 to clock change on D8 pin

  //MUST float ALL pins with resistor/sensors connected to the common capacitor
  //or you wont be able to read the others!
  bitClear(DDRB,1);bitClear(PORTB,1);     //D9 [LDR] to INPUT(0)LOW(0) 
  DDRD &= B00011111;PORTD &= B00011111;   //D6 [ref],D7[NTC],D8[300Ω] to INPUT(0)LOW(0)

//=== read the D9 resistance RISING on D8 ===========
  set_sleep_mode (SLEEP_MODE_IDLE);
  prepareForInterrupts();
  noInterrupts(); sleep_enable(); TCNT1 = 0;
  bitSet(DDRB,1);            //D9 OUTPUT 
  bitSet(PORTB,1);           //D9 HIGH
     do{
     interrupts(); sleep_cpu(); noInterrupts();
     }while(!triggered);
  //elapsedTime= timer1CounterValue;
  interrupts(); //sleep_disable(); // redundant:  sleep_disable() in ISR (TIMER1_CAPT_vect)

  bitClear(DDRB,1); bitClear(PORTB,1);  //  D9 INPUT & LOW // stops capacitor charging
  bitSet(DDRB,0);bitClear(PORTB,0);     //  D8 OUTPUT LOW  // discharge the capacitor through D8
  LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_OFF); 
  bitClear(DDRB,0);                     //  D8 INPUT 

  //re-enable timers after Dpin read:
    power_timer1_disable();    // no longer need Timer1
    power_timer0_enable();     // MUST RE-ENABLE for any delay/serial statements,etc
    power_twi_enable();        // cant use I2C bus without interrupts enabled
      #ifdef ECHO_TO_SERIAL
      power_usart0_enable();   
      Serial.print(F(" D9: "));Serial.print(timer1CounterValue);Serial.flush(); 
      #endif
    
    return timer1CounterValue;
} // terminates ReadD9riseTimeOnD8 

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
    if (triggered){return;}       // error catch for overflow or multiple triggers(?)
    timer1CounterValue = ICR1;    // transfer value held in Input Capture register
    triggered = true;
    bitClear(TIMSK1,ICIE1);       // disable input capture unit
    bitClear(TIMSK1,TOIE1);       // also disable interrupts on Timer 1 overflow
    sleep_disable();
}  // end of TIMER1_CAPT_vect

// NOTE when powering from a coincell you don't want the sensor readings to take too long
// so here we throttle the system here to only ONE overflow as an error catch with large resistors (like CDS cells in the dark)
// this fails with larger 105 capacitors that need more than 65535 clocks, but works OK with 104 caps
ISR (TIMER1_OVF_vect) {           // timer1 overflows (every 65536 system clock ticks = 122 times a second @8MHz)
    timer1CounterValue = 65534;   // (=8.19msec in SLEEP_MODE_IDLE ~1mA) sometimes I modify this max limit slightly while debugging...
    triggered = true;
    bitClear(TIMSK1,ICIE1);       // disable input capture unit
    bitClear(TIMSK1,TOIE1);       // also disable interrupts on Timer 1 overflow
    sleep_disable();
}

#endif //======================================================
//end of #if defined(ReadLDR_onD9) || defined(ReadNTC_6ref7ntc)
//=============================================================


// =================================================================================
// RTC functions
// =================================================================================
// unixtime functions modified from https://github.com/MrAlvin/RTClib 
// which is a fork JeeLab's fantastic real time clock library for Arduino
// Extracted those functions here because Alvins version has the SAME NAME as 
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
   Serial.print(F("Lowest Battery @"));
   Serial.println(LowestBattery); Serial.flush();
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

//unused pins to input-pullup     // &= causes zeros to set
  DDRB &= B11000000; // pins 13..8 set to zero for input
  DDRD &= B00000011; // pins 7..2 set as inputs
  DDRC &= B11000000; // unused A5..A0 set as inputs// can I set low if disabled?
  #ifndef ECHO_TO_SERIAL // don't mess with USART pins in ECHO_TO_SERIAL debug mode
    DDRD &= B11111100;     // d1&0 set as inputs
    PORTD &= B11111100;    // d1&0 low
  #endif
  
//SET all pins low      // with &= causes zeros to set
  PORTB &= B11000000;   // 13..8 LOW / pullups OFF(0)
  PORTD &= B00000011;   // 7..2 LOW / pullups OFF     // leave D2 for RTC alarm line pullup & possibly D3
  PORTC &= B1000000;    // A5..A0 LOW / pullups OFF   // leave A4/5=I2C bus has hardware pullups?
  
  #ifdef ECHO_TO_SERIAL
    bitClear(DDRB,5);bitSet(PORTB,5); // indicate ERROR STATUS with red LED ONLY if on UART!
  #endif
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF);  //use ADC_ON because we alreay have ADC turned off!
}
