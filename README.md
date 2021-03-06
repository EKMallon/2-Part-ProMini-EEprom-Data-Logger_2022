# 2-Part-ProMini-EEprom-Data-Logger_2022
<img src="https://github.com/EKMallon/The_Cave_Pearl_Project_CURRENT_codebuilds/blob/master/images/CavePearlProjectBanner_130x850px.jpg">

This program supports an ongoing series of DIY 'Classroom Logger' tutorials from Edward Mallon & Dr. Patricia Beddows at the Cave Pearl Project. The idea is to provide a starting point for student projects in environmental monitoring courses and/or thesis level research.<br/>
<br/>The build tutorial that matches this code can be found at:<br/>
https://thecavepearlproject.org/2022/03/09/powering-a-promini-logger-for-one-year-on-a-coin-cell/

---
<img   align="right" width="400" src="https://github.com/EKMallon/2-Part-ProMini-EEprom-Data-Logger_2022/blob/main/images/2-PartEEpromLogger_CavePearlProject_2022.jpg">
This 'low power' 2-module iteration runs the logger from a CR2032 coin cell and uses  EEprom memory to store sensor readings. This necessarily involves several power optimization steps which add significant complexity to the base code (as compared to previous versions) but hopefully everyone can read through the code and understand what is happening from the comments.<br/> <br/>
Data download & logger control are managed  through the IDE's serial monitor window at 250000 baud. 
The logger WILL NOT START taking readings until those serial handshakes are completed via the UART connection.<br/><br/>
Note that all the readings are initially buffered in opdDataBuffer[16] & sensorDataBuffer[16] arrays so there won't be any data in the EEprom until those ram buffers get transfered.  With the 1-byte RTCtemp only default configuration you will have to wait 16* sampleInterval minutes before that happens. The default 4k eeprom on the rtc module stores 4096 of those readings so takes ~2.8 days at 1min interval before it is full. (at which point the logger shuts down)<br/><br/>
The most important rule to follow when adding new sensors is that the buffer arrays can only handle additions of 1, 2, 4, or 8 bytes.
Odd byte quantities (other than one) and you end up with page boundary issues in the EEprom (where page sizes are always powers of 2)

---

Note: This script will still run on the 3-module "Modules & Jumper Wires"  loggers described in the original Sensors paper: http://www.mdpi.com/1424-8220/18/2/530 
and provides a 'no SD card' method of extending lifespan on the 2020 classroom logger described at https://thecavepearlproject.org/2020/10/22/pro-mini-classroom-datalogger-2020-update/  where multiple I2C eeproms can be added easily via the breadboard(s)
