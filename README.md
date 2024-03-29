# 2-Part-ProMini-EEprom-Data-Logger_2022
<img src="https://github.com/EKMallon/The_Cave_Pearl_Project_CURRENT_codebuilds/blob/master/images/CavePearlProjectBanner_130x850px.jpg">

This program supports an ongoing series of DIY 'Classroom Logger' tutorials from Edward Mallon & Dr. Patricia Beddows at the Cave Pearl Project. The idea is to provide a starting point for student projects in environmental monitoring courses and/or thesis level research.<br/>
<br/>The tutorial that matches this code can be found at:<br/>
https://thecavepearlproject.org/2022/03/09/powering-a-promini-logger-for-one-year-on-a-coin-cell/
<br/>with a detailed building guide video at:<br/>
https://www.youtube.com/watch?v=58ps9fUyY0Q&t=0s&ab_channel=EdwardMallon

---
<img   align="right" width="400" src="https://github.com/EKMallon/2-Part-ProMini-EEprom-Data-Logger_2022/blob/main/images/2-PartEEpromLogger_CavePearlProject_2022.jpg">
This 'low power' 2-module iteration runs the logger from a CR2032 coin cell and uses  EEprom memory to store sensor readings. This necessarily involves several power optimization steps which add significant complexity to the base code (as compared to previous versions) but hopefully everyone can read through the code and understand what is happening from the extensive comments. There are several manual configuration settings controlled by #define statements at the start of the program, and the logger will not be able to read the coincell voltage properly until you tweak the InternalReferenceConstant. <br/> <br/>
Data download & logger control are managed  through the IDE's serial monitor window at 500000 baud. 
The logger WILL NOT START taking readings until those serial handshakes are completed via the UART connection.<br/><br/>

The most important rule to follow when adding new sensors is that the buffer arrays can only handle 'powers of 2' additions of 1, 2, 4, or 8 bytes.
Odd byte quantities per sensor record (other than one) and you end up with page boundary issues in the EEprom.

---

Note: This script will still run on the 3-module "Modules & Jumper Wires"  loggers described in the original Sensors paper: http://www.mdpi.com/1424-8220/18/2/530 
and provides a 'no SD card' operation on the older 2020 classroom logger described at https://thecavepearlproject.org/2020/10/22/pro-mini-classroom-datalogger-2020-update/ but you will have to change the LED pins.
