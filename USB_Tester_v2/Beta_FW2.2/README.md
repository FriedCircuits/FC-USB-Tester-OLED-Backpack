FC-USB-Tester-OLED-Backpack
===========================
This is the beta firmware v2.2 for the USB Tester OLED Backpack. 

Can be found at:
https://friedcircuits.us/tools/46

New Features
===========================

* Cleaned up old code and added comment block to every function
* Events Trigger serial notification of start and end of threshold and percent change
* Ability to save settings to EEPROM
* Ability to disable display for long term logging to save on wear
* New Commmands
	* E:0 - Disable events
	* E:1 - mA Threshold trigger - start and end uses existing W: command to set mA threshold
	* E:2 - Single percent changed
	* P:XXX - Set percent changed trigger
	* C:0 - Output current saved settings
	* C:1 - Load saved settings
	* C:2 - Save settings
	* C:3 - Output running settings
	* D:0 - Disable display
	* D:1 - Enable display



To compile this code you will need the following libraries:
===========================

[UG8LIB] (https://github.com/olikraus/u8glib)

[TimerOne](https://github.com/PaulStoffregen/TimerOne)

[ClickButton](http://code.google.com/p/clickbutton/)

[Adafruit INA219](https://github.com/adafruit/Adafruit_INA219)

[EEPROMex] (https://github.com/thijse/Arduino-EEPROMEx)



Firmware can be used with Wizkers.io 


Firmware Update
==============================

Wizkers.io - Download and select hex from Intrument Upgrade under settings.

Hex can be loaded with AVRDUDE as well. 

Arduino IDE: http://friedcircuits.us/docs/oled-backpack-how-to-update-firmware/


License: All source code and designs are released under 

Creative Commons NonCommercial-ShareAlike 

CC BY-NC-SA

![CC BY-NC-SA](http://i.creativecommons.org/l/by-nc-sa/3.0/88x31.png)

Please contact me for commercial opportunities. 
