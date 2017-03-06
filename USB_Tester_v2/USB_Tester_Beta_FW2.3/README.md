FC-USB-Tester-OLED-Backpack
===========================
This is the beta firmware v2.2 for the USB Tester OLED Backpack. 

Can be found at:
https://friedcircuits.us/tools/46

New Features
===========================
Beta FW 2.2
* Cleaned up old code and added comment block to every function
* Added run timer, thanks to WeisTekEng
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

Hold button during boot to disable loading settings from EEPROM. 

Beta FW 2.31 - Code review help from @PhilipFreidin from fliptronics.com
* Increase sample speed default to 1kHz! This is a biggy feature, made possible by the following optimizations
* Increase I2C clock to 800kHz from 100kHz
* USB D+/- ADC uses int and ADC clock set to 250khz, a bit past recommended of 200khz
* Removed F() to free up flash and should be faster but uses 14% more RAM
* Fixed drawEnergy, wasn't using printjustifed2 so was rounding and displaying .00
* Using accumulators in readADC ISR and calculate in main loop mAh, mWh and current/voltage avg
* Optimize graph draw by moving mapping of values to drawGraph
* Optimize map function
* Reduce use of floats
* Libraries now included in source folder for portablity and customization of libraries
* Allow faster serial rate
* 2.31 fixed mAh/mWh calculations
* TODO: Handle negative current for monitoring battery charging. 

28502 Bytes used
  130 Bytes free

Uses the following libraries:
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
