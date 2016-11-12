/**
  USB Tester OLED Display 
  This displays the current and voltage from the USB Tester to the OLED display using the OLED Backpack
  Also sends data to serial port for data logging via Java app
  Aviaible at https://friedcircuits.us and docs at https://learn.friedcircuits.us

  @author William Garrido
  @version 2.01
  Created: 01/10/2013

  Changelog by Edouard Lafargue
  
  2013.04.11
  - Autoscroll of graph
  
  -2013.08.17 - William Garrido
  -Removed SVolt and BVolt
  -Moved Current\Volt under graph
  -Extend graph width of display
  -Use clearDisplay to speed up redraws
  -Reduce logo display time
  -Enabled button to cycle display speed
  -Millis based delay
  
  2013.08.27 - Edouard Lafargue
  - Add simple graph autoscale (500mA, 1A, 1.5A, 2A, 5A)
  
  2013.08.29 - Edouard Lafargue
  - Use TimerOne interrupts to read values at a fixed rate, so that
    we can energy consumption
  - New feature: multiple screens (ongoing)
  
  2013.08.30 - Edouard Lafargue
  - Display mA without decimal points
  
  2013.09.09 - William Garrido
  -Added decimal to lower display
  -Top display auto adjust if decimal needed
  -Turned off debug by default
  -Long press clears peaks and energy usage
  
  2013.09.10 - William Garrido
  -Added system display message handling with display time
  -Added Reset message
  How to use setMsg()
  Call setMsg(Message, time to display)
  -Remove decimal due to inacurrate, IC is .8mA resolution
  -fixed which voltage var for peak current tracking
  -fixed min voltage\current reset
  
  2013.09.11 - William Garrido
  -Rewrote button function using ClickButton library from 
  http://code.google.com/p/clickbutton/wiki/Usage
  Fixes screen changing when reseting, 
  so action happens after button is let go
  -Added 2nd printJustified function for decimal
  -Now mWh and mAh use decimals
  
  2013.11.10 - Ed Lafargue
  - Add support for basic serial commands
  - Keep graph memory as unsigned ints instead of floats, saves 256 bytes in memory!
  
  2014.01.12 - William Garrido
  - Improve D-\+ Analog readings by using internal 1.1v ref and dividing by 1023 instead of 1024
  - Split energy readings to multiple lines to take use of larger display and give room for larger readers
  
  2014-10-10 - William Garrido
  - Switch to UG8Lib for graphics handling
  - Split update scope function, only updates graph. Graph data updates is outside picture loop.
  - Spilt message handling so that variables are updated outside picture loop.
  - Clean up code
  - Added command to get firmware version
  
  2014-11-10-2014 - William garrido
  -Re-enable device name on startup
  -Add firmware version to display startup
  
  2016-06-23 - Weistek Eng | Jeremy G |
  -added UpdateTime
  -update time displays time on the energy page, allows you to kepe track of uptime.
  -Compile size 28,306 bytes (98%) only 300 bytes left make them count :)
  
  2016-10-30(2016-11-06) - William Garrido
  -Updated Reset to offset run time
  -Added event start/end handling with serial output
  -Shorten text on splash for Firmware, save flash space
  -Added event percent
  -Reduce flash size using read port
  -EEPROM for saving settings
  -Cmd to turn off display
  -Cleaned up comments and added function comment blocks
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include <U8glib.h>
#include <TimerOne.h>
#include <ClickButton.h>
#include <EEPROMex.h>

/**
 * Firmware version
 * displayed on splach and serial with V: command
*/
#define FW_VERSION 2.20

// All hardware pin usage is defined here:
const byte LEDPIN = 13;
const byte BTN_PIN = 10;
//USB Data Lines
const byte USB_DP = A1;
const byte USB_DM = A0;

// Define two set/clear macros to do PIN manipulation
// way way faster than Arduino's digitalWrite.
// inspired by the the _BV() macro
#define setpin(port, pin) (port) |= (1 << (pin)) 
#define clearpin(port, pin) (port) &= ~(1 << (pin))
#define DEBUG 0

//Vars for handling alerts and events via LED and serial
int16_t ledWarn = 400; //Default threshold in mA
int16_t aPercentChange = 100; //Default percent change of current for event trigger
bool eventFlag = false; //Flag for tracking if event has been triggered
long eventTime = 0; //Time in millis event was triggered
//Set even type we are triggering on or fals if no events
enum eventT {
  DISABLED = 0,
  WARN = 1, //based on set mA threshhold
  PERCENT = 2 //based on percent changed
};
eventT eventType = DISABLED; //Set event type, threshold, percent changed, or disabled(default)
enum eventData {
  NONE = 0, //No recent event
  START = 1, //Event start
  END = 2, //Event end
  SINGLE = 3 //Single event
};
eventData eventStatus = NONE;

// Graph values:
// Graph area is from 0 to 127
#define GRAPH_MEMORY 128
// We store the values as unsigned 16 bit integers (0 to 65535), rather than floats,
// so that our array is only 256 bytes long, instead of 512 (floats are 32bits).
uint16_t graph_Mem[GRAPH_MEMORY];
uint8_t ring_idx = 0; // graph_Mem is managed as a ring buffer.

// Autoscale management
uint16_t autoscale_limits[] = {100, 200, 500, 1000, 1500, 2500, 3200}; // in mA
uint8_t autoscale_size = sizeof(autoscale_limits) / sizeof(uint16_t);
uint8_t graph_MAX = 0; // Max scale by default
float autoscale_max_reading = 0;  // The memorized maximum reading over the window
uint8_t autoscale_countdown = GRAPH_MEMORY;

//Uptime tracking
long int uptimeOldMills = 0;
#define TIMEALL 1 //display time on all available screens. (where space allows)
#define TIMEENERGY 1 //Display time only on the energy screen.
uint8_t timeX = 0;
uint8_t timeY = 7;


//Button
ClickButton modeBtn(BTN_PIN, HIGH);

// Serial input buffer
#define INPUT_BUFFER_SIZE 16
char input_Buffer[INPUT_BUFFER_SIZE];
uint8_t input_Buffer_Index;

// Serial output management
unsigned long lastOutput = 0;
uint16_t serialOutputRate = 1000;

// On-screen output
unsigned long lastDisplay = 0;

//Current Sensor
Adafruit_INA219 ina219;

uint16_t OLED_REFRESH_SPEED = 100; //Startup refresh delay

uint8_t graphX = 0; //Start of X for graph
uint8_t graphY = 0;  //y placement of graph

//Init OLED Display
//Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
U8GLIB_SSD1306_128X64 display(SS, 5, 9);
  
// Voltages are now read in the interrupt routine, and available in
// global (volatile because modified within an interrupt) variables:
volatile float shuntvoltage = 0;
volatile float busvoltage = 0;
volatile float current_mA = 0;
volatile float loadvoltage = 0;
volatile float milliwatthours = 0;
volatile float milliamphours = 0;

// Serial data lines:
volatile float dpVoltage = 0;
volatile float dmVoltage = 0;

// Keep track of peak/min significant values:
volatile float peakCurrent = 0;
volatile float voltageAtPeakCurrent = 0;
volatile float minVoltage = 10;
volatile float currentAtMinVoltage = 0;
// (no need, we can compute it) volatile float peakPower = 0;
volatile float voltageAtPeakPower = 0;
volatile float currentAtPeakPower = 0;

// Keep track of variation of volts/amps over the serial refresh period
volatile float rpPeakCurrent = 0;
volatile float rpMinCurrent = 0;
volatile float rpPeakLoadVolt = 0;
volatile float rpMinLoadVolt = 0;
volatile float rpAvgCurrent = 0;
volatile float old_rpAvgCurrent = 0;
volatile float rpAvgLoadVolt = 0;
uint32_t rpSamples = 1;

// Global defines for polling frequency:
const int READFREQ = 100000; // in microseconds

// Multiple screen support
uint8_t current_screen = 0;
const byte MAX_SCREENS = 6;

//Display message handling
unsigned int setDisplayTime = 0;
char setMsgDisplay[10];
uint8_t oldScreen = 0;
bool msgDisplay = false;
const byte MSGSCREEN = 6;

//Track message display time, made global instead of static so that it is not updated during picture loop
uint8_t msgTime = 0;

//Flag to disable display, good for longtime logging without wearing display
bool enDisplay = true;

//EEPROM Settings http://playground.arduino.cc/Code/EEPROMLoadAndSaveSettings
//ID of the settings block
#define CONFIG_VERSION "1.0"
// Tell it where to store your config data in EEPROM
const int memBase = 32;
const int maxAllowedWrites = 20;
bool eOK = true;
int configAdress=0;
//Flag so we know we didn't load saved config on boot so we can still save new values.
bool skipLoadConfig = false; 
// Example settings structure
struct StoreStruct {
    char version[4];   // This is for detection of settings version
    uint8_t screenMode;
    int16_t warn;
    int16_t percent;
} savedConfig = { 
    CONFIG_VERSION,
    current_screen,
    ledWarn,
    aPercentChange
};

/**
  * Initialisation of pins, serial, display, and EEPROM. Also start timer to read INA219

  * @param none
  * @return none
  */
void setup()
{
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  Serial.begin(115200);

  //Setup button times
  modeBtn.debounceTime   = 20;   // Debounce timer in ms
  modeBtn.multiclickTime = 250;  // Time limit for multi clicks
  modeBtn.longClickTime  = 2000; // time until "held-down clicks" register

  //EEPROM Init
  EEPROM.setMemPool(memBase, EEPROMSizeATmega32u4);
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  configAdress  = EEPROM.getAddress(sizeof(StoreStruct));
  if(!(PINB & (1<<PB6))){
    eOK = loadConfig();
    if(eOK){
      current_screen = savedConfig.screenMode;
      ledWarn = savedConfig.warn;
      aPercentChange = savedConfig.percent;  
    } else {
      //Might be first time with EEPROM code so save default values
      savedConfig.screenMode = current_screen;
      savedConfig.warn = ledWarn;
      savedConfig.percent = aPercentChange;
      strcpy(savedConfig.version, CONFIG_VERSION);
      saveConfig();
      }
  } else {skipLoadConfig = true;}

  //Setup display and show splash
  display.setFont(u8g_font_6x12);
  display.setColorIndex(1);

  display.firstPage();
  do {
   display.drawStr( 2, 10, "USB Tester 2.0");
   display.drawStr( 5, 20, "FriedCircuits.us");
   display.drawStr( 2, 60, "FW: ");
   display.setPrintPos(19,60);
   display.print(FW_VERSION);
   if(skipLoadConfig){
    display.setPrintPos(120,60);
    display.print(F("*"));
   }
  }while( display.nextPage());
  delay(1700);

  // Initialize ring buffer
  for (uint8_t i=0; i < GRAPH_MEMORY; i++) {
    graph_Mem[i] = 0;
  }
  
  //Init current sensor
  ina219.begin();
  
  digitalWrite(LEDPIN, LOW);

  //Start timer for reading INA219
  Timer1.initialize(READFREQ); // 100ms reading interval
  Timer1.attachInterrupt(readADCs); 
}

/**
 * This is where we do the reading of voltage & current. Should be kept as short
 * as possible, of course. Sampling period is 100ms
 * 
 * @param none
 * @return sets global vars for consumption by graph function
 */
void readADCs() {
  // TODO: check exact length of this interrupt routine on the scope using
  // a signal line...
#ifdef DEBUG
      setpin(PORTD,5);  // Just a debug signal for my scope to check
                        // how long it takes for the loop below to complete
#endif
  
  sei(); // re-enable interrupts since the ina219 functions need those.
         // in practice, we're doing nested interrupts, gotta be careful here...
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  milliwatthours += busvoltage*current_mA*READFREQ/1e6/3600; // 1 Wh = 3600 joules
  milliamphours += current_mA*READFREQ/1e6/3600;
  
  // Update peaks, min and avg during our serial refresh period:
  if (current_mA > rpPeakCurrent)
      rpPeakCurrent = current_mA;
  if (current_mA < rpMinCurrent)
      rpMinCurrent = current_mA;
  if (loadvoltage > rpPeakLoadVolt)
      rpPeakLoadVolt  = loadvoltage;
  if (loadvoltage < rpMinLoadVolt)
      rpMinLoadVolt = loadvoltage;
  rpAvgCurrent = (rpAvgCurrent*rpSamples + current_mA)/(rpSamples+1);
  rpAvgLoadVolt = (rpAvgLoadVolt*rpSamples + loadvoltage)/(rpSamples+1);
  rpSamples++;
  
  // Update absolute peaks and mins
  if (current_mA > peakCurrent) {
      peakCurrent = current_mA;
      voltageAtPeakCurrent = loadvoltage;
  }
  
  if (loadvoltage < minVoltage) {
      minVoltage = loadvoltage;
      currentAtMinVoltage = current_mA;
  }
  
  // TODO: check if we gain time by using a peakPower variable ?
  if ( (voltageAtPeakPower*currentAtPeakPower) < (loadvoltage*current_mA)) {
      voltageAtPeakPower = loadvoltage;
      currentAtPeakPower = current_mA;
  }
  
#ifdef DEBUG
      clearpin(PORTD,5);  // Just a debug signal for my scope to check
                        // how long it takes for the loop below to complete
#endif
}


/**
 * This is the display loop, button handler, and sends events if enabled by user
 * 
 * @param none
 * @return none
 */
void loop()
{
  //display.firstPage();
  modeBtn.Update();  
  unsigned long now = millis();

  // Refresh Display  
  if (now - lastDisplay > OLED_REFRESH_SPEED){
    
    // No need to read those inside the interrupt, those are not
    // time-sensitive
    double vcc = readVcc()/1000.0;
    dpVoltage = analogRead(USB_DP) * vcc / 1023;
    dmVoltage = analogRead(USB_DM) * vcc / 1023;

   //Refresh graph from current sensor data
    drawGraph(current_mA);
    //update msg outside picture loop before next display refresh
    if(current_screen == 6){
      if (msgTime <= setDisplayTime){
      msgTime++;
      }
      else {
        current_screen = oldScreen;
        msgTime = 0;
        msgDisplay=false;
      }
    }

    display.firstPage();
    	do{
        if(enDisplay) {
          switch (current_screen) {
            case 0:
              drawScope(now);
              break;
            case 1:
              drawEnergy(now);
              break;
            case 2:
               drawPeakMins(now);
               break;
            case 3:
               drawBig((current_mA*loadvoltage)/1000, "W",2);
               break;
            case 4:
               drawBig(current_mA, "mA",0);
               break;
            case 5:
               drawBig(loadvoltage, "V",2);
               break;
            case 6: //This is a special screen only called within the sketch to take over display with user message
               drawMsg();
               break;
            default:
              drawScope(now);
          }
    drawBottomLine();
	 } } while (display.nextPage() );
    lastDisplay = now;
  }

   //Update btn again in middle of loop to help with performance of btn response
   modeBtn.Update();
  

  // Output on serial port
  if (now - lastOutput > serialOutputRate) {
    serialOutput(now);
    // Reset sampling period:
    rpPeakCurrent = 0;
    rpMinCurrent = current_mA;
    rpPeakLoadVolt = 0;
    rpMinLoadVolt = loadvoltage;
    rpAvgCurrent = current_mA;
    rpAvgLoadVolt = loadvoltage;
    rpSamples = 1;

    lastOutput = now;
  }

  // Check if we have serial input
  while (Serial.available()) {
    input_Buffer[input_Buffer_Index] = Serial.read();
    if (input_Buffer[input_Buffer_Index] == '\n') {
      input_Buffer[input_Buffer_Index] = 0;
      processInput();
      input_Buffer_Index = 0;
    } else 
      input_Buffer_Index = (input_Buffer_Index+1)%INPUT_BUFFER_SIZE;
  }

  //Calculate percent changed, if above set user threshold send single event to serial
  //TODO Handle Negative perecent change
  if(eventType == PERCENT){
    float pChange = 0;
    pChange = ((current_mA - rpAvgCurrent) / (float)rpAvgCurrent) * 100.00;
    if(pChange >= (float)aPercentChange){
      eventStatus = SINGLE;
      eventTime = now-uptimeOldMills;
      sendEvent(pChange);
    }
  }
  uint8_t btnState = (PINB & (1<<PB6)); //digitalRead(BTN_PIN);
  if ((current_mA >= ledWarn) || (btnState)){
    //digitalWrite(LEDPIN, HIGH); 
    setpin(PORTC,7);
    if(eventFlag == false && !btnState && eventType == WARN){
      eventStatus = START;
      eventTime = now-uptimeOldMills;
      sendEvent(ledWarn);
      eventFlag = true;
    }
   }
  else {
     //digitalWrite(LEDPIN, LOW);
     clearpin(PORTC,7);
     if(eventFlag == true && eventType == WARN){
       eventStatus = END;
       eventTime = now-uptimeOldMills;
       sendEvent(ledWarn);
       eventFlag = false;
     }
  }

  if (modeBtn.clicks != 0) setButtonMode(modeBtn.clicks);

}

/** 
 * Here are the commands that are supported:  
 * R:XXXX where XXXX is a delay in miliseconds (any valid integer)
 * S:X    where X is screen number (starting at 1)
 * W:XXXX where XXXX is the warning threshold in mA for switching on the blue LED
 * Z:     reset counters
 * V:     return firmware version
 * E:X    Enable/disable event triggers
 * P:XXX  Sets percent for mA changed event
 * C:X    Control config, read, load and save
 * D:X    Disable/Enable display output
 * 
 * @param none
 * @return none
 */
 void processInput() {
  if (input_Buffer[1] != ':')
    return;
  switch (input_Buffer[0]) {
    case 'R':
      serialOutputRate = atoi(&input_Buffer[2]);
      if (serialOutputRate < 150) serialOutputRate = 150; // We sample at 100ms, so we need to remain above that value
      Serial.print(F("{\"R\":")); Serial.print(serialOutputRate);Serial.println(F("}"));
      break;
    case 'S':
       current_screen = (input_Buffer[2]-49) % MAX_SCREENS;
       Serial.print(F("{\"S\":"));Serial.print(current_screen);Serial.println(F("}"));
       break;
    case 'Z':
       setButtonMode(-1);
       Serial.println(F("{\"Z\":\"OK\"}"));
       break;
    case 'W':
       ledWarn = atoi(&input_Buffer[2]);
       if (ledWarn > 3000) ledWarn = 3000;
       Serial.print(F("{\"W\":")); Serial.print(ledWarn);Serial.println(F("}"));
       break;
    case 'V':
       Serial.print(F("{\"V\":")); Serial.print(FW_VERSION);Serial.println(F("}"));
       break;
    case 'E':{
         uint8_t inputNum = (input_Buffer[2]-48);
         if(inputNum >= 0 && inputNum <= 2){
            eventType = inputNum;
            if(eventType == DISABLED){eventFlag = false;}
         } 
         Serial.print(F("{\"E\":")); Serial.print(eventType);Serial.println(F("}"));
       }
       break;
    case 'P':
      aPercentChange = atoi(&input_Buffer[2]);
      //if (aPercentChange > 100) aPercentChange = 100;
      Serial.print(F("{\"P\":")); Serial.print(aPercentChange);Serial.println(F("}"));
      break;
    case 'C':
      switch (input_Buffer[2]-48) {
        case 0: //Output currently saved config
          if(loadConfig()){
            Serial.print(F("{\"C\":"));
            Serial.print(F("{\"s\":")); Serial.print(savedConfig.screenMode); 
            Serial.print(F(", \"w\":")); Serial.print(savedConfig.warn);  
            Serial.print(F(", \"p\":")); Serial.print(savedConfig.percent);
            Serial.print(F(", \"v\":")); Serial.print(savedConfig.version);
            Serial.println(F("}}"));
          } else { Serial.println(F("Failed"));}
          break;
        case 1: //Load config from EEPROM
          if(loadConfig()){
            current_screen = savedConfig.screenMode;
            ledWarn = savedConfig.warn;
            aPercentChange = savedConfig.percent;  
            Serial.println(F("C:OK"));
          } else { Serial.println(F("C:Failed"));}
          break;
        case 2: //Save config to EEPROM
          savedConfig.screenMode = current_screen;
          savedConfig.warn = ledWarn;
          savedConfig.percent = aPercentChange;
          saveConfig();
          Serial.println(F("C:OK"));
          break;
        case 3: //Output running config
          Serial.print(F("{\"RC\":"));
          Serial.print(F("{\"s\":")); Serial.print(current_screen); 
          Serial.print(F(", \"w\":")); Serial.print(ledWarn);  
          Serial.print(F(", \"p\":")); Serial.print(aPercentChange);
          Serial.print(F(", \"v\":")); Serial.print(CONFIG_VERSION);
          Serial.println(F("}}"));
        default:
          break;
      }
      break;
    case 'D':
      if(input_Buffer[2]-48 == 0){
        enDisplay = false;
        Serial.print(F("{\"D\":")); Serial.print(enDisplay);Serial.println(F("}"));
      } else { 
          enDisplay = true; 
          Serial.print(F("{\"D\":")); Serial.print(enDisplay);Serial.println(F("}")); 
      }
      break;
    default:
      break;
  }
}

/**
 * Draws bottomline of display with current INA219 values
 * called for every screen mode
 * 
 * @param none
 * @return none - output to display buffer
 */
void drawBottomLine() {
  //Set x,y and print sensor data
  display.setPrintPos(0,64);
  display.print(loadvoltage);   display.print(F("V "));
  printJustified(current_mA);   display.print(F("mA "));
  //display.print((current_mA*loadvoltage)/1000);  display.print("W");  
  printJustified2((current_mA*loadvoltage)/1000, 2); display.print("W"); 
}

/**
 * Screen 0: Draws graph view of current
 * 
 * @param none
 * @return none - output to display buffer
 */
void drawScope(long now) {
  if(TIMEALL){updateTime(now,0);}
  
  for (uint8_t i=0; i < GRAPH_MEMORY; i++) {
    uint8_t val = 54 - map(graph_Mem[(i+ring_idx)%GRAPH_MEMORY], 0, autoscale_limits[graph_MAX], 0, 54);
    display.drawPixel(i, val);
  }
  
  // Display current scale
  // Note: commented out, not quite sure this is very important since
  // the graph is merely a trend indicator - we only have 24 pixels after
  // all
  display.setPrintPos(104,7);
  display.print((autoscale_limits[graph_MAX]+0.0)/1000);
}

/**
 * Screen 1: Draw current energy and peak power
 * 
 * @param none
 * @return none - output to display buffer
 */
void drawEnergy(long now) {
  if(TIMEENERGY){updateTime(now, 1);}
  display.setPrintPos(28,7);
  display.print(F("Energy Usage"));
  display.drawHLine(0,7,128);
  display.setPrintPos(0,17);
  printJustified(milliwatthours,2);
  display.print(F("mWh "));
  display.setPrintPos(64,17);
  printJustified(milliamphours,2);
  display.print(F("mAh"));

  display.setPrintPos(0,32);
  display.print(F("Peak: "));
  display.print(voltageAtPeakPower*currentAtPeakPower/1000);
  display.print("W");
  display.setPrintPos(0,41);
  display.print(F("@ "));
  display.print(voltageAtPeakPower);
  display.print(F("V & "));
  display.print(currentAtPeakPower);
  display.print(F("mA"));
  
  display.drawHLine(0,53,128);

}


/**
 * Screen 2: Draws peak and minimum values
 * 
 * @param none
 * @return none - output to display buffer
 */
void drawPeakMins(long now) {
  if(TIMEALL){updateTime(now,1);}
  display.setPrintPos(28,7);
  display.print(F("Peak - Mins"));
  display.drawHLine(0,7,128);
  display.setPrintPos(0,17);
  display.print(F("Peak:"));
  printJustified(peakCurrent);
  display.print(F("mA ("));
  display.print(voltageAtPeakCurrent);
  display.print(F("V) "));
  display.setPrintPos(0,32);
  display.print(F("Min:"));
  display.print(minVoltage);
  display.print(F("V ("));
  display.print(currentAtMinVoltage);
  display.print(F("mA)"));
  
  display.drawHLine(0,53,128);

}

/**
 * Screen 3: Displays one big value & unit, with X number of decimals
 * Display width is 7 digits wide, we leave the 2 rightmost digits for unit display
 * 
 * @param none
 * @return none - output to display buffer
 */
void drawBig(float val, char* unit, uint8_t decimals) {
  display.setPrintPos(50,16);
  display.setFont(u8g_font_10x20);
  if ((decimals < 2) && (val < 1000)) display.print(" ");
  if ((decimals < 1) && (val < 10000)) display.print(" ");
  if (val < 100) display.print(" ");
  if (val < 10) display.print(" ");
  display.print(val, decimals);
  display.print(unit);
  display.setFont(u8g_font_6x12);
  
  
  display.drawRFrame(0,24,128,20,1);
  float limit = 0.0;
  if(unit == "W"){
    display.setPrintPos(0,7);
    display.print(F("Peak:"));
    display.setPrintPos(0,16);
    display.print(voltageAtPeakPower*currentAtPeakPower/1000);
    display.print("W");
    
    float wattMax = 1.10;
    if(val > 10.10){limit = 0.201; wattMax = 25.10;}
    else if (val > 1.10){limit = 0.041; wattMax = 5.10;}
    else{limit = 0.009;} //1W range
    for(int x = 2; x <= 126; x++){
      if ((float)x <= ((float)val/limit)){display.drawVLine(x,26,16);}  
    }
    //display.drawVLine((voltageAtPeakCurrent/limit),26,16);
    display.drawVLine(0,44,8); 
    display.setPrintPos(1,52); display.print(F("0"));
    display.drawVLine(64,44,8);
    display.setPrintPos(32,52); display.print(wattMax/2);  
    display.setPrintPos(97,52); display.print(wattMax);
    display.drawVLine(127,44,8); 
  }
 
  
  if(unit == "mA"){
    display.setPrintPos(0,7);
    display.print(F("Peak:"));
    display.setPrintPos(0,16);
    display.print(peakCurrent);
    display.print("mA");

    limit = (float)autoscale_limits[graph_MAX]/127; 
    for(int x = 2; x < 126; x++){
      if (x <= (val/limit)){display.drawVLine(x,26,16);}  
    }
    display.drawVLine((peakCurrent/limit),26,16);
    display.drawVLine(0,44,8); 
    display.setPrintPos(1,52); display.print(F("0"));
    display.drawVLine(64,44,8); 
    display.setPrintPos(32,52); display.print(((autoscale_limits[graph_MAX]+0.0)/1000)/2); display.print(F("A"));
    display.setPrintPos(97,52); display.print((autoscale_limits[graph_MAX]+0.0)/1000); display.print(F("A"));
    display.drawVLine(127,44,8); 
  }
  if(unit == "V"){
    display.setPrintPos(0,7);
    display.print(F("D+: "));
    display.print(dpVoltage);
    display.setPrintPos(0,16);
    display.print(F("D-: "));
    display.print(dmVoltage);
  
    float voltMax = 5.10;
    if(val > 12.10){limit = 0.21; voltMax = 26.0;}
    else if (val > 5.10){limit = 0.098; voltMax = 12.10;}
    else{limit = 0.041;} //5V range
    for(int x = 2; x <= 126; x++){
      if ((float)x <= ((float)val/limit)){display.drawVLine(x,26,16);}  
    }
    //display.drawVLine((voltageAtPeakCurrent/limit),26,16);
    display.drawVLine(0,44,8); 
    display.setPrintPos(1,52); display.print(F("0"));
    display.drawVLine(64,44,8);
    display.setPrintPos(32,52); display.print(voltMax/2);  
    display.setPrintPos(97,52); display.print(voltMax);
    display.drawVLine(127,44,8); 
  }
}

/**
 * Screen -1: Draws message in larger font on entire display
 * 
 * @param none
 * @return none - output to display buffer
 */
void setMsg(char* msg, uint16_t time)
{
   if(msgDisplay == false){
     setDisplayTime = time;
     strcpy(setMsgDisplay, msg);
     msgDisplay=true;
     oldScreen = current_screen;
     current_screen=MSGSCREEN;
   }
}
void drawMsg()
{
  if (msgTime <= setDisplayTime){
  display.setPrintPos(0,16);
  display.setFont(u8g_font_10x20);
  display.print(setMsgDisplay);
  display.setFont(u8g_font_6x12);
  }
  
}

/**
 * Update ring buffer used for drawing graph on screen 0
 * 
 * @param none
 * @return none - update ring buffer and scale global vars
 */
void drawGraph(float reading) {
  /* Adjust scale: we have GRAPH_MEMORY points, so whenever
     we cross a scale boundary, we initiate a countdown timer.
     This timer goes down at each redraw if we're under the previous lower scale
     boundary, otherwise it is reset. If we stay below the scale boundary until it
     reaches zero, then we scale down. */

  if (reading > autoscale_limits[graph_MAX]) {
    // We need to scale up:
    while (reading > autoscale_limits[graph_MAX]) {
      graph_MAX++;
      if (graph_MAX == autoscale_size-1)
        break;
      }
    autoscale_countdown = GRAPH_MEMORY;
    autoscale_max_reading = 0;
  } else if (graph_MAX > 0) {
    // Do we need to scale down ?
    if (reading < autoscale_limits[graph_MAX-1]) {
      autoscale_countdown--; // If we are below the lower scale
      // Keep track of max value of reading during the time we're under current
      // scale limit, so that when we scale down, we go to the correct scale
      // value right away:
      if (reading > autoscale_max_reading)
          autoscale_max_reading = reading;
      if (autoscale_countdown == 0) {
        // Time to scale down:
        while (autoscale_max_reading < autoscale_limits[graph_MAX-1]) {
          graph_MAX--;
          if (graph_MAX == 0)
            break;
        }
        autoscale_countdown = GRAPH_MEMORY;
        autoscale_max_reading = 0;
      }
    } else {
      autoscale_countdown = GRAPH_MEMORY; // we are above the scale under us
      autoscale_max_reading = 0;
    }
  }
  graph_Mem[ring_idx] = (uint16_t) reading;
  ring_idx = (ring_idx+1)%GRAPH_MEMORY;
}

/**
 * Called at set interval by main loop to update serial
 * 
 * @param none
 * @return none - output to serial of current data
 */
void serialOutput(long now) {
  if(Serial){
    Serial.print(F("{ \"a\":{ \"max\":"));
    Serial.print(rpPeakCurrent);
    Serial.print(F(", \"min\":"));
    Serial.print(rpMinCurrent);
    Serial.print(F(", \"avg\":"));
    Serial.print(rpAvgCurrent);
    Serial.print(F("}, \"v\":{ \"max\":"));
    Serial.print(rpPeakLoadVolt);
    Serial.print(F(", \"min\":"));
    Serial.print(rpMinLoadVolt);
    Serial.print(F(", \"avg\":"));
    Serial.print(rpAvgLoadVolt);
    Serial.print(F("}, \"mah\":"));
    Serial.print(milliamphours);
    Serial.print(F(", \"mwh\":"));
    Serial.print(milliwatthours);
    Serial.print(F(", \"shunt\":"));
    Serial.print(shuntvoltage);
    Serial.print(F(", \"dp\":"));
    Serial.print(dpVoltage);
    Serial.print(F(", \"dm\":"));
    Serial.print(dmVoltage);
    Serial.print(F(", \"ram\":"));
    Serial.print(freeRam()); //Was for checking free ram during development
    Serial.print(F(", \"time\":"));
    Serial.print(now-uptimeOldMills);
    Serial.println("}");
  }
  
}

/**
 * Right-justify values and round to nearest integer, with optional decimal places
 * 
 * @param none
 * @return none - output to display buffer
 */
void printJustified(float val, uint8_t dec)
{
  val = floor(val + 0.5);
  if (val < 1000) display.print(" ");
  if (val < 100) display.print(" ");
  if (val < 10) display.print(" ");
  display.print(val,dec); 
}

/**
 * Right-justify values with 0 dec places
 * 
 * @param float with value to display
 * @return none - output to display buffer
 */
void printJustified(float val) {
  printJustified(val,0);
}

/**
 * Right-justify by padding needed spaces
 * depending on value length
 * 
 * @param float of value to display and uint8 dec places
 * @return none - output to display buffer
 */
void printJustified2(float val, uint8_t dec)
{
  //val = floor(val + 0.5);
  if (val < 1000) display.print(" ");
  if (val < 100) display.print(" ");
  if (val < 10) display.print(" ");
  display.print(val,dec); 
}

/**
 * Copy of Arduino map function converted for floats, from a forum post
 * 
 * @param none
 * @return float of mapped value to range
 */
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Calculates free SRAM - from Arduino Playground
 * 
 * @param none
 * @return int of free SRAM
 */
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/**
 * Handles setting screen and reseting points\
 * Can be used to add function based on 2+ clicks
 * 
 * @param num of button clicks
 * @return none - updates global values based on num of btn clicks
 */
void setButtonMode(int8_t btnClicks){
  switch (btnClicks){
    case -1: //longpress
        setMsg("RESET", 10);
        peakCurrent = 0;
        voltageAtPeakCurrent = 0;
        minVoltage = loadvoltage;
        currentAtMinVoltage = current_mA;
        voltageAtPeakPower = 0;
        currentAtPeakPower = 0;
        milliwatthours = 0;
        milliamphours = 0;
        uptimeOldMills = millis();
        break;
    case 1:
        current_screen = (current_screen + 1) % MAX_SCREENS;
    break;
   //default:
   
  }
}

/**
 * http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
 * Read 1.1V reference against AVcc
 * set the reference to Vcc and the measurement to the internal 1.1V reference
 * Used for analog read of D+/D- data lines
 * 
 * @param none
 * @return long VCC
 */
long readVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
   
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

/**
 * Handles converting millis to HR MIN SECS
 * and outputting to desired draw location
 * depending on screen we are currently on
 * 
 * @param long now current millis, uint8 page we are on to set draw location
 * @return none -  output to display buffer
 */
void updateTime(long now, uint8_t page)
{
  //update time using millis
  //converts millis to 12hr time. used in the energy portion.
  //This allows you to keep track of the uptime of the current test.
  //Time is reset with offset when RESET command is run. WG
   
  long hours=0;
  long mins=0;
  long secs=0;
  secs = (now-uptimeOldMills)/1000; //convect milliseconds to seconds
  mins=secs/60; //convert seconds to minutes
  hours=mins/60; //convert minutes to hours
  secs=secs-(mins*60); //subtract the coverted seconds to minutes in order to display 59 secs max 
  mins=mins-(hours*60); //subtract the coverted minutes to hours in order to display 59 minutes max
  //hours=hours-(days*24); //subtract the coverted hours to days in order to display 23 hours max
  //Display results
  if(!page){display.setPrintPos(timeX,timeY);}
  if(page){display.setPrintPos(0,50);}
  //display.print(F("Time: "));
  display.print(hours);
  display.print(":");
  display.print(mins);
  display.print(":");
  display.print(secs);
}

/**
 * Handles outputing to serial a threahold even
 * 
 * @param int16 current mA threshhold value
 * @return none -  output to serial port
 */
void sendEvent (int16_t threshhold){
  if(Serial){
      Serial.print(F("{ \"event\":{ \"i\":"));
      Serial.print(eventStatus);
      Serial.print(F(", \"t\":"));
      Serial.print(eventTime);
      Serial.print(F(", \"c\":"));
      Serial.print(eventType);
      Serial.print(F(", \"a\":"));
      Serial.print(current_mA);
      Serial.print(F(", \"w\":"));
      Serial.print(threshhold);
      Serial.println(F("}}"));
  }
}

/**
 * Loads config from EEPROM
 * Checks to see if config version matches 
 * code config version
 * 
 * @param none
 * @return bool if saved config matches code config version
 */
bool loadConfig() {
  EEPROM.readBlock(configAdress, savedConfig);
  return !strcmp(savedConfig.version, CONFIG_VERSION);
}

/**
 * Saves config to EEPROM
 * from configAddress struct
 * assumes struct has been refreshed from calling function
 * 
 * @param none
 * @return none - saves to EEPROM
 */
void saveConfig() {
   EEPROM.writeBlock(configAdress, savedConfig);
}
