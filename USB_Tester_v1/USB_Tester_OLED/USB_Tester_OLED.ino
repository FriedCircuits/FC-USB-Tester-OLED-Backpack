
/*************************************
USB Tester OLED Display - v1.2
Created by: William Garrido
Created: 01/10/2013
This displays the current and voltage from the USB Tester to the OLED display using the OLED Backpack
Also sends data to serial port for data logging via Java app
Kits are aviaible at www.tindie.com and more info can be found on www.friedcircuits.us

Changelog by Edouard Lafargue

2013.04.11
- Autoscroll of graph

-William Garrido
2013.08.17
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
- Switched to JSON for serial protocol

2013.12.14 - William Garrido
Convert for use on 128x32 OLED Backpack (v1 Series)
**************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimerOne.h>
#include <ClickButton.h>

// All hardware pin usage is defined here:
#define OLED_DC 5
#define OLED_CS SS
#define OLED_CLK SCK
#define OLED_MOSI MOSI
#define OLED_RESET 9
#define LEDPIN 13
#define BTN_PIN 10

// Define two set/clear macros to do PIN manipulation
// way way faster than Arduino's digitalWrite.
// inspired by the the _BV() macro
#define setpin(port, pin) (port) |= (1 << (pin)) 
#define clearpin(port, pin) (port) &= ~(1 << (pin))

#define DEBUG 0

uint16_t ledWarn = 350; //Threshold in mA

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
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

  
//FriedCircuits Logo for startup 
static unsigned char FriedCircuitsUSBTester[] PROGMEM = 
{
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x87, 0x87, 0xC0, 0x0C, 0x41, 0x86, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x41, 0x86, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x40, 0xCC, 0x70,
0xE1, 0xF0, 0xF0, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x0C, 0x40, 0xFC, 0xF8,
0xF3, 0xF9, 0xF9, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x0C, 0x40, 0x31, 0xCC,
0x33, 0x1B, 0x1D, 0x8C, 0x00, 0x00, 0x00, 0x00, 0x18, 0xC0, 0x00, 0x00, 0x0C, 0x40, 0x01, 0x8C,
0x1A, 0x1B, 0x0D, 0x8C, 0x00, 0x07, 0xFF, 0xFF, 0xF8, 0xC0, 0x00, 0x00, 0x0C, 0x40, 0x01, 0x8C,
0x33, 0x1B, 0x99, 0x8C, 0x00, 0x1F, 0xFF, 0xFF, 0xF8, 0xC0, 0x00, 0x00, 0x0C, 0x40, 0x39, 0xFC,
0xF3, 0xF1, 0xF9, 0xF8, 0x00, 0x78, 0x00, 0x00, 0x1F, 0xC0, 0x00, 0x00, 0x0C, 0x40, 0xFC, 0xF8,
0xC0, 0xE0, 0xF0, 0x70, 0x01, 0xE0, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x0C, 0x40, 0xC6, 0x30,
0x00, 0x08, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x40, 0xC6, 0x30,
0x00, 0x0C, 0x00, 0x00, 0x3E, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x40, 0xC6, 0x30,
0x00, 0x1E, 0x00, 0x00, 0xF0, 0x01, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x40, 0xFE, 0x30,
0xFE, 0x1F, 0x1F, 0xFF, 0xC0, 0x01, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x40, 0x7C, 0x30,
0xFE, 0x3F, 0x9F, 0xC0, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xBF, 0x8F, 0xBF, 0x1D, 0xF7, 0xE0, 0xF3, 0xBF, 0x06, 0x73, 0xBF, 0xF9, 0x83, 0x9C, 0x60,
0x00, 0xFF, 0xCF, 0xBF, 0xBD, 0xF7, 0xF9, 0xF7, 0xBF, 0x9E, 0xF3, 0xFF, 0xFF, 0x83, 0xDD, 0xF0,
0x00, 0xFF, 0xCE, 0x3F, 0xBD, 0xC7, 0xFB, 0xC7, 0xBF, 0xBE, 0xF3, 0xFB, 0xFF, 0x83, 0xDF, 0xE0,
0x00, 0xFF, 0xCF, 0xBF, 0xBD, 0xF7, 0xBB, 0x87, 0xBF, 0xB8, 0xF3, 0xF9, 0xCF, 0xC3, 0xDD, 0xF0,
0x00, 0xFF, 0xCF, 0xBF, 0xBD, 0xF7, 0xBF, 0x87, 0xBF, 0x78, 0xF7, 0x79, 0xC3, 0xC3, 0xDC, 0xF0,
0x02, 0xFF, 0xCE, 0x3F, 0xFD, 0xF7, 0xFB, 0xF7, 0xBF, 0xBE, 0x7F, 0x79, 0xC7, 0xDB, 0xFD, 0xF0,
0x06, 0xFF, 0xBE, 0x3D, 0xFD, 0xFF, 0xE3, 0xFF, 0xBB, 0xBF, 0x7F, 0x79, 0xC7, 0xB9, 0xF9, 0xE0,
0x03, 0x7F, 0x7E, 0x19, 0x99, 0xFF, 0x80, 0xFB, 0xB9, 0x8F, 0x3C, 0x39, 0xC6, 0x18, 0xF1, 0xC0,
0x03, 0xFC, 0x00, 0x00, 0x00, 0x7F, 0x07, 0xE0, 0x00, 0x00, 0x1C, 0x0C, 0x0C, 0x40, 0x18, 0x30
};

// Voltages are now read in the interrupt routine, and available in
// global (volatile because modified within an interrupt) variables:
volatile float shuntvoltage = 0;
volatile float busvoltage = 0;
volatile float current_mA = 0;
volatile float loadvoltage = 0;
volatile float milliwatthours = 0;
volatile float milliamphours = 0;


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
volatile float rpAvgLoadVolt = 0;
uint32_t rpSamples = 1;



// Global defines for polling frequency:
#define READFREQ 100000 // in microseconds

// Multiple screen support
uint8_t current_screen = 0;
#define MAX_SCREENS  6;

//Display message handling
unsigned int setDisplayTime = 0;
char setMsgDisplay[10];
uint8_t oldScreen = 0;
bool msgDisplay = false;
#define MSGSCREEN 6;


/**
 * Initialisation of the sketch
 */
void setup()
{

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  
  //Setup button times
  modeBtn.debounceTime   = 20;   // Debounce timer in ms
  modeBtn.multiclickTime = 250;  // Time limit for multi clicks
  modeBtn.longClickTime  = 2000; // time until "held-down clicks" register

  
  Serial.begin(115200);
  //Serial.println(F("USB Tester OLED Backup Online"));
   
  //Init current sensor
  ina219.begin();
  
  //by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  
  //Setup display
  display.setTextSize(1);
  display.setTextColor(WHITE);
 
  // Initialize ring buffer
  for (uint8_t i=0; i < GRAPH_MEMORY; i++) {
    graph_Mem[i] = 0;
  }

  
  //show splashscreen
  display.clearDisplay(); 
  display.drawBitmap(0, 0, FriedCircuitsUSBTester, 128, 24, WHITE);
  display.display();
  delay(1000);
  display.clearDisplay();   // clears the screen and buffer
  
  digitalWrite(LEDPIN, LOW);
  
  
  Timer1.initialize(READFREQ); // 100ms reading interval
  Timer1.attachInterrupt(readADCs); 
}

/**
 * This is where we do the reading of voltage & current. Should be kept as short
 * as possible, of course. our sampling period is 100ms
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
 * This is the display loop and button handler
 */
void loop()
{
  
  modeBtn.Update();  
  unsigned long now = millis();

  // Refresh Display  
  if (now - lastDisplay > OLED_REFRESH_SPEED){
    
    // No need to read those inside the interrupt, those are not
    // time-sensitive
    
    // For some reason the display offset jumps around every once in a while
    // when opening the USB serial port. This sounds like a big bug, but I was not
    // able to track it down (E. Lafargue)
    display.ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    display.ssd1306_command(0x0);                                   // no offset

    display.clearDisplay();
    switch (current_screen) {
      case 0:
        drawScope();
        break;
      case 1:
        drawEnergy();
        break;
      case 2:
         drawPeakMins();
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
        drawScope();
    }
    
    drawBottomLine();
    display.display();
    lastDisplay = now;
  }

  // Output on serial port
  if (now - lastOutput > serialOutputRate) {
    serialOutput();
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

  uint16_t btnState = digitalRead(BTN_PIN);
  
  if ((current_mA >= ledWarn) || (btnState)){
    digitalWrite(LEDPIN, HIGH);
  }
  else {
     digitalWrite(LEDPIN, LOW);
  }

  if (modeBtn.clicks != 0) setButtonMode(modeBtn.clicks);

}

// Here are the commands that are supported:
//   R:XXXX where XXXX is a delay in miliseconds (any valid integer)
//   S:X    where X is screen number (starting at 1)
//   W:XXXX where XXXX is the warning threshold in mA for switching on the blue LED
//   Z:     reset counters

void processInput() {
  if (input_Buffer[1] != ':')
    return;
  switch (input_Buffer[0]) {
    case 'R':
      serialOutputRate = atoi(&input_Buffer[2]);
      if (serialOutputRate < 150) serialOutputRate = 150; // We sample at 100ms, so we need to remain above that value
      Serial.print(F("{ \"R\":")); Serial.print(serialOutputRate);Serial.println("}");
      break;
    case 'S':
       current_screen = (input_Buffer[2]-49) % MAX_SCREENS;
       Serial.print(F("{\"S\":"));Serial.println(current_screen);Serial.println("}");
       break;
    case 'Z':
       setButtonMode(-1);
       Serial.println(F("{\"Z\":\"OK\"}"));
       break;
    case 'W':
       ledWarn = atoi(&input_Buffer[2]);
       if (ledWarn > 2000) ledWarn = 2000;
       Serial.print(F("{\"W\":")); Serial.println(ledWarn);Serial.println("}");
       break;
    default:
      break;
  }
}



void drawBottomLine() {
  //Set x,y and print sensor data
  display.setCursor(0,25);
  display.print(loadvoltage);   display.print(F("V "));
  printJustified(current_mA);   display.print(F("mA "));
  display.print((current_mA*loadvoltage)/1000);  display.print("W");  

}


// Our various screens:
void drawScope() {
    //Refresh graph from current sensor data
  drawGraph(current_mA);
  
  // Display current scale
  // Note: commented out, not quite sure this is very important since
  // the graph is merely a trend indicator - we only have 24 pixels after
  // all
  display.setCursor(104,0);
  display.print((autoscale_limits[graph_MAX]+0.0)/1000);

   
}

// Draw current energy values and peak
// power
void drawEnergy() {
  display.setCursor(0,0);
  printJustified(milliwatthours,2);
  display.print(F("mWh "));
  printJustified(milliamphours,2);
  display.print(F("mAh"));

  display.setCursor(0,8);
  display.print(F("Peak: "));
  display.print(voltageAtPeakPower*currentAtPeakPower/1000);
  display.print("W");
  display.setCursor(0,16);
  display.print(F("@ "));
  display.print(voltageAtPeakPower);
  display.print(F("V & "));
  display.print(currentAtPeakPower);
  display.print(F("mA"));
  
  display.drawFastHLine(0,23,128,WHITE);

}


// Displays peak and minimum values
void drawPeakMins() {
  display.setCursor(0,0);
  display.print(F("Peak:"));
  printJustified(peakCurrent);
  display.print(F("mA ("));
  display.print(voltageAtPeakCurrent);
  display.print(F("V) "));
  display.setCursor(0,9);
  display.print(F("Min:"));
  display.print(minVoltage);
  display.print(F("V ("));
  display.print(currentAtMinVoltage);
  display.print(F("mA)"));
  
  display.drawFastHLine(0,23,128,WHITE);

}

// Displays one big value & unit, with X number of decimals
// Display width is 7 digits wide, we leave the 2 rightmost digits
// for unit display
void drawBig(float val, char* unit, uint8_t decimals) {
  display.setCursor(0,0);
  display.setTextSize(3);
  if ((decimals < 2) && (val < 1000)) display.print(" ");
  if ((decimals < 1) && (val < 10000)) display.print(" ");
  if (val < 100) display.print(" ");
  if (val < 10) display.print(" ");
  display.print(val, decimals);
  display.print(unit);
  display.setTextSize(1);
}

//Displays message in large font on entire display
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
  static unsigned msgTime = 0;
  
  if (msgTime <= setDisplayTime){
  display.setCursor(0,0);
  display.setTextSize(4);
  display.print(setMsgDisplay);
  display.setTextSize(1);
  msgTime++;
  }
  else {
    current_screen = oldScreen;
    msgTime = 0;
    msgDisplay=false;
  }
  
}

// Draw the complete graph. This is also where we update the ring
// buffer for graph values
void drawGraph(float reading) {
  
  // Adjust scale: we have GRAPH_MEMORY points, so whenever
  // we cross a scale boundary, we initiate a countdown timer.
  // This timer goes down at each redraw if we're under the previous lower scale
  // boundary, otherwise it is reset. If we stay below the scale boundary until it
  // reaches zero, then we scale down.


  if (reading > autoscale_limits[graph_MAX]) {
    // We need to scale up:
    while (reading > autoscale_limits[graph_MAX]) {
      graph_MAX++;
      if (graph_MAX == autoscale_size-1)
        break;
      }
    autoscale_countdown = GRAPH_MEMORY;
    autoscale_max_reading = 0;
    // Let user know the values just got rescaled
    display.setCursor(122,0);
    display.print("*");
    delay(100);
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
        // Let user know the values just got rescaled
        display.setCursor(122,0);
        display.print("*");
        delay(100);
      }
    } else {
      autoscale_countdown = GRAPH_MEMORY; // we are above the scale under us
      autoscale_max_reading = 0;
    }
  }
  
  graph_Mem[ring_idx] = (uint16_t) reading;
  ring_idx = (ring_idx+1)%GRAPH_MEMORY;
  for (uint8_t i=0; i < GRAPH_MEMORY; i++) {
    uint8_t val = 24 - map(graph_Mem[(i+ring_idx)%GRAPH_MEMORY], 0, autoscale_limits[graph_MAX], 0, 24);
    display.drawPixel(i, val , WHITE);
  }
}


  //Serial output for data logging with Java app
void serialOutput() {
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
  Serial.print(0);
  Serial.print(F(", \"dm\":"));
  Serial.print(0);
  Serial.print(F(", \"ram\":"));
  Serial.print(freeRam()); //Was for checking free ram during development
  Serial.println("}");
  
}


// Right-justify values and round to nearest integer, with optional decimal places
void printJustified(float val, uint8_t dec)
{
  val = floor(val + 0.5);
  if (val < 1000) display.print(" ");
  if (val < 100) display.print(" ");
  if (val < 10) display.print(" ");
  
  display.print(val,dec); 

}

void printJustified(float val) {
  printJustified(val,0);
  
}


//Copy of Arduino map function converted for floats, from a forum post
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//From Arduino Playground
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//Handles setting screen and reseting points\
//Can be used to add function based on 2+ clicks
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
        break;
    
    case 1:
        current_screen = (current_screen + 1) % MAX_SCREENS;
    break;
    
   //default:
   
  }
    
}




