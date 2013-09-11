
/*************************************
USB Tester OLED Display
Created by: William Garrido
Created: 01/10/2013
This displays the current and voltage from the USB Tester to the OLED display using the OLED Backpack
Also sends data to serial port for data logging via Java app
Kits are aviaible at www.tindie.com and more info can be found on www.friedcircuits.us

Changelog by Edouard Lafargue

2013.04.11
- Autoscroll of graph

ToDo:
- Remove shunt voltage and load voltage display (are those really necessary ?)
- Autoscale of graph

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

**************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimerOne.h>


#define OLED_DC 5
#define OLED_CS SS
#define OLED_CLK SCK
#define OLED_MOSI MOSI
#define OLED_RESET 9

// Define two set/clear macros to do PIN manipulation
// way way faster than Arduino's digitalWrite.
// inspired by the the _BV() macro
#define setpin(port, pin) (port) |= (1 << (pin)) 
#define clearpin(port, pin) (port) &= ~(1 << (pin))

#define DEBUG 0

const int LEDPIN = 13;
int ledWarn = 350; //Threshold in mA

const int maxMode = 4;

// Graph values:

// Graph area is from 0 to 128 (inclusive), 128 points altogether
#define GRAPH_MEMORY 128
float graph_Mem[GRAPH_MEMORY];
int ring_idx = 0; // graph_Mem is managed as a ring buffer.

// Autoscale management
int autoscale_limits[] = {100, 200, 500, 1000, 1500, 2500, 3200}; // in mA
int autoscale_size = sizeof(autoscale_limits) / sizeof(float);
int graph_MAX = 0; // Max scale by default
int autoscale_max_reading = 0;
int autoscale_countdown = GRAPH_MEMORY;

//Button
const int btnPin = 10;

//Current Sensor
Adafruit_INA219 ina219;


int OLED_REFRESH_SPEED = 100; //Startup refresh delay
//Define refresh delay for each mode
const int speed0 = 0;
const int speed1 = 50;
const int speed2 = 100;
const int speed3 = 250;
const int speed4 = 500;

int graphX = 0; //Start of X for graph
int graphY = 0;  //y placement of graph

//Init OLED Display
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

  
//FriedCircuits Logo for startup 
static unsigned char PROGMEM FriedCircuitsUSBTester[] = 
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



// Global defines for polling frequency:
#define READFREQ 100000 // in microseconds

// Multiple screen support
int current_screen = 0;
#define MAX_SCREENS  6;

//Display message handling
unsigned int setDisplayTime = 0;
char setMsgDisplay[10];
int oldScreen = 0;
bool msgDisplay = false;
#define msgScreen 6;

/**
  Setting the last "X" seconds: use the serial link to set this up, and store value in eeprom ?
  Add a command for auto screen cycling
  Add a command to reset readings without unplugging

  Here are all the screens. Going from one screen to the next using a short button press

   - On all screens: instant voltage/current/Watts on bottom line

   - 1st screen: "Graph"
                current graph

   - 2nd screen: "Peaks / Mins"
                Peak current since startup (and voltage at that point)
                Min voltage since startup (and current at that point)

   - 3rd screen: "Energy / Power"
                Watt hours (mWh until reached 1Wh, then in Wh ? not implemented yet)
                Peak watts since startup, and current/voltage reading at that point - maximum power point
                Amp hours since startup? nice to check battery capacity when charging a battery through USB ?

*/


void setup()
{

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  
  pinMode(btnPin, INPUT);
  
  Serial.begin(115200);
  Serial.println("USB Tester OLED Backup Online");
   
  //Init current sensor
  ina219.begin();
  
  //by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  
  //Setup display
  display.setTextSize(1);
  display.setTextColor(WHITE);
 
  // Initialize ring buffer
  for (int i=0; i < GRAPH_MEMORY; i++) {
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
 * as possible, of course.
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
  
  // Update peaks and mins
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
    
  
  static unsigned long last_interrupt_time2 = 0;
  unsigned long interrupt_time2 = millis();
  
  if (interrupt_time2 - last_interrupt_time2 > OLED_REFRESH_SPEED){
  
  //Setup placement for sensor readouts
  display.clearDisplay();
  
  switch (current_screen) {
    case 0:
      drawScope();
      drawBottomLine();
      break;
    case 1:
      drawEnergy();
      drawBottomLine();
      break;
    case 2:
       drawPeakMins();
       drawBottomLine();
       break;
    case 3:
       drawBig((current_mA*loadvoltage)/1000, "W",2);
       drawBottomLine();
       break;
    case 4:
       drawBig(current_mA, "mA",0);
       drawBottomLine();
       break;
    case 5:
       drawBig(loadvoltage, "V",2);
       drawBottomLine();
       break;
    case 6: //This is a special screen only called within the sketch to take over display with user message
       drawMsg();
       break;
    default:
      drawScope();
  }
  

  
  display.display();

  serialOutput();
  
  last_interrupt_time2 = interrupt_time2;
}
 
  
  if (Serial.available() > 0){

     char in[4];
     int index = 0;
     
     while (Serial.available() > 0) {

        in[index] = Serial.read();
        index++;
     
     }
     in[index] ='\0';
     ledWarn = atoi(in);
  }
  
  int btnState = digitalRead(btnPin);
  
  if ((current_mA >= ledWarn) || (btnState)){
   
    digitalWrite(LEDPIN, HIGH);
   
  }
  else {
    
     digitalWrite(LEDPIN, LOW);
  }

  if (btnState) setButtonMode(digitalRead(btnPin));

}


void drawBottomLine() {
  //Set x,y and print sensor data
  display.setCursor(0,25);
  display.print(loadvoltage);   display.print("V ");
  printJustified(current_mA);   display.print("mA ");
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
  // display.setCursor(104,0);
  // display.print((autoscale_limits[graph_MAX]+0.0)/1000);

   
}

// Draw current energy values and peak
// power
void drawEnergy() {
  display.setCursor(0,0);
  printJustified(milliwatthours);
  display.print("mWh ");
  printJustified(milliamphours);
  display.print("mAh");

  display.setCursor(0,8);
  display.print("Peak: ");
  display.print(voltageAtPeakPower*currentAtPeakPower/1000);
  display.print("W");
  display.setCursor(0,16);
  display.print("@ ");
  display.print(voltageAtPeakPower);
  display.print("V & ");
  display.print(currentAtPeakPower);
  display.print("mA");
  
  display.drawFastHLine(0,24,128,WHITE);

}


// Displays peak and minimum values
void drawPeakMins() {
  display.setCursor(0,0);
  display.print("Peak:");
  printJustified(peakCurrent);
  display.print("mA (");
  display.print(voltageAtPeakCurrent);
  display.print("V) ");
  display.setCursor(0,9);
  display.print("Min:");
  display.print(minVoltage);
  display.print("V (");
  display.print(currentAtMinVoltage);
  display.print("mA)");
  
  display.drawFastHLine(0,23,128,WHITE);

}

// Displays one big value & unit, with X number of decimals
// Display width is 7 digits wide, we leave the 2 rightmost digits
// for unit display
void drawBig(float val, char* unit, int decimals) {
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
void setMsg(char* msg, int time)
{
   if(msgDisplay == false){
     setDisplayTime = time;
     strcpy(setMsgDisplay, msg);
     msgDisplay=true;
     oldScreen = current_screen;
     current_screen=msgScreen;
   }
}
void drawMsg()
{
  static unsigned msgTime = 0;
  
  if (msgTime <= setDisplayTime){
  display.setCursor(0,0);
  display.setTextSize(3);
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

// Draw the complete graph:
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
  
  graph_Mem[ring_idx] = reading;
  ring_idx = (ring_idx+1)%GRAPH_MEMORY;
  for (int i=0; i < GRAPH_MEMORY; i++) {
    float val = 24 - mapf(graph_Mem[(i+ring_idx)%GRAPH_MEMORY], 0, autoscale_limits[graph_MAX], 0, 24);
    display.drawPixel(i, val , WHITE);
  }
}


  //Serial output for data logging with Java app
void serialOutput() {
  Serial.print(":");
  Serial.print(busvoltage);
  Serial.print(":");
  Serial.print(shuntvoltage);
  Serial.print(":");
  Serial.print(loadvoltage);
  Serial.print(":");
  Serial.print(current_mA);
  Serial.print(":");
  //Serial.print(freeRam()); //Was for checking free ram during development
  //Serial.print(":");
  Serial.print(digitalRead(btnPin));
  Serial.println(":");
  
}


// Right-justify values and round to nearest integer
void printJustified(float val)
{
  val = floor(val + 0.5);
  if (val < 1000) display.print(" ");
  if (val < 100) display.print(" ");
  if (val < 10) display.print(" ");
  
  display.print(val,0); 

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


void setButtonMode(int button){
  static unsigned long last_interrupt_time = 0;
  static uint8_t currMode;
  unsigned long interrupt_time = millis();
  static unsigned countBtn = 0;
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200)
  {
    current_screen = (current_screen + 1) % MAX_SCREENS;
    //setMsg(getScreenType(), 10); 
   }
 
  
  //Detect long button press, counting time button is pressed
  if (interrupt_time - last_interrupt_time < 100)
  {
    countBtn++;
 
    if (countBtn > 30000){
      setMsg("RESET", 10);
      peakCurrent = 0;
      voltageAtPeakCurrent = 0;
      minVoltage = 0;
      currentAtMinVoltage = 0;
      voltageAtPeakPower = 0;
      currentAtPeakPower = 0;
      milliwatthours = 0;
      milliamphours = 0;
    }          
    
  }
  else {
   countBtn = 0; 
  }
  
 last_interrupt_time = interrupt_time;

}



