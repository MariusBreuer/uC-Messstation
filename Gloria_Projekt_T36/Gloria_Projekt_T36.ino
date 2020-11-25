#include <UTFT.h>
#include <Encoder.h>
#include "InternalTemperature.h"
#include "C:\Users\User\Documents\Arduino\Gloria_Projekt_Shared\shared.h"

//defines
#define RS        4
#define WR        3
#define CS        18
#define REST      19
#define ExtButton 37
#define EncButton 26
#define EncA      27
#define EncB      28
#define cycle     100000
#define debounce  100  //time inbetween button presses
#define T4        Serial1

//namespace for menu
namespace menu{
  const uint8_t plot  = 0x00;
  const uint8_t home  = 0x01;
  const uint8_t calA  = 0x02;
  const uint8_t calB  = 0x03;
  const uint8_t calC  = 0x04;
  const uint8_t calF  = 0x05;
}

//namespace for custom colors
namespace c{
  const word red      = 0xF800;
  const word grn      = 0x07E0;
  const word blu      = 0x001F;
  const word first    = 0x07E3;
  const word second   = 0xF800;
  const word third    = 0x00FF;
  const word darkgrey = 0x4A69;
}

// ############################## CLASS INSTANCES ####################################
UTFT                      Display(SSD1963_800ALT, RS, WR, CS, REST);
Encoder                   knob(EncA, EncB);
IntervalTimer             EncButtonTimer;
IntervalTimer             ExtButtonTimer;
IntervalTimer             Blinking;
Buffer<Plot, 600>         PlotBuffer;

// ############################## VARIABLES ####################################
//buttons
volatile bool             encButtonEvent = false;     //set true by ISR, if sensitive, set false after event processed
volatile bool             extButtonEvent = false;     //set true by ISR, if sensitive, set false after event processed
volatile bool             encButtonSensitive = true;  //set true based on current menu
volatile bool             extButtonSensitive = true;  //set true based on current menu

//encoder
long                      encPosition = 0;            //remember the last position of encoder

//system Information
bool                      recording = false;    //is T4.0 recording?
uint8_t                   currentScreen = menu::plot; //the screen that is displayed right now
volatile char             msg[100];             //buffering recieved chars before processing
volatile int8_t           msgIndex = -1;        //last position filled in msg
volatile pltmeasurements  minimum = {0};
volatile pltmeasurements  current = {0};
volatile pltmeasurements  maximum = {0};
volatile sensorValues     sensors = {0};
volatile bool             CommsError = false;
volatile bool             RefreshPlot = false;
volatile bool             RefreshCalibration = false;
unsigned long             plotPaMax     = 1200000;
signed int                plotUmMax     = 26000;
unsigned long             plotUlminMax  = 500000;
Plot                      plotOld[600] = {0};
Plot                      plotNew[600] = {0};
volatile uint8_t          newMin = 0; //signal that there are new min/max values to display
volatile uint8_t          newMax = 0;
volatile bool             blinkingOn = true;
volatile char             mode = 0;
volatile bool             modeChangeFlag = false;
volatile bool             recordingErrorFlag = false;

//home menu
const String  MenuText[5] = {
  "Calibrate Pressure Sensor A",
  "Calibrate Pressure Sensor B",
  "Calibrate Pressure Sensor C",
  "Calibrate Flowrate Sensor",
  "back"
};

//pressure calibrating steps
const String PressureSteps[4] = {
  "1. Select calibration pressure",
  "2. Apply 0 bar to the sensor",
  "3. Apply calibration pressure",
  "4. Save new parameters"
};

//flow calibrating steps
const String FlowSteps[4] = {
  "1. Select calibration volume",
  "2. Begin calibration",
  "3. End calibration",
  "4. Save new parameters"
};

const String PaSelection[4] = {"1bar", "2bar", "5bar","10bar"};
const String FlowSelection[4] = {"0,25L","0,5L", "1,0L", "2,0L"};
const unsigned long PaSelectionValues[4] = {100000000, 200000000, 500000000, 1000000000}; //mbar
const unsigned long FlowSelectionValues[4] = {250000000, 500000000, 1000000000, 2000000000}; //nL
//Fonts for Display
extern uint8_t SevenSegNumFont[]; //32x50 pixels
extern uint8_t BigFont[]; // 16x16 pixels
extern uint8_t SmallFont[];
extern uint8_t Ubuntu[];
extern uint8_t OCR_A_Extended_M[];

//Colors for Display
const unsigned int Backgroundcolor = VGA_BLACK;
const unsigned int BaseColor = VGA_WHITE;
const unsigned int StopColor = VGA_RED;
const unsigned int StartColor = c::first;
const unsigned int SelectedColor = c::first;
const unsigned int UnselectedColor = c::darkgrey;

//other Display variables
const unsigned int Ysize = 120;
const unsigned int Xsize = 600;
const unsigned int Yoffset = 136;
const unsigned int Xoffset = 8;
const unsigned int PlotOffset = 150;

void setup() {
  
//setup the Display
  Display.InitLCD(LANDSCAPE);
  Display.clrScr();
  Display.setColor(VGA_BLACK);
  Display.fillRect(0, 0, 799, 479);
  Display.setBackColor(VGA_BLACK);
  Display.setFont(SmallFont);
  
//setup debugging
  Serial.begin(9600);

//setup comms to T4.0
  T4.begin(115200);

//setup periphery
  pinMode(ExtButton, INPUT_PULLUP);
  pinMode(EncButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncButton), EncButtonChangeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ExtButton), ExtButtonChangeISR, CHANGE);

//setup min and max measurements
  minimum.Pa1   = 0xFFFFFFFF;
  minimum.Pa2   = 0xFFFFFFFF;
  minimum.Pa3   = 0xFFFFFFFF;
  minimum.um    = 32767;
  minimum.ulmin = 0xFFFFFFFF;
  
  maximum.Pa1   = 0;
  maximum.Pa2   = 0;
  maximum.Pa3   = 0;
  maximum.um    = -32768;
  maximum.ulmin = 0;
}

void loop() {

  switch (currentScreen){
    case menu::plot:
    //Serial.print("calling PlotScreen()\n");
      currentScreen = PlotScreen();
      break;
    case menu::home:
      currentScreen = HomeScreen();
      break;
    case menu::calA:
      currentScreen = CalibrationScreen('a'); //calAScreen();
      break;
    case menu::calB:
      currentScreen = CalibrationScreen('b'); //calBScreen();
      break;
    case menu::calC:
      currentScreen = CalibrationScreen('c'); //calCScreen();
      break;
    case menu::calF:
      currentScreen = CalibrationScreen('f'); //calFScreen();
      break;
    default:
      currentScreen = menu::plot;
      break;
  }
}

/*
Display Page1:

  Plot Data       |     min     now     max
                  |
                  |
  ---------------- 
  home          record        InputOrder 
   
Display Page2 "home":

  Calibrate Pressure Sensor A
  Calibrate Pressure Sensor B
  Calibrate Pressure Sensor C
   Calibrate Flowrate Sensor
       Record Settings
            Back

Display Page3 "cal. pressure":

Display Page4 "cal. flowrate":

Display Page5 "record settings":
  
                          ^^^^^^   ^^^^^^^^^^^^^^   ^^ ^^ ^^ ^^ ^^ ^^ ^^
  Aufzeichnungsstart :    Quelle   größer/kleiner   11 22 33 44 55 66 77 [UNIT]     *
                          vvvvvv   vvvvvvvvvvvvvv   vv vv vv vv vv vv vv

                          ^^^^^   ^^ ^^ ^^
  Aufzeichnungsende :     Modus   11 22 33 Sekunden
                          vvvvv   vv vv vv

  >>Speichern<<

Quelle: Taste, Drucksensor, Messuhr, Durchflussmenge, Durchflussrate
größer oder kleiner als Wert X nicht bei Taste
* Komma bei Länge 12,34 mm, 1234567 Pa, 1234 ml, 1234 ml/min
Modus: Taste, zeitbegrenzt
wenn zeitbegrenzt dann auch Sekunden einstellen
  
  //wait for start + cycle to be over
  while(current+cycle > micros()){}
  current = micros();

  plot 3 graphs: pressure a+b+c, indicator length, flow rate.
  each measurement has to fit in a 150x600 rectangle, including display of time, min+max+current value
  leave 5 pixels space top and bottom = 140
  
  Plotfield x600, with 25Hz meaning 24 seconds of plot
  in front of plot show maximum, minimum and current value. String length is 7 digit or 5 digit with ','
  with 16x16 font the text is 112 pixels wide
*/

//#######################################################################################
//                                     Screens
//#######################################################################################

uint8_t PlotScreen(){
  encButtonSensitive = true;
  extButtonSensitive = true;
  RSTencButtonEvent();
  RSTextButtonEvent();
  //sent 'plotting' mode a to T4
  T4.print("a");
  Display.setColor(Backgroundcolor);
  Display.fillRect(0,0,799,479);
  for(int i=0; i<3; ++i){
    DisplayPlotFrame(0,i*150);
  }
  DisplayHomeButton(190,465, SelectedColor);
  DisplayStartButton(396,465);
  DisplayRecordingButton(600,465,c::darkgrey, c::darkgrey);
  //print all first text
  Display.setColor(c::first);
  Display.setFont(SmallFont);
  Display.print("[ 0,00 : 12,00]", 620, 2);
  Display.print("[ 25,47 : 25,47]", 620, 202);
  Display.print("[  0,00 : 500,00]", 620, 352);
  Display.setFont(Ubuntu);
  Display.print(",",684,  17);
  Display.print(",",684, 217);
  Display.print(",",684, 367);
  Display.setFont(OCR_A_Extended_M);
  Display.print("bar", 752, 25);
  Display.print("mm", 752, 225);
  Display.print("min",752, 388);
  Display.print("ml", 762, 359);
  Display.drawHLine(752,382,46);
  Display.drawHLine(752,383,46);
  //sensor icon
  unsigned int x = 700;
  unsigned int y = 165;
  Display.fillCircle(x,y,13);
  Display.setColor(Backgroundcolor);
  Display.fillCircle(x,y-6,3);
  Display.fillCircle(x-6,y,3);
  Display.fillCircle(x+6,y,3);

  //print all second text
  Display.setColor(c::second);
  Display.setFont(SmallFont);
  Display.print("[ 0,00 : 12,00]", 620, 52);
  Display.setFont(Ubuntu);
  Display.print(",",684,  67);
  Display.setFont(OCR_A_Extended_M);
  Display.print("bar", 752, 75);
  //sensor icon
  x += 41;
  Display.fillCircle(x,y,13);
  Display.setColor(Backgroundcolor);
  Display.fillCircle(x,y-6,3);
  Display.fillCircle(x-6,y,3);
  Display.fillCircle(x+6,y,3);
  
  //print all third text
  Display.setColor(c::third);
  Display.setFont(SmallFont);
  Display.print("[ 0,00 : 12,00]", 620, 102);
  Display.setFont(Ubuntu);
  Display.print(",",684,  117);
  Display.setFont(OCR_A_Extended_M);
  Display.print("bar", 752, 125);
  //sensor icon
  x += 39;
  Display.fillCircle(x,y,13);
  Display.setColor(Backgroundcolor);
  Display.fillCircle(x,y-6,3);
  Display.fillCircle(x-6,y,3);
  Display.fillCircle(x+6,y,3);
  
  //while no button was pressed remain in PlotScreen and show data
  while(encButtonEvent == false){
    
    cli();
    bool tmpRefreshPlot         = RefreshPlot;
    char tmpmode                = mode;
    bool tmpModeChangeFlag      = modeChangeFlag;
    bool tmpExtButtonEvent      = extButtonEvent;
    bool tmpRecordingErrorFlag  = recordingErrorFlag;
    sei();

    //if an error was registered during recording
    if(tmpRecordingErrorFlag == true){
      cli();
      Blinking.end();
      sei();
      DisplayStartButton(396,465);
      DisplayRecordingButton(600,465,c::darkgrey, c::darkgrey);
      RecordingFailedPopup();
      cli();
      recordingErrorFlag = false;
      sei();
    }
    
    //if the ext button was pressed
    if(tmpExtButtonEvent == true){
      //if mode 'a' (only plotting)
      if(tmpmode == 'a'){
        //change from a to b, disable home button sensitivity
        cli();
        encButtonSensitive = false;
        sei();
        //Serial.print("change from 'a' to 'b'\n");
        T4.print("b");
      }
      //else if mode 'b' (plotting+recording)
      else if (tmpmode == 'b'){
        //change from b to a, enable home button sensitivity
        cli();
        encButtonSensitive = true;
        sei();
        //Serial.print("change from 'b' to 'a'\n");
        T4.print("a");
      }
      RSTextButtonEvent();
    }
    //if mode change flag is set
    if(tmpModeChangeFlag == true){
      //if mode changed to 'b' (plotting+recording)
      if(tmpmode == 'b'){
        //Serial.print("update mode to b\n");
        DisplayHomeButton(190,465, UnselectedColor);
        DisplayStopButton(396,465);
        DisplayRecordingButton(600,465,VGA_WHITE, VGA_RED);
        Blinking.begin(BlinkingREC,750000);
      }
      //else if mode changed to 'a' (plotting)
      else if(tmpmode == 'a'){
        //Serial.print("update mode to a\n");
        DisplayHomeButton(190,465, SelectedColor);
        Blinking.end();
        DisplayStartButton(396,465);
        DisplayRecordingButton(600,465,c::darkgrey, c::darkgrey);
      }
      cli();
      modeChangeFlag = false;
      sei();
    }
    
    if(tmpRefreshPlot == true){
      cli();
      pltmeasurements tmpCur = {current.Pa1, current.Pa2, current.Pa3, current.um, current.ulmin};
      pltmeasurements tmpMin = {minimum.Pa1, minimum.Pa2, minimum.Pa3, minimum.um, minimum.ulmin};
      pltmeasurements tmpMax = {maximum.Pa1, maximum.Pa2, maximum.Pa3, maximum.um, maximum.ulmin};
      RefreshPlot = false;
      PlotBuffer.getAll(&plotNew[0]);
      sei();
      
      for(unsigned int i = 0; i<599; ++i){
        Display.setColor(Backgroundcolor);
        Display.drawVLine(8+i,   136-plotOld[i].a,   plotOld[i].a - plotOld[i+1].a+1);
        Display.drawVLine(8+i,   286-plotOld[i].d,   plotOld[i].d - plotOld[i+1].d+1);
        Display.drawVLine(8+i,   436-plotOld[i].e,   plotOld[i].e - plotOld[i+1].e+1);
        Display.drawVLine(8+i,   136-plotOld[i].b,   plotOld[i].b - plotOld[i+1].b+1);
        Display.drawVLine(8+i,   136-plotOld[i].c,   plotOld[i].c - plotOld[i+1].c+1);
        Display.setColor(c::first);
        Display.drawVLine(8+i,   136-plotNew[i].a,   plotNew[i].a - plotNew[i+1].a+1);
        Display.drawVLine(8+i,   286-plotNew[i].d,   plotNew[i].d - plotNew[i+1].d+1);
        Display.drawVLine(8+i,   436-plotNew[i].e,   plotNew[i].e - plotNew[i+1].e+1);
        Display.setColor(c::second);
        Display.drawVLine(8+i,   136-plotNew[i].b,   plotNew[i].b - plotNew[i+1].b+1);
        Display.setColor(c::third);
        Display.drawVLine(8+i,   136-plotNew[i].c,   plotNew[i].c - plotNew[i+1].c+1);
      }
      
      for(unsigned int i = 0; i<600; ++i){
        plotOld[i] = plotNew[i];
      }

      long whole = 0;
      long fraction = 0;

      //print all first text
      Display.setColor(c::first);

      //min/max values in SmallFont
      Display.setFont(SmallFont);

      whole = tmpMin.Pa1 / 100000;
      fraction = (tmpMin.Pa1 - whole*100000)/1000;
      Display.printNumI(whole,    628, 2, 2, ' ');  // print whole
      Display.printNumI(fraction, 652, 2, 2, '0');  // print fraction
      whole = tmpMax.Pa1 / 100000;
      fraction = (tmpMax.Pa1 - whole*100000)/1000;
      Display.printNumI(whole,    692, 2, 2, ' ');  // print whole
      Display.printNumI(fraction, 716, 2, 2, '0');  // print fraction
      
      whole = tmpMin.um / 1000;
      fraction = (tmpMin.um - whole*1000)/10;
      if(fraction < 0){fraction *= -1;}
      Display.printNumI(whole,    628, 202, 3, ' ');  // print whole
      Display.printNumI(fraction, 660, 202, 2, '0');  // print fraction
      whole = tmpMax.um / 1000;
      fraction = (tmpMax.um - whole*1000)/10;
      if(fraction < 0){fraction *= -1;}
      Display.printNumI(whole,    692, 202, 3, ' ');  // print whole
      Display.printNumI(fraction, 724, 202, 2, '0');  // print fraction

      whole = tmpMin.ulmin / 1000;
      fraction = (tmpMin.ulmin - whole*1000)/10;
      Display.printNumI(whole,    628, 352, 3, ' ');  // print whole
      Display.printNumI(fraction, 660, 352, 2, '0');  // print fraction
      whole = tmpMax.ulmin / 1000;
      fraction = (tmpMax.ulmin - whole*1000)/10;
      Display.printNumI(whole,    700, 352, 3, ' ');  // print whole
      Display.printNumI(fraction, 732, 352, 2, '0');  // print fraction

      //current values in Ubuntu
      Display.setFont(Ubuntu); // 24x32

      whole = tmpCur.Pa1 / 100000;
      fraction = (tmpCur.Pa1 - whole*100000)/1000;
      Display.printNumI(whole,    644, 17, 2, ' ');  // print whole
      Display.printNumI(fraction, 704, 17, 2, '0');  // print fraction

      whole = tmpCur.um / 1000;
      fraction = (tmpCur.um - whole*1000)/10;
      if(fraction < 0){fraction *= -1;}
      Display.printNumI(whole,    620, 217, 3, ' ');  // print whole
      Display.printNumI(fraction, 704, 217, 2, '0');  // print fraction

      whole = tmpCur.ulmin / 1000;
      fraction = (tmpCur.ulmin - whole*1000)/10;
      Display.printNumI(whole,    620, 367, 3, ' ');  // print whole
      Display.printNumI(fraction, 704, 367, 2, '0');  // print fraction

      //print all second text
      Display.setColor(c::second);
      Display.setFont(SmallFont);
      whole = tmpMin.Pa2 / 100000;
      fraction = (tmpMin.Pa2 - whole*100000)/1000;
      Display.printNumI(whole,    628, 52, 2, ' ');  // print whole
      Display.printNumI(fraction, 652, 52, 2, '0');  // print fraction
      whole = tmpMax.Pa2 / 100000;
      fraction = (tmpMax.Pa2 - whole*100000)/1000;
      Display.printNumI(whole,    692, 52, 2, ' ');  // print whole
      Display.printNumI(fraction, 716, 52, 2, '0');  // print fraction
      Display.setFont(Ubuntu);
      whole = tmpCur.Pa2 / 100000;
      fraction = (tmpCur.Pa2 - whole*100000)/1000;
      Display.printNumI(whole,    644, 67, 2, ' ');  // print whole
      Display.printNumI(fraction, 704, 67, 2, '0');  // print fraction

      //print all third text
      Display.setColor(c::third);
      Display.setFont(SmallFont);
      whole = tmpMin.Pa3 / 100000;
      fraction = (tmpMin.Pa3 - whole*100000)/1000;
      Display.printNumI(whole,    628, 102, 2, ' ');  // print whole
      Display.printNumI(fraction, 652, 102, 2, '0');  // print fraction
      whole = tmpMax.Pa3 / 100000;
      fraction = (tmpMax.Pa3 - whole*100000)/1000;
      Display.printNumI(whole,    692, 102, 2, ' ');  // print whole
      Display.printNumI(fraction, 716, 102, 2, '0');  // print fraction
      Display.setFont(Ubuntu);
      whole = tmpCur.Pa3 / 100000;
      fraction = (tmpCur.Pa3 - whole*100000)/1000;
      Display.printNumI(whole,    644, 117, 2, ' ');  // print whole
      Display.printNumI(fraction, 704, 117, 2, '0');  // print fraction
    }
    delay(1);
  }
  RSTencButtonEvent();
  return menu::home;
}

uint8_t HomeScreen(){
  encButtonSensitive = true;
  extButtonSensitive = false;
  RSTencButtonEvent();
  RSTextButtonEvent();
  encPosition = (knob.read()/4);
  uint8_t selectedIcon = 1;
  Display.setColor(Backgroundcolor);
  Display.fillRect(0,0,799,479);
  SensorIcon(0);
  for(int i=1; i<6; ++i){
    DisplayHomeIcon(!(selectedIcon-i),400,60*i,MenuText[i-1]);
  }
  //while encoder button is not pushed keep checking
  while(encButtonEvent == false){
    if(encPosition > (knob.read()/4)){
      if(selectedIcon < 5){
        //display previous icon unselected
        DisplayHomeIcon(false,400,60*selectedIcon,MenuText[selectedIcon-1]);
        ++selectedIcon;
        //display new icon selected
        DisplayHomeIcon(true, 400,60*selectedIcon,MenuText[selectedIcon-1]);
        //update sensor icons
        SensorIcon(selectedIcon-1);
      }
      encPosition = (knob.read()/4);
      delay(10);
    }
    if(encPosition < (knob.read()/4)){
      if(selectedIcon > 1){
        //display previous icon unselected        
        DisplayHomeIcon(false,400,60*selectedIcon,MenuText[selectedIcon-1]);
        --selectedIcon;
        //display new icon selected
        DisplayHomeIcon(true, 400,60*selectedIcon,MenuText[selectedIcon-1]);
        //update sensor icons
        SensorIcon(selectedIcon-1);
      }
      encPosition = (knob.read()/4);
      delay(10);
    }
  }
  RSTencButtonEvent();
  switch (selectedIcon){
    case 1:
      return menu::calA;
      break;
    case 2:
      return menu::calB;
      break;
    case 3:
      return menu::calC;
      break;
    case 4:
      return menu::calF;
      break;
    case 5:
      return menu::plot;
      break;
    default:
      return menu::home;
      break;
  }
}

//#######################################################################################
//                                Display helper functions
//#######################################################################################

uint8_t CalibrationScreen(char sensor){
  //Serial.print("CalibrationScreen(");
  //Serial.print(sensor);
  //Serial.print(")\n");
  if((sensor < 'a' || sensor > 'c') && sensor != 'f'){return menu::plot;}
  cli();
  encButtonSensitive = true;
  extButtonSensitive = true;
  sei();
  RSTencButtonEvent();
  RSTextButtonEvent();
  //request sensor data from T4
  T4.print("c");
  //background
  Display.setColor(Backgroundcolor);
  Display.fillRect(0,0,799,479);
  //print title
  DisplayNextButton();
  DisplayCancelButton();
  uint8_t sensor_numeric = sensor - 0x61; // a b c = 0 1 2
  if(sensor >= 'a' && sensor <= 'c'){
    DisplayPrintSteps('p',0);
    DisplayCalSelection('p',1); // 1 = default first selected
    //print voltage 0,00V
    Display.setFont(Ubuntu);
    Display.setColor(VGA_WHITE);
    Display.print(",",   674, 438);
    Display.print("V",   766, 438);
    Display.printNumI(0, 658, 438);
    Display.printNumI(0, 694, 438, 2, '0');
  }
  else{
    sensor_numeric = 3; // f = 3
    DisplayPrintSteps('f',0);
    DisplayCalSelection('f',1); // 1 = default first selected
    //flow count is printed when calibration starts, not before
    Display.setFont(Ubuntu);
    Display.setColor(VGA_WHITE);
  }
  Display.print(MenuText[sensor_numeric], 10, 10);
  uint8_t index = 0;
  bool tmpRefreshCalibration = false;
  sensorValues tmpValues = { 0 };
  encPosition = (knob.read()/4);
  
  unsigned long LowFlowcount = 0;
  unsigned long HighFlowcount = 0;
  unsigned long ReferenceVolume = 0;
  unsigned long whole     = 0;
  unsigned long fraction  = 0;
  uint8_t       ReferenceLevel = 1; //0 = no reference, hence start with 1
  unsigned long LowPressure12bit = 0;
  unsigned long HighPressure12bit = 0;
  unsigned long ReferencePressure = 0;
  
  while(index < 5 && extButtonEvent == false){
    
    if(encButtonEvent == true){
      RSTencButtonEvent();
      ++index;
      switch(index){
      case 1:
        //select reference value
        if(sensor_numeric >= 0 && sensor_numeric <= 2){
          //selected reference Pressure
          switch(ReferenceLevel){
          case 1:
            ReferencePressure = PaSelectionValues[0]; //mPa = 1 bar
            break;
          case 2:
            ReferencePressure = PaSelectionValues[1]; //mPa = 2 bar
            break;
          case 3:
            ReferencePressure = PaSelectionValues[2]; //mPa = 5 bar
            break;
          case 4:
            ReferencePressure = PaSelectionValues[3]; //mPa = 10 bar
            break;
          default:
            break;
          }
          Serial.print("Selected calibration pressure: ");
          Serial.print(ReferencePressure);
          Serial.print(" mPa\n");
          //switch to step #2
          DisplayPrintSteps('p',1);
        }
        else{
          //selected reference volume
          switch(ReferenceLevel){
          case 1:
            ReferenceVolume = FlowSelectionValues[0]; //nl = 0.5 liter
            break;
          case 2:
            ReferenceVolume = FlowSelectionValues[1]; //nl = 1 liter
            break;
          case 3:
            ReferenceVolume = FlowSelectionValues[2]; //nl = 2 liter
            break;
          case 4:
            ReferenceVolume = FlowSelectionValues[3]; //nl = 4 liter
            break;
          default:
            break;
          }
          Serial.print("Selected calibration volume: ");
          Serial.print(ReferenceVolume);
          Serial.print(" ul\n");
          //switch to step #2
          DisplayPrintSteps('f',1);
        }
        break;
      //case 2 handeled outside of if(encButtonEvent == true) because it needs updating from encPosition
      case 2:
        //record first sensor value
        cli();
        tmpValues = {sensors.Pa1, sensors.Pa2, sensors.Pa3, sensors.flowcount};
        RefreshCalibration = false;
        switch(sensor){
        case 'a':
          LowPressure12bit = tmpValues.Pa1;
          break;
        case 'b':
          LowPressure12bit = tmpValues.Pa2;
          break;
        case 'c':
          LowPressure12bit = tmpValues.Pa3;
          break;
        case 'f':
          LowFlowcount = tmpValues.flowcount;
        default:
          break;
        }        
        //switch to step #3
        if(sensor == 'f'){
          Serial.print("Recording sensor value for low Flow: ");
          Serial.print(LowFlowcount);
          Serial.print("ul\n");
          DisplayPrintSteps('f',2);
        }
        else{
          Serial.print("Recording sensor value for low Pressure: ");
          Serial.print(LowPressure12bit);
          Serial.print("bit\n");
          DisplayPrintSteps('p',2);
        }
        //DisplayCalSelection('p',ReferencePressureLevel);
        break;
      case 3:
        //record second sensor value
        cli();
        tmpValues = {sensors.Pa1, sensors.Pa2, sensors.Pa3, sensors.flowcount};
        RefreshCalibration = false;
        switch(sensor){
        case 'a':
          HighPressure12bit = tmpValues.Pa1;
          break;
        case 'b':
          HighPressure12bit = tmpValues.Pa2;
          break;
        case 'c':
          HighPressure12bit = tmpValues.Pa3;
          break;
        case 'f':
          HighFlowcount = tmpValues.flowcount;
        default:
          break;
        }        
        //switch to step #4
        if(sensor == 'f'){
          Serial.print("Recording sensor value for high Flow: ");
          Serial.print(HighFlowcount);
          Serial.print("ul\n");
          DisplayPrintSteps('f',3);
        }
        else{
          Serial.print("Recording sensor value for high Pressure: ");
          Serial.print(HighPressure12bit);
          Serial.print("bit\n");
          DisplayPrintSteps('p',3);
        }
        break;
      case 4:
        //encButtonPressed during step #4 (saving data)
        //calculate parameters
        if(sensor == 'f'){
          unsigned long delta = HighFlowcount - LowFlowcount;
          unsigned long Factor = ReferenceVolume / delta; // [nl/t]
          UpdateCalibration(6, Factor); //7th parameter
        }
        else{
          unsigned int delta = HighPressure12bit - LowPressure12bit;
          unsigned long Factor = ReferencePressure / delta; // [mPa/bit]
          unsigned long Offset = (Factor * LowPressure12bit) / 1000; // [Pa]
          UpdateCalibration(2*sensor_numeric, Factor);      //1st, 3rd, 5th parameter
          UpdateCalibration(2*sensor_numeric + 1, Offset);  //2nd, 4th, 6th parameter
        }
        return menu::plot;
      }
    }
    if(index == 0){
      if(encPosition > (knob.read()/4) && ReferenceLevel < 4){
        ++ReferenceLevel;
        if(sensor == 'f'){
          DisplayCalSelection('f',ReferenceLevel);
        }
        else{
          DisplayCalSelection('p',ReferenceLevel);
        }
      }
      else if(encPosition < (knob.read()/4) && ReferenceLevel > 1){
        --ReferenceLevel;
        if(sensor == 'f'){
          DisplayCalSelection('f',ReferenceLevel);
        }
        else{
          DisplayCalSelection('p',ReferenceLevel);
        }
      }
      encPosition = knob.read()/4;
    }
    cli();
    tmpRefreshCalibration = RefreshCalibration;
    sei();
    if(tmpRefreshCalibration == true){
      cli();
      tmpValues = {sensors.Pa1, sensors.Pa2, sensors.Pa3, sensors.flowcount};
      RefreshCalibration = false;
      sei();
      if(sensor == 'f'){
        Display.setColor(BaseColor);
        Display.setFont(Ubuntu);
        Display.printNumI(tmpValues.flowcount - LowFlowcount, 622, 438, 7, ' ');
      }
      else{
        switch(sensor){
        case 'a':
          whole     = tmpValues.Pa1/1241.21;
          fraction  = tmpValues.Pa1/1.24121 - whole*1000;
          break;
        case 'b':
          whole     = tmpValues.Pa2/1241.21;
          fraction  = tmpValues.Pa2/1.24121 - whole*1000;
          break;
        case 'c':
          whole     = tmpValues.Pa3/1241.21;
          fraction  = tmpValues.Pa3/1.24121 - whole*1000;
          break;
        default:
          break;
        }
        Display.setColor(BaseColor);
        Display.setFont(Ubuntu);
        Display.printNumI(whole,    658, 438);
        Display.printNumI(fraction, 694, 438);
      }

    }
    delay(2);
  }
  return menu::plot;
}

//sensortype p = pressure, f = flow, other = return
//steps start at nr = 0 = first step
void DisplayPrintSteps(char sensortype, uint8_t nr){
  if(nr > 3  || (sensortype != 'p' && sensortype != 'f')){return;}
  const unsigned int x = 50;   //150
  const unsigned int y[4] = {100,200,250,300};
  Display.setFont(Ubuntu);
  for(unsigned int i=0; i<4; ++i){
    if(i == nr){
      Display.setColor(SelectedColor);
    }
    else{
      Display.setColor(UnselectedColor);
    }
    if(sensortype == 'p'){
      Display.print(PressureSteps[i], x, y[i]);
    }
    else{
      Display.print(FlowSteps[i], x, y[i]);
    }
  }
}

//sensortype p = pressure, f = flow, other = return
//selected 0 = non, 1-4 available selections, 5 = return
void DisplayCalSelection(char sensortype, uint8_t selected){
  if(selected > 4){return;}
  //const unsigned int x = 100; //[4] = {100, 270, 440, 610};
  const unsigned int y = 150;
  unsigned int strLen[4] = { 0 };
  unsigned int x[4] = {100,0,0,0};
  //5 x 24 = 120 => 120+50
  if(sensortype == 'p'){
    for(int i=0; i<3; ++i){
      strLen[i] = PaSelection[i].length()*24;
      x[i+1] = x[i] + strLen[i] + 50;
    }
    strLen[3] = PaSelection[3].length()*24;
  }
  else{
    for(int i=0; i<3; ++i){
      strLen[i] = FlowSelection[i].length()*24;
      x[i+1] = x[i] + strLen[i] + 50;
    }
    strLen[3] = FlowSelection[3].length()*24;
  }
  Display.setFont(Ubuntu);
  for(unsigned int i=0; i<4; ++i){
    if(i == selected-1){
      Display.setColor(SelectedColor);
      BoldRect(x[i]+strLen[i]/2,y+12,strLen[i]+20,50,SelectedColor); //x[i]+60
    }
    else{
      Display.setColor(UnselectedColor);
      BoldRect(x[i]+strLen[i]/2,y+12,strLen[i]+20,50,UnselectedColor); //x[i]+60
    }
    if(sensortype == 'p'){
      Display.print(PaSelection[i], x[i], y);
    }
    else{
      Display.print(FlowSelection[i], x[i], y);
    }
  }
}

void DisplayNextButton(){
  unsigned int x = 190;
  unsigned int y = 465; 
  BoldRect(x,y,72,22, SelectedColor);
  Display.setFont(BigFont);
  Display.print("NEXT",x-32,y-8);
}

void DisplayCancelButton(){
  unsigned int x = 396;
  unsigned int y = 465; 
  BoldRect(x,y,104,22, SelectedColor);
  Display.setFont(BigFont);
  Display.print("CANCEL",x-48,y-8);
}

void SensorIcon(uint8_t index){
  unsigned int x[4] = {240,420,600,770};
  const unsigned int y = 440;
  unsigned int OD = 30;
  //display pressure sensor icons
  for(unsigned int i=0; i<3; ++i){
    if(i == index){
      Display.setColor(SelectedColor);
    }
    else{
      Display.setColor(UnselectedColor);
    }    
    Display.fillCircle(x[i],y,OD/2);     //display big circle
    Display.setColor(Backgroundcolor);  //display smaller circles
    Display.fillCircle(x[i],y-OD/4,OD/7);
    Display.fillCircle(x[i]-OD/4,y,OD/7);
    Display.fillCircle(x[i]+OD/4,y,OD/7);
  }
  //display flow sensor icon
  if(index == 3){
    Display.setColor(SelectedColor);
  }
  else{
    Display.setColor(UnselectedColor);
  }
  OD = 40;
  Display.fillCircle(x[3],y,OD/2);     //display big circle
  Display.setColor(Backgroundcolor);  //display smaller circles
  Display.fillCircle(x[3],         y,        OD/9);
  Display.fillCircle(x[3],         y+OD/3.5, OD/9);
  Display.fillCircle(x[3],         y-OD/3.5, OD/9);
  Display.fillCircle(x[3]-OD/3.5,  y,        OD/9);
  Display.fillCircle(x[3]+OD/3.5,  y,        OD/9);
  Display.fillCircle(x[3]+OD/3,    y-OD/3,   OD/9);
}

//display warning that recording failed and only return once user confirmed
void RecordingFailedPopup(){
  word oldBackcolor = Display.getBackColor();
  unsigned int x = 300;
  unsigned int y = 220;
  FilledBoldRect(x,y,500,100, BaseColor, VGA_RED);
  Display.setBackColor(VGA_RED);
  Display.setColor(VGA_WHITE);
  Display.setFont(Ubuntu);
  //17 digits, 8.5 out of center, 24 pixels wide = 8.5*24 = 204
  Display.print("Recording failed!",x-204,y-32);
  Display.setFont(BigFont);
  //28 digits, 14 out of center, 16 pixels wide = 14*16 = 224
  Display.print("Press any button to continue",x-224,y+4);
  Display.setBackColor(oldBackcolor);
  cli();
  encButtonSensitive = true;
  extButtonSensitive = true;
  sei();
  RSTencButtonEvent();
  RSTextButtonEvent();
  bool tmpEncEvent = false;
  bool tmpExtEvent = false;
  while(tmpEncEvent == false && tmpExtEvent == false){
    cli();
    tmpEncEvent = encButtonEvent;
    tmpExtEvent = extButtonEvent;
    sei();
    delay(1);
  }
  RSTencButtonEvent();
  RSTextButtonEvent();
  for(int i=0; i<3; ++i){
    DisplayPlotFrame(0,i*150);
  }
}

//display home button around center xy
void DisplayHomeButton(unsigned int x, unsigned int y, unsigned int color){
  BoldRect(x,y,72,22, color);
  Display.setFont(BigFont);
  Display.print("HOME",x-32,y-8);
}

//display stop button around center xy
void DisplayStopButton(unsigned int x, unsigned int y){
  Display.setColor(Backgroundcolor);
  Display.fillRect(x-44,y-11,x+44,y+11);
  BoldRect(x,y,72,22, StopColor);
  Display.setFont(BigFont);
  Display.print("STOP",x-32,y-8);
}

//display start button around center xy
void DisplayStartButton(unsigned int x, unsigned int y){
  BoldRect(x,y,88,22, StartColor);
  Display.setFont(BigFont);
  Display.print("START",x-40,y-8);
}

//display recording button around center xy
void DisplayRecordingButton(unsigned int x, unsigned int y, unsigned int color, unsigned int circlecolor){
  BoldRect(x,y,72,22,color);//168,22,color);
  Display.setFont(BigFont);
  Display.print("rec",x-16, y-8);
  Display.setColor(circlecolor);
  Display.fillCircle(x-24,y,7);
}

void DisplayHomeIcon(bool selected, unsigned int x, unsigned int y, String text){
  unsigned int drawcolor = UnselectedColor;
  if(selected == true){
    drawcolor = SelectedColor;
  }
  BoldRect(x,y,24*text.length()+10,50,drawcolor);
  Display.setFont(Ubuntu);
  Display.print(text,x-(12*text.length()),y-16);
}

void BoldRect(unsigned int x, unsigned int y, unsigned int sizeX, unsigned int sizeY, unsigned int color){
  unsigned int xh = sizeX/2;
  unsigned int yh = sizeY/2;
  Display.setColor(Backgroundcolor);
  Display.fillRect(x-xh,y-yh,x+xh,y+yh);
  Display.setColor(color);
  Display.drawHLine(x-xh,   y-yh+1, sizeX-1);
  Display.drawHLine(x-xh+1, y-yh,   sizeX-3);
  Display.drawHLine(x-xh,   y+yh-1, sizeX-1);
  Display.drawHLine(x-xh+1, y+yh,   sizeX-3);
  Display.drawVLine(x-xh,   y-yh+2, sizeY-3);
  Display.drawVLine(x-xh+1, y-yh+2, sizeY-3);
  Display.drawVLine(x+xh,   y-yh+2, sizeY-3);
  Display.drawVLine(x+xh-1, y-yh+2, sizeY-3);
  Display.drawPixel(x-xh+2, y-yh+2);
  Display.drawPixel(x+xh-2, y-yh+2);
  Display.drawPixel(x-xh+2, y+yh-2);
  Display.drawPixel(x+xh-2, y+yh-2);
}

void FilledBoldRect(unsigned int x, unsigned int y, unsigned int sizeX, unsigned int sizeY, unsigned int color, unsigned int fillcolor){
  unsigned int xh = sizeX/2;
  unsigned int yh = sizeY/2;
  Display.setColor(fillcolor);
  Display.fillRect(x-xh+1,y-yh+1,x+xh-1,y+yh-1);
  for(int i=-1; i<2; i=i+2){
    for(int j=-1; j<2; j=j+2){
      Display.drawPixel(x + (i* xh),      y + (j* yh));
      Display.drawPixel(x + (i* (xh-1)),  y + (j* yh));
      Display.drawPixel(x + (i* xh),      y + (j* (yh-1)));
    }
  }
  Display.setColor(color);
  Display.drawHLine(x-xh,   y-yh+1, sizeX-1);
  Display.drawHLine(x-xh+1, y-yh,   sizeX-3);
  Display.drawHLine(x-xh,   y+yh-1, sizeX-1);
  Display.drawHLine(x-xh+1, y+yh,   sizeX-3);
  Display.drawVLine(x-xh,   y-yh+2, sizeY-3);
  Display.drawVLine(x-xh+1, y-yh+2, sizeY-3);
  Display.drawVLine(x+xh,   y-yh+2, sizeY-3);
  Display.drawVLine(x+xh-1, y-yh+2, sizeY-3);
  Display.drawPixel(x-xh+2, y-yh+2);
  Display.drawPixel(x+xh-2, y-yh+2);
  Display.drawPixel(x-xh+2, y+yh-2);
  Display.drawPixel(x+xh-2, y+yh-2);
}

//plotting pixel bottom left is X+8, Y+136
void DisplayPlotFrame(unsigned int x, unsigned int y){
  //fill background
  Display.setColor(Backgroundcolor);
  Display.fillRect(x,y,x+600,y+150);
  //draw vertical and horizontal base lines
  Display.setColor(BaseColor);
  Display.drawLine(x+6,y+5,x+6,y+145);
  Display.drawLine(x+7,y+5,x+7,y+145);
  Display.drawLine(x,y+137,x+615,y+137);
  Display.drawLine(x,y+138,x+615,y+138);
  //draw arrow head top
  for(int i=0; i<3; ++i){
    Display.drawLine(x+4-i,y+6+i,x+8+i,y+6+i);
  }
  //draw arrow head right
  for(int i=0; i<3; ++i){
    Display.drawLine(x+614-i,y+136-i,x+614-i,y+140+i);
  }
}

//#######################################################################################
//                                      ISR functions
//#######################################################################################

void BlinkingREC(){
  cli();
  //get before status
  unsigned int color = Display.getColor();
  uint8_t * oldFont = Display.getFont();
  if(blinkingOn == true){
    DisplayRecordingButton(600,465,VGA_WHITE, VGA_RED);
  }
  else{
    DisplayRecordingButton(600,465,VGA_RED, VGA_RED);
  }
  blinkingOn = !blinkingOn;
  Display.setFont(oldFont);
  Display.setColor(color);
}

void EncButtonChangeISR(){
  if(digitalReadFast(EncButton) == true){EncButtonTimer.end();}
  else{EncButtonTimer.begin(EncButtonTimerISR, debounce);}
}

void EncButtonTimerISR(){
  cli();
  if(encButtonSensitive == true){
    encButtonEvent = true;
    //++encButtonCount;
  }
  EncButtonTimer.end();
  sei();
}

void ExtButtonChangeISR(){
  if(digitalReadFast(ExtButton) == true){ExtButtonTimer.end();}
  else{ExtButtonTimer.begin(ExtButtonTimerISR, debounce);}
}

void ExtButtonTimerISR(){
  cli();
  if(extButtonSensitive == true){
    extButtonEvent = true;
    //++extButtonCount;
  }
  ExtButtonTimer.end();
  sei();
}

//reset ButtonEvent
void RSTencButtonEvent(){
  cli();
  encButtonEvent = false;
  sei();
}

void RSTextButtonEvent(){
  cli();
  extButtonEvent = false;
  sei();
}

void serialEvent1(){
  cli();
  char recieved = T4.read();
  //'[' signals the beginning of a new transmission
  if(recieved == '['){
    msgIndex = -1; //omit '[' from msg[]
    for(unsigned int i=0; i<100; ++i){
      msg[i] = 0x00;
    }
  }
  else if(recieved == ']'){
    //recieved end of message, analyse now
    switch(msg[0]){
    case 'a':
      //Serial.print("case a");
      //T4 in plotting mode
      AnalyseMsg('p');
      if(mode != 'a'){
        mode = 'a';
        modeChangeFlag = true;
        //Serial.print("#### change to a ####");
      }
      break;
    case 'b':
      //Serial.print("case b");
      //T4 in plotting (+recording) mode
      AnalyseMsg('p');
      if(mode != 'b'){
        mode = 'b';
        modeChangeFlag = true;
        //Serial.print("#### change to b ####");
      }
      break;
    case 'c':
      //Serial.print("case c");
      //T4 in calibration mode
      AnalyseMsg('s');
      if(mode != 'c'){
        mode = 'c';
        modeChangeFlag = true;
        //Serial.print("#### change to c ####");
      }
      break;
    case 'e':
      //T4 error during recording (mode b), default to mode a
      recordingErrorFlag = true;
      mode = 'a';
      modeChangeFlag = true;
      //Serial.print("####################### ERROR ####################\n");
      break;
    case '0':
      //Serial.print("case 0\n");
      //T4 in default mode (ie after startup), put in plotting mode
      mode = 'a';
      modeChangeFlag = true;
      T4.print("a");
    default:
      CommsError = true;
      //Serial.print("default\n");
      break;
    }
  }
  else{
    ++msgIndex;
    if(msgIndex < 100){
      msg[msgIndex] = recieved;
    }
    else{
      //unhandled error for msg recieve overflow. Expected ']' at least every 100 chars
    }
  }
  sei();
}

//#######################################################################################
//                                   OTHER FUNCTIONS
//#######################################################################################

void UpdateCalibration(uint8_t index, unsigned long value){
  Serial.print("requested to change Parameter #");
  Serial.print(index);
  Serial.print(" to ");
  Serial.print(value);
  Serial.print("\n");
  T4.clear();
  T4.print("[");
  T4.print(index);
  T4.print(",");
  T4.print(value);
  T4.print("]");
  T4.flush();
}

void AnalyseMsg(char m){
  //Serial.print("analysing msg\n");
  //T4 in plotting mode
  long      tmp = 0;
  uint8_t   end = 0;
  if(m == 'p'){
    //msg[0] @      V
    //message:     [p733,0,0,-7780,0,0,0]
    bool ok = getNumber(&tmp, 1, &end);
    current.Pa1 = tmp;
    if(ok == true){
      //Serial.print("Pa1 OK\t\t");
      ok = getNumber(&tmp, end+1, &end);
      current.Pa2 = tmp;
    }
    if(ok == true){
      //Serial.print("Pa2 OK\t\t");
      ok = getNumber(&tmp, end+1, &end);
      current.Pa3 = tmp;
    }
    if(ok == true){
      //Serial.print("Pa3 OK\t\t");
      ok = getNumber(&tmp, end+1, &end);
      current.um = tmp;
    }
    if(ok == true){
      //Serial.print("um OK\t\t");
      ok = getNumber(&tmp, end+1, &end);
      current.ulmin = tmp;
    }
    if(ok == true){
      //Serial.print("ulmin OK\t\t");
      if(msg[end+1] == '1'){
        recording = true;
      }
      else{
        recording = false;
      }
    }
    //Serial.print("\n");
    if(ok == false){
      CommsError = true;
    }
    UpdateMinMax();
    UpdatePlotBuffer();
    RefreshPlot = true;
  }
  //T4 in calibration mode
  else if(m == 's'){
    //msg[0] @      Vsensors
    //message:     [s356,0,0,0]
    bool ok = getNumber(&tmp, 1, &end);
    sensors.Pa1 = tmp;
    if(ok == true){
      ok = getNumber(&tmp, end+1, &end);
      sensors.Pa2 = tmp;
    }
    if(ok == true){
      ok = getNumber(&tmp, end+1, &end);
      sensors.Pa3 = tmp;
    }
    if(ok == true){
      ok = getNumber(&tmp, end+1, &end);
      sensors.flowcount = tmp;
    }
    if(ok == false){
      CommsError = true;
    }
    //Serial.print("new sensor data recieved\n");
    RefreshCalibration = true;
  }
}

//update minimum and maximum measurements if current is smaller/larger, used in serial event ISR
//ignore ul microliter because this unit will only climb, it will also not be plotted
void UpdateMinMax(){
  if(current.Pa1   > maximum.Pa1  ){maximum.Pa1   = current.Pa1  ; newMax |= 0x01;}
  if(current.Pa2   > maximum.Pa2  ){maximum.Pa2   = current.Pa2  ; newMax |= 0x02;}
  if(current.Pa3   > maximum.Pa3  ){maximum.Pa3   = current.Pa3  ; newMax |= 0x04;}
  if(current.um    > maximum.um   ){maximum.um    = current.um   ; newMax |= 0x08;}
  if(current.ulmin > maximum.ulmin){maximum.ulmin = current.ulmin; newMax |= 0x10;}
  if(current.Pa1   < minimum.Pa1  ){minimum.Pa1   = current.Pa1  ; newMin |= 0x01;}
  if(current.Pa2   < minimum.Pa2  ){minimum.Pa2   = current.Pa2  ; newMin |= 0x02;}
  if(current.Pa3   < minimum.Pa3  ){minimum.Pa3   = current.Pa3  ; newMin |= 0x04;}
  if(current.um    < minimum.um   ){minimum.um    = current.um   ; newMin |= 0x08;}
  if(current.ulmin < minimum.ulmin){minimum.ulmin = current.ulmin; newMin |= 0x10;}
}

void UpdatePlotBuffer(){
  Plot plt = {0};
  long tmp = (current.Pa1 * 128)/ plotPaMax;
  if(tmp > 128){tmp = 128;}
  plt.a = tmp;
  tmp = (current.Pa2 * 128)/ plotPaMax;
  if(tmp > 128){tmp = 128;}
  plt.b = tmp;
  tmp = (current.Pa3 * 128)/ plotPaMax;
  if(tmp > 128){tmp = 128;}
  plt.c = tmp;
  tmp = current.um * 64;
  tmp = (tmp / plotUmMax)+64;
  if(tmp > 128){tmp = 128;}
  else if(tmp < 0){tmp = 0;}
  plt.d = tmp;
  tmp = (current.ulmin * 128)/ plotUlminMax;
  if(tmp > 128){tmp = 128;}
  plt.e = tmp;
/*  
  Serial.print("a");
  Serial.print(plt.a);
  Serial.print("\tb");
  Serial.print(plt.b);
  Serial.print("\tc");
  Serial.print(plt.c);
  Serial.print("\td");
  Serial.print(plt.d);
  Serial.print("\te");
  Serial.print(plt.e);
  Serial.print("\n");
*/  
  PlotBuffer.setNext(plt);
}

//return true if a number was found and terminated with ',', safe the position of termination char ',' in *end
//place the number in *result and get the start position within msg
bool getNumber(signed long *result, uint8_t start, uint8_t * end){
  *end = 101;
  bool positive = true;
  //determine the sign of the number
  if(msg[start] == '-'){
    positive = false;
    ++start;
  }
  //find the ',' or ']' within msg
  for(uint8_t i = start; i<100; ++i){
    if(msg[i] == ',' || msg[i] == 0x00){
      *end = i;
      break;
    }
    //if msg[i] is neither terminating char, nor digit 0-9 leave function
    else if(msg[i] < '0' && msg[i] > '9'){
      return false;
    }
  }
  //leave if the termination char wasn't found within 10 chars, the longest number expected is 7 digits
  if( *end - start > 10){return false;}
  unsigned long factor = 1;
  * result = 0;
  for(uint8_t i = *end-1; i >= start; --i){
    * result += (msg[i] - 0x30) * factor;
    factor *= 10;
  }
  if(positive == false){
    * result = - * result;
  }
  return true;
}
