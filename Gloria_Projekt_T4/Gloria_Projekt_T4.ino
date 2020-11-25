//includes
#include <EEPROM.h>
#include "SdFat.h"
#include <SPI.h>
#include "C:\Users\User\Documents\Arduino\Gloria_Projekt_Shared\shared.h"

//16500 us * 2 = 31000 us => ca. 32Hz
//64 toggle pro sekunde
//3840 toggle pro minute

//1925 impulse pro liter
//3850 toggle pro liter
//259.74 microliter pro toggle

//3840 t/min * 259.74 ul/t
//997.402 ul/min

/*
259.74 ul/t  /  16500us/t = 0,01575 ul/us |*60mio
                          = 945000 ul/min

0.5 liter / minute
1925 toggle / minute
32 toggle / sekunde
31250 us / toggle
*/

//defines
#define flowsensor            23
#define Sensor1               A2
#define Sensor2               A1
#define Sensor3               A0
#define maxflow               18 // max. Hz on flow sensor
#define minFlowTime           1000000/maxflow/2 // convert maxflow to us, not jet done!
#define cycle                 100000 // for testing only, 10 Hz
#define Dial                  Serial1
#define T36                   Serial2
#define FILE_BASE_NAME        "Data"

//sampling speed
const unsigned long           sample_interval = 10000; //100Hz = 10000 us

// ########################### SENSOR VARIABLES #################################

volatile unsigned long        flowcount = 0;  //volatile unsigned long  flowcount = 0;  //count of toggles on the flow sensor
volatile float                flow_ul = 0;
volatile uint64_t             lastchange = 0; //volatile unsigned long  lastchange = 0; //last time of toggle
volatile float                flowrate_ulmin = 0;
sensorValues                  rawValues = { 0 };
volatile signed int           micrometer = 0;
volatile char                 readout[5] = {'0','0','0','0','0'};
volatile int                  msgIndex = -1;

// ########################### TIMING VARIABLES #################################

uint64_t                      upperLong = 0;  //used in micros64bit
uint32_t                      lastmicros = 0; //used in micros64bit

// ########################### OTHER VARIABLES ##################################

char                          lastchar = 0;
char                          mode = 0;
unsigned long                 MeasurementIndex = 0;
bool                          SDpresent = false;
bool                          recording = false;
calibration                   c = { 0 };
bool                          skipread = false;

// ########################### CLASS INSTANCES ##################################
//Sd2Card                     card;
//SdVolume                    volume;
//SdFile                      root;
//File                        loggingFile;

IntervalTimer                 Timer;
IntervalTimer                 FlowrateStopper;
Buffer<logmeasurements, 500>  MeasurementBuffer;

void setup() {
  //serial ports
  Dial.begin(9600);
  T36.begin(115200);

  //sensors
  analogReadResolution(12);
  analogReadAveraging(4);
  attachInterrupt(digitalPinToInterrupt(flowsensor), FlowISR, CHANGE);
  
  //debug
  Serial.begin(9600);
  /*
  c.Pa1factor     = 366188;
  c.Pa1offset     = 129996;
  c.Pa2factor     = 366188;
  c.Pa2offset     = 129996;
  c.Pa3factor     = 366188;
  c.Pa3offset     = 129996;
  c.flowfactor_nl = 259740;
  */
  c.Pa1factor     = getParam(0);
  c.Pa1offset     = getParam(1);
  c.Pa2factor     = getParam(2);
  c.Pa2offset     = getParam(3);
  c.Pa3factor     = getParam(4);
  c.Pa3offset     = getParam(5);
  c.flowfactor_nl = getParam(6);
}

void loop() {
  if(T36.available() > 0 && skipread == false){
    char tmp = 0;
    while(T36.available() > 0){
      tmp = T36.read();
      if(tmp >= 'a' && tmp <= 'c'){
        lastchar = tmp;
      }
    }
  }
  skipread = false;
  switch(lastchar){
  case 'a':
    Serial.print("case 'a':\n");
    //sent plot data
    Plotting();
    break;
  case'b':
    Serial.print("case 'b':\n");
    //sent plot data and record on SD card, sent 'e' for error if Recording failed
    if(Recording() == false){
      //sent error notification to T36
      T36.print("[e]");
      //default to mode a
      lastchar = 'a';
      mode = 'a';
    }
    break;
  case 'c':
    Serial.print("case 'c':\n");
    //sent calibration data and optionally recieve new parameter values
    Calibrating();
    break;
  default:
    Serial.print("...\n");
    Serial.flush();
    T36.print("[0]");
    T36.flush();
    break;
  }
  delay(10);
}

void Plotting(){
  //set mode to plotting
  mode = 'a';
  
  //start interval timer to take measurements every 50ms (20Hz)
  Timer.begin(Measure, 100000);
  
  //while mode is unchanged
  while(lastchar == 'a'){
    //check for new command from T36
    if(T36.available()>0){
      char tmp = 0;
      while(T36.available() > 0){
        tmp = T36.read();
        if(tmp >= 'a' && tmp <= 'c'){
          lastchar = tmp;
        }
      }
      //Serial.print("a: recieved char '");Serial.print(lastchar);Serial.print("'\n");
      if(lastchar != 'a'){skipread = true;} // skip reading next char if a new mode is detected
    }
    delay(1);
  }
  Timer.end();  
}

bool Recording(){
  //clear buffer to only safe most recent data to SD card
  MeasurementBuffer.clear();

  //reset measurement index
  MeasurementIndex = 0;
  
  //set mode for Measure ISR
  mode = 'b';
  
  //begin measuring at 100Hz (10000 us)
  Timer.begin(Measure, 10000);
  
  //try to connect to SD card, leave Recording() if failed
  //SDClass recSD;
  SdFat recSD;
  //if(recSD.begin() == false){
  if(recSD.begin(SS,SD_SCK_MHZ(45)) == false){
    Timer.end();
    //Serial.print("SD not found\n");
    recording = false;
    return false;
  }

  //set recording bit true
  recording = true;

  //determine file name
  char fileName[13] = FILE_BASE_NAME "00.csv";
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  while (recSD.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      Timer.end();
      recording = false;
      //Serial.print("can't find free filename\n");
      return false;
    }
  }
  //create file
  //File logFile = recSD.open(fileName, FILE_WRITE);
  SdFile logFile;
  if(logFile.open(fileName, O_WRONLY | O_CREAT | O_EXCL) == false){
    recording = false;
    //Serial.print("can't create new file\n");
    return false;
  }
  
  //write header
  logFile.print("us\tbar\tbar\tbar\tmm\tml\tml/min\r\n");
  //Serial.print("created file >"); Serial.print(fileName); Serial.print("<\n");

  //struct to hold the most recent measurement pulled from MeasurementBuffer
  logmeasurements temp = { 0 };

  //while mode remains unchanged
  while(lastchar == 'b'){
    //check for new data from T36
    if(T36.available() > 0){
      char tmp = 0;
      while(T36.available() > 0){
        tmp = T36.read();
        if(tmp >= 'a' && tmp <= 'c'){
          lastchar = tmp;
        }
      }
      //Serial.print("b: recieved char '");Serial.print(lastchar);Serial.print("'\n");
      if(lastchar != 'b'){skipread = true;} // skip reading next char if a new mode is detected
    }
    //if data is in MeasurementBuffer safe that to the SD card
    cli();
    bool result = MeasurementBuffer.getNext(&temp);
    sei();
    if(result == true){
      String nextline = MeasurementString(temp);
      //Serial.print("writing: ");
      //Serial.print(nextline);
      logFile.print(nextline);
    }
    delay(3);
  }
  //left while loop because new mode was commanded, keep writing to SD card until buffer is empty
  //while measurements remain in the buffer keep logging to SD card
  cli();
  bool result = MeasurementBuffer.getNext(&temp);
  sei();
  while(result == true){
    String nextline = MeasurementString(temp);
    //Serial.print("writing: ");
    //Serial.print(nextline);
    logFile.print(nextline);
    cli();
    result = MeasurementBuffer.getNext(&temp);
    sei();
    //add small delay so isr still works
    delay(3);
  }
  logFile.close();  
  //Serial.print("SD ready to remove\n");
  Timer.end();
  //set recording bit false
  recording = false;
  return true;
}

void Calibrating(){
  //set mode to calibrating
  mode = 'c';
  
  //start interval timer to take measurements every 50ms (20Hz)
  Timer.begin(Measure, 100000);
  
  //while mode is unchanged
  while(lastchar == 'c'){
    //check for new command from T36
    if(T36.available()>0){
      char tmp = 0;
      while(T36.available() > 0){
        tmp = T36.read();
        //if tmp is a mode command safe it in lastchark
        if(tmp >= 'a' && tmp <= 'c'){
          Serial.print("modechange from 'c' to '");
          Serial.write(tmp);
          Serial.print("'\n");
          lastchar = tmp;
        }
        //else if tmp is the start of a parameter then listen further
        else if (tmp == '['){
          Timer.end();
          char msg[12] = { 0 };
          unsigned int i = 0;
          while(i<=12){
            if(T36.available() > 0){
              tmp = T36.read();
              if(tmp == ']'){
                Serial.print("found ']' at i=");
                Serial.print(i);
                Serial.print("\n");
                //decrease i so it doesn't trigger the if(i == 12) error
                --i;
                break;
              }
              else{
                msg[i] = tmp;
                ++i;
              }
            }
          }
          Serial.print("recieved >>>");
          for(int j=0; j<12; ++j){Serial.write(msg[j]);}
          Serial.print("<<<\n");
          //if i = 12 then the message didn't terminate with ']' in time, send error
          if(i == 12){
            Serial.print("msg didn't terminate with ']', sending 'e' to T36\n");
            T36.print("e");
          }
          //else analyse the message
          else{
            uint8_t paramNr = 0;
            unsigned long paramValue = 0;
            if(analyseMsg(&(msg[0]), &paramNr, &paramValue) == true){
              //send 's'uccess to T36
              Serial.print("sending 's' to T36\n");
              T36.print("s");
              //safe parameter in EEPROM
              setParam(paramNr, paramValue);
              //update parameter in calibration struct
              c.Pa1factor     = getParam(0);
              c.Pa1offset     = getParam(1);
              c.Pa2factor     = getParam(2);
              c.Pa2offset     = getParam(3);
              c.Pa3factor     = getParam(4);
              c.Pa3offset     = getParam(5);
              c.flowfactor_nl = getParam(6);
            }
            else{
              Serial.print("sending 'e' to T36\n");
              T36.print("e");
            }
          }
          //start timer again
          Timer.begin(Measure, 100000);
        }
      }
      //Serial.print("c: recieved char '");Serial.print(lastchar);Serial.print("'\n");
      if(lastchar != 'c'){skipread = true;} // skip reading next char if a new mode is detected
    }
  }
  Timer.end();
}

//#########################################################################################################################
//                                                      MEASURE
//#########################################################################################################################

//interval timer function
void Measure(){
  //measure pressure sensors
  unsigned long t = millis();
  rawValues.Pa1 = analogRead(Sensor1);
  rawValues.Pa2 = analogRead(Sensor2);
  rawValues.Pa3 = analogRead(Sensor3);
  
  //get flow sensor values
  cli();
  rawValues.flowcount = flowcount;
  sei();
  
  pltmeasurements pltm = { 0 };
  logmeasurements logm = { 0 };
  
  switch(mode){
  case 'a':
    //plotting, get all measurements
    cli();
    pltm.Pa1   = (rawValues.Pa1 * c.Pa1factor / 1000.0) - c.Pa1offset; // [bit * mPa/bit / 1000] - [Pa]
    pltm.Pa2   = (rawValues.Pa2 * c.Pa2factor / 1000.0) - c.Pa2offset;
    pltm.Pa3   = (rawValues.Pa3 * c.Pa3factor / 1000.0) - c.Pa3offset;
    pltm.um    = micrometer;
    //m.ul unused in plotting
    pltm.ulmin = flowrate_ulmin;
    sei();
    SendPlotData(pltm,'a');
    break;
  
  case 'b':
    //recording, get all measurements
    logm.timest = t;
    cli();
    logm.Pa1    = (rawValues.Pa1 * c.Pa1factor / 1000.0) - c.Pa1offset;
    logm.Pa2    = (rawValues.Pa2 * c.Pa2factor / 1000.0) - c.Pa2offset;
    logm.Pa3    = (rawValues.Pa3 * c.Pa3factor / 1000.0) - c.Pa3offset;
    logm.um     = micrometer;
    logm.ul     = flowcount * (c.flowfactor_nl / 1000.0 );
    logm.ulmin  = flowrate_ulmin;
    MeasurementBuffer.setNext(logm);
    ++MeasurementIndex;
    sei();
    //plot every 10th measurement
    if(MeasurementIndex % 10 == 0){
      pltm.Pa1    = logm.Pa1;
      pltm.Pa2    = logm.Pa2;
      pltm.Pa3    = logm.Pa3;
      pltm.um     = logm.um;
      pltm.ulmin  = logm.ulmin;
      SendPlotData(pltm,'b');
    }
    break;
  
  case 'c':
    SendSensorData(rawValues);
    break;
  default:
    break;  
  }
}

//#########################################################################################################################
//                                              COMMUNICATION FUNCTIONS
//#########################################################################################################################

//send plot data via uart to T36
void SendPlotData(pltmeasurements m, char mode){
  T36.print("[");
  T36.write(mode);
  T36.print(m.Pa1, DEC);
  T36.print(",");
  T36.print(m.Pa2, DEC);
  T36.print(",");
  T36.print(m.Pa3, DEC);
  T36.print(",");
  T36.print(m.um, DEC);
  T36.print(",");
  T36.print(m.ulmin, DEC);
  T36.print(",");
  T36.print(recording);
  T36.print("]");

  
  Serial.print("[");
  Serial.write(mode);
  Serial.print(m.Pa1, DEC);
  Serial.print(",");
  Serial.print(m.Pa2, DEC);
  Serial.print(",");
  Serial.print(m.Pa3, DEC);
  Serial.print(",");
  Serial.print(m.um, DEC);
  Serial.print(",");
  Serial.print(m.ulmin, DEC);
  Serial.print(",");
  Serial.print(recording);
  Serial.print("]");
  Serial.print("\n");
  
}

//send sensor data via uart to T36
void SendSensorData(sensorValues s){
  T36.print("[c");
  T36.print(s.Pa1, DEC);
  T36.print(",");
  T36.print(s.Pa2, DEC);
  T36.print(",");
  T36.print(s.Pa3, DEC);
  T36.print(",");
  T36.print(s.flowcount, DEC);
  T36.print("]");
  Serial.print("[c");
  Serial.print(s.Pa1, DEC);
  Serial.print(",");
  Serial.print(s.Pa2, DEC);
  Serial.print(",");
  Serial.print(s.Pa3, DEC);
  Serial.print(",");
  Serial.print(s.flowcount, DEC);
  Serial.print("]\n");
}

//return string for writing to SD card
String MeasurementString(logmeasurements data){
  String buf;
  buf += String(data.timest);
  buf += "\t";
  buf += String(data.Pa1/100000.0,2);
  buf += "\t";
  buf += String(data.Pa2/100000.0,2);
  buf += "\t";
  buf += String(data.Pa3/100000.0,2);
  buf += "\t";
  buf += String(data.um/1000.0,2);
  buf += "\t";
  buf += String(data.ul/1000.0,2);
  buf += "\t";
  buf += String(data.ulmin/1000.0,2);
  buf += "\r\n";
  return buf;
}

//set nr and value according to the content of msg[20]
//msg doesn't include the brackets [] from recieving
//example msg = >>>1,12345<<<
bool analyseMsg(char * msg, uint8_t * nr, unsigned long * val){
  Serial.print("analyseMsg with msg = >>>");
  for(int i = 0; i<12; ++i){
    Serial.write(*(msg+i));
  }
  Serial.print("<<<\n");
  if(*msg < '0' || *msg > '6'){
    Serial.print("msg[0] is no parameter nr in range of 0 to 6\n");
    return false;
    }
  else{
    *nr = *msg - 0x30;
  }
  if(*(msg+1) != ','){
    Serial.print("msg[1] is not ','\n");
    return false;
  }
  unsigned int valend = 0;
  for(int i = 2; i<12; ++i){
    //make sure that msg only contains correct chars
    if((*(msg+i) < '0' || *(msg+i) >'9') && *(msg+i) != 0x00){
      Serial.print("msg[");
      Serial.print(i);
      Serial.print("] is not a digit and not 0x00\n");
      return false;
    }
    //update valend to i when msg[i] is a digit
    else if(*(msg+i) >= '0' && *(msg+i) <= '9'){
      valend = i;
    }
  }
  Serial.print("valend = ");Serial.print(valend);Serial.print("\n");
  if(valend == 0){
    Serial.print("msg didn't contain a parameter value\n");
    return false;
  }
  unsigned long factor = 1;
  *val = 0;
  for(int i = valend; i>=2; --i){
    *val += (*(msg+i) - 0x30) * factor;
    factor *= 10;
  }
  Serial.print("converted msg to nr = ");
  Serial.print(*nr);
  Serial.print(" and val = ");
  Serial.print(*val);
  Serial.print("\n");
  return true;
}

//#########################################################################################################################
//                                                  EEPROM FUNCTIONS
//#########################################################################################################################

void setParam(uint8_t nr, unsigned long value){
  if(nr <= 12){
    byte mbyte = value & 0xFF;
    EEPROM.write((nr*4), mbyte);
    mbyte = (value & 0x0000FF00)>> 8;
    EEPROM.write((nr*4) + 1, mbyte);
    mbyte = (value & 0x00FF0000)>> 16;
    EEPROM.write((nr*4) + 2, mbyte);
    mbyte = (value & 0xFF000000)>> 24;
    EEPROM.write((nr*4) + 3, mbyte);
  }
}

unsigned long getParam(uint8_t nr){
  //highest byte at highest EEPROM address
  if(nr <= 6){
    unsigned long result = 0x00000000;
    byte mbyte = EEPROM.read(nr*4);
    result = mbyte;
    mbyte = EEPROM.read((nr*4) + 1);
    result |= mbyte<<8;
    mbyte = EEPROM.read((nr*4) + 2);
    result |= mbyte<<16;
    mbyte = EEPROM.read((nr*4) + 3);
    result |= mbyte<<24;
    return result;
  }
  return 0;
}

//#########################################################################################################################
//                                                     ISR FUNCTIONS
//#########################################################################################################################

void FlowISR(){
  cli();
  FlowrateStopper.begin(resetFlow,1000000); 
  //if flowchangetime or longer has passed since last ISR
  uint64_t thischange = micros64bit();
  if(lastchange + minFlowTime < thischange){
    //flowcount used by calibration, flow in ml used by SD card, flowrate used by SD card and plotting
    ++flowcount;

//  ul      =              nl            / 1 000 [nl/ul]
    flow_ul = (flowcount * c.flowfactor_nl) / 1000.0;

//  ul/min         =      ( nl / (nl/ul) )     / (         ( us )         /  (us/min) )
//  flowrate       =   (c.flowfactor_nl/1000)  / ((micros() - lastchange) / 60,000,000);
    flowrate_ulmin = ((c.flowfactor_nl * 6000) / (thischange-lastchange)) * 10;
    lastchange = thischange;
  }
  sei();
}

//called to set flowrate = 0 when no pin toggle is detected within some time
void resetFlow(){
  cli();
  flowrate_ulmin = 0;
  FlowrateStopper.end();
  sei();
}

void serialEvent1(){
  cli();
  char recieved = Dial.read();
  if(recieved == 0x12){msgIndex = 0;}
  else if(msgIndex ==  1){readout[0] = recieved;}
  else if(msgIndex > 4 && msgIndex < 9){readout[msgIndex-4] = recieved;}
  if(msgIndex == 8){
    //decode message
    micrometer = 0;
    unsigned int factor = 10;
    int tmp = 0;
    for(int i = 4; i>0; --i){
      tmp = readout[i] - 48;
      //if tmp is not a digit, leave
      if(tmp < 0 || tmp > 9){
        micrometer = 0;
        ++msgIndex;
        sei();
        return;
      }
      else{
        micrometer += tmp * factor;
        factor *= 10;
      }
    }
    //determine sign
    if(readout[0] == '-'){micrometer = - micrometer;}
  }
  ++msgIndex;
  sei();
}


uint64_t micros64bit(){
  uint32_t t = micros();
  if(t < lastmicros){
    ++upperLong;
  }
  lastmicros = t;
  return (upperLong << 32) + t;
}
