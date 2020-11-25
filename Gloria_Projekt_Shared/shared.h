#include <Arduino.h>
#include "HardwareSerial.h"

//#ifndef vector
//#include <vector>
//#endif

struct pltmeasurements{
    unsigned long Pa1       = 0; //Pascal, Sensor 1         max  1 200 000 Pa       12  bar
    unsigned long Pa2       = 0; //Pascal, Sensor 2         max  1 200 000 Pa       12  bar
    unsigned long Pa3       = 0; //Pascal, Sensor 3         max  1 200 000 Pa       12  bar
    signed int    um        = 0; //micro Meter              max     25 470 um       25  mm
    //unsigned long ul        = 0; //micro Liter              max 99 000 000 ul       99  Liter
    unsigned long ulmin     = 0; //micro Liter per Minute   max    500 000 ulmin    0.5 Liter/min
};

struct logmeasurements{
    unsigned long timest    = 0;
    unsigned long Pa1       = 0; //Pascal, Sensor 1         max  1 200 000 Pa       12  bar
    unsigned long Pa2       = 0; //Pascal, Sensor 2         max  1 200 000 Pa       12  bar
    unsigned long Pa3       = 0; //Pascal, Sensor 3         max  1 200 000 Pa       12  bar
    signed int    um        = 0; //micro Meter              max     25 470 um       25  mm
    unsigned long ul        = 0; //micro Liter              max 99 000 000 ul       99  Liter
    unsigned long ulmin     = 0; //micro Liter per Minute   max    500 000 ulmin    0.5 Liter/min
};

struct calibration{
    unsigned long Pa1factor     = 0; //unit: mPa/bit micro Pascal per LSB of 12 bit Measurement
    unsigned long Pa1offset     = 0; //unit: Pa Pascal
    unsigned long Pa2factor     = 0; //unit: mPa/bit micro Pascal per LSB of 12 bit Measurement
    unsigned long Pa2offset     = 0; //unit: Pa Pascal
    unsigned long Pa3factor     = 0; //unit: mPa/bit micro Pascal per LSB of 12 bit Measurement
    unsigned long Pa3offset     = 0; //unit: Pa Pascal
    unsigned long flowfactor_nl = 0; //unit: nl/toggle
};

struct sensorValues{
    unsigned int Pa1        = 0; //12 bit analog measurement
    unsigned int Pa2        = 0; //12 bit analog measurement
    unsigned int Pa3        = 0; //12 bit analog measurement
    unsigned int flowcount  = 0; //counter of flowsensor toggles
};

struct Plot{
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  uint8_t d = 0;
  uint8_t e = 0;
};

template <class T, int S>
class Buffer{
  private:
    unsigned int    first   = 0;
    unsigned int    last    = 0;
    unsigned int    n       = 0;
    unsigned int    max     = S;
    T               ring[S];
  
  public:
    bool getNext(T * element){
      //return false and write empty T in 'element' if buffer is empty (n ==0);
      if(n == 0){
        //Serial.print("\t\t\t\t\t\tgetNext n==0, return nothing\n");
        T nothing;
        *element = nothing;
        return false;
      }
      //if n > 0 then get the element at 'first' position, decrease count n by one
      else{
        //Serial.print("\t\t\t\t\t\tgetNext n = ");Serial.print(n);Serial.print(" > 0, return first=");Serial.print(first);Serial.print("\n");
        *element = ring[first];
        --n;
        //only move first forward one if the buffer still contains elements
        if(n != 0){
          if(first == max-1){
            first = 0;
          }
          else{
            ++first;
          }
        }
        //Serial.print("\t\t\t\t\t\tnew first=");Serial.print(first);Serial.print(" new n=");Serial.print(n);Serial.print("\n");
        return true;
      }
    };
    bool getAll(T * firstelement){
      if(firstelement == nullptr){return false;}
      unsigned int counter = 0;
      for(unsigned long i = first; i < max; ++i){
        *(firstelement + counter) = ring[i];
        ++counter;
      }
      for(unsigned long i = 0; i < first; ++i){
        *(firstelement + counter) = ring[i];
        ++counter;
      }
      return true;
    }
    void fillAll(T element){
        for(unsigned int i = 0; i < max; ++i){
            ring[i] = element;
        }
    }
    bool setNext(T next){
      //if ring is full (n=max) then move both 'last' and 'first' forward and overwrite the oldest element that was at the old 'first' position, return false because overflow occured
      if(n == max){
        //Serial.print("\t\t\t\t\t\tbuffer full, old last=");Serial.print(last);Serial.print(" old first=");Serial.print(first);Serial.print("\n");
        if(last == max-1){
          last = 0;
          first = 1;
        }
        else if(first == max-1){
          ++last;
          first = 0;
        }
        else{
          ++last;
          ++first;
        }
        //Serial.print("\t\t\t\t\t\tnew last=");Serial.print(last);Serial.print(" new first=");Serial.print(first);Serial.print("\n");
        ring[last] = next;
       return false;
      }
      //otherwise if the buffer is empty, don't move 'last' forward, but store new element at 'last' position, increase n by one, return true because no overflow occured
      else if(n == 0){
        //Serial.print("\t\t\t\t\t\tbuffer empty, last=");Serial.print(last);Serial.print("    new n=1\n");
        ring[last] = next;
        n = 1;
      }
      else{
        //Serial.print("\t\t\t\t\t\tbuffer not full, last =");Serial.print(last);Serial.print("\n");
        if(last == max-1){
          last = 0;
        }
        else{
          ++last;
        }
        ring[last] = next;
        ++n;
        //Serial.print("\t\t\t\t\t\tnew last=");Serial.print(last);Serial.print("    new n=");Serial.print(n);Serial.print("\n");
        return true;
      }
      return true;
    };
    void clear(){
      T nothing = { 0 };
      for(unsigned int i = 0; i<max; ++i){
        ring[i] = nothing;
      }
      n = 0;
      first = 0;
      last = 0;
    };
};
