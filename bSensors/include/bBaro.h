/********* blob robotics 2014 *********
 *  title: bMs5611.h
 *  brief: interface for Ms5611 baro
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_BARO_H
#define B_BARO_H

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

#include "bTask.h"

namespace blob {

class Baro : public Task
{
  public:
    virtual float getTemp      () {return _temp;}
    virtual float getPress     () {return _press;}
    virtual float getAltitude  () {return _altitude;}
    virtual float getElevation () {return _elevation;}
    virtual float getHeight    () {return _ready*(_altitude - _elevation);}
    
    void print (bool ln = true) {
#if defined(__AVR_ATmega32U4__)
      if (Serial) {
        Serial.print("blob::Baro - ");
        Serial.print(_temp); Serial.print(" ");
        Serial.print(_press); Serial.print(" ");
        Serial.print(_altitude); Serial.print(" ");
        Serial.print(_elevation); Serial.print(" ");
        Serial.print(getHeight()); Serial.print(" ");
        Serial.print(_dt);
        if (ln)
          Serial.println(" - ");
        else
          Serial.print(" - ");
      }
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
      std::cout << "blob::Baro - " 
                << _temp << " " 
                << _press << " " 
                << _altitude << " " 
                << _elevation << " " 
                << getHeight() << " "
                << _dt << " - ";
      if (ln) std::cout << std::endl;       
#endif // defined(__linux__)
    }

  protected:
    bool _setElevation;
  
    float _temp;
    float _press;
    float _altitude;
    float _elevation;

  public:
    Baro () : _setElevation(true) {}; // just to initialize _setElevation

};

}

#endif /* B_MS5611_H */
