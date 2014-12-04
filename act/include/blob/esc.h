/********* blob robotics 2014 *********
 *  title: esc.h
 *  brief: interface for ESC motor control
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_ESC_H
#define B_ESC_H

#if defined(__AVR_ATmega32U4__)
  #include "Arduino.h"
#endif

#include "blob/pwm.h"

namespace blob {

class ESC // : Motor
{
  public:
    ESC(uint32_t minUs = 1000, uint32_t maxUs = 2000, uint32_t periodUs = 10000);
// Motor
    boolean isReady();
    
    void init ();
    void setOutput (float output); // normalized output [0.0 - 1.0]
    float getOutput (); // normalized output [0.0 - 1.0]
    boolean isOn ();
    void switchOn ();
    void switchOff ();
// ESC    
    void setRange (); // us
    void setConfig (); // here standard esc config
    uint32_t getMaxUs (); // us
    uint32_t getMinUs (); // us
    uint32_t getPeriod (); // us
    uint8_t getIndex (); 
    uint8_t getPin ();

  private:
// Motor
    boolean _ready;
    boolean _on;
// ESC    
    blob::PWM _pwm;
    uint32_t _maxUs;
    uint32_t _minUs;  
    // standard config parameters missing

};

}

#endif /* B_ESC_H */

