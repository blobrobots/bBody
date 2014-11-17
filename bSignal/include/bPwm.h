/********* blob robotics 2014 *********
 *  title: bPwm.h
 *  brief: interface for PWM signal generator
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_PWM_H
#define B_PWM_H

#include "bTypes.h"

#define B_PWM_MAX_NUM 4

namespace blob {
  
class PWM
{
  public:
    PWM();

    bool isReady();
    
    void init ();
    //void setPeriod (uint32_t us); // (us)
    void setDutyCycle (uint32_t us); // (us)
    uint32_t getPeriod (); // (us)
    uint32_t getDutyCycle (); // (us)  
    uint8_t getPin ();
    uint8_t getIndex ();
    
    static uint8_t getNumInUse();

  private:
    static const uint8_t _pinList[B_PWM_MAX_NUM];
    static uint8_t _inUse;
    
    uint8_t _index;
    uint8_t _pin;
    
    uint32_t _period;
    uint32_t _dutyCycle;
};

}

#endif /* B_PWM_H */

