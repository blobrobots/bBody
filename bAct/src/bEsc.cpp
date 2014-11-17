/********* blob robotics 2014 *********
 *  title: bEsc.cpp
 *  brief: driver for ESC motor control
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "bEsc.h"

blob::ESC::ESC(uint32_t minUs, uint32_t maxUs, uint32_t periodUs) {
  _ready = false;
  _on = false;
  if(minUs < maxUs)
  {
    _minUs = minUs;
    _maxUs = maxUs;
  }
  
}

// Motor
boolean blob::ESC::isReady() {
  return _ready;
}
    
void blob::ESC::init () {
  if(_pwm.getIndex() != 0xFF)
  {
    _pwm.init();
    _ready = true;
  }
}

void blob::ESC::setOutput (float output) {
  
  if(_ready && _on) {
    uint32_t outUs = (uint32_t)(_minUs + output*(_maxUs-_minUs));
    _pwm.setDutyCycle(outUs);
  }
}

float blob::ESC::getOutput () {
  return (float)((_pwm.getDutyCycle()-_minUs)/(_maxUs-_minUs));
}

boolean blob::ESC::isOn () {
  return _on;
}

void blob::ESC::switchOn () {
  if(_ready && !_on)
  {
    _on = true;
  }
}

void blob::ESC::switchOff () {
  if(_ready && _on)
  {
    _on = false;
  }

}

// ESC    
void blob::ESC::setRange () {
// Here procedure sequence
}
void blob::ESC::setConfig () {
// Here procedure sequence
}

uint32_t blob::ESC::getMaxUs () {return _maxUs;}

uint32_t blob::ESC::getMinUs () {return _minUs;}

uint32_t blob::ESC::getPeriod () {return _pwm.getPeriod();}

uint8_t blob::ESC::getIndex () {return _pwm.getIndex();}

uint8_t blob::ESC::getPin () {return _pwm.getPin();}

