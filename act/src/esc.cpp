/******************************************************************************
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Blob Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal 
 * in the Software without restriction, including without limitation the rights 
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
 * SOFTWARE.
 * 
 * \file       esc.cpp
 * \brief      driver for ESC to control brushless motor
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/esc.h"

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
bool blob::ESC::isReady() {
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

