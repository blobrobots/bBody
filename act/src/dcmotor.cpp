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

#include "blob/dcmotor.h"
#include "blob/math.h"

// http://blog.whatgeek.com.pt/arduino/l298-dual-h-bridge-motor-driver/

blob::DCMotor::DCMotor () {

  _ready   = false;
  _on      = false;
  _output  = 0.f;
  _stalled = false;

}
    
bool blob::DCMotor::init () {

  if(!_ready && _pwmFwd.init() && _pwmBwd.init()) {    
    _ready = true;
    _pwmFwd.setDutyCycle(0);
    _pwmBwd.setDutyCycle(0);
  }
  return _ready;
}

bool blob::DCMotor::init (uint8_t enablePin) {

  _enablePin=enablePin;

#if defined(__AVR_ATmega32U4__)
  pinMode(_enablePin, OUTPUT);
  digitalWrite(_enablePin,LOW);
#endif
  
  return init();

}

bool blob::DCMotor::setOutput (float output) {
  
  if(!_ready || !_on || _stalled || output < -1.f || output > 1.f)
    return false;
  
  _output=output;
  uint16_t usFwd = 0; // us forward
  uint16_t usBwd = 0; // us backwards
  uint16_t us = 0.f;
  
  us = (uint16_t)(math::rabs(_output)*_pwmFwd.getPeriod());

  if(_output > 0.f) {
    usFwd = us;
  } else if (_output < 0.f) {
    usBwd = us;
  }
  
  _pwmFwd.setDutyCycle(usFwd);
  _pwmBwd.setDutyCycle(usBwd);

  return true;
}

bool blob::DCMotor::switchOn () {

  if(_ready) {
#if defined(__AVR_ATmega32U4__)
    digitalWrite(_enablePin, HIGH);
#endif
    _pwmFwd.setDutyCycle(0);
    _pwmBwd.setDutyCycle(0);

    _on = true;
    _stalled = false;

    return true;
  }
  return false;
}

bool blob::DCMotor::switchOff () {

  if(_ready) {
#if defined(__AVR_ATmega32U4__)
    digitalWrite(_enablePin,LOW);
#endif
    _output=0;
    _pwmFwd.setDutyCycle(0);
    _pwmBwd.setDutyCycle(0);

    _on = false;    
    _stalled = false;

    return true;
  }
  return false;
}

bool blob::DCMotor::stall () {
  if(_ready && _on)
  {
    _output=0;
    _pwmFwd.setDutyCycle(_pwmFwd.getPeriod());
    _pwmBwd.setDutyCycle(_pwmBwd.getPeriod());

    _stalled = true;

    return true;
  }
  return false;
}

bool blob::DCMotor::release () {
  if(_ready && _on)
  {
    _pwmFwd.setDutyCycle(0);
    _pwmBwd.setDutyCycle(0);

    _stalled = false;

    return true;
  }
  return false;
}

bool blob::DCMotor::isStalled() {
  return _stalled;
}

