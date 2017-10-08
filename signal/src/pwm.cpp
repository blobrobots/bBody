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
 * \file       pwm.cpp
 * \brief      driver for PWM signal generator and reader
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/pwm.h"

#if defined(__AVR_ATmega32U4__)
  #define ARDUINO_ARCH_AVR
  #include <PinChangeInterrupt.h>
#endif

uint8_t blob::PWMin::_inUse = 0;
uint8_t blob::PWMout::_inUse = 0;

blob::PWMin *blob::PWMin::_ref[B_PWM_MAX_NUM] = {NULL,NULL,NULL,NULL};

#if defined(__AVR_ATmega32U4__)
// D9, D10, D5, D6
const uint8_t  blob::PWMout::_pinList [B_PWM_MAX_NUM] = {9,10,5,6};
// roll, pitch, yaw, aux1
const uint8_t  blob::PWMin::_pinList [B_PWM_MAX_NUM] = {16,14,15,8};
#else
const uint8_t  blob::PWMout::_pinList [B_PWM_MAX_NUM] = {1,2,3,4};
const uint8_t  blob::PWMin::_pinList [B_PWM_MAX_NUM] = {1,2,3,4};
#endif

blob::PWM::PWM () {
  
  _index     = 0xFF;
  _pin       = 0xFF;
  _period    = 0;
  _dutyCycle = 0;
} // PWM::PWM

bool    blob::PWM::isReady() {return (_index < B_PWM_MAX_NUM);}

uint16_t blob::PWM::getDutyCycle () {return _dutyCycle;}

uint16_t blob::PWM::getPeriod () {return _period;}

uint16_t blob::PWM::getFrequency () {return (uint16_t)(1.0f/_period);}

uint8_t blob::PWM::getPin () {return _pin;}

uint8_t blob::PWM::getIndex () {return _index;}

bool blob::PWMout::init() {

  if(_inUse >= B_PWM_MAX_NUM) 
    return false;

  if(isReady())
    return true;

  _index = _inUse++;
  _pin = _pinList[_index];
  
#if defined(__AVR_ATmega32U4__)

// FIXME: config parameters
  _period = 2041;  

  pinMode(_pin, OUTPUT); // set the digital pin as output:
  
  switch(_index) {
    case 0:
      TCCR1A |=  (1<<WGM11); // phase correct mode & no prescaler
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) & ~(1<<CS11) & ~(1<<CS12);
      TCCR1B |=  (1<<WGM13) |  (1<<CS10); 
      ICR1   |=  0x3FFF;     // top to 16383;     
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
      break;
      
    case 1:
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
      break;
      
    case 2:
      // phase and freq. correct mode & top to 1023 but with enhanced mode 2047
      TCCR4E |=  (1<<ENHC4); // enhanced pwm mode
      TCCR4B &= ~(1<<CS41);     
      TCCR4B |=  (1<<CS42) | (1<<CS40); // prescaler to 16
      TCCR4D |=  (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; 
      TCCR4A |=  (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A 
      break;
      
    case 3:
      TCCR4C |=  (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
      break;
  }
#endif // defined(__AVR_ATmega32U4__)
  
  setDutyCycle(0);
  
  return true;
} // PWMout::init

bool blob::PWMout::setDutyCycle(uint16_t us) {
  
  if(us > _period || _index >= B_PWM_MAX_NUM)
    return false;

#if defined(__AVR_ATmega32U4__)    
  switch(_index) {
    // timer 1A & 1B [1000:2000] => [8000:16000]
    case 0:
      OCR1A = us<<3; //  pin 9
      break;
    case 1:
      OCR1B = us<<3; //  pin 10
      break;
      
    // to write values > 255 to timer 4 A/B we need to split the bytes
    // Timer 4 A & D [1000:2000] => [1000:2000]    
    case 2:
      TC4H  =  (2047-us)>>8; 
      OCR4A = ((2047-us)&0xFF); //  pin 5
      break;
      
    case 3:
      TC4H  =  us>>8; 
      OCR4D = (us&0xFF); //  pin 6
      break;
  }
#endif // defined(__AVR_ATmega32U4__)
  _dutyCycle = us;
  
  return true;
} // PWMout::setDutyCycle

uint8_t blob::PWMout::getNumInUse () {return _inUse;}

// https://quadmeup.com/read-rc-pwm-signal-with-arduino/

uint8_t blob::PWMin::getNumInUse () {return _inUse;}

void  blob::PWMin::sync() {
  for(int i=0; i<_inUse; i++)
    _ref[i]->syncPeriod();
}

bool blob::PWMin::init() {

  if(_inUse >= B_PWM_MAX_NUM) 
    return false;

  _index = _inUse++;
  _pin = _pinList[_index];
  _ref[_index] = this;

#if defined(__AVR_ATmega32U4__)

// FIXME: config parameters
  _period = 0;  
  _pwmStart = 0;
  _pinHigh = false;
  
  pinMode(_pin, INPUT); // set the digital pin as input
  // FIXME: use array of callbacks
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(_pinList[0]),onPinChange0,CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(_pinList[1]),onPinChange1,CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(_pinList[2]),onPinChange2,CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(_pinList[3]),onPinChange3,CHANGE);

  return true;

#endif // defined(__AVR_ATmega32U4__)

  return false;
} // PWMin::init

#if defined(__AVR_ATmega32U4__)

void blob::PWMin::onPinChange0(void) {
  _ref[0]->onPinChange();
} // PWMin::onPinChange0

void blob::PWMin::onPinChange1(void) {
  _ref[1]->onPinChange();
} // PWMin::onPinChange1

void blob::PWMin::onPinChange2(void) {
  _ref[2]->onPinChange();
} // PWMin::onPinChange2

void blob::PWMin::onPinChange3(void) {
  _ref[3]->onPinChange();
} // PWMin::onPinChange3

void blob::PWMin::onPinChange() {

  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(_pin));

  if(trigger == RISING) {

    _period = micros()-_pwmStart;
    _pwmStart = micros();
    _pinHigh = true;
    
  } else if(trigger == FALLING) {
    _dutyCycle = micros() - _pwmStart;
    _pinHigh = false;
  }
}

bool  blob::PWMin::syncPeriod () {
  if(micros()-_pwmStart > 65535) {
    if(_pinHigh) {
      _period=0xFFFF;
      _dutyCycle=0xFFFF;
    } else {
      _period=0;
      _dutyCycle=0;
    }
    return false;
  }
  return true;
}
#endif // defined(__AVR_ATmega32U4__)

/*
http://www.righto.com/2009/07/secrets-of-arduino-pwm.html

Setting    Divisor    Frequency
0x01        1        31250
0x02        8        3906.25
0x03     64        488.28125
0x04     256        122.0703125
0x05        1024        30.517578125

TCCR1B = TCCR1B & 0b11111000 | <setting>;
*/

