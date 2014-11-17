/********* blob robotics 2014 *********
 *  title: bPwm.cpp
 *  brief: driver for PWM signal generator
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "bPwm.h"

uint8_t blob::PWM::_inUse = 0;
const uint8_t blob::PWM::_pinList [B_PWM_MAX_NUM] = {9,10,5,6};

blob::PWM::PWM () {
  
  if(_inUse < B_PWM_MAX_NUM) {
    _index = _inUse;
    _inUse++;
    _pin = _pinList[_index];
    
    _period = 10000; // = 10 ms
    _dutyCycle = 0;
    
  } else {
    _index = 0xFF;
    _pin = 0xFF;
    _period = 0;
    _dutyCycle = 0;
  }
} // PWM::PWM

void blob::PWM::init() {
#if defined(__AVR_ATmega32U4__)
  switch(_index) {
    case 0:
      TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
      TCCR1B |= (1<<WGM13) | (1<<CS10); 
      ICR1   |= 0x3FFF; // TOP to 16383;     
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
      break;
      
    case 1:
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
      break;
      
    case 2:
      TCCR4E |= (1<<ENHC4); // enhanced pwm mode
      TCCR4B &= ~(1<<CS41);     
      TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
      TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
      TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A 
      break;
      
    case 3:
      TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
      break;
  }
#endif // defined(__AVR_ATmega32U4__)
} // PWM::init

void blob::PWM::setDutyCycle(uint32_t us) {
  
  if(_index < 1000 || _index > 2000)
    return;
#if defined(__AVR_ATmega32U4__)    
  switch(_index) {
    // Timer 1A & 1B [1000:2000] => [8000:16000]
    case 0:
      OCR1A = us<<3; //  pin 9
      break;
    case 1:
      OCR1B = us<<3; //  pin 10
      break;
      
    // to write values > 255 to timer 4 A/B we need to split the bytes      
    case 2:
      TC4H = (2047-us)>>8; 
      OCR4A = ((2047-us)&0xFF); //  pin 5
      break;
      
    case 3:
      TC4H = us>>8; 
      OCR4D = (us&0xFF); //  pin 6
      break;
  }
#endif // defined(__AVR_ATmega32U4__)
  _dutyCycle = us;
  
} // PWM::setDutyCycle

uint32_t blob::PWM::getDutyCycle () {return _dutyCycle;}

uint32_t blob::PWM::getPeriod () {return _period;}

uint8_t blob::PWM::getPin () {return _pin;}

uint8_t blob::PWM::getIndex () {return _index;}

uint8_t blob::PWM::getNumInUse () {return _inUse;}
