/********* blob robotics 2014 *********
 *  title: i2c.cpp
 *  brief: driver for generic I2C device
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/

#if defined(__AVR_ATmega32U4__)
  #include "Wire.h" 
#endif

#include "blob/i2c.h"

bool blob::I2C::_started = false;
long blob::I2C::_freq = 0;
    
blob::I2C::I2C (uint8_t address, long freq)
{
   _address = address;
   if(!_started)
     _freq = freq;
} // I2C::I2C

void blob::I2C::init()
{
  if(!_started) {
    _started = true;
#if defined(__AVR_ATmega32U4__)
    Wire.begin(); // join i2c bus (address optional for master)
    TWBR = ((F_CPU/_freq) - 16)/2; // change the I2C clock rate to 400kHz
#if defined(__DEBUG__)
    if(Serial) {
      Serial.print (F("Initializing I2C bus at "));
      Serial.print (_freq);
      Serial.println (F(" Hz"));
    }
#endif
#endif // defined(__AVR_ATmega32U4__)
  }
} // I2C::init

void blob::I2C::writeReg (const uint8_t reg, const  uint8_t data)
{
  if(_started) {
#if defined(__AVR_ATmega32U4__)
     Wire.beginTransmission(_address); // transmit to device addr
     Wire.write(reg);        // sends register
     Wire.write(data);       // sends data
     Wire.endTransmission(); // stop transmitting
#endif // defined(__AVR_ATmega32U4__)
  }

} // I2C::writeReg

void blob::I2C::readReg (const uint8_t reg, const  uint8_t size, uint8_t *data)
{
  if(_started) {
#if defined(__AVR_ATmega32U4__)
     uint8_t i = 0;
     Wire.beginTransmission(_address); // transmit to device addr
     Wire.write(reg);        // sends register
     Wire.endTransmission(); // stop transmitting  
     Wire.requestFrom(_address, size);    // request size bytes from slave device addr
     while(Wire.available() && i < size) { // slave may send less than requested
        data[i++] = Wire.read();           // receive a byte as character
     }
#endif // defined(__AVR_ATmega32U4__)
  }   
} // I2C::readReg

void blob::I2C::request(const uint8_t reg)
{
  if(_started) {
#if defined(__AVR_ATmega32U4__)
    Wire.beginTransmission(_address); // transmit to device addr
    Wire.write(reg);        // sends register
    Wire.endTransmission(); // stop transmitting
#endif // defined(__AVR_ATmega32U4__)
  }  
} // I2C::request

/* FIXME borrar */
void blob::I2C::receive(const uint8_t size, uint8_t *data)
{
  if(_started) {
#if defined(__AVR_ATmega32U4__)
    uint8_t i = 0;
    Wire.requestFrom(_address, size);     // request size bytes from slave device addr
    while(Wire.available() && i < size) { // slave may send less than requested
      data[i++] = Wire.read();            // receive a byte as character
    }
#endif // defined(__AVR_ATmega32U4__)
  }  
} // I2C::receive

uint8_t blob::I2C::getAddress() {return _address;}

long blob::I2C::getFreq() {return _freq;}
