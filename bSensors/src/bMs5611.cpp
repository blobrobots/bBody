/********* blob robotics 2014 *********
 *  title: bMs5611.cpp
 *  brief: driver for Hcm5883 magnetometer
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "bMs5611.h"

#include "bMath.h"

#if defined(__linux__)
  #include <cstring>
  #include <iostream>
#endif // defined(__linux__)

/* Oversampling ratio 
Precision en la medida. Cuanto mayor sea la 
precision mayor es el tiempo que tarda el sensor el generar el dato. Si
se pasa 0 como parametro se toma el valor por defecto que es OSR_4096,
que mide con una precision de 0.012 mbar y tarda en generar cada medida
8.3 mseg. Los posibles valores del OSR son los siguientes:   
- OSR_256  ->  Resolucion 0.065 mbar, tiempo medida 0.5 mseg.
- OSR_512  ->  Resolucion 0.042 mbar, tiempo medida 1.1 mseg.
- OSR_1024 ->  Resolucion 0.027 mbar, tiempo medida 2.1 mseg.
- OSR_2048 ->  Resolucion 0.018 mbar, tiempo medida 4.1 mseg.
- OSR_4096 ->  Resolucion 0.012 mbar, tiempo medida 8.3 mseg.
*/

#define MS5611_OSR_256     0x00
#define MS5611_OSR_512     0x02
#define MS5611_OSR_1024    0x04
#define MS5611_OSR_2048    0x06
#define MS5611_OSR_4096    0x08

#define MS5611_RESET_REG   0x1E
#define MS5611_PRESS_REG   0x40
#define MS5611_TEMP_REG    0x50

const int32_t blob::MS5611::MAX_TEMP_RATE = 2;
const int32_t blob::MS5611::MAX_PRESS_RATE = 2;
const int32_t blob::MS5611::INIT_CYCLES = 100;
const int32_t blob::MS5611::MAX_I2C_ERRORS = 10;

blob::MS5611::MS5611 (uint8_t address)  : _i2c(address)
{
  
   memset(_c,0,sizeof(_c));
   _osr = MS5611_OSR_4096; // Oversampling ratio, should be 4096
   
   _rawTemp = 0;
   _prevTemp = 0;
   _rawPress = 0;
   _prevPress = 0;
   
   _temp = 0.0f;
   _press = 0.0f;
   _altitude = 0.0f;
   _elevation = 0.0f;
} // MS5611::MS5611

void blob::MS5611::init()
{
  uint8_t msg[2];
  uint8_t nError = 0;
#if defined(__AVR_ATmega32U4__)
  if(Serial) {
    Serial.print("init ms5611 at 0x"); 
    Serial.print(getAddress(),HEX);
    Serial.print("..."); 
  }
#endif // defined(__AVR_ATmega32U4__)
#if defined (__linux__)
  std::cout << "init ms5611 at 0x" << std::hex << getAddress() << std::dec << " ...";
#endif // defined(__linux__)

  _i2c.init();

  blob::Task::delay(10);
  
  _i2c.request(MS5611_RESET_REG); // reset FIXME add reset() function

  blob::Task::delay(40);
  
  // Read Calibration Data C1-C6
  for(uint8_t i = 0; i < 6; i++) {
    msg[0] = msg[1] = 0;
    nError = 0;
    uint8_t reg = 0xA2+2*i;
    while (!_c[i])
    {
      blob::Task::delay(10);

      _i2c.readReg(reg, 2, msg);
      _c[i] = (msg[0]<<8) | msg[1];         
      if (!_c[i]) {
        if(nError++ == MAX_I2C_ERRORS)
        {
          return;
        }
      }
    }
  }
  
  blob::Task::delay(10);

#if defined(__AVR_ATmega32U4__)
  if(Serial) Serial.println(" done.");
#endif // defined(__AVR_ATmega32U4__)
#if defined (__linux__)
  std::cout << " done." << std::endl;
#endif // defined(__linux__)
} // MS5611::init

/* Update data from barometer */
void blob::MS5611::update()
{
  static float lastDt = _dt;
  
  if(updateTempPress()) {
    
    updateAltitude();
    
    /* update elevation at the beginning */
    if(!_ready && _index < INIT_CYCLES) {
      if(_index == 0)
        _elevation = _altitude;
      else
        _elevation = 0.9f*_elevation+0.1f*_altitude;
    } else {
      _ready = true;
    }
    _dt = lastDt + _dt;
    _index++;
    
#ifdef __DEBUG__
    print();
#endif

  } else {
    float auxDt = _dt;
    _dt = lastDt;
    lastDt = auxDt;
  }
  
  return;
} // MS5611::update

/* Update raw data from barometer */
bool blob::MS5611::updateTempPress()
{
  static uint8_t updateStep = 0;
  
  bool retval = false;
  uint8_t msg[3] = {0, 0 ,0};
  
  if(updateStep == 0) {
      // request temperature
      _i2c.request(MS5611_TEMP_REG + _osr);
      updateStep = 1;

  } else if (updateStep == 1) {
    // receive temperature after 20 ms wait
    _i2c.readReg(0x00, 3, msg); // i2cRead(nI2C, i2cAddr, 0x00, msg, 3) != OK)
    if(msg)
      _rawTemp = ((uint32_t)msg[0]<<16 | (uint32_t)msg[1]<<8 | (uint32_t)msg[2]); 
    // request pressure
    _i2c.request(MS5611_PRESS_REG + _osr);
    updateStep = 2;

  } else if (updateStep == 2) {
    // receive pressure after 20 ms wait
    _i2c.readReg(0x00, 3, msg);
    if(msg)
      _rawPress = ((uint32_t)msg[0]<<16 | (uint32_t)msg[1]<<8 | (uint32_t)msg[2]); 
    
    // request temperature
    _i2c.request(MS5611_TEMP_REG + _osr);
    updateStep = 1;
    retval = true;
  }

  return retval;
} // MS5611::updateTempPress

/* Update data from barometer */
void blob::MS5611::updateAltitude()
{  
  int64_t offset       = 0;
  int64_t sensitivity  = 0;
  int64_t offset2      = 0;
  int64_t sensitivity2 = 0;
  
  int32_t dT = _rawTemp - ((int32_t)_c[4] << 8);
  int32_t auxTemp =  (2000 + (int32_t)(((int64_t)dT * (int64_t)_c[5]) >> 23));   
  
  /* update corrected temperature in Celsius (previous to filter)*/
  _temp = (float)auxTemp/100.f;
   
  /* rate limiter filter */
  if(_prevTemp != 0)
    auxTemp = _prevTemp + blob::math::constrained(auxTemp - _prevTemp, -MAX_TEMP_RATE, MAX_TEMP_RATE);
  
  _prevTemp = auxTemp;
     
  /* calculate pressure */
  offset      = ((uint64_t)_c[1] << 16) + (((int64_t)_c[3] * (int64_t)dT) >> 7);
  sensitivity = ((uint64_t)_c[0] << 15) + (((int64_t)_c[2] * (int64_t)dT) >> 8);

  if (auxTemp < 2000)
  {
    offset2      = 5 * ((auxTemp - 2000)*(auxTemp - 2000)) / 2;
    sensitivity2 = 5 * ((auxTemp - 2000)*(auxTemp - 2000)) / 4;

    if (auxTemp < -1500)
    {
      offset2      = offset2 + 7 * ((auxTemp + 1500)*(auxTemp + 1500));
      sensitivity2 = sensitivity2 + 11 * ((auxTemp + 1500)*(auxTemp + 1500)) / 2;
    }
  } 
   
  offset      = offset - offset2;
  sensitivity = sensitivity - sensitivity2;
  
  /* pressure in milibars e-2 */
  int32_t auxPress = (((_rawPress * sensitivity) >> 21) - offset) >> 15;
      
  /* pressure in millibars (previous to filter) */
  _press = ((float)auxPress)/100.f;
  
  /* rate limiter filter */
  if(_prevPress != 0)
    auxPress = _prevPress + blob::math::constrained(auxPress - _prevPress, -MAX_PRESS_RATE, MAX_PRESS_RATE);
    
  _prevPress = auxPress;
  
  /* calculate altitude from filtered pressure and temperature */
  _altitude = (44330.0f * (1.0f - pow((float)auxPress / 101325.0f, 0.190295f)));
   
  return;
} // MS5611::updateAltitude

uint8_t blob::MS5611::getAddress ()
{
   return _i2c.getAddress ();

} // MS5611::getAddress
