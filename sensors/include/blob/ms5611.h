/********* blob robotics 2014 *********
 *  title: ms5611.h
 *  brief: interface for Ms5611 baro
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_MS5611_H
#define B_MS5611_H

#include "blob/i2c.h"
#include "blob/baro.h"

// #define B_MS5611_ADDRESS  0x76 // CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)
#define B_MS5611_ADDRESS  0x77 // CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)

namespace blob {

class MS5611 : public Baro
{
  public:
    MS5611(uint8_t address = B_MS5611_ADDRESS);

    void init   ();
    void update ();

    uint8_t getAddress();
    
  private:
  
    blob::I2C _i2c;
    
    uint16_t _c[6];   // calibration coefs
    uint8_t _osr;     // oversampling ratio     
    
    uint32_t _rawTemp;
    int32_t _prevTemp;
    uint32_t _rawPress;
    int32_t _prevPress;

    bool updateTempPress ();
    void updateAltitude  ();

    static const int32_t MAX_TEMP_RATE;
    static const int32_t MAX_PRESS_RATE;
    static const int32_t INIT_CYCLES;
    static const int32_t MAX_I2C_ERRORS;
};

}

#endif /* B_MS5611_H */
