/********* blob robotics 2014 *********
 *  title: i2c.h
 *  brief: interface for generic I2C device
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_I2C_H
#define B_I2C_H

#include "blob/types.h"

namespace blob {

class I2C
{
  public:
    I2C (uint8_t address = 0x00, long freq = 400000);

    uint8_t getAddress();
    long getFreq ();
    void init ();
    void writeReg (const uint8_t reg, const uint8_t data);
    void readReg  (const uint8_t reg, uint8_t size, uint8_t *data);
    void request  (const uint8_t reg);
    void receive  (uint8_t size, uint8_t *data);

  private:

    uint8_t _address;
    static long _freq;
    static bool _started;
};

}

#endif /* B_I2C_H */
