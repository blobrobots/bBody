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
 * \file       ms5611.h
 * \brief      interface for Ms5611 barometer
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_MS5611_H
#define B_MS5611_H

#include "blob/i2c.h"
#include "blob/baro.h"

/* CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC) */
// #define B_MS5611_ADDRESS  0x76

/* CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND) */
#define B_MS5611_ADDRESS  0x77

namespace blob {

/**
 * Implements driver for MS5611 barometer.
 */
class MS5611 : public Baro, public Temp, public Task
{
  public:
    /**
     * Initializes barometer i2c address and internal variables.
     */
    MS5611(uint8_t address = B_MS5611_ADDRESS);

    /**
     * Initializes communication with sensor.
     */
    void init   ();

    /**
     * Updates data from sensor and altitude.
     */
    void update ();

    /**
     * Provides sensor i2c address.
     * \return  sensor i2c address
     */
    uint8_t getAddress();
    
  private:

    /**
     * Performs data adquisition of temperature and pressure from sensor.
     * \return  true if successul, false otherwise
     */
    bool updateTempPress ();

    /**
     * Corrects raw pressure with temperature and calculates altitude.
     */
    void updateAltitude  ();

    blob::I2C _i2c;     /**< i2c channel */
    
    uint16_t _c[6];     /**< calibration coeficients */
    uint8_t _osr;       /**< oversampling ratio */     
    
    uint32_t _rawTemp;  /**< raw temperature */
    int32_t _prevTemp;  /**< previous raw temperature */
    uint32_t _rawPress; /**< raw pressure */
    int32_t _prevPress; /**< previous raw pressure */

    static const int32_t MAX_TEMP_RATE;  /**< maximum temperature change rate */
    static const int32_t MAX_PRESS_RATE; /**< maximum pressure change rate */
    static const int32_t INIT_CYCLES;    /**< setup cycles  */
    static const int32_t MAX_I2C_ERRORS; /**< maximum number of i2c errors */
};

}

#endif /* B_MS5611_H */
