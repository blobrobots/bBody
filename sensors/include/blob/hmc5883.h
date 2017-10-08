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
 * \file       hmc5883.h
 * \brief      interface for Hmc5883 magnetometer
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_HMC5883_H
#define B_HMC5883_H

#include "blob/i2c.h"
#include "blob/math.h"
#include "blob/task.h"

#define B_HMC5883_ADDRESS  0x1E

namespace blob {

/**
 * Implements driver for HMC5883 magnetometer.
 */
class HMC5883 : public Mag, public Task
{
  public:
    /** 
     * Status register possible values
     */
    enum Status {Ready=0x01, Lock=0x02, RegEnabled=0x04};

    /** 
     * Mode register (MR) possible values (MD1..MD0)
     */
    enum Mode {Continuous=0x00, Single=0x01, Idle=0x02, Sleep=0x03};

    /** 
     * Measurement mode at Configuration Register A (CRA) (MSO, MS1)
     */
    enum MeasurementMode {Normal=0x00, PositiveBias=0x01, NegativeBias=0x02};

    /**
     * Initializes sensor i2c address and internal variables.
     */
    HMC5883(uint8_t address=B_HMC5883_ADDRESS);

    /**
     * Initializes communication with sensor.
     */
    void init   ();

    /**
     * Updates data from gyroscope, accelerometer and magnetometer if coupled.
     */
    void update ();

    /**
     * Updates calibration parameters for sensors.
     * \return  true if successful, false otherwise
     */
    bool calibrate ();

    /**
     * Updates data from magnetometer.
     */
    void updateMag ();

    /**
     * Provides sensor i2c address.
     * \return  sensor i2c address
     */
    uint8_t getAddress();

    /**
     * Provides sensor i2c register where data is stored.
     * \return  sensor i2c data register
     */
    uint8_t getDataReg();
    
    /**
     * Changes sensor mode (continuous, single, idle, sleep).
     * \param mode  sensor mode
     * \return      true if successful, false otherwise
     * \sa Mode 
     */
    bool setMode (const uint8_t& mode);

    /**
     * Provides sensor mode (continuous, single, idle, sleep).
     * \return  sensor mode
     * \sa Mode 
     */
    uint8_t getMode ();

    /**
     * Changes sensor measurement mode (normal, positive bias, negative bias).
     * \param mode  sensor measurement mode
     * \return      true if successful, false otherwise
     * \sa MeasurementMode
     */
    bool setMeasurementMode (const uint8_t& mode);

    /**
     * Provides sensor measurement mode (normal, positive bias, negative bias).
     * \return  sensor measurement mode
     * \sa MeasurementMode
     */
    uint8_t getMeasurementMode ();

    /**
     * Changes sensor measurement gain.
     * \param gain  sensor measurement gain
     * \return      true if successful, false otherwise
     */
    bool setMeasurementGain (const float& gain);

    /**
     * Provides sensor measurement gain.
     * \return  sensor measurement gain
     */
    float getMeasurementGain ();

    /**
     * Changes sensor output rate. Valid values are 0.75, 1.5, 3, 7.5, 15, 30 
     * and 75 Hz.
     * \param rate  sensor output rate (0.75, 1.5, 3, 7.5, 15, 30,75)
     * \return      true if successful, false otherwise
     */
    bool setOutputRate (const float& rate);

    /**
     * Provides sensor output rate. Valid values are 0.75, 1.5, 3, 7.5, 15, 30 
     * and 75 Hz.
     * \return  sensor output rate (0.75, 1.5, 3, 7.5, 15, 30,75)
     */
    float getOutputRate ();

    /**
     * Changes sensor sample averaging Valid values are 1, 2, 4 and 8
     * \param average  sensor sample averaging (1, 2, 4, 8)
     * \return         true if successful, false otherwise
     */
    bool setSampleAverage (const uint8_t& average);

    /**
     * Provides sensor sample averaging Valid values are 1, 2, 4 and 8
     * \return  sensor sample averaging (1, 2, 4, 8)
     */
    uint8_t getSampleAverage ();

    /**
     * Provides sensor status (ready, lock, reg.enabled).
     * \return  sensor status
     * \sa Status
     */    
    uint8_t getStatus();
    
    /**
     * Provides sensor calibration gain vector.
     * \return  sensor calibration gain vector
     */
    Vector3d<float> getGain();

    /**
     * Provides sensor scale.
     * \return  sensor scale
     */
    float getScale ();
    
  private:
    /**
     * Provides configuration register A.
     * \return  configuration register A
     * \sa
     */
    uint8_t getConfigRegA ();

    /**
     * Provides configuration register B.
     * \return  configuration register B
     * \sa
     */
    uint8_t getConfigRegB ();


    blob::I2C _i2c;         /**< i2c bus manager */

    Vector3d<int32_t> _raw; /**< raw magnetometer readings */
    Vector3d<float> _gain;  /**< magnetometer calibration gain vector */
    float _scale;           /**< magnetometer scale */
};

}

#endif /* B_HM5883_H */
