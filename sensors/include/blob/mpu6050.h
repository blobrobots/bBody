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
 * \file       mpu6050.h
 * \brief      interface for Mpu6050 imu
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_MPU6050_H
#define B_MPU6050_H

#include "blob/i2c.h"
#include "blob/imu.h"

#define B_MPU6050_ADDRESS    0x68 // address pin AD0 low (GND)
// #define B_MPU6050_ADDRESS    0x69 // address pin AD0 high (VCC)

namespace blob {

/**
 * Implements driver for MPU6050 IMU (gyroscope and accelerometer). It also has 
 * the possibility to connect an external magnetometer.
 */
class MPU6050 : public Imu, public Mag, public Temp, public Task
{
  public:
    /**
     * Initializes sensor i2c address and internal variables.
     */
    MPU6050(uint8_t address=B_MPU6050_ADDRESS);
    
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
     * Updates data from accelerometer only.
     */    
    void updateAcc ();

    /**
     * Updates data from gyroscope only.
     */    
    void updateGyro ();

    /**
     * Updates data from magnetometer only if coupled.
     */
    void updateMag ();
    
    /**
     * Updates data from gyroscope and accelerometer.
     */
    void updateAccGyro ();
    
    /**
     * Provides sensor i2c address.
     * \return  sensor i2c address
     */
    uint8_t getAddress();

    /**
     * Indicates if a magnetometer is connected and providing data.
     * \return  true if magnetometer is connected and providing data
     */
    bool isMagCoupled();

    /**
     * Connnects a magnetometer as a slave i2c device.
     * \param magAddr     i2c address of slave magnetometer
     * \param magDataReg  i2c register to retrieve data from magnetometer
     * \param magScale    scale for magnetometer data
     * \param magGain     gain to be applied to magnetometer data
     */
    void coupleMag (uint8_t magAddr, uint8_t magDataReg, float magScale = 1.f, 
                    Vector3d<float> magGain = Vector3d<float>(1.f, 1.f, 1.f) );

    /**
     * Puts i2c bus into bypass mode to communicate with slave device.
     * \param on  true if i2c bypass mode is to be siwtched on, false if off
     */
    void setI2cBypass (bool on);

    /**
     * Puts i2c bus into master mode.
     * \param on  true if i2c master mode is to be switched on, false if off
     */
    void setI2cMasterMode (bool on);

  private:

    blob::I2C _i2c;     /**< i2c bus manager */  
    
    bool _magCoupled;   /**< flag indicatting if magnetometer is connected */
    
    Vector3d<int32_t> _gyroRaw;   /**< raw gyroscope readings */
    Vector3d<int32_t> _accRaw;    /**< raw accelerometer readings */
    Vector3d<int32_t> _magRaw;    /**< raw magnetometer readings */
    int32_t _tempRaw;             /**< raw temperature reading */

    Vector3d<float> _gyroOff;     /**< gyroscope calibration offsets */
    Vector3d<float> _accOff;      /**< accelerometer calibration offsets */
    Vector3d<float> _magGain;     /**< magnetometer calibration gain */
    
    float _gyroScale;             /**< gyroscope scale */
    float _accScale;              /**< accelerometer scale */
    float _magScale;              /**< magnetometer scale */
};

}

#endif /* B_MPU6050_H */
