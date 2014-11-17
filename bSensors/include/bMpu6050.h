/********* blob robotics 2014 *********
 *  title: bMpu6050.h
 *  brief: interface for Mpu6050 imu
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_MPU6050_H
#define B_MPU6050_H

#include "bI2c.h"
#include "bImu.h"

#define B_MPU6050_ADDRESS    0x68 // address pin AD0 low (GND)
// #define B_MPU6050_ADDRESS    0x69 // address pin AD0 high (VCC)

namespace blob {

class MPU6050 : public Imu
{
  public:
    MPU6050(uint8_t address = B_MPU6050_ADDRESS);

    uint8_t getAddress();
    bool isMagCoupled();
    void coupleMag (uint8_t magAddr, uint8_t magDataReg, float magScale = 1.f, Vector3d<float> magGain = Vector3d<float>(1.f, 1.f, 1.f));
    void setI2cBypass (bool on);
    void setI2cMasterMode (bool on);
    
    void init   ();
    void update ();        // update acc, gyro and mag if coupled
    
    bool calibrate ();
    
    void updateAcc ();     // update only acc
    void updateGyro ();    // update only gyro
    void updateMag ();     // update mag if coupled
    void updateAccGyro (); // update acc and gyro
    void updateEuler (); // estimate euler
    
    float getTemp ();
    
  private:

    blob::I2C _i2c;
    
    bool _magCoupled;
    
    Vector3d<int32_t> _accRaw;
    Vector3d<int32_t> _gyroRaw;
    Vector3d<int32_t> _magRaw;
    int32_t _tempRaw;

    float _temp;
    
    Vector3d<float> _accOff;
    Vector3d<float> _gyroOff;
    Vector3d<float> _magGain;
    
    float _accScale;
    float _gyroScale;
    float _magScale;
};

}

#endif /* B_MPU6050_H */
