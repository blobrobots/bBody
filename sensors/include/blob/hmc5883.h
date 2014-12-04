/********* blob robotics 2014 *********
 *  title: hmc5883.h
 *  brief: interface for Hmc5883 imu
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_HMC5883_H
#define B_HMC5883_H

#include "blob/i2c.h"
#include "blob/math.h"
#include "blob/task.h"

#define B_HMC5883_ADDRESS  0x1E

namespace blob {

class HMC5883 : public Task
{
  public:
    HMC5883(uint8_t address = B_HMC5883_ADDRESS);

    uint8_t getAddress();
    uint8_t getDataReg();
    
    void init ();
    void update ();
    
    void setMode (const uint8_t mode);
    void setMeasurementMode (const uint8_t mode);
    void setMeasurementGain (const uint8_t gain);
    void setOutputRate (const uint8_t rate);
    uint8_t getStatus();
    uint8_t getConfigRegA ();
    uint8_t getConfigRegB ();
    
    bool calibrate ();
    
    void updateMag ();
    
    Vector3d<float> getGain();
    float getScale ();
    
    void print(bool ln = true);

  private:
  
    blob::I2C _i2c;

    Vector3d<int32_t> _raw;
    Vector3d<float> _gain;
    Vector3d<float> _mag;
    float _scale;
};

}

#endif /* B_HM5883_H */
