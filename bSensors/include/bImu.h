/********* blob robotics 2014 *********
 *  title: bImu.h
 *  brief: interface for generic imu
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_IMU_H
#define B_IMU_H

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

#include "bTask.h"
#include "bMath.h"

namespace blob {

class Imu : public Task
{
  public:
    virtual Vector3d<float> getAcc  () {return _acc;}
    virtual Vector3d<float> getGyro () {return _gyro;}
    virtual Vector3d<float> getMag  () {return _mag;}
    virtual Vector3d<float> getEuler() {return _euler;}

    void print (bool ln = true) {
#if defined(__AVR_ATmega32U4__)
      if (Serial) {
        Serial.print("blob::Imu - ");
        Serial.print(_euler.x); Serial.print(" ");
        Serial.print(_euler.y); Serial.print(" ");
        Serial.print(_euler.z); Serial.print(" ");
        Serial.print(_acc.x); Serial.print(" ");
        Serial.print(_acc.y); Serial.print(" ");
        Serial.print(_acc.z); Serial.print(" ");
        Serial.print(_gyro.x); Serial.print(" ");
        Serial.print(_gyro.y); Serial.print(" ");
        Serial.print(_gyro.z); Serial.print(" ");
        Serial.print(_mag.x); Serial.print(" ");
        Serial.print(_mag.y); Serial.print(" ");
        Serial.print(_mag.z); Serial.print(" ");
        Serial.print(_dt); 
        if (ln)
          Serial.println(" - ");
        else
          Serial.print(" - ");
      }
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
      std::cout << "blob::Imu - " 
                << _euler.x << " " 
                << _euler.y << " " 
                << _euler.z << " " 
                << _acc.x << " " 
                << _acc.y << " " 
                << _acc.z << " "
                << _gyro.x << " " 
                << _gyro.y << " " 
                << _gyro.z << " "
                << _mag.x << " " 
                << _mag.y << " " 
                << _mag.z << " "
                << _dt << " - ";
      if(ln) std::cout << std::endl;       
#endif // defined(__linux__)
    }

  protected:
    Vector3d<float> _acc;
    Vector3d<float> _gyro;
    Vector3d<float> _mag;
    Vector3d<float> _euler;  
};

}

#endif /* B_IMU_H */
