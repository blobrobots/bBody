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
 * \file       mag.h
 * \brief      interface for generic magnetometer
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_MAG_H
#define B_MAG_H

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

#include "blob/math.h"

namespace blob {

/**
 * Implements generic magnetometer class.
 */
class Mag
{
  public:

    /**
     * Provides three axis magnetometer measurements in gauss.
     * \return  three axis magnetometer measurements in gauss
     */
    virtual Vector3d<float> getMag  () {return _mag;}

    /**
     * Prints three axis magnetometer data on standard output.
     * \param ln  indicates if an end of line character is to be printed
     */
    void printMag (bool ln = true) {
#if defined(__AVR_ATmega32U4__)
      if (Serial) {
        Serial.print("blob::Mag - ");
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
      std::cout << "blob::Mag - " 
                << _mag.x << " " 
                << _mag.y << " " 
                << _mag.z << " "
                << _dt << " - ";
      if(ln) std::cout << std::endl;       
#endif // defined(__linux__)
    }

  protected:
    Vector3d<float> _mag; /**< three axis magnetometer measurements in gauss */
  
};

}

#endif /* B_IMU_H */
