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
 * \file       ahrs.h
 * \brief      interface for generic attitude and heading ref. system (AHRS)
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_AHRS_H
#define B_AHRS_H

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

#include "blob/math.h"

namespace blob {

/**
 * Implements generic attitude and heading reference system (AHRS).
 */
class Ahrs
{
  public:

    /**
     * Provides euler angles in radians and NED reference system.
     * \return  euler angles in radians and NED reference system
     */
    virtual Vector3d<float> getEuler() {return _euler;}

    /**
     * Prints euler angles on standard output.
     * \param ln  indicates if an end of line character is to be printed
     */
    void printAhrs (bool ln = true) {
#if defined(__AVR_ATmega32U4__)
      if (Serial) {
        Serial.print("blob::Ahrs - ");
        Serial.print(_euler.x); Serial.print(" ");
        Serial.print(_euler.y); Serial.print(" ");
        Serial.print(_euler.z); Serial.print(" ");
        Serial.print(_dt); 
        if (ln)
          Serial.println(" - ");
        else
          Serial.print(" - ");
      }
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
      std::cout << "blob::Ahrs - " 
                << _euler.x << " " 
                << _euler.y << " " 
                << _euler.z << " " 
                << _dt << " - ";
      if(ln) std::cout << std::endl;       
#endif // defined(__linux__)
    }

  protected:
    Vector3d<float> _euler;  /**< euler angles in radians and NED ref. system */
};

}

#endif /* B_AHRS_H */
