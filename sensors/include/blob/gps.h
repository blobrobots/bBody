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
 * \file       gps.h
 * \brief      interface for generic global positioning system (GPS)
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_GPS_H
#define B_GPS_H

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

#include "blob/math.h"

namespace blob {

/**
 * Implements generic attitude and heading reference system (AHRS).
 */
class Gps
{
  public:

    /**
     * Provides latitude (deg), longitude (deg) and altitude (m).
     * \return  three dimension vector with latitude, longitude, altitude
     */
    virtual Vector3d<float> getLla() {return _lla;}

    /**
     * Provides three axis velocity (m/s) in NED reference system.
     * \return  three dimension vector with NED velocity (m/s)
     */    
    virtual Vector3d<float> getVel() {return _vel;}

    /**
     * Provides cartesian position (meters) in NED reference system and with 
     * origin on wake up latitude, longitude and altitude.
     * \return  three dimension vector with NED position (m)
     */    
    virtual Vector3d<float> getPos() {return _pos;}

    /**
     * Indicates if data from sensor is valid.
     * \return  true if data is valid, false otherwise
     */
    virtual bool isValid() {return _valid;}

    /**
     * Prints GPS data on standard output.
     * \param ln  indicates if an end of line character is to be printed
     */
    void printGps (bool ln = true) {
#if defined(__AVR_ATmega32U4__)
      if (Serial) {
        Serial.print("blob::Gps - ");
        Serial.print(_valid); Serial.print(" ");
        Serial.print(_lla[0]); Serial.print(" ");
        Serial.print(_lla[1]); Serial.print(" ");
        Serial.print(_lla[2]); Serial.print(" ");
        Serial.print(_vel.x); Serial.print(" ");
        Serial.print(_vel.y); Serial.print(" ");
        Serial.print(_vel.z); Serial.print(" ");
        Serial.print(_pos.x); Serial.print(" ");
        Serial.print(_pos.y); Serial.print(" ");
        Serial.print(_pos.z); Serial.print(" ");
        Serial.print(_dt); 
        if (ln)
          Serial.println(" - ");
        else
          Serial.print(" - ");
      }
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
      std::cout << "blob::Gps - " 
                << _valid << " " 
                << _lla[0] << " " 
                << _lla[1] << " " 
                << _lla[2] << " " 
                << _vel.x << " " 
                << _vel.y << " " 
                << _vel.z << " " 
                << _pos.x << " " 
                << _pos.y << " " 
                << _pos.z << " " 
                << _dt << " - ";
      if(ln) std::cout << std::endl;       
#endif // defined(__linux__)
    }

  protected:
    bool _valid;           /**< flag indicating if last measurement was valid */
    Vector3d<float> _lla;  /**< latitude (deg) longitude (deg) altitude (m) */
    Vector3d<float> _vel;  /**< velocity (m/s) in NED reference system */
    Vector3d<float> _pos;  /**< position (m) from wakeup point in NED */
};

}

#endif /* B_AHRS_H */
