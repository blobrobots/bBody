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
 * \file       baro.h
 * \brief      interface for MS5611 barometer
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_BARO_H
#define B_BARO_H

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

#include "blob/types.h"

namespace blob {

/**
 * Implements generic barometer class.
 */
class Baro
{
  public:
    /**
     * Initializes barometer internal variables to reset elevation.
     */
    Baro () : _setElevation(true) {};

    /**
     * Provides pressure in pascals.
     * \return  pressure in pascals
     */
    virtual float getPress     () {return _press;}

    /**
     * Provides altitude over sea level in meters.
     * \return  altitude over sea level in meters
     */
    virtual float getAltitude  () {return _altitude;}

    /**
     * Provides altitude of ground over sea level in meters.
     * \return  altitude of ground over sea level in meters
     */
    virtual float getElevation () {return _elevation;}

    /**
     * Provides distance to ground in meters.
     * \return  distance to ground in meters
     */
    virtual float getHeight    () {return (_altitude - _elevation);}
    
    /**
     * Prints barometer data on standard output. Includes temperature, pressure, 
     * altitude elevation and height.
     * \param ln  indicates if an end of line character is to be printed
     */
    void printBaro (bool ln = true) {

#if defined(__AVR_ATmega32U4__)
      if (Serial) {
        Serial.print("blob::Baro - ");
        Serial.print(_press); Serial.print(" ");
        Serial.print(_altitude); Serial.print(" ");
        Serial.print(_elevation); Serial.print(" ");
        Serial.print(getHeight()); Serial.print(" ");
        Serial.print(_dt);
        if (ln)
          Serial.println(" - ");
        else
          Serial.print(" - ");
      }
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
      std::cout << "blob::Baro - " 
                << _press << " " 
                << _altitude << " " 
                << _elevation << " " 
                << getHeight() << " "
                << _dt << " - ";
      if (ln) std::cout << std::endl;       
#endif // defined(__linux__)
    }

  protected:
    bool _setElevation; /**< flag to recalculate ground altitude */
  
    float _press;       /**< pressure in pascals */
    float _altitude;    /**< altitude over sea level in meters */
    float _elevation;   /**< altitude of ground over sea level in meters */

};

}

#endif /* B_MS5611_H */
