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
 * \file       esc.h
 * \brief      interface for ESC to control brushless motor
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_ESC_H
#define B_ESC_H

#if defined(__AVR_ATmega32U4__)
  #include "Arduino.h"
#endif

#include "blob/pwm.h"

namespace blob {

class ESC // : Motor
{
  public:
    ESC (uint32_t minUs = 1000, uint32_t maxUs = 2000, uint32_t periodUs = 10000);
// Motor
    boolean isReady();
    
    void    init();
    void    setOutput (float output); // normalized output [0.0 - 1.0]
    float   getOutput (); // normalized output [0.0 - 1.0]
    bool    isOn ();
    void    switchOn ();
    void    switchOff ();
// ESC    
    void     setRange ();  // us
    void     setConfig (); // here standard esc config
    uint32_t getMaxUs ();  // us
    uint32_t getMinUs ();  // us
    uint32_t getPeriod (); // us
    uint8_t  getIndex (); 
    uint8_t  getPin ();

  private:
// Motor
    bool _ready;
    bool _on;
// ESC    
    blob::PWM _pwm;
    uint32_t _maxUs;
    uint32_t _minUs;  
    // standard config parameters missing

};

}

#endif /* B_ESC_H */

