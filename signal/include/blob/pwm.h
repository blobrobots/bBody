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
 * \file       pwm.cpp
 * \brief      interface for PWM signal generator
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_PWM_H
#define B_PWM_H

#include "blob/types.h"

#define B_PWM_MAX_NUM 4

namespace blob {

/**
 * Implements Pulse-Width Modulation (PWM) signal generator.
 */
class PWM
{
  public:
    /**
     * Initializes PWM generator configuration.
     */
    PWM ();
    /**
     * Indicates whether PWM generator is ready or not.
     * \return  true if ready, false otherwise
     */
    bool isReady();
    /**
     * Initializes PWM signal.
     * \return  true if successful, false otherwise
     */
    virtual bool init ()=0;
    /**
     * Provides current duty cycle of PWM signal.
     * \return  duty cycle length in us
     */
    uint16_t getDutyCycle ();
    /**
     * Provides auto-assigned hardware pin of PWM signal.
     * \param us  duty cycle length in us
     * \return  hardware pin of PWM signal
     */
    uint8_t  getPin ();
    /**
     * Provides channel index [0-B_PWM_MAX_NUM] of PWM signal.
     * \return  channel index of PWM signal
     */
    uint8_t  getIndex ();    
    /**
     * Provides PWM signal frequency in Hz.
     * \return  PWM signal frequency in Hz
     */
    uint16_t getFrequency ();    // (Hz)
    /**
     * Provides PWM signal period in us.
     * \return  PWM signal period in us
     */    
    uint16_t getPeriod ();       // (us)

  protected:
    uint8_t _index; /**< current PWM generator channel index [0-B_PWM_MAX_NUM]*/
    uint8_t _pin;   /**< current PWM generator board pin available */
    uint16_t _period;     /**< PWM period in us*/
    uint16_t _dutyCycle;  /**< PWM signal dutycycle in us*/
};

/**
 * Implements Pulse-Width Modulation (PWM) signal generator.
 */
class PWMout : public PWM
{
  public:
    /**
     * Initializes PWM signal.
     * \return  true if successful, false otherwise
     */
    bool init ();
    /**
     * Changes duty cycle of PWM signal.
     * \param us  duty cycle length in us
     * \return  true if successful, false otherwise
     */
    bool setDutyCycle (uint16_t us);
    /**
     * Provides total number of PWM generators enabled.
     * \return  total number of PWM generators enabled
     */
    static uint8_t  getNumInUse();

  protected:
    static const uint8_t  _pinList[B_PWM_MAX_NUM];  /**< board pins available */
    static uint8_t        _inUse;       /**< number of PWM generators enabled */
};

/**
 * Implements Pulse-Width Modulation (PWM) signal reader.
 */
class PWMin : public PWM
{
  public:
    /**
     * Initializes PWM signal.
     * \return  true if successful, false otherwise
     */
    bool init ();
    /**
     * Provides total number of PWM generators enabled.
     * \return  total number of PWM generators enabled
     */
    static uint8_t getNumInUse ();
    /**
     * Synchronizes all input pwm signals to check if they are inside margins
     * dutycycle between min/max and period is less than 65556 (max uint16_t).
     */
    static void sync ();
    /**
     * Synchronizes period, checking if it is less than 65556 (max uint16_t).
     * \return  true if period is inside margins, false otherwise
     */
    bool syncPeriod ();

#if defined(__AVR_ATmega32U4__)
    /**
     * Callback to detect pin changes to calculate signal dutycycle and period
     */
    void onPinChange ();
    /**
     * Callback to detect pin 0 changes to calculate signal dutycycle and period
     */
    static void onPinChange0 ();
    /**
     * Callback to detect pin 1 changes to calculate signal dutycycle and period
     */
    static void onPinChange1 ();
    /**
     * Callback to detect pin 2 changes to calculate signal dutycycle and period
     */
    static void onPinChange2 ();
    /**
     * Callback to detect pin 3 changes to calculate signal dutycycle and period
     */
    static void onPinChange3 ();
#endif // defined(__AVR_ATmega32U4__)    

  protected:
    bool _pinHigh;      /**< indicates if pwm signal input pin is high or low */    
    uint32_t _pwmStart;              /**< us count on which pwm signal starts */
    static const uint8_t  _pinList[B_PWM_MAX_NUM];  /**< board pins available */
    static uint8_t        _inUse;       /**< number of PWM generators enabled */
    static PWMin * _ref[B_PWM_MAX_NUM]; /**< list of currently available 
                                            PWMin objects for external access */
};

}

#endif /* B_PWM_H */

