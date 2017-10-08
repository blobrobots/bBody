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
 * \file       hmc5883.cpp
 * \brief      driver for Hmc5883 magnetometer
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include "blob/hmc5883.h"

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

/* Register addresses */
#define HMC5883_CONFIG_REG_A  0x00
#define HMC5883_CONFIG_REG_B  0x01
#define HMC5883_MODE_REG      0x02
#define HMC5883_DATA_REG      0x03
#define HMC5883_STATUS_REG    0x09
#define HMC5883_ID_REG        0x0A

/* Data register addresses */
#define HMC5883_DATA_REG_X    HMC5883_DATA_REG
#define HMC5883_DATA_REG_Y   (HMC5883_DATA_REG+4)
#define HMC5883_DATA_REG_Z   (HMC5883_DATA_REG+2)

/* Self test values */
#define HMC5883_X_SELF_TEST_GAUSS    1.16f /* x axis level with bias current */
#define HMC5883_Y_SELF_TEST_GAUSS    1.16f /* y axis level with bias current */
#define HMC5883_Z_SELF_TEST_GAUSS    1.08f /* z axis level with bias current */
#define HMC5883_LOW_SELF_TEST_GAUSS  243.f/390.f /* low limit when gain=5 */
#define HMC5883_HIGH_SELF_TEST_GAUSS 575.f/390.f /* high limit when gain=5*/

/* Measurement mode (M01..M00) at Configuration Register A (CRA) */
#define HMC5883_MEASUREMENT_MODE_NULLMASK (0b11111100)

/* Data Output Rate (D02..DO0) at Configuration Register A (CRA) */
#define HMC5883_RATE_NULLMASK  (0b11100011) 
#define HMC5883_RATE_0_75_HZ   (0)
#define HMC5883_RATE_1_5_HZ    (1<<2)
#define HMC5883_RATE_3_HZ      (2<<2)
#define HMC5883_RATE_7_5_HZ    (3<<2)
#define HMC5883_RATE_15_HZ     (4<<2)  /* Default value */
#define HMC5883_RATE_30_HZ     (5<<2)
#define HMC5883_RATE_75_HZ     (6<<2)

/* Data valid sample averaging at Configuration Register A (CRA) */
#define HMC5883_SAMPLE_AVERAGING_NULLMASK (0b10011111) 
#define HMC5883_SAMPLE_AVERAGING_1   0
#define HMC5883_SAMPLE_AVERAGING_2  (1<<5)
#define HMC5883_SAMPLE_AVERAGING_4  (2<<5)
#define HMC5883_SAMPLE_AVERAGING_8  (3<<5)

/* Gain (DN2..GN0) at Configuration Register B (CRB) */
#define HMC5883_GAIN_NULLMASK (0b00011111) 
#define HMC5883_GAIN_0_9_GA    0
#define HMC5883_GAIN_1_2_GA   (1<<5)  /* Default value */
#define HMC5883_GAIN_1_9_GA   (2<<5)
#define HMC5883_GAIN_2_5_GA   (3<<5)
#define HMC5883_GAIN_4_0_GA   (4<<5)
#define HMC5883_GAIN_4_6_GA   (5<<5)
#define HMC5883_GAIN_5_5_GA   (6<<5)
#define HMC5883_GAIN_7_9_GA   (7<<5)

blob::HMC5883::HMC5883 (uint8_t address)  : _i2c(address)
{
   _ready  = false;

   _gain.x = 1.0f;
   _gain.y = 1.0f;
   _gain.z = 1.0f;
   _scale = 1.0f;

} // HMC5883::HMC5883

void blob::HMC5883::init()
{
#if defined(__DEBUG__)
#if defined(__AVR_ATmega32U4__)
  if(Serial) {
    Serial.print("init hmc5883 at 0x"); 
    Serial.print(getAddress(),HEX);
    Serial.print(" ..."); 
  }
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  std::cout << "init hmc5883 at 0x" << std::hex << getAddress() << std::dec << " ...";
#endif // defined(__linux__)  
#endif // defined(__DEBUG__)
  _i2c.init();

  blob::Task::delayMs(10);

  _ready = calibrate();

  blob::Task::delayMs(10);
   
  setSampleAveraging(8.f);
  setOutputRate(75.f);
  setMeasurementMode(Normal);
  setMeasurementGain(1.9f);
  setMode(Continuous);
  
  blob::Task::delayMs(100);
#if defined(__DEBUG__)
#if defined(__AVR_ATmega32U4__)
  if(Serial) Serial.println(" done.");
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  std::cout << " done." << std::endl;
#endif // defined(__linux__)
#endif // defined(__DEBUG__)
} // HMC5883::init

bool  blob::HMC5883::calibrate()
{
   blob::Vector3d<int32_t> total; // 32 bit totals so they won't overflow.
   bool retval = true;    // Error indicator
#if defined(__DEBUG__)
#if defined(__AVR_ATmega32U4__)
  if(Serial) Serial.print("calibrating hmc5883... "); 
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  std::cout << "calibrating hmc5883... ";
#endif // defined(__linux__)
#endif // defined(__DEBUG__)
  blob::Task::delayMs(50);  // Wait before start

  setMeasurementMode(PositiveBias);

/* the first measurement after a gain change maintains the same gain as the 
   previous setting. The new gain is effective from the second measurement on */

  setGain(4.6f);
  setMode(Single);

  blob::Task::delayMs(100);

  update();  // Get one sample, and discard it

  for (uint8_t i = 0; i < 10; i++) { // Collect 10 samples
    setMode(Single);

    blob::Task::delayMs(100);

    update();

    // Since the measurements are noisy, they should be averaged rather than taking the max.
    total += _raw;

    // Detect saturation.
    if (-(1<<12) >= blob::math::minimum(_raw.x, blob::math::minimum(_raw.y, _raw.z))) {
      retval = false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  // Apply the negative bias. (Same gain)
  setMeasurementMode(NegativeBias);

  for (uint8_t i=0; i<10; i++) {

    setMode(Single);

    blob::Task::delayMs(100);

    update();  // Get the raw values in case the scales have already been changed.

    // Since the measurements are noisy, they should be averaged.
    total -= _raw;
      
    // Detect saturation.
    if (-(1<<12) >= blob::math::minimum(_raw.x, blob::math::minimum(_raw.y, _raw.z))) {
      retval = false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  if (retval) {
    _gain.x = fabs(820.0f*HMC5883_X_SELF_TEST_GAUSS*2.f*10.0f/total.x);
    _gain.y = fabs(820.0f*HMC5883_Y_SELF_TEST_GAUSS*2.f*10.0f/total.y);
    _gain.z = fabs(820.0f*HMC5883_Z_SELF_TEST_GAUSS*2.f*10.0f/total.z);

#if defined (__DEBUG__)
#if defined(__AVR_ATmega32U4__)
    if(Serial) {
      Serial.print("successful: gain = ("); 
      Serial.print(_gain.x);
      Serial.print(", ");
      Serial.print(_gain.y);
      Serial.print(", ");
      Serial.print(_gain.z);
      Serial.println(")");       
    }
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
std::cout << "successful: gain = (" << _gain.x << ", "<<  _gain.y 
                            << ", " << _gain.z << ")" << std::endl;
#endif // defined(__linux__)
#endif // defined (__DEBUG__)
  } else { //Something went wrong so get a best guess
#if defined (__DEBUG__)
#if defined(__AVR_ATmega32U4__)
    if(Serial) Serial.println("error: gain = (1.0,1.0,1.0)"); 
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
    std::cout << "error: gain = (1.0,1.0,1.0)" << std::endl;
#endif // defined(__linux__)
#endif // defined (__DEBUG__)
     _gain = blob::Vector3d <float> (1.f, 1.f, 1.f);
   } 
   return retval;
}

/* Update data from magnetometer */
void blob::HMC5883::update()
{
   updateMag ();
   _index++;

#if defined(__DEBUG__)
  print();
#endif // defined(__DEBUG__)

} // HMC5883::update

/* Update data from magnetometer */
void blob::HMC5883::updateMag ()
{
   uint8_t buffer[6];
   _i2c.readReg (HMC5883_DATA_REG, 6, buffer);

   _raw.x = (buffer[0]<<8) | buffer[1];
   _raw.y = (buffer[4]<<8) | buffer[5];
   _raw.z = (buffer[2]<<8) | buffer[3];

   _mag = (blob::Vector3d<float>)_raw/_scale;
   if(_ready)
     _mag&=_gain;

} // HMC5883::updateMag

float blob::HMC5883::getScale ()
{
   return _scale;

} // HMC5883::getAddress

uint8_t blob::HMC5883::getAddress ()
{
   return _i2c.getAddress ();

} // HMC5883::getAddress

uint8_t blob::HMC5883::getDataReg ()
{
   return HMC5883_DATA_REG;

} // HMC5883::getDataReg

blob::Vector3d<float> blob::HMC5883::getGain ()
{
   return _gain;
} // HMC5883::getGain

bool blob::HMC5883::setMeasurementGain (const float& gain)
{          
   uint8_t g=0;
   switch(gain)
   {
      case 0.9f:
        g=HMC5883_GAIN_0_9_GA;
        _scale = 1280.f;
        break;
      case 1.2f:
        g=HMC5883_GAIN_1_2_GA;
        _scale = 1024.f;
        break;
      case 1.9f:
        g=HMC5883_GAIN_1_9_GA;
        _scale = 768.f;
        break;
      case 2.5f:
        g=HMC5883_GAIN_2_5_GA;
        _scale = 614.f;
        break;
      case 4.f:
        g=HMC5883_GAIN_4_0_GA;
        _scale = 415.f;
        break;
      case 4.6f:
        g=HMC5883_GAIN_4_6_GA;
        _scale = 361.f;
        break;
      case 5.5f:
        g=HMC5883_GAIN_5_5_GA;
        _scale = 307.f;
        break;
      case 7.9f:
        g=HMC5883_GAIN_7_9_GA;
        _scale = 219.f;
        break;
      default:
        _scale = 1.f;
        return false;     
   }
   _i2c.writeReg(HMC5883_CONFIG_REG_B, gain);
  /* read register and set new rate */
  uint8_t regB = getConfigRegB();
  regB &= HMC5883_GAIN_NULLMASK;
  regB |= g;
  _i2c.writeReg(HMC5883_CONFIG_REG_B, regB);  

  return true;
} // HMC5883::setMeasurementGain

float blob::HMC5883::getMeasurementGain ()
{ 
  float gain = 0.f; 
  uint8_t g=0;
  _i2c.readReg(HMC5883_CONFIG_REG_B, 1, &g);
  g &= ~HMC5883_GAIN_NULLMASK;
  switch(g)
  {
    case HMC5883_GAIN_0_9_GA:
      gain = 0.9f:
      break;
    case HMC5883_GAIN_1_2_GA:
      gain = 1.2f:
      break;
    case HMC5883_GAIN_1_9_GA:
      gain = 1.9f;
      break;
    case HMC5883_GAIN_2_5_GA:
      gain = 2.5f;
      break;
    case HMC5883_GAIN_4_0_GA:
      gain = 4.f;
      break;
    case HMC5883_GAIN_4_6_GA:
      gain = 4.6f;
      break;
    case HMC5883_GAIN_5_5_GA:
      gain = 5.5f;
      break;
    case HMC5883_GAIN_7_9_GA:
      gain = 7.9f;
      break;
    default:
      gain = 0.f;     
  }
  return gain;
} // HMC5883::getMeasurementGain


bool blob::HMC5883::setOutputRate (const float& rate)
{
  uint8_t r = 0x00;
  switch (rate)
  {
    case 0.75f:
      r=HMC5883_RATE_0_75_HZ;
      break;
    case 1.5f:
      r=HMC5883_RATE_1_5_HZ;
      break;
    case 3.0f: 
      r=HMC5883_RATE_3_HZ;
      break;
    case 7.5f:
      r=HMC5883_RATE_7_5_HZ;
      break;
    case 15.f:
      r=HMC5883_RATE_15_HZ;
      break;
    case 30.f:
      r=HMC5883_RATE_30_HZ;
      break;
    case 75.f:
      r=HMC5883_RATE_75_HZ;
      break;
    default:
      return false;  
  }

  /* read register and set new rate */
  uint8_t regA = getConfigRegA();
  regA &= HMC5883_RATE_NULLMASK;
  regA |= r;
  _i2c.writeReg(HMC5883_CONFIG_REG_A, regA);
  return true;
}


float blob::HMC5883::getOutputRate ()
{
  float rate = 0.f;
  uint8_t r = 0x00;

  _i2c.readReg(HMC5883_CONFIG_REG_A, 1, &r);
  r &= ~HMC5883_RATE_NULLMASK;

  switch (r)
  {
    case HMC5883_RATE_0_75_HZ:
      rate = 0.75f;
      break;
    case HMC5883_RATE_1_5_HZ:
      rate = 1.5f;
      break;
    case HMC5883_RATE_3_HZ: 
      rate = 3.0f;
      break;
    case HMC5883_RATE_7_5_HZ:
      rate = 7.5f;
      break;
    case HMC5883_RATE_15_HZ:
      rate = 15.f;
      break;
    case HMC5883_RATE_30_HZ:
      rate = 30.f;
      break;
    case HMC5883_RATE_75_HZ:
      rate = 75.f;
      break;
    default:
      rate = 0.f;  
  }

  return rate;
}

bool blob::HMC5883::setSampleAverage (const uint8_t& average)
{
  uint8_t sa = 0x00;
  switch (average)
  {
    case 1:
      sa=HMC5883_SAMPLE_AVERAGING_1;
      break;
    case 2:
      sa=HMC5883_SAMPLE_AVERAGING_2;
      break;
    case 4:
      sa=HMC5883_SAMPLE_AVERAGING_4;
      break;
    case 8:
      sa=HMC5883_SAMPLE_AVERAGING_8;
      break;
    default:
      return false;  
 
  /* read register and set new rate */
  uint8_t regA = getConfigRegA() 
  regA &= HMC5883_SAMPLE_AVERAGING_NULLMASK;
  regA |= sa;
  _i2c.writeReg(HMC5883_CONFIG_REG_A, regA);  
  
  return true;
}

uint8_t blob::HMC5883::getSampleAverage ()
{
  uint8_t average = 0;
  uint8_t sa = 0x00;

  _i2c.readReg(HMC5883_CONFIG_REG_A, 1, &sa);
  sa &= ~HMC5883_SAMPLE_AVERAGING_NULLMASK;

  switch (sa)
  {
    case HMC5883_SAMPLE_AVERAGING_1:
      average=1;
      break;
    case HMC5883_SAMPLE_AVERAGING_2:
      average=2;
      break;
    case HMC5883_SAMPLE_AVERAGING_4:
      average=4;
      break;
    case HMC5883_SAMPLE_AVERAGING_8:
      average=8;
      break;
    default:
      average = 0;  
  }
  return true;
}

bool blob::HMC5883::setMeasurementMode (const uint8_t& mode)
{
  if (mode!=Normal && mode!=PositiveBias && mode!=NegativeBias)
  {
    return false;
  }

  /* read register and set new mode */
  uint8_t regA = getConfigRegA();
  regA &= HMC5883_MEASUREMENT_MODE_NULLMASK;
  regA |= mode;
  _i2c.writeReg(HMC5883_CONFIG_REG_A, regA); 
  return true;
}

uint8_t blob::HMC5883::getMeasurementMode ()
{
  if (mode!=Normal && mode!=PositiveBias && mode!=NegativeBias)
  {
    return false;
  }

  /* read register and set new mode */
  uint8_t regA = getConfigRegA();
  regA &= HMC5883_MEASUREMENT_MODE_NULLMASK;
  regA |= mode;
  _i2c.writeReg(HMC5883_CONFIG_REG_A, regA); 
  return true;
}

bool blob::HMC5883::setMode (const uint8_t mode)
{
  if (mode!=Continuous && Mode!=Single && Mode != Idle && Mode != Sleep)
    return false;

  _i2c.writeReg(HMC5883_MODE_REG, mode);

  return true;
}

uint8_t blob::HMC5883::getMode ()
{
  unit8_t reg;
  _i2c.readReg(HMC5883_MODE_REG, 1, &reg);

  return reg;
}

uint8_t blob::HMC5883::getStatus ()
{    
  uint8_t reg;  
  _i2c.readReg  (HMC5883_STATUS_REG, 1,  &reg);
  return reg;
}

uint8_t blob::HMC5883::getConfigRegA ()
{
  uint8_t reg;
  _i2c.readReg (HMC5883_CONFIG_REG_A, 1, &reg);
  return reg;
}

uint8_t blob::HMC5883::getConfigRegB ()
{
  uint8_t reg;
  _i2c.readReg (HMC5883_CONFIG_REG_B, 1, &reg);
  return reg;
}

void blob::HMC5883::print(bool ln)
{
#if defined(__DEBUG__)
#if defined(__AVR_ATmega32U4__)
  if(Serial)
  {
    Serial.print("blob::Hmc5883 - ");
    Serial.print(_mag.x); Serial.print(" ");
    Serial.print(_mag.y); Serial.print(" ");
    Serial.print(_mag.z); Serial.print(" ");
    Serial.println(_dt);    
    if(ln) {
      Serial.println(" - ");
    } else {
      Serial.print(" - ");
    }
  }
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  std::cout << "blob::Hmc5883 - " 
            << _mag.x << " " 
            << _mag.y << " " 
            << _mag.z << " "
            << _dt << " - ";
  if(ln) std::cout << std::endl;       
#endif // defined(__linux__)
#endif // defined(__DEBUG__)
}
