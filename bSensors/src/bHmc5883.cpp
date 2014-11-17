/********* blob robotics 2014 *********
 *  title: bHmc5883.cpp
 *  brief: driver for Hcm5883 magnetometer
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "bHmc5883.h"

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

/* Direcciones registros */
#define HMC5883_CONFIG_REG_A 0x00
#define HMC5883_CONFIG_REG_B 0x01
#define HMC5883_MODE_REG     0x02
#define HMC5883_DATA_REG     0x03
#define HMC5883_STATUS_REG   0x09
#define HMC5883_ID_REG       0x0A

/* Direcciones registros de datos */
#define HMC5883_RA_DATAX_H   HMC5883_DATA_REG
#define HMC5883_RA_DATAY_H   (HMC5883_RA_DATAX_H+2)
#define HMC5883_RA_DATAZ_H   (HMC5883_RA_DATAX_H+4)

#define HMC5883_X_SELF_TEST_GAUSS (+1.16f)    //!< X axis level when bias current is applied.
#define HMC5883_Y_SELF_TEST_GAUSS (+1.16f)    //!< Y axis level when bias current is applied.
#define HMC5883_Z_SELF_TEST_GAUSS (+1.08f)    //!< Z axis level when bias current is applied.
#define HMC5883_SELF_TEST_LOW   (243.0f/390.0f)   //!< Low limit when gain is 5.
#define HMC5883_SELF_TEST_HIGH  (575.0f/390.0f)   //!< High limit when gain is 5.

/** Mode (MD1..MD0) at Mode Register (MR) */
#define HMC5883_MODE_CONTINUOUS  0x00
#define HMC5883_MODE_SINGLE      0x01
#define HMC5883_MODE_IDLE        0x02
#define HMC5883_MODE_SLEEP       0x03

/* Status Register */
#define HMC5883_STATUS_RDY           0x01 
#define HMC5883_STATUS_LOCK          0x02
#define HMC5883_STATUS_REG_ENABLED   0x04

/* Data Output Rate (D02..DO0) at Configuration Register A (CRA) */
#define HMC5883_RATE_0_75_HZ   (0)
#define HMC5883_RATE_1_5_HZ    (1 << 2)
#define HMC5883_RATE_3_HZ      (2 << 2)
#define HMC5883_RATE_7_5_HZ    (3 << 2)
#define HMC5883_RATE_15_HZ     (4 << 2)  /* Default value */
#define HMC5883_RATE_30_HZ     (5 << 2)
#define HMC5883_RATE_75_HZ     (6 << 2)

/* Data valid sample averaging at Configuration Register A (CRA) */
#define HMC5883_SAMPLE_AVERAGING_1   0
#define HMC5883_SAMPLE_AVERAGING_2  (1 << 5)
#define HMC5883_SAMPLE_AVERAGING_4  (2 << 5)
#define HMC5883_SAMPLE_AVERAGING_8  (3 << 5)

/* Measurement Mode, MSO, MS1 at Configuration Register A (CRA) */
#define HMC5883_NORMAL_MEASURE  0 //0x10 // 0x00
#define HMC5883_POSITIVE_BIAS   1 //0x11 // 0x01
#define HMC5883_NEGATIVE_BIAS   2 //0x12 // 0x02

/* Mode (MD1..MD0) at Mode Register (MR) */
#define HMC5883_MODE_CONTINUOUS  0x00
#define HMC5883_MODE_SINGLE      0x01
#define HMC5883_MODE_IDLE        0x02
#define HMC5883_MODE_SLEEP       0x03

/* Gain (DN2..GN0) at Configuration Register B (CRB) */
#define HMC5883_GAIN_0_9_GA   (0)
#define HMC5883_GAIN_1_2_GA   (1 << 5)  /* Default value */
#define HMC5883_GAIN_1_9_GA   (2 << 5)
#define HMC5883_GAIN_2_5_GA   (3 << 5)
#define HMC5883_GAIN_4_0_GA   (4 << 5)
#define HMC5883_GAIN_4_6_GA   (5 << 5)
#define HMC5883_GAIN_5_5_GA   (6 << 5)
#define HMC5883_GAIN_7_9_GA   (7 << 5)

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
  _i2c.init();

  blob::Task::delay(10);

  _ready = calibrate();

  blob::Task::delay(10);
   
  //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
  _i2c.writeReg(HMC5883_CONFIG_REG_A, (HMC5883_SAMPLE_AVERAGING_8 | HMC5883_RATE_75_HZ | HMC5883_NORMAL_MEASURE) ); 
  setMeasurementGain(HMC5883_GAIN_1_9_GA);
  _i2c.writeReg(HMC5883_MODE_REG, HMC5883_MODE_CONTINUOUS); //Mode register             -- 000000 00    continuous Conversion Mode
  
  blob::Task::delay(100);
#if defined(__AVR_ATmega32U4__)
  if(Serial) Serial.println(" done.");
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  std::cout << " done." << std::endl;
#endif //defined(__linux__)
} // HMC5883::init

bool  blob::HMC5883::calibrate()
{
   blob::Vector3d<int32_t> total; // 32 bit totals so they won't overflow.
   bool retval = true;    // Error indicator
#if defined(__AVR_ATmega32U4__)
  if(Serial) Serial.print("calibrating hmc5883... "); 
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  std::cout << "calibrating hmc5883... ";
#endif //defined(__linux__)

  blob::Task::delay(50);  // Wait before start

  _i2c.writeReg(HMC5883_CONFIG_REG_A, 0x010 + HMC5883_POSITIVE_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias

  // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
  // The new gain setting is effective from the second measurement and on.

  _i2c.writeReg(HMC5883_CONFIG_REG_B, 2 << 5);  //Set the Gain
  _i2c.writeReg(HMC5883_MODE_REG, HMC5883_MODE_SINGLE);

  blob::Task::delay(100);

  update();  // Get one sample, and discard it

  for (uint8_t i = 0; i < 10; i++) { // Collect 10 samples
    _i2c.writeReg(HMC5883_MODE_REG, HMC5883_MODE_SINGLE);

    blob::Task::delay(100);

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
  _i2c.writeReg(HMC5883_CONFIG_REG_A, 0x010 + HMC5883_NEGATIVE_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.

  for (uint8_t i=0; i<10; i++) {

    _i2c.writeReg(HMC5883_MODE_REG, HMC5883_MODE_SINGLE);

    blob::Task::delay(100);

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

#ifdef __DEBUG__
  print();
#endif

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

void blob::HMC5883::setMeasurementGain (const uint8_t gain)
{          
   switch(gain)
   {
      case HMC5883_GAIN_0_9_GA:
        _scale = 1280.f;
        break;
      case HMC5883_GAIN_1_2_GA:
        _scale = 1024.f;
        break;
      case HMC5883_GAIN_1_9_GA:
        _scale = 768.f;
        break;
      case HMC5883_GAIN_2_5_GA:
        _scale = 614.f;
        break;
      case HMC5883_GAIN_4_0_GA:
        _scale = 415.f;
        break;
      case HMC5883_GAIN_4_6_GA:
        _scale = 361.f;
        break;
      case HMC5883_GAIN_5_5_GA:
        _scale = 307.f;
        break;
      case HMC5883_GAIN_7_9_GA:
        _scale = 219.f;
        break;
      default:
        _scale = 1.f;
        return;     
   }
   _i2c.writeReg(HMC5883_CONFIG_REG_B, gain);
   
} // HMC5883::configureGain

void blob::HMC5883::setOutputRate (const uint8_t rate)
{
  if (rate == HMC5883_RATE_0_75_HZ || 
      rate == HMC5883_RATE_1_5_HZ  || 
      rate == HMC5883_RATE_3_HZ    || 
      rate == HMC5883_RATE_7_5_HZ  ||
      rate == HMC5883_RATE_15_HZ   || 
      rate == HMC5883_RATE_30_HZ   ||
      rate == HMC5883_RATE_75_HZ )
   {
     /* read register and set new rate */
     uint8_t regA = getConfigRegA() | rate; // FIXME shold be &0xF|mode
     _i2c.writeReg(HMC5883_CONFIG_REG_A, regA);  

   }   
}

void blob::HMC5883::setMeasurementMode (const uint8_t mode)
{
   if (mode == HMC5883_NORMAL_MEASURE || 
       mode == HMC5883_POSITIVE_BIAS  || 
       mode == HMC5883_NEGATIVE_BIAS  )
   {
     /* read register and set new mode */
     uint8_t regA = getConfigRegA() | mode; // FIXME shold be &0xF|mode
     _i2c.writeReg(HMC5883_CONFIG_REG_A, regA); 
   } 
}

void blob::HMC5883::setMode (const uint8_t mode)
{
  if (mode == HMC5883_MODE_CONTINUOUS || 
      mode == HMC5883_MODE_SINGLE     || 
      mode == HMC5883_MODE_IDLE       || 
      mode == HMC5883_MODE_SLEEP      )
  {     
    _i2c.writeReg(HMC5883_MODE_REG, mode);
  }
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
}
