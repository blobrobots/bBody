/********* blob robotics 2014 *********
 *  title: mpu6050.cpp
 *  brief: driver for Mpu6050 imu
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "blob/mpu6050.h"
 
#include "blob/units.h"
#include "blob/physics.h" 

#if defined(__linux__)
  #include <iostream>
#endif // defined(__linux__)

#define MPU6050_RP_KP                 2.5f
#define MPU6050_RP_KI                 0.001f
#define MPU6050_YAW_KP                2.5f
#define MPU6050_YAW_KI                0.001f

#define MPU6050_LPF                   0.9f
#define MPU6050_YERK                  0.01f // m/s3

#define MPU6050_RA_SELF_TEST_X        0x0D
#define MPU6050_RA_SELF_TEST_Y        0x0E
#define MPU6050_RA_SELF_TEST_Z        0x0F
#define MPU6050_RA_SELF_TEST_A        0x10
#define MPU6050_RA_SMPLRT_DIV         0x19
#define MPU6050_RA_CONFIG             0x1A
#define MPU6050_RA_GYRO_CONFIG        0x1B
#define MPU6050_RA_ACC_CONFIG         0x1C
#define MPU6050_RA_MOT_THR            0x1F
#define MPU6050_RA_FIFO_EN            0x23
#define MPU6050_RA_I2C_MST_CTRL       0x24
#define MPU6050_RA_I2C_SLV0_ADDR      0x25
#define MPU6050_RA_I2C_SLV0_REG       0x26
#define MPU6050_RA_I2C_SLV0_CTRL      0x27
#define MPU6050_RA_I2C_SLV1_ADDR      0x28
#define MPU6050_RA_I2C_SLV1_REG       0x29
#define MPU6050_RA_I2C_SLV1_CTRL      0x2A
#define MPU6050_RA_I2C_SLV2_ADDR      0x2B
#define MPU6050_RA_I2C_SLV2_REG       0x2C
#define MPU6050_RA_I2C_SLV2_CTRL      0x2D
#define MPU6050_RA_I2C_SLV3_ADDR      0x2E
#define MPU6050_RA_I2C_SLV3_REG       0x2F
#define MPU6050_RA_I2C_SLV3_CTRL      0x30
#define MPU6050_RA_I2C_SLV4_ADDR      0x31
#define MPU6050_RA_I2C_SLV4_REG       0x32
#define MPU6050_RA_I2C_SLV4_DO        0x33
#define MPU6050_RA_I2C_SLV4_CTRL      0x34
#define MPU6050_RA_I2C_SLV4_DI        0x35
#define MPU6050_RA_I2C_MST_STATUS     0x36
#define MPU6050_RA_INT_PIN_CFG        0x37
#define MPU6050_RA_INT_ENABLE         0x38
#define MPU6050_RA_INT_STATUS         0x3A
#define MPU6050_RA_ACCEL_XOUT_H       0x3B
#define MPU6050_RA_ACCEL_XOUT_L       0x3C
#define MPU6050_RA_ACCEL_YOUT_H       0x3D
#define MPU6050_RA_ACCEL_YOUT_L       0x3E
#define MPU6050_RA_ACCEL_ZOUT_H       0x3F
#define MPU6050_RA_ACCEL_ZOUT_L       0x40
#define MPU6050_RA_TEMP_OUT_H         0x41
#define MPU6050_RA_TEMP_OUT_L         0x42
#define MPU6050_RA_GYRO_XOUT_H        0x43
#define MPU6050_RA_GYRO_XOUT_L        0x44
#define MPU6050_RA_GYRO_YOUT_H        0x45
#define MPU6050_RA_GYRO_YOUT_L        0x46
#define MPU6050_RA_GYRO_ZOUT_H        0x47
#define MPU6050_RA_GYRO_ZOUT_L        0x48
#define MPU6050_RA_EXT_SENS_DATA_00   0x49
#define MPU6050_RA_EXT_SENS_DATA_01   0x4A
#define MPU6050_RA_EXT_SENS_DATA_02   0x4B
#define MPU6050_RA_EXT_SENS_DATA_03   0x4C
#define MPU6050_RA_EXT_SENS_DATA_04   0x4D
#define MPU6050_RA_EXT_SENS_DATA_05   0x4E
#define MPU6050_RA_EXT_SENS_DATA_06   0x4F
#define MPU6050_RA_EXT_SENS_DATA_07   0x50
#define MPU6050_RA_EXT_SENS_DATA_08   0x51
#define MPU6050_RA_EXT_SENS_DATA_09   0x52
#define MPU6050_RA_EXT_SENS_DATA_10   0x53
#define MPU6050_RA_EXT_SENS_DATA_11   0x54
#define MPU6050_RA_EXT_SENS_DATA_12   0x55
#define MPU6050_RA_EXT_SENS_DATA_13   0x56
#define MPU6050_RA_EXT_SENS_DATA_14   0x57
#define MPU6050_RA_EXT_SENS_DATA_15   0x58
#define MPU6050_RA_EXT_SENS_DATA_16   0x59
#define MPU6050_RA_EXT_SENS_DATA_17   0x5A
#define MPU6050_RA_EXT_SENS_DATA_18   0x5B
#define MPU6050_RA_EXT_SENS_DATA_19   0x5C
#define MPU6050_RA_EXT_SENS_DATA_20   0x5D
#define MPU6050_RA_EXT_SENS_DATA_21   0x5E
#define MPU6050_RA_EXT_SENS_DATA_22   0x5F
#define MPU6050_RA_EXT_SENS_DATA_23   0x60
#define MPU6050_RA_I2C_SLV0_DO        0x63
#define MPU6050_RA_I2C_SLV1_DO        0x64
#define MPU6050_RA_I2C_SLV2_DO        0x65
#define MPU6050_RA_I2C_SLV3_DO        0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_RA_SIGNAL_PATH_RESET  0x68
#define MPU6050_RA_MOT_DETECT_CTRL    0x69
#define MPU6050_RA_USER_CTRL          0x6A
#define MPU6050_RA_PWR_MGMT_1         0x6B
#define MPU6050_RA_PWR_MGMT_2         0x6C
#define MPU6050_RA_FIFO_COUNTH        0x72
#define MPU6050_RA_FIFO_COUNTL        0x73
#define MPU6050_RA_FIFO_R_W           0x74
#define MPU6050_RA_WHO_AM_I           0x75

#define MPU6050_DISABLE               0
#define MPU6050_ENABLE                1

#define MPU6050_CLOCK_INTERNAL        0x00
#define MPU6050_CLOCK_PLL_XGYRO       0x01
#define MPU6050_CLOCK_PLL_YGYRO       0x02
#define MPU6050_CLOCK_PLL_ZGYRO       0x03
#define MPU6050_CLOCK_PLL_EXT32K      0x04
#define MPU6050_CLOCK_PLL_EXT19M      0x05
#define MPU6050_CLOCK_KEEP_RESET      0x07

#define MPU6050_GYRO_FS_250           0x00
#define MPU6050_GYRO_FS_500           0x01
#define MPU6050_GYRO_FS_1000          0x02
#define MPU6050_GYRO_FS_2000          0x03

#define MPU6050_ACCEL_FS_2            0x00
#define MPU6050_ACCEL_FS_4            0x01
#define MPU6050_ACCEL_FS_8            0x02
#define MPU6050_ACCEL_FS_16           0x03

#define MPU6050_LPF_256HZ             0
#define MPU6050_LPF_188HZ             1
#define MPU6050_LPF_98HZ              2
#define MPU6050_LPF_42HZ              3
#define MPU6050_LPF_20HZ              4
#define MPU6050_LPF_10HZ              5
#define MPU6050_LPF_5HZ               6

#define MPU6050_IRQ_MOTION            0x40 
#define MPU6050_IRQ_FIFO_OVFLOW       0x10
#define MPU6050_IRQ_I2C_MST_INT       0x08
#define MPU6050_IRQ_DATA_RDY          0x01

#define MPU6050_FIFO_TEMP             0x80
#define MPU6050_FIFO_GYRO             0x70
#define MPU6050_FIFO_ACCEL            0x08
    
blob::MPU6050::MPU6050 (uint8_t address) : _i2c(address)
{
   _ready  = false;
   _magCoupled = false;

   _tempRaw = 0;
   _temp = 0.f;
   
   _magGain = blob::Vector3d<float> (1.0, 1.0, 1.0);
   
   _accScale  = 1.0f;
   _gyroScale = 1.0f;
   _magScale  = 1.0f;

} // MPU6050::MPU6050

void blob::MPU6050::init()
{
#if defined(__DEBUG__)
#if defined(__AVR_ATmega32U4__)
  if(Serial) {
    Serial.print("init mpu6050 at 0x"); 
    Serial.print(getAddress(),HEX);
    Serial.print(" ..."); 
  }
#endif //defined(__AVR_ATmega32U4__)  
#if defined(__linux__)
  std::cout << "init mpu6050 at 0x" << std::hex << getAddress() << std::dec << "..."; 
#endif //defined(__linux__)  
#endif
   _i2c.init();
    
  blob::Task::delayMs(10);
  
   _i2c.writeReg(MPU6050_RA_PWR_MGMT_1, 0x80); //PWR_MGMT_1    -- DEVICE_RESET 1

  blob::Task::delayMs(10);
   
   // SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
   _i2c.writeReg(MPU6050_RA_PWR_MGMT_1, 0x03);  
   
   // EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
   _i2c.writeReg(MPU6050_RA_CONFIG, MPU6050_LPF_256HZ);
   
   // Gyro config: FS_SEL = 3: Full scale set to 2000 deg/s
   _i2c.writeReg(MPU6050_RA_GYRO_CONFIG, 0x18); 
   _gyroScale = 16.384f;
   // Acc config: FS_SEL = 2 (Full Scale = +/-8G)  ; ACCELL_HPF=0
   _i2c.writeReg(MPU6050_RA_ACC_CONFIG, 0x10);
   _accScale = 4096.f;
   //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
   //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

   // _ready = calibrate();
#if defined(__DEBUG__)
#if defined(__AVR_ATmega32U4__)
  if(Serial) Serial.println(" done."); 
#endif // defined(__AVR_ATmega32U4__)  
#if defined(__linux__)
  std::cout << " done." << std::endl; 
#endif //defined(__linux__)  
#endif
} // MPU6050::init

/* Calibrate gyro and acc, assuming on flat surface */
bool blob::MPU6050::calibrate()
{
  uint8_t samples = 100;
  updateAccGyro();

  blob::Task::delayMs(5);

  for (uint8_t i = 0; i < samples; i++) { // Collect 100 samples
    updateAccGyro();
    for (uint8_t j = 0; j < 3; j++) {
      _accOff[j]  += _acc[j]/samples;
      _gyroOff[j] += _gyro[j]/samples;
    }
 }
 _accOff[3] -= blob::gravity;  
   
 return true;
} // MPU6050::calibrate

/* Enable I2C bypass for AUX I2C */
void blob::MPU6050::coupleMag (uint8_t magAddr, uint8_t magDataReg, float magScale, blob::Vector3d<float> magGain)
{
#if defined(__DEBUG__)
#if defined(__AVR_ATmega32U4__)
  if(Serial) Serial.print(F("coupling magnetometer to mpu6050..."));
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  std::cout << "coupling magnetometer to mpu6050...";
#endif // defined(__linux__)
#endif
  // INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
  _i2c.writeReg(MPU6050_RA_INT_PIN_CFG, 0b00000010);
  // DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
  _i2c.writeReg(MPU6050_RA_USER_CTRL, 0b00100000);
  // INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
  _i2c.writeReg(MPU6050_RA_INT_PIN_CFG, 0b00000000);
  // MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
  _i2c.writeReg(MPU6050_RA_I2C_MST_CTRL, 0x0D);
  // I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
  _i2c.writeReg(MPU6050_RA_I2C_SLV0_ADDR, 0x80|magAddr);
  // 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
  _i2c.writeReg(MPU6050_RA_I2C_SLV0_REG, magDataReg); // HCM5883_DATA_REGISTER
  // I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
  _i2c.writeReg(MPU6050_RA_I2C_SLV0_CTRL, 0x86);

  _magScale = magScale;
  _magGain  = magGain; 
  _magCoupled = true;

#if defined(__DEBUG__)
#if defined(__AVR_ATmega32U4__)
  if(Serial) Serial.println(F("done."));
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  std::cout << "done." << std::endl;
#endif // defined(__linux__)
#endif

} // MPU6050::coupleMag

bool blob::MPU6050::isMagCoupled()
{
   return _magCoupled;

} // MPU6050::isMagCoupled

uint8_t blob::MPU6050::getAddress()
{
   return _i2c.getAddress();

} // MPU6050::getAddress

/* Refresh data from accelerometer */
void blob::MPU6050::updateAcc()
{
   uint8_t buffer[6];
   _i2c.readReg (MPU6050_RA_ACCEL_XOUT_H, 6, buffer);

   _accRaw.x = (buffer[0]<<8) | buffer[1];
   _accRaw.y = (buffer[2]<<8) | buffer[3];
   _accRaw.z = (buffer[4]<<8) | buffer[5];

   _acc = ((blob::Vector3d<float>)_accRaw/_accScale)*blob::gravity;    // m/s2

   if(_ready)
     _acc-=_accOff;
} // MPU6050::updateAcc

/* Refresh data from gyroscope */
void blob::MPU6050::updateGyro()
{
   uint8_t buffer[6];
   _i2c.readReg (MPU6050_RA_GYRO_XOUT_H, 6, buffer);

   _gyroRaw.x = (buffer[0]<<8) | buffer[1];
   _gyroRaw.y = (buffer[2]<<8) | buffer[3];
   _gyroRaw.z = (buffer[4]<<8) | buffer[5];

   _gyro = blob::Units::degToRad((blob::Vector3d<float>)_gyroRaw/_gyroScale);  // rad/s
   
   if(_ready)
     _gyro-=_gyroOff;
  
} // MPU6050::updateGyro

/* Refresh data from magnetometer */
void blob::MPU6050::updateMag()
{
  if(_magCoupled) {
    uint8_t buffer[6];
    _i2c.readReg (MPU6050_RA_EXT_SENS_DATA_00, 6, buffer);

    _magRaw.x = (buffer[0]<<8) | buffer[1];
    _magRaw.y = (buffer[4]<<8) | buffer[5];
    _magRaw.z = (buffer[2]<<8) | buffer[3];

    _mag = (blob::Vector3d<float>)_magRaw/_magScale;
      
    _mag&=_magGain;  
  }
} // MPU6050::updateMag

/* Refresh data from acc and gyro */
void blob::MPU6050::updateAccGyro()
{
   uint8_t buffer[14];
   _i2c.readReg (MPU6050_RA_ACCEL_XOUT_H, 14, buffer);

   _accRaw.x  = (buffer[0]<<8) | buffer[1];
   _accRaw.y  = (buffer[2]<<8) | buffer[3];
   _accRaw.z  = (buffer[4]<<8) | buffer[5];
   
   _tempRaw   = (buffer[6]<<8) | buffer[7];
   
   _gyroRaw.x = (buffer[8]<<8) | buffer[9];
   _gyroRaw.y = (buffer[10]<<8) | buffer[11];
   _gyroRaw.z = (buffer[12]<<8) | buffer[13];

   _acc  = ((blob::Vector3d<float>)_accRaw/_accScale)*blob::gravity; // m/s2
   _gyro = blob::Units::degToRad((blob::Vector3d<float>)_gyroRaw/_gyroScale); // rad/s
   _temp    = (float)_tempRaw/340.f + 36.53f; // C 

    if(_ready)
    {
      _acc-=_accOff;
      _gyro-=_gyroOff;
    }  
} // MPU6050::updateAccGyro

/* Refresh data from acc, gyro and mag */
void blob::MPU6050::update()
{
  if(!_magCoupled) {
     updateAccGyro();
  }
  else
  {
    uint8_t buffer[20];
    _i2c.readReg (MPU6050_RA_ACCEL_XOUT_H, 20, buffer);

    _accRaw.x  = (buffer[0]<<8) | buffer[1];
    _accRaw.y  = (buffer[2]<<8) | buffer[3];
    _accRaw.z  = (buffer[4]<<8) | buffer[5];
    
    _tempRaw   = (buffer[6]<<8) | buffer[7];
    
    _gyroRaw.x = (buffer[8]<<8) | buffer[9];
    _gyroRaw.y = (buffer[10]<<8) | buffer[11];
    _gyroRaw.z = (buffer[12]<<8) | buffer[13];
    
    _magRaw.x  = (buffer[14]<<8) | buffer[15];
    _magRaw.y  = (buffer[18]<<8) | buffer[19];
    _magRaw.z  = (buffer[16]<<8) | buffer[17];

    _acc  = ((blob::Vector3d<float>)_accRaw/_accScale)*blob::gravity; // m/s2
    _gyro = blob::Units::degToRad((blob::Vector3d<float>)_gyroRaw/_gyroScale);  // rad/s
    _mag  = (blob::Vector3d<float>)_magRaw/_magScale;    // T
    _temp = (float)_tempRaw/340.f + 36.53f; // C 
  
    if(_ready)
    {
      _acc-=_accOff;
      _gyro-=_gyroOff;
    }
    _mag&=_magGain;      
  
  }
  
  if(_ready)
    updateEuler();
  
  _index++;

#if defined(__DEBUG__)
  print();
#endif

} // MPU6050::update

float blob::MPU6050::getTemp ()
{
  return _temp;
}

void blob::MPU6050::setI2cMasterMode (bool on) 
{
   uint8_t reg;
       
   _i2c.readReg (MPU6050_RA_USER_CTRL, 1, &reg);
   
   if (on == true)
      reg |= 0x20;
   else
      reg &= ~0x20;
   
   _i2c.writeReg (MPU6050_RA_USER_CTRL, reg);
   
} // MPU6050::setI2cMasterMode


void blob::MPU6050::setI2cBypass (bool on) 
{  
   uint8_t reg;  
   _i2c.readReg (MPU6050_RA_INT_PIN_CFG, 1, &reg);
   
   if (on == true)
      reg |= 0x02;
   else
      reg &= ~0x02;
   
   _i2c.writeReg (MPU6050_RA_INT_PIN_CFG, reg);
   
} // MPU6050::setI2cBypass

void blob::MPU6050::updateEuler ()
{
  static Vector3d<float> af(0.f, 0.f, blob::gravity);
  static Vector3d<float> prev_a(0.f, 0.f, blob::gravity);
  static Vector3d<float> mf(0.f, 0.f, 0.f);
  static Vector3d<float> eInt (0.f, 0.f, 0.f);
  Vector3d<float> a, m, e;
  
  /* pre-filter accelerometer readings */
  af.x = prev_a.x + blob::math::constrained(_acc.x - prev_a.x, -MPU6050_YERK, MPU6050_YERK);
  af.y = prev_a.y + blob::math::constrained(_acc.y - prev_a.y, -MPU6050_YERK, MPU6050_YERK);
  af.z = prev_a.z + blob::math::constrained(_acc.z - prev_a.z, -MPU6050_YERK, MPU6050_YERK);
  
  prev_a = af;
  
  /* pre-filter magnetometer readings */
  mf = MPU6050_LPF*mf + (1.f-MPU6050_LPF)*m;
  float norm = mf.length();
  if (norm > 0)
    m = mf/norm;
  else
    m = mf;
    
  /* normalise the measurements */
  norm = af.length();
  if (norm > 0)
    a = af/norm;
  else
    a = af;
  float aw = blob::math::constrained(static_cast<float>(1.f - 2.f*fabs(1.f - norm/blob::gravity)),0.f,1.f);
  
  float roll = atan2(_acc.y, _acc.z);
  float pitch = atan2(_acc.y, sqrt(_acc.y*_acc.y + _acc.z*_acc.z));
  
  e.x = aw*( roll - _euler.x)/_dt;
  e.y = aw*( pitch - _euler.y)/_dt;
  
  eInt.x = eInt.x + e.x*_dt;
  eInt.y = eInt.y + e.y*_dt;
  
  _euler.x += _gyro.x + e.x*MPU6050_RP_KP + eInt.x*MPU6050_RP_KI;
  _euler.y += _gyro.y + e.y*MPU6050_RP_KP + eInt.y*MPU6050_RP_KI;
  
  float yaw = atan2((-mf.y*cos(_euler.x) + mf.z*sin(_euler.x) ) , (mf.x*cos(_euler.y) + mf.y*sin(_euler.y)*sin(_euler.x)+ mf.z*sin(_euler.y)*cos(_euler.x)));
  e.z = (yaw - _euler.z)/_dt;
  eInt.z = eInt.z + e.z*_dt;
  _euler.z += _gyro.z + e.z*MPU6050_YAW_KP + eInt.y*MPU6050_YAW_KI; 
  
} // MPU6050::updateEuler

/*
void blob::MPU6050::updateQuaternion ()
{
  static float q[4] = {1.f, 0.f, 0.f, 0.f};
  
  static Vector3d<float> af(0.f, 0.f, blob::gravity);
  static Vector3d<float> prev_a(0.f, 0.f, blob::gravity);
  static Vector3d<float> mf(0.f, 0.f, 0.f);
  static Vector3d<float> eInt (0.f, 0.f, 0.f);
  
  Vector3d<float> a, g, m, e;
  
  float   bx = 0.f, bz = 0.f, hx = 0.f, hy = 0.f, hz = 0.f,
          vx = 0.f, vy = 0.f, vz = 0.f, wx = 0.f, wy = 0.f, wz = 0.f,
          iq0 = 0.f, iq1 = 0.f, iq2 = 0.f, iq3 = 0.f, 
          norm = 1.f, anorm = 1.f, aw = 0.f;
  
  float q0q0 = q[0]*q[0];
  float q0q1 = q[0]*q[1];
  float q0q2 = q[0]*q[2];
  float q0q3 = q[0]*q[3];
  float q1q1 = q[1]*q[1];
  float q1q2 = q[1]*q[2];
  float q1q3 = q[1]*q[3];
  float q2q2 = q[2]*q[2];
  float q2q3 = q[2]*q[3];
  float q3q3 = q[3]*q[3];
    
  // pre-filter accelerometer readings
  af.x = prev_a.x + blob::math::constrained(_acc.x - prev_a.x, -MPU6050_YERK, MPU6050_YERK);
  af.y = prev_a.y + blob::math::constrained(_acc.y - prev_a.y, -MPU6050_YERK, MPU6050_YERK);
  af.z = prev_a.z + blob::math::constrained(_acc.z - prev_a.z, -MPU6050_YERK, MPU6050_YERK);
  
  prev_a = af;
  
  // normalise the measurements
  anorm = af.length();
  if (anorm > 0)
    a = af/anorm;
  else
    a = af;
  
  // pre-filter magnetometer readings
  mf = MPU6050_LPF*mf + (1.f-MPU6050_LPF)*m;
  norm = mf.length();
  if (norm > 0)
    m = mf/norm;
  else
    m = mf;
  
  prev_m = mf;
  
  // compute reference direction of flux
  hx = 2.f*m.x*(0.5f - q2q2 - q3q3) + 2.f*m.y*(q1q2 - q0q3) + 2.f*m.z*(q1q3 + q0q2);
  hy = 2.f*m.x*(q1q2 + q0q3) + 2.f*m.y*(0.5f - q1q1 - q3q3) + 2.f*m.z*(q2q3 - q0q1);
  hz = 2.f*m.x*(q1q3 - q0q2) + 2.f*m.y*(q2q3 + q0q1) + 2.f*m.z*(0.5f - q1q1 - q2q2);        
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;
  
  // estimated direction of gravity and flux (v and w)
  vx = 2.f*(q1q3 - q0q2);
  vy = 2.f*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.f*bx*(0.5f - q2q2 - q3q3) + 2.f*bz*(q1q3 - q0q2);
  wy = 2.f*bx*(q1q2 - q0q3) + 2.f*bz*(q0q1 + q2q3);
  wz = 2.f*bx*(q0q2 + q1q3) + 2.f*bz*(0.5f - q1q1 - q2q2); 
	
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  aw = blob::math::constrained(1.f - 2.f*fabsf(1.f - anorm/blob::gravity),0.f,1.f);
  e.x = aw*(a.y*vz - a.z*vy); // + (m.y*wz - m.z*wy);
  e.y = aw*(a.z*vx - a.x*vz); // + (m.z*wx - m.x*wz);
  e.z = aw*(a.x*vy - a.y*vx) + (m.x*wy - m.y*wx); 

  // integral error (bias)
  eInt = eInt + e*_dt;
  
  // adjusted gyroscope measurements
  g = _gyro + e*MPU6050_RP_KP + eInt*MPU6050_RP_KI;
  
  // integrate quaternion rate and normalise
  q[0] += (-q[1]*g.x - q[2]*g.y - q[3]*g.z)*_dt;
  q[1] += (q[0]*g.x + q[2]*g.z - q[3]*g.y)*_dt;
  q[2] += (q[0]*g.y - q[1]*g.z + q[3]*g.x)*_dt;
  q[3] += (q[0]*g.z + q[1]*g.y - q[2]*g.x)*_dt; 
  
  // normalise quaternion
  norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if(norm > 0)
  {
    q[0] = q[0]/norm;
    q[1] = q[1]/norm;
    q[2] = q[2]/norm;
    q[3] = q[3]/norm;
  }
 
  _euler.x  = atan2(2.f*(q[0]*q[1] + q[2]*q[3]), 1.f - 2.f*(q[1]*q[1] + q[2]*q[2]));
  _euler.y = asin (2.f*(q[0]*q[2] - q[1]*q[3]));
  _euler.z   = atan2(2.f*(q[0]*q[3] + q[1]*q[2]), 1.f - 2.f*(q[2]*q[2] + q[3]*q[3]));
}
*/
