/********* blob robotics 2014 *********
 *  title: bat.cpp
 *  brief: project for Blob Bat vehicle
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "blob/types.h"
#include "blob/mpu6050.h"
#include "blob/hmc5883.h"
#include "blob/ms5611.h"
#include "blob/loc.h"
#include "blob/comms.h"

// only led: 17: TX LED (RED)
#if defined(__AVR_ATmega32U4__)
const int ledPin =  17;  // the number of the LED pin
int ledState = LOW;      // ledState used to set the LED
#endif // defined(__AVR_ATmega32U4__)

blob::Vector3d<float> g, a, m;
  
blob::MPU6050 imu;
blob::HMC5883 mag;
blob::MS5611  baro;
blob::Loc     loc;
blob::Comms   comms;

uint32_t t = 0;
uint16_t _lastImuIndex = 0, _lastLocIndex = 0;

bool _started = false;

void setup() {
#if defined(__AVR_ATmega32U4__)
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
#endif // defined(__AVR_ATmega32U4__)

  comms.init();
  
#if defined(__DEBUG__) && defined(__AVR_ATmega32U4__)
  if(Serial)
  {
    Serial.println(" ");
    Serial.println("detected AVR_ATmega32U4");
  }
#endif // defined(__DEBUG__) && defined(__AVR_ATmega32U4__)
  
  imu.init(); // initialize mpu6050
  imu.setI2cMasterMode(false); // get directly to mag 
  imu.setI2cBypass(true); // get directly to mag

  mag.init(); // initialize mag  

  imu.coupleMag(mag.getAddress(), mag.getDataReg(), // couple mag to mpu6050  
                mag.getScale(), mag.getGain());
 
  baro.init(); // init baro
    
  loc.init(); // init localization
  loc.couple(&imu); // couple imu
  loc.couple(&baro); // couple baro
  t=millis();
}

void loop()
{
  if (comms.receive() == true)
  {
    switch (comms.getMsgType())
    {
      case blob::Comms::Command:
        if (comms.getCmd (blob::Comms::Start))
        {
          _started = true;
        }
        break;
      default: break;
    }
  }
  if(_started)
  {
    imu.loop(20);
    baro.loop(20);  
    loc.loop(20);
     
    if(_lastLocIndex != loc.getIndex()) { // || _lastLocIndex != loc.getIndex()

      _lastImuIndex = imu.getIndex();
      _lastLocIndex = loc.getIndex();
    
      //comms.send(imu.getEuler(), imu.getAcc(), loc.getVelFlu(), loc.getPos());
      Serial.println((millis()-t),DEC);
      t = millis();
    }
/*  nav.update(comms.receive(),loc);
  
    comms.send(loc,nav);
    comms.send(loc.getEuler(), loc.getAcc(), loc.getVel(),loc.getPos())
*/
  }
 // blob::Task::delayMs(1);    // delay in between reads for stability
}

 
 

