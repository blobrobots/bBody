/********* blob robotics 2014 *********
 *  title: test_arduino_pwm_out.cpp
 *  brief: test for comms library (arduino)
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/

#include "blob/types.h"
#include "blob/pwm.h"
#include "blob/math.h"

#define NUM_PIN B_PWM_MAX_NUM

blob::PWMin in[NUM_PIN];
blob::PWMout out[NUM_PIN];
int mode=0;

float pwm2cmd (uint16_t us, uint16_t minUs, uint16_t maxUs, uint16_t deadUs,
                                                       float min, float max);
void setup()
{  
  Serial.begin(115200);
  while (!Serial);

  Serial.println("blobtest: please indicate 1 for \"in\", 2 for \"out\" or 3 "
                   "to connect \"out\" with \"in\"");  
  while(mode==0) {  
      
    if (Serial && Serial.available()) {
      mode = atoi(Serial.readString().c_str());
      if(mode<1||mode>3) {
        mode = 0;
        Serial.println("input error");
        Serial.println("blobtest: please indicate 1 for \"in\", 2 for \"out\" or 3 "
                   "to connect \"out\" with \"in\"");  
      }
    }
  }
  
  if(mode==1) {
  
    for(int i=0; i<NUM_PIN; i++)
      in[i].init();

    Serial.print("blobtest to read ");  
    Serial.print(NUM_PIN);  
    Serial.println(" pwm signals in us: ");
  } else if (mode == 2) {

    for(int i=0; i<NUM_PIN; i++)
      out[i].init();

    Serial.println("blobtest to generate pwm signals");
    Serial.print("please type dutycycle for ");  
    Serial.print(NUM_PIN);  
    Serial.print(" pwm signals in us: ");
  } else if (mode == 3) {

    for(int i=0; i<NUM_PIN; i++) {
      in[i].init();
      out[i].init();
    }

    Serial.print("blobtest to connect ");  
    Serial.print(NUM_PIN);  
    Serial.println(" pwm signals in us: ");

    out[0].setDutyCycle(1000);
    out[1].setDutyCycle(1330);
    out[2].setDutyCycle(1660);
    out[3].setDutyCycle(2000);
  }
}

void loop()
{
  if(mode==1 || mode == 3) {

    blob::PWMin::sync();

    Serial.print("period ");
    for(int i=0; i<NUM_PIN; i++) {
      Serial.print(in[i].getPeriod());
      Serial.print(" ");
    }

    Serial.print("dutycycle ");
    for(int i=0; i<NUM_PIN; i++) {
      Serial.print(in[i].getDutyCycle());
      Serial.print(" ");
    }

    Serial.print("cmd ");
    for(int i=0; i<NUM_PIN; i++) {
      Serial.print(pwm2cmd(in[i].getDutyCycle(),1040,1900,200,-1.f,1.f));
      //Serial.print(in[i].getDutyCycle());
      Serial.print(" ");
    }

    Serial.print("\r");

  } else if (mode == 2) {

    uint16_t dutycycle = 0;
    if (Serial && Serial.available())
    {
      dutycycle = atoi(Serial.readString().c_str());
      Serial.print(dutycycle);
      Serial.println(" us");
      
      bool res = true;
      int i = 0;
      for(i=0; i<NUM_PIN && res==true; i++)
        res &= out[i].setDutyCycle(dutycycle);
      
      if(res==false)
      {
        Serial.print("error unable to set new dutycycle ");
        Serial.print(dutycycle); Serial.print(" for pwm_"); Serial.println(i);  
      }

      Serial.print("please type new dutycycle for ");
      Serial.print(NUM_PIN);
      Serial.print(" pwm signals in us: ");
    }
  }
}

/**
 * Transforms RC us into normalized input
 */
float pwm2cmd (uint16_t us, uint16_t minUs, uint16_t maxUs, uint16_t deadUs,
                                                       float min, float max) {       
  float output = 0.f;

  if (us > (minUs-deadUs/2) && us < (maxUs+deadUs/2)) {

    float midUs   = (float)(maxUs + minUs)*0.5f;
    float rangeUs = (float)(blob::math::rabs(maxUs - minUs) - deadUs);
         
    float mid   = (max + min)*0.5f;
    float range = (max - min);


    if (us > (midUs + deadUs/2))
      output = (mid + range*((float)us - (midUs + (float)deadUs/2))/rangeUs);     
    else if (us < (midUs - deadUs/2))
      output = (mid - range*((midUs - (float)deadUs/2) - (float)us)/rangeUs);
    else
      output = mid;
	}
  return blob::math::constrained(output, min, max);
}
