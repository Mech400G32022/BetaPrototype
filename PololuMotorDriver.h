
#ifndef __POLOLU_H__
#define __POLOLU_H__

#include "Arduino.h"
#include <math.h> 

#define DEFAULT_PWM_RES 13
#define DEFAULT_PWM_FREQ 18311

class PololuDcMotor {
	public:
		PololuDcMotor(int DIR_pin, int SLP_pin, int PWM_pin_p);
		void setPower(float power);
		//TODO float getCurrent();
		void setPWMFrequency(float freq);
		int deadzone = 0;
    void setBrake(bool toBrake);
	
	private:
		int DIR;
		int PWM_pin;
		int CS;
    int SLP;
		
		int pwm_res = DEFAULT_PWM_RES;
		int pwm_max = 8191;
   bool braking = true;
};



#endif
