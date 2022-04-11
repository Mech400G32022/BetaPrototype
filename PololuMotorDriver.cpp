#include "Arduino.h"


#include "PololuMotorDriver.h"
#include <math.h> 

PololuDcMotor::PololuDcMotor(int DIR_pin, int SLP_pin, int PWM_pin_p) {
	DIR = DIR_pin;
	SLP = SLP_pin;
	PWM_pin = PWM_pin_p;
	
	pinMode(DIR, OUTPUT);
	pinMode(SLP, OUTPUT);
	pinMode(PWM_pin, OUTPUT);
	
	setPWMFrequency(DEFAULT_PWM_FREQ);
}

void PololuDcMotor::setPWMFrequency(float freq){
	pwm_res = (int)(log2(150e6 + freq) - log2(freq));
	pwm_max = (int)(pow(2, pwm_res) - 1);
	
	analogWriteFrequency(PWM_pin, freq);
	analogWriteResolution(pwm_res);
}

void PololuDcMotor::setBrake(bool to_brake){
  braking = to_brake;
}

void PololuDcMotor::setPower(float power){
	
	//Out of 100
  if (power < 0){
      power = -power;
      digitalWrite(DIR, HIGH);
  } else {
    digitalWrite(DIR, LOW);
  }
  
  if (power > 100.0){
	  power = 100;
  }
  
  int pwm_val = 0;
  
  if (abs(power) > 0.01) {
	  pwm_val = deadzone + power * (pwm_max - deadzone) / 100;
    digitalWrite(SLP, HIGH);
  } else if (!braking) {
    digitalWrite(SLP, LOW);
  }
  
  
  analogWrite(PWM_pin, pwm_val);
}
