#include "PID.h"
#include <math.h>
#include <Arduino.h>

PID::PID (float kp_p, float ki_p, float kd_p){
    kp = kp_p;
    ki = ki_p;
    kd = kd_p;
 }

float PID::update(long new_value){
  long diff = new_value - previous;
  previous = new_value;

  if (fabs(new_value - target) < 4){
    digitalWrite(13, HIGH);
    //return 0;
  }else {
    error += new_value - target; 
    digitalWrite(13, LOW);
  }
  return kp*(new_value - target) + ki * error + kd * diff; 

}
