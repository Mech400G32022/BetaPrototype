#include "LeadComp.h"
#include <math.h>

LeadCompensator::LeadCompensator (float kp_p, float alpha_p, float Ts, float f_break_p){
    kp = kp_p/alpha_p;
    alpha = alpha_p;
    T = Ts;
    f_break = f_break_p;
    filter_out[1] = 0;
    filter_out[0] = 0;
    previous_val[1] = 0;
    previous_val[0] = 0;
 }

float LeadCompensator::update(double new_value){
  float err = new_value - target;
  
  float a_v = a();
  float ret_val = kp*alpha*err + a_v*filter_out[0] + kp * (1 - alpha - a_v) * previous_val[0];
  filter_out[1] = filter_out[0];
  filter_out[0] = ret_val;
  previous_val[1] = previous_val[0];
  previous_val[0] = err;
  return ret_val;
}

float LeadCompensator::a(){
  return exp(-T*f_break);
}

float LeadCompensator::B(){
  return a()*T*(alpha-1)*f_break;
}
