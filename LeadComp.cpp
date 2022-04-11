#include "LeadComp.h"
#include <math.h>

LeadCompensator::LeadCompensator (float kp_p, float alpha_p, float Ts, float f_break){
    kp = kp_p;
    alpha = alpha_p;
    T = Ts;
    w_break = 2.0*M_PI*f_break;
    filter_out[1] = 0;
    filter_out[0] = 0;
    previous_val[1] = 0;
    previous_val[0] = 0;
 }

float LeadCompensator::update(double new_value){
  float a_v = a();
  float ret_val = (B() - a_v + 1.0)*kp*previous_val[0] + (a_v*(a_v - 1.0) - B())*previous_val[1] +
            2.0*a_v*filter_out[0] - a_v*a_v*filter_out[1];
  filter_out[1] = filter_out[0];
  filter_out[0] = ret_val;
  previous_val[1] = previous_val[0];
  previous_val[0] = new_value;
  return ret_val;
}

float LeadCompensator::a(){
  return exp(-T*w_break);
}

float LeadCompensator::B(){
  return a()*T*(alpha-1)*w_break;
}
