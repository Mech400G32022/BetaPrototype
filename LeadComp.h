#include <math.h>

class LeadCompensator {
  public:
  LeadCompensator (float kp_p, float alpha_p, float Ts, float f_break_p);
  float update(double new_value);

  long target = 0;

  private:  
  double kp = 1.0;
  double alpha = 1.0;
  double T = 1.0;
  double f_break = 0;

  
  float a();
  float B();
  float filter_out[2];
  float previous_val[2];
  
};
