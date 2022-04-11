#include <math.h>

class PID {
  public:
  PID (float kp_p, float ki_p, float kd_p);
  float update(long new_value);

  
  float kp = 0;
  float kd = 0;
  float ki = 0;
  long error = 0;
  long previous = 0;
  long target = 0;

  long err_deadzone = 0;
  
};
