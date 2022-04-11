#ifndef __TIMER_EX__
#define __TIMER_EX__

#include "Arduino.h"
#include <math.h> 

class IntervalExecuter {
	public:
		IntervalExecuter(int millisecond_interval, void (*handler)());
		void setInterval(int new_interval);
		void executeIfTime();
	private:
		void (*fcn)();
		int interval;	
		long last_execution;
};


#endif
