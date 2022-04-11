#include "TimerExecuter.h"

IntervalExecuter::IntervalExecuter(int millisecond_interval, void (*handler)()){
	fcn = handler;
	interval = millisecond_interval;
}

void IntervalExecuter::setInterval(int millisecond_interval){
	interval = millisecond_interval;
}

void IntervalExecuter::executeIfTime(){
	long c_time = millis();
	if (c_time - last_execution >= interval) {
		fcn();
		last_execution = c_time;
	}
}
