#include "TimeDt.h"
#include <esp_timer.h>

// TimeDt class implementation
// provides dt betweens two updates 
// 

// update dt 
void TimeDt::update( float val ) {
	if ( firstpass ) {
		dtmin = val / 3.0;
		dtmax = dt * 3.0;
		dt = 0.0;
		previoustime = esp_timer_get_time();
		firstpass = false;
	} else {
		dt = esp_timer_get_time() - previoustime;
		if ( (dt < dtmin) || (dt > dtmax) )  dt = 0.0;
	}
}

// Get dt
float TimeDt::get() {
	return dt;
}
