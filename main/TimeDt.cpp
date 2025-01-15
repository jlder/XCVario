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
		currentime = esp_timer_get_time();
		previoustime = currentime;
		firstpass = false;
	} else {
		currentime = esp_timer_get_time();
		dt =  currentime - previoustime;
		if ( (dt < dtmin) || (dt > dtmax) )  dt = 0.0;
	}
}

// Get dt
float TimeDt::getdt() {
	return (float)dt;
}

// Get time
int64_t TimeDt::gettime() {
	return currentime;
}