#include "TimeDt.h"
#include <esp_timer.h>

// TimeDt class implementation
// provides dt betweens two updates 
// 

// update dt 
void TimeDt::DTupdate( float val ) {
	writing = true;
	if ( firstpass ) {
		dtmin = (int16_t)val * 0.33;
		dtmax = (int16_t)val * 3.0;
		deltat.DSinit( val );	
		currentime = esp_timer_get_time();
		previoustime = currentime;
		firstpass = false;
	} else {
		currentime = esp_timer_get_time();
		dt =  currentime - previoustime;
		previoustime = currentime;
		if ( (dt > dtmin) && (dt < dtmax) ) {
			deltat.DSupdate( (float)dt );
		} else {
			dt = 0;
		}
	}
	writing = false;
}

// Get dt
float TimeDt::DTget() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - currentime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}	
	return (float)dt;
}

// Get dt
float TimeDt::DTgetds() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - currentime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}	
	return deltat.DSsum();
}

// Get time
int64_t TimeDt::DTgettime() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - currentime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}		
	return currentime;
}