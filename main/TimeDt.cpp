#include "TimeDt.h"
#include <esp_timer.h>

// TimeDt class implementation
// provides dt betweens two updates 
// 

// update dt 
void TimeDt::update( float val ) {
	writing = true;
	if ( firstpass ) {
		dtmin = val * 0.33;
		dtmax = dt * 3.0;
		dt_1 = 0.0;
		dt_2 = 0.0;
		dt_3 = 0.0;
		dt_4 = 0.0;	
		currentime = esp_timer_get_time();
		previoustime = currentime;
		firstpass = false;
	} else {
		currentime = esp_timer_get_time();
		dt =  currentime - previoustime;
		previoustime = currentime;
		if ( (dt < dtmin) || (dt > dtmax) )  dt = 0.0;
		dt_1 = dt_2;
		dt_2 = dt_3;
		dt_3 = dt_4;
		dt_4 = dt;		
	}
	writing = false;
}

// Get dt
float TimeDt::getdt() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - currentime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}	
	return (float)dt;
}

// Get dt
float TimeDt::getdtds() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - currentime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}	
	return (float)( dt_1 + dt_2 + dt_3 + dt_4 );
}

// Get time
int64_t TimeDt::gettime() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - currentime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}		
	return currentime;
}