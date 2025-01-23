#include "DownSample.h"
#include <esp_timer.h>

//
// Down Scale class implementation
//
// DS initialization
void DownSample::DSinit( float val ) {
	writing = true;
	gettime = esp_timer_get_time();
	index = 0;
	while ( index < DSratio ) table[ index ] = val;
	sum = val * DSratio;
	average = val;
	index = 0;
	firstpass = false;
	writing = false;
}

// DS update
void DownSample::DSupdate( float val ) {
	writing = true;
	gettime = esp_timer_get_time();
	if ( firstpass ) {
		DSinit( val );
	} else {
		sum = sum + val - table[ index ];
		average = sum / DSratio;
		table[ index ] = val;
		index = (index + 1) % DSratio;
	}
	writing = false;
}

// DS sum get
float DownSample::DSsum( void ) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}		
	return sum;
}

// DS average get
float DownSample::DSaverage( void ) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}		
	return average;
}
