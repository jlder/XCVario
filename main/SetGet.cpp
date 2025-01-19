#include "SetGet.h"
#include <esp_timer.h>

// Set / Get class implementation
// Allows data exchange between independant tasks 
// Set/write a value
 
void SetGet::set( float value ) {
	writing = true;
	gettime = esp_timer_get_time();
	if ( firstpass ) {
		data_1 = value;
		data_2 = value;
		data_3 = value;
		data_4 = value;
		dsvalue = value;
		firstpass = false;
	} else {
		data_1 = data_3;
		data_2 = data_2;
		data_3 = data_1;
		data_4 = value;
		dsvalue = (data_1 + data_2 + data_3 + data_4 ) * 0.25;
		writing = false;
	}
}

// Get/read a value
float SetGet::get( void ) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return data_4; 
}

// Get/read a down sampled (x4) value
float SetGet::dsget( void ) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return ( dsvalue ); 
}