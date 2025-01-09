#include "SetGet.h"
#include <esp_timer.h>

// Set / Get class implementation
// Allows data exchange between independant tasks 
// Set/write a value
 
void SetGet::set( float value ) {
	writing = true;
	gettime = esp_timer_get_time();		
	data = value;
	writing = false;
}
// Get/read a value
float SetGet::get( void ) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return data; 
}