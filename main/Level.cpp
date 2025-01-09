#include "Level.h"
#include <esp_timer.h>
#include <math.h>
#include <stdint.h>


// Level class implementation
// estimate the level of a stream of data using absolute value of the data filtered with low pass filters of different period when data increases or decreases

// initialize with stream sample rate, low pass frequency for signal increase, low pass frequency for signal decrease
void Level::init( float SampleRate, float LoPassInc, float LoPassDec ) {
	fcinc1 = SampleRate / ( SampleRate + LoPassInc );
	fcinc2 = 1 - fcinc1;
	fcdec1 = SampleRate / ( SampleRate + LoPassDec ); 
	fcdec2 = 1 - fcdec1;
	CurrentLevel = 0.0;
	writing = false;
	InitDone = true;
}	

// Get current filtered signal level
float Level::get() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return CurrentLevel;
}

// Update signal level using low pass filters
void Level::update( float val ) {
	writing = true;
	gettime = esp_timer_get_time();
	if ( InitDone ) {
		if ( CurrentLevel < abs( val ) ) {
			CurrentLevel = fcinc1 * CurrentLevel + fcinc2 * abs( val );
		} else {
			CurrentLevel = fcdec1 * CurrentLevel + fcdec2 * abs( val );			
		}
	} else {
		CurrentLevel = val;
	}
	writing = false;
}		
