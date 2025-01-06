#include "LowPass.h"
#include <esp_timer.h>

// Low Pass filter class implementation
// LP filter initialization
void LowPassFilter::LPinit( float cutoffperiod, float dt ) {
	alpha = cutoffperiod / (cutoffperiod + dt);
	beta = 1.0 - alpha;
}

// LP filter update
void LowPassFilter::LPupdate( float input ) {
	writing = true;
	gettime = esp_timer_get_time();
	output1 = alpha * output1 + beta * input;
	output2 = alpha * output2 + beta * output1;
	writing = false;
}

// LP filter first order output
float LowPassFilter::LowPass1(void) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}		
	return output1;
}

// LP filter second order outuput
float LowPassFilter::LowPass2(void) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}		
	return output2;
}
