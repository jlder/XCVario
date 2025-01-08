#include "LowPass.h"
#include <esp_timer.h>

//
// Low Pass filter class implementation
// LP filter initialization
void LowPassFilter::LPinit( float cutoffperiod, float dt ) {
	LowPassFilter::LPinit( cutoffperiod, dt, 2 );
}

void LowPassFilter::LPinit( float cutoffperiod, float dt, int16_t order ) {
	alpha = cutoffperiod / (cutoffperiod + dt);
	beta = 1.0 - alpha;
	FilterOrder = order;
}

// LP filter update
void LowPassFilter::LPupdate( float input ) {
	writing = true;
	gettime = esp_timer_get_time();
	output1 = alpha * output1 + beta * input;
	output2 = alpha * output2 + beta * output1;
	writing = false;
}

// LP filter update
float LowPassFilter::LowPassUpdate( float input ) {
	writing = true;
	gettime = esp_timer_get_time();
	output1 = alpha * output1 + beta * input;
	if (FilterOrder < 2 ) {
		writing = false;
		return output1;
	} else {
		output2 = alpha * output2 + beta * output1;
		writing = false;
		return output2;
	}
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
