#include "Complementary.h"
#include <esp_timer.h>
#include <stdint.h>


// Complementary filter class implementation
//

// initialize with stream sample rate and complementary filter period
void Complementary::CFinit( float SamplingRate, float Period ) {
	SampleRate = SamplingRate;
	FilterPeriod = Period;
	fc1 = SampleRate * Period;
	fc1 = fc1 / ( fc1 + 1 );
	fc2 = 1.0 - fc1;
	CFValue = 0.0;
	writing = false;
	InitDone = false;
}	

// Change complementary filter period
void Complementary::CFsetPeriod( float Period ) {
	fc1 = SampleRate * Period;
	fc1 = fc1 / ( fc1 + 1 );
	fc2 = 1 - fc1;
}

// Get current filter Period
float Complementary::CFgetPeriod() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return FilterPeriod;
}

// Update Complementary filter output using current stream delta time betweenlast samples, derivative value and signal value
void Complementary::CFupdate( float dt, float Derivative, float Signal ) {
	writing = true;
	gettime = esp_timer_get_time();
	if ( InitDone ) {
		CFValue = fc1 * ( CFValue + Derivative * dt ) + fc2 * Signal;
		result.DSupdate( CFValue );
		deltat.DSupdate( dt );
	} else {
		CFValue = Signal;
		result.DSinit( CFValue );
		deltat.DSinit( dt );
		InitDone = true;
	}
	writing = false;
}

// Get Complementary Filter output
float Complementary::CFget() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return CFValue;
}

// Get Complementary down sampled output
float Complementary::CFgetds() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return result.DSaverage();
}

// Get Complementary Filter output
float Complementary::CFgetdtds() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return deltat.DSsum();
}