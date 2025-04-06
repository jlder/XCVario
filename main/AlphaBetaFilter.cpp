
#include "AlphaBetaFilter.h"
#include <esp_timer.h>
#include <cmath>

//
// alpha beta filter class implementation
// 
// AB filter initialization
void AlphaBeta::ABinit( float N, float dtTypical ) {
	ABinit( N, dtTypical, 0.0, 0.0, 0.0, 0.0, 0.0 );
}
void AlphaBeta::ABinit( float N, float dtTypical, float _Threshold ) {
	ABinit( N, dtTypical, _Threshold, 0.0, 0.0, 0.0, 0.0 );
}
void AlphaBeta::ABinit( float N, float dtTypical, float _Threshold, float _filtMin, float _filtMax ) {
	ABinit( N, dtTypical, _Threshold, _filtMin, _filtMax, 0.0, 0.0 );
}
void AlphaBeta::ABinit( float N, float dtTypical, float _Threshold, float _filtMin, float _filtMax, float _primMin, float _primMax ) {
	ABNupdate( N );
	dtAvg = dtTypical;
	dtMax = dtTypical * 3.0;
	dtMin = dtTypical * 0.33;
	firstpass = true;
	Threshold = _Threshold;
	filtMin = _filtMin;
	filtMax = _filtMax;
	primMin = _primMin;
	primMax = _primMax;
}

// AB filter depth N update
void AlphaBeta::ABNupdate( float N ) {
	#define Kd 1.3
	#define Ka 0.25
	N= round( N );
	if ( N >= 2.0  ) {
		NAB = N;
		alpha =  2.0 * (2.0 * N - 1.0) / N / (N + 1.0);
		beta = Kd * 6.0 / N / (N + 1.0);
		gamma = Ka * 12.0 / N / (N + 1.0) / (N + 2.0);
	}
}

int16_t AlphaBeta::ABNget() {
	return NAB;
}

void AlphaBeta::Init( float dt, float val, float valprim, float valacc ) {
		filt_update = val;
		prim_update = valprim;
		acc_update = valacc;
		filter.DSinit( val );		
		deriv.DSinit( valprim );
		deltat.DSinit( dt );
		ZicketCount = 2 * MaxZicket;		
}

// AB filter update		
void AlphaBeta::ABupdate(float dt, float RawData ) {
	// process sample if dt above dtMin and below dtMax (dtMin typicaly average dt -/ 3 and dtMax typicaly 3 x average dt)
	if ( firstpass ) { // initialize filter variables when first called
		// Initialize filter parameters
		Init( dt, RawData, 0.0, 0.0 );
		Dt = 0;
		firstpass = false;
	} else {
		// if dt is within acceptable limits
		if ( dt > dtMin && dt < dtMax  ) {
			// predict filt and prim from previous state
			Dt = Dt + dt;
			filt_predict = filt_update + dt * prim_update + 0.5 * dt * dt * acc_update;
			prim_predict = prim_update + acc_update * dt;
			// innovation is the difference between measured value and prediction 
			innovation = RawData - filt_predict;
			// compute filt, prim and acc updates using innovation
			filt_update = filt_predict + alpha * innovation;
			prim_update = prim_predict + beta * innovation / dt;
			acc_update = acc_update  + gamma * innovation / dt / dt;
			// 
			if ( ZicketCount <= MaxZicket ) { 
				// if filter stable (ZicketCount below max zicket) test if data within threshold, filt and prim limits				
				if ( ( (fabs(innovation) < Threshold ) || (Threshold == 0.0)) &&
					 ( (filt_update > filtMin && filt_update < filtMax) || ( filtMin == 0.0 && filtMax == 0.0 ) ) &&
					 ( (prim_update > primMin && prim_update < primMax) || ( primMin == 0.0 && primMax == 0.0 ) )    ) {
					// new data below threshold
					// filter is stable, Dt = 0 and ZicketCount = 0, update filter outputs filt, prim using filt/prim_update with innovation
					Dt = 0;
					ZicketCount = 0;
					writing = true;
					gettime = esp_timer_get_time();
					filt = filt_update;
					prim = prim_update;
					writing = false;
				} else {
					// new data beyond threshold, filt and prim limits
					// increase ZicketCount
					ZicketCount++;
					// filter is stable but we have a zicket, update filt using last valid filt and prim. Dt is the sum of dt since begining of zicket
					filt = filt + Dt * prim;
					if ( ZicketCount > MaxZicket ) {
						// if ZicketCount is above max zicket, filter is considered unstable and we probably are getting into a step change
						// we reset filter to start tracking at current value RawData
						Init( dt, RawData, 0.0, 0.0 );
						// filter unstable, we increase ZicketCount to create hysteresis
						ZicketCount = 2 * MaxZicket;
					}
				}
			} else {
				// filter is unstable, we consider we are through a step change
				if ( fabs(innovation) < Threshold || (Threshold == 0.0) ) {
					// if innovation is below threshold, filter is converging toward stability and we reduce the zicket number
					ZicketCount--;
				} else {
					// if innovation is above threshold, filter is still unstable and we increase ZicketCount to create hysteresis
					ZicketCount = 2 * MaxZicket;
				}
				//
				if ( ZicketCount <= MaxZicket ) {
					// if ZicketCount goes below stability criteria, filter is considered stable again and we reset filter with new parameters to resume tracking
					// filter is now stable, ZicketCount = 0 and Dt = 0;
					ZicketCount = 0;
					Dt = 0.0;
					// we update filter outputs filt, prim using latest filt/prim_update with innovation.
					writing = true;
					gettime = esp_timer_get_time();
					filt = filt_update;
					prim = prim_update;
					writing = false;
					// we resintialize down sampled values
					filter.DSinit( filt );		
					deriv.DSinit( prim );
					deltat.DSinit( dt );					
				} else {
					// if filter still not stable, update filt using last valid filt and prim
					filt = filt + Dt * prim;
				}
			}
			// In all cases, update unfiltered output and down scaled flter data
			unfiltered = RawData;
			filter.DSupdate( filt );
			deriv.DSupdate( prim );
			deltat.DSupdate( dt );
		}
	}
}

// AB filter filtered output
float AlphaBeta::ABfilt(void) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return filt;
}

// AB filter derivative output
float AlphaBeta::ABprim(void) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return prim;
}

// AB filter stability check
bool AlphaBeta::ABstable(void) {
	bool test = true;
	if ( ZicketCount == 0 ) return test; else return !test;
}

// AB filter unfiltered output
float AlphaBeta::ABraw(void) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return unfiltered;
}

// AB filter down scale x DSratio
float AlphaBeta::ABfiltds( void ) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}	
	return ( filter.DSaverage() );
}

// AB derivative (prim) down scale x DSratio
float AlphaBeta::ABprimds( void ) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}	
	return ( deriv.DSaverage() );
}

// AB dt down scale x DSratio
float AlphaBeta::ABdtds( void ) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}	
	return ( deltat.DSsum() );
}
