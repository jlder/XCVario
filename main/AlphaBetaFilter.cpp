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
	N= round( N );
	if ( N >= 2.0  ) {
		NAB = N;
		alpha =  (2.0 * (2.0 * N - 1.0) / N / (N + 1.0));
		beta = (6.0 / N / (N + 1.0));
	}
}

int16_t AlphaBeta::ABNget() {
	return NAB;
}

void AlphaBeta::Init( float dt, float val, float valprim, float valacc ) {
		filt_update = val;
		prim_update = valprim;
		acc_update = valacc;
		Dt = 0.0;
		filter.DSinit( val );		
		deriv.DSinit( prim );
		deltat.DSinit( dt );
}

// AB filter update		
void AlphaBeta::ABupdate(float dt, float RawData ) {
	#define MaxZicket 3 // maximum number of concecuitives zickets to let the filter track the signal. If zicket is higher a step change in signal is suspected
	// process sample if dt above dtMin and below dtMax (dtMin typicaly average dt -/ 3 and dtMax typicaly 3 x average dt)
	if ( firstpass ) { // initialize filter variables when first called
		// Initialize filter parameters
		Init( dt, RawData, 0.0, 0.0 );
		zicket = 0;
		firstpass = false;
	} else {
		if ( dt > dtMin && dt < dtMax  ) {
			// predict filt and prim from previous state
			Dt = Dt + dt;
			filt_predict = filt_update + Dt * prim_update + 0.5 * Dt * Dt * acc_update;
			prim_predict = prim_update + acc_update * Dt;
			// innovation is the difference between measured value and prediction 
			innovation = RawData - filt_predict;
			// compute filt, prim and acc updates
			filt_update = filt_predict + alpha * innovation;
			prim_update = prim_predict + beta * innovation / Dt;
			acc_update = acc_update  + gamma * innovation / Dt / Dt; 
			if ( zicket <= MaxZicket ) { 
				// if filter stable (below max zicket) test if data within threshold, filt and prim limits				
				if ( ( (abs(innovation) < Threshold ) || (Threshold == 0.0)) &&
					 ( (filt_update > filtMin && filt_update < filtMax) || ( filtMin == 0.0 && filtMax == 0.0 ) ) &&
					 ( (prim_update > primMin && prim_update < primMax) || ( primMin == 0.0 && primMax == 0.0 ) )    ) {
					// new data below threshold
					// filter is stable, Dt = 0 and zicket = 0, update filter outputs filt, prim 
					Dt = 0;
					zicket = 0;
					writing = true;
					gettime = esp_timer_get_time();
					filt = filt_update;
					prim = prim_update;
					writing = false;
				} else {
					// new data beyond threshold, filt and prim limits
					// increase zicket
					zicket++;
					if ( zicket > MaxZicket ) {
						// if zicket is above max zicket, filter is considered unstable and we probably are getting into a step change
						// we don't update filter outputs filt, prim
						// we reset filter to start tracking at current value RawData
						Init( dt, RawData, 0.0, 0.0 );
						// filter unstable, we increase zicket to create hysteresis
						zicket = 2 * MaxZicket;
					}
				}
			} else {
				// filter is unstable, we consider we have been through a step change
				// we don't update filter outputs filt, prim
				//
				if ( abs(innovation) < Threshold || (Threshold == 0.0) ) {
					// if innovation is below threshold, filter is converging toward stability and we reduce the zicket number
					zicket--;
				} else {
					// if innovation is above threshold, filter is still unstable and we increase zicket to create hysteresis
					zicket = 2 * MaxZicket;
				}
				//
				if ( zicket <= MaxZicket ) {
					// if zicket goes below stability criteria, filter is considered stable and we reset filter with new parameters to resume tracking
					// filter is now stable, zicket = 0 and Dt = 0;
					zicket = 0;
					Dt = 0.0;
					// we update filter outputs filt, prim 
					writing = true;
					gettime = esp_timer_get_time();
					filt = filt_update;
					prim = prim_update;
					writing = false;
					// we resintialize down sampled values
					filter.DSinit( filt );		
					deriv.DSinit( prim );
					deltat.DSinit( dt );					
				}						
			}
		}
		// In all cases, update unfiltered output and down scaled flter data
		unfiltered = RawData;
		filter.DSupdate( filt );
		deriv.DSupdate( prim );
		deltat.DSupdate( dt );
	}
}

// AB filter filtered output
float AlphaBeta::ABfilt(void) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return filt_update;
}

// AB filter derivative output
float AlphaBeta::ABprim(void) {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return prim_update;
}

// AB filter stability check
bool AlphaBeta::ABstable(void) {
	bool test = true;
	if ( zicket == 0 ) return test; else return !test;
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
