#include "AlphaBetaFilter.h"
#include <esp_timer.h>

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
	if ( N != 0.0  ) {
		alpha =  (2.0 * (2.0 * N - 1.0) / N / (N + 1.0));
		beta = (6.0 / N / (N + 1.0));
		dtMax = dtTypical * 4.0;
		dtMin = dtTypical / 4.0;
	}
	firstpass = true;
	Threshold = _Threshold;
	filtMin = _filtMin;
	filtMax = _filtMax;
	primMin = _primMin;
	primMax = _primMax;
}

// AB filter depth N update
void AlphaBeta::ABNupdt( float N ) {
	if ( N > 0.0  ) {
		alpha =  (2.0 * (2.0 * N - 1.0) / N / (N + 1.0));
		beta = (6.0 / N / (N + 1.0));
	}
}

// AB filter update		
void AlphaBeta::ABupdate(float dt, float RawData ) {
	#define MaxZicket 2 // maximum number of concecuitives zickets to let the filter track the signal. If zicket is higher a step change in signal is suspected
	// process sample if dt above dtMin and below dtMax (dtMin typicaly average dt / 4 and dtMax typicaly 4 x average dt)
	writing = false;
	if ( dt > dtMin && dt < dtMax  ) {
		if ( firstpass ) { // initialize filter variables when first called
			writing = true;
			gettime = esp_timer_get_time();
			filt = RawData;
			_filt = RawData;
			prim = 0.0;
			_prim = 0.0;
			writing = false;
			firstpass = false;
			zicket = 4*MaxZicket;
		} else {
			if ( zicket <= MaxZicket ) { 
				// if filter stable
				delta = RawData - filt;
				if ( (abs(delta) < Threshold ) || (Threshold == 0.0) ) {
					// new data below threshold
					writing = true;
					gettime = esp_timer_get_time();
					prim = prim + beta * delta / dt;
					if ( primMin != 0.0 || primMax != 0.0 ) {
						if ( prim < primMin ) prim = primMin;
						if ( prim > primMax ) prim = primMax;
					}
					filt = filt + alpha * delta + prim * dt;
					if ( filtMin != 0.0 || filtMax != 0.0 ) {
						if ( filt < filtMin ) filt = filtMin;
						if ( filt > filtMax ) filt = filtMax;
					}
					writing = false;
					zicket = 0;
				} else {
					// new data above threshold, additional zicket
					zicket++;
					if ( zicket > MaxZicket ) {
						// if new zicket makes filter unstable (step change), arm and switch to alternate AB filter
						_prim = prim;
						_filt = filt;
						zicket = 4*MaxZicket;
					}
				}
			} else {
				// if filter unstable - step change
				// update alternate filter to track step change
				_delta = RawData - _filt;
				_prim = _prim + beta * _delta / dt;
				_filt = _filt + alpha * _delta + _prim * dt;
				// if new data below threshold, reduce number of zicket
				if ( abs(_delta) < Threshold || (Threshold == 0.0) ) zicket--; else zicket = 4*MaxZicket;
				if ( zicket <= MaxZicket ) {
					// if number of zicket below stability criteria, arm and switch to primary filter
					writing = true;
					gettime = esp_timer_get_time();
					prim = _prim;
					filt = _filt;
					writing = false;
					zicket = 0;
				}						
			}
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
bool AlphaBeta::Stable(void) {
	bool test = true;
	if ( zicket == 0 ) return test; else return !test;
}
