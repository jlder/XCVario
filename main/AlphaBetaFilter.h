/**
  * AlphaBetaFilter.h 
  * header for alpha/beta filter
  */

#ifndef AlphaBetaFilter_H_
#define AlphaBetaFilter_H_

#include <stdint.h>
#include "DownSample.h"

#define MaxZicket 3 // maximum number of concecuitives zickets to let the filter track the signal. If ZicketCount is higher a step change in signal is suspected

// alpha beta filter class
class AlphaBeta {
private:
	float unfiltered = 0.0;
	float dtAvg = 0.0;
	float dtMax = 0.0;
	float dtMin = 0.0;
	float filt = 0.0;
	float prim = 0.0;
	float filt_predict = 0.0;
	float prim_predict = 0.0;
	float innovation = 0.0;
	float filt_update = 0.0;
	float prim_update = 0.0;
	float acc_update = 0.0;
	float NAB;
	float alpha = 0.0;
	float beta = 0.0;
	float gamma = 0.0;
	float Threshold = 0.0;
	float primMin = 0.0;
	float primMax = 0.0;
	float filtMin = 0.0;
	float filtMax = 0.0;
	bool firstpass = true;
	int ZicketCount = 0;
	bool writing = false;
	int64_t gettime = 0.0;
	void Init( float dt, float val, float valprim, float valacc );
	DownSample filter, deriv, deltat;
public:
	void ABinit( float N, float dtTypical );
	void ABinit( float N, float dtTypical, float _Threshold );
	void ABinit( float N, float dtTypical, float _Threshold, float _filtMin, float _filtMax );
	void ABinit( float N, float dtTypical, float _Threshold, float _filtMin, float _filtMax, float _primMin, float _primMax );
	void ABNupdate( float N );
	int16_t ABNget();
	void ABupdate(float dt, float RawData );
	float ABfilt(void);
	float ABprim(void);
	bool ABstable(void);
	float ABraw(void);
	float ABfiltds();
	float ABprimds();
	float ABdtds();
};

 #endif /* AlphaBetaFilter_H_ */