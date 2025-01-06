/**
  *
  * AlphaBetaFilter.h 
  * header for alpha/beta filter
  *
  */

#ifndef AlphaBetaFilter_H_
#define AlphaBetaFilter_H_

#include <stdint.h>

// alpha beta filter class
class AlphaBeta {
private:
	float dtMax = 0.0;
	float dtMin = 0.0;
	float delta = 0.0;
	float prim = 0.0;
	float filt = 0.0;
	float _delta = 0.0;
	float _prim = 0.0;
	float _filt = 0.0;	
	float alpha = 0.0;
	float beta = 0.0;
	float Threshold = 0.0;
	float primMin = 0.0;
	float primMax = 0.0;
	float filtMin = 0.0;
	float filtMax = 0.0;
	bool firstpass = true;
	int zicket = 0;
	bool writing = false;
	int64_t gettime = 0.0;
public:
	void ABinit( float N, float dtTypical );
	void ABinit( float N, float dtTypical, float _Threshold );
	void ABinit( float N, float dtTypical, float _Threshold, float _filtMin, float _filtMax );
	void ABinit( float N, float dtTypical, float _Threshold, float _filtMin, float _filtMax, float _primMin, float _primMax );
	void ABNupdt( float N );
	void ABupdate(float dt, float RawData );
	float ABfilt(void);
	float ABprim(void);
	bool Stable(void);
};

 #endif /* AlphaBetaFilter_H_ */