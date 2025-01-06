/**
  *
  * filter_data_classes.h 
  * header for alpha/beta filter, low pass filter and data access
  *
  */

#ifndef filter_data_classes_H_
#define filter_data_classes_H_

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

class LowPassFilter {
private:
	float output1;
	float output2;
	float alpha = 0.0; // Filter coefficients
	float beta = 1.0;
	bool writing = false;
	int64_t gettime;
public:
	void LPinit( float cutoffperiod, float dt );
    void LPupdate( float input );
	float LowPass1(void);
	float LowPass2(void);
 };
 
 
class SetGet {
private:
	bool writing = false;
	float data = 0.0;
	int64_t gettime;
public:
	void Set( float value );
    float Get( void );
 };
 
 #endif /* filter_data_classes_H_ */