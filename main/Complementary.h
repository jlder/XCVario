/**
  *
  * Complementary.h 
  *
  */

#ifndef Complementary_H_
#define Complementary_H_

#include <stdint.h>
#include "DownSample.h"

// Complementary filter class
//
class Complementary {
private:
	bool InitDone = false;
	float SampleRate;
	float FilterPeriod;
	float fc1 = 0.0;
	float fc2 = 1.0;
	float CFValue = 0.0;
	bool writing = false;
	int64_t gettime;
	DownSample result, deltat;
public:
	void CFinit( float SamplingRate, float Period );
	void CFsetPeriod( float Period );
	float CFgetPeriod();
	void CFupdate( float dt, float Derivative, float Signal );
	float CFget();
	float CFgetds();
	float CFgetdtds();
};

 #endif /* Complementary_H_ */
