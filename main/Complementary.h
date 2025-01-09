/**
  *
  * Complementary.h 
  *
  */

#ifndef Complementary_H_
#define Complementary_H_

#include <stdint.h>

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
public:
	void init( float SamplingRate, float Period );
	void setPeriod( float Period );
	float getPeriod();
	float update( float dt, float Derivative, float Signal );
	float get();
};

 #endif /* Complementary_H_ */
