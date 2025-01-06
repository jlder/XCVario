/**
  *
  * LowPass.h 
  * header for low pass filter
  *
  */

#ifndef LowPass_H_
#define LowPass_H_

#include <stdint.h>

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
  
 #endif /* LowPass_H_ */