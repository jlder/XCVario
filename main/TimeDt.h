/**
  *
  * TimeDt.h 
  * header for TimeDt to get dt between 2 calls
  *
  */

#ifndef TimeDt_H_
#define TimeDt_H_

#include <stdint.h>
#include "DownSample.h"

class TimeDt {
private:
	bool firstpass = true;
	int64_t currentime;
	int64_t previoustime;
	int16_t dt;
	int16_t dtmin, dtmax;
	bool writing = false;
	DownSample deltat;	
public:
	void DTupdate( float value );
    float DTget( void );
	float DTgetds( void );
	int64_t DTgettime( void );
 };
 
 #endif /* TimeDt_H_ */