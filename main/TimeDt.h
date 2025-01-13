/**
  *
  * TimeDt.h 
  * header for TimeDt to get dt between 2 calls
  *
  */

#ifndef TimeDt_H_
#define TimeDt_H_

#include <stdint.h>

class TimeDt {
private:
	bool firstpass = true;
	int64_t previoustime;
	int64_t dt;
	int16_t dtmin, dtmax;
public:
	void update( float value );
    float get( void );
 };
 
 #endif /* TimeDt_H_ */