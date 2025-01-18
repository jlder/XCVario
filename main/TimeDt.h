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
	int64_t currentime;
	int64_t previoustime;
	int16_t dt;
	int16_t dt_1;
	int16_t dt_2;
	int16_t dt_3;
	int16_t dt_4;	
	int16_t dtmin, dtmax;
	bool writing = false;
public:
	void update( float value );
    float getdt( void );
	float getdtds( void );
	int64_t gettime( void );
 };
 
 #endif /* TimeDt_H_ */