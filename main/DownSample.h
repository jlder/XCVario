/**
  * DownSample.h 
  */

#ifndef DownSample_H_
#define DownSample_H_

#include <stdint.h>

#define DSratio 4

class DownSample {
private:
	float sum = 0.0;
	float average = 0.0;
	int16_t index = 0;
	float table[10];
	bool firstpass;
	bool writing = false;
	int64_t gettime;
public:
	void DSinit( float val );
    void DSupdate( float val );
    float DSsum( void );
	float DSaverage( void );	
 };
  
 #endif /* DownSample_H_ */