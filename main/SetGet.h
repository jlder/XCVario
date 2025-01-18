/**
  *
  * SetGet.h 
  * header for set & get data access between independant tasks
  *
  */

#ifndef SetGet_H_
#define SetGet_H_

#include <stdint.h>

class SetGet {
private:
	bool firstpass = true;
	bool writing = false;
	float data_1 = 0.0;
	float data_2 = 0.0;
	float data_3 = 0.0;
	float data_4 = 0.0;
	float dsvalue = 0.0;
	int64_t gettime;
public:
	void set( float value );
    float get( void );
    float dsget( void );	
 };
 
 #endif /* SetGet_H_ */