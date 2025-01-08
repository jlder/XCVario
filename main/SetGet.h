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
	bool writing = false;
	float data = 0.0;
	int64_t gettime;
public:
	void Set( float value );
    float Get( void );
 };
 
 #endif /* SetGet_H_ */