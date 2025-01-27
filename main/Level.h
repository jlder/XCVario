/**
  *
  * Level.h 
  *
  */

#ifndef Level_H_
#define Level_H_

#include <stdint.h>

// Level class
class Level {
private:
	bool InitDone = false;
	float fcinc1, fcinc2;
	float fcdec1, fcdec2;
	float CurrentLevel = 0.0;
	bool writing = false;
	int64_t gettime;
public:
	void LVinit( float SampleRate, float LoPassInc, float LoPassDec );
	void LVupdate( float val );
	float LVget();
};

 #endif /* Level_H_ */
