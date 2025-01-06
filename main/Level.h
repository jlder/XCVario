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
	float fclo1, fclo2;
	float fchi1, fchi2;
	float CurrentLevel;
	bool writing = false;
	int64_t gettime;
public:
	void Init( float SampleRate, float LoPass, float HiPass);
	void Update( float val );
	float GetLevel();
};

 #endif /* Level_H_ */
