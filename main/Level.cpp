#include "Level.h"
#include <esp_timer.h>
#include <math.h>
#include <stdint.h>


// Level class implementation

void Level::Init( float rate, float lowp, float hip) {
	fclo1 = rate / ( rate + lowp );
	fclo2 = 1 - fclo1;
	fchi1 = rate / ( rate + hip );
	fchi2 = 1 - fchi1;	
	writing = false;
	InitDone = true;
}	

float Level::GetLevel() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return CurrentLevel;
}

void Level::Update( float val ) {
	writing = true;
	gettime = esp_timer_get_time();
	if ( InitDone ) {
		if ( CurrentLevel < abs( val ) ) {
			CurrentLevel = fclo1 * CurrentLevel + fclo2 * abs( val );
		} else {
			CurrentLevel = fchi1 * CurrentLevel + fchi2 * abs( val );			
		}
	} else {
			CurrentLevel = val;
	}
	writing = false;
}		
