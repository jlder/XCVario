/**
  * Magdwick.h 
  */

#ifndef Magdwick_H_
#define Magdwick_H_

#include <stdint.h>

// Magdwick AHRS implementation, including Euler angles, gravity components and gravity module
//
class Magdwick {
private:
	bool InitDone = false;
	int16_t SkipSamplesCount = 20;
	int16_t AverageSamplesCount = 40;
	int16_t SamplesCount = 0;
	int16_t SumCount = 1;
	float Beta = 0.003;
	float AccelGravModule;
	float Roll = 0.0;
	float Pitch = 0.0;
	float Yaw = 0.0;
	float q0 = 0.0;
	float q1 = 0.0;
	float q2 = 0.0;
	float q3 = 0.0;
	float GravIMUx;
	float GravIMUy;
	float GravIMUz;
	float SumAx = 0.0;
	float SumAy = 0.0;
	float SumAz = 0.0;
	bool writing = false;
	bool writingaccelgravmodule = false;
	int64_t gettime = 0;
	float GRAVITY = 9.807;
public:
	void Init();
	void SetBeta( float val );
	float GetRoll();
	float GetPitch();
	float GetYaw();
	void SetGravity( float val );
	float Gravx();
	float Gravy(); 
	float Gravz();
	float AccelGravityModule(); 
	void Update( float dt, float gx, float gy, float gz, float ax, float ay, float az ); 
};

 #endif /* Magdwick_H_ */
