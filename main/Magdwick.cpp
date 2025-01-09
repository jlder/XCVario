#include "Magdwick.h"
#include <esp_timer.h>
#include <math.h>
#include <stdint.h>


// Magdwick AHRS class implementation

// force Initialisation of quaternion with attitude from accels
void Magdwick::init() {
	InitDone = false;
	SamplesCount = 0;
	SumCount = 1;
	SumAx = 0.0;
	SumAy = 0.0;
	SumAz = 0.0;
}	

// set Beta for Magdwick, parameters which allows attitude from gyros to converge toward attitude from acceleration
void Magdwick::setBeta( float val ) {
	Beta = val;
}

// Get Roll from quaternion
float Magdwick::getRoll() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return Roll;
}

// Get Pitch from quaternion
float Magdwick::getPitch() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return Pitch;
}

// Get Yaw from quaternion
float Magdwick::getYaw() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return Yaw;
}

// Set local gravity value, which is used to compute gravity components in function of quaternion attitude
void Magdwick::setGravity( float val ) {
	GRAVITY = val;
}

// Get gravity x component from attitude
float Magdwick::Gravx() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return GravIMUx;
}

// Get gravity y component from attitude
float Magdwick::Gravy() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return GravIMUy;
}

// Get gravity z component from attitude
float Magdwick::Gravz() {
	while( writing ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return GravIMUz;
}

// Get module of gravity from accelerometers
float Magdwick::AccelGravityModule() {
	while( writingaccelgravmodule ) {
		if ( abs( (int64_t)esp_timer_get_time() - gettime ) > 1000 ) break; // wait for 1 ms max if writing is in process
	}
	return AccelGravModule;
}

// Update AHRS. Update quaternion, compute Euler angles and Gravity components from attitude.
// When Update is called for the first time, quarternion is initialized using averaged accelerometers
//
void Magdwick::update(	float dt, float gx, float gy, float gz, float ax, float ay, float az ) {
	float  recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float AverageAx, AverageAy, AverageAz;
	if ( InitDone ) {
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		writingaccelgravmodule = true;
		gettime = esp_timer_get_time();		
		AccelGravModule = sqrt(ax * ax + ay * ay + az * az);
		writingaccelgravmodule = false;
		if ( AccelGravModule != 0.0 ) {
			recipNorm = 1.0 / AccelGravModule;
			// Normalise accelerometer measurement
			ax = ax * recipNorm;
			ay = ay * recipNorm;
			az = az * recipNorm;
			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0 * q0;
			_2q1 = 2.0 * q1;
			_2q2 = 2.0 * q2;
			_2q3 = 2.0 * q3;
			_4q0 = 4.0 * q0;
			_4q1 = 4.0 * q1;
			_4q2 = 4.0 * q2;
			_8q1 = 8.0 * q1;
			_8q2 = 8.0 * q2;
			q0q0 = q0 * q0;
			q1q1 = q1 * q1;
			q2q2 = q2 * q2;
			q3q3 = q3 * q3;
			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
			if ( (s0 + s1 + s2 + s3) != 0 ) {
				// normalise step magnitude
				recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
				s0 = s0 * recipNorm;
				s1 = s1 * recipNorm;
				s2 = s2 * recipNorm;
				s3 = s3 * recipNorm;
			}
			// Apply feedback step
			qDot1 = qDot1 - Beta * s0;
			qDot2 = qDot2 - Beta * s1;
			qDot3 = qDot3 - Beta * s2;
			qDot4 = qDot4 - Beta * s3;
		}
		// Integrate rate of change of quaternion
		q0 = q0 + qDot1 * dt;
		q1 = q1 + qDot2 * dt;
		q2 = q2 + qDot3 * dt;
		q3 = q3 + qDot4 * dt;

		// Normalise quaternion*/
		if ( (q0 * q1 * q2 * q3) != 0 ) {
			recipNorm = 1.0 / sqrt( q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 );
		} else {
			recipNorm = 1.0;
		}
		q0 = q0 * recipNorm;
		q1 = q1 * recipNorm;
		q2 = q2 * recipNorm;
		q3 = q3 * recipNorm;
		
		writing = true;
		gettime = esp_timer_get_time();
		
		// compute Euler angles
		if ( abs(q1 * q3 - q0 * q2) < 0.5 ) {
			Pitch = asin(-2.0 * (q1 * q3 - q0 * q2));
		} else {
			Pitch = M_PI / 2.0 * signbit((q0 * q2 - q1 * q3 ));
		}
		Roll = atan2((q0 * q1 + q2 * q3), (0.5 - q1 * q1 - q2 * q2));
		Yaw = atan2(2.0 * (q1 * q2 + q0 * q3), (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3));
		if (Yaw < 0.0 ) Yaw = Yaw + 2.0 * M_PI;
		if (Yaw > 2.0 * M_PI) Yaw = Yaw - 2.0 * M_PI;
		
		// compute gravity estimation using IMU quaternion
		GravIMUx = -GRAVITY * 2.0 * (q1 * q3 - q0 * q2);
		GravIMUy = -GRAVITY * 2.0 * (q2 * q3 + q0 * q1);
		GravIMUz = -GRAVITY * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

		writing = false;
				
	} else {
		// initialization of quaternion with attitude from accelerometers
		SamplesCount++;
		if ( SamplesCount > SkipSamplesCount ) {
			SumAx = SumAx + ax;
			SumAy = SumAy + ay;
			SumAz = SumAz + az;
			AverageAx = SumAx / SumCount;
			AverageAy = SumAy / SumCount;
			AverageAz = SumAz / SumCount;
			Roll = atan2( -AverageAy, -AverageAz );
			Pitch = asin( AverageAx/ sqrt( AverageAx * AverageAx + AverageAy * AverageAy + AverageAz * AverageAz ) );
			Yaw = 0.0;
			q0=((cos(Roll/2.0)*cos(Pitch/2.0)*cos(Yaw/2.0)+sin(Roll/2.0)*sin(Pitch/2.0)*sin(Yaw/2.0)));
			q1=((sin(Roll/2.0)*cos(Pitch/2.0)*cos(Yaw/2.0)-cos(Roll/2.0)*sin(Pitch/2.0)*sin(Yaw/2.0)));
			q2=((cos(Roll/2.0)*sin(Pitch/2.0)*cos(Yaw/2.0)+sin(Roll/2.0)*cos(Pitch/2.0)*sin(Yaw/2.0)));
			q3=((cos(Roll/2.0)*cos(Pitch/2.0)*sin(Yaw/2.0)-sin(Roll/2.0)*sin(Pitch/2.0)*cos(Yaw/2.0)));
			SumCount++;
			if ( SumCount > AverageSamplesCount ) InitDone = true;
		}
	}
}
