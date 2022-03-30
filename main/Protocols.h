/*
 * Protocols.h
 *
 *  Created on: Dec 20, 2017
 *      Author: iltis
 */
#ifndef PROTOCOLS_H_
#define PROTOCOLS_H_

#include <string>
#include "Setup.h"
#include "S2F.h"
#include "SetupNG.h"

// Supported Protocols
typedef enum protocol_t  { P_OPENVARIO, P_BORGELT, P_CAMBRIDGE, P_EYE_PEYA, P_EYE_PEYI, P_AHRS_RPYL, P_AHRS_APENV1, P_GENERIC, P_XCVARIO, P_XCVARIOFT, P_XCVARIO_DEVEL } proto_t;


class Protocols {
public:
	Protocols( S2F * as2f );
	virtual ~Protocols( );
    void sendNmeaHDM( float heading );
    void sendNmeaHDT( float heading );
	 void sendNmeaIMU( float accelTime=0, float ax=0, float ay=0, float az=0, float gyroTime=0,float gx=0, float gy=0, float gz=0 );
	 void sendNmeaSEN( float statTime=0, float statP=0, float teTime=0, float teP=0, float dynTime=0, float dynP=0, float OATemp=0, float MPUtempcel=0,
                  int fix=0, float gnsstime=0, float gnssaltitude=0, float gnssgroundspeed=0, float gnssspeedx=0, float gnssspeedy=0, float gnssspeedz=0 );
   void sendItem( const char *key, char type, void *value, int len, bool ack=false );
	void sendNMEA( proto_t proto, char* str, float baro, float dp, float te, float temp, float ias, float tas,
                  float mc, int bugs, float ballast, bool cruise, float alt, 
                  bool validTemp=false, float ax=0, float ay=0, float az=0, float gx=0, float gy=0, float gz=0,
                  float accelTime=0, float gyroTime=0, float statP=0, float statTime=0, float teP=0, float teTime=0, float dynP=0, float dynTime=0, float OATemp=0, float MPUtempcel=0,
                  int fix=0, float gnsstime=0, float gnssaltitude=0, float gnssgroundspeed=0, float gnssspeedx=0, float gnssspeedy=0, float gnssspeedz=0 );

	static void parseNMEA( const char *str );
	static int calcNMEACheckSum(const char *nmea);
	static int getNMEACheckSum(const char *nmea);


private:
	static S2F *   _s2f;
	static float   _mc_prev;
	static float   _qnh_prev;
};

#endif /* PROTOCOLS_H_ */
