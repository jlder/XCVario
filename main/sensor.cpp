


#include "sensor.h"

#include "Cipher.h"
#include "BME280_ESP32_SPI.h"
#include <driver/adc.h>
#include <driver/gpio.h>
#include "mcp3221.h"
#include "mcp4018.h"
#include "ESP32NVS.h"
#include "MP5004DP.h"
#include "MS4525DO.h"
#include "ABPMRR.h"
#include "BMPVario.h"
#include "BTSender.h"
#include "Protocols.h"
#include "DS18B20.h"
#include "Setup.h"
#include "ESPAudio.h"
#include "SetupMenu.h"
#include "ESPRotary.h"
#include "AnalogInput.h"
#include "Atmosphere.h"
#include "IpsDisplay.h"
#include "S2F.h"
#include "Version.h"
#include "Polars.h"
#include "Flarm.h"
#include "Blackboard.h"
#include "SetupMenuValFloat.h"

#include <SPI.h>
#include <AdaptUGC.h>
#include <OTA.h>
#include "SetupNG.h"
#include "Switch.h"
#include "AverageVario.h"

#include "MPU.hpp"        // main file, provides the class itself
#include "mpu/math.hpp"   // math helper for dealing with MPU data
#include "mpu/types.hpp"  // MPU data types and definitions
#include "I2Cbus.hpp"
#include "KalmanMPU6050.h"
#include "WifiApp.h"
#include "WifiClient.h"
#include "Serial.h"
#include "LeakTest.h"
#include "Units.h"
#include "Flap.h"
#include "SPL06-007.h"
#include "StraightWind.h"
#include "CircleWind.h"
#include <coredump_to_server.h>
#include "canbus.h"
#include "Router.h"

#include "UbloxGNSSdecode.h"

#include "sdkconfig.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
#include <esp32/rom/miniz.h>
#include <esp32-hal-adc.h> // needed for adc pin reset
#include <soc/sens_reg.h> // needed for adc pin reset
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <string>
#include <cstdio>
#include <cstring>
#include "DataMonitor.h"
#include "AdaptUGC.h"
#include "CenterAid.h"
#include "MPU.hpp"


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// glider specific parameters
//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
#ifdef LS6
	float CLA = 5.75;
	float KAoB = -3.5; // MOD#1 Latest signs
	float KGx = 4.1;
 	float SURFACE = 10.53;
	float WEIGHT = 273;
	float WINGLOAD = WEIGHT / SURFACE;
	float SPEED1 = 90;
	float SINK1 = -0.6;
	float SPEED2 = 145;
	float SINK2 = -1.21;
	float SPEED3 = 180;
	float SINK3 = -1.91;
	float MAXBALLAST = 120.0;
#endif

#ifdef VENTUS3
	float CLA = 5.98;
	float KAoB = -2.97; // MOD#1 Latest signs
	float KGx = 12.0;
 	float SURFACE = 10.5;
	float WEIGHT = 408;
	float WINGLOAD = WEIGHT / SURFACE;
	float SPEED1 = 27.2 * 3.6;
	float SINK1 = -0.52;
	float SPEED2 = 38.3 * 3.6;
	float SINK2 = -0.75;
	float SPEED3 = 55.0 * 3.6;
	float SINK3 = -2.0;
	float MAXBALLAST = 120.0;
#endif

// #include "sound.h"

/*
BMP:
    SCK - This is the SPI Clock pin, its an input to the chip
    SDO - this is the Serial Data Out / Master In Slave Out pin (MISO), for data sent from the BMP183 to your processor
    SDI - this is the Serial Data In / Master Out Slave In pin (MOSI), for data sent from your processor to the BME280
    CS - this is the Chip Select pin, drop it low to start an SPI transaction. Its an input to the chip
 */

#define CS_bme280BA GPIO_NUM_26   // before CS pin 33
#define CS_bme280TE GPIO_NUM_33   // before CS pin 26
#define FREQ_BMP_SPI 13111111/2
#define SPL06_007_BARO 0x77
#define SPL06_007_TE   0x76

#define MGRPS 360

#define DegToRad (M_PI / 180.0)

MCP3221 *MCP=0;
DS18B20  ds18b20( GPIO_NUM_23 );  // GPIO_NUM_23 standard, alternative  GPIO_NUM_17

AirspeedSensor *asSensor=0;
StraightWind theWind;

xSemaphoreHandle xMutex=NULL;
xSemaphoreHandle spiMutex=NULL;

xSemaphoreHandle BTMutex=NULL;
xSemaphoreHandle I2CMutex=NULL;
xSemaphoreHandle dataMutex=NULL;

S2F Speed2Fly;
Protocols OV( &Speed2Fly );

AnalogInput Battery( (22.0+1.2)/1200, ADC_ATTEN_DB_0, ADC_CHANNEL_7, ADC_UNIT_1 );

// Fligth Test
TaskHandle_t mpid = NULL;
// Fligth Test

TaskHandle_t apid = NULL;
TaskHandle_t bpid = NULL;
TaskHandle_t tpid = NULL;
TaskHandle_t dpid = NULL;

e_wireless_type wireless;

PressureSensor *baroSensor = 0;
PressureSensor *teSensor = 0;

AdaptUGC *MYUCG = 0;  // ( SPI_DC, CS_Display, RESET_Display );
IpsDisplay *display = 0;
CenterAid  *centeraid = 0;

bool topDown = false;

OTA *ota = 0;

ESPRotary Rotary;
SetupMenu  *Menu = 0;
DataMonitor DM;

// Gyro and acceleration sensor
I2C_t& i2c = i2c1;  // i2c0 or i2c1
I2C_t& i2c_0 = i2c0;  // i2c0 or i2c1
MPU_t MPU;         // create an object
mpud::float_axes_t accelG;
mpud::float_axes_t gyroDPS;
mpud::float_axes_t accelG_Prev;
mpud::float_axes_t gyroDPS_Prev; 

#define MaxGyroVariation 1.0 // TDB if need to change for other gliders, in particular for motor gliders
#define MaxAccelVariation 10.0 // TDB if need to change for other gliders, in particular for motor gliders

// Fligth Test

float GyroModulePrimLevel = 0.0;

float AccelModulePrimLevel = 0.0;
float dynKp = 0.1;
float DynPeriodVelbi = 8.0;
float DynPeriodVelbiLow = 2.0;
float WingLoad = 40.0;
float GravModuleLP = 0.0;


// Magnetic sensor / compass
Compass *compass = 0;
BTSender btsender;

// IMU variables	
mpud::float_axes_t accelISUNEDMPU;
mpud::float_axes_t PrevaccelISUNEDMPU;
mpud::float_axes_t accelISUNEDBODY;	
mpud::float_axes_t gyroRPS;
mpud::float_axes_t PrevgyroRPS;
mpud::float_axes_t gyroISUNEDMPU;
mpud::float_axes_t gyroISUNEDBODY;
mpud::float_axes_t gyroCorr;
static float GravityModule = 0.0;
static float GravityModuleErr = 0.0;
static float AccelGravModuleFilt = 0.0;
static int32_t gyrobiastemptimer = 0;
static float integralFBx = 0.0;
static float integralFBy = 0.0;
static float integralFBz = 0.0;
static float Pitch = 0.0;
static float Roll = 0.0;
static float RollModule = 0.0;
static float RollModuleLevel = 0.0;
static float Yaw = 0.0;
static float q0 = 1.0;
static float q1 = 0.0;
static float q2 = 0.0;
static float q3 = 0.0;
static float BiasQuatGx = 0.0;
static float BiasQuatGy = 0.0;
static float BiasQuatGz = 0.0;
static float cosRoll = 1.0;
static float sinRoll = 0.0;
static float cosPitch = 1.0;
static float sinPitch = 0.0;
static float Vh = 0.0;
static float DHeading = 0.0;
static float cosDHeading = 1.0;
static float sinDHeading = 0.0;
static float Ub = 0.0;
static float Vb = 0.0;
static float Wb = 0.0;
static float UiPrim = 0.0;
static float ViPrim = 0.0;
static float WiPrim = 0.0;
static float TASbiSquare = 0.0;

float GnssTrack;// MOD#4 gyro bias
float PseudoHeadingPrim;// MOD#4 gyro bias

// variables for gravity estimation
mpud::float_axes_t GravIMU;
mpud::float_axes_t gravISUNEDBODY;

// installation and calibration variables
mpud::float_axes_t currentAccelBias;
mpud::float_axes_t currentAccelGain;
mpud::float_axes_t GroundGyroBias;
mpud::float_axes_t NewGroundGyroBias;
// get installation parameters tilt, sway, distCG
// compute trigonometry
float DistCGVario = 0.0; // distance from CG to vario
float Sway = 0.0; // vario installation roll angle
float Tilt = 0.0; // vario installation pitch angle
float S_S = 0.0; // sinus Sway
float S_T = 0.0; // sinus Tilt
float C_S = 1.0; // cos Sway
float C_T = 1.0; // cos tilt
float STmultSS = 0.0; // ST * SS
float STmultCS = 0.0; // ST * CS
float SSmultCT = 0.0; // SS * CT
float CTmultCS = 1.0; // CT * CS


static char str[500]; 	// string for flight test message broadcast on wireless // TODO reduce size
static int64_t ProcessTimeIMU = 0.0;
static int64_t ProcessTimeSensors = 0.0;
static int64_t gyroTime;  // time stamp for gyros
static int64_t prevgyroTime;
static float dtGyr = PERIOD40HZ; // period between last gyro samples
static int64_t statTime; // time stamp for statP
static int64_t prevstatTime;
static float dtStat = PERIOD10HZ;
static float Prevp = 0.0;
static int64_t dynPTime;
static int64_t prevdynPTime;
static float dtdynP = PERIOD10HZ;
static float PrevdynP=0.0;
static float MPUtempcel; // MPU chip temperature
static float GNSSRouteraw;
static float deltaUbS = 0.0;
static float UbPrimS = 0.0;
static float UbFS = 0.0;
static float deltaVbS = 0.0;
static float VbPrimS = 0.0;
static float VbFS = 0.0;
static float deltaWbS = 0.0;
static float WbPrimS = 0.0;
static float WbFS = 0.0;
static float UbiPrim = 0.0;
static float VbiPrim = 0.0;
static float WbiPrim = 0.0;

float opt_TE = 1;

float Mahonykp = 0.05;
float MagdwickBeta = 0.002;
float CurrentBeta;

static int32_t cur_gyro_bias[3];

bool IMUstream = false; // IMU FT stream
bool SENstream = false; // Sensors FT stream
bool SENDataReady = false; // test SEN data readiness
bool SEN50DataReady = false; // test SEN data readiness
bool CALstream = false; // accel calibration stream
bool CALfirstpass = true; // reset accel calibration data
bool TSTstream = false; // Test stream
bool AHRSstream = false; // stream to test AHRS

// MOD#2 add RTK begin
double RTKtime;
double prevRTKtime;
float dtRTKtime = 0.125;
float RTKEvel = 0.0;
float RTKNvel = 0.0;
float RTKUvel = 0.0;
char RTKmode ='N';
float RTKage = 9.999;
float RTKratio = 0.0;
float RTKEproj = 0.0;
float RTKNproj= 0.0;
float RTKUproj = 0.0;
float RTKheading = 0.0;
// MOD#2 add RTK end

// MOD#5 add Allystar TAU1201 velocity
float dtAllytime = 0.1;
double prevAllytime = 0.1;
double Allytime = 0.0;
float AllyvelN;
float AllyvelE;
float AllyvelU;
float Allyvel3D = 0.0;
float Allyvel2D;
// MOD#5 add Allystar TAU1201 velocity

bool LABtest = false; // LAB switch to limit to one ground bias evaluation
float localGravity = 9.807; // local gravity used during accel calibration. Value is entered using BT $CAL command
uint16_t BIAS_Init = 0; // Bias initialization status (0= no init, n = nth bias calculation
float Bias_AoB = 0.0;

static float GRAVITY = 9.807;

static float dynamicP; // filtered dynamic pressure
static float baroP=0; // barometric pressure
static float temperature=15.0;
static float XCVTemp=15.0;//External temperature for MPU temp control

#define RhoSLISA 1.225
static float Rho;
//static float CASraw;
//static float deltaCAS;
//static float CASprim = 0.0;
// static float CAS = 0.0;
static float Rhocorr = RhoSLISA;
static float Vzbaro = 0.0;
// static float ALT = 0.0;
// static float EnergyFilt = 0.0;
// static float deltaTE;
// static float TEPrim = 0.0;
//static float TEFilt = 0.0;

float NEnergy = 20.0;
float alphaEnergy;
float betaEnergy;
float PeriodVelbi = 8.0;
float LastPeriodVelbi = 7.0;
float fcVelbi1;
float fcVelbi2;
float fcVelbiLow1;
float fcVelbiLow2;

float ALTbiN = 9.0;
float TASbiN = 5.0;
bool NALTbiTASbiChanged = true;

static float Ubi = 0.0;
static float Vbi = 0.0;
static float Wbi = 0.0;
static float Vxbi = 0.0;
static float Vybi = 0.0;
static float Vzbi = 0.0;
static float ALTbi = 0.0;

// TODO event counter
int16_t Event = 0;
int16_t EventHoldTime = 0;

static float battery=0.0;

float slipAngle = 0.0;

// global color variables for adaptable display variant
uint8_t g_col_background=255; // black
uint8_t g_col_highlight=0;
uint8_t g_col_header_r=101+g_col_background/5;
uint8_t g_col_header_g=108+g_col_background/5;
uint8_t g_col_header_b=g_col_highlight;
uint8_t g_col_header_light_r=161-g_col_background/4;
uint8_t g_col_header_light_g=168-g_col_background/3;
uint8_t g_col_header_light_b=g_col_highlight;
uint16_t gear_warning_holdoff = 0;
uint8_t gyro_flash_savings=0;

t_global_flags gflags = { true, false, false, false, false, false, false, false, false, false, false, false, false, false };

int  ccp=60;
float tas = 0;
float cas = 0;
float aTE = 0;
float alt_external;
float altSTD;
float netto = 0;
float as2f = 0;
float s2f_delta = 0;
float polar_sink = 0;

gnss_data_t *chosenGnss;
float GNSSstatus=0.0;

float      stall_alarm_off_kmh=0;
uint16_t   stall_alarm_off_holddown=0;

int count=0;
int countIMU=0;
int flarm_alarm_holdtime=0;
int the_can_mode = CAN_MODE_MASTER;
int active_screen = 0;  // 0 = Vario

float mpu_target_temp=45.0;

static int mtick = 0; //counter to schedule specific tasks within a function*

AdaptUGC *egl = 0;

// Fligth Test
extern UbloxGnssDecoder s1UbloxGnssDecoder;
extern UbloxGnssDecoder s2UbloxGnssDecoder;

// alpha beta filter class
class AlphaBeta {
private:
	float dtMax = 0.0;
	float dtMin = 0.0;
	float delta = 0.0;
	float prim = 0.0;
	float filt = 0.0;
	float _delta = 0.0;
	float _prim = 0.0;
	float _filt = 0.0;	
	float alpha = 0.0;
	float beta = 0.0;
	float Threshold = 0.0;
	float primMin = 0.0;
	float primMax = 0.0;
	float filtMin = 0.0;
	float filtMax = 0.0;
	bool firstpass = true;
	int zicket = 0;
	bool writing = false;
	float gettime = 0.0;
public:
	void ABinit( float N, float dtTypical ) {
		ABinit( N, dtTypical, 0.0, 0.0, 0.0, 0.0, 0.0 );
	}
	void ABinit( float N, float dtTypical, float _Threshold ) {
		ABinit( N, dtTypical, _Threshold, 0.0, 0.0, 0.0, 0.0 );
	}
	void ABinit( float N, float dtTypical, float _Threshold, float _filtMin, float _filtMax ) {
		ABinit( N, dtTypical, _Threshold, _filtMin, _filtMax, 0.0, 0.0 );
	}
	void ABinit( float N, float dtTypical, float _Threshold, float _filtMin, float _filtMax, float _primMin, float _primMax ) {
		if ( N != 0.0  ) {
			alpha =  (2.0 * (2.0 * N - 1.0) / N / (N + 1.0));
			beta = (6.0 / N / (N + 1.0));
			dtMax = dtTypical * 4.0;
			dtMin = dtTypical / 4.0;
		}
		firstpass = true;
		Threshold = _Threshold;
		filtMin = _filtMin;
		filtMax = _filtMax;
		primMin = _primMin;
		primMax = _primMax;
	}
	void ABNupdt( float N ) {
		if ( N > 0.0  ) {
			alpha =  (2.0 * (2.0 * N - 1.0) / N / (N + 1.0));
			beta = (6.0 / N / (N + 1.0));
		}
	}		
	void ABupdate(float dt, float RawData ) {
		#define MaxZicket 2 // maximum number of concecuitives zickets to let the filter track the signal. If zicket is higher a step change in signal is suspected
		// process sample if dt above dtMin and below dtMax (dtMin typicaly average dt / 4 and dtMax typicaly 4 x average dt)
		writing = false;
		if ( dt > dtMin && dt < dtMax  ) {
			if ( firstpass ) { // initialize filter variables when first called
				writing = true;
				gettime = esp_timer_get_time();
				filt = RawData;
				_filt = RawData;
				prim = 0.0;
				_prim = 0.0;
				writing = false;
				firstpass = false;
				zicket = 4*MaxZicket;
			} else {
				if ( zicket <= MaxZicket ) { 
					// if filter stable
					delta = RawData - filt;
					if ( (abs(delta) < Threshold ) || (Threshold == 0.0) ) {
						// new data below threshold
						writing = true;
						gettime = esp_timer_get_time();
						prim = prim + beta * delta / dt;
						if ( primMin != 0.0 || primMax != 0.0 ) {
							if ( prim < primMin ) prim = primMin;
							if ( prim > primMax ) prim = primMax;
						}
						filt = filt + alpha * delta + prim * dt;
						if ( filtMin != 0.0 || filtMax != 0.0 ) {
							if ( filt < filtMin ) filt = filtMin;
							if ( filt > filtMax ) filt = filtMax;
						}
						writing = false;
						zicket = 0;
					} else {
						// new data above threshold, additional zicket
						zicket++;
						if ( zicket > MaxZicket ) {
							// if new zicket makes filter unstable (step change), arm and switch to alternate AB filter
							_prim = prim;
							_filt = filt;
							zicket = 4*MaxZicket;
						}
					}
				} else {
					// if filter unstable - step change
					// update alternate filter to track step change
					_delta = RawData - _filt;
					_prim = _prim + beta * _delta / dt;
					_filt = _filt + alpha * _delta + _prim * dt;
					// if new data below threshold, reduce number of zicket
					if ( abs(_delta) < Threshold || (Threshold == 0.0) ) zicket--; else zicket = 4*MaxZicket;
					if ( zicket <= MaxZicket ) {
						// if number of zicket below stability criteria, arm and switch to primary filter
						writing = true;
						gettime = esp_timer_get_time();
						prim = _prim;
						filt = _filt;
						writing = false;
						zicket = 0;
					}						
				}
			}
		}
	}
	float ABfilt(void) {
		while( writing ) {
			if ( abs( esp_timer_get_time() - gettime ) > 0.001 ) break; // wait for 1 ms max if writing is in process
		}
		return filt;
	}
	float ABprim(void) {
		while( writing ) {
			if ( abs( esp_timer_get_time() - gettime ) > 0.001 ) break; // wait for 1 ms max if writing is in process
		}
		return prim;
	}
	bool Stable(void) {
		bool test = true;
		if ( zicket == 0 ) return test; else return !test;
	}
};

class LowPassFilter {
private:
	float output1 = 0.0;
	float output2 = 0.0;
	float alpha = 1.0; // Filter coefficients
	float beta = 0.0;
	bool writing = false;
	float gettime = 0.0;
public:
	void LPinit( float cutoffperiod, float dt ) {
		alpha = cutoffperiod / (cutoffperiod + dt);
		beta = 1.0 - alpha;
	}
    void LPupdate( float input ) {
		writing = true;
		gettime = esp_timer_get_time();
		output1 = alpha * output1 + beta * input;
		output2 = alpha * output2 + beta * output1;
		writing = false;
    }
	float LowPass1(void) {
		while( writing ) {
			if ( abs( esp_timer_get_time() - gettime ) > 0.001 ) break; // wait for 1 ms max if writing is in process
		}		
		return output1;
	}
	float LowPass2(void) {
		while( writing ) {
			if ( abs( esp_timer_get_time() - gettime ) > 0.001 ) break; // wait for 1 ms max if writing is in process
		}		
		return output2;
	}
 };
 
 
class SetGet {
private:
	bool writing = false;
	float data = 0.0;
	float gettime = 0.0;
public:
	void Set( float value ) {
		writing = true;
		gettime = esp_timer_get_time();		
		data = value;
		writing = false;
	}
    float Get( void ) {
		while( writing ) {
			if ( abs( esp_timer_get_time() - gettime ) > 0.001 ) break; // wait for 1 ms max if writing is in process
		}
		return data; 
    }
 };

// alpha beta class for gyros
AlphaBeta gyroNEDx, gyroNEDy, gyroNEDz;

// alpha beta class for accels
AlphaBeta accelNEDBODYx, accelNEDBODYy, accelNEDBODYz;
float accelNEDBODYzNorm;

// alpha beta class for gyro and accel module
AlphaBeta GyroModule, AccelModule;

// alpha beta filters classes for AHRS roll and pitch
AlphaBeta RollAHRS, PitchAHRS;

// alpha beta class for kinetic accels
AlphaBeta UiPrimF, ViPrimF, WiPrimF;

// alpha beta filters classes for GNSS vector coordinates
AlphaBeta GnssVx, GnssVy, GnssVz;

// alpha beta filters for Energy and average Energy calculations
AlphaBeta KinEnergy, ALTbiEnergy, TASbiEnergy;

// declare alpha beta for CAS and ALT
AlphaBeta CAS, ALT;

// Outside temp alpha beta class 
AlphaBeta OATemp;

// declare low pass for TotalEnergy and AverageTotalEnergy
LowPassFilter TotalEnergy, AverageTotalEnergy;

// declare low pass filter for GyroBiasx,y and z
LowPassFilter GyroBiasx, GyroBiasy, GyroBiasz;

// declare low pass filters for motor glider
LowPassFilter AccelMotor1, AccelMotor2;

// declare low pass for outside temperature
LowPassFilter temperatureLP;

// declare low pass for AoB bias
LowPassFilter BiasAoB;


// declare SetGet class to reduce read write conflicts between tasks
SetGet statP, teP, dynP, TAS, Vztotbi, TASbi, AoA, AoB, gyroCorrx;

#define GYRO_FS (mpud::GYRO_FS_250DPS)


float getTAS() { return tas; };

bool do_factory_reset() {
	return( SetupCommon::factoryReset() );
}

void drawDisplay(void *pvParameters){
	while (1) {
		if( Flarm::bincom ) {
			if( gflags.flarmDownload == false ) {
				gflags.flarmDownload = true;
				display->clear();
				Flarm::drawDownloadInfo();
			}
			// Flarm IGC download is running, display will be blocked, give Flarm
			// download all cpu power.
			vTaskDelay(20/portTICK_PERIOD_MS);
			continue;
		}
		else if( gflags.flarmDownload == true ) {
			gflags.flarmDownload = false;
			display->clear();
		}
		// TickType_t dLastWakeTime = xTaskGetTickCount();
		if( gflags.inSetup != true ) {
			float t=OAT.get();
			if( gflags.validTemperature == false )
				t = DEVICE_DISCONNECTED_C;
			float airspeed = 0;
			if( airspeed_mode.get() == MODE_IAS )
				airspeed = ias.get();
			else if( airspeed_mode.get() == MODE_TAS )
				airspeed = tas;
			else if( airspeed_mode.get() == MODE_CAS )
				airspeed = cas;
			else
				airspeed = ias.get();

			// Stall Warning Screen
			if( stall_warning.get() && gload_mode.get() != GLOAD_ALWAYS_ON ){  // In aerobatics stall warning is contra productive, we concentrate on G-Load Display if permanent enabled
				if( gflags.stall_warning_armed ){
					float acceleration=accelG[0];
					if( acceleration < 0.3 )
						acceleration = 0.3;  // limit acceleration effect to minimum 30% of 1g
					float acc_stall= stall_speed.get() * sqrt( acceleration + ( ballast.get()/100));  // accelerated and ballast(ed) stall speed
					if( ias.get() < acc_stall && ias.get() > acc_stall*0.7 ){
						if( !gflags.stall_warning_active ){
							Audio::alarm( true );
							display->drawWarning( "! STALL !", true );
							gflags.stall_warning_active = true;
						}
					}
					else{
						if( gflags.stall_warning_active ){
							Audio::alarm( false );
							display->clear();
							gflags.stall_warning_active = false;
						}
					}
					if( ias.get() < stall_alarm_off_kmh ){
						stall_alarm_off_holddown++;
						if( stall_alarm_off_holddown > 1200 ){  // ~30 seconds holddown
							gflags.stall_warning_armed = false;
							stall_alarm_off_holddown=0;
						}
					}
					else{
						stall_alarm_off_holddown=0;
					}
				}
				else{
					if( ias.get() > stall_speed.get() ){
						gflags.stall_warning_armed = true;
						stall_alarm_off_holddown=0;
					}
				}
			}
			if( gear_warning.get() ){
				if( !gear_warning_holdoff ){
					int gw = 0;
					if( gear_warning.get() == GW_EXTERNAL ){
						gw = gflags.gear_warn_external;
					}else{
						gw = digitalRead( SetupMenu::getGearWarningIO() );
						if( gear_warning.get() == GW_FLAP_SENSOR_INV || gear_warning.get() == GW_S2_RS232_RX_INV ){
							gw = !gw;
						}
					}
					if( gw ){
						if( ESPRotary::readSwitch() ){   // Acknowledge Warning -> Warning OFF
							gear_warning_holdoff = 25000;  // 5 min
							Audio::alarm( false );
							display->clear();
							gflags.gear_warning_active = false;
							SetupMenu::catchFocus( false );
						}
						else if( !gflags.gear_warning_active && !gflags.stall_warning_active ){
							Audio::alarm( true );
							display->drawWarning( "! GEAR !", false );
							gflags.gear_warning_active = true;
							SetupMenu::catchFocus( true );
						}
					}
					else{
						if( gflags.gear_warning_active ){
							SetupMenu::catchFocus( false );
							Audio::alarm( false );
							display->clear();
							gflags.gear_warning_active = false;
						}
					}
				}
				else{
					gear_warning_holdoff--;
				}
			}

			// Flarm Warning Screen
			if( flarm_warning.get() && !gflags.stall_warning_active && Flarm::alarmLevel() >= flarm_warning.get() ){ // 0 -> Disable
				// ESP_LOGI(FNAME,"Flarm::alarmLevel: %d, flarm_warning.get() %d", Flarm::alarmLevel(), flarm_warning.get() );
				if( !gflags.flarmWarning ) {
					gflags.flarmWarning = true;
					delay(100);
					display->clear();
					flarm_alarm_holdtime = 250;
				}
			}
			else{
				if( gflags.flarmWarning && (flarm_alarm_holdtime == 0) ){
					gflags.flarmWarning = false;
					display->clear();
					Audio::alarm( false );
				}
			}
			if( gflags.flarmWarning )
				Flarm::drawFlarmWarning();
			// G-Load Display
			// ESP_LOGI(FNAME,"Active Screen = %d", active_screen );
			if( (((float)accelG[0] > gload_pos_thresh.get() || (float)accelG[0] < gload_neg_thresh.get()) && gload_mode.get() == GLOAD_DYNAMIC ) ||
					( gload_mode.get() == GLOAD_ALWAYS_ON ) || ((active_screen << SCREEN_GMETER) & 1)  )
			{
				if( !gflags.gLoadDisplay ){
					gflags.gLoadDisplay = true;
				}
			}
			else{
				if( gflags.gLoadDisplay ) {
					gflags.gLoadDisplay = false;
				}
			}
			if( gflags.gLoadDisplay ) {
				display->drawLoadDisplay( (float)accelG[0] );
			}
			// G-Load Alarm when limits reached
			if( gload_mode.get() != GLOAD_OFF  ){
				if( (float)accelG[0] > gload_pos_limit.get() || (float)accelG[0] < gload_neg_limit.get()  ){
					if( !gflags.gload_alarm ) {
						Audio::alarm( true );
						gflags.gload_alarm = true;
					}
				}else
				{
					if( gflags.gload_alarm ) {
						Audio::alarm( false );
						gflags.gload_alarm = false;
					}
				}
			}
			// Vario Screen
			if( !(gflags.stall_warning_active || gflags.gear_warning_active || gflags.flarmWarning || gflags.gLoadDisplay )  ) {
				// ESP_LOGI(FNAME,"TE=%2.3f", te_vario.get() );
// modif gfm affichage d'une tension batterie nulle tant que les biais gyros n'ont pas été initialisés
				if (  (BIAS_Init > 0)  || (TAS.Get() > 15.0) ){
					display->drawDisplay( airspeed, Vztotbi.Get() /*te_vario.get()*/, AverageTotalEnergy.LowPass1()/*aTE*/, polar_sink, altitude.get(), t, battery, s2f_delta, as2f, average_climb.get(), Switch::getCruiseState(), gflags.standard_setting, flap_pos.get() );
				}	
				else {
					display->drawDisplay( airspeed,  Vztotbi.Get() /*te_vario.get()*/, AverageTotalEnergy.LowPass1() /*aTE*/, polar_sink, altitude.get(), t, 0.0, s2f_delta, as2f, average_climb.get(), Switch::getCruiseState(), gflags.standard_setting, flap_pos.get() );
				}
// fin modif gfm
				}
			if( screen_centeraid.get() ){
				if( centeraid ){
					centeraid->tick();
				}
			}
		}
		if( flarm_alarm_holdtime )
			flarm_alarm_holdtime--;
//		drawDisplayTime = (esp_timer_get_time()/1000.0) - drawDisplayTime;
//		ESP_LOGI(FNAME,"drawDisplay: %0.1f  / %0.1f", drawDisplayTime, 20.0 );
		vTaskDelay(20/portTICK_PERIOD_MS);
		if( uxTaskGetStackHighWaterMark( dpid ) < 512  )
			ESP_LOGW(FNAME,"Warning drawDisplay stack low: %d bytes", uxTaskGetStackHighWaterMark( dpid ) );
	}
}

// depending on mode calculate value for Audio and set values accordingly
void doAudio(){
	polar_sink = Speed2Fly.sink( ias.get() );
	float netto = Vztotbi.Get() /*te_vario.get()*/ - polar_sink; // TODO clean new energt calcul / audio
	as2f = Speed2Fly.speed( netto, !Switch::getCruiseState() );
	s2f_delta = s2f_delta + ((as2f - ias.get()) - s2f_delta)* (1/(s2f_delay.get()*10)); // low pass damping moved to the correct place
	// ESP_LOGI( FNAME, "te: %f, polar_sink: %f, netto %f, s2f: %f  delta: %f", aTES2F, polar_sink, netto, as2f, s2f_delta );
	if( vario_mode.get() == VARIO_NETTO || (Switch::getCruiseState() &&  (vario_mode.get() == CRUISE_NETTO)) ){
		if( netto_mode.get() == NETTO_RELATIVE )
			Audio::setValues( Vztotbi.Get() /*te_vario.get()*/ - polar_sink + Speed2Fly.circlingSink( ias.get() ), s2f_delta );// TODO clean new energt calcul / audio
		else if( netto_mode.get() == NETTO_NORMAL )
			Audio::setValues( Vztotbi.Get() /*te_vario.get()*/ - polar_sink, s2f_delta );// TODO clean new energt calcul / audio
	}
	else {
		Audio::setValues( Vztotbi.Get() /*te_vario.get()*/, s2f_delta ); // TODO add 1.5 factor to vario audio to make sound more "nervous"
	}
}


void audioTask(void *pvParameters){
	while (1)
	{
		TickType_t xLastWakeTime = xTaskGetTickCount();
		if( Flarm::bincom ) {
			// Flarm IGC download is running, audio will be blocked, give Flarm
			// download all cpu power.
			vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
			continue;
		}
		doAudio();
		Router::routeXCV();
//		audioTaskTime = (esp_timer_get_time()/1000.0) - audioTaskTime;
//		ESP_LOGI(FNAME,"audioTask: %0.1f  / %0.1f", audioTaskTime, 100.0 );
		if( uxTaskGetStackHighWaterMark( apid )  < 512 )
			ESP_LOGW(FNAME,"Warning audio task stack low: %d", uxTaskGetStackHighWaterMark( apid ) );
		vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
	}
}

void MagdwickUpdateIMU(	float dt, float Beta, float gx, float gy, float gz, float ax, float ay, float az, float &AccelGravModule ) {

	float  recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	AccelGravModule = sqrt(ax * ax + ay * ay + az * az);
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
}


void MahonyUpdateIMU(float dt, float gxraw, float gyraw, float gzraw, float ax, float ay, float az, float &AccelGravModule ) {

#define Nlimit 0.15 // stability criteria for gravity estimation from accels in m/s²
#define Kp 0.7 // proportional feedback to sync quaternion

#define Gyroprimlimit 0.3

float gx, gy, gz;
float QuatModule, recipNorm;
float halfvx = 0.0;
float halfvy = 0.0;
float halfvz = 0.0;
float halfex = 0.0;
float halfey = 0.0;
float halfez = 0.0;
float qa, qb, qc;
float dynKi = Kp/10;

	// Estimate direction of gravity from IMU quaternion
	halfvx = q1 * q3 - q0 * q2;
	halfvy = q0 * q1 + q2 * q3;
	halfvz = q0 * q0 - 0.5 + q3 * q3;

	// Update IMU quaternion
	// IMU quaternion is updated by integrating rate of change (gyro)
	// gyros are corrected using estimated bias and estimaded error with vertical from accelerometer (gravity)
	//
	// correct raw gyro with estimated gyro bias
	gx = gxraw;// + Bias_Gx; // error on x should be added to gyro
	gy = gyraw;// + Bias_Gy; // error on y should be added to gyro
	gz = gzraw;// + Bias_Gz; // error on z should be added to gyro
	
	// Compute feedback error only if accelerometer measurement is valid (avoids NaN in accelerometer normalisation)
	AccelGravModule = sqrt( ax * ax + ay * ay + az * az );
	if ( AccelGravModule != 0.0) {
		// gyro should be corrected using error between vertical from IMU quaternion and observered vertical from accels.
		// gyro correction is performed with PI feedback using Kp and Ki (proportional & integral) coefficients.

		dynKp = Mahonykp; 
		dynKi = Mahonykp/10;
		
		// Normalise accelerometer measurement
		recipNorm = 1.0 / AccelGravModule;
		ax *=recipNorm;
		ay *=recipNorm;
		az *=recipNorm;
		// Estimate error between vertical from IMU quaternion and vertical from accels. Error is sum of cross product between Quaternion and accels gravity estimations
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
		// integral feedback
		integralFBx = integralFBx + dynKi * halfex * dt;
		integralFBy = integralFBy + dynKi * halfey * dt;
		integralFBz = integralFBz + dynKi * halfez * dt;
		// apply integral feedback to gyros
		gx = gx + integralFBx; 
		gy = gy + integralFBy;
		// gz = gz + integralFBz; // do not integrate error on the z axis
		// Apply proportional feedback to gyros
		gx = gx + dynKp * halfex;
		gy = gy + dynKp * halfey;
		gz = gz + dynKp * halfez;
	}

	// Integrate rate of change of IMU quaternion
	gx = gx * 0.5 * dt;
	gy = gy * 0.5 * dt;
	gz = gz * 0.5 * dt;
	qa = q0;
	qb = q1;
	qc = q2;
	q0 +=(-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise IMU quaternion
	QuatModule = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if ( QuatModule != 0.0) {	
		recipNorm = 1.0 / QuatModule;	
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
	}

}

static void processIMU(void *pvParameters)
{
// processIMU process reads all sensors information required for processing total energy calculation, including attitude estimation
// it is also able to stream flight test data to BT which is used in post processing to validate algorithms.
// This process gets following information:
// - MPU, converts accels and gyros to NED frame in International System Units : m/s² for accels and rad/s for rotation rates.
// - Sensors data, static, TE, dynamic pressure, OAT and MPU temp
// - Ublox GNSS data. 

	// MPU data
	mpud::float_axes_t RawaccelISUNEDMPU;
	mpud::float_axes_t deltaGyroRPS;
	mpud::float_axes_t GyroRPSPrim;
	mpud::float_axes_t GyroRPSFilt;
	GyroRPSPrim.x = 0.0;
	GyroRPSFilt.x = 0.0;
	GyroRPSPrim.y = 0.0;
	GyroRPSFilt.y = 0.0;
	GyroRPSPrim.z = 0.0;
	GyroRPSFilt.z = 0.0;

	mpud::float_axes_t deltaaccelISUNEDMPU;
	mpud::float_axes_t accelISUNEDMPUPrim;
	mpud::float_axes_t accelISUNEDMPU;
	accelISUNEDMPUPrim.x = 0.0;
	accelISUNEDMPU.x = 0.0;
	accelISUNEDMPUPrim.y = 0.0;
	accelISUNEDMPU.y = 0.0;
	accelISUNEDMPUPrim.z = 0.0;
	accelISUNEDMPU.z = -9.807;

	#define GroundAccelprimlimit 2.5 // m/s3
	#define	GroundGyroprimlimit 0.55// rad/s2
	
	mpud::raw_axes_t accelRaw;
	mpud::raw_axes_t gyroRaw;
	
	// variables for accel calibration
	float accelMaxx = 0.0;
	float accelMaxy = 0.0;
	float accelMaxz = 0.0;
	float accelMinx = 0.0;
	float accelMiny = 0.0;
	float accelMinz = 0.0;
	float accelAvgx = 0.0;	
	float accelAvgy = 0.0;	
	float accelAvgz = 0.0;	
	int16_t gyromodulestable = 8; // MOD#6 improve accel calibration
	float GyrxzAmplitudeBIdyn = 0.0;	

	GravIMU.x = 0.0;
	GravIMU.y = 0.0;
	GravIMU.z = 0.0;
	
	// variables for bias estimation
	int16_t gyrostable = 0;
	int16_t averagecount = 0;
	float GxBias = 0.0;
	float GyBias = 0.0;
	float GzBias = 0.0;
	// variables for gravity estimation
	float Gravx = 0.0;
	float Gravy = 0.0;
	float Gravz = 0.0;	
	// variables for Euler/quaternion initialisation
	float PitchInit = 0.0;
	float RollInit = 0.0;
	float YawInit = 0.0;
	int16_t AttitudeInit = 0;
	
	// initialize prevgyrotime
	prevgyroTime = esp_timer_get_time()/1000.0;
	PrevgyroRPS.x = 0.0;
	PrevgyroRPS.y = 0.0;
	PrevgyroRPS.z = 0.0;
	PrevaccelISUNEDMPU.x = 0.0;
	PrevaccelISUNEDMPU.y = 0.0;
	PrevaccelISUNEDMPU.z = -9.807;
	
	// alpha beta gyros parameters
	#define NGyro 5 //  AB Filter parameter
	#define Gyrodt 0.025 // typical Gyro dt	
	#define GyroOutlier 4.0 // 4 rad/s maximum variation sample to sample
	#define Gyromin -4.0
	#define Gyromax 4.0
	//gyroRPSx.ABinit( NGyro, Gyrodt, GyroOutlier, Gyromin, Gyromax );
	//gyroRPSy.ABinit( NGyro, Gyrodt, GyroOutlier, Gyromin, Gyromax );
	//gyroRPSz.ABinit( NGyro, Gyrodt, GyroOutlier, Gyromin, Gyromax );
	gyroNEDx.ABinit( NGyro, Gyrodt, GyroOutlier, Gyromin, Gyromax );
	gyroNEDy.ABinit( NGyro, Gyrodt, GyroOutlier, Gyromin, Gyromax );
	gyroNEDz.ABinit( NGyro, Gyrodt, GyroOutlier, Gyromin, Gyromax );	

	// alpha beta accels parameters
	float NAccel = 5.0; //  AB Filter N parameter
	#define Accdt 0.025 // typical Acc dt	
	#define AccelOutlier 60.0 // 60 m/s² maximum variation sample to sample
	#define Accelmin -60.0
	#define Accelmax 60.0
	accelNEDBODYx.ABinit( NAccel, Accdt, AccelOutlier, Accelmin, Accelmax );
	accelNEDBODYy.ABinit( NAccel, Accdt, AccelOutlier, Accelmin, Accelmax );	
	accelNEDBODYz.ABinit( NAccel, Accdt, AccelOutlier, Accelmin, Accelmax );	
	
	// alpha beta gyro and accel module parameters
	#define NModule 4 //  AB Filter parameter
	#define Moduledt 0.025 // typical Module dt	
	GyroModule.ABinit( NModule, Moduledt );
	AccelModule.ABinit( NModule, Moduledt );

	// alpha beta AHRS parameters
	#define NAHRS 5 //  AB Filter parameter
	#define AHRSdt 0.025 // typical AHRS dt	
	#define AHRSOutliers 1.0 // remove outiliers 1.0 rad away from signal sample to sample
	#define AHRSmin -4.0
	#define AHRSmax 4.0
	RollAHRS.ABinit( NAHRS, AHRSdt, AHRSOutliers, AHRSmin, AHRSmax );
	PitchAHRS.ABinit( NAHRS, AHRSdt, AHRSOutliers, AHRSmin, AHRSmax );
	
	// alpha beta parameters for kinetic accels
	#define NIPRIM 5
	#define IPrimdt 0.025
	UiPrimF.ABinit( NIPRIM, IPrimdt );
	ViPrimF.ABinit( NIPRIM, IPrimdt );
	WiPrimF.ABinit( NIPRIM, IPrimdt );
	
	// LP filter initialization
	#define GyroCutoffPeriod 500 //  very long term average ~500 seconds
	GyroBiasx.LPinit( GyroCutoffPeriod, Gyrodt ); // LP period GyroCutoffPeriod seconds and sample period Gyrodt second
	GyroBiasy.LPinit( GyroCutoffPeriod, Gyrodt ); // LP period GyroCutoffPeriod seconds and sample period Gyrodt second
	GyroBiasz.LPinit( GyroCutoffPeriod, Gyrodt ); // LP period GyroCutoffPeriod seconds and sample period Gyrodt second
	float PitchPrim;
	float RollPrim;
	
	// alpha beta parameters for CAS and ALT
	#define NCAS 8 // CAS alpha/beta filter coeff
	#define CASdt 0.025 // average CAS dt	
	#define SpeedOutliers 30.0 // 30 m/s maximum variation sample to sample
	#define CASmin 0.0
	#define CASmax 100.0
	#define CASPrimmin -30.0
	#define CASPrimmax 30.0
	CAS.ABinit( NCAS, CASdt, SpeedOutliers, CASmin, CASmax, CASPrimmin, CASPrimmax );
	#define NALT 8 // ALT alpha/beta coeff
	#define ALTdt 0.025 // average ALT dt	
	#define AltitudeOutliers 30.0 // 30 m maximum variation sample to sample
	#define Altmin -500.0
	#define Altmax 12000
	ALT.ABinit( NALT, ALTdt, AltitudeOutliers, Altmin, Altmax );

	float dp = 0.0;
	bool ALTbiFirstPass = true;	
	
	// compute once the filter parameters in functions of values in FLASH
	PeriodVelbi = velbi_period.get(); // period in second for baro/inertial velocity. period long enough to reduce effect of baro wind gradients
	LastPeriodVelbi = PeriodVelbi;
	
	Mahonykp = kp_Mahony.get(); // get last kp value from NV memory
	MagdwickBeta = Beta_Magdwick.get(); // get last ki value from NV memory
	ALTbiN = ALTbi_N.get(); // get last N for ALTbi A/B filter from NV memory
	TASbiN = TASbi_N.get(); // get last N delta between ALTbi and TASbi from NV memory
	
	SENDataReady = false;
	SEN50DataReady = false;
	
	if( nmea_streams.get() & FT_IMU ) {
		IMUstream = true;
		AHRSstream = false;
	} else
		IMUstream = false;
	if( nmea_streams.get() & FT_SENSOR ) {
		SENstream = true;
		AHRSstream = false;
	} else
		SENstream = false;
	if( nmea_streams.get() & FT_AHRS ) {
		IMUstream = false;
		SENstream = false;
		AHRSstream = true;
	} else
		AHRSstream = false;
		
	while (1) {
		countIMU++;
		TickType_t xLastWakeTime_mpu =xTaskGetTickCount();
		
		// get raw gyro data
		esp_err_t errMPU = MPU.rotation(&gyroRaw);// read raw gyro data
 		if( errMPU == ESP_OK ){ 
			// compute precise dt between current and previous samples
			prevgyroTime = gyroTime;
			gyroTime = esp_timer_get_time()/1000.0; // record time of gyro measurement in milli second
			dtGyr = (gyroTime - prevgyroTime) / 1000.0; // period between last two valid samples in second
			// If dtGyr is negative or abnormaly high, skip filters and baro inertial and AHRS updates using dtGyr = 0
			if (dtGyr < 0.0 or dtGyr > 0.075) dtGyr = 0.0;
			gyroDPS = mpud::gyroDegPerSec(gyroRaw, GYRO_FS); // For compatibility with Eckhard code only. Convert raw gyro to Gyro_FS full scale in degre per second 
			gyroRPS = mpud::gyroRadPerSec(gyroRaw, GYRO_FS); // convert raw gyro to Gyro_FS full scale in radians per second
			// convert gyro coordinates to ISU : rad/s NED MPU and remove bias
			//xSemaphoreTake( dataMutex, 3/portTICK_PERIOD_MS ); // prevent data conflicts for 3ms max.
			// TODO just for flight test. If Magdwick Beta and Mahonykp are set to zero for gyro drift analysis, do not apply bias estimation to gyro values
			if ( MagdwickBeta != 0.0 || Mahonykp != 0.0 ) {
				gyroISUNEDMPU.x = -(gyroRPS.z - GroundGyroBias.z);
				gyroISUNEDMPU.y = -(gyroRPS.y - GroundGyroBias.y);
				gyroISUNEDMPU.z = -(gyroRPS.x - GroundGyroBias.x);				
			} else {
				gyroISUNEDMPU.x = -gyroRPS.z;
				gyroISUNEDMPU.y = -gyroRPS.y;
				gyroISUNEDMPU.z = -gyroRPS.x;
			}			
			// convert NEDMPU to NEDBODY and apply bias estimation
			gyroISUNEDBODY.x = C_T * gyroISUNEDMPU.x + STmultSS * gyroISUNEDMPU.y + STmultCS * gyroISUNEDMPU.z; // + BiasQuatGx;
			gyroISUNEDBODY.y = C_S * gyroISUNEDMPU.y - S_S * gyroISUNEDMPU.z; //+ BiasQuatGy;
			gyroISUNEDBODY.z = -S_T * gyroISUNEDMPU.x + SSmultCT  * gyroISUNEDMPU.y + CTmultCS * gyroISUNEDMPU.z; // + BiasQuatGz;
			// update gyro filters
			gyroNEDx.ABupdate(dtGyr, gyroISUNEDBODY.x );
			gyroNEDy.ABupdate(dtGyr, gyroISUNEDBODY.y );			
			gyroNEDz.ABupdate(dtGyr, gyroISUNEDBODY.z );			
			// apply gyro filters
			gyroCorr.x = gyroNEDx.ABfilt();
			gyroCorrx.Set( gyroCorr.x );
			gyroCorr.y = gyroNEDy.ABfilt();
			gyroCorr.z = gyroNEDz.ABfilt();
			//xSemaphoreGive( dataMutex );
		}
		// get accel data
		errMPU = MPU.acceleration(&accelRaw);// read raw gyro data
		if( errMPU == ESP_OK ){ // read raw acceleration
			accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_8G);  // For compatibility with Eckhard code only. Convert raw data to to 8G full scale
			// convert accels coordinates to ISU : m/s² NED MPU
			RawaccelISUNEDMPU.x = ((-accelG.z*9.807) - currentAccelBias.x ) * currentAccelGain.x;
			RawaccelISUNEDMPU.y = ((-accelG.y*9.807) - currentAccelBias.y ) * currentAccelGain.y;
			RawaccelISUNEDMPU.z = ((-accelG.x*9.807) - currentAccelBias.z ) * currentAccelGain.z;

			// convert from MPU to BODY and filter with A/B
			//xSemaphoreTake( dataMutex, 3/portTICK_PERIOD_MS ); // prevent data conflicts for 3ms max.				
			accelISUNEDBODY.x = C_T * RawaccelISUNEDMPU.x + STmultSS * RawaccelISUNEDMPU.y + STmultCS * RawaccelISUNEDMPU.z + ( gyroCorr.y * gyroCorr.y + gyroCorr.z * gyroCorr.z ) * DistCGVario ;
			accelNEDBODYx.ABupdate( dtGyr, accelISUNEDBODY.x );
			accelISUNEDBODY.y = C_S * RawaccelISUNEDMPU.y - S_S * RawaccelISUNEDMPU.z;
			accelNEDBODYy.ABupdate( dtGyr, accelISUNEDBODY.y  );
			accelISUNEDBODY.z = -S_T * RawaccelISUNEDMPU.x + SSmultCT * RawaccelISUNEDMPU.y + CTmultCS * RawaccelISUNEDMPU.z ;
			accelNEDBODYz.ABupdate( dtGyr, accelISUNEDBODY.z );
			accelNEDBODYzNorm = -accelNEDBODYz.ABfilt() / GRAVITY;
			//xSemaphoreGive( dataMutex );
						
		}

		// compute acceleration module variation using unfiltered accels
		// update accel module filter
		AccelModule.ABupdate(dtGyr, sqrt(RawaccelISUNEDMPU.x * RawaccelISUNEDMPU.x + RawaccelISUNEDMPU.y * RawaccelISUNEDMPU.y + RawaccelISUNEDMPU.z * RawaccelISUNEDMPU.z ) );
		// asysmetric filter with fast raise and slow decay
		#define fcAccelLevel 3.0 // 3Hz low pass to filter 
		#define fcAL1 (40.0/(40.0+fcAccelLevel))
		#define fcAL2 (1.0-fcAL1)		
		if ( AccelModulePrimLevel < abs(AccelModule.ABprim()) ) {
			AccelModulePrimLevel = abs(AccelModule.ABprim());
		} else {
			AccelModulePrimLevel = fcAL1 * AccelModulePrimLevel +  fcAL2 * abs(AccelModule.ABprim());
		}
		// Adjust Accel A/B filter N value in function of accel module prim level
		// asysmetric filter with fast rise ~0.5 second and slow decay ~3 seconds period
		#define fcNRAccel 10.0 // 10.0Hz rise low pass to filter 
		#define fcNRA1 (40.0/(40.0+fcNRAccel))
		#define fcNRA2 (1.0-fcNRA1)
		#define fcNDAccel 1.0 // 1.0Hz decay low pass to filter 
		#define fcNDA1 (40.0/(40.0+fcNDAccel))
		#define fcNDA2 (1.0-fcNDA1)
		float NAccelupdt = AccelModulePrimLevel / 3.0;
		if (NAccelupdt < 6.0 ) NAccelupdt = 6.0;
		if (NAccelupdt > 30.0 ) NAccelupdt = 30.0;		
		if ( NAccel < NAccelupdt ) {
			NAccel = fcNRA1 * NAccel +  fcNRA2 * NAccelupdt;
		} else {
			NAccel = fcNDA1 * NAccel +  fcNDA2 * NAccelupdt;
		}		
		accelNEDBODYx.ABNupdt( NAccel );
		accelNEDBODYy.ABNupdt( NAccel );	
		accelNEDBODYz.ABNupdt( NAccel );		
		
		// compute gyro module variation using unfiltered gyros
		// update gyro module filter
		GyroModule.ABupdate( dtGyr, sqrt(gyroISUNEDMPU.x * gyroISUNEDMPU.x + gyroISUNEDMPU.y * gyroISUNEDMPU.y + gyroISUNEDMPU.z * gyroISUNEDMPU.z ) );
		// asymetric filter with fast raise and slow decay
		#define fcGyroLevel 3.0 // 3Hz low pass to filter 
		#define fcGL1 (40.0/(40.0+fcGyroLevel))
		#define fcGL2 (1.0-fcGL1)		
		if ( GyroModulePrimLevel < abs(GyroModule.ABprim()) ) {
			GyroModulePrimLevel = abs(GyroModule.ABprim());
		} else {
			GyroModulePrimLevel = fcGL1 * GyroModulePrimLevel +  fcGL2 * abs(GyroModule.ABprim());
		}
		
		// get raw static pressure
		bool ok=false;
		float p = 0;
		Prevp = statP.Get();
		p = baroSensor->readPressure(ok);
		if ( ok ) {			
			prevstatTime = statTime;
			statTime = esp_timer_get_time()/1000; // record static time in milli second
			dtStat = (statTime - prevstatTime) / 1000.0; // period between last two valid static pressure samples in second	
			if (dtStat == 0) dtStat = PERIOD40HZ;
			statP.Set( p );
			baroP = p;	// for compatibility with Eckhard code
			Prevp = p;
		} else {
			statP.Set( Prevp );
			baroP = Prevp;
		}
		
		// get raw te pressure
		p = teSensor->readPressure(ok);
		if ( ok ) {
			teP.Set( p );
		}
		
		// get raw dynamic pressure
		if( asSensor ) {
			PrevdynP = dynP.Get();
			dp =  asSensor->readPascal(0, ok);
		}
		if( ok ) {			
			prevdynPTime = dynPTime;
			dynPTime = esp_timer_get_time()/1000.0; // record dynPTimeTE time in milli second		
			dtdynP = (dynPTime - prevdynPTime) / 1000.0; // period between last two valid dynamic pressure samples in second
			if (dtdynP == 0) dtdynP = PERIOD40HZ;
			dynP.Set( dp );
		}
		else {
			dynamicP = PrevdynP;
			dynP.Set( PrevdynP );
		}
		if ( dynP.Get() <= 0.0 ) dynP.Set( 0.0 );
		dynamicP = dynP.Get(); // for compatibility with Eckhard code
		if ( dynamicP < 60.0 ) dynamicP = 0.0;

		// update CAS filter
		CAS.ABupdate( dtdynP, sqrt(2 * dynP.Get() / RhoSLISA) );
		
		// update TAS filter
		Rhocorr = sqrt(RhoSLISA/Rho);
		TAS.Set( Rhocorr * CAS.ABfilt() );

		// update altitude filter
		ALT.ABupdate( dtStat, (1.0 - pow( (statP.Get()-(QNH.get()-1013.25)) * 0.000986923 , 0.1902891634 ) ) * (273.15 + 15) * 153.846153846 );
		
		// Initialize ALTbi
		if (ALTbiFirstPass) {
			ALTbi = ALT.ABfilt();
			ALTbiFirstPass = false;
		}	
		// update Vz baro
		// in glider operation, gaining altitude and energy is considered positive. However in NED representation vertical axis is positive pointing down.
		// therefore Vzbaro in NED is the opposite of altitude variation.
		Vzbaro = -ALT.ABprim();
		

		// attitude initialization when XCVario starts during first 100 iterations 
		if ( AttitudeInit <= 100 ) { // initialize quaternions at xcvario start using first 100 samples
			if ( AttitudeInit >= 50 ) { // skip first 49 samples before calculating attitude from accels
				if ( AttitudeInit == 50 ) {
					RollInit = atan2(-accelNEDBODYy.ABfilt(), -accelNEDBODYz.ABfilt());
					PitchInit = asin( accelNEDBODYx.ABfilt()/ sqrt(accelNEDBODYx.ABfilt()* accelNEDBODYx.ABfilt()+ accelNEDBODYy.ABfilt()* accelNEDBODYy.ABfilt()+ accelNEDBODYz.ABfilt() * accelNEDBODYz.ABfilt() ));
					YawInit   = 0.0;
				} else {
					RollInit = RollInit + atan2(-accelNEDBODYy.ABfilt(), -accelNEDBODYz.ABfilt());
					PitchInit = PitchInit + asin( accelNEDBODYx.ABfilt()/ sqrt(accelNEDBODYx.ABfilt()* accelNEDBODYx.ABfilt()+ accelNEDBODYy.ABfilt()* accelNEDBODYy.ABfilt()+ accelNEDBODYz.ABfilt() * accelNEDBODYz.ABfilt() ));
					if ( AttitudeInit == 100 ) {
						RollInit = RollInit / 50.0;
						PitchInit = PitchInit / 50.0;
						q0=((cos(RollInit/2.0)*cos(PitchInit/2.0)*cos(YawInit/2.0)+sin(RollInit/2.0)*sin(PitchInit/2.0)*sin(YawInit/2.0)));
						q1=((sin(RollInit/2.0)*cos(PitchInit/2.0)*cos(YawInit/2.0)-cos(RollInit/2.0)*sin(PitchInit/2.0)*sin(YawInit/2.0)));
						q2=((cos(RollInit/2.0)*sin(PitchInit/2.0)*cos(YawInit/2.0)+sin(RollInit/2.0)*cos(PitchInit/2.0)*sin(YawInit/2.0)));
						q3=((cos(RollInit/2.0)*cos(PitchInit/2.0)*sin(YawInit/2.0)-sin(RollInit/2.0)*sin(PitchInit/2.0)*cos(YawInit/2.0)));						
					}
				}					
			}
			AttitudeInit++;			
		} else { // after xcvario is started and quaternion attitude initialized perform normal calculations
			if (!CALstream) { // if not in calibration mode MOD#8
				if ( TAS.Get() > 15.0 ) {	// Update IMU, only consider centrifugal forces if TAS > 15 m/s
					// estimate gravity in body frame taking into account centrifugal corrections
					//xSemaphoreTake( dataMutex, 3/portTICK_PERIOD_MS ); // prevent data conflicts for 3ms max.
					gravISUNEDBODY.x = accelNEDBODYx.ABfilt()- gyroCorr.y * TASbi.Get() * AoA.Get() + gyroCorr.z * TASbi.Get() * AoB.Get() - UbiPrim;
					gravISUNEDBODY.y = accelNEDBODYy.ABfilt()- gyroCorr.z * TASbi.Get() + gyroCorr.x * TASbi.Get() * AoA.Get() - VbiPrim;
					gravISUNEDBODY.z = accelNEDBODYz.ABfilt()+ gyroCorr.y * TASbi.Get() - gyroCorr.x * TASbi.Get() * AoB.Get() - WbiPrim;
					//xSemaphoreGive( dataMutex );
				} else {
					// estimate gravity in body frame using accels only
					gravISUNEDBODY.x = accelNEDBODYx.ABfilt();
					gravISUNEDBODY.y = accelNEDBODYy.ABfilt();
					gravISUNEDBODY.z = accelNEDBODYz.ABfilt();
				}

				// Update quaternions
				if ( Mahonykp != 0.0 ) {
					MahonyUpdateIMU( dtGyr, gyroCorr.x, gyroCorr.y, gyroCorr.z, -gravISUNEDBODY.x, -gravISUNEDBODY.y, -gravISUNEDBODY.z, GravityModule );								
				} else {
					// Compute dynamic Beta
					if ( TAS.Get() > 15.0 ) {	// Compute dynamic Beta only when centrifugal forces are active, TAS > 15 m/s
						#define BetaRollMax 0.12 // Roll max to consider Beta increase 0.012 rad ~7°
						#define BetaRollx10 0.0 // Roll at which Beta is 10 times MagdwickBeta
						#define BetaGain  (BetaRollMax - BetaRollx10)
						#define MaxGravityError 0.12					
						GravModuleLP = 0.9 * GravModuleLP  + 0.1 * abs( GravityModule - GRAVITY );					
						if ( (RollModuleLevel < BetaRollMax) && ( GravModuleLP < MaxGravityError ) ) {
							CurrentBeta = MagdwickBeta * pow( 10.0, (BetaRollMax - RollModuleLevel) / BetaGain );
						} else {
							CurrentBeta = MagdwickBeta;
						}
					} else {
						// When static use higher Beta = 0.003 to let AHRS synchronize with accels attitude faster
						CurrentBeta = 0.003;
					}
					MagdwickUpdateIMU( dtGyr, CurrentBeta, gyroCorr.x, gyroCorr.y, gyroCorr.z, -gravISUNEDBODY.x, -gravISUNEDBODY.y, -gravISUNEDBODY.z, GravityModule );
				}	

				// Compute Euler angles from IMU quaternion
				if ( abs(q1 * q3 - q0 * q2) < 0.5 ) {
					Pitch = asin(-2.0 * (q1 * q3 - q0 * q2));
				} else {
					Pitch = M_PI / 2.0 * signbit((q0 * q2 - q1 * q3 ));
				}
				Roll = atan2((q0 * q1 + q2 * q3), (0.5 - q1 * q1 - q2 * q2));
				Yaw = atan2(2.0 * (q1 * q2 + q0 * q3), (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3));
				if (Yaw < 0.0 ) Yaw = Yaw + 2.0 * M_PI;
				if (Yaw > 2.0 * M_PI) Yaw = Yaw - 2.0 * M_PI;

				// compute & filter RollModule
				RollModule = 0.8 * RollModule + 0.2 * abs( atan2(-gravISUNEDBODY.y, -gravISUNEDBODY.z) );
				// asysmetric filter with fast raise and slow decay
				#define fcRollModuleDn 1.0 // 1Hz low pass slow decay to filter noise and fast pilot roll variations
				#define fcRMD1 (40.0/(40.0+fcRollModuleDn))
				#define fcRMD2 (1.0-fcRMD1)	
				#define fcRollModuleUp 3.0 // 3.0Hz low pass fast decay
				#define fcRMU1 (40.0/(40.0+fcRollModuleUp))
				#define fcRMU2 (1.0-fcRMU1)					
				if ( RollModuleLevel < RollModule ) {
					RollModuleLevel = fcRMU1 * RollModuleLevel +  fcRMU2 * RollModule;
				} else {
					RollModuleLevel = fcRMD1 * RollModuleLevel +  fcRMD2 * RollModule;
				}				
				
				// Update roll and pitch alpha beta filter. This provides Roll and Pitch derivatives for bias analysis 
				RollAHRS.ABupdate( dtGyr, Roll );// MOD#4 gyro bias
				PitchAHRS.ABupdate( dtGyr, Pitch );// MOD#4 gyro bias
				
				#ifdef COMPUTEBIAS
				// gyro bias estimation when using Magdwick i.e. Mahonykp = 0.0
				// When TAS below 15 m/s and roll and pitch are below respective thresholds, 
				// x and y gyros biases are computed by long term average of the gyro values minus corresponding axis attitude variation
				// z gyro bias is computed by long term average of gyro value minus heading variation estimation
				#define RollLimit 0.175 // max lateral gravity acceleration (normalized acceleration) for 10° roll
				#define PitchLimit 0.175 // max longitudinal gravity acceleration (normalized acceleration) for 10° pitch
				#define HeadingPrimLimit 0.175 // max heading variation to avoid outliers,0.175 rad/s ~ 10°/s
				#define GMaxBias 0.005 // limit biais correction to 5 mrad/s
				if ( (TAS.Get() > 15.0) && (abs(Roll) < RollLimit)  && (abs(Pitch) < PitchLimit) ) {
					// When there is no outliers from d(roll)/dt and d(pitch)/dt, compute bias as Gx - d(roll)/dt and Gy - d(pitch)/dt long term average.
					RollPrim = RollAHRS.ABprim();
					if ( abs(RollPrim) < RollLimit ) GyroBiasx.LPupdate( gyroCorr.x - RollPrim );
					PitchPrim = PitchAHRS.ABprim();
					if ( abs(PitchPrim) < PitchLimit ) GyroBiasy.LPupdate( gyroCorr.y - PitchPrim );					
					// compute pseudo heading from GNSS
					GnssTrack = atan2( GnssVy.ABfilt(), GnssVx.ABfilt() );
					PseudoHeadingPrim = ( GnssVy.ABprim() * cos(GnssTrack) - GnssVx.ABprim() * sin(GnssTrack) ) / TASbi.Get();
					// compute Gz - pseudo heading variation long term average.		
					if ( abs(PseudoHeadingPrim) < HeadingPrimLimit ) GyroBiasz.LPupdate( gyroCorr.z - PseudoHeadingPrim );
					// update gyros biases variables
					BiasQuatGx = GyroBiasx.LowPass2();
					BiasQuatGy = GyroBiasy.LowPass2();
					BiasQuatGz = GyroBiasz.LowPass2();		
					// limit bias estimation	
					if ( abs(BiasQuatGx) > GMaxBias ) BiasQuatGx = copysign( GMaxBias, BiasQuatGx);
					if ( abs(BiasQuatGy) > GMaxBias ) BiasQuatGy = copysign( GMaxBias, BiasQuatGy);		
					if ( abs(BiasQuatGz) > GMaxBias ) BiasQuatGz = copysign( GMaxBias, BiasQuatGz);		
				}
				#endif		
				
				// compute sin and cos for Roll and Pitch from IMU quaternion since they are used in multiple calculations
				cosRoll = cos( Roll );
				sinRoll = sin( Roll );
				cosPitch = cos( Pitch );
				sinPitch = sin( Pitch );

				// compute kinetic acceleration from accels, gravity from IMU and centrifugal forces from accels and baro inertial speeds
				// compute gravity estimation using IMU quaternion
				GravIMU.x = -GRAVITY * 2.0 * (q1 * q3 - q0 * q2);
				GravIMU.y = -GRAVITY * 2.0 * (q2 * q3 + q0 * q1);
				GravIMU.z = -GRAVITY * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
				
				// compute kinetic accelerations using accelerations, corrected with gravity and centrifugal accels
				UiPrim = accelNEDBODYx.ABfilt()- GravIMU.x - gyroCorr.y * TASbi.Get() * AoA.Get() + gyroCorr.z * TASbi.Get() * AoB.Get();
				ViPrim = accelNEDBODYy.ABfilt()- GravIMU.y - gyroCorr.z * TASbi.Get() + gyroCorr.x * TASbi.Get() * AoA.Get();			
				WiPrim = accelNEDBODYz.ABfilt()- GravIMU.z + gyroCorr.y * TASbi.Get() - gyroCorr.x * TASbi.Get() * AoB.Get();
				
				// alternate kinetic accel solution using 3D baro inertial speeds
				// UiPrim = accelNEDBODYx.ABfilt()- GravIMU.x - gyroCorr.y * Wbi + gyroCorr.z * Vbi;
				// ViPrim = accelNEDBODYy.ABfilt()- GravIMU.y - gyroCorr.z * Ubi + gyroCorr.x * Wbi;			
				// WiPrim = accelNEDBODYz.ABfilt()- GravIMU.z + gyroCorr.y * Ubi - gyroCorr.x * Vbi;			

				// Kinectic accels alpha/beta short filter
				UiPrimF.ABupdate( dtGyr, UiPrim );
				ViPrimF.ABupdate( dtGyr, ViPrim );
				WiPrimF.ABupdate( dtGyr, WiPrim );

				// Compute baro interial acceleration in body frame
				// Compute dynamic period for baro inertiel filter
				#define PeriodVelbiGain 2.0
				#define GyrAmplitudeLimit 0.4
				// Gyro x and z amplitude used to adjust Baro Inertial filter
				GyrxzAmplitudeBIdyn = abs(gyroCorr.x)*0.9 + abs(gyroCorr.z)*0.4;			
				if ( GyrxzAmplitudeBIdyn < GyrAmplitudeLimit ) {
					//DynPeriodVelbi = 0.95 * DynPeriodVelbi + 0.05 * PeriodVelbi / ( 1 + GyrxzAmplitudeBIdyn / (GyrAmplitudeLimit/PeriodVelbiGain) );
					DynPeriodVelbi = 0.95 * DynPeriodVelbi + 0.05 * PeriodVelbi / ( 1 + GyrxzAmplitudeBIdyn * (PeriodVelbiGain - 1.0) / GyrAmplitudeLimit );					
				} else {
					DynPeriodVelbi = 0.95 * DynPeriodVelbi + 0.05 * PeriodVelbi / PeriodVelbiGain;
				}
				fcVelbi1 = ( DynPeriodVelbi / ( DynPeriodVelbi + dtGyr ));
				fcVelbi2 = ( 1.0 - fcVelbi1 );
				#define VelbiLow 4.0
				DynPeriodVelbiLow = DynPeriodVelbi / VelbiLow;
				fcVelbiLow1 = ( DynPeriodVelbiLow / ( DynPeriodVelbiLow + dtGyr ));
				fcVelbiLow2 = ( 1.0 - fcVelbi1 );				
				
				// Compute baro interial acceleration ( complementary filter between inertial accel derivatives and baro accels )
				//xSemaphoreTake( dataMutex, 3/portTICK_PERIOD_MS ); // prevent data conflicts for 3ms max.			
				UbiPrim = fcVelbi1 * ( UbiPrim + UiPrimF.ABprim() * dtGyr ) + fcVelbi2 * UbPrimS;
				if( opt_TE == 1 ) {
					VbiPrim = fcVelbiLow1 * ( VbiPrim + ViPrimF.ABprim() * dtGyr ) + fcVelbiLow2 * VbPrimS;			
					WbiPrim = fcVelbiLow1 * ( WbiPrim + WiPrimF.ABprim() * dtGyr ) + fcVelbiLow2 * WbPrimS;					
				} else {
					VbiPrim = fcVelbi1 * ( VbiPrim + ViPrimF.ABprim() * dtGyr ) + fcVelbi2 * VbPrimS;			
					WbiPrim = fcVelbi1 * ( WbiPrim + WiPrimF.ABprim() * dtGyr ) + fcVelbi2 * WbPrimS;
				}
				
				// Compute baro interial velocity ( complementary filter between baro inertial acceleration and baro speed )
				Ubi = fcVelbi1 * ( Ubi + UbiPrim * dtGyr ) + fcVelbi2 * Ub;
				if( opt_TE == 1 ) {
					Vbi = fcVelbiLow1 * ( Vbi + VbiPrim * dtGyr ) + fcVelbiLow2 * Vb;
					Wbi = fcVelbiLow1 * ( Wbi + WbiPrim * dtGyr ) + fcVelbiLow2 * Wb;
					
				} else {
					Vbi = Vb;
					Wbi = fcVelbi1 * ( Wbi + WbiPrim * dtGyr ) + fcVelbi2 * Wb;
				}				

				// baro inertial TAS & TAS square in any frame
				// TASbiSquare = Ubi * Ubi + Vbi * Vbi + Wbi * Wbi;
				if ( opt_TE == 1 ) {
					TASbiSquare = Ubi * Ubi + Vbi * Vbi + Wbi * Wbi;
				} else {
					TASbiSquare = Ubi * Ubi + Wbi * Wbi; // TODO removed Vbi from calculation due to unexplained high Vbi variations					
				}
				TASbi.Set( sqrt( TASbiSquare ) );
				
				//xSemaphoreGive( dataMutex );
								
			}
			
			// Gyro bias and local gravity estimates when TAS < 15 m/s and the vario is considered potentially stable on ground
			if ( TAS.Get() < 15.0 ) {
				// Estimate gyro bias and gravity, except if doing Lab test then only one estimation is performed
				if ( (!LABtest) || (BIAS_Init < 1 ) ) {
					// When MPU temperature is controled and temperature is locked   or   when there is no temperature control
					if ( (HAS_MPU_TEMP_CONTROL && (MPU.getSiliconTempStatus() == MPU_T_LOCKED)) || !HAS_MPU_TEMP_CONTROL ) {
						// count cycles when temperature is locked
						gyrobiastemptimer++;
						// detect if gyro and accel variations is below stability threshold using an alpha/beta filter to estimate variation over short period of time
						// if temperature conditions has been stable for more than 30 seconds (1200 = 30x40hz) and there is very little angular and acceleration variation
						if ( (gyrobiastemptimer > 1200) && (GyroModulePrimLevel < GroundGyroprimlimit) && (AccelModulePrimLevel < GroundAccelprimlimit) ) {
							gyrostable++;
							// during first 2.5 seconds, initialize gyro and gravity data
							if ( gyrostable < 100 ) {
								GxBias = gyroRPS.x;
								GyBias = gyroRPS.y;
								GzBias = gyroRPS.z;							
								Gravx = RawaccelISUNEDMPU.x;
								Gravy = RawaccelISUNEDMPU.y;
								Gravz = RawaccelISUNEDMPU.z;
								averagecount = 1;
							} else {
								// between 2.5 seconds and 22.5 seconds, accumulate gyro and gravity data
								if ( gyrostable <900 ) {
									GxBias += gyroRPS.x;
									GyBias += gyroRPS.y;
									GzBias += gyroRPS.z;
									Gravx += RawaccelISUNEDMPU.x;
									Gravy += RawaccelISUNEDMPU.y;
									Gravz += RawaccelISUNEDMPU.z;
									averagecount++;
								} else {
									// wait for 2.5 seconds (900 to 1000) before updating bias and gravity (just in case glider starts to move, discard previous bias/gravity assesment)
									// then compute updated bias/gravity
									if ( gyrostable > 1000 ) {
										NewGroundGyroBias.x = GxBias / averagecount;
										NewGroundGyroBias.y = GyBias / averagecount;
										NewGroundGyroBias.z = GzBias / averagecount;
										Gravx /= averagecount;
										Gravy /= averagecount;
										Gravz /= averagecount;
										GRAVITY = sqrt(Gravx*Gravx+Gravy*Gravy+Gravz*Gravz);
										AccelGravModuleFilt = GRAVITY;
										BIAS_Init++;
										//sprintf(str,"$BIAS,%lld,%.6f,%.6f,%.6f,%.6f\r\n", gyroTime, NewGroundGyroBias.z, NewGroundGyroBias.y, NewGroundGyroBias.x, GRAVITY );					
										//Router::sendXCV(str);										
										// Store bias and gravity in non volatile memory if:
										// - they have been identified for the first time
										// - or every 10 times up to the first 100 bias identifications 
										if ( BIAS_Init == 1 || ( (BIAS_Init < 100) && (BIAS_Init % 10 ) ) ) {
											if ( MagdwickBeta != 0.0 || Mahonykp != 0.0 ) {											
												gyro_bias.set(NewGroundGyroBias);
												gravity.set(GRAVITY);
											}
										}								
										gyrostable = 0; // reset stability counter when bias/gravity have been obtained
									}
								} 
							}
							//sprintf(str,"gyrostable = %d, average count = %d, number of bias estimates = %d \r\n", gyrostable, averagecount, BIAS_Init );					
							//Router::sendXCV(str);	
						} else {
							gyrostable = 0; // reset gyro stability counter if temperature not stable or movement detected
						}
					}
				}
				// Adjust progressively ground gyro bias with new ground bias estimate
				if ( MagdwickBeta != 0.0 || Mahonykp != 0.0 ) {
					if ( abs(GroundGyroBias.x - NewGroundGyroBias.x) > 0.00001 ) GroundGyroBias.x = 0.995 * GroundGyroBias.x + 0.005 * NewGroundGyroBias.x;
					if ( abs(GroundGyroBias.y - NewGroundGyroBias.y) > 0.00001 ) GroundGyroBias.y = 0.995 * GroundGyroBias.y + 0.005 * NewGroundGyroBias.y;	
					if ( abs(GroundGyroBias.z - NewGroundGyroBias.z) > 0.00001 ) GroundGyroBias.z = 0.995 * GroundGyroBias.z + 0.005 * NewGroundGyroBias.z;			
				} 				
				// Only for laboratory calibration of the accelerometers
				// stream accel data and compute offsts/gains
				if ( CALstream ) {
					if ( CALfirstpass ) {
						accelMaxx = 0.0;
						accelMinx = 0.0;
						accelMaxy = 0.0;
						accelMiny = 0.0;
						accelMaxz = 0.0;
						accelMinz = 0.0;
						gyromodulestable = 8;						
						CALfirstpass = false;
					}
					// If gyro are stable
					if ( GyroModulePrimLevel < GroundGyroprimlimit  &&  AccelModulePrimLevel < GroundAccelprimlimit) { //MOD#6 improve calibration process
						if ( gyromodulestable > 5 ) gyromodulestable--;  
						if ( gyromodulestable == 5 ) {
							accelAvgx = -accelG.z*9.807;
							accelAvgy = -accelG.y*9.807;
							accelAvgz = -accelG.x*9.807;
							gyromodulestable = 4;
						}
						if ( gyromodulestable < 5 ) {
							accelAvgx = 0.8 * accelAvgx + 0.2 * (-accelG.z*9.807);
							accelAvgy = 0.8 * accelAvgy + 0.2 * (-accelG.y*9.807);
							accelAvgz = 0.8 * accelAvgz + 0.2 * (-accelG.x*9.807);
							if ( gyromodulestable > 1 ) gyromodulestable--;
						}					
						// store max and min with a short ~10 Hz low pass // MOD#9
						if ( gyromodulestable == 1 ) {
							if ( accelAvgx > accelMaxx ) accelMaxx = accelMaxx*0.5 + accelAvgx*0.5;
							if ( accelAvgy > accelMaxy ) accelMaxy = accelMaxy*0.5 + accelAvgy*0.5;
							if ( accelAvgz > accelMaxz ) accelMaxz = accelMaxz*0.5 + accelAvgz*0.5;
							if ( accelAvgx < accelMinx ) accelMinx = accelMinx*0.5 + accelAvgx*0.5;
							if ( accelAvgy < accelMiny ) accelMiny = accelMiny*0.5 + accelAvgy*0.5;
							if ( accelAvgz < accelMinz ) accelMinz = accelMinz*0.5 + accelAvgz*0.5;
						}
					} else {
						gyromodulestable = 8;
					}
					/*
					CAL data
					$CAL,
					time in ms,
					Gyro x in rad/s,
					Gyro y in rad/s,
					Gyro z in rad/s,
					Gyromodule prim in unit,
					Gyro module prim limit in unit,
					Acceleration in X-Axis in m/s²,
					Acceleration max in X-Axis in m/s²,
					Acceleration min in X-Axis in m/s²,				
					Acceleration in Y-Axis in m/s²,
					Acceleration max in Y-Axis in m/s²,
					Acceleration min in Y-Axis in  m/s²,				
					Acceleration in Z-Axis in m/s²,
					Acceleration max in Z-Axis in m/s²,
					Acceleration min in Z-Axis in m/s²,
					Acceleration bias x in m/s²,
					Acceleration bias y in m/s²
					Acceleration bias z in m/s²	
					Acceleration gain x in m/s²,
					Acceleration gain y in m/s²
					Acceleration gain z in m/s²				
					<CR><LF>	
					*/
					if ( gyromodulestable == 1 ) {
						if( gflags.gload_alarm ) {
							Audio::alarm( false );
							gflags.gload_alarm = false;
						}					
						sprintf(str,"$CAL,%lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
							gyroTime, gyroCorr.x, gyroCorr.y, gyroCorr.z, GyroModulePrimLevel, GroundGyroprimlimit,
							accelAvgx, accelMaxx, accelMinx, accelAvgy, accelMaxy, accelMiny, accelAvgz, accelMaxz, accelMinz,
							(accelMaxx+accelMinx)/2, (accelMaxy+accelMiny)/2, (accelMaxz+accelMinz)/2,
							localGravity /((accelMaxx-accelMinx)/2), localGravity /((accelMaxy-accelMiny)/2), localGravity/((accelMaxz-accelMinz)/2) );
					} else {
						if( !gflags.gload_alarm ) {
							Audio::alarm( true );
							gflags.gload_alarm = true;
						}
						sprintf(str,"$CAL,%lld,%.4f,%.4f,%.4f,%.4f,%.4f, - , - , - , - , - , - , - , - , - ,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
							gyroTime, gyroCorr.x, gyroCorr.y, gyroCorr.z, GyroModulePrimLevel, GroundGyroprimlimit,
							(accelMaxx+accelMinx)/2, (accelMaxy+accelMiny)/2, (accelMaxz+accelMinz)/2,
							localGravity /((accelMaxx-accelMinx)/2), localGravity /((accelMaxy-accelMiny)/2), localGravity/((accelMaxz-accelMinz)/2) );					
					}
					Router::sendXCV(str);
				}	
			} else {
				// if moving ( TAS > 15 m/s )
				// if bias and gravtity have been estimated more than once, store last available bias and gravity in FLASH
				if ( BIAS_Init > 1  ) {
					gyro_bias.set(NewGroundGyroBias);
					gravity.set(GRAVITY);
				}
			}	
		}

			/*
				// Sent at 40Hz when IMUstream selected
				$I,
				MPU (gyro) time in milli second,
				Acceleration in BODY X-Axis in tenth milli m/s²,
				Acceleration in BODY Y-Axis in tenth milli m/s²,
				Acceleration in BODU Z-Axis in tenth milli m/s²,				
				Rotation BODY X-Axis in hundredth of milli rad/s,
				Rotation BODY Y-Axis in hundredth of milli rad/s,
				Rotation BODY Z-Axis in hundredth of milli rad/s,
				<CR><LF>	
			*/
			/*
				// Sent at 40Hz when AHRSstream selected
				$J,
				MPU (gyro) time in milli second,
				Acceleration in BODY X-Axis in tenth milli m/s²,
				Acceleration in BODY Y-Axis in tenth milli m/s²,
				Acceleration in BODU Z-Axis in tenth milli m/s²,				
				Rotation BODY X-Axis in hundredth of milli rad/s,
				Rotation BODY Y-Axis in hundredth of milli rad/s,
				Rotation BODY Z-Axis in hundredth of milli rad/s,
				UiPrim*10000.0, ViPrim*10000.0),WiPrim*10000.0,
				UbiPrim*10000.0, VbiPrim*10000.0, WbiPrim*10000.0,
				gravISUNEDBODY.x*10000.0, gravISUNEDBODY.y*10000.0, gravISUNEDBODY.z*10000.0,
				q0*100000.0, q1*100000.0,q2*100000.0,q3*100000.0			
				<CR><LF>	
			*/			
			/*
				// Sent at 1Hz when AHRSstream selected
				$K,
				MPU (gyro) time in milli second,
				Acceleration in BODY X-Axis in tenth milli m/s²,
				Acceleration in BODY Y-Axis in tenth milli m/s²,
				Acceleration in BODU Z-Axis in tenth milli m/s²,				
				Rotation BODY X-Axis in hundredth of milli rad/s,
				Rotation BODY Y-Axis in hundredth of milli rad/s,
				Rotation BODY Z-Axis in hundredth of milli rad/s,
				UiPrim*10000.0, ViPrim*10000.0),WiPrim*10000.0,
				UbiPrim*10000.0, VbiPrim*10000.0, WbiPrim*10000.0,
				gravISUNEDBODY.x*10000.0, gravISUNEDBODY.y*10000.0, gravISUNEDBODY.z*10000.0,
				q0*100000.0, q1*100000.0,q2*100000.0,q3*100000.0,
				accelNEDBODYx.ABfilt()*10000.0, accelNEDBODYy.ABfilt()*10000.0, accelNEDBODYz.ABfilt()*10000.0,
				accelNEDBODYx.ABprim()*10000.0, accelNEDBODYy.ABprim()*10000.0, accelNEDBODYz.ABprim()*10000.0,
				gyroNEDx.ABfilt()*100000.0, gyroNEDy.ABfilt()*100000.0, gyroNEDz.ABfilt()*100000.0,
				gyroNEDx.ABprim()*100000.0, gyroNEDy.ABprim()*100000.0, gyroNEDz.ABprim()*100000.0,
				UiPrimF.ABfilt()*10000.0, ViPrimF.ABfilt()*10000.0, WiPrimF.ABfilt()*10000.0),
				UiPrimF.ABprim()*10000.0, ViPrimF.ABprim()*10000.0, WiPrimF.ABprim()*10000.0)				
				<CR><LF>	
			*/			
			/*
				// Sent at 10Hz
				$A,
				CurrentBeta * 10000
				dynP * 10
				TAS * 100
				AoA * 1000
				AoB * 1000
				Speed2Fly.cw( CAS.ABfilt() ) * 10000
				Speed2Fly.getN() * 100
				WingLoad * 10
				fcVelbi1 * 10000 
				UbPrimS * 10000
				VbPrimS * 10000
				WbPrimS * 10000
				RTKNproj*1000
				RTKEproj*1000
				-RTKUproj*1000
				temperatureLP.LowPass1()*10.0
				statTime
				statP*100.0
				GnssVx.ABfilt()*100
				GnssVy.ABfilt()*100				
				GnssVz.ABfilt()*100				
				<CR><LF>				
			*/			
			
		if ( !(countIMU % 40) && AHRSstream ) {
			// Send $I and $A
			sprintf(str,"$K,%lld,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n$A,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%lld,%i,%i,%i,%i\r\n",
				gyroTime,
				(int32_t)(accelISUNEDBODY.x*10000.0), (int32_t)(accelISUNEDBODY.y*10000.0), (int32_t)(accelISUNEDBODY.z*10000.0),
				(int32_t)(gyroISUNEDBODY.x*100000.0), (int32_t)(gyroISUNEDBODY.y*100000.0),(int32_t)(gyroISUNEDBODY.z*100000.0),
				(int32_t)(UiPrim*10000.0), (int32_t)(ViPrim*10000.0),(int32_t)(WiPrim*10000.0),
				(int32_t)(UbiPrim*10000.0), (int32_t)(VbiPrim*10000.0),(int32_t)(WbiPrim*10000.0),
				(int32_t)(gravISUNEDBODY.x*10000.0), (int32_t)(gravISUNEDBODY.y*10000.0), (int32_t)(gravISUNEDBODY.z*10000.0),
				(int32_t)(q0*100000.0), (int32_t)(q1*100000.0),(int32_t)(q2*100000.0),(int32_t)(q3*100000.0),
				(int32_t)(accelNEDBODYx.ABfilt()*10000.0), (int32_t)(accelNEDBODYy.ABfilt()*10000.0), (int32_t)(accelNEDBODYz.ABfilt()*10000.0),
				(int32_t)(accelNEDBODYx.ABprim()*10000.0), (int32_t)(accelNEDBODYy.ABprim()*10000.0), (int32_t)(accelNEDBODYz.ABprim()*10000.0),
				(int32_t)(gyroNEDx.ABfilt()*100000.0), (int32_t)(gyroNEDy.ABfilt()*100000.0), (int32_t)(gyroNEDz.ABfilt()*100000.0),
				(int32_t)(gyroNEDx.ABprim()*100000.0), (int32_t)(gyroNEDy.ABprim()*100000.0), (int32_t)(gyroNEDz.ABprim()*100000.0),
				(int32_t)(UiPrimF.ABfilt()*10000.0), (int32_t)(ViPrimF.ABfilt()*10000.0), (int32_t)(WiPrimF.ABfilt()*10000.0),
				(int32_t)(UiPrimF.ABprim()*10000.0), (int32_t)(ViPrimF.ABprim()*10000.0), (int32_t)(WiPrimF.ABprim()*10000.0),
				(int32_t)(CurrentBeta*10000.0), (int32_t)(dynP.Get()*10.0),(int32_t)(TAS.Get()*100.0),(int32_t)(AoA.Get()*1000.0),(int32_t)(AoB.Get()*1000.0),
				(int32_t)(Speed2Fly.cw( CAS.ABfilt() )*10000.0), (int32_t)(Speed2Fly.getN()*100.0),(int32_t)(WingLoad*100.0),(int32_t)(fcVelbi1*10000.0),
				(int32_t)(UbPrimS*10000.0), (int32_t)(VbPrimS*10000.0),(int32_t)(WbPrimS*10000.0),
				(int32_t)(RTKNproj*1000),(int32_t)(RTKEproj*1000),(int32_t)(-RTKUproj*1000),
				(int32_t)(temperatureLP.LowPass1()*10.0), statTime, (int32_t)(statP.Get()*100.0), (int32_t)(GnssVx.ABfilt()*100), (int32_t)(GnssVy.ABfilt()*100), (int32_t)(GnssVz.ABfilt()*100)
				); 
			xSemaphoreTake( BTMutex, 2/portTICK_PERIOD_MS ); // prevent BT conflicts for 2ms max.
			Router::sendXCV(str);
			xSemaphoreGive( BTMutex );
		} else {
			if ( !(countIMU % 4) && AHRSstream  ) {
				// Send $J and $A
				sprintf(str,"$J,%lld,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n$A,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%lld,%i,%i,%i,%i\r\n",
					gyroTime,
					(int32_t)(accelISUNEDBODY.x*10000.0), (int32_t)(accelISUNEDBODY.y*10000.0), (int32_t)(accelISUNEDBODY.z*10000.0),
					(int32_t)(gyroISUNEDBODY.x*100000.0), (int32_t)(gyroISUNEDBODY.y*100000.0),(int32_t)(gyroISUNEDBODY.z*100000.0),
					(int32_t)(UiPrim*10000.0), (int32_t)(ViPrim*10000.0),(int32_t)(WiPrim*10000.0),
					(int32_t)(UbiPrim*10000.0), (int32_t)(VbiPrim*10000.0),(int32_t)(WbiPrim*10000.0),
					(int32_t)(gravISUNEDBODY.x*10000.0), (int32_t)(gravISUNEDBODY.y*10000.0), (int32_t)(gravISUNEDBODY.z*10000.0),
					(int32_t)(q0*100000.0), (int32_t)(q1*100000.0),(int32_t)(q2*100000.0),(int32_t)(q3*100000.0),
					(int32_t)(CurrentBeta*10000.0), (int32_t)(dynP.Get()*10.0),(int32_t)(TAS.Get()*100.0),(int32_t)(AoA.Get()*1000.0),(int32_t)(AoB.Get()*1000.0),
					(int32_t)(Speed2Fly.cw( CAS.ABfilt() )*10000.0), (int32_t)(Speed2Fly.getN()*100.0),(int32_t)(WingLoad*100.0),(int32_t)(fcVelbi1*10000.0),
					(int32_t)(UbPrimS*10000.0), (int32_t)(VbPrimS*10000.0),(int32_t)(WbPrimS*10000.0),
					(int32_t)(RTKNproj*1000),(int32_t)(RTKEproj*1000),(int32_t)(-RTKUproj*1000),
					(int32_t)(temperatureLP.LowPass1()*10.0), statTime, (int32_t)(statP.Get()*100.0), (int32_t)(GnssVx.ABfilt()*100), (int32_t)(GnssVy.ABfilt()*100), (int32_t)(GnssVz.ABfilt()*100)				
					); 
				xSemaphoreTake( BTMutex, 2/portTICK_PERIOD_MS ); // prevent BT conflicts for 2ms max.
				Router::sendXCV(str);
				xSemaphoreGive( BTMutex );
			} else {
				if ( AHRSstream ) {
					// Send $J
					sprintf(str,"$J,%lld,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n",
						gyroTime,
						(int32_t)(accelISUNEDBODY.x*10000.0), (int32_t)(accelISUNEDBODY.y*10000.0), (int32_t)(accelISUNEDBODY.z*10000.0),
						(int32_t)(gyroISUNEDBODY.x*100000.0), (int32_t)(gyroISUNEDBODY.y*100000.0),(int32_t)(gyroISUNEDBODY.z*100000.0),
						(int32_t)(UiPrim*10000.0), (int32_t)(ViPrim*10000.0),(int32_t)(WiPrim*10000.0),
						(int32_t)(UbiPrim*10000.0), (int32_t)(VbiPrim*10000.0),(int32_t)(WbiPrim*10000.0),
						(int32_t)(gravISUNEDBODY.x*10000.0), (int32_t)(gravISUNEDBODY.y*10000.0), (int32_t)(gravISUNEDBODY.z*10000.0),
						(int32_t)(q0*100000.0), (int32_t)(q1*100000.0),(int32_t)(q2*100000.0),(int32_t)(q3*100000.0)
						);
					xSemaphoreTake( BTMutex, 2/portTICK_PERIOD_MS ); // prevent BT conflicts for 2ms max.
					Router::sendXCV(str);
					xSemaphoreGive( BTMutex );						
				} else {
					if ( IMUstream ) {
					// Send $I
					sprintf(str,"$I,%lld,%i,%i,%i,%i,%i,%i,%lld,%i,%i,%i\r\n",
						gyroTime,
						(int32_t)(accelISUNEDBODY.x*10000.0), (int32_t)(accelISUNEDBODY.y*10000.0), (int32_t)(accelISUNEDBODY.z*10000.0),
						(int32_t)(gyroISUNEDBODY.x*100000.0), (int32_t)(gyroISUNEDBODY.y*100000.0),(int32_t)(gyroISUNEDBODY.z*100000.0),
						statTime, (int32_t)(statP.Get()*100.0),(int32_t)(teP.Get()*100.0), (int32_t)(dynP.Get()*10)						
					);						
					xSemaphoreTake( BTMutex, 2/portTICK_PERIOD_MS ); // prevent BT conflicts for 2ms max.
					Router::sendXCV(str);
					xSemaphoreGive( BTMutex );
					}
				}
			}				
		}
		
		if ( SENstream ) {
			/* Sensor data
				$S1,			
				GNSS time in milli second,
				GNSS speed x or north in centimeters/s,
				GNSS speed y or east in centimeters/s,
				GNSS speed z or down in centimeters/s,
				Pitch in milli rad,
				Roll in milli rad,
				Yaw in milli rad,
				Vzbaro in cm/s,
				AoA angle in mrad,
				AoB  angle in mrad,
				Ubi in cm/s,
				Vbi in cm/s,
				Wbi in cm/s,
				Vzbi in cm/s,			
				TotalEnergy in cm/s,
				CurrentBeta in tenthousand of unit,
				NAccel in ten of unit,
				DynPeriodVelbi in thousands of second
				<CR><LF>		
			*/
			/* 
				$S2,
				Outside Air Temperature in tenth of °C,
				MPU temperature in tenth °C,
				GNSS fix 0 to 6   3=3D   4= 3D diff  5= RTK Float  6 = RTK integer
				GNSS number of satelites used  or  RTK ratio * 10 
				New Ground Gyro bias z in hundredth of milli rad/s,
				New Ground Gyro bias y in hundredth of milli rad/s,
				New Ground Gyro bias x in hundredth of milli rad/s,			
				IMU Gyro bias x in hundredth of milli rad/s,
				IMU Gyro bias y in hundredth of milli rad/s,
				IMU Gyro bias z in hundredth of milli rad/s,
				XCVtemp (temperature inside vario) in tenth of °C,
				PeriodVelbi (Baro Inertial period in tenth of seconds),
				te_filt (TE filter period in tenth of second),
				Mahonykp in tenthousandth of unit,
				MagdwickBeta in tenthousandth of unit,
				ALTbiN ALTbi N A/B filter in tenth of unit,
				TASbiN delta between ALTbi and TASbi N in tenth of unit,
				opt_TE 1 or 2,
				BIAS_Init number of gyro bias estimates on ground,
				FTVERSION,
				SOFTVERSION
			*/	
			/* 
				$S3,
				UiPrim in hundred of m/s²,
				ViPrim,
				Wiprim,
				UbPrimS in hubdred of m/s²,
				VbPrimS,
				WbPrimS,
				UiPrimF.ABprim() in hundred of m/s3,
				ViPrimF.ABprim(),
				WiPrimF.ABprim(),			
				UbiPrim in hundred of m/s²,
				VbiPrim,
				WbiPrim,
				Bias_AoB in mrad
				RTKNproj in thousandths of meter;
				RTKEproj in thousandths of meter;
				RTKDproj in thousandths of meter;
				RTKheading in tenth of degre;
				ALTbi in cm,
				DHeading in mrad,
				UbFS in cm/s,
				VbFS in cm/s,
				WbFS in cm/s,
				AccelModulePrimLevel in hundredth of m/s3,
				GyroModulePrimLevel  in hundredth of m/s3,
				Event Event counter in unit
				Vb in cm/s
				PseudoHeadingPrim in hundredth of milli rad/s,			
			*/				
			if ( SEN50DataReady ) {
				SEN50DataReady = false;
				// send $S1 and $S2 every 50 cycles = 5 seconds
				sprintf(str,"$S1,%lld,%i,%i,%i,%lld,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n$S3,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n$S2,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n",				
					// $S1 stream
					statTime, (int32_t)(statP.Get()*100.0),(int32_t)(teP.Get()*100.0), (int16_t)(dynP.Get()*10), 
					(int64_t)(chosenGnss->time*1000.0), (int16_t)(chosenGnss->speed.x*100), (int16_t)(chosenGnss->speed.y*100), (int16_t)(chosenGnss->speed.z*100),
					(int32_t)(Pitch*1000.0), (int32_t)(Roll*1000.0), (int32_t)(Yaw*1000.0),
					(int32_t)(Vzbaro*100),
					(int32_t)(AoA.Get()*1000), (int32_t)(AoB.Get()*1000),
					(int32_t)(Ubi*100), (int32_t)(Vbi*100),(int32_t)(Wbi*100), (int32_t)(Vzbi*100),				
					(int32_t)(Vztotbi.Get()*100),
					(int32_t)(CurrentBeta*10000), 
					(int32_t)(NAccel * 10),
					(int32_t)(DynPeriodVelbi*1000),
					// $S3 stream
					(int32_t)(UiPrim*100),(int32_t)(ViPrim*100),(int32_t)(WiPrim*100),
					(int32_t)(UbPrimS*100), (int32_t)(VbPrimS*100),(int32_t)(WbPrimS*100),
					(int32_t)(UiPrimF.ABprim()*100), (int32_t)(ViPrimF.ABprim()*100),(int32_t)(WiPrimF.ABprim()*100),	
					(int32_t)(UbiPrim*100), (int32_t)(VbiPrim*100),(int32_t)(WbiPrim*100),
					(int32_t)(Bias_AoB*1000),
					(int32_t)(RTKNproj*1000),(int32_t)(RTKEproj*1000),(int32_t)(-RTKUproj*1000),(int32_t)(RTKheading*10),(int32_t)(ALTbi*100),
					(int32_t)(DHeading*1000),(int32_t)(UbFS*100),(int32_t)(VbFS*100),(int32_t)(WbFS*100),
					(int32_t)(AccelModulePrimLevel*100),(int32_t)(GyroModulePrimLevel*100), (int32_t)(Event), (int32_t)(Vb*100),
					(int32_t)(PseudoHeadingPrim*100000),
					// $S2 stream
					(int16_t)(temperatureLP.LowPass1()*10.0), (int16_t)(MPUtempcel*10.0), chosenGnss->fix, chosenGnss->numSV,
					(int32_t)(NewGroundGyroBias.x*100000.0), (int32_t)(NewGroundGyroBias.y*100000.0), (int32_t)(NewGroundGyroBias.z*100000.0),				
					(int32_t)(BiasQuatGx*100000.0), (int32_t)(BiasQuatGy*100000.0), (int32_t)(BiasQuatGz*100000.0),
					(int16_t)(XCVTemp*10.0), (int16_t) (PeriodVelbi*10),
					(int32_t)(te_filt.get()*10),(int32_t)(Mahonykp*10000),(int32_t)(MagdwickBeta*10000), (int32_t)(ALTbiN*10), (int32_t)(TASbiN*10), (int32_t)(opt_TE),
					(int32_t)BIAS_Init,
					(int32_t)(FTVERSION),(int32_t)(SOFTVERSION)
					);
				xSemaphoreTake( BTMutex, 2/portTICK_PERIOD_MS );				
				Router::sendXCV(str);
				xSemaphoreGive( BTMutex );
			} else {
				if ( SENDataReady ) {
					SENDataReady = false;
					// send $S1 only every 100ms
					sprintf(str,"$S1,%lld,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n$S3,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\r\n",
						(int64_t)(chosenGnss->time*1000.0), (int16_t)(chosenGnss->speed.x*100), (int16_t)(chosenGnss->speed.y*100), (int16_t)(chosenGnss->speed.z*100),
						(int32_t)(Pitch*1000.0), (int32_t)(Roll*1000.0), (int32_t)(Yaw*1000.0),
						(int32_t)(Vzbaro*100),
						(int32_t)(AoA.Get()*1000), (int32_t)(AoB.Get()*1000),
						(int32_t)(Ubi*100), (int32_t)(Vbi*100),(int32_t)(Wbi*100), (int32_t)(Vzbi*100),				
						(int32_t)(Vztotbi.Get()*100),
						(int32_t)(CurrentBeta*10000), 
						(int32_t)NAccel * 10,
						(int32_t)(DynPeriodVelbi*1000),
						// $S3 stream
						(int32_t)(UiPrim*100),(int32_t)(ViPrim*100),(int32_t)(WiPrim*100),
						(int32_t)(UbPrimS*100), (int32_t)(VbPrimS*100),(int32_t)(WbPrimS*100),   
						(int32_t)(UiPrimF.ABprim()*100), (int32_t)(ViPrimF.ABprim()*100),(int32_t)(WiPrimF.ABprim()*100),	
						(int32_t)(UbiPrim*100), (int32_t)(VbiPrim*100),(int32_t)(WbiPrim*100),
						(int32_t)(Bias_AoB*1000),
						(int32_t)(RTKNproj*1000),(int32_t)(RTKEproj*1000),(int32_t)(-RTKUproj*1000),(int32_t)(RTKheading*10),(int32_t)(ALTbi*100),
						(int32_t)(DHeading*1000),(int32_t)(UbFS*100),(int32_t)(VbFS*100),(int32_t)(WbFS*100),
						(int32_t)(AccelModulePrimLevel*100),(int32_t)(GyroModulePrimLevel*100), (int32_t)(Event),(int32_t)(Vb*100),
						(int32_t)(PseudoHeadingPrim*100000)						
					);
					xSemaphoreTake( BTMutex, 2/portTICK_PERIOD_MS );				
					Router::sendXCV(str);
					xSemaphoreGive( BTMutex );
				}					
			}
		}		

		Router::routeXCV();
		
		ProcessTimeIMU = (esp_timer_get_time()/1000.0) - gyroTime;
		if ( ProcessTimeIMU > 8 && TAS.Get() < 15.0 ) {
			ESP_LOGI(FNAME,"processIMU: %i / 25", (int16_t)(ProcessTimeIMU) );
		}		
		mtick++;
		vTaskDelayUntil(&xLastWakeTime_mpu, 25/portTICK_PERIOD_MS);  // 25 ms = 40 Hz loop
		if( (mtick % 40) == 0) {  // test stack every second
			if( uxTaskGetStackHighWaterMark( mpid ) < 1024 && TAS.Get() < 15.0 )
				 ESP_LOGW(FNAME,"Warning MPU and sensor task stack low: %d bytes", uxTaskGetStackHighWaterMark( mpid ) );
		}
	}		
}			


static void lazyNvsCommit()
{
	uint16_t dummy;
	if ( xQueueReceive(SetupCommon::commitSema, &dummy, 0) ) {
		SetupCommon::commitNow();
	}
}

static void toyFeed()
{
	xSemaphoreTake(xMutex,portMAX_DELAY );
	// reduce also messages from 10 per second to 5 per second to reduce load in XCSoar
	// maybe just 1 or 2 per second
	static char lb[150];

	if( ahrs_rpyl_dataset.get() ){
		OV.sendNMEA( P_AHRS_RPYL, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature,
				-accelG[2], accelG[1],accelG[0], gyroDPS.x, gyroDPS.y, gyroDPS.z );
		OV.sendNMEA( P_AHRS_APENV1, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature,
				-accelG[2], accelG[1],accelG[0], gyroDPS.x, gyroDPS.y, gyroDPS.z );
	}
	if( nmea_protocol.get() == BORGELT ) {
		OV.sendNMEA( P_BORGELT, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altSTD, gflags.validTemperature  );
		OV.sendNMEA( P_GENERIC, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altSTD, gflags.validTemperature  );
	}
	else if( nmea_protocol.get() == OPENVARIO ){
		OV.sendNMEA( P_OPENVARIO, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature  );
	}
	else if( nmea_protocol.get() == CAMBRIDGE ){
		OV.sendNMEA( P_CAMBRIDGE, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature  );
	}
	else if( nmea_protocol.get() == XCVARIO ) {
		OV.sendNMEA( P_XCVARIO, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature,
				-accelG[2], accelG[1],accelG[0], gyroDPS.x, gyroDPS.y, gyroDPS.z );
	}
	else if( nmea_protocol.get() == NMEA_OFF ) {
		;
	}
	else
		ESP_LOGE(FNAME,"Protocol %d not supported error", nmea_protocol.get() );
	xSemaphoreGive(xMutex);
}


void clientLoop(void *pvParameters)
{
	int ccount = 0;
	gflags.validTemperature = true;
	while (true)
	{
		TickType_t xLastWakeTime = xTaskGetTickCount();
		ccount++;
		aTE += (te_vario.get() - aTE)* (1/(10*vario_av_delay.get()));

		if( !(ccount%5) )
		{
			double tmpalt = altitude.get(); // get pressure from altitude
			if( (fl_auto_transition.get() == 1) && ((int)( Units::meters2FL( altitude.get() )) + (int)(gflags.standard_setting) > transition_alt.get() ) ) {
				ESP_LOGI(FNAME,"Above transition altitude");
				baroP = baroSensor->calcPressure(1013.25, tmpalt); // above transition altitude
			}
			else {
				baroP = baroSensor->calcPressure( QNH.get(), tmpalt);
			}
			dynamicP = Atmosphere::kmh2pascal(ias.get());
			tas = Atmosphere::TAS2( ias.get(), altitude.get(), OAT.get() );
			if( airspeed_mode.get() == MODE_CAS )
				cas = Atmosphere::CAS( dynamicP );
			XCVTemp = bmpVario.bmpTemp;
			if( gflags.haveMPU && HAS_MPU_TEMP_CONTROL ){
				MPU.temp_control( ccount,XCVTemp );
			}
			if( accelG[0] > gload_pos_max.get() ){
				gload_pos_max.set( (float)accelG[0] );
			}else if( accelG[0] < gload_neg_max.get() ){
				gload_neg_max.set(  (float)accelG[0] );
			}
			toyFeed();
			Router::routeXCV();
			if( true && !(ccount%5) ) { // todo need a mag_hdm.valid() flag
				if( compass_nmea_hdm.get() ) {
					xSemaphoreTake( xMutex, portMAX_DELAY );
					OV.sendNmeaHDM( mag_hdm.get() );
					xSemaphoreGive( xMutex );
				}

				if( compass_nmea_hdt.get() ) {
					xSemaphoreTake( xMutex, portMAX_DELAY );
					OV.sendNmeaHDT( mag_hdt.get() );
					xSemaphoreGive( xMutex );
				}
			}
			lazyNvsCommit();
			esp_task_wdt_reset();
			if( uxTaskGetStackHighWaterMark( bpid ) < 512 )
				ESP_LOGW(FNAME,"Warning client task stack low: %d bytes", uxTaskGetStackHighWaterMark( bpid ) );
		}
//		clientLoopTime = (esp_timer_get_time()/1000.0) - clientLoopTime;
//		ESP_LOGI(FNAME,"clientLoop: %0.1f  / %0.1f", clientLoopTime, 100.0 );
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
	}
}

void readSensors(void *pvParameters){
	
	float CL = 0.0;
	float prevCL = 0.0;
	float dAoA = 0.0;
	float AoARaw = 0.0;
	float AccelzFiltAoA = 0.0;


	// Wind speed variables
	float Vgx = 0.0;
	float Vgy = 0.0;
	float VgxPrev = 0.0;
	float VgyPrev = 0.0;
	float Vgxpast[15] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	float Vgypast[15] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };	
	float Vhbipast[15] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	float DeltaVgx;
	float DeltaVgy;
	float SegmentSquare;
	float Segment = 0.0;
	float MidSegmentx;
	float MidSegmenty;
	float Median = 0.0;
	float MedianDirx;
	float MedianDiry;
	float Windx = 0.0;
	float Windy = 0.0;
	float Windxalt;
	float Windyalt;
	float fcWind = 1.0;
	float fcWind1 = 1.0;
	float fcWind2 = 0.0;
	float FilteredWindx = 0.0;
	float FilteredWindy = 0.0;
	float VhPrev = 0.0;
	float VhAvg = 0.0;
	float VhHeading = 0.0;
	

	
	// alpha beta GNSS parameters
	#define NGNSS 4 //  Filter parameter
	#define GNSSdt 0.25 // typical GNSS dt
	#define GNSSOutliers 30.0 // 30 m/s maximum variation sample to sample
	#define Vgnssmin -100.0
	#define Vgnssmax 100.0
	GnssVx.ABinit( NGNSS, GNSSdt, GNSSOutliers, Vgnssmin, Vgnssmax );
	GnssVy.ABinit( NGNSS, GNSSdt, GNSSOutliers, Vgnssmin, Vgnssmax );
	GnssVz.ABinit( NGNSS, GNSSdt, GNSSOutliers, Vgnssmin, Vgnssmax );

	// alpha beta filters paramegters for Energy and average Energy
	#define NTOTENR 6 // Energy alpha/beta coeff
	#define ENRdt 0.1 // average Energy dt
	#define EnergyOutliers 10.0 // 10 m/s maximum variation sample to sample
	#define EnergyPrimMin -30.0
	#define EnergyPrimMax 30.0
	KinEnergy.ABinit(  NEnergy,  ENRdt, EnergyOutliers, 0.0, 0.0, EnergyPrimMin, EnergyPrimMax );

	// LP filter initialization
	BiasAoB.LPinit( 200.0, 0.1 ); // bias AoB LP filter initialization with 200 seconds filter period and 0.1 second sample period
	AverageTotalEnergy.LPinit( 20.0, 0.1 ); // average total energy initialization with 20 seconds filter period and 0.1 second sample period

	#define DSR 15 // maximum number of samples spacing to compute wind.
	int16_t tickDSR = 1;
	
	// compute once the filter parameters in functions of values in FLASH
	NEnergy = te_filt.get(); // Total Energy alpha/beta N
	
	opt_TE = te_opt.get(); // get last d(TE)/dt calculation option
	
	statTime = (esp_timer_get_time()/1000) - 25 ; // initialize statTime to get a 25ms dtStat at startup

	SENDataReady = false;
	SEN50DataReady = false;

	while (1)
	{ 
		count++;

		TickType_t xLastWakeTime = xTaskGetTickCount();
		
		ProcessTimeSensors = (esp_timer_get_time()/1000.0);
		
		// get MPU temp
		MPUtempcel = MPU.getTemperature();
		
		// get Ublox GNSS data
		// when GNSS receiver is connected to S1 interface
		gnss_data_t *gnss1 = s1UbloxGnssDecoder.getGNSSData(1);
		// when GNSS receiver is connected to S2 interface
		gnss_data_t *gnss2 = s2UbloxGnssDecoder.getGNSSData(2);
		// select gnss with better fix
		chosenGnss = (gnss2->fix >= gnss1->fix) ? gnss2 : gnss1;
		GNSSRouteraw = chosenGnss->route;
		
		// MOD#2 add RTK begin
		#ifdef RTK
		if ( RTKtime >= 0 ) {
			if ( RTKmode != 'N' ) {
				if (RTKmode == 'A' ) chosenGnss->fix = 3; // GNSS 3D
				if (RTKmode == 'D' ) chosenGnss->fix = 4; // GNSS 3D diff
				if (RTKmode == 'F' ) chosenGnss->fix = 5; // GNSS RTK float
				if (RTKmode == 'R' ) chosenGnss->fix = 6; // GNSS RTK Real Time Kinematic
				GNSSstatus = chosenGnss->fix;
				chosenGnss->numSV = RTKratio * 10;
				chosenGnss->time = RTKtime;
				chosenGnss->speed.x = RTKNvel;
				chosenGnss->speed.y = RTKEvel;
				chosenGnss->speed.z = -RTKUvel;
				dtRTKtime = RTKtime - prevRTKtime;
				prevRTKtime = RTKtime;
				if ( dtRTKtime > 0.0 ) {
					GnssVx.ABupdate( dtRTKtime, chosenGnss->speed.x );
					GnssVy.ABupdate( dtRTKtime, chosenGnss->speed.y );		
					GnssVz.ABupdate( dtRTKtime, chosenGnss->speed.z );
				} 				
				GNSSRouteraw = atan2(GnssVy.ABfilt(),GnssVx.ABfilt());			
			} else {
				chosenGnss->fix = 0;
			}
		} else {
			chosenGnss->fix = 8; // GNSS RTK checksum error or bad time			
			chosenGnss->time = -1;
		}			
		#endif
		// MOD#2 add RTK end

		
		// MOD#5 add Allystar TAU1201 begin
		#ifdef ALLYSTAR
		if ( Allytime >= 0 ) {
			chosenGnss->fix = 7; // GNSS Allystar TAU1301 fix	
			chosenGnss->time = Allytime;
			dtAllytime = Allytime - prevAllytime;
			prevAllytime = Allytime;
			chosenGnss->speed.x = AllyvelN;
			chosenGnss->speed.y = AllyvelE;
			chosenGnss->speed.z = -AllyvelU;
			if ( dtAllytime > 0.0 ) {
				GnssVx.ABupdate( dtAllytime, chosenGnss->speed.x );
				GnssVy.ABupdate( dtAllytime, chosenGnss->speed.y );		
				GnssVz.ABupdate( dtAllytime, chosenGnss->speed.z );
			} 
			GNSSRouteraw = atan2(GnssVy.ABfilt(),GnssVx.ABfilt());
		} else {
			chosenGnss->fix = 8; // GNSS Allystar TAU1301 checksum error or bad time			
			chosenGnss->time = -1;
		}
		#endif
		
		// compute CAS, ALT and Vzbaro using alpha/beta filters.  TODO consider using atmospher.h functions
		
		// compute Rho at current pressure and OAT
		if (statP.Get() > 500.0) {
			Rho = (100.0 * statP.Get() / 287.058 / (273.15 + temperatureLP.LowPass1()));
		} else {
			Rho = RhoSLISA;
		}

		// compute AoA (Angle of attack) and AoB (Angle od slip)
		#define FreqAlpha 0.66 // Hz
		#define fcAoA1 (10.0/(10.0+FreqAlpha))
		#define fcAoA2 (1.0-fcAoA1)
		#define FreqBeta 0.66 // Hz
		#define fcAoB1 (10.0/(10.0+FreqBeta))
		#define fcAoB2 (1.0-fcAoB1)		
		WingLoad = gross_weight.get() / polar_wingarea.get();  // should be only computed when pilot change weight settings in XCVario
		//xSemaphoreTake( dataMutex, 3/portTICK_PERIOD_MS ); // prevent data conflicts for 3ms max.		
		if ( (dynP.Get()>100.0) && (CAS.ABfilt() >10.0) && (TAS.Get()>10.0) && (abs(accelNEDBODYz.ABfilt()) > 1.0) ) { // compute AoA and AoB only when dynamic pressure is above 100 Pa, CAS & TAS abobe 10m/s and accel z above 1 m/s²
			AccelzFiltAoA = 0.8 * AccelzFiltAoA + 0.2 * accelNEDBODYz.ABfilt(); // simple ~3 Hz low pass on accel z
			CL = -AccelzFiltAoA * 2 / RhoSLISA * WingLoad / CAS.ABfilt() / CAS.ABfilt();
			dAoA = ( CL - prevCL ) / CLA;
			prevCL = CL;
			if (abs(AccelzFiltAoA) > 1.0) { //when not close to Az=0, hybridation of aoa from drag & aoa from lift
				AoARaw = -(accelNEDBODYx.ABfilt()/ accelNEDBODYz.ABfilt()) - Speed2Fly.cw( CAS.ABfilt() ) / Speed2Fly.getN();
				AoA.Set( fcAoA1 * ( AoA.Get() + dAoA ) + fcAoA2 * AoARaw );
			}  else { //when  close to Az=0, only aoa from lift considered
                AoA.Set( AoA.Get() + dAoA ) ;
            }			
			AoB.Set( fcAoB1 * AoB.Get() + fcAoB2 * ( KAoB * WingLoad * accelNEDBODYy.ABfilt()/ dynP.Get() - KGx * gyroCorrx.Get() / TAS.Get()) - Bias_AoB );	
		} else {
			AoA.Set( 0.0 );
			AoB.Set( 0.0 );
		}
		//xSemaphoreGive( dataMutex );
		#ifdef COMPUTEBIAS
			// if TAS > ~110 km/h and bank is less than ~4.5°, long term average of AoB to detect bias
			#define RollLimitAoB 0.08 // max roll for AoB bias estimation
			#define MinTASAoB 30.0 // minimum speed to evaluate AoB bias
			#define AoBMaxBias 0.1 // limit biais correction to 100 mrad/s
			if ( ( TAS.Get() > MinTASAoB ) && ( abs(Roll) < RollLimitAoB ) ) {
				BiasAoB.LPupdate( AoB.Get() );
				Bias_AoB = BiasAoB.LowPass2();
				if ( abs(Bias_AoB) > AoBMaxBias ) Bias_AoB = copysign( AoBMaxBias, Bias_AoB);
			}
		#endif
		
		// Compute trajectory pneumatic speeds components in body frame NEDBODY
		// Vh corresponds to the trajectory horizontal speed and Vzbaro corresponds to the vertical speed in earth frame
		Vh = TAS.Get() * cos( Pitch - cosRoll * AoA.Get() - sinRoll * AoB.Get() ); // MOD#1 Latest signs
		// Pitch and Roll correspond to the attitude of the glider and DHeading corresponds to the heading deviation due to the Beta and Alpha.
		DHeading = -(AoB.Get() * cosRoll - AoA.Get() * sinRoll ) / ( cosPitch + AoB.Get() * sinPitch * sinRoll + AoA.Get() * sinPitch * cosRoll ); // MOD#1 Latest signs
		cosDHeading = cos( DHeading );
		sinDHeading = sin( DHeading );
		// applying DCM from earth to body frame (using Pitch, Roll and DHeading Yaw angles) to Vh and Vzbaro trajectory components in earth frame to reproject speed in TE referential onto body axis. 
		Ub = cosPitch * cosDHeading * Vh - sinPitch * Vzbaro;
		Vb = ( sinRoll * sinPitch * cosDHeading - cosRoll * sinDHeading ) * Vh + sinRoll * cosPitch * Vzbaro;
		Wb = ( cosRoll * sinPitch * cosDHeading + sinRoll * sinDHeading ) * Vh + cosRoll * cosPitch * Vzbaro;

		// Baro acceleration derivative Short period alpha/beta filter
		// U/V/WbPrimS are used to compute U/V/WbiPrim, baro inertial accelerations
		#define NBaroAccS 7.0 // accel kinetic alpha/beta filter coeff
		#define alphaBaroAccS (2.0 * (2.0 * NBaroAccS - 1.0) / NBaroAccS / (NBaroAccS + 1.0))
		#define betaBaroAccS (6.0 / NBaroAccS / (NBaroAccS + 1.0) )			
		deltaUbS = Ub - UbFS;
		UbPrimS = UbPrimS + betaBaroAccS * deltaUbS / dtStat;
		UbFS = UbFS + alphaBaroAccS * deltaUbS + UbPrimS * dtStat;
		deltaVbS = Vb - VbFS;
		VbPrimS = VbPrimS + betaBaroAccS * deltaVbS / dtStat;
		VbFS = VbFS + alphaBaroAccS * deltaVbS + VbPrimS * dtStat;			
		deltaWbS = Wb - WbFS;
		WbPrimS = WbPrimS + betaBaroAccS * deltaWbS / dtStat;
		WbFS = WbFS + alphaBaroAccS * deltaWbS + WbPrimS * dtStat;
			
		// baro interial speed in earth frame
		Vxbi = cosPitch * Ubi + sinRoll * sinPitch * Vb + cosRoll * sinPitch * Wbi;
		Vybi = cosRoll * Vb - sinRoll * Wbi;
		Vzbi = -sinPitch * Ubi + sinRoll * cosPitch * Vb + cosRoll * cosPitch * Wbi;

		// baro inertial altitude
		// ALTbi is computed using a complementary filter with baro altitude and baro inertial vertical speed in earth frame
		#define PeriodAltbi 1.0 // period in second for baro/inertial altitude. Baro/inertial velocity improves baro sensor response
		#define fcAltbi1 ( PeriodAltbi / ( PeriodAltbi + PERIOD10HZ ))
		#define fcAltbi2 ( 1.0 - fcAltbi1 )		
		ALTbi = fcAltbi1 * ( ALTbi - Vzbi * dtStat ) + fcAltbi2	* ALT.ABfilt();

		// Energy variation d(TE)/dt options
		/*
		if (opt_TE == 1.0 ) {
			// option 1
			// energy variation calculation d(E/mg)/dt = - Vzbi + d(1/2 1/g TASbi²)/dt
			// update kinetic energy filter
			KinEnergy.ABupdate( dtStat, ( TASbiSquare / GRAVITY / 2.0 ) );
			// filter total energy variation for display to pilot
			//TotalEnergy.LPupdate( te_filt.get(), dtStat, (-Vzbi + KinEnergy.ABprim()) );
			Vztotbi.Set( -Vzbi + KinEnergy.ABprim() );
		} else {
			// option 2
			// energy variation calculation d(E/mg)/dt = d(ALT)/dt + d(1/2 1/g TAS²)/dt
			// compute and filter with AB using different N value both terms of equation
			// check if A/B filters N has changed and update
			if ( NALTbiTASbiChanged ) {
				#define ALTbiTASbiEnergdt 0.1 // average Energy dt
				#define ALTbiTASbiEnergyOutliers 10.0 // 10 m/s maximum variation sample to sample
				#define ALTbiTASbiEnergyPrimMin -50.0
				#define ALTbiTASbiEnergyPrimMax 50.0
				ALTbiEnergy.ABinit(  ALTbiN,  ALTbiTASbiEnergdt, ALTbiTASbiEnergyOutliers, 0.0, 0.0, ALTbiTASbiEnergyPrimMin, ALTbiTASbiEnergyPrimMax );
				TASbiEnergy.ABinit(  ALTbiN + TASbiN,  ALTbiTASbiEnergdt, ALTbiTASbiEnergyOutliers, 0.0, 0.0, ALTbiTASbiEnergyPrimMin, ALTbiTASbiEnergyPrimMax );
				NALTbiTASbiChanged = false;
			}
			ALTbiEnergy.ABupdate( dtStat, ALTbi );
			TASbiEnergy.ABupdate( dtStat, ( TASbiSquare / GRAVITY / 2.0 ) );			
			// Total Energy is sum of both potential and kinetic energies variations
			Vztotbi.Set( ALTbiEnergy.ABprim() + TASbiEnergy.ABprim() );
		} */
		// TODO only use second solution to compute total energy variation
		// energy variation calculation d(E/mg)/dt = d(ALT)/dt + d(1/2 1/g TAS²)/dt
		// compute and filter with AB using different N value both terms of equation
		// check if A/B filters N has changed and update
		if ( NALTbiTASbiChanged ) {
			#define ALTbiTASbiEnergdt 0.1 // average Energy dt
			#define ALTbiTASbiEnergyOutliers 10.0 // 10 m/s maximum variation sample to sample
			#define ALTbiTASbiEnergyPrimMin -50.0
			#define ALTbiTASbiEnergyPrimMax 50.0
			ALTbiEnergy.ABinit(  ALTbiN,  ALTbiTASbiEnergdt, ALTbiTASbiEnergyOutliers, 0.0, 0.0, ALTbiTASbiEnergyPrimMin, ALTbiTASbiEnergyPrimMax );
			TASbiEnergy.ABinit(  ALTbiN + TASbiN,  ALTbiTASbiEnergdt, ALTbiTASbiEnergyOutliers, 0.0, 0.0, ALTbiTASbiEnergyPrimMin, ALTbiTASbiEnergyPrimMax );
			NALTbiTASbiChanged = false;
		}
		ALTbiEnergy.ABupdate( dtStat, ALTbi );
		TASbiEnergy.ABupdate( dtStat, ( TASbiSquare / GRAVITY / 2.0 ) );			
		// Total Energy is sum of both potential and kinetic energies variations
		Vztotbi.Set( ALTbiEnergy.ABprim() + TASbiEnergy.ABprim() );

		
		// long term average filter
		AverageTotalEnergy.LPupdate( Vztotbi.Get() );		

		#ifdef COMPUTEWIND1
		// TODO test and optimze wind calculation
		// compute wind speed using GNSS and horizontal true airspeed
		Vgx = 0.5 * Vgx + 0.5 * GnssVx.ABfilt(); // GNSS x coordinate with short low pass to reduce noise
		Vgy = 0.5 * Vgy + 0.5 * GnssVy.ABfilt(); // GNSS y coordinate with short low pass to reduce noise
		DeltaVgx = Vgx-VgxPrev; // Variation of x speed coordinate
		DeltaVgy = Vgy-VgyPrev; // Variation of y speed coordinate
		SegmentSquare = DeltaVgx*DeltaVgx+DeltaVgy*DeltaVgy; // squared module of segment between speed vectors extremities
		Segment = sqrt(SegmentSquare); // module of segment
		float Vhbi = sqrt( Vxbi * Vxbi + Vybi * Vybi ); // Vhbi baro inertiel horizontal speed in earth frame
		VhAvg = ( Vhbi + VhPrev ) / 2; // average horizontal speed
		if ( (Segment > 0.75) && (VhAvg > Segment/2) && (VgxPrev != 0.0) && (VgyPrev != 0.0) && (DeltaVgx != 0.0) && (DeltaVgy != 0.0) ) {
			MidSegmentx = (Vgx+VgxPrev)/2; // mid segment x
			MidSegmenty = (Vgy+VgyPrev)/2; // mid segment y
			Median = sqrt(VhAvg*VhAvg-SegmentSquare/4); // module of median between segment center and true airspedd origin (usinf average of current and previous true airspeed
			MedianDirx = -DeltaVgy/Segment; // direction of median x
			MedianDiry = DeltaVgx/Segment; // direction of median y
			Windx = MidSegmentx + Median * MedianDirx; // wind x coordinate
			Windy = MidSegmenty + Median * MedianDiry; // wind y coordinate
			Windxalt = MidSegmentx - Median * MedianDirx; // alternate wind x coordinate
			Windyalt = MidSegmenty - Median * MedianDiry; // alternate wind y coordinate
			if ( (Windx*Windx+Windy*Windy) > (Windxalt*Windxalt+Windyalt*Windyalt) ) { // Pick wind coordinates closest to GNSS vectors origin.
				Windx = Windxalt;
				Windy = Windyalt;
			}				
			fcWind = 0.02 * 17.0 / Segment * Median; // fc low pass filter
			fcWind1 = fcWind / (fcWind + 0.1); 
			fcWind2 = 1 - fcWind1;
			FilteredWindx = fcWind1 * FilteredWindx + fcWind2 * Windx;
			FilteredWindy = fcWind1 * FilteredWindy + fcWind2 * Windy;
			VhHeading = M_PI + atan2(Vgx-Windx, Vgy-Windy);
			if ( VhHeading < 0 ) VhHeading = VhHeading + 2.0 * M_PI;
			VhHeading = VhHeading * 180.0 / M_PI;
			VgxPrev = Vgx;
			VgyPrev = Vgy;
			VhPrev = Vhbi;
		} else {				
			tickDSR++;
			if ( tickDSR > DSR ) {
				VgxPrev = Vgx;
				VgyPrev = Vgy;
				VhPrev = Vhbi;
				tickDSR = 1;
			}
		}
		#endif
	
		#ifdef COMPUTEWIND2
		// compute wind speed using GNSS and horizontal baro inertial true airspeed

		float Vhbi; // horizontal baro inertial true airspeed
		float VhbiAvg;
		bool gotwind = false;
		#define MINSEGMENT 0.75 // minimum segment size , i.e. minimu GNSS speed variation

		// very short low pass filter on GNSS speed to reduce noise
		Vgx = 0.5 * Vgx + 0.5 * GnssVx.ABfilt();
		Vgy = 0.5 * Vgy + 0.5 * GnssVy.ABfilt();

		// compute Vhbi baro inertial horizontal speed in earth frame
		Vhbi = sqrt( Vxbi * Vxbi + Vybi * Vybi );

		// consider last 15 samples to verify if speed variation is significant enough and compute wind as soon as condition is met
		for ( int i=0; i<15 ; i++ ) {
			if ( !gotwind ) {
				// compute wind variation coordinates
				DeltaVgx = Vgx-Vgxpast[i];
				DeltaVgy = Vgy-Vgypast[i];
				// compute segment length corresponding to GNSS speed variation ( segment square and segment)
				SegmentSquare = DeltaVgx*DeltaVgx+DeltaVgy*DeltaVgy; // squared module of segment between gnss speed vectors
				Segment = sqrt(SegmentSquare); // module of segment between gnss speed vectors
				// algorithm expects horizontal true airspeed to be constant but average is used to reduce error
				VhbiAvg = ( Vhbi + Vhbipast[i] ) / 2; // average horizontal baro inertial speed
				// test if no wind yet and long enough segment and conditions met to avoid calc errors
				if ( (Segment > MINSEGMENT ) && (VhbiAvg > Segment/2) && (Vgxpast[i] != 0.0) && (Vgypast[i] != 0.0) && (DeltaVgx != 0.0) && (DeltaVgy != 0.0) ) {
					MidSegmentx = (Vgx+Vgxpast[i])/2; // mid segment x
					MidSegmenty = (Vgy+Vgypast[i])/2; // mid segment y
					Median = sqrt(VhbiAvg*VhbiAvg-SegmentSquare/4); // module of median between segment center and true airspedd origin (usinf average of current and previous true airspeed
					MedianDirx = -DeltaVgy/Segment; // direction of median x
					MedianDiry = DeltaVgx/Segment; // direction of median y
					// There are two solutions to the problem
					Windx = MidSegmentx + Median * MedianDirx; // wind x coordinate
					Windy = MidSegmenty + Median * MedianDiry; // wind y coordinate
					Windxalt = MidSegmentx - Median * MedianDirx; // alternate wind x coordinate
					Windyalt = MidSegmenty - Median * MedianDiry; // alternate wind y coordinate
					// find the solution which is on the good side of the GNSS vectors origin.
					if ( (Windx*Windx+Windy*Windy) > (Windxalt*Windxalt+Windyalt*Windyalt) ) { // Pick wind coordinates closest to GNSS vectors origin.
						Windx = Windxalt;
						Windy = Windyalt;
					}
					// low pass filter on wind in function of segment and median size. Larger segment and median size increase filter cutoff frequency                                               
					fcWind = 0.02 * 17.0 / Segment * Median; // fc low pass filter
					fcWind1 = fcWind / (fcWind + 0.1); 
					fcWind2 = 1 - fcWind1;
					FilteredWindx = fcWind1 * FilteredWindx + fcWind2 * Windx;
					FilteredWindy = fcWind1 * FilteredWindy + fcWind2 * Windy;
					// using GNSS speed and wind speed, compute Vh = TAS heading
					VhHeading = M_PI + atan2(Vgx-Windx, Vgy-Windy);
					if ( VhHeading < 0 ) VhHeading = VhHeading + 2.0 * M_PI;
					VhHeading = VhHeading * 180.0 / M_PI;
					gotwind = true;
				}
			}
			// slide/update  samples window
			if ( i <14 ) {
				Vgxpast[i+1] = Vgxpast[i];
				Vgypast[i+1] = Vgypast[i];
				Vhbipast[i+1] = Vhbipast[i];
			}
			Vgxpast[0] = Vgx;
			Vgypast[0] = Vgy;
			Vhbipast[0] = Vhbi;
		}                                            
		#endif
				
		//
		// Eckhard code
		//

		/* // TODO replace Eckhard code with flight test values
		float iasraw = Atmosphere::pascal2kmh( dynamicP );

		float T = OATemp;
		float tasraw = 0;
		if( baroP != 0 )
			tasraw =  Atmosphere::TAS( iasraw , baroP, T);  // True airspeed in km/h

		if( airspeed_mode.get() == MODE_CAS ){
			float casraw=Atmosphere::CAS( dynamicP );
			cas += (casraw-cas)*0.25;       // low pass filter
			// ESP_LOGI(FNAME,"IAS=%f, TAS=%f CAS=%f baro=%f", iasraw, tasraw, cas, baroP );
		}
		static float new_ias = 0;
		new_ias = ias.get() + (iasraw - ias.get())*0.25;
		if( (int( ias.get()+0.5 ) != int( new_ias+0.5 ) ) || !(count%20) ){
			ias.set( new_ias );  // low pass filter
		}
		if( airspeed_max.get() < ias.get() ){
			airspeed_max.set( ias.get() );
		}
		// ESP_LOGI("FNAME","P: %f  IAS:%f IASF: %d", dynamicP, iasraw, ias );
		if( !compass || !(compass->externalData()) ){
			tas += (tasraw-tas)*0.25;       // low pass filter
		}
		// ESP_LOGI(FNAME,"IAS=%f, T=%f, TAS=%f baroP=%f", ias, T, tas, baroP );

		// Slip angle estimation
		float as = tas/3.6;                  // tas in m/s
		const float K = 4000 * 180/M_PI;      // airplane constant and Ay correction factor
		if( tas > 25.0 ){
			slipAngle += ((accelG[1]*K / (as*as)) - slipAngle)*0.09;   // with atan(x) = x for small x
			// ESP_LOGI(FNAME,"AS: %f m/s, CURSL: %f°, SLIP: %f", as, -accelG[1]*K / (as*as), slipAngle );
		} */ // TODO replace Eckhard code with flight test values

		cas = CAS.ABfilt() * 3.6;
		if ( cas < 30.0 ) cas = 0.0;
		if( (int( ias.get()+0.5 ) != int( cas+0.5 ) ) || !(count%20) ){
			ias.set( cas );  // low pass filter
		}		
		
		// TODO remove unecessary code for flgiht test
		xSemaphoreTake(xMutex,portMAX_DELAY );

		float te = bmpVario.readTE( TAS.Get() );
		if( (int( te_vario.get()*20 +0.5 ) != int( te*20 +0.5)) || !(count%10) ){  // a bit more fine granular updates than 0.1 m/s as of sound
			te_vario.set( te );  // max 10x per second
		}
		xSemaphoreGive(xMutex);
		// TODO remove unecessary code for flgiht test
		
		// ESP_LOGI(FNAME,"count %d ccp %d", count, ccp );
		if( !(count % ccp) ) {
			AverageVario::recalcAvgClimb();
		}
		 /* if (FLAP) { FLAP->progress(); }		*/ // TODO remove unecessary code for flgiht test
		 
		// TODO remove unecessary code for flgiht test
		//xSemaphoreTake(xMutex,portMAX_DELAY );
		//baroP = baroSensor->readPressure(ok);   // 10x per second
		//xSemaphoreGive(xMutex);
		// ESP_LOGI(FNAME,"Baro Pressure: %4.3f", baroP );
		float altSTD = 0;
		if( Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL )
			altSTD = alt_external;
		else
			altSTD = baroSensor->calcAVGAltitudeSTD( baroP );
		float new_alt = 0;
		if( alt_select.get() == AS_TE_SENSOR ) // TE
			new_alt = bmpVario.readAVGalt();
		else if( alt_select.get() == AS_BARO_SENSOR  || alt_select.get() == AS_EXTERNAL ){ // Baro or external
			if(  alt_unit.get() == ALT_UNIT_FL ) { // FL, always standard
				new_alt = altSTD;
				gflags.standard_setting = true;
				// ESP_LOGI(FNAME,"au: %d", alt_unit.get() );
			}else if( (fl_auto_transition.get() == 1) && ((int)Units::meters2FL( altSTD ) + (int)(gflags.standard_setting) > transition_alt.get() ) ) { // above transition altitude
				new_alt = altSTD;
				gflags.standard_setting = true;
				ESP_LOGI(FNAME,"auto:%d alts:%f ss:%d ta:%f", fl_auto_transition.get(), altSTD, gflags.standard_setting, transition_alt.get() );
			}
			else {
				if( Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL )
					new_alt = altSTD + ( QNH.get()- 1013.25)*8.2296;  // correct altitude according to ISA model = 27ft / hPa
				else
					new_alt = baroSensor->calcAVGAltitude( QNH.get(), baroP );
				gflags.standard_setting = false;
				// ESP_LOGI(FNAME,"QNH %f baro: %f alt: %f SS:%d", QNH.get(), baroP, alt, gflags.standard_setting  );
			}
		}
		if( (int( new_alt +0.5 ) != int( altitude.get() +0.5 )) || !(count%20) ){
			// ESP_LOGI(FNAME,"New Altitude: %.1f", new_alt );
			altitude.set( new_alt );
		}

		/* aTE = bmpVario.readAVGTE(); */ // TODO remove unecessary code for flgiht test
		doAudio();

		if( !Flarm::bincom && ((count % 2) == 0 ) ){
			toyFeed();
			vTaskDelay(2/portTICK_PERIOD_MS);
		}

		// ESP_LOGI(FNAME,"Compass, have sensor=%d  hdm=%d ena=%d", compass->haveSensor(),  compass_nmea_hdt.get(),  compass_enable.get() );
		if( compass ){
			if( !Flarm::bincom && ! compass->calibrationIsRunning() ) {
				// Trigger heading reading and low pass filtering. That job must be
				// done periodically.
				bool ok;
				float heading = compass->getGyroHeading( &ok );
				if(ok){
					if( (int)heading != (int)mag_hdm.get() && !(count%10) ){
						mag_hdm.set( heading );
					}
					if( !(count%5) && compass_nmea_hdm.get() == true ) {
						xSemaphoreTake( xMutex, portMAX_DELAY );
						OV.sendNmeaHDM( heading );
						xSemaphoreGive( xMutex );
					}
				}
				else{
					if( mag_hdm.get() != -1 )
						mag_hdm.set( -1 );
				}
				float theading = compass->filteredTrueHeading( &ok );
				if(ok){
					if( (int)theading != (int)mag_hdt.get() && !(count%10) ){
						mag_hdt.set( theading );
					}
					if( !(count%5) && ( compass_nmea_hdt.get() == true )  ) {
						xSemaphoreTake( xMutex, portMAX_DELAY );
						OV.sendNmeaHDT( theading );
						xSemaphoreGive( xMutex );
					}
				}
				else{
					if( mag_hdt.get() != -1 )
						mag_hdt.set( -1 );
				}
			}
		}
		if( accelG[0] > gload_pos_max.get() ){
			gload_pos_max.set( (float)accelG[0] );
		}else if( accelG[0] < gload_neg_max.get() ){
			gload_neg_max.set(  (float)accelG[0] );
		}
		
		/* // TODO remove unecessary code for flgiht test
		// Check on new clients connecting
		if ( CAN && CAN->GotNewClient() ) { // todo use also for Wifi client?!
			while( client_sync_dataIdx < SetupCommon::numEntries() ) {
				if ( SetupCommon::syncEntry(client_sync_dataIdx++) ) {
					break; // Hit entry to actually sync and send data
				}
			}
			if ( client_sync_dataIdx >= SetupCommon::numEntries() ) {
				// Synch complete
				client_sync_dataIdx = 0;
				CAN->ResetNewClient();
			}
		}
		 */ // TODO remove unecessary code for flgiht test
		lazyNvsCommit();
		XCVTemp = bmpVario.bmpTemp;
		if( gflags.haveMPU && HAS_MPU_TEMP_CONTROL ){
			// ESP_LOGI(FNAME,"MPU temp control; T=%.2f", MPU.getTemperature() );
			MPU.temp_control( count,XCVTemp);
		}
		
		// TODO event counter
		if ( (ESPRotary::readLongPressed()) && (EventHoldTime == 0) ) {
			Event++;
			EventHoldTime = 5;
			Audio::alarm( true, 60, AUDIO_ALARM_STALL );
		} else {
			if ( EventHoldTime > 0 ) {
				EventHoldTime--;
			} else {
				Audio::alarm( false, 60, AUDIO_ALARM_STALL );
				EventHoldTime = 0;
			}
		}	

		if ( SENstream ) {
			// see definition of $S1, $S2, $S3 in IMU section
			if ( !(count % 50) ) { 
				// send $S1, $S2, $S3 stream
				SEN50DataReady = true;
				
			} else {
				// send $S1 and $S3 only every 100ms
				SENDataReady = true;
			}
		}

		ProcessTimeSensors = (esp_timer_get_time()/1000.0) - ProcessTimeSensors;
		if ( ProcessTimeSensors > 30 && TAS.Get() < 15.0 ) {
			ESP_LOGI(FNAME,"readSensors: %i / 100", (int16_t)(ProcessTimeSensors) );
		}		
		esp_task_wdt_reset();
		if( uxTaskGetStackHighWaterMark( bpid ) < 512 && TAS.Get() < 15.0)
			ESP_LOGW(FNAME,"Warning sensor task stack low: %d bytes", uxTaskGetStackHighWaterMark( bpid ) );
		vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
	}
}

static int ttick = 0;
static float temp_prev = -3000;

void readTemp(void *pvParameters){
	
	temperatureLP.LPinit( 4.0, 1.0 ); // Init LP filter with 4 second period and 1 second update rate

	while (1) {
		TickType_t xLastWakeTime = xTaskGetTickCount();
		float t=15.0;
		battery = Battery.get();
		// ESP_LOGI(FNAME,"Battery=%f V", battery );
		if( !SetupCommon::isClient() ) {  // client Vario will get Temperature info from main Vario
			t = ds18b20.getTemp();
			if( t ==  DEVICE_DISCONNECTED_C ) {
				if( gflags.validTemperature == true ) {
					//ESP_LOGI(FNAME,"Temperatur Sensor disconnected or out of realistic range , please plug Temperature Sensor or check sensor");
					gflags.validTemperature = false;
				}
			}
			else
			{
				if( gflags.validTemperature == false ) {
					//ESP_LOGI(FNAME,"Temperatur Sensor connected, temperature valid");
					gflags.validTemperature = true;
				}
				// ESP_LOGI(FNAME,"temperature=%2.1f", temperature );
				OATemp.ABupdate( 1.0, t );
				temperature =  OATemp.ABfilt();
				if( temperature > 65.0 )
					temperature = 20 - ALT.ABfilt() * .0065;  // if temperature error, switch to standard ISA + 5
				temperatureLP.LPupdate( temperature );
				if( abs(temperatureLP.LowPass1() - temp_prev) > 0.1 ){
					OAT.set( std::round(temperatureLP.LowPass1()*10)/10 );
					//ESP_LOGI(FNAME,"NEW temperature=%2.1f, prev T=%2.1f", temperatureLP.LowPass1(), temp_prev );
					temp_prev = temperatureLP.LowPass1();
				}
			}
			//ESP_LOGV(FNAME,"temperature=%f", temperature );
			Flarm::tick();
			if( compass )
				compass->tick();
		}else{
			if( (OAT.get() > -45.0) && (OAT.get() < 65.0) )
				gflags.validTemperature = true;
		}
		theWind.tick();
		CircleWind::tick();
		Flarm::progress();
//		readTempTime = (esp_timer_get_time()/1000.0) - readTempTime;
//		ESP_LOGI(FNAME,"readTemp: %0.1f  / %0.1f", readTempTime, 1000.0 );
		vTaskDelayUntil(&xLastWakeTime, 1000/portTICK_PERIOD_MS);
		esp_task_wdt_reset();
		if( (ttick++ % 5) == 0) {
			//ESP_LOGI(FNAME,"Free Heap: %d bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT) );
			if( uxTaskGetStackHighWaterMark( tpid ) < 256 )
				ESP_LOGW(FNAME,"Warning temperature task stack low: %d bytes", uxTaskGetStackHighWaterMark( tpid ) );
			if( heap_caps_get_free_size(MALLOC_CAP_8BIT) < 20000 )
				ESP_LOGW(FNAME,"Warning heap_caps_get_free_size getting low: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
		}
	}
}

static esp_err_t _coredump_to_server_begin_cb(void * priv)
{
	ets_printf("================= CORE DUMP START =================\r\n");
	return ESP_OK;
}

static esp_err_t _coredump_to_server_end_cb(void * priv)
{
	ets_printf("================= CORE DUMP END ===================\r\n");
	return ESP_OK;
}

static esp_err_t _coredump_to_server_write_cb(void * priv, char const * const str)
{
	ets_printf("%s\r\n", str);
	return ESP_OK;
}

void register_coredump() {
	coredump_to_server_config_t coredump_cfg = {
			.start = _coredump_to_server_begin_cb,
			.end = _coredump_to_server_end_cb,
			.write = _coredump_to_server_write_cb,
			.priv = NULL,
	};
	if( coredump_to_server(&coredump_cfg) != ESP_OK ){  // Dump to console and do not clear (will done after fetched from Webserver)
		//ESP_LOGI( FNAME, "+++ All green, no coredump found in FLASH +++");
	}
}


// Sensor board init method. Herein all functions that make the XCVario are launched and tested.
void system_startup(void *args){
	accelG[0] = 1;  // earth gravity default = 1 g
	accelG[1] = 0;
	accelG[2] = 0;
	gyroDPS.x = 0;
	gyroDPS.y = 0;
	gyroDPS.z = 0;
	cur_gyro_bias[0] = 0;
	cur_gyro_bias[1] = 0;
	cur_gyro_bias[2] = 0;

	bool selftestPassed=true;
	int line = 1;
	ESP_LOGI( FNAME, "Now setup I2C bus IO 21/22");
	i2c.begin(GPIO_NUM_21, GPIO_NUM_22, 100000 ); // TODO check optimum frequency. Reduce from 400khz to original 100khz just in case it helps solving vario stability.
	Router::begin();
	theWind.begin();

	MCP = new MCP3221();
	MCP->setBus( &i2c );
	gpio_set_drive_capability(GPIO_NUM_23, GPIO_DRIVE_CAP_1);

	esp_wifi_set_mode(WIFI_MODE_NULL);
	spiMutex = xSemaphoreCreateMutex();
	Menu = new SetupMenu();
	// esp_log_level_set("*", ESP_LOG_INFO);
	ESP_LOGI( FNAME, "Log level set globally to INFO %d; Max Prio: %d Wifi: %d",  ESP_LOG_INFO, configMAX_PRIORITIES, ESP_TASKD_EVENT_PRIO-5 );
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI( FNAME,"This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
					(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
	ESP_LOGI( FNAME,"Silicon revision %d, ", chip_info.revision);
	ESP_LOGI( FNAME,"%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	ESP_LOGI(FNAME, "QNH.get() %.1f hPa", QNH.get() );
	NVS.begin();
	register_coredump();
	Polars::begin();

	the_can_mode = can_mode.get(); // initialize variable for CAN mode
	if( hardwareRevision.get() == HW_UNKNOWN ){  // per default we assume there is XCV-20
		ESP_LOGI( FNAME, "Hardware Revision unknown, set revision 2 (XCV-20)");
		hardwareRevision.set(XCVARIO_20);
	}

	if( display_orientation.get() ){
		ESP_LOGI( FNAME, "TopDown display mode flag set");
		topDown = true;
	}

	wireless = (e_wireless_type)(wireless_type.get()); // we cannot change this on the fly, so get that on boot
	AverageVario::begin();
	stall_alarm_off_kmh = stall_speed.get()/3;

	Battery.begin();  // for battery voltage
	xMutex=xSemaphoreCreateMutex();
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	ccp = (int)(core_climb_period.get()*10);
	SPI.begin( SPI_SCLK, SPI_MISO, SPI_MOSI, CS_bme280BA );
	xSemaphoreGive(spiMutex);

	egl = new AdaptUGC();
	egl->begin();
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	ESP_LOGI( FNAME, "setColor" );
	egl->setColor( 0, 255, 0 );
	ESP_LOGI( FNAME, "drawLine" );
	egl->drawLine( 20,20, 20,80 );
	ESP_LOGI( FNAME, "finish Draw" );
	xSemaphoreGive(spiMutex);

	MYUCG = egl; // new AdaptUGC( SPI_DC, CS_Display, RESET_Display );
	display = new IpsDisplay( MYUCG );
	Flarm::setDisplay( MYUCG );
	DM.begin( MYUCG );
	display->begin();
	display->bootDisplay();

	// int valid;
	String logged_tests;
	logged_tests += "\n\n\n";
	Version V;
	std::string ver( " Ver.: " );
	ver += V.version();
	char hw[24];
	sprintf( hw,", XCV-%d", hardwareRevision.get()+18);  // plus 18, e.g. 2 = XCVario-20
	std::string hwrev( hw );
	ver += hwrev;
	display->writeText(line++, ver.c_str() );
	sleep(1);
	bool doUpdate = software_update.get();
	if( Rotary.readSwitch() ){
		doUpdate = true;
		ESP_LOGI(FNAME,"Rotary pressed: Do Software Update");
	}
	if( doUpdate ) {
		if( hardwareRevision.get() == XCVARIO_20) { // only XCV-20 uses this GPIO for Rotary
			ESP_LOGI( FNAME,"Hardware Revision detected 2");
			Rotary.begin( GPIO_NUM_4, GPIO_NUM_2, GPIO_NUM_0);
		}
		else  {
			ESP_LOGI( FNAME,"Hardware Revision detected 3");
			Rotary.begin( GPIO_NUM_36, GPIO_NUM_39, GPIO_NUM_0);
			gpio_pullup_en( GPIO_NUM_34 );
			if( gflags.haveMPU && HAS_MPU_TEMP_CONTROL && !gflags.mpu_pwm_initalized  ){ // series 2023 does not have slope support on CAN bus but MPU temperature control
				MPU.pwm_init();
				gflags.mpu_pwm_initalized = true;
			//gpio_set_level(GPIO_NUM_34,0);
			}
		}
		ota = new OTA();
		ota->begin();
		ota->doSoftwareUpdate( display );
	}
	esp_err_t err=ESP_ERR_NOT_FOUND;
	MPU.setBus(i2c);  // set communication bus, for SPI -> pass 'hspi'
	MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address or handle, for SPI -> pass 'mpu_spi_handle'
	err = MPU.reset();
	ESP_LOGI( FNAME,"MPU Probing returned %d MPU enable: %d ", err, attitude_indicator.get() );
	if( err == ESP_OK ){
		if( hardwareRevision.get() < XCVARIO_21 ){
			ESP_LOGI( FNAME,"MPU avail, increase hardware revision to 3 (XCV-21)");
			hardwareRevision.set(XCVARIO_21);  // there is MPU6050 gyro and acceleration sensor, at least we got an XCV-21
		}
		gflags.haveMPU = true;
		// TODO just for flight test. If Magdwick Beta and Mahony Kp are set to zero for gyro drift analysis, diasble MPU temperature control.
		if ( Beta_Magdwick.get() != 0.0  ||  kp_Mahony.get() != 0.0 ) {
			mpu_target_temp = mpu_temperature.get();
		} else {
			mpu_target_temp = -1;
		}
		ESP_LOGI( FNAME,"MPU initialize");
		MPU.initialize();  // this will initialize the chip and set default configurations
		MPU.setSampleRate(400);  // in (Hz)
		MPU.setAccelFullScale(mpud::ACCEL_FS_8G);
		MPU.setGyroFullScale( GYRO_FS );
		MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);  // smoother data
		
		// clear gyro and accel MPU offsets, just in case
		mpud::raw_axes_t zeroBias;
		zeroBias.x = 0.0;
		zeroBias.y = 0.0;
		zeroBias.z = 0.0;
		MPU.setGyroOffset(zeroBias);
		MPU.setAccelOffset(zeroBias);
	
		mpud::raw_axes_t accelRaw;
		delay( 50 );
		
		// TODO set data for Flight Test gliders only. To be removed later in final code using XCVario setup
		polar_wingload.set( WINGLOAD );
		polar_speed1.set( SPEED1 );
		polar_sink1.set( SINK1 );
		polar_speed2.set( SPEED2 );
		polar_sink2.set( SINK2 );
		polar_speed3.set( SPEED3 );
		polar_sink3.set( SINK3 );
		polar_max_ballast.set( MAXBALLAST );
		polar_wingarea.set( SURFACE );
		empty_weight.set( WEIGHT );

		// check GRAVITY and accels offset/gain 
		GRAVITY = gravity.get();
		if(abs(GRAVITY-9.807) > 0.5 ) GRAVITY = 9.807;
		currentAccelBias = accl_bias.get();
		currentAccelGain = accl_gain.get();	
		// Check value just in case FLASH is not correct to reset to neutral values (OK range is +- 2 m/s² for bias and +-20% on gain)
		if ( abs(currentAccelBias.x)>2.0 || abs(currentAccelBias.y)>2.0 || abs(currentAccelBias.z)>2.0 || 
			abs(currentAccelGain.x-1.0)>0.2 || abs(currentAccelGain.y-1.0)>0.2 || abs(currentAccelGain.z-1.0)>0.2 ){
				currentAccelBias.x = 0.0;
				currentAccelBias.y = 0.0;
				currentAccelBias.z = 0.0;
				currentAccelGain.x = 1.0;
				currentAccelGain.y = 1.0;
				currentAccelGain.z = 1.0;
		}
				// TODO just for flight test. If Magdwick Beta and Mahony Kp are set to zero for gyro drift analysis, diasble MPU temperature control.
		if ( Beta_Magdwick.get() != 0.0  ||  kp_Mahony.get() != 0.0 ) {
			// get last known ground gyro biases
			GroundGyroBias = gyro_bias.get();
			// Check value just in case FLASH is not correct to reset to neutral values (OK range is +-0.2 rad/s)
			if ( abs(GroundGyroBias.x)>0.2 || abs(GroundGyroBias.y)>0.2 || abs(GroundGyroBias.z)>0.2 ) {
					GroundGyroBias.x = 0.0;
					GroundGyroBias.y = 0.0;
					GroundGyroBias.z = 0.0;
			}
			NewGroundGyroBias = GroundGyroBias;
		} else {
			GroundGyroBias.x = 0.0;
			GroundGyroBias.y = 0.0;
			GroundGyroBias.z = 0.0;
			NewGroundGyroBias = GroundGyroBias;
			gyro_bias.set(NewGroundGyroBias);		
		}
		// get installation parameters tilt, sway, distCG
		DistCGVario = distCG.get();
		// Check value just in case FLASH is not correct to reset to neutral values (OK range is dist from CG to XCV between 0 an 3 meters and sway and tilt +-0.4 rad)
		if ( DistCGVario < 0.0 || DistCGVario > 3.0 ) DistCGVario = 0.0;
		Sway = sway.get();
		Tilt = tilt.get();
		if ( abs(Sway) > 0.4 || abs(Tilt) > 0.4 ) {
			Sway = 0.0;
			Tilt = 0.0;
		}
		// compute trigonometry
		S_S = sin(Sway);
		C_S = cos(Sway);
		S_T = sin(Tilt);
		C_T = cos(Tilt);
		STmultSS = S_T * S_S;
		STmultCS = S_T * C_S;
		SSmultCT = S_S * C_T;
		CTmultCS = C_T * C_S;
		
		// create mutex for BT synchronization
		BTMutex = xSemaphoreCreateMutex();	
		// create mutex for data synchronization between processIMU and readSensors
		dataMutex = xSemaphoreCreateMutex();
		// create mutex for I2C
		I2CMutex = xSemaphoreCreateMutex();
	
		char ahrs[50];
		float accel = 0;
		for( auto i=0; i<11; i++ ){
			esp_err_t err = MPU.acceleration(&accelRaw);  // fetch raw data from the registers
			if( err != ESP_OK )
				ESP_LOGE(FNAME, "AHRS acceleration I2C read error");
			accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_8G);  // raw data to gravity
			ESP_LOGI( FNAME,"MPU %.2f", accelG[0] );
			delay( 5 );
			if( i>0 )
				accel += sqrt(accelG[0]*accelG[0]+accelG[1]*accelG[1]+accelG[2]*accelG[2]);
		}
		sprintf( ahrs,"AHRS Sensor: OK (%.2f g)", accel/10 );
		display->writeText( line++, ahrs );
		logged_tests += "MPU6050 AHRS test: PASSED\n";
		IMU::init();
		IMU::read();
		// TODO not compatible with flight test code( FNAME,"MPU current offsets accl:%d/%d/%d gyro:%d/%d/%d ZERO:%d", ab.x, ab.y, ab.z, gb.x,gb.y,gb.z, gb.isZero() );
	}
	else{
		ESP_LOGI( FNAME,"MPU reset failed, check HW revision: %d",hardwareRevision.get() );
		if( hardwareRevision.get() >= XCVARIO_21 ) {
			ESP_LOGI( FNAME,"hardwareRevision detected = 3, XCVario-21+");
			display->writeText( line++, "AHRS Sensor: NOT FOUND");
			logged_tests += "MPU6050 AHRS test: NOT FOUND\n";
		}
	}
	char id[16] = { 0 };
	strcpy( id, custom_wireless_id.get().id );
	ESP_LOGI(FNAME,"Custom Wirelss-ID from Flash: %s len: %d", id, strlen(id) );
	if( strlen( id ) == 0 ){
		custom_wireless_id.set( SetupCommon::getDefaultID() ); // Default ID created from MAC address CRC
		ESP_LOGI(FNAME,"Empty ID: Initialize empty Wirelss-ID: %s", custom_wireless_id.get().id );
	}
	ESP_LOGI(FNAME,"Custom Wirelss-ID: %s", custom_wireless_id.get().id );

	String wireless_id;
	if( wireless == WL_BLUETOOTH ) {
		wireless_id="BT ID: ";
		btsender.begin();
	}
	else
		wireless_id="WLAN SID: ";
	wireless_id += SetupCommon::getID();
	display->writeText(line++, wireless_id.c_str() );
	Cipher::begin();
	if( Cipher::checkKeyAHRS() ){
		ESP_LOGI( FNAME, "AHRS key valid=%d", gflags.ahrsKeyValid );
	}else{
		ESP_LOGI( FNAME, "AHRS key invalid=%d, disable AHRS Sensor", gflags.ahrsKeyValid );
		if( attitude_indicator.get() )
			attitude_indicator.set(0);
	}

	ESP_LOGI(FNAME,"Airspeed sensor init..  type configured: %d", airspeed_sensor_type.get() );
	int offset;
	bool found = false;
	if( hardwareRevision.get() >= XCVARIO_21 ){ // autodetect new type of sensors
		ESP_LOGI(FNAME," HW revision 3, check configured airspeed sensor");
		bool valid_config=true;
		switch( airspeed_sensor_type.get() ){
		case PS_TE4525:
			asSensor = new MS4525DO();
			ESP_LOGI(FNAME,"MS4525DO configured");
			break;
		case PS_ABPMRR:
			asSensor = new ABPMRR();
			ESP_LOGI(FNAME,"ABPMRR configured");
			break;
		case PS_MP3V5004:
			asSensor = new MP5004DP();
			ESP_LOGI(FNAME,"PS_MP3V5004 configured");
			break;
		default:
			valid_config = false;
			ESP_LOGI(FNAME,"No valid config found");
			break;
		}
		if( valid_config ){
			ESP_LOGI(FNAME,"There is valid config for airspeed sensor: check this one..");
			asSensor->setBus( &i2c );
			if( asSensor->selfTest( offset ) ){
				ESP_LOGI(FNAME,"Selftest for configured sensor OKAY");
				found = true;
			}
			else
				delete asSensor;
		}
		if( !found ){   // behaves same as above, so we can't detect this, needs to be setup in factory
			ESP_LOGI(FNAME,"Configured sensor not found");
			asSensor = new MS4525DO();
			asSensor->setBus( &i2c );
			ESP_LOGI(FNAME,"Try MS4525");
			if( asSensor->selfTest( offset ) ){
				airspeed_sensor_type.set( PS_ABPMRR );
				found = true;
			}
			else
				delete asSensor;
		}
		if( !found ){
			ESP_LOGI(FNAME,"MS4525 sensor not found");
			asSensor = new ABPMRR();
			asSensor->setBus( &i2c );
			ESP_LOGI(FNAME,"Try ABPMRR");
			if( asSensor->selfTest( offset ) ){
				airspeed_sensor_type.set( PS_ABPMRR );
				found = true;
			}
			else
				delete asSensor;
		}
		if( !found ){
			ESP_LOGI(FNAME,"ABPMRR sensor not found");
			ESP_LOGI(FNAME,"Try MP5004DP");
			asSensor = new MP5004DP();
			if( asSensor->selfTest( offset ) ){
				ESP_LOGI(FNAME,"MP5004DP selfTest OK");
				airspeed_sensor_type.set( PS_MP3V5004 );
				found = true;
			}
			else
				delete asSensor;
		}
	}
	else {
		ESP_LOGI(FNAME,"HW revision 2");
		ESP_LOGI(FNAME,"Aispeed sensor set MP3V5004" );
		if( airspeed_sensor_type.get() != PS_MP3V5004 )
			airspeed_sensor_type.set( PS_MP3V5004 );
		asSensor = new MP5004DP();
		if( asSensor->selfTest( offset ) ){
			ESP_LOGI(FNAME,"MP5004DP selfTest OK");
			airspeed_sensor_type.set( PS_MP3V5004 );
			found = true;
		}
		else
			ESP_LOGI(FNAME,"MP5004DP selfTest FAILED");
	}
	if( found ){
		ESP_LOGI(FNAME,"AS Speed sensors self test PASSED, offset=%d", offset);
		asSensor->doOffset();
		bool offset_plausible = asSensor->offsetPlausible( offset );
		bool ok=false;
		float p=asSensor->readPascal(60, ok);
		if( ok )
			dynamicP = p;
		ias.set( Atmosphere::pascal2kmh( dynamicP ) );
		ESP_LOGI(FNAME,"Aispeed sensor current speed=%f", ias.get() );
		if( !offset_plausible && ( ias.get() < 50 ) ){
			ESP_LOGE(FNAME,"Error: air speed presure sensor offset out of bounds, act value=%d", offset );
			display->writeText( line++, "AS Sensor: NEED ZERO" );
			logged_tests += "AS Sensor offset test: FAILED\n";
			selftestPassed = false;
		}
		else {
			ESP_LOGI(FNAME,"air speed offset test PASSED, readout value in bounds=%d", offset );
			char s[40];
			if( ias.get() > 50 ) {
				sprintf(s, "AS Sensor: %d km/h", (int)(ias.get()+0.5) );
				display->writeText( line++, s );
			}
			else
				display->writeText( line++, "AS Sensor: OK" );
			logged_tests += "AS Sensor offset test: PASSED\n";
		}
	}
	else{
		ESP_LOGE(FNAME,"Error with air speed pressure sensor, no working sensor found!");
		display->writeText( line++, "AS Sensor: NOT FOUND");
		logged_tests += "AS Sensor: NOT FOUND\n";
		selftestPassed = false;
		asSensor = 0;
	}
	ESP_LOGI(FNAME,"Now start T sensor test");
	// Temp Sensor test

	if( !SetupCommon::isClient()  ) {
		ESP_LOGI(FNAME,"Now start T sensor test");
		ds18b20.begin();
		// OAT alpha beta filter parameters
		#define NOAT 6 // Outside Temp AB filter coeff
		#define OATdt 1.0 // average OAT dt		
		#define TempOutliers 20 // 20° maximum variation sample to sample 
		OATemp.ABinit( NOAT, OATdt, TempOutliers );
		temperature = ds18b20.getTemp();
		OATemp.ABupdate( 0.1, temperature );
		if( temperature == DEVICE_DISCONNECTED_C ) {
			ESP_LOGE(FNAME,"Error: Self test Temperatur Sensor failed; returned T=%2.2f", temperature );
			display->writeText( line++, "Temp Sensor: NOT FOUND");
			gflags.validTemperature = false;
			logged_tests += "External Temperature Sensor: NOT FOUND\n";
		} else {
			// read OAT sensor multiple times until temperature is within range and stable
			for ( int nbsample = 0; nbsample < 20 && !OATemp.Stable(); nbsample++ ) {
				temperature = ds18b20.getTemp();
				// if temperature out of range, re-init the filter
				if ( temperature < -45.0 || temperature > 65.0 ) OATemp.ABinit( NOAT, OATdt, TempOutliers ); else OATemp.ABupdate( 0.1, temperature );
				delay( 100 );
			}
			if ( OATemp.Stable() ) {
				ESP_LOGI(FNAME,"Self test Temperature Sensor PASSED; returned T=%2.2f", temperature );
				display->writeText( line++, "Temp Sensor: OK");
				gflags.validTemperature = true;
				logged_tests += "External Temperature Sensor:PASSED\n";
				OAT.set( OATemp.ABfilt() ); 
			} else {
				ESP_LOGI(FNAME,"Self test Temperatur Sensor FAILED; not stable returned T=%2.2f", temperature );
				display->writeText( line++, "Temp Sensor: FAILED use 15°C");
				gflags.validTemperature = true;
				logged_tests += "External Temperature Sensor:UNSTABLE use 15°C\n";
				OAT.set( 15.0 );
			}
		}
		ESP_LOGI(FNAME,"End T sensor test");
	}
	ESP_LOGI(FNAME,"Absolute pressure sensors init, detect type of sensor type..");

	float ba_t, ba_p, te_t, te_p;
	SPL06_007 *splBA = new SPL06_007( SPL06_007_BARO );
	SPL06_007 *splTE = new SPL06_007( SPL06_007_TE );
	splBA->setBus( &i2c );
	splTE->setBus( &i2c );
	bool baok =  splBA->begin();
	bool teok =  splTE->begin();
	if( baok || teok ){
		ESP_LOGI(FNAME,"SPL06_007 type detected");
		i2c.begin(GPIO_NUM_21, GPIO_NUM_22, 100000 );  // higher speed, we have 10K pullups on that board
		baroSensor = splBA;
		teSensor = splTE;
	}
	else{
		delete splBA;
		ESP_LOGI(FNAME,"No SPL06_007 chip detected, now check BMP280");
		BME280_ESP32_SPI *bmpBA = new BME280_ESP32_SPI();
		BME280_ESP32_SPI *bmpTE= new BME280_ESP32_SPI();
		bmpBA->setSPIBus(SPI_SCLK, SPI_MOSI, SPI_MISO, CS_bme280BA, FREQ_BMP_SPI);
		bmpTE->setSPIBus(SPI_SCLK, SPI_MOSI, SPI_MISO, CS_bme280TE, FREQ_BMP_SPI);
		bmpTE->begin();
		bmpBA->begin();
		baroSensor = bmpBA;
		teSensor = bmpTE;
		gpio_set_pull_mode(CS_bme280BA, GPIO_PULLUP_ONLY );
		gpio_set_pull_mode(CS_bme280TE, GPIO_PULLUP_ONLY );

	}
	bool tetest=true;
	bool batest=true;
	delay(200);

	if( !baroSensor->selfTest( ba_t, ba_p)  ) {
		ESP_LOGE(FNAME,"HW Error: Self test Barometric Pressure Sensor failed!");
		display->writeText( line++, "Baro Sensor: NOT FOUND");
		selftestPassed = false;
		batest=false;
		logged_tests += "Baro Sensor Test: NOT FOUND\n";
	}
	else {
		ESP_LOGI(FNAME,"Baro Sensor test OK, T=%f P=%f", ba_t, ba_p);
		display->writeText( line++, "Baro Sensor: OK");
		logged_tests += "Baro Sensor Test: PASSED\n";
	}
	if( !teSensor->selfTest(te_t, te_p) ) {
		ESP_LOGE(FNAME,"HW Error: Self test TE Pressure Sensor failed!");
		display->writeText( line++, "TE Sensor: NOT FOUND");
		selftestPassed = false;
		tetest=false;
		logged_tests += "TE Sensor Test: NOT FOUND\n";
	}
	else {
		ESP_LOGI(FNAME,"TE Sensor test OK,   T=%f P=%f", te_t, te_p);
		display->writeText( line++, "TE Sensor: OK");
		logged_tests += "TE Sensor Test: PASSED\n";
	}
	if( tetest && batest ) {
		ESP_LOGI(FNAME,"Both absolute pressure sensor TESTs SUCCEEDED, now test deltas");
		if( (abs(ba_t - te_t) >4.0)  && ( ias.get() < 50 ) ) {   // each sensor has deviations, and new PCB has more heat sources
			selftestPassed = false;
			ESP_LOGE(FNAME,"Severe T delta > 4 °C between Baro and TE sensor: °C %f", abs(ba_t - te_t) );
			display->writeText( line++, "TE/Baro Temp: Unequal");
			logged_tests += "TE/Baro Sensor T diff. <4°C: FAILED\n";
		}
		else{
			ESP_LOGI(FNAME,"Abs p sensors temp. delta test PASSED, delta: %f °C",  abs(ba_t - te_t));
			// display->writeText( line++, "TE/Baro Temp: OK");
			logged_tests += "TE/Baro Sensor T diff. <2°C: PASSED\n";
		}
		float delta = 2.5; // in factory we test at normal temperature, so temperature change is ignored.
		if( abs(factory_volt_adjust.get() - 0.00815) < 0.00001 )
			delta += 1.8; // plus 1.5 Pa per Kelvin, for 60K T range = 90 Pa or 0.9 hPa per Sensor, for both there is 2.5 plus 1.8 hPa to consider
		if( (abs(ba_p - te_p) >delta)  && ( ias.get() < 50 ) ) {
			selftestPassed = false;
			ESP_LOGI(FNAME,"Abs p sensors deviation delta > 2.5 hPa between Baro and TE sensor: %f", abs(ba_p - te_p) );
			display->writeText( line++, "TE/Baro P: Unequal");
			logged_tests += "TE/Baro Sensor P diff. <2hPa: FAILED\n";
		}
		else
			ESP_LOGI(FNAME,"Abs p sensor deta test PASSED, delta: %f hPa", abs(ba_p - te_p) );
		// display->writeText( line++, "TE/Baro P: OK");
		logged_tests += "TE/Baro Sensor P diff. <2hPa: PASSED\n";

	}
	else
		ESP_LOGI(FNAME,"Absolute pressure sensor TESTs failed");

	bmpVario.begin( teSensor, baroSensor, &Speed2Fly );
	bmpVario.setup();
	esp_task_wdt_reset();
	ESP_LOGI(FNAME,"Audio begin");
	Audio::begin( DAC_CHANNEL_1 );
	ESP_LOGI(FNAME,"Poti and Audio test");
	if( !Audio::selfTest() ) {
		ESP_LOGE(FNAME,"Error: Digital potentiomenter selftest failed");
		display->writeText( line++, "Digital Poti: Failure");
		selftestPassed = false;
		logged_tests += "Digital Audio Poti test: FAILED\n";
	}
	else{
		ESP_LOGI(FNAME,"Digital potentiometer test PASSED");
		logged_tests += "Digital Audio Poti test: PASSED\n";
		display->writeText( line++, "Digital Poti: OK");
	}

	// 2021 series 3, or 2022 model with new digital poti CAT5171 also features CAN bus
	String resultCAN;
	if( Audio::haveCAT5171() ) // todo && CAN configured
	{
		CAN = new CANbus();
		if( CAN->selfTest(false) ){  // series 2023 has fixed slope control, prio slope bit for AHRS temperature control
			resultCAN = "OK";
			ESP_LOGE(FNAME,"CAN Bus selftest (no RS): OK");
			logged_tests += "CAN Interface: OK\n";
			if( hardwareRevision.get() != XCVARIO_23 ){
				ESP_LOGI(FNAME,"CAN Bus selftest without RS control OK: set hardwareRevision 5 (XCV-23)");
				hardwareRevision.set(XCVARIO_23);  // XCV-23, including AHRS temperature control
			}
		}
		else{
			if( CAN->selfTest(true) ){  // if slope bit is to be handled, there is no temperature control
				resultCAN = "OK";
				ESP_LOGE(FNAME,"CAN Bus selftest RS: OK");
				logged_tests += "CAN Interface: OK\n";
				if( hardwareRevision.get() != XCVARIO_22 ){
					hardwareRevision.set(XCVARIO_22);  // XCV-22, CAN but no AHRS temperature control
				}
			}
			else{
				resultCAN = "FAIL";
				logged_tests += "CAN Bus selftest: FAILED\n";
				ESP_LOGE(FNAME,"Error: CAN Interface failed");
			}
		}
	}

	float bat = Battery.get(true);
	if( bat < 1 || bat > 28.0 ){
		ESP_LOGE(FNAME,"Error: Battery voltage metering out of bounds, act value=%f", bat );
		if( resultCAN.length() )
			display->writeText( line++, "Bat Meter/CAN: ");
		else
			display->writeText( line++, "Bat Meter/CAN: Fail/" + resultCAN );
		logged_tests += "Battery Voltage Sensor: FAILED\n";
		selftestPassed = false;
	}
	else{
		ESP_LOGI(FNAME,"Battery voltage metering test PASSED, act value=%f", bat );
		if( resultCAN.length() )
			display->writeText( line++, "Bat Meter/CAN: OK/"+ resultCAN );
		else
			display->writeText( line++, "Bat Meter: OK");
		logged_tests += "Battery Voltage Sensor: PASSED\n";
	}

	Serial::begin();
	// Factory test for serial interface plus cable
	String result("Serial ");
	if( Serial::selfTest( 1 ) )
		result += "S1 OK";
	else
		result += "S1 FAIL";
	if( (hardwareRevision.get() >= XCVARIO_21) && serial2_speed.get() ){
		if( Serial::selfTest( 2 ) )
			result += ",S2 OK";
		else
			result += ",S2 FAIL";
	}
	if( abs(factory_volt_adjust.get() - 0.00815) < 0.00001 ){
		display->writeText( line++, result.c_str() );
	}
	Serial::taskStart();

	if( wireless == WL_BLUETOOTH ) {
		if( btsender.selfTest() ){
			display->writeText( line++, "Bluetooth: OK");
			logged_tests += "Bluetooth test: PASSED\n";
		}
		else{
			display->writeText( line++, "Bluetooth: FAILED");
			logged_tests += "Bluetooth test: FAILED\n";
		}
	}else if ( wireless == WL_WLAN_MASTER || wireless == WL_WLAN_STANDALONE ){
		WifiApp::wifi_init_softap();
	}
	// 2021 series 3, or 2022 model with new digital poti CAT5171 also features CAN bus
	if(  can_speed.get() != CAN_SPEED_OFF && (resultCAN == "OK") && CAN )
	{
		ESP_LOGI(FNAME, "Now start CAN Bus Interface");
		CAN->begin();  // start CAN tasks and driver
	}

	if( compass_enable.get() == CS_CAN ){
		ESP_LOGI( FNAME, "Magnetic sensor type CAN");
		compass = new Compass( 0 );  // I2C addr 0 -> instantiate without I2C bus and local sensor
	}
	else if( compass_enable.get() == CS_I2C ){
		ESP_LOGI( FNAME, "Magnetic sensor type I2C");
		compass = new Compass( 0x0D, ODR_50Hz, RANGE_2GAUSS, OSR_512, &i2c_0 );
	}
	// magnetic sensor / compass selftest
	if( compass ) {
		compass->begin();
		ESP_LOGI( FNAME, "Magnetic sensor enabled: initialize");
		err = compass->selfTest();
		if( err == ESP_OK )		{
			// Activate working of magnetic sensor
			ESP_LOGI( FNAME, "Magnetic sensor selftest: OKAY");
			display->writeText( line++, "Compass: OK");
			logged_tests += "Compass test: OK\n";
		}
		else{
			ESP_LOGI( FNAME, "Magnetic sensor selftest: FAILED");
			display->writeText( line++, "Compass: FAILED");
			logged_tests += "Compass test: FAILED\n";
			selftestPassed = false;
		}
		compass->start();  // start task
	}

	Speed2Fly.begin();
	Version myVersion;
	ESP_LOGI(FNAME,"Program Version %s", myVersion.version() );
	ESP_LOGI(FNAME,"%s", logged_tests.c_str());
	if( !selftestPassed )
	{
		ESP_LOGI(FNAME,"\n\n\nSelftest failed, see above LOG for Problems\n\n\n");
		display->writeText( line++, "Selftest FAILED");
		if( !Rotary.readSwitch() )
			sleep(4);
	}
	else{
		ESP_LOGI(FNAME,"\n\n\n*****  Selftest PASSED  ********\n\n\n");
		display->writeText( line++, "Selftest PASSED");
		if( !Rotary.readSwitch() )
			sleep(2);
	}
	if( Rotary.readSwitch() )
	{
		LeakTest::start( baroSensor, teSensor, asSensor );
	}
	Menu->begin( display, baroSensor, &Battery );

	if ( wireless == WL_WLAN_CLIENT || the_can_mode == CAN_MODE_CLIENT ){
		ESP_LOGI(FNAME,"Client Mode");
	}
	else if( ias.get() < 50.0 ){
		ESP_LOGI(FNAME,"Master Mode: QNH Autosetup, IAS=%3f (<50 km/h)", ias.get() );
		// QNH autosetup
		float ae = elevation.get();
		float qnh_best = QNH.get();
		bool ok;
		baroP = baroSensor->readPressure(ok);
		if( ae > 0 ) {
			float step=10.0; // 80 m
			float min=1000.0;
			for( float qnh = 870; qnh< 1085; qnh+=step ) {
				float alt = 0;
				if( Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL )
					alt = alt_external + (qnh  - 1013.25) * 8.2296;  // correct altitude according to ISA model = 27ft / hPa
				else
					alt = baroSensor->readAltitude( qnh, ok);
				float diff = alt - ae;
				// ESP_LOGI(FNAME,"Alt diff=%4.2f  abs=%4.2f", diff, abs(diff) );
				if( abs( diff ) < 100 )
					step=1.0;  // 8m
				if( abs( diff ) < 10 )
					step=0.05;  // 0.4 m
				if( abs( diff ) < abs(min) ) {
					min = diff;
					qnh_best = qnh;
					// ESP_LOGI(FNAME,"New min=%4.2f", min);
				}
				if( diff > 1.0 ) // we are ready, values get already positive
					break;
				delay(50);
			}
			ESP_LOGI(FNAME,"Auto QNH=%4.2f\n", qnh_best);
			QNH.set( qnh_best );
		}
		display->clear();
		if( NEED_VOLTAGE_ADJUST ){
			ESP_LOGI(FNAME,"Do Factory Voltmeter adj");
			SetupMenuValFloat::showMenu( 0.0, SetupMenuValFloat::meter_adj_menu );
		}else{
			SetupMenuValFloat *qnh_menu = SetupMenu::createQNHMenu();
			SetupMenuValFloat::showMenu( qnh_best, qnh_menu );
		}
	}
	else
	{
		gflags.inSetup = false;
		display->clear();
	}

	if ( flap_enable.get() ) {
		Flap::init(MYUCG);
	}
	if( hardwareRevision.get() == XCVARIO_20 ){
		Rotary.begin( GPIO_NUM_4, GPIO_NUM_2, GPIO_NUM_0);  // XCV-20 uses GPIO_2 for Rotary
	}
	else {
		Rotary.begin( GPIO_NUM_36, GPIO_NUM_39, GPIO_NUM_0);
		gpio_pullup_en( GPIO_NUM_34 );
		if( gflags.haveMPU && HAS_MPU_TEMP_CONTROL && !gflags.mpu_pwm_initalized  ){ // series 2023 does not have slope support on CAN bus but MPU temperature control
			MPU.pwm_init();
			gflags.mpu_pwm_initalized = true;
			///gpio_set_level(GPIO_NUM_34,0);

		}

	}
	delay( 100 );



	if ( SetupCommon::isClient() ){
		if( wireless == WL_WLAN_CLIENT ){
			display->clear();

			int line=1;
			display->writeText( line++, "Wait for WiFi Master" );
			char mxcv[30] = "";
			if( master_xcvario.get() != 0 ){
				sprintf( mxcv+strlen(mxcv), "XCVario-%d", (int) master_xcvario.get() );
				display->writeText( line++, mxcv );
			}
			line++;
			std::string ssid = WifiClient::scan( master_xcvario.get() );
			if( ssid.length() ){
				display->writeText( line++, "Master XCVario found" );
				char id[30];
				sprintf( id, "Wifi ID: %s", ssid.c_str() );
				display->writeText( line++, id );
				display->writeText( line++, "Now start, sync" );
				WifiClient::start();
				delay( 5000 );
				gflags.inSetup = false;
				display->clear();
			}
			else{
				display->writeText( 3, "Abort Wifi Scan" );
			}
		}
		else if( the_can_mode == CAN_MODE_CLIENT ){
			display->clear();
			display->writeText( 1, "Wait for CAN Master" );
			while( 1 ) {
				if( CAN && CAN->connectedXCV() ){
					display->writeText( 3, "Master XCVario found" );
					display->writeText( 4, "Now start, sync" );
					delay( 5000 );
					gflags.inSetup = false;
					display->clear();
					break;
				}
				delay( 100 );
				if( Rotary.readSwitch() ){
					display->writeText( 3, "Abort CAN bus wait" );
					break;
				}
			}
		}
	}
	if( screen_centeraid.get() ){
		centeraid = new CenterAid( MYUCG );
	}
	
	xTaskCreatePinnedToCore(&processIMU, "processIMU", 4096, NULL, 15, &mpid, 0);
	
	if( SetupCommon::isClient() ){
		xTaskCreatePinnedToCore(&clientLoop, "clientLoop", 4096, NULL, 11, &bpid, 0);
		xTaskCreatePinnedToCore(&audioTask, "audioTask", 4096, NULL, 11, &apid, 0);
	}
	else {
		xTaskCreatePinnedToCore(&readSensors, "readSensors", 5120, NULL, 14, &bpid, 0);

	}
	xTaskCreatePinnedToCore(&readTemp, "readTemp", 3000, NULL, 5, &tpid, 0);       // increase stack by 500 byte
	xTaskCreatePinnedToCore(&drawDisplay, "drawDisplay", 6144, NULL, 7, &dpid, 0); // increase stack by 1K

	Audio::startAudio();
}

extern "C" void  app_main(void)
{
	// Init timer infrastructure
	esp_timer_init();

	Audio::boot();
	ESP_LOGI(FNAME,"app_main" );
	ESP_LOGI(FNAME,"Now init all Setup elements");
	bool setupPresent;
	SetupCommon::initSetup( setupPresent );
	Cipher::begin();
	if( !setupPresent ){
		if( Cipher::init() )
			attitude_indicator.set(1);
	}
	else
		ESP_LOGI(FNAME,"Setup already present");
	esp_log_level_set("*", ESP_LOG_INFO);

	system_startup( 0 );
	vTaskDelete( NULL );
}
