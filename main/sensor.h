#ifndef _SENSOR_H_
#define _SENSOR_H_

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// compile options
//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
#define FTVERSION 13
#define SOFTVERSION 31
//
#define COMPUTEBIAS   // code to estimate gyro bias
//
//#define COMPUTEWIND1   // code to compute wind with GNSS
//
//#define COMPUTEWIND2   // code to compute wind with GNSS
//
#define FILTERMPU  // code to filter MPU data at ~ 7 Hz


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// glider specific parameters
//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
#define LS6
//#define VENTUS3


#ifdef LS6
#define ALLYSTAR // code for Allystar GNSS
#endif

#ifdef VENTUS3
#define RTK // code for RTK PX1122R GNSS
#endif

#include "MPU.hpp"        // main file, provides the class itself
#include "AnalogInput.h"
#include "Protocols.h"
#include "MP5004DP.h"
#include "MS4525DO.h"
#include "IpsDisplay.h"
#include "ESPRotary.h"
#include "Compass.h" // 3-Axis Magnetic Sensor
#include <hal/gpio_types.h>
#include "SetupMenu.h"
#include "S2F.h"
#include "StraightWind.h"
#include "DataMonitor.h"
#include "AdaptUGC.h"
#include "canbus.h"
#include "CenterAid.h"
#include "vector_3d.h"
#include "BMPVario.h"
#include "AirspeedSensor.h"

// Display 4 Wire SPI and Display CS
#define RESET_Display  GPIO_NUM_5       // Reset pin for Display
#define CS_Display     GPIO_NUM_13      // CS pin 13 is for Display
#define SPI_SCLK       GPIO_NUM_14      // SPI Clock pin 14
#define SPI_DC         GPIO_NUM_15      // SPI Data/Command pin 15
#define SPI_MOSI       GPIO_NUM_27      // SPI SDO Master Out Slave In pin
#define SPI_MISO       GPIO_NUM_32      // SPI SDI Master In Slave Out


#define PERIOD10HZ 0.1 // constant for filters in the 10 Hz loop
#define PERIOD40HZ 0.025 // constant for filters in the 40 Hz loop

typedef struct global_flags{
	bool inSetup :1;
	bool haveMPU :1;
	bool ahrsKeyValid  :1;
	bool gload_alarm :1;
	bool  standard_setting :1;
	bool stall_warning_active :1;
	bool stall_warning_armed :1;
	bool flarmWarning :1 ;
	bool gLoadDisplay :1;
	bool gear_warning_active :1;
	bool flarmDownload :1 ; // Flarm IGC download flag
	bool validTemperature :1 ;
	bool mpu_pwm_initalized: 1;
	bool gear_warn_external :1;
} t_global_flags;

extern t_global_flags gflags;
extern BMPVario bmpVario;
extern CANbus* CAN;
extern StraightWind theWind;
extern xSemaphoreHandle xMutex;
extern int active_screen;
extern CenterAid *centeraid;
extern AirspeedSensor *asSensor;

extern SetupMenu  *Menu;
extern xSemaphoreHandle display_mutex;

extern e_wireless_type wireless;

// MPU6050 sensor
extern mpud::float_axes_t accelG;
extern mpud::float_axes_t gyroDPS;

extern float getTAS();
void doAudio( float te );

extern I2C_t& i2c;
extern I2C_t& i2c_0;
extern AnalogInput *AnalogInWk;

extern float airspeed;
extern float aTE;
extern float tas;
extern float cas;
extern float aTES2F;
extern float as2f;
extern float s2f_delta;
extern float polar_sink;
extern float alt_external;
extern float wksensor;
extern float slipAngle;

extern S2F Speed2Fly;
extern float meanClimb;
extern Protocols OV;
extern int the_can_mode;

extern IpsDisplay *display;

extern ESPRotary Rotary;

extern DataMonitor DM;

extern xSemaphoreHandle spiMutex;

extern Compass *compass;

class AdaptUGC;
extern AdaptUGC *MYUCG;

extern vector_ijk gravity_vector;

#define NEED_VOLTAGE_ADJUST (abs(factory_volt_adjust.get() - 0.00815) < 0.00001)

extern float mpu_target_temp;

extern MPU_t MPU;

// There is no temperature control for XCV hardware < 23, GPIO Pin there is wired to CAN slope control
#define HAS_MPU_TEMP_CONTROL (CAN && !CAN->hasSlopeSupport())

extern bool IMUstream;
extern bool SENstream;
extern bool CALstream;
extern bool CALfirstpass;
extern bool TSTstream;
extern bool LABtest;
extern bool AHRSstream;
extern float localGravity;
extern float NEnergy;
extern float alphaEnergy;
extern float betaEnergy;
extern float PeriodVelbi;
extern float LastPeriodVelbi;
extern float Mahonykp;
extern float MagdwickBeta;
extern float ALTbiN;
extern float TASbiN;
extern bool NALTbiTASbiChanged;
extern float opt_TE;

extern double RTKtime;
extern float RTKEvel;
extern float RTKNvel;
extern float RTKUvel;
extern char RTKmode;
extern float RTKage;
extern float RTKratio;
extern float RTKEproj;
extern float RTKNproj;
extern float RTKUproj;
extern float RTKheading;

// MOD#5 add Allystar TAU1201 velocity
extern double Allytime;
extern float AllyvelN;
extern float AllyvelE;
extern float AllyvelU;
extern float Allyvel3D;
extern float Allyvel2D;
// MOD#5 add Allystar TAU1201 velocity

extern float accelNEDBODYzNorm;

#endif
