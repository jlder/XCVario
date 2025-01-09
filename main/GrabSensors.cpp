		
	while (1) {
		countIMU++;
		TickType_t xLastWakeTime_mpu =xTaskGetTickCount();
		
		// process time measurement
		ProcessTimeGrabSensors = (esp_timer_get_time()/1000.0);
		
		// get gyro data
		esp_err_t errMPU = MPU.rotation(&GyroRaw);// read raw gyro data
 		if( errMPU == ESP_OK ){ 
			// compute precise gyro dt in mili second, between current and previous sample
			prevGyroTime = GyroTime;
			GyroTime = esp_timer_get_time()/1000.0; // record time of gyro measurement in milli second
			dtGyr = (GyroTime - prevGyroTime) / 1000.0; // period between last two valid samples in mili second
			// If dtGyr is negative or abnormaly high, skip filters, baro inertial and AHRS updates using dtGyr = 0
			if (dtGyr < 0.0 or dtGyr > 0.075) dtGyr = 0.0;
			// Convert raw gyro to rad/second
			GyroRPS = mpud::gyroRadPerSec(GyroRaw, GYRO_FS); // convert raw gyro to Gyro_FS full scale in radians per second
			// convert from gyro to MPU frame (NED) and remove bias estimated when MPU is stable on ground
			GyroMPUx = -(GyroRPS.z - GroundGyroBiasz);
			GyroMPUy = -(GyroRPS.y - GroundGyroBiasy);
			GyroMPUz = -(GyroRPS.x - GroundGyroBiasx);				
			// convert from MPU to BODY frame and remove bias estimated in flight
			GyroBODYx = C_T * gyroMPUx + STmultSS * gyroMPUy + STmultCS * gyroMPUz; // + BiasQuatGx;
			GyroBODYy = C_S * gyroMPUy - S_S * gyroMPUz; // + BiasQuatGy;
			GyroBODYz = -S_T * gyroMPUx + SSmultCT  * gyroMPUy + CTmultCS * gyroMPUz; // + BiasQuatGz;
			// update gyro filters
			Gyrox.ABupdate(dtGyr, gyroBODYx );
			Gyroy.ABupdate(dtGyr, gyroBODYy );			
			Gyroz.ABupdate(dtGyr, gyroBODYz );			
		}
		// get accel data
		errMPU = MPU.acceleration(&accelRaw);// read raw gyro data
		if( errMPU == ESP_OK ){ // read raw acceleration
			// compute precise accel dt in mili second, between current and previous sample
			prevAccTime = AccTime;
			AccTime = esp_timer_get_time()/1000.0; // record time of accel measurement in milli second
			dtAcc = (AccTime - prevAccTime) / 1000.0; // period between last two valid samples in mili second
			// If dtAcc is negative or abnormaly high, skip filters, baro inertial and AHRS updates using dtAcc = 0
			if (dtAcc < 0.0 or dtAcc > 0.075) dtAcc = 0.0;		
			accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_8G);  // Convert raw data to to 8G full scale
			// convert accels to MPU m/s²
			AccMPUx = ((-accelG.z*9.807) - currentAccBiasx ) * currentAccGainx;
			AccMPUy = ((-accelG.y*9.807) - currentAccBiasy ) * currentAccGainy;
			AccMPUz = ((-accelG.x*9.807) - currentAccBiasz ) * currentAccGainz;
			// convert from MPU to BODY
			AccBODYx = C_T * AccMPUx + STmultSS * AccMPUy + STmultCS * AccMPUz + ( Gyrox.ABFilt() * Gyrox.ABFilt() + Gyroz.ABFilt() * Gyroz.ABFilt() ) * DistCGVario ;
			AccBODYy = C_S * AccMPUy - S_S * AccMPUz;				
			AccBODYz = -S_T * AccMPUx + SSmultCT * AccMPUy + CTmultCS * AccMPU.z ;
			// Correct acceleration due to distance from CG
			AccBODYx = AccBODYx + ( Gyrox.ABFilt() * Gyrox.ABFilt() + Gyroz.ABFilt() * Gyroz.ABFilt() ) * DistCGVario ;			
			// update gyro filters
			Accx.ABupdate( dtAcc, AccBODYx );
			Accy.ABupdate( dtAcc, AccBODYy  );
			Accz.ABupdate( dtAcc, AccBODYz );
			Nz.Set( -Accz.ABfilt() / GRAVITY );
		}

		// get static pressure
		bool ok=false;
		float p = 0;
		Prevp = statP.Get();
		p = baroSensor->readPressure(ok);
		if ( ok ) {
			// compute precise Stat dt in mili second, between current and previous sample
			prevStatTime = StatTime;
			StatTime = esp_timer_get_time()/1000.0; // record time of static measurement in milli second
			dtStat = (StatTime - prevStatTime) / 1000.0; // period between last two valid samples in mili second
			// If dtStat is negative or abnormaly high, skip filters using dtStat = 0
			if (dtStat < 0.0 or dtStat > 0.075) dtStat = 0.0;			
			prevstatTime = statTime;
			statTime = esp_timer_get_time()/1000; // record static time in milli second
			dtStat = (statTime - prevstatTime) / 1000.0; // period between last two valid static pressure samples in second	
			StatP.Set( p );
			baroP = p;	// for compatibility with Eckhard code
			Prevp = p;
		} else {
			StatP.Set( Prevp );
			baroP = Prevp;
		}
		
		// get Te pressure
		p = teSensor->readPressure(ok);
		if ( ok ) {
			TeP.Set( p );
		}
		
		// get dynamic pressure
		if( asSensor ) {
			PrevdynP = DynP.Get();
			dp =  asSensor->readPascal(0, ok);
			if ( dp < 0 ) dp = 0.0;
		}
		if( ok ) {			
			// compute precise Dp dt in mili second, between current and previous sample
			prevDpTime = DpTime;
			DpTime = esp_timer_get_time()/1000.0; // record time of Dp measurement in milli second
			dtDp = (DpTime - prevDpTime) / 1000.0; // period between last two valid samples in mili second
			// If dtDp is negative or abnormaly high, skip filters using dtDp = 0
			if (dtDp < 0.0 or dtDp > 0.075) dtDp = 0.0;
			DynP.Set( dp );
		}
		else {
			dynP.Set( PrevdynP );
		}

		dynamicP = dynP.Get(); // for compatibility with Eckhard code
		if ( dynamicP < 60.0 ) dynamicP = 0.0;

		// compute CAS and update filter
		CAS.ABupdate( dtDdynP, sqrt(2 * dynP.Get() / RhoSLISA) );

		// compute altitude and update altitude filter
		ALT.ABupdate( dtStat, (1.0 - pow( (StatP.Get()-(QNH.get()-1013.25)) * 0.000986923 , 0.1902891634 ) ) * 44330.7692307249 );
		
		if (!CALstream) { // if not in calibration mode
			// estimate gravity using accels in body frame taking into account centrifugal corrections
			GravAccx = Accx.ABfilt()- Gyroy.ABfilt() * TASbi.Get() * AoA.Get() + Gyroz.ABfilt() * TASbi.Get() * AoB.Get() - Ubi.prim();
			GravAccy = Accy.ABfilt()- Gyroz.ABfilt() * TASbi.Get() + Gyrox.ABfilt() * TASbi.Get() * AoA.Get() - Vbi.prim();
			GravAccz = Accz.ABfilt()+ Gyroy.ABfilt() * TASbi.Get() - Gyrox.ABfilt() * TASbi.Get() * AoB.Get() - Wbi.prim();

			// update Magdwick AHRS
			AHRS.update( dtGyr, Gyrox.ABfilt(), Gyroy.ABfilt(), Gyroz.ABfilt(), -GravAccx, -GravAccy, -GravAccz );

			// Update roll and pitch alpha beta filter. This provides Roll and Pitch derivatives for bias analysis 
			RollAHRS.ABupdate( dtGyr, AHRS.GetRoll() );
			PitchAHRS.ABupdate( dtGyr, AHRS.GetPitch() );	
				
			// compute kinetic accelerations using accelerations, corrected with gravity from AHRS and centrifugal accels
			UiPrim.ABupdate( accelNEDBODYx.ABfilt()- AHRS.Gravx() - Gyroy.ABfilt() * TASbi.Get() * AoA.Get() + Gyroz.ABfilt() * TASbi.Get() * AoB.Get() );
			ViPrim.ABupdate( accelNEDBODYy.ABfilt()- AHRS.Gravy() - Gyroz.ABfilt() * TASbi.Get() + Gyrox.ABfilt() * TASbi.Get() * AoA.Get() );			
			WiPrim.ABupdate( accelNEDBODYz.ABfilt()- AHRS.Gravz() + Gyroy.ABfilt() * TASbi.Get() - Gyrox.ABfilt() * TASbi.Get() * AoB.Get() );
			
			// alternate kinetic accel solution using 3D baro inertial speeds
			// UiPrim = accelNEDBODYx.ABfilt()- GravIMU.x - gyroCorr.y * Wbi + gyroCorr.z * Vbi;
			// ViPrim = accelNEDBODYy.ABfilt()- GravIMU.y - gyroCorr.z * Ubi + gyroCorr.x * Wbi;			
			// WiPrim = accelNEDBODYz.ABfilt()- GravIMU.z + gyroCorr.y * Ubi - gyroCorr.x * Vbi;			

			// Compute baro interial acceleration ( complementary filter between inertial accel derivatives and baro speed derivates )
			UbiPrim.update( dtGyr, UiPrim.ABprim(), Ub.ABprim() );
			VbiPrim.update( dtGyr, ViPrim.ABprim(), Vb.ABprim());			
			WbiPrim.update( dtGyr, WiPrim.ABprim(), Wb.ABprim());
			
			// Compute baro interial velocity ( complementary filter between baro inertial acceleration and baro speed )
			Ubi.update( dtGyr, UbiPrim.get(), Ub.get() );
			Vbi.update( dtGyr, VbiPrim.get(), Vb.get() );				
			Wbi.update( dtGyr, WbiPrim.get(), Wb.get() );
			
			// Gyro bias and local gravity estimates when TAS < 15 m/s and the vario is considered potentially stable on ground
			if ( TAS.Get() < 15.0 ) {
				// Estimate gyro bias and gravity, except if doing Lab test then only one estimation is performed
			}
		} else {
			// Accels calibration
		}
				
		// compute process time and report
		ProcessTimeGrabSensors = (esp_timer_get_time()/1000.0) - ProcessTimeGrabSensors;
		if ( ProcessTimeGrabSensors > 8 && TAS.Get() < 15.0 ) {
			ESP_LOGI(FNAME,"processIMU: %i / 25", (int16_t)(ProcessTimeIMU) );
		}
		
		mtick++;
		vTaskDelayUntil(&xLastWakeTime_mpu, 25/portTICK_PERIOD_MS);  // 25 ms = 40 Hz loop
		
		// test stack every second and report
		if( (mtick % 40) == 0) {  
			if( uxTaskGetStackHighWaterMark( mpid ) < 1024 && TAS.Get() < 15.0 )
				 ESP_LOGW(FNAME,"Warning MPU and sensor task stack low: %d bytes", uxTaskGetStackHighWaterMark( mpid ) );
		}
	}		
}					
