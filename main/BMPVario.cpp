#include "BMPVario.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <atomic>
#include <logdef.h>
#include "S2F.h"
#include "AverageVario.h"

const double sigmaAdjust = 255 * 2.0/33;  // 2 Vss
int BMPVario::holddown = 0;

void BMPVario::begin( PressureSensor *te, PressureSensor *baro, S2F *aS2F  ) {
	_sensorTE = te;
	_sensorBARO = baro;
	_init = true;

	_S2FTE = 0.0;
	myS2F = aS2F;
}

double BMPVario::readAVGTE() {
	return _avgTE;
}

float BMPVario::readS2FTE() {
	_S2FTE += ( (float)_TEF - _S2FTE ) * ( 1/(s2f_delay.get() * 10) );
	return _S2FTE;
}


uint64_t lastrts = 0;
void BMPVario::setup() {
	_qnh = QNH.get();
	_damping = vario_delay.get();
	_filter_len = 10;
	lastrts = esp_timer_get_time();
}


double BMPVario::readTE( float tasmps, float ptasmps ) {
	if ( _test )     // we are in testmode, just return what has been set
		return _TEF;
	bool success;
	bmpTemp = _sensorTE->readTemperature( success );
	// ESP_LOGI(FNAME,"BMP temp=%0.1f", bmpTemp );
	if( te_comp_enable.get() ) {
		_currentAlt = _sensorBARO->readAltitude(_qnh, success );
		if( !success )
			_currentAlt = lastAltitude;  // ignore readout when failed
		// JLD
		//  TE electronic vario calculation using ptasmps which is filtered in function of electronic filter: te_comp_adjust: 0= no compensation  x= time during which wind gradient and turbulence are filtered
		float ealt = (ptasmps*ptasmps)/19.62;  // altitude variation from kinetic energy ~ h = ptas²/2g
		_currentAlt += ealt; // add kenetic altitude correction to altitude from static port
		// JLD
		ESP_LOGD(FNAME,"Energiehöhe @%0.1f km/h: %0.1f", tasmps*3.6, ealt);
	}
	else{
		_currentAlt = _sensorTE->readAltitude(_qnh, success );
		if( !success )
			_currentAlt = lastAltitude;  // ignore readout when failed
		// JLD
		// implement electronic compensation to reduce effect from wind gradients and turbulences
		// compensation adjustment is function of te_comp_adjust: 0= no compensation  x= time during which wind gradient and turbulence are filtered
		float ealt = ((ptasmps*ptasmps) - (tasmps*tasmps)) /19.62; // kinetic energy difference between filtered tas and raw tas, this should correspond to wind gradient and turbulence altitude contribution. Dh = (ptasmps² - tasmps²) / 2 g
		_currentAlt += ealt; // add kinetic altitude correction to altitude from TE port
		// JLD
	}
	// ESP_LOGI(FNAME,"TE alt: %4.3f m", _currentAlt );

	uint64_t rts = esp_timer_get_time();
	float delta = (float)(rts - lastrts)/1000000.0;   // in seconds
	if( delta < 0.075 )  // ensure every 100 mS one calculation
		return _TEF;

	// ESP_LOGI(FNAME,"Vario delta=%2.3f sec", delta );

	lastrts = rts;
	// ESP_LOGI(FNAME, "TE-Alt %0.1f  NM:", _currentAlt );
	if( _init  ){
		vTaskDelay(100 / portTICK_PERIOD_MS);
		_currentAlt = _sensorTE->readAltitude(_qnh, success ) * 1.03; // we want have some beep when powerd on
		lastAltitude = _currentAlt;
		predictAlt = _currentAlt;
		Altitude = _currentAlt;
		averageAlt = _currentAlt;
		// ESP_LOGI(FNAME,"Initial Alt=%0.1f",Altitude);
		ESP_LOGI(FNAME, "Initial Alt=%0.1f",Altitude );

		// analogOut();  // set defaults
		_init = false;
	}
	averageAlt += (_currentAlt - averageAlt) * 0.1;
	double err = (abs(_currentAlt - predictAlt) * 1000) + 1;
	// ESP_LOGI(FNAME,"BMPVario new alt %0.1f err %0.1f", _currentAlt, err);
	double diff = (abs(_currentAlt - Altitude) * 1000) + 1;
	// if(diff > 1000000)  // more than 10 m alt diff in 0.1 second not plausible ( > 400 km/h vertical )
	// 	return _TEF;  // fancy errored value ignored, return last TE
	double kg = (diff / (err*_errorval + diff)) * _alpha;
	Altitude += (_currentAlt - Altitude) * kg;
	double TE = Altitude - lastAltitude;
	// ESP_LOGI(FNAME," TE %0.1f diff %0.1f", TE, diff);
	lastAltitude = Altitude;

	// ESP_LOGI(FNAME,"++++++ DELTA %0.4f", delta );

	TEarr[index++] = TE / delta;
	if (index >= _filter_len ) {
		index = 0;
	}
	double TEFR = 0;
	for ( int i = 0; i < _filter_len; i++) {
		TEFR += TEarr[i];
	}
	TEFR = TEFR / _filter_len;

	predictAlt = Altitude + (TEFR * delta);
	_TEF = _TEF + ((TEFR - _TEF)/ delta ) /(_damping/delta);

	_avgTE += (_TEF - _avgTE)* (1/(10*vario_av_delay.get()));   // _av_damping in seconds, we have 10 samples per second.
	if( holddown > 0 ) {
		holddown--;
	}
	else
	{
		AverageVario::newSample( _TEF );
	}
	// Bird catcher
	// if( (TE > 0.1) || (TE < -0.1) ){
	// 	ESP_LOGI(FNAME,"Vario alt: %f, Vario: %f, t-delta=%2.3f sec", _currentAlt, TE, delta );
	//}
	return _TEF;
}

void BMPVario::setTE( double te ) {
	_test = true;
	_TEF = te;
	// calcAnalogOut();
}  // for testing purposes


BMPVario bmpVario;
