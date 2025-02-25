#include "main.h"
#include "BNO08x.h"

#include "SH2_Inc/sh2_hal.h"
#include "SH2_Inc/sh2.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SPI_TIMEOUT 1000 // ms, timeout fed to HAL_SPI functions
#define INT_TIMEOUT 500 // ms, time before the sensor is considered unresponsive when the program expects it to assert INT

SPI_HandleTypeDef* _spi;

GPIO_TypeDef *_NCS_Port;
uint16_t _NCS_Pin;

GPIO_TypeDef *_NINT_Port;
uint16_t _NINT_Pin;

GPIO_TypeDef *_NRST_Port;
uint16_t _NRST_Pin;

struct sh2_Hal_s _hal;

sh2_ProductIds_t prod_ids = {0};

static sh2_SensorValue_t *_sensor_value = NULL;

bool _was_rst = false;

// ==================== Hardware abstraction ==================== //

static inline void _select(){
	HAL_GPIO_WritePin(_NCS_Port, _NCS_Pin, GPIO_PIN_RESET);
}

static inline void _deselect(){
	HAL_GPIO_WritePin(_NCS_Port, _NCS_Pin, GPIO_PIN_SET);
}

static inline bool _sensorReady(){
	return !HAL_GPIO_ReadPin(_NINT_Port, _NINT_Pin);
}

static bno_err_t _waitForSensRdy(uint16_t timeout){
	for(uint16_t i = 0; i < timeout; i++){
		if(_sensorReady()) return bno_ok; // sensor is still not ready
		HAL_Delay(1);
	}
	return bno_timeout;
	/*
	uint32_t t = HAL_GetTick();
	while(!_sensorReady() && HAL_GetTick() - t < timeout);

	if(!_sensorReady()) return bno_timeout; // sensor is still not ready

	return bno_ok;
	*/
}

void _hardwareReset(){
	HAL_GPIO_WritePin(_NRST_Port, _NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(_NRST_Port, _NRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(_NRST_Port, _NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}


uint32_t _getTimeUs(sh2_Hal_t *self){
	return HAL_GetTick() * 1000;
}

static int _write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len){
	if(_waitForSensRdy(INT_TIMEOUT) != bno_ok) return 0;

	_select();

	if(HAL_SPI_Transmit(_spi, pBuffer, len, SPI_TIMEOUT) != HAL_OK){
		_deselect();
		return 0;
	}

	_deselect();

	return len;
}

static int _read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us){
	if(_waitForSensRdy(INT_TIMEOUT) != bno_ok) return 0;

	uint16_t packet_size = 0;

	_select();

	if(HAL_SPI_Receive(_spi, pBuffer, 4, SPI_TIMEOUT) != HAL_OK){
		_deselect();
		return 0;
	}

	_deselect();

	// Determine amount to read
	packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;
	// Unset the "continue" bit
	packet_size &= ~0x8000;

	if (packet_size > len){
		return 0;
	}

	if(_waitForSensRdy(INT_TIMEOUT) != bno_ok){
		return 0;
	}

	_select();

	if(HAL_SPI_Receive(_spi, pBuffer, packet_size, SPI_TIMEOUT) != HAL_OK){
		_deselect();
		return 0;
	}

	_deselect();

	//*t_us = _getTimeUs(self);

	return packet_size;
}

static int _open(sh2_Hal_t *self){

	_waitForSensRdy(500);

	return 0;
}

static void _close(sh2_Hal_t *self){
	//HAL_GPIO_WritePin(_NRST_Port, _NRST_Pin, GPIO_PIN_RESET);
}

// ==================== Low Level ==================== //

// non-sensor events
static void _eventCallback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  if (pEvent->eventId == SH2_RESET) {
    _was_rst = true;
  }
}

// sensor events (data), with event given by the lib when sensor is serviced and _sensor_value the memory space to store the information
static void _sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  if (sh2_decodeSensorEvent(_sensor_value, event) != SH2_OK) {
    _sensor_value->timestamp = 0;
    return;
  }
}

// ==================== High Level ==================== //

bno_err_t bnoInit(bno08x_t* b){
	_deselect();

	_spi = b->spi;
	_NCS_Port = b->NCS_Port;
	_NCS_Pin = b->NCS_Pin;
	_NINT_Port = b->NINT_Port;
	_NINT_Pin = b->NINT_Pin;
	_NRST_Port = b->NRST_Port;
	_NRST_Pin = b->NRST_Pin;

	_hal.write = _write;
	_hal.read = _read;
	_hal.open = _open;
	_hal.close = _close;
	_hal.getTimeUs = _getTimeUs;

	bno_err_t err = 0;

	_hardwareReset();

	// Open SH2 interface (also registers non-sensor event handler.)
	if (sh2_open(&_hal, _eventCallback, NULL) != SH2_OK) {
		return false;
	}

	// Check connection partially by getting the product id's
	memset(&prod_ids, 0, sizeof(prod_ids));
	err = sh2_getProdIds(&prod_ids);
	if (err != SH2_OK) {
		return false;
	}

	// Register sensor listener
	sh2_setSensorCallback(_sensorHandler, NULL);

	return bno_ok;
}

bool bnoProcess(sh2_SensorValue_t *value) {
	_sensor_value = value;

	value->timestamp = 0;

	sh2_service();

	if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
		// no new events
		return false;
	}

	return true;
}

void bnoGetData(){

}

// ==================== State Getters ==================== //

bool bnoWasReset(){
	bool was_reset = _was_rst;
	_was_rst = false;
	return was_reset;
}

// ==================== Setters/Getters ==================== //

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool bnoEnableReportInterval(sh2_SensorId_t sensorId, uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}
bool bnoEnableReport(sh2_SensorId_t sensorId) {
	return bnoEnableReportInterval(sensorId, 10000);
}

