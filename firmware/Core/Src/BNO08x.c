#include "main.h"
#include "BNO08x.h"
#include "BNO08x_shtp_registers.h"

#include "stdbool.h"
#include "stdio.h"

#define SPI_TIMEOUT 1000 // ms, timeout fed to HAL_SPI functions
#define INT_TIMEOUT 1000 // ms, max time when waiting for the sensor to assert INT

// ==================== Hardware abstraction ==================== //

static inline void _select(bno08x_t* b){
	HAL_GPIO_WritePin(b->NCS_Port, b->NCS_Pin, GPIO_PIN_RESET);
}

static inline void _deselect(bno08x_t* b){
	HAL_GPIO_WritePin(b->NCS_Port, b->NCS_Pin, GPIO_PIN_SET);
}

static inline void _reset(bno08x_t* b){
	HAL_GPIO_WritePin(b->NRST_Port, b->NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(b->NRST_Port, b->NRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1); // tnrst is 10us
	HAL_GPIO_WritePin(b->NRST_Port, b->NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(100); // t1 + t2 is about 100ms
}

static inline bool _sens_rdy(bno08x_t* b){
	return !HAL_GPIO_ReadPin(b->NINT_Port, b->NINT_Pin);
}


// ==================== Low Level ==================== //

static bool _waitForSensRdy(bno08x_t* b){ // returns 1 if timed out
	uint32_t t = HAL_GetTick();
	while(!_sens_rdy(b) || HAL_GetTick() - t > INT_TIMEOUT);

	if(!_sens_rdy(b)) return 1; // sensor is still not ready

	return 0;
}

static bool _retrieve(bno08x_t* b, bno_packet_t* packet){ // returns 1 if data was read successfully
	if(!_sens_rdy(b)) return 0; // sensor has nothing to tell us

	_select(b);

	uint8_t raw_header[4];

	HAL_SPI_Receive(b->spi, raw_header, 4, SPI_TIMEOUT);

	packet->header = *((shtp_header_t*) raw_header);

	b->seq_nb[packet->header.channel] = packet->header.seq_numb;

	if (packet->header.length == 0){
		_deselect(b);
		return 0;
	}

	//printf("header %d %d %d %d\n", packet->header.length, packet->header.channel, packet->header.seq_numb, packet->header.followup);

	if(packet->header.length >= BNO_MAX_PACKET_SIZE) return 0;

	HAL_SPI_Receive(b->spi, packet->data, packet->header.length -4, SPI_TIMEOUT);

	_deselect(b);

	packet->avail = 1;

	return 1;
}

static void _send(bno08x_t* b, bno_packet_t* packet){
	packet->header.followup = 0;
	b->seq_nb[packet->header.channel]++;

	_select(b);

	HAL_SPI_Transmit(b->spi, (uint8_t*)&(packet->header), 4, SPI_TIMEOUT);
	HAL_SPI_Transmit(b->spi, (uint8_t*) (packet->data), packet->header.length -4, SPI_TIMEOUT);

	_deselect(b);
}

// ==================== High Level ==================== //

bool bnoProcess(bno08x_t* b){ // returns 1 if data has been read
	if(!b->listen) return 0;
	if(!_sens_rdy(b)) return 0;

	return _retrieve(b, b->incoming);
}


bno_err_t bnoInit(bno08x_t* b){
	_deselect(b);

	_reset(b);

	// TODO : check for coms

	if(_waitForSensRdy(b)) return bno_shtp_advert; // shtp advertissement
	_retrieve(b, b->incoming);

	if(_waitForSensRdy(b)) return bno_exec_rst; // executable reset message
	_retrieve(b, b->incoming);


	if(_waitForSensRdy(b)) return bno_sh2_init; // sh2 init message
	_retrieve(b, b->incoming);

	b->listen = true;

	return bno_ok;
}


