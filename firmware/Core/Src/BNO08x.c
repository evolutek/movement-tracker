#include "main.h"
#include "BNO08x.h"
#include "BNO08x_shtp_registers.h"

#include <stdbool.h>
#include <stdio.h>

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
	// do not change those timings, i'm not sure why, but reducing them causes issues on reboots
	HAL_GPIO_WritePin(b->NRST_Port, b->NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(b->NRST_Port, b->NRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(b->NRST_Port, b->NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
}

static inline bool _sensRdy(bno08x_t* b){
	return !HAL_GPIO_ReadPin(b->NINT_Port, b->NINT_Pin);
}


// ==================== Low Level ==================== //

static bool _waitForSensRdyTimeout(bno08x_t* b, uint16_t timeout){ // returns 1 if timed out
	uint32_t t = HAL_GetTick();
	while(!_sensRdy(b) && HAL_GetTick() - t < timeout);

	if(!_sensRdy(b)) return 1; // sensor is still not ready

	return 0;
}
static inline bool _waitForSensRdy(bno08x_t* b){ return _waitForSensRdyTimeout(b, INT_TIMEOUT);}


static bool _retrieve(bno08x_t* b, bno_packet_t* packet){ // returns 1 if data was read successfully
	if(!_sensRdy(b)) return 0; // sensor has nothing to tell us

	_select(b);

	uint8_t raw_header[4];

	HAL_SPI_Receive(b->spi, raw_header, 4, SPI_TIMEOUT);

	packet->header = *((shtp_header_t*) raw_header);

	b->incom_seq_nb[packet->header.channel] = packet->header.seq_numb;

	if (packet->header.length == 0){ // TODO : not even sure if that's possible, as the header alone is already 4 bytes
		//printf("empty\n");
		_deselect(b);
		return 0;
	}

	if(packet->header.length -4 >= BNO_MAX_PACKET_SIZE) packet->header.length = BNO_MAX_PACKET_SIZE;

	HAL_SPI_Receive(b->spi, packet->data, packet->header.length -4, SPI_TIMEOUT);

	printf("< lgth %d, chan %d, seq_nb %d, contin %d\n", packet->header.length, packet->header.channel, packet->header.seq_numb, packet->header.followup);
	//printf("data : ");
	//for(uint16_t i = 0; i < packet->header.length -4; i++){
	//	printf("%02x ", packet->data[i]);
	//}
	//printf("\n");

	_deselect(b);

	packet->avail = 1;

	return 1;
}

static void _send(bno08x_t* b, shtp_channel_t channel, uint8_t data[], uint16_t data_length){ // Note : data_length does not include header length
	b->outgo_seq_nb[channel]++;

	shtp_header_t header = {
		.length = data_length + 4,
		.channel = channel,
		.followup = 0,
		.seq_numb = b->outgo_seq_nb[channel],
	};

	//printf("outgoing : ");
	//uint8_t* raw_header = (uint8_t*) &header;
	//for(uint8_t i = 0; i < 4; i++){
	//	printf("%02x ", raw_header[i]);
	//}
	//printf("\n");

	printf("> lgth %d, contin %d, chan %d, nb %d\n", header.length, header.followup, header.channel, header.seq_numb);

	_select(b);

	HAL_SPI_Transmit(b->spi, (uint8_t*)&(header), 4, SPI_TIMEOUT);
	HAL_SPI_Transmit(b->spi, data, data_length, SPI_TIMEOUT);

	_deselect(b);
}

// ==================== High Level ==================== //

bool bnoProcess(bno08x_t* b){ // returns 1 if data has been read
	if(!b->initialized) return 0;

	return _retrieve(b,&(b->incoming));
}


bno_err_t bnoInit(bno08x_t* b){
	_deselect(b);

	_reset(b);

	if(_waitForSensRdy(b)) return bno_shtp_advert; // shtp advertissement
	_retrieve(b,&(b->incoming));

	if(_waitForSensRdy(b)) return bno_exec_rst; // executable reset message
	_retrieve(b,&(b->incoming));

	if(_waitForSensRdy(b)) return bno_sh2_init; // sh2 init message
	_retrieve(b,&(b->incoming));

	if(_waitForSensRdy(b)) return bno_unknown_report; // unknown packet sent at startup, channel 0 length 55
	_retrieve(b,&(b->incoming));

	// now that the boot messages are cleared, we can test the communication to the device
	uint8_t data[] = {
		product_id_request,
		0
	};
	_send(b, hub_control, data, sizeof(data));

	if(_waitForSensRdy(b)) return bno_coms;
	_retrieve(b,&(b->incoming));

	printf("< data (lgth %d): ", b->incoming.header.length);
	for(uint16_t i = 0; i < b->incoming.header.length -4; i++){
		printf("%02x ", b->incoming.data[i]);
	}
	printf("\n");

	//if(b->incoming.data[0] != product_id_response) return bno_sequence;

	b->initialized = 1;
	return bno_ok;
}
