#include "stm32g4xx_hal.h"
#include "main.h"
#include "BNO08x.h"
#include "BNO08x_shtp_registers.h"

#include "stdbool.h"
#include "stdio.h"

#define SPI_INTERFACE hspi1
#define SPI_TIMEOUT 1000

#define BNO_MAX_PACKET_SIZE 300

typedef struct bno_packet_s {
	shtp_header_t header;
	uint8_t data[BNO_MAX_PACKET_SIZE];
} bno_packet_t;

// ==================== Variables ==================== //

static bool interrupt_triggered = false;
static bool listen = false;
static uint16_t sequence_number[6] = {0};

bno_packet_t default_packet = {0};

// ==================== Hardware abstraction ==================== //

static inline void _enable(){
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
}

static inline void _disable(){
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
}

static inline void _reset(){
	HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_RESET);
	HAL_Delay(1); // tnrst is 10us
	HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_SET);
	HAL_Delay(100); // t1 + t2 is about 100ms
}

static inline bool _awaiting(){
	return !HAL_GPIO_ReadPin(INT_IMU_GPIO_Port, INT_IMU_Pin);
}


// ==================== Low Level ==================== //

static void _waitForAvail(){ // TODO : implement timeout
	while(!_awaiting());
}

static void _waitForInterrupt(){ // TODO : implement timeout
	while(!interrupt_triggered);
	interrupt_triggered = false;
}

static bool _retrieve(bno_packet_t* packet){
	if(!_awaiting()) return 0; // sensor has nothing to tell us

	_enable();

	uint8_t raw_header[4];

	HAL_SPI_Receive(&SPI_INTERFACE, raw_header, 4, SPI_TIMEOUT);

	packet->header = *((shtp_header_t*) raw_header);

	sequence_number[packet->header.channel] = packet->header.seq_numb;

	if (packet->header.length == 0){
		printf("fail\n");
		_disable();
		return 0;
	}

	printf("raw %2X %2X %d %d\n", raw_header[0], raw_header[1], raw_header[2], raw_header[3]);
	printf("header %d %d %d %d\n", packet->header.length, packet->header.channel, packet->header.seq_numb, packet->header.followup);

	if(packet->header.length >= BNO_MAX_PACKET_SIZE) return 0;

	HAL_SPI_Receive(&SPI_INTERFACE, packet->data, packet->header.length -4, SPI_TIMEOUT);

	_disable();

	return 1;
}

static void _send(bno_packet_t* packet){
	packet->header.followup = 0;
	sequence_number[packet->header.channel]++;

	_enable();

	HAL_SPI_Transmit(&SPI_INTERFACE, (uint8_t*)&(packet->header), 4, SPI_TIMEOUT);
	HAL_SPI_Transmit(&SPI_INTERFACE, (uint8_t*) (packet->data), packet->header.length -4, SPI_TIMEOUT);

	_disable_slave();
}

// ==================== High Level ==================== //


void bnoInterrupt(){
	//printf("int\n");

	if(!listen) return;
	interrupt_triggered = true;
	_retrieve(&default_packet);

	printf("data\n");
}

void bnoInit(){
	_disable();

	_reset();

	printf("rst\n");

	_waitForAvail(); // shtp advertissement
	_retrieveData(&default_packet);

	printf("adv\n");

	_waitForAvail(); // executable reset message
	_retrieveData(&default_packet);

	printf("exec\n");

	_waitForAvail(); // sh2 init message
	_retrieveData(&default_packet);

	printf("init\n");

	//listen = true;
}


