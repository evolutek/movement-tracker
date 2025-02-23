#ifndef INC_BNO08X_H_
#define INC_BNO08X_H_

#include <stdbool.h>
#include "BNO08x_shtp_registers.h"

#define BNO_MAX_PACKET_SIZE 300 // biggest message we could encounter is 276 bytes long

typedef struct {
	bool avail; // packet not processed yet
	shtp_header_t header;
	uint8_t data[BNO_MAX_PACKET_SIZE];
} bno_packet_t;

typedef enum {
	bno_ok = 0,

	bno_err = 1, // unknown error

	// Init errors :
	bno_coms = 10, // could not communicate with the chip
	bno_shtp_advert = 11, // could not retrive shtp advert message
	bno_exec_rst = 12, // could not retrive sensor's reset message
	bno_sh2_init = 13, // could not retrive sh2's init message
	bno_sequence = 14, // sensor did not deliver expected data when asked
} bno_err_t;

typedef struct {
	SPI_HandleTypeDef* spi;

	GPIO_TypeDef *NCS_Port;
	uint16_t NCS_Pin;

	GPIO_TypeDef *NINT_Port;
	uint16_t NINT_Pin;

	GPIO_TypeDef *NRST_Port;
	uint16_t NRST_Pin;

	// ======== READ ONLY ======== //

	uint8_t incom_seq_nb[6]; // sequence numbers for incoming packets (one for each channel)
	uint8_t outgo_seq_nb[6]; // sequence numbers for outgoing packets (one for each channel)

	bno_packet_t incoming; // default memory space for incoming packets

	bool initialized;
} bno08x_t ;

bno_err_t bnoInit();
bool bnoProcess(bno08x_t* b); // returns 1 if data was read


#endif /* INC_BNO08X_H_ */
