#ifndef INC_PAA5163_H_
#define INC_PAA5163_H_

#include "main.h"

typedef enum {
	paa_err = 0, // unknown error
	paa_ok = 1, // all good

	paa_coms = 10, // could not communicate with the chip
	paa_init = 11, // something when wrong during init flow
} paa_err_t;

typedef struct {
	SPI_HandleTypeDef* spi;

	GPIO_TypeDef *NCS_Port;
	uint16_t NCS_Pin;
} paa5163_t;

paa_err_t paaInit(paa5163_t* p);
void paaReadMotion(paa5163_t* p);

#endif /* INC_PAA5163_H_ */
