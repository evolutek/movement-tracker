#ifndef INC_BNO085_H_
#define INC_BNO085_H_

#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdbool.h>

#define BNO_MAX_PACKET_SIZE 300//128 //Packets can be up to 32k but we don't have that much RAM.
#define BNO_MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)
#define BNO_STANDARD_INT_TIMEOUT 250 // standard timeout for the _wait_for_int_blocking function, ms

bool bno_setup(void);
void bno_enable_rotation_vector(uint16_t millisBetweenReports);
uint16_t bno_get_readings(void);
float bno_get_yaw(void);

#endif /* INC_BNO085_H_ */
