#ifndef INC_ADNS9800_H_
#define INC_ADNS9800_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#define DEFAULT_COEF 67.2 // units (as seen per the sensors) per millimeter of actual travel of the module

void adnsInit();
void adnsSetCoef(float);
bool adnsUpdate(void);
void adnsSetDebugReports(bool);
void adnsEnableDebugReports(void);

float adnsX(void);
float adnsY(void);

float adns_raw_x(void);
float adns_raw_y(void);

#endif /* INC_ADNS9800_H_ */
