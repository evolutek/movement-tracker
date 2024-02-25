#ifndef INC_ADNS9800_H_
#define INC_ADNS9800_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#define ADNS_REG_Product_ID                           0x00
#define ADNS_REG_Revision_ID                          0x01
#define ADNS_REG_Motion                               0x02
#define ADNS_REG_delta_x_L                            0x03
#define ADNS_REG_delta_x_H                            0x04
#define ADNS_REG_delta_y_L                            0x05
#define ADNS_REG_delta_y_H                            0x06
#define ADNS_REG_SQUAL                                0x07
#define ADNS_REG_Pixel_Sum                            0x08
#define ADNS_REG_Maximum_Pixel                        0x09
#define ADNS_REG_Minimum_Pixel                        0x0a
#define ADNS_REG_Shutter_Lower                        0x0b
#define ADNS_REG_Shutter_Upper                        0x0c
#define ADNS_REG_Frame_Period_Lower                   0x0d
#define ADNS_REG_Frame_Period_Upper                   0x0e
#define ADNS_REG_Configuration_I                      0x0f
#define ADNS_REG_Configuration_II                     0x10
#define ADNS_REG_Frame_Capture                        0x12
#define ADNS_REG_SROM_Enable                          0x13
#define ADNS_REG_Run_Downshift                        0x14
#define ADNS_REG_Rest1_Rate                           0x15
#define ADNS_REG_Rest1_Downshift                      0x16
#define ADNS_REG_Rest2_Rate                           0x17
#define ADNS_REG_Rest2_Downshift                      0x18
#define ADNS_REG_Rest3_Rate                           0x19
#define ADNS_REG_Frame_Period_Max_Bound_Lower         0x1a
#define ADNS_REG_Frame_Period_Max_Bound_Upper         0x1b
#define ADNS_REG_Frame_Period_Min_Bound_Lower         0x1c
#define ADNS_REG_Frame_Period_Min_Bound_Upper         0x1d
#define ADNS_REG_Shutter_Max_Bound_Lower              0x1e
#define ADNS_REG_Shutter_Max_Bound_Upper              0x1f
#define ADNS_REG_LASER_CTRL0                          0x20
#define ADNS_REG_Observation                          0x24
#define ADNS_REG_Data_Out_Lower                       0x25
#define ADNS_REG_Data_Out_Upper                       0x26
#define ADNS_REG_SROM_ID                              0x2a
#define ADNS_REG_Lift_Detection_Thr                   0x2e
#define ADNS_REG_Configuration_V                      0x2f
#define ADNS_REG_Configuration_IV                     0x39
#define ADNS_REG_Power_Up_Reset                       0x3a
#define ADNS_REG_Shutdown                             0x3b
#define ADNS_REG_Inverse_Product_ID                   0x3f
#define ADNS_REG_Motion_Burst                         0x50
#define ADNS_REG_SROM_Load_Burst                      0x62
#define ADNS_REG_Pixel_Burst                          0x64


#define DEFAULT_COEF 67.2 // units (as seen per the sensors) per millimeter of actual travel of the module

void adnsInit(SPI_HandleTypeDef* spi_port);
void adnsSetCoef(float);
bool adnsUpdate(void);
void adnsReset(void);
void adnsEnableDebugReports(bool);

float adnsX(void);
float adnsY(void);

#endif /* INC_ADNS9800_H_ */
