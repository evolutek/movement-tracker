#ifndef INC_BNO085_H_
#define INC_BNO085_H_

#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdbool.h>

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define BNO_SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define BNO_SHTP_REPORT_COMMAND_REQUEST 0xF2
#define BNO_SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define BNO_SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define BNO_SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define BNO_SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define BNO_SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define BNO_SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define BNO_REPORTID_ACCELEROMETER 0x01
#define BNO_REPORTID_GYROSCOPE 0x02
#define BNO_REPORTID_MAGNETIC_FIELD 0x03
#define BNO_REPORTID_LINEAR_ACCELERATION 0x04
#define BNO_REPORTID_ROTATION_VECTOR 0x05
#define BNO_REPORTID_GRAVITY 0x06
#define BNO_REPORTID_UNBNO_REG_CALIBRATED_GYRO 0x07
#define BNO_REPORTID_GAME_ROTATION_VECTOR 0x08
#define BNO_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define BNO_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define BNO_REPORTID_TAP_DETECTOR 0x10
#define BNO_REPORTID_STEP_COUNTER 0x11
#define BNO_REPORTID_STABILITY_CLASSIFIER 0x13
#define BNO_REPORTID_RAW_ACCELEROMETER 0x14
#define BNO_REPORTID_RAW_GYROSCOPE 0x15
#define BNO_REPORTID_RAW_MAGNETOMETER 0x16
#define BNO_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define BNO_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define BNO_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define BNO_FRS_RECORDID_ACCELEROMETER 0xE302
#define BNO_FRS_RECORDID_GYROSCOPE_BNO_REG_CALIBRATED 0xE306
#define BNO_FRS_RECORDID_MAGNETIC_FIELD_BNO_REG_CALIBRATED 0xE309
#define BNO_FRS_RECORDID_ROTATION_VECTOR 0xE30B

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define BNO_EXECUTABLE_RESET_COMPLETE 0x1

//Command IDs from section 6.4, page 42
//These are used to BNO_REG_CALIBRATE, initialize, set orientation, BNO_REG_TARE etc the sensor
#define BNO_COMMANDID_ERRORS 1
#define BNO_COMMANDID_COUNTER 2
#define BNO_COMMANDID_BNO_REG_TARE 3
#define BNO_COMMANDID_INITIALIZE 4
#define BNO_COMMANDID_DCD 6
#define BNO_COMMANDID_ME_BNO_REG_CALIBRATE 7
#define BNO_COMMANDID_DCD_PERIOD_SAVE 9
#define BNO_COMMANDID_OSCILLATOR 10
#define BNO_COMMANDID_CLEAR_DCD 11

#define BNO_ID_CALIBRATE_ACCEL 0
#define BNO_ID_CALIBRATE_GYRO 1
#define BNO_ID_CALIBRATE_MAG 2
#define BNO_ID_CALIBRATE_PLANAR_ACCEL 3
#define BNO_ID_CALIBRATE_ACCEL_GYRO_MAG 4
#define BNO_ID_CALIBRATE_STOP 5

#define BNO_ID_TARE_NOW 0
#define BNO_ID_TARE_PERSIST 1
#define BNO_ID_TARE_SET_REORIENTATION 2

#define BNO_REG_TARE_AXIS_ALL 0x07
#define BNO_REG_TARE_AXIS_Z   0x04

#define BNO_ID_TARE_ROTATION_VECTOR 0
#define BNO_ID_TARE_GAME_ROTATION_VECTOR 1
#define BNO_ID_TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define BNO_ID_TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define BNO_ID_TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define BNO_ID_TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

//TODO : getting packets of more than the max packet size breaks the whole receiver function
#define BNO_MAX_PACKET_SIZE 256//128 //Packets can be up to 32k but we don't have that much RAM.
#define BNO_MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)
#define BNO_STANDARD_INT_TIMEOUT 127

bool setup_bno(void);
bool bno_data_available(void);
void bno_enable_rotation_vector(uint16_t timeBetweenReports);

#endif /* INC_BNO085_H_ */
