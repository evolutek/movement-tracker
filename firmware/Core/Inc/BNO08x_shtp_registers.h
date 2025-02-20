#ifndef INC_BNO08X_SHTP_REGISTERS_H_
#define INC_BNO08X_SHTP_REGISTERS_H_

#include "stdbool.h"

typedef struct __attribute__ ((__packed__)) shtp_header_s {
	uint16_t length : 15;
	bool followup : 1; // indicate that the message is a continuation of the previous one
	uint8_t channel;
	uint8_t seq_numb;
} shtp_header_t;

typedef enum shtp_channel_e {
	command = 0,
	executable = 1,
	hub_control = 2,
	input_sensor = 3,
	wake_input = 4,
	gyro = 5,
} shtp_channel_t;



typedef enum exec_command_e {
	reset = 1,
	on = 2,
	sleep = 3,
} exec_command_t;

typedef enum exec_response_e {
	reset_complete = 1,
} exec_response_t;



typedef enum bno_command_e { // "response" are from bno to host, others are the other way around
	get_feature_request = 0xFE,
	get_feature_command = 0xFD,
	get_feature_response = 0xFC,

	product_id_request = 0xF9,
	product_id_response = 0xF8,

	frs_write_request = 0xF7,
	frs_write_data = 0xF6,
	frs_write_response = 0xF5,

	frs_read_request = 0xF4,
	frs_read_response = 0xF3,

	command_request = 0xF2,
	command_response = 0xF1,
} bno_command_t;



typedef enum bno_frs_record_s {
	agm_static_calibration = 0x7979,
	agm_nominal_calibration = 0x4D4D,
	sra_static_calibration = 0x8A8A,
	sra_nominal_calibration = 0x4E4E,
	dynamic_calibration = 0x1F1F,
	motion_engine_power_management = 0xD3E2,
	system_orientation = 0x2D3E,
	primary_accelerometer_orientation = 0x2D41,
	gyroscope_orientation = 0x2D46,
	magnetometer_orientation = 0x2D4C,
	ar_vr_stabilisation_rotation_vector = 0x3E2D,
	/* TODO
0x3E2E AR/VR stabilization – game rotation vector
0xC274 Significant Motion detector configuration
0x7D7D Shake detector configuration
0xD7D7 Maximum fusion period
0x4B4B Serial number
0x39AF Environmental sensor - Pressure calibration
0x4D20 Environmental sensor - Temperature calibration
0x1AC9 Environmental sensor - Humidity calibration
0x39B1 Environmental sensor - Ambient light calibration
0x4DA2 Environmental sensor - Proximity calibration
0xD401 ALS Calibration
0xD402 Proximity Sensor Calibration
0xED85 Stability detector configuration
0x74B4 User record
0xD403 MotionEngine Time Source Selection
0xA1A2 Gyro-Integrated Rotation Vector configuration
	 */
} bno_frs_record__t;

#endif /* INC_BNO08X_SHTP_REGISTERS_H_ */
