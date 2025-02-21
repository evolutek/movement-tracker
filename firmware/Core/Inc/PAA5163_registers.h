#ifndef INC_PAA5163_REGISTERS_H_
#define INC_PAA5163_REGISTERS_H_

typedef enum {
	product_id = 0x00,
	revision_id = 0x01,

	motion = 0x02,
	delta_x_l = 0x03,
	delta_x_h = 0x04,
	delta_y_l = 0x05,
	delta_y_h = 0x06,

	squal = 0x07,

	rawdata_sum = 0x08,
	maximum_rawdata = 0x09,
	minimum_rawdata = 0x0A,

	shutter_lower = 0x0B,
	shutter_upper = 0x0C,

	observation = 0x15,
	motion_burst = 0x16,
	squal2 = 0x17,

	power_up_reset = 0x3A,
	shutdown = 0x3B,
	set_resolution = 0x47,

	resolution_x_lower = 0x48,
	resolution_x_upper = 0x49,
	resolution_y_lower = 0x4A,
	resolution_y_upper = 0x4B,

	rawdata_grab = 0x58,
	rawdata_grab_status = 0x59,

	orientation = 0x5B,
	motion_control = 0x5C,

	inverse_product_id = 0x5F,
} paa5163_registers_t;


#endif /* INC_PAA5163_REGISTERS_H_ */
