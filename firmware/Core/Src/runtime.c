#include "runtime.h"
#include "main.h"

#include "PAA5163.h"
#include "BNO08x.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

bool interrupts_enabled = 0;
bool data_ready = 0;

paa5163_t paa = {
	.spi = &hspi1,

	.NCS_Port = CS_PAA_GPIO_Port,
	.NCS_Pin = CS_PAA_Pin,
	.NRST_Port = RST_PAA_GPIO_Port,
	.NRST_Pin = RST_PAA_Pin,

	.invert_x = 1,
};

bno08x_t bno = {
	.spi = &hspi1,

	.NCS_Port = CS_IMU_GPIO_Port,
	.NCS_Pin = CS_IMU_Pin,
	.NINT_Port = INT_IMU_GPIO_Port,
	.NINT_Pin = INT_IMU_Pin,
	.NRST_Port = RST_IMU_GPIO_Port,
	.NRST_Pin = RST_IMU_Pin,
};

// Note : As the BNO triggers an interrupt evey couple of tens of ms, it is the only one actually triggering interrupts (the paa is read using polling methods)

// WARNING : Both libraries could attempt to access the spi bus at the same time if read operations are done inside the interrupts !

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(!interrupts_enabled) return;

	if(GPIO_Pin == bno.NINT_Pin){ // BNO data ready

		printf("Int : paa %d, bno %d\n", HAL_GPIO_ReadPin(INT_PAA_GPIO_Port, INT_PAA_Pin),HAL_GPIO_ReadPin(INT_IMU_GPIO_Port, INT_IMU_Pin));


		data_ready = 1;
	}
}


void setup(void){
	paa_err_t paa_init_exit = paaInit(&paa);
	bno_err_t bno_init_exit = bnoInit(&bno);

	printf("PAA5160 Init exit code : %d\n", paa_init_exit);
	printf("BNO08x Init exit code : %d\n", bno_init_exit);

	if(paa_init_exit != paa_ok || bno_init_exit != bno_ok) { // for now, if one of the sensors could not be initialized properly, reboot to try again
		printf("WARNING : a sensor could not be initialiezd, rebooting ...\n");
		HAL_Delay(500);
		NVIC_SystemReset();
	}

	/*
	bno_enable_rotation_vector(POLL_RATE);
	*/

	interrupts_enabled = 1;
}

void loop(void){
	paaReadMotion(&paa); // paa read is quite fast compared to the bno processing, which is why it is done before it
	bnoProcess(&bno);

	if(data_ready){

	}


	/*
	if (isTimeDeltaElapsed(last_poll_time, POLL_RATE+1)){
		last_poll_time = getCurrentTime();
		if(bno_get_readings()){
			if (first_read) {theta_reference = bno_get_yaw();first_read = false;}
			else raw_theta = bno_get_yaw();

			theta = raw_theta - theta_reference;
			while(theta > M_PI) theta -= 2*M_PI;
			while(theta <= -M_PI) theta += 2*M_PI;

			//printf("reference : %.5f raw theta %.3f accuracy %d \n",theta_reference,raw_theta,bno_get_accuracy());
		}
	}
	*/
}
