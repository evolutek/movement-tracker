#include <runtime.h>
#include "main.h"

#include <stdio.h>
#include <math.h>

#include "PAA5163.h"

#include <time.h>
//#include "BNO08x.h"

timestamp_t last_poll_time = 0;
bool first_read = false;
float theta_reference = 0, raw_theta = 0, theta = 0;

#define POLL_RATE 50 //ms, approx (non interrupt)

//TODO : remove the interrupt capability for IMU_INT

paa5163_t paa = {
	.spi = &hspi1,

	.NCS_Port = CS_PAA_GPIO_Port,
	.NCS_Pin = CS_PAA_Pin,
};


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_15){ // BNO
		//bnoInterrupt();
	} else { // PAA

	}
}


void setup(void){
	printf("paa init %d\n", paaInit(&paa));


	//bnoInit();
/*
	if(bno_setup()) printf("IMU initialized successfully\n");
	else printf("=== Could NOT initialize the BNO085 ! ===\n");
	bno_enable_rotation_vector(POLL_RATE);
	*/
}

void loop(void){
	paaReadMotion(&paa);
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
