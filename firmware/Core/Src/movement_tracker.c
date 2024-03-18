#include "BNO085.h"
#include "ADNS9800.h"
#include "movement_tracker.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

#define POLL_RATE 39 //ms, approx (non interrupt), only applicable to the IMU
#define MVT_RELATIVE_ANGLE -0.0577//relative to the robot, radians


static timestamp_t last_poll_time = 0;

float delta_adns_x = 0, delta_adns_y = 0;
float theta = 0, raw_theta = 0, delta_theta = 0;
float delta_x = 0, delta_y = 0, x = 0, y = 0;

bool first_read = true;

uint8_t data_buffer;

/*	bytes order :
	data[0] : number of bytes to be received/sent
	data[1] : MSB is operation type (eg. read/write)
			  other bytes are operation identifier
	data[2...] : actual data
*/



//TODO : remove the interrupt capability for IMU_INT

void setup(void){
	//adnsEnableDebugReports();
	adnsInit();
	printf("ADNS should now be initialized\n");

	if(bno_setup()) printf("IMU initialized successfully\n");
	else printf("=== Could NOT initialize the BNO085 ! ===\n");
	bno_enable_rotation_vector(40);

	last_poll_time = getCurrentTime();
}

void loop(void){
	if (isTimeDeltaElapsed(last_poll_time, POLL_RATE)){
		last_poll_time = getCurrentTime();
		if(bno_get_readings()){
			if (first_read) {delta_theta = bno_get_yaw();first_read = false;}
			else raw_theta = bno_get_yaw();

			theta = raw_theta - delta_theta;

			if(adnsUpdate() != 0){

				//j'ai l'impression qu'il drop des packets de l'adns

				delta_adns_x = adnsX(); delta_adns_y = adnsY();

				delta_x = delta_adns_x*cos(theta+MVT_RELATIVE_ANGLE) - delta_adns_y*sin(theta+MVT_RELATIVE_ANGLE);
				delta_y = delta_adns_x*sin(theta+MVT_RELATIVE_ANGLE) + delta_adns_y*cos(theta+MVT_RELATIVE_ANGLE);
				x += delta_x;
				y += delta_y;
				printf("x %.2f y %.2f t %.2f \n",x,y,theta);
			}
		}
	}
	//if(x < 3 || y < 3) if (delta_x != 0 || delta_y != 0) printf("x %.2f y %.2f t %.2f dx %.2f dy %.2f rt %.2f dt %.2f \n",x,y,theta,delta_x,delta_y,raw_theta, delta_theta);
	//if((x < 3 && x > -3) || (y < 3 && y > -3))

	/*
	int n;
	if(scanf("%d", &n))
		printf("R %d\n", n);
	*/

	//HAL_I2C_Slave_Transmit_IT(hi2c2, 0x52, data_buffer, 10);

}
