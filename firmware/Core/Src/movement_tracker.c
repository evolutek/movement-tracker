#include "BNO085.h"
#include "ADNS9800.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

double delta_adns_x = 0, delta_adns_y = 0, theta = 0;
double delta_x = 0, delta_y = 0, x = 0, y = 0;

//TODO : remove the interrupt capability for IMU_INT

void setup(void){
	//adnsEnableDebugReports();
	adnsInit();
	printf("ADNS should now be initialized\n");

	if(bno_setup()) printf("IMU initialized successfully\n");
	else printf("=== Could NOT initialize the BNO085 ! ===\n");
	bno_enable_rotation_vector(30);

}

void loop(void){
	HAL_Delay(15);
	if(bno_get_readings()) theta = bno_get_yaw();
	if(adnsUpdate() != 0){delta_adns_x = adnsX(); delta_adns_y = adnsY();}

	delta_x = delta_adns_x*cos(theta) - delta_adns_y*sin(theta);
	delta_y = delta_adns_x*sin(theta) + delta_adns_y*cos(theta);

	x += delta_x;
	y += delta_y;

	if (delta_x != 0 || delta_y != 0) printf("dx %.2f dy %.2f x %.2f y %.2f t %.2f \n",delta_x,delta_y,x,y,theta);
}
