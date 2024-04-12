#include "BNO085.h"
#include "ADNS9800.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
#include "time.h"

static timestamp_t last_poll_time = 0;

float delta_adns_x = 0, delta_adns_y = 0;
float theta = 0, raw_theta = 0, theta_reference = 0;
float delta_x = 0, delta_y = 0, x = 0, y = 0;

bool first_read = true;

#define MVT_RELATIVE_ANGLE -0.0577//relative to the robot, radians

void computePosition(int poll_rate){
	if (isTimeDeltaElapsed(last_poll_time, poll_rate)){
		last_poll_time = getCurrentTime();
		if(bno_get_readings()){
			if (first_read) {theta_reference = bno_get_yaw();first_read = false;}
			else raw_theta = bno_get_yaw();

			theta = raw_theta - theta_reference;
			if(theta > M_PI) theta -= 2*M_PI;
			if(theta <= -M_PI) theta += 2*M_PI;

			if(adnsUpdate() != 0){

				//j'ai l'impression qu'il drop des packets de l'adns

				delta_adns_x = adnsX(); delta_adns_y = adnsY();

				delta_x = delta_adns_x*cos(theta+MVT_RELATIVE_ANGLE) - delta_adns_y*sin(theta+MVT_RELATIVE_ANGLE);
				delta_y = delta_adns_x*sin(theta+MVT_RELATIVE_ANGLE) + delta_adns_y*cos(theta+MVT_RELATIVE_ANGLE);

				//adaptation des valeurs à la table EVO
				y -= delta_x;
				x += delta_y;

				//printf("x %.2f y %.2f t %.2f rx %.2f ry %.2f\n",x,y,theta, adns_raw_x(), adns_raw_y());
			}
		}
	}
}

float getHeading(void){
	return theta;
}

float getX(void){
	return x;
}

float getY(void){
	return y;
}

void setX(float value){
	x = value;
}
void setY(float value){
	y = value;
}
void setT(float value){
	theta_reference = raw_theta - value;
	if(theta_reference > M_PI) theta_reference -= 2*M_PI;
	if(theta_reference <= -M_PI) theta_reference += 2*M_PI;

}
