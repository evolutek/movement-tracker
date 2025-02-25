#include "runtime.h"
#include "main.h"

#include "PAA5163.h"
#include "BNO08x.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

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
sh2_SensorValue_t sensorValue;
#define BNO_REPORT SH2_GAME_ROTATION_VECTOR

// WARNING : Both libraries could attempt to access the spi bus at the same time if read operations are done inside the interrupts !

void setBnoReports(){
	printf("Setting reports\n");
	if (! bnoEnableReport(BNO_REPORT)) {
		printf("Could not enable game vector\n");
	}
}

void setup(void){
	printf("PAA Init...");
	paa_err_t paa_init_exit = paaInit(&paa);
	printf(" %d \nBNO Init...", paa_init_exit);
	bno_err_t bno_init_exit = bnoInit(&bno);
	printf(" %d\n",bno_init_exit);


	if(paa_init_exit != paa_ok || bno_init_exit != bno_ok) { // for now, if one of the sensors could not be initialized properly, reboot to try again
		printf("FATAL ERROR : a sensor could not be initialized, rebooting...\n");
		HAL_Delay(500);
		NVIC_SystemReset();
	}

	sh2_ProductIds_t* ids = bnoGetProdIds();
	printf("BNO080 Sensors :\n");
	for (int n = 0; n < ids->numEntries; n++) {
		printf("\tPart %ld\n",ids->entry[n].swPartNumber);
		printf("\tVersion %d.%d.%d\n",ids->entry[n].swVersionMajor,ids->entry[n].swVersionMinor,ids->entry[n].swVersionPatch);
		printf("\tBuild %ld\n",ids->entry[n].swBuildNumber);
	}

	setBnoReports();
}

void loop(void){
	HAL_Delay(5);

	//paaReadMotion(&paa); // paa read is quite fast compared to the bno processing, which is why it is done before it

	if (bnoWasReset()) {
		printf("Sensor RST !\n");
		setBnoReports();

	}


	  if (!bnoProcess(&sensorValue)) {
	    return;
	  }
	  printf("data\n");
	  switch (sensorValue.sensorId) {
	    case SH2_GAME_ROTATION_VECTOR:
	    	/*
	      Serial.print("Game Rotation Vector - r: ");
	      Serial.print(sensorValue.un.gameRotationVector.real);
	      Serial.print(" i: ");
	      Serial.print(sensorValue.un.gameRotationVector.i);
	      Serial.print(" j: ");
	      Serial.print(sensorValue.un.gameRotationVector.j);
	      Serial.print(" k: ");
	      Serial.println(sensorValue.un.gameRotationVector.k);
	      */
	      break;
	  }

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
