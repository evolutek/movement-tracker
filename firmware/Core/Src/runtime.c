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

#define constrain(x, floor, ceiling) ((x < floor ? floor : x) > ceiling ? ceiling : x)

// WARNING : Both libraries could attempt to access the spi bus at the same time if read operations are done inside the interrupts !

void setBnoReports(){
	printf("Setting reports... ");
	if (!bnoEnableReportInterval(SH2_ROTATION_VECTOR, 10000)) {
		printf("ERROR\n");
		return;
	}
	printf("OK\n");
}

void setup(void){
	printf("BNO Init... ");
	bno_err_t bno_init_exit = bnoInit(&bno);
	printf("%d (%s) \nPAA Init... ", bno_init_exit, (bno_init_exit == bno_ok ? "OK" : "ERROR"));
	paa_err_t paa_init_exit = 0;//paaInit(&paa);
	printf("%d (%s)\n",paa_init_exit, (paa_init_exit == paa_ok ? "OK" : "ERROR"));

	if(paa_init_exit != paa_ok || bno_init_exit != bno_ok) { // for now, if one of the sensors could not be initialized properly, reboot to try again
		printf("FATAL : a sensor could not be initialized, rebooting...\n");
		HAL_Delay(500);
		NVIC_SystemReset();
	}

	sh2_ProductIds_t* ids = bnoGetProdIds();
	printf("BNO080 Sensors :\n");
	for (int n = 0; n < ids->numEntries; n++) {
		printf("\tPart %ld, ",ids->entry[n].swPartNumber);
		printf("Version %d.%d.%d, ",ids->entry[n].swVersionMajor,ids->entry[n].swVersionMinor,ids->entry[n].swVersionPatch);
		printf("Build %ld\n",ids->entry[n].swBuildNumber);
	}

	setBnoReports();
}

void loop(void){
	#warning demander un report en produit tout le temps un juste après ...
	//setBnoReports();

	//HAL_Delay(6);

	//paaReadMotion(&paa); // paa read is quite fast compared to the bno processing, which is why it is done before it

	uint8_t rst = bnoWasReset();
	if (rst) {
		printf("Reset : %s (%d)\n", bno_reset_reason[constrain(rst, 0, 5)], rst);
		setBnoReports();
	}


	  if (!bnoProcess(&sensorValue)) {
	    return;
	  }
	  switch (sensorValue.sensorId) {
	  case SH2_GAME_ROTATION_VECTOR:
		  printf("GRV : r %.2f, i %.2f, j %.2f, k %.2f\n", sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
	      break;
	  case SH2_ROTATION_VECTOR :
	  	  printf("RV : r %.2f, i %.2f, j %.2f, k %.2f\n", sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k);
	  	  break;
	  default :
		  printf("rprt %d\n", sensorValue.sensorId);
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
