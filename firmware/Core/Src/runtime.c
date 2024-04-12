#include "BNO085.h"
#include "ADNS9800.h"
#include "movement_tracker.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
#include <runtime.h>
#include <time.h>

uint8_t I2C_REGISTERS[50] = {1,2,3,4,5,6,7,8,9,10};
// Note : adress 0 is used to indicate commands, no data should be put there

/*
 * 0 réservé
 * 1 heading (1/4)
 * 2 heading (2/4)
 * 3 heading (3/4)
 * 4 heading (4/4)
 */

extern I2C_HandleTypeDef hi2c1;

// CAREFUL ! messages take an awefully long time to send, and can interfere with I2C comms, expect problems when enabling it (don't have time to debug this for now)
bool debug_i2c = false;

#define RxSIZE  51
uint8_t RxData[RxSIZE];
uint8_t rxcount=0;
uint8_t txcount=0;

uint8_t startPosition = 0;
uint8_t bytesRrecvd = 0;
uint8_t bytesTransd = 0;

#define POLL_RATE 39 //ms, approx (non interrupt)

//TODO : remove the interrupt capability for IMU_INT

void setup(void){
	//adnsEnableDebugReports();
	adnsInit();
	printf("ADNS should now be initialized\n");

	if(bno_setup()) printf("IMU initialized successfully\n");
	else printf("=== Could NOT initialize the BNO085 ! ===\n");
	bno_enable_rotation_vector(40);

	HAL_I2C_EnableListen_IT(&hi2c2);
}

void loop(void){

	computePosition(POLL_RATE);

	float theta = getHeading();
	uint8_t *theta_split = (uint8_t *)&theta;
	I2C_REGISTERS[1]=(theta_split[0]);
	I2C_REGISTERS[2]=(theta_split[1]);
	I2C_REGISTERS[3]=(theta_split[2]);
	I2C_REGISTERS[4]=(theta_split[3]);

	float x = getX();

	uint8_t *x_split = (uint8_t *)&x;
	I2C_REGISTERS[5]=(x_split[0]);
	I2C_REGISTERS[6]=(x_split[1]);
	I2C_REGISTERS[7]=(x_split[2]);
	I2C_REGISTERS[8]=(x_split[3]);

	float y = getY();

	uint8_t *y_split = (uint8_t *)&y;
	I2C_REGISTERS[9]=(y_split[0]);
	I2C_REGISTERS[10]=(y_split[1]);
	I2C_REGISTERS[11]=(y_split[2]);
	I2C_REGISTERS[12]=(y_split[3]);
}

void process_data(){
	if(RxData[0] == 0) { // commands
		switch(RxData[1]){
			case 0x01:
				printf("resetXY\n");
				setX(0);
				setY(0);
				break;

			case 0x02:
				printf("setTXY\n");
				float *theta_split = (float *)&RxData[2];
				float theta = *theta_split;
				setT(theta);
				float *x_split = (float *)&RxData[6];
				float x = *x_split;
				setX(x);
				float *y_split = (float *)&RxData[10];
				float y = *y_split;
				setY(y);
				break;
		}
	} else { // mem write
		//for(uint8_t i = 0; i < rxcount; i++)
		//	I2C_REGISTERS[i + RxData[0]] = RxData[i];
	}
}


void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
	HAL_I2C_EnableListen_IT(hi2c);
	if(debug_i2c)printf("list cplt\n");
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
	if(debug_i2c) printf("add match\n");
	if (TransferDirection == I2C_DIRECTION_TRANSMIT){  // if the master wants to transmit the data
		//RxData[0] = 0;  // reset the RxData[0] to clear any residue address from previous call
		rxcount = 0;
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData, 1, I2C_FIRST_FRAME);
		rxcount++;
	} else {
		txcount = 0;
		startPosition = RxData[0]; // transmission can only happen if the slave has received an order to send specific data
		//RxData[0] = 0;  // Reset the start register as we have already copied it
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &I2C_REGISTERS[startPosition+txcount], 1, I2C_FIRST_FRAME);
		if(debug_i2c)printf("fsent %d : %d \n",startPosition+txcount, I2C_REGISTERS[startPosition+txcount]);
		txcount++;
	}
	if(debug_i2c) printf("start\n");
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//will try to send the last byte, which will fail because the master already received the right number of bytes
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &I2C_REGISTERS[startPosition+txcount], 1, I2C_NEXT_FRAME);
	if(debug_i2c) printf("sent %d : %d \n",startPosition+txcount, I2C_REGISTERS[startPosition+txcount]);
	txcount++; // WARNING : txcount will always be greater than the actual number of bytes received by the master
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if (rxcount < RxSIZE){
		if(debug_i2c)printf("recv %d \n",rxcount);
		if (rxcount == RxSIZE-1){
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, &RxData[rxcount], 1, I2C_LAST_FRAME);
		} else {
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, &RxData[rxcount], 1, I2C_NEXT_FRAME);
		}
		rxcount++;
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
	uint32_t error_code = HAL_I2C_GetError(hi2c);
	if(debug_i2c)printf("I2C error %ld \n", error_code);

	if(error_code == 4){
		if(txcount == 0){ // error triggered after only receiving
			printf("trtmt\n");
			process_data();
		} else {
			txcount = 0;
		}
	}

	if(debug_i2c)printf("rx %d tx %d \n", rxcount, txcount);
	HAL_I2C_EnableListen_IT(hi2c);
}

