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

/*
 * 0 réservé
 * 1 heading (1/4)
 * 2 heading (2/4)
 * 3 heading (3/4)
 * 4 heading (4/4)
 */

extern I2C_HandleTypeDef hi2c1;

#define RxSIZE  11
uint8_t RxData[RxSIZE];
uint8_t rxcount=0;
uint8_t txcount=0;

uint8_t startPosition = 0;
uint8_t bytesRrecvd = 0;
uint8_t bytesTransd = 0;

#define POLL_RATE 39 //ms, approx (non interrupt), only applicable to the IMU

//TODO : remove the interrupt capability for IMU_INT

void setup(void){
	//adnsEnableDebugReports();
	adnsInit();
	printf("ADNS should now be initialized\n");

	if(bno_setup()) printf("IMU initialized successfully\n");
	else printf("=== Could NOT initialize the BNO085 ! ===\n");
	bno_enable_rotation_vector(40);
}

void loop(void){

	computePosition(POLL_RATE);


	float theta = getHeading();

	uint8_t *theta_split = (uint8_t *)&theta;
	I2C_REGISTERS[0]=(theta_split[0]);
	I2C_REGISTERS[1]=(theta_split[1]);
	I2C_REGISTERS[2]=(theta_split[2]);
	I2C_REGISTERS[3]=(theta_split[3]);

	printf("theta %.2f %d %d %d %d \n",theta, I2C_REGISTERS[0],I2C_REGISTERS[1],I2C_REGISTERS[2],I2C_REGISTERS[3]);

	//HAL_I2C_Slave_Transmit_IT(hi2c2, 0x52, data_buffer, 10);

}

/*
 * Voir https://controllerstech.com/stm32-as-i2c-slave-part-6/#info_box
 * (j'ai déjà téléchargé le code, pas la peine de le refaire)
 *
 * il fait concrêtement exactement ce que je veux, à l'exception qu'il fait QUE des registres, et que j'aurais besoin dans mon cas de commandes également
 * pour ça, prévoir une addresse en dehors des registres (juste avant ? 0x00 ? a voir) qui le passe en "mode commande",
 * dans lequel le but n'est pas de foutre la data reçue dans le tableau des registres mais de le mettre dans un endroit accessible plus tard
 * par un parser
 */


void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
	for(int i = 0; i <rxcount-1; i++){
		printf("%d ",RxData[i]);
	} printf("\n");

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
		//printf("sent adress %d equals to %d \n",startPosition+txcount, I2C_REGISTERS[startPosition+txcount]);
		txcount++;
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//will try to send the last byte, which will fail because the master already received the right number of bytes
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &I2C_REGISTERS[startPosition+txcount], 1, I2C_NEXT_FRAME);
	printf("sent adress %d equals to %d \n",startPosition+txcount, I2C_REGISTERS[startPosition+txcount]);
	txcount++; // WARNING : txcount will always be greater than the actual number of bytes received by the master
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if (rxcount < RxSIZE){
		printf("received %d \n",rxcount);
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
	printf("I2C error %ld", error_code);
	if(error_code == 4) printf(" (Master has terminated the communication)");
	printf("\n");

	if(txcount != 0){

	}
	printf("rx %d tx %d \n", rxcount, txcount);
	HAL_I2C_EnableListen_IT(hi2c);
}

