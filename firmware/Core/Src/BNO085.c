#include "BNO085.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <stdbool.h>

static bool _debug = 1;

//Registers
static const uint8_t CHANNEL_COMMAND = 0;
static const uint8_t CHANNEL_EXECUTABLE = 1;
static const uint8_t CHANNEL_CONTROL = 2;
static const uint8_t CHANNEL_REPORTS = 3;
static const uint8_t CHANNEL_WAKE_REPORTS = 4;
static const uint8_t CHANNEL_GYRO = 5;

//Global Variables
static uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
static uint8_t shtpData[BNO_MAX_PACKET_SIZE];
static uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
static uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
static uint32_t metaData[BNO_MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

static inline void _enable_slave(){
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
}

static inline void _disable_slave(){
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
}


static inline void _reset_slave_blocking(){
	HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(RST_IMU_GPIO_Port, RST_IMU_Pin, GPIO_PIN_SET);
}

//Blocking wait for BNO080 to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
static bool _wait_for_bno_int_blocking_timeout(uint8_t timeout){
	for (uint8_t counter = 0; counter < timeout; counter++){
		if (!HAL_GPIO_ReadPin(INT_IMU_GPIO_Port, INT_IMU_Pin))
			return (true);
		HAL_Delay(1);
	}
	return (false);
}

static bool _wait_for_bno_int_blocking(){
	return _wait_for_bno_int_blocking_timeout(BNO_STANDARD_INT_TIMEOUT);
}

bool bno_data_available(){
	return !HAL_GPIO_ReadPin(INT_IMU_GPIO_Port, INT_IMU_Pin);
}

void print_header(void){
		//Print the four byte header
		printf("Header:");
		for (uint8_t x = 0; x < 4; x++)
		{
			printf(" ");
			if (shtpHeader[x] < 0x10)
				printf("0");
			printf("%01X",shtpHeader[x]);
		}
		printf("\n");
}

void print_packet(void){

	uint16_t packetLength = (uint16_t)shtpHeader[1] << 8 | shtpHeader[0];

	//Print the four byte header
	print_header();

	uint16_t printLength = packetLength - 4;
	if (printLength > 40){printLength = 40; printf("(Shortened) ");} //Artificial limit. We don't want the phone book.

	printf("Body:");
	for (uint8_t x = 0; x < printLength; x++)
	{
		printf(" ");
		if (shtpData[x] < 0x10)
			printf("0");
		printf("%01X",shtpData[x]);
	}
	printf("\n");
	if (packetLength & 1 << 15){
		printf(" [Continued packet] ");
		packetLength &= ~(1 << 15);
	}

	printf("Length: %u\n", packetLength);

	printf("Channel: ");
	switch (shtpHeader[2]){
		case 0: printf("Command"); break;
		case 1: printf("Executable"); break;
		case 2: printf("Control"); break;
		case 3: printf("Sensor-report"); break;
		case 4: printf("Wake-report"); break;
		case 5: printf("Gyro-vector"); break;
		default: printf("Raw header : %hu", shtpHeader[2]); break;
	}
	printf("\n");
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool _receive_packet(void){

	if (!bno_data_available())
		return (false); //Data is not available

	//Get first four bytes to find out how much data we need to read
	_enable_slave();

	//Get the first four bytes, aka the packet header
	HAL_SPI_Receive(&hspi1, &shtpHeader[0], 1, 100); // aka LSB
	HAL_SPI_Receive(&hspi1, &shtpHeader[1], 1, 100); // aka MSB
	HAL_SPI_Receive(&hspi1, &shtpHeader[2], 1, 100); // aka channel number
	HAL_SPI_Receive(&hspi1, &shtpHeader[3], 1, 100); // aka sequence number
	//HAL_SPI_Receive(&hspi1, &shtpHeader[0], 4, 100); // untested but should work . . . ?

	//Calculate the number of data bytes in this packet
	uint16_t dataLength = (((uint16_t)shtpHeader[1]/*MSB*/) << 8) | ((uint16_t)shtpHeader[0]/*LSB*/);
	dataLength &= ~(1 << 15); //Clear the MSbit.
	//This bit indicates if this package is a continuation of the last. Ignore it for now.
	//TODO catch this as an error and exit
	if (dataLength == 0)
	{
		//Packet is empty
		if (_debug) {print_header();printf("Packet empty !");}
		return (false); //All done
	}
	dataLength -= 4; //Remove the header bytes from the data count

	//Read incoming data into the shtpData array
	uint8_t data_buffer = 0xFF;
	for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
	{
		uint8_t incoming = 0;
		HAL_SPI_TransmitReceive(&hspi1, &data_buffer, &incoming, 1, 100);
		if (dataSpot < BNO_MAX_PACKET_SIZE)	//BNO080 can respond with upto 270 bytes, avoid overflow
			shtpData[dataSpot] = incoming; //Store data into the shtpData array
	}

	_disable_slave(); //Release BNO080

	if(_debug){printf("Packet successfully retrieved \n");print_packet();}


	// Quickly check for reset complete packet. No need for a seperate parser.
	// This function is also called after soft reset, so we need to catch this
	// packet here otherwise we need to check for the reset packet in multiple
	// places.
	if (shtpHeader[2] == CHANNEL_EXECUTABLE && shtpData[0] == BNO_EXECUTABLE_RESET_COMPLETE)
	{
		//_hasReset = true;
	}

	return (true); //We're done!
}

bool _send_packet(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	//Wait for BNO080 to indicate it is available for communication
	if (_wait_for_bno_int_blocking() == false)
		return false;

	//BNO080 has max CLK of 3MHz, MSB first,
	//The BNO080 uses CPOL = 1 and CPHA = 1. This is mode 3
	_enable_slave();

	//Send the 4 byte packet header
	uint8_t header_buffer[4];
	header_buffer[0] = (packetLength & 0xFF); //Packet length LSB // uhhhhhh & 0xFF ?
	header_buffer[1] = (packetLength >> 8); //Packet length MSB
	header_buffer[2] = channelNumber;
	header_buffer[3] = (sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel
	HAL_SPI_Transmit(&hspi1, header_buffer, 4, 200);

	//Send the user's data packet
	HAL_SPI_Transmit(&hspi1, shtpData, dataLength, 1000);

	_disable_slave();

	return (true);
}

bool setup_bno(void){

	_disable_slave();
	_reset_slave_blocking();

	//Wait for first assertion of INT before using WAK pin. Can take ~104ms
	if(!_wait_for_bno_int_blocking()) return false;

	//At system startup, the hub must send its full advertisement message (see 5.2 and 5.3) to the
	//host. It must not send any other data until this step is complete.
	//When BNO080 first boots it broadcasts big startup packet
	//Read it and dump it
	if(!_wait_for_bno_int_blocking()) return false; //Wait for assertion of INT before reading advert message.
	_receive_packet();

	//The BNO080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)
	//Read it and dump it
	if(!_wait_for_bno_int_blocking()) return false; //Wait for assertion of INT before reading Init response
	_receive_packet();

	//Check communication with device
	shtpData[0] = BNO_REG_SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	if(!_send_packet(CHANNEL_CONTROL, 2)){
		printf("Send command to the BNO085 failed");
		return false;
	}

	//Now we wait for response
	if(!_wait_for_bno_int_blocking()) return false;
	if (_receive_packet()){
		if (shtpData[0] == BNO_REG_SHTP_REPORT_PRODUCT_ID_RESPONSE){
			if (_debug){
				printf("SW Version Major: 0x%02X", shtpData[2]);
				printf(" SW Version Minor: 0x%02X \n", shtpData[3]);
				uint32_t SW_Part_Number = ((uint32_t)shtpData[7] << 24) | ((uint32_t)shtpData[6] << 16) | ((uint32_t)shtpData[5] << 8) | ((uint32_t)shtpData[4]);
				printf("SW Part Number: %ld \n",SW_Part_Number);
				uint32_t SW_Build_Number = ((uint32_t)shtpData[11] << 24) | ((uint32_t)shtpData[10] << 16) | ((uint32_t)shtpData[9] << 8) | ((uint32_t)shtpData[8]);
				printf("SW Build Number: %ld \n", SW_Build_Number);
				uint16_t SW_Version_Patch = ((uint16_t)shtpData[13] << 8) | ((uint16_t)shtpData[12]);
				printf("SW Version Patch: %d \n",SW_Version_Patch);
			}
			return (true);
		}
	}

	return (false); //Something went wrong
}
