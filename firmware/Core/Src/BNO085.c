#include "BNO085.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

static short _debug = 0;
// 1 for minimal
// 2 for all (including tranfer reports, LOTS of stuff)

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

uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
uint32_t timeStamp;
uint8_t calibrationStatus; //Byte R0 of ME Calibration Response

//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
//See the read metadata example for more info
int16_t rotationVector_Q1 = 14;
int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;
int16_t angular_velocity_Q1 = 10;
int16_t gravity_Q1 = 8;

/*============================ Debug ============================*/

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

/*============================ Hardware abstraction ============================*/

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
static inline bool _sensor_awaiting(){
	return !HAL_GPIO_ReadPin(INT_IMU_GPIO_Port, INT_IMU_Pin);
}

//Blocking wait for BNO080 to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
static bool _wait_for_int_blocking_timeout(uint8_t timeout){
	for (uint8_t counter = 0; counter < timeout; counter++){
		if (!HAL_GPIO_ReadPin(INT_IMU_GPIO_Port, INT_IMU_Pin))
			return (true);
		HAL_Delay(1);
	}
	return (false);
}
static bool _wait_for_int_blocking(){
	return _wait_for_int_blocking_timeout(BNO_STANDARD_INT_TIMEOUT);
}

/*============================ Low Level ============================*/

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
static bool _receive_packet(void){

	if (!_sensor_awaiting())
		return (false); //Data is not available

	//Get first four bytes to find out how much data we need to read
	_enable_slave();

	//Get the first four bytes, aka the packet header
	HAL_SPI_Receive(&hspi1, shtpHeader, 4, 100);

	//Calculate the number of data bytes in this packet
	uint16_t dataLength = (((uint16_t)shtpHeader[1]/*MSB*/) << 8) | ((uint16_t)shtpHeader[0]/*LSB*/);
	dataLength &= ~(1 << 15); //Clear the MSbit.
	//This bit indicates if this package is a continuation of the last. Ignore it for now.
	//TODO catch this as an error and exit
	if (dataLength == 0){
		//Packet is empty
		if (_debug) printf("Packet empty !");
		if (_debug == 2) print_header();
		return (false); //All done
	}

	dataLength -= 4; //Remove the header bytes from the data count
	//Read incoming data into the shtpData array
	if (dataLength > BNO_MAX_PACKET_SIZE)  dataLength = BNO_MAX_PACKET_SIZE;
	HAL_SPI_Receive(&hspi1, shtpData, dataLength, 500);

	_disable_slave(); //Release BNO080

	if(_debug == 1){printf("Packet successfully retrieved \n");print_packet();}

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
static bool _send_packet(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	//Wait for BNO080 to indicate it is available for communication
	if (_wait_for_int_blocking() == false)
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

static void _set_feature_command(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig){
	long microsBetweenReports = (long)timeBetweenReports * (long)1000;

	shtpData[0] = BNO_SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;								   //Feature flags
	shtpData[3] = 0;								   //Change sensitivity (LSB)
	shtpData[4] = 0;								   //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;								   //Batch Interval (LSB)
	shtpData[10] = 0;								   //Batch Interval
	shtpData[11] = 0;								   //Batch Interval
	shtpData[12] = 0;								   //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	  //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	  //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	_send_packet(CHANNEL_CONTROL, 17);
}

static float _quaternion_to_float(int16_t fixedPointValue, uint8_t qPoint){
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}


//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
static uint16_t _parse_input_report(void){
	printf("parsing report...\n");
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));

	// The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
	if(shtpHeader[2] == CHANNEL_GYRO) {
		rawQuatI = (uint16_t)shtpData[1] << 8 | shtpData[0];
		rawQuatJ = (uint16_t)shtpData[3] << 8 | shtpData[2];
		rawQuatK = (uint16_t)shtpData[5] << 8 | shtpData[4];
		rawQuatReal = (uint16_t)shtpData[7] << 8 | shtpData[6];
		rawFastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
		rawFastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
		rawFastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

		return BNO_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
	}

	uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
	uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
	uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports
	uint16_t data6 = 0;

	if (dataLength - 5 > 9){
		data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if (dataLength - 5 > 11){
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}
	if (dataLength - 5 > 13){
		data6 = (uint16_t)shtpData[5 + 15] << 8 | shtpData[5 + 14];
	}

	//Store these generic values to their proper global variable
	switch (shtpData[5]){
	/*case (BNO_REPORTID_ACCELEROMETER):
		accelAccuracy = status;
		rawAccelX = data1;
		rawAccelY = data2;
		rawAccelZ = data3;
		break;
	case (BNO_REPORTID_LINEAR_ACCELERATION):
		accelLinAccuracy = status;
		rawLinAccelX = data1;
		rawLinAccelY = data2;
		rawLinAccelZ = data3;
		break;
	case (BNO_REPORTID_GYROSCOPE):
		gyroAccuracy = status;
		rawGyroX = data1;
		rawGyroY = data2;
		rawGyroZ = data3;
		break;
	case (BNO_REPORTID_UNCALIBRATED_GYRO):
		UncalibGyroAccuracy = status;
		rawUncalibGyroX = data1;
		rawUncalibGyroY = data2;
		rawUncalibGyroZ = data3;
		rawBiasX  = data4;
		rawBiasY  = data5;
		rawBiasZ  = data6;
		break;
	case (BNO_REPORTID_MAGNETIC_FIELD):
		magAccuracy = status;
		rawMagX = data1;
		rawMagY = data2;
		rawMagZ = data3;
		break;*/
	case (BNO_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR):
	case (BNO_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR):
	case (BNO_REPORTID_GAME_ROTATION_VECTOR):
	case (BNO_REPORTID_ROTATION_VECTOR):
		quatAccuracy = status;
		rawQuatI = data1;
		rawQuatJ = data2;
		rawQuatK = data3;
		rawQuatReal = data4;
		//Only available on rotation vector and ar/vr stabilized rotation vector,
		// not game rot vector and not ar/vr stabilized rotation vector
		rawQuatRadianAccuracy = data5;
		break;
	/*case (BNO_REPORTID_TAP_DETECTOR):
		tapDetector = shtpData[5 + 4]; //Byte 4 only
		break;
	case (BNO_REPORTID_STEP_COUNTER):
		stepCount = data3; //Bytes 8/9
		break;
	case (BNO_REPORTID_STABILITY_CLASSIFIER):
		stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
		break;
	case (BNO_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER):
		activityClassifier = shtpData[5 + 5]; //Most likely state

		//Load activity classification confidences into the array
		for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
			_activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
		break;
	case (BNO_REPORTID_RAW_ACCELEROMETER):
		memsRawAccelX = data1;
		memsRawAccelY = data2;
		memsRawAccelZ = data3;
		break;
	case (BNO_REPORTID_RAW_GYROSCOPE):
		memsRawGyroX = data1;
		memsRawGyroY = data2;
		memsRawGyroZ = data3;
		break;
	case (BN0_REPORTID_RAW_MAGNETOMETER):
		memsRawMagX = data1;
		memsRawMagY = data2;
		memsRawMagZ = data3;
		break;
	case (BNO_SHTP_REPORT_COMMAND_RESPONSE):
		if (_printDebug == true){
			_debugPort->println(F("!"));
		}
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE){
			if (_printDebug == true){
				_debugPort->println(F("ME Cal report found!"));
			}
			calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
		}
		break;
	case(BNO_REPORTID_GRAVITY):
		gravityAccuracy = status;
		gravityX = data1;
		gravityY = data2;
		gravityZ = data3;
		break;*/
	default :
		return 0;
	}
	//TODO additional feature reports may be strung together. Parse them all.

	return shtpData[5];
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
static uint16_t _parse_command_report(void){
	printf("parsing command...\n");
	if (shtpData[0] == BNO_SHTP_REPORT_COMMAND_RESPONSE){
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == BNO_COMMANDID_ME_CALIBRATE){
			calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
		}
		return shtpData[0];
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return 0;
}
/*============================ High Level ============================*/

bool bno_setup(void){

	_disable_slave();
	_reset_slave_blocking();

	//Wait for first assertion of INT before using WAK pin. Can take ~104ms
	if(!_wait_for_int_blocking()) return false;

	//At system startup, the hub must send its full advertisement message (see 5.2 and 5.3) to the
	//host. It must not send any other data until this step is complete.
	//When BNO080 first boots it broadcasts big startup packet
	//Read it and dump it
	if(!_wait_for_int_blocking()) return false; //Wait for assertion of INT before reading advert message.
	_receive_packet();

	//The BNO080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)
	//Read it and dump it
	if(!_wait_for_int_blocking()) return false; //Wait for assertion of INT before reading Init response
	_receive_packet();

	//Check communication with device
	shtpData[0] = BNO_SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	if(!_send_packet(CHANNEL_CONTROL, 2)){
		printf("Send command to the BNO085 failed");
		return false;
	}

	//Now we wait for response
	if(!_wait_for_int_blocking()) return false;
	if (_receive_packet() && shtpData[0] == BNO_SHTP_REPORT_PRODUCT_ID_RESPONSE){
		if (_debug){
			printf("SW Version Major: 0x%04X", shtpData[2]);
			printf(" SW Version Minor: 0x%04X \n", shtpData[3]);
			uint32_t SW_Part_Number = ((uint32_t)shtpData[7] << 24) | ((uint32_t)shtpData[6] << 16) | ((uint32_t)shtpData[5] << 8) | ((uint32_t)shtpData[4]);
			printf("SW Part Number: %ld \n",SW_Part_Number);
			uint32_t SW_Build_Number = ((uint32_t)shtpData[11] << 24) | ((uint32_t)shtpData[10] << 16) | ((uint32_t)shtpData[9] << 8) | ((uint32_t)shtpData[8]);
			printf("SW Build Number: %ld \n", SW_Build_Number);
			uint16_t SW_Version_Patch = ((uint16_t)shtpData[13] << 8) | ((uint16_t)shtpData[12]);
			printf("SW Version Patch: %d \n",SW_Version_Patch);
		}
		return (true);
	}
	return (false); //Something went wrong
}

void bno_enable_rotation_vector(uint16_t timeBetweenReports){
	_set_feature_command(BNO_REPORTID_ROTATION_VECTOR, timeBetweenReports, 0);
}

uint16_t bno_get_readings(void){

	if (!_sensor_awaiting())
		return (0); //Data is not available
	printf("%d",shtpHeader[2]);
	if (_receive_packet() == true){
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == BNO_SHTP_REPORT_BASE_TIMESTAMP){
			return _parse_input_report(); //This will update the rawAccelX, etc variables depending on which feature report is found
		} else if (shtpHeader[2] == CHANNEL_CONTROL){
			return _parse_command_report(); //This will update responses to commands, calibrationStatus, etc.
		} else if (shtpHeader[2] == CHANNEL_GYRO){
			return _parse_input_report();
		}
	}
	return 0;
}

float bno_get_yaw(void){
	 // get quaternion arguments
	float dqw = _quaternion_to_float(rawQuatReal, rotationVector_Q1);
	float dqx = _quaternion_to_float(rawQuatI, rotationVector_Q1);
	float dqy = _quaternion_to_float(rawQuatJ, rotationVector_Q1);
	float dqz = _quaternion_to_float(rawQuatK, rotationVector_Q1);

	//printf("%d %d %d %d \n", rawQuatReal,rawQuatI,rawQuatJ,rawQuatK);

	float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;

	// yaw (z-axis rotation)
	float t3 = +2.0 * (dqw * dqz + dqx * dqy);
	float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
	float yaw = atan2(t3, t4);

	return (yaw);
}


