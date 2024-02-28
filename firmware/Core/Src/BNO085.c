#include "BNO085.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <stdbool.h>


//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
boolean bno_init(uint8_t deviceAddress, TwoWire &wirePort, uint8_t intPin){
	_deviceAddress = deviceAddress; //If provided, store the I2C address from user
	_i2cPort = &wirePort;			//Grab which port the user wants us to use
	_int = intPin;					//Get the pin that the user wants to use for interrupts. By default, it's 255 and we'll not use it in dataAvailable() function.
	if (_int != 255)
	{
		pinMode(_int, INPUT_PULLUP);
	}

	//We expect caller to begin their I2C port, with the speed of their choice external to the library
	//But if they forget, we start the hardware here.
	//_i2cPort->begin();

	//Begin by resetting the IMU
	softReset();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			if (_printDebug == true)
			{
				_debugPort->print(F("SW Version Major: 0x"));
				_debugPort->print(shtpData[2], HEX);
				_debugPort->print(F(" SW Version Minor: 0x"));
				_debugPort->print(shtpData[3], HEX);
				uint32_t SW_Part_Number = ((uint32_t)shtpData[7] << 24) | ((uint32_t)shtpData[6] << 16) | ((uint32_t)shtpData[5] << 8) | ((uint32_t)shtpData[4]);
				_debugPort->print(F(" SW Part Number: 0x"));
				_debugPort->print(SW_Part_Number, HEX);
				uint32_t SW_Build_Number = ((uint32_t)shtpData[11] << 24) | ((uint32_t)shtpData[10] << 16) | ((uint32_t)shtpData[9] << 8) | ((uint32_t)shtpData[8]);
				_debugPort->print(F(" SW Build Number: 0x"));
				_debugPort->print(SW_Build_Number, HEX);
				uint16_t SW_Version_Patch = ((uint16_t)shtpData[13] << 8) | ((uint16_t)shtpData[12]);
				_debugPort->print(F(" SW Version Patch: 0x"));
				_debugPort->println(SW_Version_Patch, HEX);
			}
			return (true);
		}
	}

	return (false); //Something went wrong
}
