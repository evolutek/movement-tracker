#include "ADNS9800.h"
#include "ADNS9800_firmware.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

uint8_t initComplete=0;
uint8_t testctr=0;
SPI_HandleTypeDef* _spi_port;

long _xydat[2];
float _xydelta[2] = {0,0};
float _delta_y = 0;
float _delta_x = 0;
float _units_per_millimeter = DEFAULT_COEF;
double _current_calculated_x = 0;
double _current_calculated_y = 0;

bool _debug = 0;
bool _reports = 0;


/*============================ Public ============================*/

void adnsInit(SPI_HandleTypeDef* spi_port){
	_spi_port = spi_port;
	//SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

	_adnsPerformStartup();
	if (_debug) _adnsDispRegisters();
	HAL_Delay(100);
	initComplete=9;

	if(_debug) printf("The optical sensor should now be initialized");
}

void adnsUpdate(void) {
	//SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
	_adnsUpdatePointer();

	_xydelta[0] = _adnsConvTwosComp(_xydat[0]);
	_xydelta[1] = - _adnsConvTwosComp(_xydat[1]);

}

void adnsReset(void){
    _current_calculated_x = 0;
    _current_calculated_y = 0;
}

void adnsSetCoef(float coef){
	_units_per_millimeter = coef;
}

void adnsEnableReports(bool state){
	_debug = state;
}

float adnsX(void){
		return _current_calculated_x/_units_per_millimeter;
}

float adnsY(void){
		return _current_calculated_y/_units_per_millimeter;
}

/*============================ Private ============================*/

void _adnsComBegin(){
	HAL_GPIO_WritePin(CS_ADNS_GPIO_Port, CS_ADNS_Pin, GPIO_PIN_RESET);
}

void _adnsComEnd(){
	HAL_GPIO_WritePin(CS_ADNS_GPIO_Port, CS_ADNS_Pin, GPIO_PIN_SET);
}

uint8_t _adnsReadReg(uint8_t ADNS_REG_addr){
  _adnsComBegin();

  // send adress of the register, with MSBit = 0 to indicate it's a read
  uint8_t buffer = ADNS_REG_addr & 0x7f;
  HAL_SPI_Transmit(_spi_port, &buffer, 1, 100);

  delayMicroseconds(100); // tSRAD
  // read data

  uint8_t data = 0x00;
  HAL_SPI_Receive(_spi_port, &data, 1, 100);

  delayMicroseconds(1); // tSCLK-_ncs for read operation is 120ns
  _adnsComEnd();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-_ncs

  return data;
}

void _adnsWriteReg(uint8_t ADNS_REG_addr, uint8_t data){
	_adnsComBegin();

	//send adress of the register, with MSBit = 1 to indicate it's a write

	uint8_t register_buffer = ADNS_REG_addr | 0x80;
    HAL_SPI_Transmit(_spi_port, &register_buffer, 1, 100);
    //sent data
    uint8_t data_buffer = data;
    HAL_SPI_Transmit(_spi_port, &data_buffer, 1, 100);

    delayMicroseconds(20); // tSCLK-_ncs for write operation
    _adnsComEnd();
    delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-_ncs. Could be shortened, but is looks like a safe lower bound
}

void _adnsUploadFirmware(){
  // send the firmware to the chip, cf p.18 of the datasheet

  if (_debug) printf("Uploading optical sensors's firmware...");

  // set the configuration_IV register in 3k firmware mode
  _adnsWriteReg(ADNS_REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved

  // write 0x1d in SROM_enable reg for initializing
  _adnsWriteReg(ADNS_REG_SROM_Enable, 0x1d);

  // wait for more than one frame period
  HAL_Delay(10);// assume that the frame rate is as low as 100fps... even if it should never be that low

  // write 0x18 to SROM_enable to start SROM download
  _adnsWriteReg(ADNS_REG_SROM_Enable, 0x18);

  // write the SROM file (=firmware data)
  _adnsComBegin();

  uint8_t buffer = ADNS_REG_SROM_Load_Burst | 0x80;
  HAL_SPI_Transmit(_spi_port, &buffer, 1, 100); // write burst destination adress

  delayMicroseconds(15);

  // send all uint8_ts of the firmware

  HAL_SPI_Transmit(_spi_port, (uint8_t *)&_adns_firmware_data, ADNS_FIRMWARE_LENGHT, 100);
  /* déjà géré par la fonction transmit . . . si j'ai bien compris
  unsigned char c;
  for(int i = 0; i < ADNS_FIRMWARE_LENGHT; i++){
    c = (unsigned char)pgm_read_uint8_t(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  */
  _adnsComEnd();
  }

void _adnsPerformStartup(void){
  _adnsComEnd(); // ensure that the serial port is reset
  _adnsComBegin(); // ensure that the serial port is reset
  _adnsComEnd(); // ensure that the serial port is reset
  _adnsWriteReg(ADNS_REG_Power_Up_Reset, 0x5a); // force reset
  HAL_Delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  _adnsReadReg(ADNS_REG_Motion);
  _adnsReadReg(ADNS_REG_delta_x_L);
  _adnsReadReg(ADNS_REG_delta_x_H);
  _adnsReadReg(ADNS_REG_delta_y_L);
  _adnsReadReg(ADNS_REG_delta_y_H);
  // upload the firmware
  _adnsUuploadFirmware();
  HAL_Delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytess (like by writing 0x00...) it would not work.
  uint8_t laser_ctrl0 = _adnsReadReg(ADNS_REG_LASER_CTRL0);
  _adnsWriteReg(ADNS_REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );

  HAL_Delay(1);

  if (_debug) printf("Optical Chip Initialized");
  }

void _adnsUpdatePointer(void){
	_adnsComBegin();
	_xydat[0] = (_adnsReadReg(ADNS_REG_delta_x_L) | (_adnsReadReg(ADNS_REG_delta_x_H) << 8));
	_xydat[1] = (_adnsReadReg(ADNS_REG_delta_y_L) | (_adnsReadReg(ADNS_REG_delta_y_H) << 8));
	_adnsComEnd();
}

void _dispRegisters(void){
	int oreg[7] = {0x00,0x3F,0x2A,0x02};
	char* oregname[] = {"Product_ID","Inverse_Product_ID","SROM_Version","Motion"};
	uint8_t regres;

	_adnsComBegin();

	for(int i=0; i<4; i++){
		uint8_t buffer = oreg[i];
		HAL_SPI_Transmit(_spi_port, (uint8_t *)&buffer, 1, 100); // write burst destination adress
		HAL_Delay(1);

		printf("ADNS9800 Registers :");
		printf(oregname[i]);
		printf("%#02X", oreg[i]);

		HAL_SPI_Receive(_spi_port, &regres, 1, 100);
		printf(" %#02X",regres);

		HAL_Delay(1);
	}
	_adnsComEnd();
}

long _adnsConvTwosComp(long b){
  //Convert from 2's complement
  if(b & 0x8000){
    b = -1 * ((b ^ 0xffff) + 1);
    }
  return b;
}

