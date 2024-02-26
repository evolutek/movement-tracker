#include "ADNS9800.h"
#include "ADNS9800_firmware.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include "micros.h"
#include <stdio.h>
#include <stdbool.h>

SPI_HandleTypeDef* _spi_port;

long _xydat[2]; // raw sensor data
long _xydelta[2]; // delta from last read in dots
double adns_delta[2]; // delta from last read in mm

float _units_per_millimeter = DEFAULT_COEF;

bool _debug = 0;

/*============================ Private ============================*/

void _adnsComBegin(){
	HAL_GPIO_WritePin(CS_ADNS_GPIO_Port, CS_ADNS_Pin, GPIO_PIN_RESET);
}

void _adnsComEnd(){
	HAL_GPIO_WritePin(CS_ADNS_GPIO_Port, CS_ADNS_Pin, GPIO_PIN_SET);
}

uint8_t _adnsReadReg(uint8_t ADNS_REG_addr){
  _adnsComBegin();

  ADNS_REG_addr &= 0x7f;
  HAL_SPI_Transmit(_spi_port, &ADNS_REG_addr, 1, 100);// send adress of the register, with MSBit = 0 to indicate it's a read

  delay_us(100); // tSRAD

  uint8_t data = 0x00;
  HAL_SPI_Receive(_spi_port, &data, 1, 100);// read data

  delay_us(1); // tSCLK-_ncs for read operation is 120ns
  _adnsComEnd();
  delay_us(19); //  tSRW/tSRR (=20us) minus tSCLK-_ncs

  return data;
}

void _adnsWriteReg(uint8_t ADNS_REG_addr, uint8_t data){
	_adnsComBegin();

	ADNS_REG_addr |= 0x80;
    HAL_SPI_Transmit(_spi_port, &ADNS_REG_addr, 1, 100);//send adress of the register, with MSBit = 1 to indicate it's a write

    HAL_SPI_Transmit(_spi_port, &data, 1, 100);//send data

    delay_us(20); // tSCLK-_ncs for write operation
    _adnsComEnd();
    delay_us(100); // tSWW/tSWR (=120us) minus tSCLK-_ncs. Could be shortened, but is looks like a safe lower bound
}

void _adnsUploadFirmware(){// send the firmware to the chip, cf p.18 of the datasheet

  if (_debug) printf("Uploading optical sensors's firmware... \n");

  _adnsWriteReg(ADNS_REG_Configuration_IV, 0x02); // set the configuration_IV register in 3k firmware mode   bit 1 = 1 for 3k mode, other bits are reserved

  _adnsWriteReg(ADNS_REG_SROM_Enable, 0x1d);// write 0x1d in SROM_enable reg for initializing

  HAL_Delay(10);// wait for more than one frame period assuming that the frame rate is as low as 100fps... even if it should never be that low

  _adnsWriteReg(ADNS_REG_SROM_Enable, 0x18); // write 0x18 to SROM_enable to start SROM download

  // write the SROM file (=firmware data)
  _adnsComBegin();

  uint8_t buffer = ADNS_REG_SROM_Load_Burst | 0x80;
  HAL_SPI_Transmit(_spi_port, &buffer, 1, 100); // write burst destination adress

  delay_us(15);

  // send all uint8_ts of the firmware (on ne peut pas utiliser la fonction transmit directement, car il faut respecter le délais de 15us)
  uint8_t c;
  for(int i = 0; i < ADNS_FIRMWARE_LENGHT; i++){
    c = _adns_firmware_data + i;
    HAL_SPI_Transmit(_spi_port, &c, 1, 100);
    delay_us(15);
  }

  _adnsComEnd();
  }

void _adnsPerformStartup(void){ // see datasheet page 20
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
  _adnsUploadFirmware();
  HAL_Delay(10);
  // enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytess (like by writing 0x00...) it would not work.
  uint8_t laser_ctrl0 = _adnsReadReg(ADNS_REG_LASER_CTRL0);
  _adnsWriteReg(ADNS_REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );

  HAL_Delay(1);

  if (_debug) printf("Optical chip started up \n");
}

void _adnsDispRegisters(void){ // display basic information of the module (primarely to check communication)
	int oreg[7] = {0x00,0x3F,0x2A,0x02};
	char* oregname[] = {"Product_ID","Inverse_Product_ID","SROM_Version","Motion"};
	uint8_t regres;

	_adnsComBegin();

	printf("ADNS9800 Registers : \n");
	for(int i=0; i<4; i++){
		uint8_t buffer = oreg[i];
		HAL_SPI_Transmit(_spi_port, &buffer, 1, 100); // write burst destination adress
		HAL_Delay(1);

		printf(oregname[i]);
		printf(" (at adress 0x%02X) :", oreg[i]);

		HAL_SPI_Receive(_spi_port, &regres, 1, 100);
		printf(" 0x%02X \n",regres);

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

void _adnsUpdatePointer(void){
	_adnsComBegin();
	_xydat[0] = (_adnsReadReg(ADNS_REG_delta_x_L) | (_adnsReadReg(ADNS_REG_delta_x_H) << 8));
	_xydat[1] = (_adnsReadReg(ADNS_REG_delta_y_L) | (_adnsReadReg(ADNS_REG_delta_y_H) << 8));
	_adnsComEnd();
}
/*============================ Public ============================*/

void adnsInit(SPI_HandleTypeDef* spi_port){
	_spi_port = spi_port;
	//SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

	_adnsPerformStartup();
	if (_debug) _adnsDispRegisters();
	if (_debug) printf("coef is set to %.4f dots per millimeter \n", _units_per_millimeter);
	HAL_Delay(100);

	if(_debug) printf("ADNS9800 initialization done \n");
}

bool adnsUpdate(void) {
	//SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

	_adnsUpdatePointer();

	_xydelta[0] = _adnsConvTwosComp(_xydat[0]);
	_xydelta[1] = - _adnsConvTwosComp(_xydat[1]);

	if (_xydelta[0] == 0 && _xydelta[1] == 0) return 0;

	adns_delta[0] = _xydelta[0] / _units_per_millimeter;
	adns_delta[1] = _xydelta[1] / _units_per_millimeter;

	return 1;
}

void adnsSetCoef(float coef){
	_units_per_millimeter = coef;
}

void adnsEnableDebugReports(void){
	adnsSetDebugReports(1);
}

void adnsSetDebugReports(bool state){
	printf("Reports for the ADNS 9800 are now set to %d \n", state);
	_debug = state;
}

double adnsX(void){
	return adns_delta[0];
}

double adnsY(void){
	return adns_delta[1];
}

