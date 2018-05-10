/*************************************************** 
  This is a library for the Adafruit Thermocouple Sensor w/MAX31856

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/product/3263
  
  These sensors use SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
****************************************************/

#include <stdlib.h>
#include "Adafruit_MAX31856.h"

// Hardware SPI init
Adafruit_MAX31856::Adafruit_MAX31856(SPI_HandleTypeDef* spi_port,
									 GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
	spi = spi_port;
	gpio_port = cs_port;
	gpio_pin = cs_pin;
}

bool Adafruit_MAX31856::begin(void) {
	// assert on any fault
	writeRegister8(MAX31856_MASK_REG, 0x0);
	writeRegister8(MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0);
	setThermocoupleType(MAX31856_TCTYPE_K);

	return true;
}


void Adafruit_MAX31856::setThermocoupleType(max31856_thermocoupletype_t type) {
	uint8_t t = readRegister8(MAX31856_CR1_REG);
	t &= 0xF0; // mask off bottom 4 bits
	t |= (uint8_t)type & 0x0F;
	writeRegister8(MAX31856_CR1_REG, t);
}

max31856_thermocoupletype_t Adafruit_MAX31856::getThermocoupleType(void) {
	uint8_t t = readRegister8(MAX31856_CR1_REG);
	t &= 0x0F;

	return (max31856_thermocoupletype_t)(t);
}

uint8_t Adafruit_MAX31856::readFault(void) {
	return readRegister8(MAX31856_SR_REG);
}

void Adafruit_MAX31856::setColdJunctionFaultThreshholds(int8_t low, int8_t high) {
	writeRegister8(MAX31856_CJLF_REG, low);
	writeRegister8(MAX31856_CJHF_REG, high);
}

void Adafruit_MAX31856::setTempFaultThreshholds(float flow, float fhigh) {
	int16_t low, high;

	flow *= 16;
	low = flow;

	fhigh *= 16;
	high = fhigh;

	writeRegister8(MAX31856_LTHFTH_REG, high >> 8);
	writeRegister8(MAX31856_LTHFTL_REG, high);

	writeRegister8(MAX31856_LTLFTH_REG, low >> 8);
	writeRegister8(MAX31856_LTLFTL_REG, low);
}

void Adafruit_MAX31856::oneShotTemperature(void) {

	writeRegister8(MAX31856_CJTO_REG, 0x0);

	uint8_t t = readRegister8(MAX31856_CR0_REG);

	t &= ~MAX31856_CR0_AUTOCONVERT; // turn off autoconvert!
	t |= MAX31856_CR0_1SHOT;

	writeRegister8(MAX31856_CR0_REG, t);
}

float Adafruit_MAX31856::readCJTemperature(void) {
	oneShotTemperature();

	int16_t temp16 = readRegister16(MAX31856_CJTH_REG);
	float tempfloat = temp16;
	tempfloat /= 256.0;

	return tempfloat;
}

float Adafruit_MAX31856::readThermocoupleTemperature(void) {
	oneShotTemperature();

	int32_t temp24 = readRegister24(MAX31856_LTCBH_REG);
	if (temp24 & 0x800000) {
		temp24 |= 0xFF000000;  // fix sign
	}

	temp24 >>= 5;  // bottom 5 bits are unused

	float tempfloat = temp24;
	tempfloat /= 128.0;

	return tempfloat;
}

/**********************************************/

uint8_t Adafruit_MAX31856::readRegister8(uint8_t addr) {
	uint8_t ret = 0;
	readRegisterN(addr, &ret, 1);

	return ret;
}

uint16_t Adafruit_MAX31856::readRegister16(uint8_t addr) {
	uint8_t buffer[2] = {0, 0};
	readRegisterN(addr, buffer, 2);

	uint16_t ret = buffer[0];
	ret <<= 8;
	ret |=  buffer[1];
  
	return ret;
}

uint32_t Adafruit_MAX31856::readRegister24(uint8_t addr) {
	uint8_t buffer[3] = {0, 0, 0};
	readRegisterN(addr, buffer, 3);

	uint32_t ret = buffer[0];
	ret <<= 8;
	ret |=  buffer[1];
	ret <<= 8;
	ret |=  buffer[2];
  
	return ret;
}


HAL_StatusTypeDef Adafruit_MAX31856::readRegisterN(uint8_t addr, uint8_t buffer[],
												   uint8_t n)
{
	addr &= 0x7F; // make sure top bit is not set

	// CS pin low
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);

	// Send the register address
	HAL_StatusTypeDef ok = HAL_SPI_Transmit(spi, &addr, 1, 10);
	if (ok != HAL_OK)
		return ok;

	// Read data
	ok = HAL_SPI_Receive(spi, buffer, n, 10);
	if (ok != HAL_OK)
		return ok;
	
	// CS pin high
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
	return HAL_OK;
}


HAL_StatusTypeDef Adafruit_MAX31856::writeRegister8(uint8_t addr, uint8_t data) {
	addr |= 0x80; // make sure top bit is set

	// CS pin low
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);

	// Prepare transmit buffer
	uint8_t tx[] = {addr, data};

	// Transmit data
	HAL_StatusTypeDef ok = HAL_SPI_Transmit(spi, tx, 2, 10);
	
	// CS pin high
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);

	return ok;
}
