/*
 * nunchuk.c
 *
 *  Created on: Mar 8, 2016
 *      Author: narmstrong
 */

#include "i2c.h"

#define NUNCHUK_I2C_ADDR 0x52

uint8_t write_buf_1[2] = {0xF0, 0x55};
uint8_t write_buf_2[2] = {0xFB, 0x00};
uint8_t write_buf_3[1] = {0x00};

int nunchuk_init()
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Master_Transmit(&hi2c3, NUNCHUK_I2C_ADDR<<1, write_buf_1, 2, 2);
	if (status != HAL_OK) {
		printf("Failed to init1 nunchuk...\r\n");
		return -1;
	}

	status = HAL_I2C_Master_Transmit(&hi2c3, NUNCHUK_I2C_ADDR<<1, write_buf_2, 2, 2);
	if (status != HAL_OK) {
		printf("Failed to init2 nunchuk...\r\n");
		return -1;
	}

	return 0;
}

int nunchuk_update(uint8_t *report)
{
	HAL_StatusTypeDef status;
	uint8_t data[6];

	status = HAL_I2C_Master_Transmit(&hi2c3, NUNCHUK_I2C_ADDR<<1, write_buf_3, 1, 2);
	if (status != HAL_OK) {
		printf("Failed to ping nunchuk...\r\n");
		return -1;
	}

	status = HAL_I2C_Master_Receive(&hi2c3, NUNCHUK_I2C_ADDR<<1, data, 6, 2);
	if (status != HAL_OK) {
		printf("Failed to get nunchuk data...\r\n");
		return -1;
	}

	if (data[0] == 255)
		data[0] = 254;
	if (data[0] == 0)
		data[0] = 1;

	// Invert Y
	data[1] = 255 - data[1];

	if (data[1] == 255)
		data[1] = 254;
	if (data[1] == 0)
		data[1] = 1;

	report[0] = ~(data[5]) & 3;
	report[1] = (int8_t)data[0] - 128;
	report[2] = (int8_t)data[1] - 128;

	return 0;
}
