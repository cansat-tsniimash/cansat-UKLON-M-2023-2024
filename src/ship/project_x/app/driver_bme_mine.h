/*
 * driver_bme_mine.h
 *
 *  Created on: 15 дек. 2023 г.
 *      Author: Install
 */

#ifndef DRIVER_BME_MINE_H_
#define DRIVER_BME_MINE_H_

#include "Shift_Register/shift_reg.h"
#include "BME280/bme280.h"
#include "BME280/bme280_defs.h"

typedef struct bus{
	I2C_HandleTypeDef *hi2c;
	uint8_t addr;
}bus_t;

void bme_driver(struct bme280_dev *bme, struct bus *spi_bus);


#endif /* DRIVER_BME_MINE_H_ */
