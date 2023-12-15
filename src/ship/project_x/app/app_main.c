/*
 * app_main.c
 *
 *  Created on: 1 дек. 2023 г.
 *      Author: Install
 */
#include "Shift_Register/shift_reg.h"
#include "BME280/bme280.h"
#include "BME280/bme280_defs.h"
#include "driver_bme_mine.h"

extern SPI_HandleTypeDef hspi2;
void app_main(){
	shift_reg_t sr_imu;
	sr_imu.bus = &hspi2;
	sr_imu.latch_port = GPIOC;
	sr_imu.latch_pin = GPIO_PIN_1;
	sr_imu.oe_port = GPIOC;
	sr_imu.oe_pin = GPIO_PIN_13;
	shift_reg_init(&sr_imu);
	shift_reg_write_16(&sr_imu, 0xFFFF);

	struct bus spi_bus;
	spi_bus.sr_imu = &sr_imu;
	spi_bus.hspi = &hspi2;
	spi_bus.pin = 2;

	struct bme280_dev bme;
	bme_driver(&bme, &spi_bus);


	struct bme280_data bme_data;
	while(1){


		bme280_get_sensor_data(BME280_ALL, &bme_data, &bme);
		shift_reg_oe(&sr_imu, true);
		HAL_Delay(100);
		shift_reg_oe(&sr_imu, false);
		HAL_Delay(100);
	}
}

