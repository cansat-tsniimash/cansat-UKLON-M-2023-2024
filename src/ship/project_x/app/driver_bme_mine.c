/*
 * driver_bme_mine.c
 *
 *  Created on: 15 дек. 2023 г.
 *      Author: Install
 */


#include "driver_bme_mine.h"

BME280_INTF_RET_TYPE bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
	struct bus *spi_bus = (struct bus *)intf_ptr;
	shift_reg_write_bit_16(spi_bus->sr_imu, spi_bus->pin, false);
	reg_addr = reg_addr | (1 << 7);
	HAL_SPI_Transmit(spi_bus->hspi, &reg_addr, 1, 300);
	HAL_SPI_Receive(spi_bus->hspi, reg_data, len, 300);
	shift_reg_write_bit_16(spi_bus->sr_imu, spi_bus->pin, true);
	return 0;
}

BME280_INTF_RET_TYPE bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
	struct bus *spi_bus = (struct bus *)intf_ptr;
	shift_reg_write_bit_16(spi_bus->sr_imu, spi_bus->pin, false);
	reg_addr = reg_addr & ~(1 << 7);
	HAL_SPI_Transmit(spi_bus->hspi, &reg_addr, 1, 300);
	HAL_SPI_Transmit(spi_bus->hspi, (uint8_t *)reg_data, len, 300);
	shift_reg_write_bit_16(spi_bus->sr_imu, spi_bus->pin, true);
	return 0;
}
void bme_delay(uint32_t period, void *intf_ptr){
	if(period/1000 == 0){
		HAL_Delay(1);
	}
	else{
		HAL_Delay(period/1000);
	}
}
void bme_driver(struct bme280_dev *bme, struct bus *spi_bus){
	bme->read = bme_read;
	bme->write = bme_write;
	bme->delay_us = bme_delay;
	bme->intf_ptr = &spi_bus;
	bme->intf = BME280_SPI_INTF;
	bme280_soft_reset(bme);
	bme280_init(bme);
	bme->settings.osr_p = BME280_OVERSAMPLING_16X;
	bme->settings.osr_t = BME280_OVERSAMPLING_16X;
	bme->settings.osr_h = BME280_OVERSAMPLING_16X;
	bme->settings.filter = BME280_FILTER_COEFF_16;
	bme->settings.standby_time = BME280_STANDBY_TIME_1000_MS ;
	bme280_set_sensor_settings(BME280_STANDBY_SEL | BME280_FILTER_SEL | BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL , bme);
	bme280_set_sensor_mode(BME280_NORMAL_MODE, bme);
}

