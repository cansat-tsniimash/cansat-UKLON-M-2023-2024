/*
 * driver_bme_mine.c
 *
 *  Created on: 15 дек. 2023 г.
 *      Author: Install
 */


#include "driver_bme_mine.h"
#include "ADS1115/i2c-crutch.h"


BME280_INTF_RET_TYPE bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
	struct bus *i2c_bus = (struct bus *)intf_ptr;
	int res = HAL_I2C_Master_Transmit(i2c_bus->hi2c, i2c_bus->addr, &reg_addr, 1, 100);
	if (res != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(i2c_bus->hi2c, 100);
		return res;
	}
	res = HAL_I2C_Master_Receive(i2c_bus->hi2c, i2c_bus->addr, reg_data, len, 100);
	if (res != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(i2c_bus->hi2c, 100);
	}
	return 0;
}

BME280_INTF_RET_TYPE bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
	struct bus *i2c_bus = (struct bus *)intf_ptr;
	uint8_t buf[2] = {reg_addr, 0};
	for(int i= 0; i<len; i++){
		buf[1] = reg_data[i];
		int res = HAL_I2C_Master_Transmit(i2c_bus->hi2c, i2c_bus->addr, buf, 2, 100);
		if (res != HAL_OK)
		{
			I2C_ClearBusyFlagErratum(i2c_bus->hi2c, 100);
			return res;
		}
		buf[0] += 1;
	}
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
void bme_driver(struct bme280_dev *bme, struct bus *i2c_bus){
	bme->read = bme_read;
	bme->write = bme_write;
	bme->delay_us = bme_delay;
	bme->intf_ptr = i2c_bus;
	bme->intf = BME280_I2C_INTF;
	bme280_soft_reset(bme);
	bme280_init(bme);
	bme->settings.osr_p = BME280_OVERSAMPLING_16X;
	bme->settings.osr_t = BME280_OVERSAMPLING_16X;
	bme->settings.osr_h = BME280_OVERSAMPLING_16X;
	bme->settings.filter = BME280_FILTER_COEFF_16;
	bme->settings.standby_time = BME280_STANDBY_TIME_0_5_MS ;
	bme280_set_sensor_settings(BME280_STANDBY_SEL | BME280_FILTER_SEL | BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL, bme);
	bme280_set_sensor_mode(BME280_NORMAL_MODE, bme);
}

