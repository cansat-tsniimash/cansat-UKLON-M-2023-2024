/*
 * app_main.c
 *
 *  Created on: 1 дек. 2023 г.
 *      Author: Install
 */
#include "Shift_Register/shift_reg.h"
#include "BME280/bme280.h"
#include "BME280/bme280_defs.h"
extern SPI_HandleTypeDef hspi2;
typedef struct bus{
	shift_reg_t *sr_imu;
	SPI_HandleTypeDef *hspi;
	uint8_t pin;
}bus_t;


BME280_INTF_RET_TYPE bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
	struct bus *spi_bus = (struct bus *)intf_ptr;
	shift_reg_write_bit_16(spi_bus->sr_imu, spi_bus->pin, false);
	reg_addr = reg_addr | (1 << 7);
	HAL_SPI_Transmit(spi_bus->hspi, &reg_addr, 1, 300);
	HAL_SPI_Receive(spi_bus->hspi, reg_data, len, 300);
	shift_reg_write_bit_16(spi_bus->sr_imu, spi_bus->pin, true);\
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
	bme.read = bme_read;
	bme.write = bme_write;
	bme.delay_us = bme_delay;
	bme.intf_ptr = &spi_bus;
	bme.intf = BME280_SPI_INTF;
	bme280_init(&bme);
	bme.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme.settings.osr_t = BME280_OVERSAMPLING_16X;
	bme.settings.osr_h = BME280_OVERSAMPLING_16X;
	bme.settings.filter = BME280_FILTER_COEFF_16;
	bme.settings.standby_time = BME280_STANDBY_TIME_1000_MS ;
	bme280_set_sensor_settings(BME280_STANDBY_SEL | BME280_FILTER_SEL | BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL , &bme);
	while(1){
		shift_reg_oe(&sr_imu, true);
		HAL_Delay(100);
		shift_reg_oe(&sr_imu, false);
		HAL_Delay(100);
	}
}

