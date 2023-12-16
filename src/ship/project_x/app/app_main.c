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
#include "LIS3MDL/DLIS3.h"
#include "LSM6DS3/DLSM.h"
#include "1Wire_DS18B20/one_wire.h"
#include "Photorezistor/photorezistor.h"

extern ADC_HandleTypeDef hadc1;
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

	stmdev_ctx_t lis;
	struct lis_spi_intf_sr spi_lis;
	spi_lis.sr_pin = 3;
	spi_lis.spi = &hspi2;
	spi_lis.sr = &sr_imu;
	lisset_sr(&lis, &spi_lis);

	stmdev_ctx_t lsm;
	struct lsm_spi_intf_sr spi_lsm;
	spi_lsm.spi = &hspi2;
	spi_lsm.sr_pin = 4;
	spi_lsm.sr = &sr_imu;
	lsmset_sr(&lsm, &spi_lsm);

	ds18b20_t ds;
	ds.onewire_port = GPIOA;
	ds.onewire_pin = 1;
	onewire_init(&ds);
	ds18b20_set_config(&ds, 100, 0, DS18B20_RESOLUTION_12_BIT);
	ds18b20_start_conversion(&ds);

	photorezistor_t pht;
	pht.resist = 2200;
	pht.hadc = &hadc1;

	struct bme280_data bme_data;
	float temp_lis, temp_lsm;
	float mag[3];
	float acc_g[3];
	float gyro_dps[3];
	uint16_t raw_t;
	bool crc_ok;
	uint32_t first = HAL_GetTick();
	volatile float lux;
	while(1){
		lux = photorezistor_get_lux(pht);
		lisread(&lis, &temp_lis, &mag);
		lsmread(&lsm, &temp_lsm, &acc_g, &gyro_dps);
		bme280_get_sensor_data(BME280_ALL, &bme_data, &bme);
		if(HAL_GetTick() >= first + 750){
			ds18b20_read_raw_temperature(&ds, &raw_t, &crc_ok);
			ds18b20_start_conversion(&ds);
			first = HAL_GetTick();
		}
	}
}

