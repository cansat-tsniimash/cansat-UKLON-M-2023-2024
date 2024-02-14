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
#include "ADS1115/ADS1115.h"
#include "nRF24L01_PL/nrf24_upper_api.h"
#include "nRF24L01_PL/nrf24_lower_api_stm32.h"

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;

typedef enum
{
	RADIO_PACKET,
	RADIO_WAIT,
	RADIO_PACKET1,
	RADIO_PACKET2,
} radio_t;

typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	uint16_t acc[3]; /* Данные акселерометра */
	uint16_t gyr[3];/* Данные  гироскопа*/
	uint16_t mag[3];/*Данные магнитометра*/
	uint16_t crc;
}packet_imu_t;
typedef struct {
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	float lat;/*широта*/
	float lon;/*долгота*/
	float height;/*высота*/
	uint8_t fix;
	uint16_t DS_temp;
	uint16_t crc;
}packet_atgm_t;
typedef struct {
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	float lat;/*широта*/
	float lon;/*долгота*/
	float height;/*высота*/
	uint8_t fix;
	uint16_t crc;
}packet_NEO6M_t;
typedef struct {
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	float CO;
	float NO2;
	float NH3;
	uint32_t pres; /*давление*/
	uint8_t hum; /*влажность*/
	int16_t temp; /*температура*/
	uint16_t crc;
}packet_MICS_t;
typedef struct {
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	float roll;
	float yaw;
	float pitch;
	uint32_t pres; /*давление*/
	int16_t temp; /*температура*/
	uint16_t crc;
}packet_GY25_t;

typedef struct {
	uint16_t flag;
	uint16_t id;
	uint32_t time;
	uint32_t pres; /*давление*/
	int16_t temp; /*температура*/
	uint32_t accel[3];
	uint16_t crc;
}packet_t;

void app_main(){
	shift_reg_t sr_imu;
	sr_imu.bus = &hspi2;
	sr_imu.latch_port = GPIOC;
	sr_imu.latch_pin = GPIO_PIN_1;
	sr_imu.oe_port = GPIOC;
	sr_imu.oe_pin = GPIO_PIN_13;
	shift_reg_init(&sr_imu);
	shift_reg_write_16(&sr_imu, 0xFFFF);

	shift_reg_t sr_nrf;
	sr_nrf.bus = &hspi2;
	sr_nrf.latch_port = GPIOC;
	sr_nrf.latch_pin = GPIO_PIN_4;
	sr_nrf.oe_port = GPIOC;
	sr_nrf.oe_pin = GPIO_PIN_5;
	shift_reg_init(&sr_nrf);
	shift_reg_write_8(&sr_nrf, 0xFF);

	nrf24_spi_pins_sr_t spi_nrf24;
	spi_nrf24.pos_CE = 0;
	spi_nrf24.pos_CS = 1;
	spi_nrf24.this = &sr_nrf;
	nrf24_lower_api_config_t nrf24;
	nrf24_spi_init_sr(&nrf24, &hspi2, &spi_nrf24);

	nrf24_mode_power_down(&nrf24);

	nrf24_rf_config_t nrf24_rf_config;
	nrf24_rf_config.data_rate = NRF24_DATARATE_250_KBIT;
	nrf24_rf_config.rf_channel = 95;
	nrf24_rf_config.tx_power = NRF24_TXPOWER_MINUS_18_DBM;
	nrf24_setup_rf(&nrf24, &nrf24_rf_config);

	nrf24_protocol_config_t nrf24_protocol_config;
	nrf24_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf24_protocol_config.auto_retransmit_count = 0;
	nrf24_protocol_config.auto_retransmit_delay = 0;
	nrf24_protocol_config.crc_size = NRF24_CRCSIZE_DISABLE;
	nrf24_protocol_config.en_ack_payload = true;
	nrf24_protocol_config.en_dyn_ack = true;
	nrf24_protocol_config.en_dyn_payload_size = true;
	nrf24_setup_protocol(&nrf24, &nrf24_protocol_config);
	nrf24_pipe_set_tx_addr(&nrf24, 0xacacacacac);

	nrf24_mode_standby(&nrf24);
	nrf24_mode_tx(&nrf24);


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

	ads1115_t ADS;
	ADS.hi2c = &hi2c1;
	ADS.DevAddress = 0b1001000 << 1;
	ads1115_init(&ADS);

	struct bme280_data bme_data;
	packet_imu_t pack_imu;
	float temp_lis, temp_lsm;
	float mag[3];
	float acc_g[3];
	float gyro_dps[3];
	uint16_t raw_t;
	bool crc_ok;
	uint32_t first = HAL_GetTick();
	volatile float lux;
	while(1){
		uint16_t ads_raw[3];
		float ads_conv[3];

		lux = photorezistor_get_lux(pht);
		lisread(&lis, &temp_lis, &mag);
		lsmread(&lsm, &temp_lsm, &acc_g, &gyro_dps);
		for(int i = 0; i < 3; i++){
			pack_imu.mag[i] = mag[i] * 1000;
			pack_imu.acc[i] = acc_g[i] * 1000;
			pack_imu.gyr[i] = gyro_dps[i] * 1000;
		}
		bme280_get_sensor_data(BME280_ALL, &bme_data, &bme);
		if(HAL_GetTick() >= first + 750){
			ds18b20_read_raw_temperature(&ds, &raw_t, &crc_ok);
			ds18b20_start_conversion(&ds);
			first = HAL_GetTick();

		nrf24_fifo_status_t rx_status;
		nrf24_fifo_status_t tx_status;
		int test = 0;
		uint8_t buf[32];
		uint32_t radio_time;
		int next_stade;
		switch (test)
		{
		case RADIO_WAIT:
			nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
			if(tx_status == NRF24_FIFO_EMPTY)
				test = next_stade;
			if (HAL_GetTick() - radio_time > 50)
			{
				nrf24_fifo_flush_tx(&nrf24);
				test = next_stade;
			}
			break;
		case RADIO_PACKET1:
			nrf24_fifo_write(&nrf24, buf, 32, false);
			radio_time = HAL_GetTick();
			test = RADIO_WAIT;
			next_stade = RADIO_PACKET2;
		case RADIO_PACKET2:
			nrf24_fifo_write(&nrf24, buf, 32, false);
			radio_time = HAL_GetTick();
			test = RADIO_WAIT;
			next_stade = RADIO_PACKET1;
		}




		}
		for(int i = 0; i < 3; i++){
			ads1115_write_mux(i+4, &ADS);
			ads1115_req_single(&ADS);
			HAL_Delay(1);
			ads1115_read_single(&ADS, ads_raw[i]);
			ads_conv[i] = ads1115_convert(&ADS, ads_raw[i]);
		}
	}
}

