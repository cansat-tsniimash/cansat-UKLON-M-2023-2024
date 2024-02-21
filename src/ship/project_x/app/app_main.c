/*
 * app_main.c
 *
 *  Created on: 1 дек. 2023 г.
 *      Author: Install
 */
#include "main.h"
#include <stm32f4xx.h>
#include "Shift_Register/shift_reg.h"
#include "BME280/bme280.h"
#include "BME280/bme280_defs.h"
#include "driver_bme_mine.h"
#include "LIS3MDL/DLIS3.h"
#include "LSM6DS3/DLSM.h"
#include <math.h>
#include "1Wire_DS18B20/one_wire.h"
#include "Photorezistor/photorezistor.h"
#include "ADS1115/ADS1115.h"
#include "nRF24L01_PL/nrf24_upper_api.h"
#include "nRF24L01_PL/nrf24_lower_api_stm32.h"
#include "BME280/DriverForBME280.h"
#include "fatfs.h"

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;

#define BURST_TIME 2000

typedef enum
{
	STATE_INIT,
	STATE_BEFORE_FLIGHT,
	STATE_ROCKET,
	STATE_DESCENT_A,
	STATE_DESCENT_B,
	STATE_DESCENT_C,

} machine_state_t;

typedef enum
{
	RADIO_PACKET_IMU,
	RADIO_PACKET_ATGM,
	RADIO_PACKET_NEO6M,
	RADIO_PACKET_MICS,
	RADIO_PACKET_GY25,
	RADIO_PACKET_ORG,
	RADIO_WAIT,

} radio_t;


#pragma pack(push,1)
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	int16_t acc[3]; /* Данные акселерометра */
	int16_t gyr[3];/* Данные  гироскопа*/
	int16_t mag[3];/*Данные магнитометра*/
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
	int16_t DS_temp;
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
	int16_t temp; /*температура*/
	uint32_t pres; /*давление*/
	int16_t accel[3];
	uint8_t crc;
}packet_t;
#pragma pack(pop)

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
	struct bme280_dev bmp;
	bme_driver(&bmp, &spi_bus);

	struct bme280_dev bme;
	struct bme_spi_intf bme_spi_intf;
	bme_spi_intf.GPIO_Pin = GPIO_PIN_11;
	bme_spi_intf.GPIO_Port = GPIOA;
	bme_spi_intf.spi = &hspi2;
	bme_init_default(&bme, &bme_spi_intf);

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


	packet_imu_t pack_imu;
	pack_imu.flag = 0xF1;
	pack_imu.num = 0;
	packet_t pack_org;
	pack_org.flag = 0xAAAA;
	packet_GY25_t pack_GY25;
	pack_GY25.flag = 0xF2;
	pack_GY25.num = 0;
	packet_MICS_t pack_MICS;
	pack_MICS.flag = 0xF3;
	pack_MICS.num = 0;
	packet_NEO6M_t pack_NEO6M;
	pack_NEO6M.flag = 0xAB;
	pack_NEO6M.num = 0;
	packet_atgm_t pack_atgm;
	pack_atgm.flag = 0xAC;
	pack_atgm.num = 0;


	uint16_t ads_raw[3];
	float ads_conv[4];
	float temp_lis, temp_lsm;
	float mag[3];
	float acc_g[3];
	float gyro_dps[3];
	uint16_t raw_temp;
	bool crc_ok;
	uint32_t first = HAL_GetTick();
	uint32_t wait_time = 0;

	float lux;
	nrf24_fifo_status_t rx_status;
	nrf24_fifo_status_t tx_status;
	int radio_state_now = RADIO_PACKET_ORG;
	int machine_state_now = STATE_BEFORE_FLIGHT;
	uint8_t buf[32] = "Hello, its me";
	uint32_t radio_time = 0;
	radio_t next_stade;
	machine_state_t next_state;
	int pkt_count = 0;
	int comp = 0;
	float our_light = 0;
	int num_light_take = 0;
	struct bme280_data bme_data;
	uint32_t pressure_on_ground;
	bme280_get_sensor_data(BME280_ALL, &bme_data, &bmp);
	pressure_on_ground = bme_data.pressure;
	FATFS fileSystem;
	FIL testFile;
	UINT testBytes;
	FRESULT res_mount = f_mount(&fileSystem, "", 1);
	if (res_mount == FR_OK)
	{
		FRESULT res = f_open(&testFile, "test.txt", FA_WRITE | FA_CREATE_ALWAYS);
		if (res == FR_OK)
		{
			res = f_write(&testFile, buf, sizeof(buf), &testBytes);
			if (res == FR_OK)
			{
				res = f_sync(&testFile);
				if (res == FR_OK)
				res = f_close(&testFile);
			}
		}
	}




	while(1){


		lux = photorezistor_get_lux(pht);
		lisread(&lis, &temp_lis, &mag);
		lsmread(&lsm, &temp_lsm, &acc_g, &gyro_dps);
		for(int i = 0; i < 3; i++){
			pack_imu.mag[i] = mag[i] * 1000;
			pack_imu.acc[i] = acc_g[i] * 1000;
			pack_imu.gyr[i] = gyro_dps[i] * 1000;
			pack_org.accel[i] = acc_g[i] * 1000;
		}
		bme280_get_sensor_data(BME280_ALL, &bme_data, &bmp);
		pack_GY25.temp = bme_data.temperature * 100;
		pack_GY25.pres = bme_data.pressure;
		pack_org.temp = bme_data.temperature * 100;
		pack_org.pres = bme_data.pressure;

		bme_data = bme_read_data(&bme);
		pack_MICS.temp = bme_data.temperature * 100;
		pack_MICS.pres = bme_data.pressure;
		pack_MICS.hum = bme_data.humidity;

		float height_on_BME280 = 44330.0*(1.0 - pow((float)bme_data.pressure/pressure_on_ground, 1.0/5.255));

		if(HAL_GetTick() >= first + 750)
		{
			ds18b20_read_raw_temperature(&ds, &raw_temp, &crc_ok);
			pack_atgm.DS_temp = raw_temp;
			ds18b20_start_conversion(&ds);
			first = HAL_GetTick();
		}

		for(int i = 0; i < 3; i++){
			ads1115_write_mux(i+4, &ADS);
			ads1115_req_single(&ADS);
			HAL_Delay(1);
			ads1115_read_single(&ADS, &ads_raw[i]);
			ads_conv[i] = ads1115_convert(&ADS, ads_raw[i]);
		}
		pack_MICS.CO = ads_conv[0];
		pack_MICS.NO2 = ads_conv[1];
		pack_MICS.NH3 = ads_conv[2];

		switch (machine_state_now){
		case STATE_INIT:
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET){
				if(wait_time == 0){
					wait_time = HAL_GetTick();
				}
				if (HAL_GetTick() >= wait_time + 5000){
					machine_state_now = next_state;
					our_light = our_light / num_light_take;
					wait_time = 0;
					break;
				}
				num_light_take += 1;
				our_light += lux;
			}
			break;
		case STATE_BEFORE_FLIGHT:
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET){
				machine_state_now = next_state;
			}
			break;
		case STATE_ROCKET:
			if(lux >= our_light * 0.8){
				machine_state_now = next_state;
			}
			break;
		case STATE_DESCENT_A:
			if(height_on_BME280 >= 150.0){
				machine_state_now = next_state;
			}
			break;
		case STATE_DESCENT_B:
			if (wait_time = 0){
				wait_time = HAL_GetTick();
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			}
			if (HAL_GetTick() >= wait_time + BURST_TIME){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				machine_state_now = next_state;
			}
			break;
		case STATE_DESCENT_C:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		}

		switch (radio_state_now)
		{
		case RADIO_WAIT:
			 if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET)
			 {
				nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
				nrf24_irq_get(&nrf24, &comp);
				nrf24_irq_clear(&nrf24, comp);
				if(tx_status == NRF24_FIFO_EMPTY)
					radio_state_now = next_stade;
			 }
			if (HAL_GetTick() - radio_time > 50)
			{
				nrf24_fifo_flush_tx(&nrf24);
				radio_state_now = next_stade;
			}
			break;
		case RADIO_PACKET_ORG:
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack_org, 32, false);//sizeof(packet_t), false);
			radio_time = HAL_GetTick();
			radio_state_now = RADIO_WAIT;
			next_stade = RADIO_PACKET_ATGM;
			break;
		case RADIO_PACKET_ATGM:
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack_atgm, 32, false);//sizeof(packet_atgm_t), false);
			radio_time = HAL_GetTick();
			radio_state_now = RADIO_WAIT;
			next_stade = RADIO_PACKET_NEO6M;
			break;
		case RADIO_PACKET_NEO6M:
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack_NEO6M, 32, false);//sizeof(packet_NEO6M_t), false);
			radio_time = HAL_GetTick();
			radio_state_now = RADIO_WAIT;
			next_stade = RADIO_PACKET_IMU;
			break;
		case RADIO_PACKET_IMU:
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack_imu, 32, false);
			radio_time = HAL_GetTick();
			radio_state_now = RADIO_WAIT;
			pkt_count++;
			if (pkt_count > 10)
			{
				next_stade = RADIO_PACKET_ORG;
				pkt_count = 0;
			}
			else
				next_stade = RADIO_PACKET_MICS;
			break;

		case RADIO_PACKET_MICS:
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack_MICS, 32, false);//sizeof(packet_MICS_t), false);
			radio_time = HAL_GetTick();
			radio_state_now = RADIO_WAIT;
			next_stade = RADIO_PACKET_GY25;
			break;
		case RADIO_PACKET_GY25:
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack_GY25, 32, false);//sizeof(packet_GY25_t), false);
			radio_time = HAL_GetTick();
			radio_state_now = RADIO_WAIT;
			next_stade = RADIO_PACKET_IMU;
			break;
		}


	}
}

