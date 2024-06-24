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
#include <ATGM336H/nmea_gps.h>
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
#include "packet.h"
#include "csv.h"
#include "gy953/gy953spi.h"
#include "gy953spi_board_stm32f1.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern SPI_HandleTypeDef hspi;
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
	RADIO_PACKET_GY25_IMU,
	RADIO_PACKET_ORG,
	RADIO_WAIT,

} radio_t;


#include <stdio.h>

unsigned char bitwiseXORChecksum(const unsigned char *data, int length) {
    unsigned char checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void app_main()
{
	float lat;
	float lon;
	float alt;
	int fix_;
	int64_t cookie;
	shift_reg_t sr_imu;
	sr_imu.bus = &hspi2;
	sr_imu.latch_port = GPIOC;
	sr_imu.latch_pin = GPIO_PIN_1;
	sr_imu.oe_port = GPIOC;
	sr_imu.oe_pin = GPIO_PIN_13;
	shift_reg_init(&sr_imu);
	shift_reg_write_16(&sr_imu, 0xFFFF);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	shift_reg_t sr_nrf;
	sr_nrf.bus = &hspi2;
	sr_nrf.latch_port = GPIOC;
	sr_nrf.latch_pin = GPIO_PIN_4;
	sr_nrf.oe_port = GPIOC;
	sr_nrf.oe_pin = GPIO_PIN_5;
	shift_reg_init(&sr_nrf);
	shift_reg_write_8(&sr_nrf, 0xFF);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

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
	nrf24_rf_config.tx_power = NRF24_TXPOWER_MINUS_0_DBM;
	nrf24_setup_rf(&nrf24, &nrf24_rf_config);

	nrf24_protocol_config_t nrf24_protocol_config;
	nrf24_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf24_protocol_config.auto_retransmit_count = 0;
	nrf24_protocol_config.auto_retransmit_delay = 0;
	nrf24_protocol_config.crc_size = NRF24_CRCSIZE_1BYTE;
	nrf24_protocol_config.en_ack_payload = false;
	nrf24_protocol_config.en_dyn_ack = false;
	nrf24_protocol_config.en_dyn_payload_size = false;
	nrf24_setup_protocol(&nrf24, &nrf24_protocol_config);
	nrf24_pipe_set_tx_addr(&nrf24, 0xacacacacac);

	nrf24_pipe_config_t pipe_config;
	for (int i = 1; i < 6; i++)
	{
		pipe_config.address = 0xcfcfcfcfcf;
		pipe_config.address = (pipe_config.address & ~((uint64_t)0xff << 32)) | ((uint64_t)(i + 7) << 32);
		pipe_config.enable_auto_ack = true;
		pipe_config.payload_size = -1;
		nrf24_pipe_rx_start(&nrf24, i, &pipe_config);
	}

	nrf24_mode_standby(&nrf24);
	nrf24_mode_tx(&nrf24);

	struct bus i2c_bus;
	i2c_bus.addr = BME280_I2C_ADDR_PRIM << 1;
	i2c_bus.hi2c = &hi2c1;
	struct bme280_dev bme;
	bme_driver(&bme, &i2c_bus);

	struct bme280_dev bmp;
	struct bme_spi_intf_sr bmp_spi_intf;
	bmp_spi_intf.sr = &sr_imu;
	bmp_spi_intf.sr_pin = 2;
	bmp_spi_intf.spi = &hspi2;
	bme_init_default_sr(&bmp, &bmp_spi_intf);

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
	ds.onewire_pin = GPIO_PIN_1;
	onewire_init(&ds);
	ds18b20_set_config(&ds, 100, -100, DS18B20_RESOLUTION_12_BIT);
	ds18b20_start_conversion(&ds);

	photorezistor_t pht;
	pht.resist = 2200;
	pht.hadc = &hadc1;

	ads1115_t ADS;
	ADS.hi2c = &hi2c1;
	ADS.DevAddress = 0b1001000 << 1;
	ads1115_init(&ADS);


	packet_imu_union_t pack_imu = {0};
	pack_imu.pack.flag = 0xF1;
	pack_imu.pack.num = 0;
	packet_union_t pack_org = {0};
	pack_org.pack.flag = 0xAAAA;
	packet_GY25_union_t pack_GY25 = {0};
	pack_GY25.pack.flag = 0xF2;
	pack_GY25.pack.num = 0;
	packet_MICS_union_t pack_MICS = {0};
	pack_MICS.pack.flag = 0xF3;
	pack_MICS.pack.num = 0;
	packet_NEO6M_union_t pack_NEO6M = {0};
	pack_NEO6M.pack.flag = 0xAB;
	pack_NEO6M.pack.num = 0;
	packet_atgm_union_t pack_atgm = {0};
	pack_atgm.pack.flag = 0xAC;
	pack_atgm.pack.num = 0;
	packet_GY25_imu_union_t pack_GY25_imu = {0};
	pack_GY25_imu.pack.flag = 0xF4;
	pack_GY25_imu.pack.num = 0;


	uint16_t ads_raw[3];
	float ads_conv[4];
	uint16_t raw_temp;
	bool crc_ok;
	uint32_t first = HAL_GetTick();
	uint32_t wait_time = 0;

	float lux;
	nrf24_fifo_status_t rx_status;
	nrf24_fifo_status_t tx_status;
	radio_t radio_state_now = RADIO_PACKET_IMU;
	machine_state_t machine_state_now = STATE_INIT;

	uint32_t radio_time = 0;
	radio_t next_state;
	float sbros_height = 150;


	gps_init();
	//__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
	//__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
	uint64_t gps_time_s;
	uint32_t gps_time_us;

	int pkt_count = 0;
	int comp = 0;
	float our_light = 0;
	int num_light_take = 0;
	struct bme280_data bme_data;
	float height_on_BME280 = 0;
	uint32_t pressure_on_ground;
	bme280_get_sensor_data(BME280_ALL, &bme_data, &bmp);
	pressure_on_ground = bme_data.pressure;
	FATFS fileSystem;
	FIL imuFile;
	FIL logFile;
	FIL orgFile;
	FIL GY25File;
	FIL GY25imuFile;
	FIL MICSFile;
	FIL NEO6MFile;
	FIL atgmFile;
	UINT testBytes;
	uint32_t start_time_sd = 0;
	FRESULT res_org = FR_NO_FILE;
	FRESULT res_GY25 = FR_NO_FILE;
	FRESULT res_GY25_imu = FR_NO_FILE;
	FRESULT res_MICS = FR_NO_FILE;
	FRESULT res_NEO6M = FR_NO_FILE;
	FRESULT res_atgm = FR_NO_FILE;
	FRESULT res_imu = FR_NO_FILE;
	FRESULT res_log = FR_NO_FILE;
	FRESULT res_mount = FR_NO_FILE;
	char buffer[300];
	int string_num;

	int16_t mag_raw[3] = {0};
	int16_t acc_raw[3] = {0};
	int16_t gyro_raw[3] = {0};
	int16_t acc[3] = {0};
	int16_t gyro[3] = {0};
	int16_t mag[3] = {0};
	int16_t raw_roll_pitch[3] = {0};
	int16_t quatr[4] = {0};
	uint32_t gy953_start = HAL_GetTick();


	gy953spi_board_t board = { .hspi = &hspi1, .cs_port = GPIOB, .cs_pin = GPIO_PIN_1 };
	gy953spi_t ahrs;
	gy953spi_init(&ahrs, &board);
	gy953spi_reset(&ahrs);
	gy953spi_prepare(&ahrs);
	gy953spi_set_refresh_rate(&ahrs, gy953spi_refresh_rate_200hz);



	while(1)
	{


		lux = photorezistor_get_lux(pht);
		pack_imu.pack.lux = lux;

		lis3mdl_magnetic_raw_get(&lis, mag_raw);
		lsm6ds3_acceleration_raw_get(&lsm, acc_raw);
		lsm6ds3_angular_rate_raw_get(&lsm, gyro_raw);
		for(int i = 0; i < 3; i++)
		{
			pack_imu.pack.mag[i] = mag_raw[i];
			pack_imu.pack.acc[i] = acc_raw[i];
			pack_imu.pack.gyr[i] = gyro_raw[i];
			pack_org.pack.accel[i] = lsm6ds3_from_fs16g_to_mg(acc_raw[i]);
		}

		bme280_get_sensor_data(BME280_ALL, &bme_data, &bmp);
		pack_GY25.pack.temp = bme_data.temperature * 100;
		pack_GY25.pack.pres = bme_data.pressure;
		pack_org.pack.temp = bme_data.temperature * 100;
		pack_org.pack.pres = bme_data.pressure;

		gps_work();
		gps_get_coords(&cookie, &lat, &lon, &alt, &fix_);
		gps_get_time(&cookie, &gps_time_s, &gps_time_us);
		pack_NEO6M.pack.lat = lat;
		pack_NEO6M.pack.lon = lon;
		pack_NEO6M.pack.height = alt;
		pack_NEO6M.pack.fix = fix_;

		bme_data = bme_read_data(&bme);
		pack_MICS.pack.temp = bme_data.temperature * 100;
		pack_MICS.pack.pres = bme_data.pressure;
		pack_MICS.pack.hum = bme_data.humidity * 100;
		height_on_BME280 = 44330.0*(1.0 - pow((float)bme_data.pressure/pressure_on_ground, 1.0/5.255));

		if (HAL_GetTick() >= gy953_start + 25)
		{
			gy953_start = HAL_GetTick();

			gy953spi_update(&ahrs);
			gy953spi_get_acc(&ahrs, acc);
			int acc_rn = gy953spi_get_acc_range(&ahrs);
			gy953spi_get_gyro(&ahrs, gyro);
			int gyr_rn = gy953spi_get_gyro_range(&ahrs);
			gy953spi_get_mag(&ahrs, mag);
			int mag_rn = gy953spi_get_mag_range(&ahrs);
			gy953spi_get_rpy(&ahrs, raw_roll_pitch);
			gy953spi_get_quat(&ahrs, quatr);
			pack_GY25_imu.pack.acc_range = acc_rn;
			pack_GY25_imu.pack.gyr_range = gyr_rn;
			pack_GY25_imu.pack.mag_range = mag_rn;

			for(int i = 0; i < 4; i++)
			{
				pack_GY25.pack.quatr[i] = quatr[i];
			}
			for(int i = 0; i < 3; i++)
			{
				pack_GY25.pack.raw_roll_pitch[i] = raw_roll_pitch[i];
				pack_GY25_imu.pack.mag[i] = mag[i];
				pack_GY25_imu.pack.acc[i] = acc[i];
				pack_GY25_imu.pack.gyr[i] = gyro[i];
			}
		}

		if(HAL_GetTick() >= first + 750)
		{
			ds18b20_read_raw_temperature(&ds, &raw_temp, &crc_ok);
			pack_atgm.pack.DS_temp = raw_temp / 16.0 * 10.0;
			ds18b20_start_conversion(&ds);
			first = HAL_GetTick();
		}

		static uint32_t ads1115_deadline = 0;
		static const uint32_t ads1115_period = 10;
		static int ads1115_mux = 0;
		uint32_t ads1115_now = HAL_GetTick();
		if (ads1115_now >= ads1115_deadline)
		{
			ads1115_read_single(&ADS, &ads_raw[ads1115_mux]);
			ads_conv[ads1115_mux] = ads1115_convert(&ADS, ads_raw[ads1115_mux]);

			ads1115_mux += 1;
			if (ads1115_mux >= 3)
				ads1115_mux = 0;

			ads1115_write_mux(4 + ads1115_mux, &ADS);
			ads1115_req_single(&ADS);

			ads1115_deadline = ads1115_now + ads1115_period;
		}

		pack_MICS.pack.CO = ads_conv[0];
		pack_MICS.pack.NO2 = ads_conv[1];
		pack_MICS.pack.NH3 = ads_conv[2];

		switch (machine_state_now){
		case STATE_INIT:
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
			{
				if(wait_time == 0)
				{
					wait_time = HAL_GetTick();
				}
				if (HAL_GetTick() >= wait_time + 2000)
				{
					machine_state_now = STATE_BEFORE_FLIGHT;
					our_light = our_light / num_light_take;
					wait_time = 0;
					shift_reg_write_bit_16(&sr_imu, 9, 0);
					shift_reg_write_bit_16(&sr_imu, 10, 0);
					shift_reg_write_bit_16(&sr_imu, 11, 0);
					break;
				}
				num_light_take += 1;
				our_light += lux;
			}
			break;
		case STATE_BEFORE_FLIGHT:
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET){
				if(wait_time == 0)
				{
					wait_time = HAL_GetTick();
				}
				if (HAL_GetTick() >= wait_time + 5000){
					machine_state_now = STATE_ROCKET;
					shift_reg_write_bit_16(&sr_imu, 9, 0);
					shift_reg_write_bit_16(&sr_imu, 10, 0);
					shift_reg_write_bit_16(&sr_imu, 11, 1);
					break;
				}
			}
			break;
		case STATE_ROCKET:
			if(lux >= our_light * 0.8){
				machine_state_now = STATE_DESCENT_A;
				shift_reg_write_bit_16(&sr_imu, 9, 0);
				shift_reg_write_bit_16(&sr_imu, 10, 1);
				shift_reg_write_bit_16(&sr_imu, 11, 0);
			}
			break;
		case STATE_DESCENT_A:
			if(height_on_BME280 <= sbros_height){
				machine_state_now = STATE_DESCENT_B;
				shift_reg_write_bit_16(&sr_imu, 9, 0);
				shift_reg_write_bit_16(&sr_imu, 10, 1);
				shift_reg_write_bit_16(&sr_imu, 11, 1);
				wait_time = HAL_GetTick();
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			break;
		case STATE_DESCENT_B:
			if (HAL_GetTick() >= wait_time + BURST_TIME)
			{
				machine_state_now = STATE_DESCENT_C;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
				shift_reg_write_bit_16(&sr_imu, 9, 1);
				shift_reg_write_bit_16(&sr_imu, 10, 0);
				shift_reg_write_bit_16(&sr_imu, 11, 0);
			}
			break;
		case STATE_DESCENT_C:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		}


		pack_imu.pack.state = (machine_state_now << 3) | comp;

		if (res_mount != FR_OK)
		{
			shift_reg_write_bit_16(&sr_imu, 12, 0);
		}
		else{
			shift_reg_write_bit_16(&sr_imu, 12, 1);
		}

		if(fix_ == 0)
		{
			shift_reg_write_bit_16(&sr_imu, 8, 0);
		}
		else{
			shift_reg_write_bit_16(&sr_imu, 8, 1);
		}


		if (HAL_GetTick() - start_time_sd >= 50)
		{
			if (res_log != FR_OK && res_mount == FR_OK)
			{
				f_close(&logFile);
				res_log = f_open(&logFile, "log.bin", FA_WRITE | FA_OPEN_APPEND);
			}
			if (res_org != FR_OK && res_mount == FR_OK)
			{
				f_close(&orgFile);
				res_org = f_open(&orgFile, "org.csv", FA_WRITE | FA_OPEN_APPEND);
			}
			if (res_GY25 != FR_OK && res_mount == FR_OK)
			{
				f_close(&GY25File);
				res_GY25 = f_open(&GY25File, "GY25.csv", FA_WRITE | FA_OPEN_APPEND);
			}
			if (res_GY25_imu != FR_OK && res_mount == FR_OK)
			{
				f_close(&GY25imuFile);
				res_GY25_imu = f_open(&GY25imuFile, "GY25_imu.csv", FA_WRITE | FA_OPEN_APPEND);
			}
			if (res_MICS != FR_OK && res_mount == FR_OK)
			{
				f_close(&MICSFile);
				res_MICS = f_open(&MICSFile, "MICS.csv", FA_WRITE | FA_OPEN_APPEND);
			}
			if (res_NEO6M != FR_OK && res_mount == FR_OK)
			{
				f_close(&NEO6MFile);
				res_NEO6M = f_open(&NEO6MFile, "NEO6M.csv", FA_WRITE | FA_OPEN_APPEND);
			}
			if (res_atgm != FR_OK && res_mount == FR_OK)
			{
				f_close(&atgmFile);
				res_atgm = f_open(&atgmFile, "atgm.csv", FA_WRITE | FA_OPEN_APPEND);
			}
			if (res_imu != FR_OK && res_mount == FR_OK)
			{
				f_close(&imuFile);
				res_imu = f_open(&imuFile, "imu.csv", FA_WRITE | FA_OPEN_APPEND);
			}

			if (res_log != FR_OK || res_org != FR_OK || res_GY25 != FR_OK || res_GY25_imu != FR_OK || res_MICS != FR_OK || res_NEO6M != FR_OK || res_atgm != FR_OK || res_imu != FR_OK || res_mount != FR_OK)
			{
				f_mount(0, "", 1);
				extern Disk_drvTypeDef disk;
				disk.is_initialized[0] = 0;
				res_mount = f_mount(&fileSystem, "", 1);
				res_log = f_open(&logFile, "log.bin", FA_WRITE | FA_OPEN_APPEND);
				res_org = f_open(&orgFile, "org.csv", FA_WRITE | FA_OPEN_APPEND);
				f_puts("flag; id; time; temp; pres; accl0;  accl1;  accl2; crc\n", &orgFile);
				res_GY25 = f_open(&GY25File, "GY25.csv", FA_WRITE | FA_OPEN_APPEND);
				f_puts("flag; num; time; roll; pitch; yaw; pres; temp; crc\n", &GY25File);
				res_GY25_imu = f_open(&GY25imuFile, "GY25_imu.csv", FA_WRITE | FA_OPEN_APPEND);
				f_puts("flag; num; time; accl0;  accl1;  accl2; gyr0; gyr1; gyr2; mag0; mag1; mag2; acc_range; gyr_range; mag_range; crc\n", &GY25imuFile);
				res_MICS = f_open(&MICSFile, "MICS.csv", FA_WRITE | FA_OPEN_APPEND);
				f_puts("flag; num; time; CO; NO2; NH3; pres; hum; temp; crc\n", &MICSFile);
				res_NEO6M = f_open(&NEO6MFile, "NEO6M.csv", FA_WRITE | FA_OPEN_APPEND);
				f_puts("flag; num; time; lat; lon; height; fix; crc\n", &NEO6MFile);
				res_atgm = f_open(&atgmFile, "atgm.csv", FA_WRITE | FA_OPEN_APPEND);
				f_puts("flag; num; time; lat; lon; height; fix; DS_temp; crc\n", &atgmFile);
				res_imu = f_open(&imuFile, "imu.csv", FA_WRITE | FA_OPEN_APPEND);
				f_puts("flag; num; time; accl0; accl1; accl2; gyr0; gyr1; gyr2; mag0; mag1; mag2; lux; crc\n", &imuFile);
			}
			else
			{
				res_log = f_sync(&logFile);
				res_org = f_sync(&orgFile);
				res_GY25 = f_sync(&GY25File);
				res_GY25_imu = f_sync(&GY25imuFile);
				res_MICS = f_sync(&MICSFile);
				res_NEO6M = f_sync(&NEO6MFile);
				res_atgm = f_sync(&atgmFile);
				res_imu = f_sync(&imuFile);
			}
			start_time_sd = HAL_GetTick();
		}

		uint32_t start = HAL_GetTick();
		uint32_t radio_cyc_time = HAL_GetTick();
		while (HAL_GetTick() - radio_cyc_time <= 14)
		{
			switch (radio_state_now)
			{
			case RADIO_WAIT:
				if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET)
				{
					nrf24_irq_get(&nrf24, &comp);
					nrf24_irq_clear(&nrf24, comp);
					nrf24_fifo_flush_tx(&nrf24);
					radio_state_now = next_state;
				}
				if (HAL_GetTick() - radio_time > 2)
				{
					nrf24_fifo_flush_tx(&nrf24);
					radio_state_now = next_state;
				}
				break;
			case RADIO_PACKET_ORG:
				pack_org.pack.time = HAL_GetTick();
				pack_org.pack.crc = bitwiseXORChecksum(pack_org.buf, sizeof(packet_t) - 1);
				nrf24_fifo_write(&nrf24, pack_org.buf, 32, false);
				if (res_mount == FR_OK)
				{
					//binres_log
					res_log = f_write(&logFile, &pack_org.pack, sizeof(packet_t), &testBytes);

					//csv
					string_num = sd_parse_to_bytes_pack_org(buffer, &pack_org.pack);
					res_org = f_write(&orgFile, buffer, string_num, &testBytes);
				}
				radio_time = HAL_GetTick();
				radio_state_now = RADIO_WAIT;
				next_state = RADIO_PACKET_ATGM;
				break;
			case RADIO_PACKET_ATGM:
				pack_atgm.pack.num++;
				pack_atgm.pack.time = HAL_GetTick();
				pack_atgm.pack.crc = bitwiseXORChecksum(pack_atgm.buf, sizeof(packet_atgm_t) - 2);
				nrf24_fifo_write(&nrf24, pack_atgm.buf, 32, false);
				if (res_mount == FR_OK)
				{
					//binres_log
					res_log = f_write(&logFile, &pack_atgm.pack, sizeof(packet_atgm_t), &testBytes);

					//csv
					string_num = sd_parse_to_bytes_pack_atgm(buffer, &pack_atgm.pack);
					res_atgm = f_write(&atgmFile, buffer, string_num, &testBytes);
				}
				radio_time = HAL_GetTick();
				radio_state_now = RADIO_WAIT;
				next_state = RADIO_PACKET_NEO6M;
				break;
			case RADIO_PACKET_NEO6M:
				pack_NEO6M.pack.num++;
				pack_NEO6M.pack.time = HAL_GetTick();
				pack_NEO6M.pack.crc = bitwiseXORChecksum(pack_NEO6M.buf, sizeof(packet_NEO6M_t) - 2);
				nrf24_fifo_write(&nrf24, pack_NEO6M.buf, 32, false);
				if (res_mount == FR_OK)
				{
					//binres_log
					res_log = f_write(&logFile, &pack_NEO6M.pack, sizeof(packet_NEO6M_t), &testBytes);

					//csv
					string_num = sd_parse_to_bytes_pack_NEO6M(buffer, &pack_NEO6M.pack);
					res_NEO6M = f_write(&NEO6MFile, buffer, string_num, &testBytes);
				}
				radio_time = HAL_GetTick();
				radio_state_now = RADIO_WAIT;
				next_state = RADIO_PACKET_IMU;
				break;
			case RADIO_PACKET_IMU:
				pack_imu.pack.num++;
				pack_imu.pack.time = HAL_GetTick();
				pack_imu.pack.crc = bitwiseXORChecksum(pack_imu.buf, sizeof(packet_imu_t) - 2);
				nrf24_fifo_write(&nrf24, pack_imu.buf, 32, false);
				if (res_mount == FR_OK)
				{
					//binres_log
					res_log = f_write(&logFile, &pack_imu.pack, sizeof(packet_imu_t), &testBytes);

					//csv
					string_num = sd_parse_to_bytes_pack_imu(buffer, &pack_imu.pack);
					res_imu = f_write(&imuFile, buffer, string_num, &testBytes);
				}
				radio_time = HAL_GetTick();
				radio_state_now = RADIO_WAIT;
				pkt_count++;
				if (pkt_count > 10)
				{
					next_state = RADIO_PACKET_ORG;
					pkt_count = 0;
				}
				else
					next_state = RADIO_PACKET_MICS;
				break;
			case RADIO_PACKET_MICS:
				pack_MICS.pack.num++;
				pack_MICS.pack.time = HAL_GetTick();
				pack_MICS.pack.crc = bitwiseXORChecksum(pack_MICS.buf, sizeof(packet_MICS_t) - 2);
				nrf24_fifo_write(&nrf24, pack_MICS.buf, 32, false);
				if (res_mount == FR_OK)
				{
					//binres_log
					res_log = f_write(&logFile, &pack_MICS.pack, sizeof(packet_MICS_t), &testBytes);

					//csv
					string_num = sd_parse_to_bytes_pack_MICS(buffer, &pack_MICS.pack);
					res_MICS = f_write(&MICSFile, buffer, string_num, &testBytes);
				}
				radio_time = HAL_GetTick();
				radio_state_now = RADIO_WAIT;
				next_state = RADIO_PACKET_GY25;
				break;
			case RADIO_PACKET_GY25:
				pack_GY25.pack.num++;
				pack_GY25.pack.time = HAL_GetTick();
				pack_GY25.pack.crc = bitwiseXORChecksum(pack_GY25.buf, sizeof(packet_GY25_t) - 2);
				nrf24_fifo_write(&nrf24, pack_GY25.buf, 32, false);
				if (res_mount == FR_OK)
				{
					//binres_log
					res_log = f_write(&logFile, &pack_GY25.pack, sizeof(packet_GY25_t), &testBytes);

					//csv
					string_num = sd_parse_to_bytes_pack_GY25(buffer, &pack_GY25.pack);
					res_GY25 = f_write(&GY25File, buffer, string_num, &testBytes);
				}
				radio_time = HAL_GetTick();
				radio_state_now = RADIO_WAIT;
				next_state = RADIO_PACKET_GY25_IMU;
				break;
			case RADIO_PACKET_GY25_IMU:
				pack_GY25_imu.pack.num++;
				pack_GY25_imu.pack.time = HAL_GetTick();
				pack_GY25_imu.pack.crc = bitwiseXORChecksum(pack_GY25_imu.buf, sizeof(packet_GY25_imu_t) - 2);
				nrf24_fifo_write(&nrf24, pack_GY25_imu.buf, 32, false);
				if (res_mount == FR_OK)
				{
					//binres_log
					res_log = f_write(&logFile, &pack_GY25_imu.pack, sizeof(packet_GY25_imu_t), &testBytes);

					//csv
					string_num = sd_parse_to_bytes_pack_GY25_imu(buffer, &pack_GY25_imu.pack);
					res_GY25_imu = f_write(&GY25imuFile, buffer, string_num, &testBytes);
				}
				radio_time = HAL_GetTick();
				radio_state_now = RADIO_WAIT;
				next_state = RADIO_PACKET_IMU;
				break;
			}
			if (next_state == RADIO_PACKET_IMU) break;
		}

		uint32_t stop = HAL_GetTick();
		volatile uint32_t dt = stop - start;
		volatile int x = 0;
	}
}

