/*
 * app_main.c
 *
 *  Created on: 1 дек. 2023 г.
 *      Author: Install
 */
#include "Shift_Register/shift_reg.h"

extern SPI_HandleTypeDef hspi2;

void app_main(){
	shift_reg_t sr_imu;

	sr_imu.bus = &hspi2;
	sr_imu.latch_port = GPIOC;
	sr_imu.latch_pin = GPIO_PIN_1;
	sr_imu.oe_port = GPIOB;
	sr_imu.oe_pin = GPIO_PIN_14;
	shift_reg_init(&sr_imu);
	shift_reg_write_16(&sr_imu, 0xFFFF);
	while(1){
		shift_reg_write_bit_16(&sr_imu, 9, 0);
		HAL_Delay(100);
		shift_reg_write_bit_16(&sr_imu, 10, 0);
		shift_reg_write_bit_16(&sr_imu, 9, 1);
		HAL_Delay(100);
		shift_reg_write_bit_16(&sr_imu, 10, 1);
	}
}

