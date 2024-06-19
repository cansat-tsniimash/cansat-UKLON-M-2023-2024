#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


#include "gy953spi_defs.h"
#include "gy953spi_board.h"


typedef float gy953_float_t;


typedef enum gy953spi_refresh_rate_t
{
	gy953spi_refresh_rate_50hz = 3,
	gy953spi_refresh_rate_100hz = 4,
	gy953spi_refresh_rate_200hz = 5
} gy953spi_refresh_rate_t;


typedef enum gy953spi_acc_range_t
{
	gy953spi_acc_range_2g = 0,
	gy953spi_acc_range_4g = 1,
	gy953spi_acc_range_8g = 2,
	gy953spi_acc_range_16g = 3,
} gy953spi_acc_range_t;


typedef enum gy953spi_gyro_range_t
{
	gy953spi_gyro_range_250dps = 0,
	gy953spi_gyro_range_500dps = 1,
	gy953spi_gyro_range_1000dps = 2,
	gy953spi_gyro_range_2000pds = 3,
} gy953spi_gyro_range_t;


typedef enum gy953spi_mag_range_t
{
	gy953spi_gyro_range_14bit = 0, // 0:14bit(0.6ut, 4915),
	gy953spi_gyro_range_16bit = 1, // 1:16bit(0.15ut, 4915), чтобы бы это не значило
} gy953spi_mag_range_t;


typedef enum gy953spi_calib_quality_t
{
	gy953spi_calib_quality_best = 0,
	// 1 и 2 - тоже допустимые значения
	gy953spi_calib_quality_worst = 3,
} gy953spi_calib_quality_t;


typedef struct gy953spi_t
{
	gy953spi_board_t * _board;
	uint8_t _cfg_a;
	uint8_t _memory[GY953_MEMORY_SIZE];
} gy953spi_t;


// Иницилизация структуры драйрвера
int gy953spi_init(gy953spi_t * self, void * board_user_arg);

// Приведение состояния драйвера в соответствии с состянием прибора
int gy953spi_prepare(gy953spi_t * self);

// Сброс датчика до "фабричных настроек"
// Неизвестно сколько длится этот процесс
// После этого следует делать prepare еще раз
int gy953spi_reset(gy953spi_t * self);

int gy953spi_calibrate(gy953spi_t * self, bool imu, bool mag);

// Включает/выключает отдельные сенсоры
// Если выключить хотя бы один, то оно перстает работать как AHRS
// и refresh rate падает в 0
// И похоже что хотябы один какой-то будет работать всегда
// А еще похоже что не все комбинации доступны
int gy953spi_set_enable(gy953spi_t * self, bool acc, bool gyro, bool mag);
bool gy953spi_get_enable_acc(gy953spi_t * self);
bool gy953spi_get_enable_gyro(gy953spi_t * self);
bool gy953spi_get_enable_mag(gy953spi_t * self);

// Устанавливает частоту обновления данных
int gy953spi_set_refresh_rate(gy953spi_t * self, gy953spi_refresh_rate_t rate);
// Имеет полное право вернуть 0, если какие-то сенсоры отключены
// И оно не рабоает как AHRS
int gy953spi_get_refresh_rate(gy953spi_t * self);

int gy953spi_update(gy953spi_t * self);

// точность согласно gy953spi_calib_quality_t
int gy953spi_get_acc_accuracy(gy953spi_t * self);
int gy953spi_get_gyro_accuracy(gy953spi_t * self);
int gy953spi_get_mag_accuracy(gy953spi_t * self);

// значения которые всегда тут получаются (возможно можно настроить другие но хз как)
gy953spi_acc_range_t gy953spi_get_acc_range(gy953spi_t * self);
gy953spi_gyro_range_t gy953spi_get_gyro_range(gy953spi_t * self);
gy953spi_mag_range_t gy953spi_get_mag_range(gy953spi_t * self);

// По диапазону значений смотрите в gy953spi_get_range
int gy953spi_get_acc(gy953spi_t * self, int16_t raw_xyz[3]);
int gy953spi_get_acc_g(gy953spi_t * self, gy953_float_t xyz[3]);
int gy953spi_acc_raw_to_g(
		gy953_float_t values[3], const int16_t raw_values[3],
		gy953spi_acc_range_t range
);

int gy953spi_get_gyro(gy953spi_t * self, int16_t raw_xyz[3]);
int gy953spi_get_gyro_dps(gy953spi_t * self, gy953_float_t xyz[3]);
int gy953spi_gyro_raw_to_dps(
		gy953_float_t values[3], const int16_t raw_values[3],
		gy953spi_gyro_range_t range
);

int gy953spi_get_mag(gy953spi_t * self, int16_t raw_xyz[3]);
// Нормированный вектор
int gy953spi_get_mag_norm(gy953spi_t * self, gy953_float_t xyz[3]);


// в degrees * 10**2
int gy953spi_get_rpy(gy953spi_t * self, int16_t raw_roll_pitch_yaw_raw[3]);
int gy953spi_get_rpy_deg(gy953spi_t * self, gy953_float_t roll_pitch_yaw[3]);
int gy953spi_rpy_raw_to_deg(
		gy953_float_t roll_pitch_yaw[3], const int16_t raw_values[3]
);

// в 1 * 10**4
int gy953spi_get_quat(gy953spi_t * self, int16_t values[4]);
int gy953spi_get_quat_norm(gy953spi_t * self, gy953_float_t values[4]);

bool gy953spi_get_updated(gy953spi_t * self);

int gy953spi_write_reg(gy953spi_t * self, uint8_t addr, uint8_t * data, int len);
int gy953spi_read_reg(gy953spi_t * self, uint8_t addr, uint8_t * data, int len);

