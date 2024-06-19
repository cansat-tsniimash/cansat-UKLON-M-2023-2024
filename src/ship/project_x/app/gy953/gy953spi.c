#include "gy953spi.h"

#include <string.h>
#include <errno.h>
#include <math.h>


#include "gy953spi_defs.h"


// У меня нет ответов по поводу того, что здесь происходит
// Все украдено отсюда: https://github.com/sumotoy/GY953/
// документации нет


int gy953spi_init(gy953spi_t * self, void * board_user_arg)
{
	memset(self, 0x00, sizeof(*self));

	self->_board = gy953spi_board_ctor(board_user_arg);
	if (!self->_board)
		return ENODEV;

	self->_cfg_a = GY953_REG_CFG_A_ONES_MASK; // хоть так
	return 0;
}


int gy953spi_prepare(gy953spi_t * self)
{
	int rc = gy953spi_read_reg(self, GY953_REG_CFG_A, &self->_cfg_a, 1);
	return rc;
}


int gy953spi_reset(gy953spi_t * self)
{
	uint8_t reset_bit = GY953_REG_CFG_B_RESET_MASK | GY953_REG_CFG_B_ONES_MASK;
	int rc = gy953spi_write_reg(self, GY953_REG_CFG_B, &reset_bit, 1);
	return rc;
}


int gy953spi_calibrate(gy953spi_t * self, bool imu, bool mag)
{
	if (!imu && !mag)
		return EINVAL;

	uint8_t reg_b = GY953_REG_CFG_B_ONES_MASK;

	if (imu)
		reg_b |= GY953_REG_CFG_B_IMU_CAL_MASK;

	if (mag)
		reg_b |= GY953_REG_CFG_B_MAG_CAL_MASK;

	int rc = gy953spi_write_reg(self, GY953_REG_CFG_B, &reg_b, 1);
	if (rc)
		return rc;

	return 0;
}


int gy953spi_set_enable(gy953spi_t * self, bool acc, bool gyro, bool mag)
{
	const uint8_t old_cfg_a = self->_cfg_a;
	self->_cfg_a &= ~(GY953_REG_CFG_A_EN_ACC_MASK | GY953_REG_CFG_A_EN_GYRO_MASK | GY953_REG_CFG_A_EN_MAG_MASK);

	if (acc) self->_cfg_a |= GY953_REG_CFG_A_EN_ACC_MASK;
	if (gyro) self->_cfg_a |= GY953_REG_CFG_A_EN_GYRO_MASK;
	if (mag) self->_cfg_a |= GY953_REG_CFG_A_EN_MAG_MASK;

	int rc;
	if (acc && gyro && mag)
	{
		// В таком случае должен стоять какой-то update rate
		const int freq = self->_cfg_a & GY953_REG_CFG_A_FREQ_MASK;
		switch (freq)
		{
		case gy953spi_refresh_rate_50hz:
		case gy953spi_refresh_rate_100hz:
		case gy953spi_refresh_rate_200hz:
			// что-то уже стоит
			break;

		default:
			// Нужно что-то поставить
			rc = gy953spi_set_refresh_rate(self, gy953spi_refresh_rate_100hz);
			return rc;
		}
	}

	if (self->_cfg_a == old_cfg_a)
		return 0;

	rc = gy953spi_write_reg(self, GY953_REG_CFG_A, &self->_cfg_a, 1);
	return rc;
}


bool gy953spi_get_enable_acc(gy953spi_t * self) { return self->_cfg_a & GY953_REG_CFG_A_EN_ACC_MASK; }
bool gy953spi_get_enable_gyro(gy953spi_t * self) { return self->_cfg_a & GY953_REG_CFG_A_EN_GYRO_MASK; }
bool gy953spi_get_enable_mag(gy953spi_t * self) { return self->_cfg_a & GY953_REG_CFG_A_EN_MAG_MASK; }


int gy953spi_set_refresh_rate(gy953spi_t * self, gy953spi_refresh_rate_t rate)
{
	const uint8_t old_cfg_a = self->_cfg_a;

	// В любом случае нужно включать все три датчика
	// Иначе оно как AHRS работать не станет
	self->_cfg_a |=
			GY953_REG_CFG_A_EN_MAG_MASK
			| GY953_REG_CFG_A_EN_GYRO_MASK
			| GY953_REG_CFG_A_EN_ACC_MASK
	;

	self->_cfg_a &= ~(GY953_REG_CFG_A_FREQ_MASK);
	self->_cfg_a |= (uint8_t)rate & GY953_REG_CFG_A_FREQ_MASK;
	if (self->_cfg_a == old_cfg_a)
		return 0;

	int rc = gy953spi_write_reg(self, GY953_REG_CFG_A, &self->_cfg_a, 1);
	return rc;
}


int gy953spi_get_refresh_rate(gy953spi_t * self)
{
	return self->_cfg_a & GY953_REG_CFG_A_FREQ_MASK;
}


int gy953spi_update(gy953spi_t * self)
{
	// Регистры тут нумеруются с 1. А индекс у них от нуля
	// Оч не удобно - добавим этот ноль самостоятельно
	return gy953spi_read_reg(
			self, GY953_REG_CFG_A,
			self->_memory + 1, sizeof(self->_memory) - 1);
}


int gy953spi_get_acc_accuracy(gy953spi_t * self)
{
	const int rv = (
			self->_memory[GY953_REG_STATUS_C] >> GY953_REG_STATUS_C_ACC_QUAL_OFFSET
	) & GY953_REG_STATUS_C_QUAL_MASK;
	return rv;

}


int gy953spi_get_gyro_accuracy(gy953spi_t * self)
{
	const int rv = (
			self->_memory[GY953_REG_STATUS_C] >> GY953_REG_STATUS_C_GYRO_QUAL_OFFSET
	) & GY953_REG_STATUS_C_QUAL_MASK;
	return rv;
}


int gy953spi_get_mag_accuracy(gy953spi_t * self)
{
	const int rv = (
			self->_memory[GY953_REG_STATUS_C] >> GY953_REG_STATUS_C_MAG_QUAL_OFFSET
	) & GY953_REG_STATUS_C_QUAL_MASK;
	return rv;
}


gy953spi_acc_range_t gy953spi_get_acc_range(gy953spi_t * self)
{
	const int rv = (
			self->_memory[GY953_REG_STATUS_D] >> GY953_REG_STATUS_D_ACC_RANGE_OFFSET
	) & GY953_REG_STATUS_D_RANGE_MASK;
	return rv;

}


gy953spi_gyro_range_t gy953spi_get_gyro_range(gy953spi_t * self)
{
	const int rv = (
			self->_memory[GY953_REG_STATUS_D] >> GY953_REG_STATUS_D_GYRO_RANGE_OFFSET
	) & GY953_REG_STATUS_D_RANGE_MASK;
	return rv;
}


gy953spi_mag_range_t gy953spi_get_mag_range(gy953spi_t * self)
{
	const int rv = (
			self->_memory[GY953_REG_STATUS_D] >> GY953_REG_STATUS_D_MAG_RANGE_OFFSET
	) & GY953_REG_STATUS_D_RANGE_MASK;
	return rv;
}


int gy953spi_get_acc(gy953spi_t * self, int16_t raw_xyz[3])
{
	uint16_t values[3];

	values[0] = ((uint16_t)self->_memory[GY953_REG_ACC_X] << 8) | self->_memory[GY953_REG_ACC_X + 1];
	values[1] = ((uint16_t)self->_memory[GY953_REG_ACC_Y] << 8) | self->_memory[GY953_REG_ACC_Y + 1];
	values[2] = ((uint16_t)self->_memory[GY953_REG_ACC_Z] << 8) | self->_memory[GY953_REG_ACC_Z + 1];
	memcpy(raw_xyz, values, sizeof(values));

	return 0;
}


int gy953spi_get_acc_g(gy953spi_t * self, gy953_float_t xyz[3])
{
	int16_t raws[3];
	int rc = gy953spi_get_acc(self, raws);
	if (rc)
		return rc;

	const gy953spi_acc_range_t range = gy953spi_get_acc_range(self);
	rc = gy953spi_acc_raw_to_g(xyz, raws, range);
	if (rc)
		return rc;

	return 0;
}

int gy953spi_acc_raw_to_g(
		gy953_float_t values[3], const int16_t raw_values[3],
		gy953spi_acc_range_t range
)
{
	gy953_float_t coeff;
	switch (range)
	{
	case gy953spi_acc_range_2g: coeff = 2.0; break;
	case gy953spi_acc_range_4g: coeff = 4.0; break;
	case gy953spi_acc_range_8g: coeff = 8.0; break;
	case gy953spi_acc_range_16g: coeff = 16.0; break;
	default:
		return EINVAL;
	}

	values[0] = (gy953_float_t)raw_values[0] * coeff / (gy953_float_t)INT16_MAX;
	values[1] = (gy953_float_t)raw_values[1] * coeff / (gy953_float_t)INT16_MAX;
	values[2] = (gy953_float_t)raw_values[2] * coeff / (gy953_float_t)INT16_MAX;

	return 0;
}


int gy953spi_get_gyro(gy953spi_t * self, int16_t raw_xyz[3])
{
	uint16_t values[3];

	values[0] = ((uint16_t)self->_memory[GY953_REG_GYRO_X] << 8) | self->_memory[GY953_REG_GYRO_X + 1];
	values[1] = ((uint16_t)self->_memory[GY953_REG_GYRO_Y] << 8) | self->_memory[GY953_REG_GYRO_Y + 1];
	values[2] = ((uint16_t)self->_memory[GY953_REG_GYRO_Z] << 8) | self->_memory[GY953_REG_GYRO_Z + 1];
	memcpy(raw_xyz, values, sizeof(values));

	return 0;
}


int gy953spi_get_gyro_dps(gy953spi_t * self, gy953_float_t xyz[3])
{
	int16_t raws[3];
	int rc = gy953spi_get_gyro(self, raws);
	if (rc) return rc;

	gy953spi_gyro_range_t range = gy953spi_get_gyro_range(self);
	rc = gy953spi_gyro_raw_to_dps(xyz, raws, range);
	if (rc)
		return rc;

	return 0;
}


int gy953spi_gyro_raw_to_dps(
		gy953_float_t values[3], const int16_t raw_values[3],
		gy953spi_gyro_range_t range
)
{
	gy953_float_t coeff;
	switch (range)
	{
	case gy953spi_gyro_range_250dps: coeff = 250.0; break;
	case gy953spi_gyro_range_500dps: coeff = 500.0; break;
	case gy953spi_gyro_range_1000dps: coeff = 1000.0; break;
	case gy953spi_gyro_range_2000pds: coeff = 2000.0; break;

	default:
		return EINVAL;
	};

	values[0] = (gy953_float_t)raw_values[0] * coeff / (gy953_float_t)INT16_MAX;
	values[1] = (gy953_float_t)raw_values[1] * coeff / (gy953_float_t)INT16_MAX;
	values[2] = (gy953_float_t)raw_values[2] * coeff / (gy953_float_t)INT16_MAX;

	return 0;
}


int gy953spi_get_mag(gy953spi_t * self, int16_t values_[3])
{
	uint16_t values[3];

	values[0] = ((uint16_t)self->_memory[GY953_REG_MAG_X] << 8) | self->_memory[GY953_REG_MAG_X + 1];
	values[1] = ((uint16_t)self->_memory[GY953_REG_MAG_Y] << 8) | self->_memory[GY953_REG_MAG_Y + 1];
	values[2] = ((uint16_t)self->_memory[GY953_REG_MAG_Z] << 8) | self->_memory[GY953_REG_MAG_Z + 1];
	memcpy(values_, values, sizeof(values));

	return 0;
}


int gy953spi_get_mag_norm(gy953spi_t * self, gy953_float_t xyz[3])
{
	int16_t raw_xyz[3];
	int rc = gy953spi_get_mag(self, raw_xyz);
	if (rc)
		return rc;

	const gy953_float_t sum = \
		  (gy953_float_t )raw_xyz[0] * raw_xyz[0]
		+ (gy953_float_t )raw_xyz[1] * raw_xyz[1]
		+ (gy953_float_t )raw_xyz[2] * raw_xyz[2]
	;

	const gy953_float_t coeff = 1.0 / sqrt(sum);
	xyz[0] = raw_xyz[0] * coeff;
	xyz[1] = raw_xyz[1] * coeff;
	xyz[2] = raw_xyz[2] * coeff;
	return 0;
}


int gy953spi_get_rpy(gy953spi_t * self, int16_t raw_roll_pitch_yaw_raw[3])
{
	uint16_t values[3];

	values[0] = ((uint16_t)self->_memory[GY953_REG_ROLL] << 8) | self->_memory[GY953_REG_ROLL + 1];
	values[1] = ((uint16_t)self->_memory[GY953_REG_PITCH] << 8) | self->_memory[GY953_REG_PITCH + 1];
	values[2] = ((uint16_t)self->_memory[GY953_REG_YAW] << 8) | self->_memory[GY953_REG_YAW + 1];
	memcpy(raw_roll_pitch_yaw_raw, values, sizeof(values));

	return 0;
}


#define GY953_RPY_RANGE 100

int gy953spi_get_rpy_deg(gy953spi_t * self, gy953_float_t values[3])
{
	int16_t raws[3];

	int rc = gy953spi_get_rpy(self, raws);
	if (rc) return rc;

	values[0] = (gy953_float_t)raws[0] / (gy953_float_t)GY953_RPY_RANGE;
	values[1] = (gy953_float_t)raws[1] / (gy953_float_t)GY953_RPY_RANGE;
	values[2] = (gy953_float_t)raws[2] / (gy953_float_t)GY953_RPY_RANGE;

	return 0;
}


int gy953spi_get_quat(gy953spi_t * self, int16_t raw_values[4])
{
	uint16_t values[4];

	values[0] = ((uint16_t)self->_memory[GY953_REG_Q0] << 8) | self->_memory[GY953_REG_Q0+1];
	values[1] = ((uint16_t)self->_memory[GY953_REG_Q1] << 8) | self->_memory[GY953_REG_Q1+1];
	values[2] = ((uint16_t)self->_memory[GY953_REG_Q2] << 8) | self->_memory[GY953_REG_Q2+1];
	values[3] = ((uint16_t)self->_memory[GY953_REG_Q3] << 8) | self->_memory[GY953_REG_Q3+1];
	memcpy(raw_values, values, sizeof(values));

	return 0;
}


#define GY953_QUAT_RANGE 10000

int gy953spi_get_quat_norm(gy953spi_t * self, gy953_float_t values[4])
{
	int16_t raws[4];
	int rc = gy953spi_get_quat(self, raws);
	if (rc) return rc;

	values[0] = (gy953_float_t)raws[0] / (gy953_float_t)GY953_QUAT_RANGE;
	values[1] = (gy953_float_t)raws[1] / (gy953_float_t)GY953_QUAT_RANGE;
	values[2] = (gy953_float_t)raws[2] / (gy953_float_t)GY953_QUAT_RANGE;
	values[3] = (gy953_float_t)raws[3] / (gy953_float_t)GY953_QUAT_RANGE;

	return 0;
}


bool gy953spi_get_updated(gy953spi_t * self)
{
	return (self->_memory[GY953_REG_STATUS_C] & GY953_REG_STATUS_C_NEW_MASK) != 0;
}


int gy953spi_write_reg(gy953spi_t * self, uint8_t addr, uint8_t * data, int len)
{
	if (len <= 0)
		return EINVAL;

	const uint8_t command = addr | 0x40; // Это догадка, но кажется верная
	int rc = gy953spi_board_write(self->_board, command, data, len);
	return rc;
}


int gy953spi_read_reg(gy953spi_t * self, uint8_t addr, uint8_t * data, int len)
{
	if (len <= 0)
		return EINVAL;

	const uint8_t command = addr | 0xC0; // Это догадка, но кажется верная
	int rc = gy953spi_board_read(self->_board, command, data, len);
	if (rc)
		return rc;

	// Маленький костыль
	if (addr == GY953_REG_CFG_A && len > 0)
		self->_cfg_a = data[0];

	return 0;
}
