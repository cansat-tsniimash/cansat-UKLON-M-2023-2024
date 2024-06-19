#pragma once


// 1 байт для конфигурации
#define GY953_REG_CFG_A					0x01
#define GY953_REG_CFG_A_EN_MAG_MASK 	(1 << 6)
#define GY953_REG_CFG_A_EN_GYRO_MASK 	(1 << 5)
#define GY953_REG_CFG_A_EN_ACC_MASK 	(1 << 4)
#define GY953_REG_CFG_A_FREQ_MASK 		((1 << 0) | (1 << 1) | (1 << 2))
#define GY953_REG_CFG_A_ONES_MASK 		(1 << 3) // всегда должны быть 1
#define GY953_REG_CFG_A_ZEROS_MASK 		(1 << 7) // всегда должны быть 0

// 1 байт для калибровки
#define GY953_REG_CFG_B					0x02
#define GY953_REG_CFG_B_RESET_MASK		(1 < 7) // сброс до "фабричных настроек"
#define GY953_REG_CFG_B_MAG_CAL_MASK	(1 < 3) // тест и калбировка магнитометра
#define GY953_REG_CFG_B_IMU_CAL_MASK	(1 < 2) // тест и калбировка акселерометра и гироскопа
#define GY953_REG_CFG_B_ONES_MASK		((1 << 0) | (1 << 4)) // всегда 1
#define GY953_REG_CFG_B_ZEROS_MASK		((1 << 1) | (1 << 5) | (1 << 6)) // всегда 0

// везде int16 в big endian в twos-complement (как обычно)
#define GY953_REG_ACC_X		0x03
#define GY953_REG_ACC_Y		0x05
#define GY953_REG_ACC_Z		0x07

#define GY953_REG_GYRO_X	0x09
#define GY953_REG_GYRO_Y	0x0B
#define GY953_REG_GYRO_Z	0x0D

#define GY953_REG_MAG_X		0x0F
#define GY953_REG_MAG_Y		0x11
#define GY953_REG_MAG_Z		0x13

#define GY953_REG_ROLL		0x15
#define GY953_REG_PITCH		0x17
#define GY953_REG_YAW		0x19

#define GY953_REG_Q0		0x1B
#define GY953_REG_Q1		0x1D
#define GY953_REG_Q2		0x1F
#define GY953_REG_Q3		0x21

// 1 байт uint8_t
#define GY953_REG_STATUS_D	0x23
#define GY953_REG_STATUS_D_RANGE_MASK			0x03 // 0b11
#define GY953_REG_STATUS_D_ACC_RANGE_OFFSET		4
#define GY953_REG_STATUS_D_GYRO_RANGE_OFFSET	2
#define GY953_REG_STATUS_D_MAG_RANGE_OFFSET		0

// 1 байт uint8_t
#define GY953_REG_STATUS_C	0x24
#define GY953_REG_STATUS_C_NEW_MASK			(1 << 7)
#define GY953_REG_STATUS_C_QUAL_MASK		0x03 // 0b11
#define GY953_REG_STATUS_C_ACC_QUAL_OFFSET	4
#define GY953_REG_STATUS_C_GYRO_QUAL_OFFSET	2
#define GY953_REG_STATUS_C_MAG_QUAL_OFFSET	0

// 1 байт int8_t (?)
#define GY953_REG_ACC_SUM	0x25
#define GY953_REG_GYRO_SUM	0x26
#define GY953_REG_MAG_SUM	0x27
#define GY953_REG_RPY_SUM	0x28
#define GY953_REG_Q_SUM		0x29


#define GY953_MEMORY_SIZE	(GY953_REG_Q_SUM) // так то -1 ВЕЗДE, но неудобно же.
											  // Поэтому сделаем вид, что оно у нас есть


