/*
 * packet.h
 *
 *  Created on: Feb 24, 2024
 *      Author: Роберт
 */

#ifndef PACKET_H_
#define PACKET_H_

#pragma pack(push,1)
typedef struct
{
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	int16_t acc[3]; /* Данные акселерометра */
	int16_t gyr[3];/* Данные  гироскопа*/
	int16_t mag[3];/*Данные магнитометра*/
	uint8_t state;
	float lux;
	uint16_t crc;
} packet_imu_t;

typedef union
{
	packet_imu_t pack;
	uint8_t buf[32];
}packet_imu_union_t;

typedef struct
{
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	int16_t acc[3]; /* Данные акселерометра */
	int16_t gyr[3];/* Данные  гироскопа*/
	int16_t mag[3];/*Данные магнитометра*/
	uint16_t crc;
} packet_GY25_imu_t;

typedef union
{
	packet_imu_t pack;
	uint8_t buf[32];
}packet_GY25_imu_union_t;

typedef struct
{
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	float lat;/*широта*/
	float lon;/*долгота*/
	float height;/*высота*/
	uint8_t fix;
	int16_t DS_temp;
	uint16_t crc;
} packet_atgm_t;

typedef union
{
	packet_atgm_t pack;
	uint8_t buf[32];
}packet_atgm_union_t;


typedef struct
{
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	float lat;/*широта*/
	float lon;/*долгота*/
	float height;/*высота*/
	uint8_t fix;
	uint16_t crc;
} packet_NEO6M_t;

typedef union
{
	packet_NEO6M_t pack;
	uint8_t buf[32];
}packet_NEO6M_union_t;


typedef struct
{
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
} packet_MICS_t;

typedef union
{
	packet_MICS_t pack;
	uint8_t buf[32];
} packet_MICS_union_t;


typedef struct
{
	uint8_t flag;
	uint16_t num;
	uint32_t time;
	int16_t raw_roll_pitch[3];
	int16_t quatr[4];
	uint32_t pres; /*давление*/
	int16_t temp; /*температура*/
	uint16_t crc;
} packet_GY25_t;

typedef union
{
	packet_GY25_t pack;
	uint8_t buf[32];
} packet_GY25_union_t;

typedef struct
{
	uint16_t flag;
	uint16_t id;
	uint32_t time;
	int16_t temp; /*температура*/
	uint32_t pres; /*давление*/
	int16_t accel[3];
	uint8_t crc;
} packet_t;

typedef union
{
	packet_t pack;
	uint8_t buf[32];
} packet_union_t;

#pragma pack(pop)

#endif /* PACKET_H_ */
