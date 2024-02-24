/*
 * packet.h
 *
 *  Created on: Feb 24, 2024
 *      Author: Роберт
 */

#ifndef PACKET_H_
#define PACKET_H_

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

#endif /* PACKET_H_ */
