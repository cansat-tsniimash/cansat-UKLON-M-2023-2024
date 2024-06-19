/*
 * csv.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Роберт
 */
#include <stdio.h>
#include <string.h>
#include "packet.h"
uint16_t sd_parse_to_bytes_pack_org(char *buffer, packet_t *pack_org)
{
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%d;%ld;%d;%d;%d;%d;\n",
			pack_org->flag, pack_org->id, pack_org->time, pack_org->temp, pack_org->pres, pack_org->accel[0], pack_org->accel[1], pack_org->accel[2], pack_org->crc);
	return num_written;
}


uint16_t sd_parse_to_bytes_pack_GY25(char *buffer, packet_GY25_t *packet_GY25)
{
	/*memset(buffer, 0, 300);

	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%f;%f;%f;%ld;%d;%d\n",
			packet_GY25->flag, packet_GY25->num, packet_GY25->time, packet_GY25->roll, packet_GY25->yaw, packet_GY25->pitch, packet_GY25->pres, packet_GY25->temp, packet_GY25->crc);
	return num_written;*/
}


uint16_t sd_parse_to_bytes_pack_MICS(char *buffer, packet_MICS_t *packet_MICS)
{
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%f;%f;%f;%ld;%d;%d;%d\n",
			packet_MICS->flag, packet_MICS->num, packet_MICS->time, packet_MICS->CO, packet_MICS->NO2, packet_MICS->NH3, packet_MICS->pres, packet_MICS->hum, packet_MICS->temp, packet_MICS->crc);
	return num_written;
}


uint16_t sd_parse_to_bytes_pack_NEO6M(char *buffer, packet_NEO6M_t *packet_NEO6M)
{
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%f;%f;%f;%d;%d\n",
			packet_NEO6M->flag, packet_NEO6M->num, packet_NEO6M->time, packet_NEO6M->lat, packet_NEO6M->lon, packet_NEO6M->height, packet_NEO6M->fix, packet_NEO6M->crc);
	return num_written;
}

uint16_t sd_parse_to_bytes_pack_atgm(char *buffer, packet_atgm_t *packet_atgm)
{
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%f;%f;%f;%d;%d;%d\n",
			packet_atgm->flag, packet_atgm->num, packet_atgm->time, packet_atgm->lat, packet_atgm->lon, packet_atgm->height, packet_atgm->fix, packet_atgm->DS_temp, packet_atgm->crc);
	return num_written;
}

uint16_t sd_parse_to_bytes_pack_imu(char *buffer, packet_imu_t *packet_imu)
{
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%ld;%d;%d;%d;%d;%d;%d;%d;%d;%d;%f;%d\n",
			packet_imu->flag, packet_imu->num, packet_imu->time, packet_imu->acc[0], packet_imu->acc[1], packet_imu->acc[2], packet_imu->gyr[0], packet_imu->gyr[1], packet_imu->gyr[2], packet_imu->mag[0], packet_imu->mag[1], packet_imu->mag[2], packet_imu->lux, packet_imu->crc);
	return num_written;
}
