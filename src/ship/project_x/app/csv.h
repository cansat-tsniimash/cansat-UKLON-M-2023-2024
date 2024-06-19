/*
 * csv.h
 *
 *  Created on: Feb 28, 2024
 *      Author: Роберт
 */

#ifndef CSV_H_
#define CSV_H_

uint16_t sd_parse_to_bytes_pack_org(char *buffer, packet_t *pack_org);

uint16_t sd_parse_to_bytes_pack_GY25(char *buffer, packet_GY25_t *packet_GY25);

uint16_t sd_parse_to_bytes_pack_GY25_imu(char *buffer, packet_GY25_imu_t *packet_GY25_imu);

uint16_t sd_parse_to_bytes_pack_MICS(char *buffer, packet_MICS_t *packet_MICS);


uint16_t sd_parse_to_bytes_pack_NEO6M(char *buffer, packet_NEO6M_t *packet_NEO6M);


uint16_t sd_parse_to_bytes_pack_atgm(char *buffer, packet_atgm_t *packet_atgm);

uint16_t sd_parse_to_bytes_pack_imu(char *buffer, packet_imu_t *packet_imu);


#endif /* CSV_H_ */
