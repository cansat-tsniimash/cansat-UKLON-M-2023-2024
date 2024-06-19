#pragma once


struct gy953spi_board_t;
typedef struct gy953spi_board_t gy953spi_board_t;

gy953spi_board_t * gy953spi_board_ctor(void * user_arg);
void gy953spi_board_dtor(gy953spi_board_t * board);

int gy953spi_board_write(gy953spi_board_t * board, uint8_t command, uint8_t * data, int len);
int gy953spi_board_read(gy953spi_board_t * board, uint8_t command, uint8_t * data, int len);
