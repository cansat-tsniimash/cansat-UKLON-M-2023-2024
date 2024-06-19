#include "gy953spi_board_stm32f1.h"


static void cs_down(gy953spi_board_t * board)
{
	HAL_GPIO_WritePin(board->cs_port, board->cs_pin, GPIO_PIN_RESET);
}


static void cs_up(gy953spi_board_t * board)
{
	HAL_GPIO_WritePin(board->cs_port, board->cs_pin, GPIO_PIN_SET);
}



gy953spi_board_t * gy953spi_board_ctor(void * user_arg)
{
	// Изящно замыкаем
	return (gy953spi_board_t *)user_arg;
}


void gy953spi_board_dtor(gy953spi_board_t * board)
{
	return; // ничего здесь не делаем
}


int gy953spi_board_write(gy953spi_board_t * board, uint8_t command, uint8_t * data, int len)
{
	cs_down(board);
	HAL_SPI_Transmit(board->hspi, &command, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(board->hspi, data, len, HAL_MAX_DELAY);
	cs_up(board);

	return 0;
}


int gy953spi_board_read(gy953spi_board_t * board, uint8_t command, uint8_t * data, int len)
{
	cs_down(board);
	HAL_SPI_Transmit(board->hspi, &command, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(board->hspi, data, len, HAL_MAX_DELAY);
	cs_up(board);

	return 0;
}
