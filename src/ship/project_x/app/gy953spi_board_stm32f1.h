#pragma once


#include <stm32f4xx_hal.h>

#include "gy953/gy953spi_board.h"


struct gy953spi_board_t
{
	// Требует SPI режима 3 (CPHA = 1, CPOL = 1)
	SPI_HandleTypeDef * hspi;
	GPIO_TypeDef * cs_port;
	int cs_pin;
};
