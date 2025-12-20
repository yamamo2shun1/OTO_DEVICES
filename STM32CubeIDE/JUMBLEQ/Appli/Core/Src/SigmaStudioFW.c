/*
 * SigmaStudioFW.c
 *
 *  Created on: Aug 8, 2025
 *      Author: shun
 */

#include "SigmaStudioFW.h"

#include "spi.h"

static volatile bool spi_tx_complete = true;
static uint8_t spi_tx_buf[16];

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        spi_tx_complete = true;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        spi_tx_complete = true;
    }
}

void SIGMA_WRITE_REGISTER_BLOCK(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData)
{
    // HAL_StatusTypeDef status;
    uint8_t data[1 + 2 + length];

    data[0] = devAddress;
    data[1] = (uint8_t) ((address >> 8) & 0x00FF);
    data[2] = (uint8_t) (address & 0x00FF);
    for (int i = 0; i < length; i++)
    {
        data[i + 3] = pData[i];
    }
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi5, data, 1 + 2 + length, 100);

    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "[%X] spi write error\n", address);
    }
}

void SIGMA_WRITE_REGISTER_BLOCK_IT(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData)
{
    while (!spi_tx_complete)
    {
        // wait previous transmission complete
        __NOP();
    }

    spi_tx_buf[0] = devAddress;
    spi_tx_buf[1] = (uint8_t) ((address >> 8) & 0x00FF);
    spi_tx_buf[2] = (uint8_t) (address & 0x00FF);
    for (int i = 0; i < length; i++)
    {
        spi_tx_buf[i + 3] = pData[i];
    }

    spi_tx_complete          = false;
    HAL_StatusTypeDef status = HAL_SPI_Transmit_IT(&hspi5, spi_tx_buf, 1 + 2 + length);
    if (status != HAL_OK)
    {
        spi_tx_complete = true;
        SEGGER_RTT_printf(0, "[%X] spi write error\n", address);
    }
}

void SIGMA_SAFELOAD_WRITE_DATA(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData)
{
    // HAL_StatusTypeDef status;
    uint8_t data[1 + 2 + length];

    data[0] = devAddress;
    data[1] = (uint8_t) ((dataAddress >> 8) & 0x00FF);
    data[2] = (uint8_t) (dataAddress & 0x00FF);
    for (int i = 0; i < length; i++)
    {
        data[i + 3] = pData[i];
    }
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi5, data, 1 + 2 + length, 100);

    if (status != HAL_OK)
    {
        SEGGER_RTT_printf("SAFELOAD::[%X] spi write error\n", dataAddress);
    }
}

void SIGMA_WRITE_DELAY(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData)
{
    HAL_Delay(15);
}
