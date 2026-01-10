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

static volatile bool spi_rx_complete = true;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        spi_tx_complete = true;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        spi_rx_complete = true;
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
        SEGGER_RTT_printf(0, "SAFELOAD::[%X] spi write error\n", dataAddress);
    }
}

void SIGMA_WRITE_DELAY(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData)
{
    HAL_Delay(15);
}

void SIGMA_READ_REGISTER(uint8_t devAddress, uint16_t address, uint16_t length, uint8_t* pData)
{
    // ADAU1466 SPI Read: Chip Address with R/W bit = 1 (read)
    // Format: [Chip Addr | 0x01] [Addr High] [Addr Low] [Dummy bytes for read]
    uint8_t tx_data[1 + 2 + length];
    uint8_t rx_data[1 + 2 + length];

    // Set R/W bit to 1 for read operation (bit 0)
    tx_data[0] = devAddress | 0x01;
    tx_data[1] = (uint8_t) ((address >> 8) & 0x00FF);
    tx_data[2] = (uint8_t) (address & 0x00FF);
    // Send dummy bytes (0x00) during read phase to clock out data from ADAU1466
    for (int i = 0; i < length; i++)
    {
        tx_data[i + 3] = 0x00;
    }

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi5, tx_data, rx_data, 1 + 2 + length, 100);

    if (status == HAL_OK)
    {
        for (int i = 0; i < length; i++)
        {
            pData[i] = rx_data[i + 3];
        }
        SEGGER_RTT_printf(0, "[%04X] Read: ", address);
        for (int i = 0; i < length; i++)
        {
            SEGGER_RTT_printf(0, "%02X ", pData[i]);
        }
        SEGGER_RTT_printf(0, "\n");
    }
    else
    {
        SEGGER_RTT_printf(0, "[%X] spi read error: %d\n", address, status);
    }
}
