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

static volatile bool spi_txrx_complete = true;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        spi_tx_complete = true;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi5)
    {
        spi_txrx_complete = true;
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
    // Use static buffer to avoid stack overflow
    static uint8_t data[2048];

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
    // Use static buffer to avoid stack overflow
    static uint8_t data[64];  // SAFELOAD typically uses small data size

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
    // SPI Read用の静的バッファ（IT転送中にスコープ外にならないようにするため）
    static uint8_t tx_buf[64];
    static uint8_t rx_buf[64];

    // バッファサイズチェック
    if (length > 64 - 3)
    {
        SEGGER_RTT_printf(0, "[%X] spi read error: length too large\n", address);
        return;
    }

    // 前回の転送完了を待つ
    while (!spi_txrx_complete)
    {
        __NOP();
    }

    // ADAU1466 SPI Read: Chip Address with R/W bit = 1 (read)
    // Format: [Chip Addr | 0x01] [Addr High] [Addr Low] [Dummy bytes for read]

    // Set R/W bit to 1 for read operation (bit 0)
    tx_buf[0] = devAddress | 0x01;
    tx_buf[1] = (uint8_t) ((address >> 8) & 0x00FF);
    tx_buf[2] = (uint8_t) (address & 0x00FF);
    // Send dummy bytes (0x00) during read phase to clock out data from ADAU1466
    for (int i = 0; i < length; i++)
    {
        tx_buf[i + 3] = 0x00;
    }

    spi_txrx_complete        = false;
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(&hspi5, tx_buf, rx_buf, 1 + 2 + length);

    if (status == HAL_OK)
    {
        // 転送完了を待つ
        while (!spi_txrx_complete)
        {
            __NOP();
        }

        for (int i = 0; i < length; i++)
        {
            pData[i] = rx_buf[i + 3];
        }
#if 0
        SEGGER_RTT_printf(0, "[%04X] Read: ", address);
        for (int i = 0; i < length; i++)
        {
            SEGGER_RTT_printf(0, "%02X ", pData[i]);
        }
        SEGGER_RTT_printf(0, "\n");
#endif
    }
    else
    {
        spi_txrx_complete = true;
        SEGGER_RTT_printf(0, "[%X] spi read error: %d\n", address, status);
    }
}
