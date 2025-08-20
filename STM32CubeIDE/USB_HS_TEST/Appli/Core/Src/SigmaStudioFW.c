/*
 * SigmaStudioFW.c
 *
 *  Created on: Aug 8, 2025
 *      Author: shun
 */

#include "SigmaStudioFW.h"

#include "spi.h"

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
    HAL_SPI_Transmit(&hspi5, data, 1 + 2 + length, 10000);
#if 0
    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "[%X] spi write error\n", address);
    }
#endif
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
    HAL_SPI_Transmit(&hspi5, data, 1 + 2 + length, 10000);
}

void SIGMA_WRITE_DELAY(uint8_t devAddress, uint16_t dataAddress, uint16_t length, uint8_t* pData)
{
    HAL_Delay(15);
}
