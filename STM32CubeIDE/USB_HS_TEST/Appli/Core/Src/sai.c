/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : SAI.c
 * Description        : This file provides code for the configuration
 *                      of the SAI instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sai.h"

/* USER CODE BEGIN 0 */
extern uint32_t sai_buf[];     // RX バッファ（main.c）
extern uint32_t sai_tx_buf[];  // TX バッファ（main.c）
/* USER CODE END 0 */

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockA2;
DMA_NodeTypeDef __attribute__((aligned(32))) Node_GPDMA1_Channel3;
DMA_QListTypeDef __attribute__((aligned(32))) List_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel3;
DMA_NodeTypeDef __attribute__((aligned(32))) Node_GPDMA1_Channel2;
DMA_QListTypeDef __attribute__((aligned(32))) List_GPDMA1_Channel2;
DMA_HandleTypeDef handle_GPDMA1_Channel2;

/* SAI1 init function */
void MX_SAI1_Init(void)
{

    /* USER CODE BEGIN SAI1_Init 0 */

    /* USER CODE END SAI1_Init 0 */

    /* USER CODE BEGIN SAI1_Init 1 */

    /* USER CODE END SAI1_Init 1 */

    hsai_BlockA1.Instance                    = SAI1_Block_A;
    hsai_BlockA1.Init.Protocol               = SAI_FREE_PROTOCOL;
    hsai_BlockA1.Init.AudioMode              = SAI_MODESLAVE_RX;
    hsai_BlockA1.Init.DataSize               = SAI_DATASIZE_32;
    hsai_BlockA1.Init.FirstBit               = SAI_FIRSTBIT_MSB;
    hsai_BlockA1.Init.ClockStrobing          = SAI_CLOCKSTROBING_FALLINGEDGE;
    hsai_BlockA1.Init.Synchro                = SAI_ASYNCHRONOUS;
    hsai_BlockA1.Init.OutputDrive            = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockA1.Init.FIFOThreshold          = SAI_FIFOTHRESHOLD_HF;
    hsai_BlockA1.Init.SynchroExt             = SAI_SYNCEXT_DISABLE;
    hsai_BlockA1.Init.MonoStereoMode         = SAI_STEREOMODE;
    hsai_BlockA1.Init.CompandingMode         = SAI_NOCOMPANDING;
    hsai_BlockA1.Init.TriState               = SAI_OUTPUT_NOTRELEASED;
    hsai_BlockA1.Init.PdmInit.Activation     = DISABLE;
    hsai_BlockA1.Init.PdmInit.MicPairsNbr    = 0;
    hsai_BlockA1.Init.PdmInit.ClockEnable    = SAI_PDM_CLOCK1_ENABLE;
    hsai_BlockA1.FrameInit.FrameLength       = 64;
    hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
    hsai_BlockA1.FrameInit.FSDefinition      = SAI_FS_STARTFRAME;
    hsai_BlockA1.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
    hsai_BlockA1.FrameInit.FSOffset          = SAI_FS_FIRSTBIT;
    hsai_BlockA1.SlotInit.FirstBitOffset     = 0;
    hsai_BlockA1.SlotInit.SlotSize           = SAI_SLOTSIZE_DATASIZE;
    hsai_BlockA1.SlotInit.SlotNumber         = 2;
    hsai_BlockA1.SlotInit.SlotActive         = 0x00000003;  // 0x0000FFFF;
    if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SAI1_Init 2 */

    /* USER CODE END SAI1_Init 2 */
}
/* SAI2 init function */
void MX_SAI2_Init(void)
{

    /* USER CODE BEGIN SAI2_Init 0 */

    /* USER CODE END SAI2_Init 0 */

    /* USER CODE BEGIN SAI2_Init 1 */

    /* USER CODE END SAI2_Init 1 */

    hsai_BlockA2.Instance                    = SAI2_Block_A;
    hsai_BlockA2.Init.Protocol               = SAI_FREE_PROTOCOL;
    hsai_BlockA2.Init.AudioMode              = SAI_MODESLAVE_TX;
    hsai_BlockA2.Init.DataSize               = SAI_DATASIZE_32;
    hsai_BlockA2.Init.FirstBit               = SAI_FIRSTBIT_MSB;
    hsai_BlockA2.Init.ClockStrobing          = SAI_CLOCKSTROBING_FALLINGEDGE;
    hsai_BlockA2.Init.Synchro                = SAI_ASYNCHRONOUS;
    hsai_BlockA2.Init.OutputDrive            = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockA2.Init.FIFOThreshold          = SAI_FIFOTHRESHOLD_HF;
    hsai_BlockA2.Init.SynchroExt             = SAI_SYNCEXT_DISABLE;
    hsai_BlockA2.Init.MonoStereoMode         = SAI_STEREOMODE;
    hsai_BlockA2.Init.CompandingMode         = SAI_NOCOMPANDING;
    hsai_BlockA2.Init.TriState               = SAI_OUTPUT_NOTRELEASED;
    hsai_BlockA2.Init.PdmInit.Activation     = DISABLE;
    hsai_BlockA2.Init.PdmInit.MicPairsNbr    = 0;
    hsai_BlockA2.Init.PdmInit.ClockEnable    = SAI_PDM_CLOCK1_ENABLE;
    hsai_BlockA2.FrameInit.FrameLength       = 64;
    hsai_BlockA2.FrameInit.ActiveFrameLength = 32;
    hsai_BlockA2.FrameInit.FSDefinition      = SAI_FS_STARTFRAME;
    hsai_BlockA2.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
    hsai_BlockA2.FrameInit.FSOffset          = SAI_FS_FIRSTBIT;
    hsai_BlockA2.SlotInit.FirstBitOffset     = 0;
    hsai_BlockA2.SlotInit.SlotSize           = SAI_SLOTSIZE_DATASIZE;
    hsai_BlockA2.SlotInit.SlotNumber         = 2;
    hsai_BlockA2.SlotInit.SlotActive         = 0x00000003;  // 0x0000FFFF;
    if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SAI2_Init 2 */

    /* USER CODE END SAI2_Init 2 */
}
static uint32_t SAI1_client = 0;
static uint32_t SAI2_client = 0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* saiHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    DMA_NodeConfTypeDef NodeConfig;
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    /* SAI1 */
    if (saiHandle->Instance == SAI1_Block_A)
    {
        /* SAI1 clock enable */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
        PeriphClkInit.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_PIN;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        if (SAI1_client == 0)
        {
            __HAL_RCC_SAI1_CLK_ENABLE();

            /* Peripheral interrupt init*/
            HAL_NVIC_SetPriority(SAI1_A_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SAI1_A_IRQn);
        }
        SAI1_client++;

        /**SAI1_A_Block_A GPIO Configuration
        PE4     ------> SAI1_FS_A
        PE5     ------> SAI1_SCK_A
        PE6     ------> SAI1_SD_A
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* Peripheral DMA init*/

        NodeConfig.NodeType                         = DMA_GPDMA_LINEAR_NODE;
        NodeConfig.Init.Request                     = GPDMA1_REQUEST_SAI1_A;
        NodeConfig.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
        NodeConfig.Init.Direction                   = DMA_PERIPH_TO_MEMORY;
        NodeConfig.Init.SrcInc                      = DMA_SINC_FIXED;
        NodeConfig.Init.DestInc                     = DMA_DINC_INCREMENTED;
        NodeConfig.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_WORD;
        NodeConfig.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_WORD;
        NodeConfig.Init.SrcBurstLength              = 1;
        NodeConfig.Init.DestBurstLength             = 1;
        NodeConfig.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
        NodeConfig.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
        NodeConfig.Init.Mode                        = DMA_NORMAL;
        NodeConfig.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
        NodeConfig.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
        NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
        NodeConfig.SrcAddress                       = (uint32_t) &SAI1_Block_A->DR;  // SAI1_A のデータレジスタ
        NodeConfig.DstAddress                       = (uint32_t) sai_buf;            // 受信先バッファ
        NodeConfig.DataSize                         = SAI_BUF_SIZE * 2;              // 転送アイテム数（32bitワード数）
        if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel3) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel3, NULL, &Node_GPDMA1_Channel3) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel3) != HAL_OK)
        {
            Error_Handler();
        }

        handle_GPDMA1_Channel3.Instance                         = GPDMA1_Channel3;
        handle_GPDMA1_Channel3.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
        handle_GPDMA1_Channel3.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
        handle_GPDMA1_Channel3.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
        handle_GPDMA1_Channel3.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
        handle_GPDMA1_Channel3.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
        if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel3) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel3, &List_GPDMA1_Channel3) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(saiHandle, hdmarx, handle_GPDMA1_Channel3);
#if 0
        if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel3, DMA_CHANNEL_NPRIV) != HAL_OK)
        {
            Error_Handler();
        }
#endif
    }
    /* SAI2 */
    if (saiHandle->Instance == SAI2_Block_A)
    {
        /* SAI2 clock enable */

        /** Initializes the peripherals clock
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
        PeriphClkInit.Sai2ClockSelection   = RCC_SAI2CLKSOURCE_PIN;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        if (SAI2_client == 0)
        {
            __HAL_RCC_SAI2_CLK_ENABLE();

            /* Peripheral interrupt init*/
            HAL_NVIC_SetPriority(SAI2_A_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SAI2_A_IRQn);
        }
        SAI2_client++;

        /**SAI2_A_Block_A GPIO Configuration
        PD11     ------> SAI2_SD_A
        PD12     ------> SAI2_FS_A
        PD13     ------> SAI2_SCK_A
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_PIN_13;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* Peripheral DMA init*/

        NodeConfig.NodeType                         = DMA_GPDMA_LINEAR_NODE;
        NodeConfig.Init.Request                     = GPDMA1_REQUEST_SAI2_A;
        NodeConfig.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
        NodeConfig.Init.Direction                   = DMA_MEMORY_TO_PERIPH;
        NodeConfig.Init.SrcInc                      = DMA_SINC_INCREMENTED;
        NodeConfig.Init.DestInc                     = DMA_DINC_FIXED;
        NodeConfig.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_WORD;
        NodeConfig.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_WORD;
        NodeConfig.Init.SrcBurstLength              = 1;
        NodeConfig.Init.DestBurstLength             = 1;
        NodeConfig.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT1 | DMA_DEST_ALLOCATED_PORT0;
        NodeConfig.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
        NodeConfig.Init.Mode                        = DMA_NORMAL;
        NodeConfig.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
        NodeConfig.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
        NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
        NodeConfig.SrcAddress                       = (uint32_t) sai_tx_buf;         // 送信元バッファ
        NodeConfig.DstAddress                       = (uint32_t) &SAI2_Block_A->DR;  // SAI2_A のデータレジスタ
        NodeConfig.DataSize                         = SAI_BUF_SIZE * 2;              // 転送アイテム数（32bitワード数）
        if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel2) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel2, NULL, &Node_GPDMA1_Channel2) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel2) != HAL_OK)
        {
            Error_Handler();
        }

        handle_GPDMA1_Channel2.Instance                         = GPDMA1_Channel2;
        handle_GPDMA1_Channel2.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
        handle_GPDMA1_Channel2.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
        handle_GPDMA1_Channel2.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
        handle_GPDMA1_Channel2.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
        handle_GPDMA1_Channel2.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
        if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel2) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel2, &List_GPDMA1_Channel2) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(saiHandle, hdmatx, handle_GPDMA1_Channel2);
#if 0
        if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel2, DMA_CHANNEL_NPRIV) != HAL_OK)
        {
            Error_Handler();
        }
#endif
    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* saiHandle)
{

    /* SAI1 */
    if (saiHandle->Instance == SAI1_Block_A)
    {
        SAI1_client--;
        if (SAI1_client == 0)
        {
            /* Peripheral clock disable */
            __HAL_RCC_SAI1_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(SAI1_A_IRQn);
        }

        /**SAI1_A_Block_A GPIO Configuration
        PE4     ------> SAI1_FS_A
        PE5     ------> SAI1_SCK_A
        PE6     ------> SAI1_SD_A
        */
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

        HAL_DMA_DeInit(saiHandle->hdmarx);
    }
    /* SAI2 */
    if (saiHandle->Instance == SAI2_Block_A)
    {
        SAI2_client--;
        if (SAI2_client == 0)
        {
            /* Peripheral clock disable */
            __HAL_RCC_SAI2_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(SAI2_A_IRQn);
        }

        /**SAI2_A_Block_A GPIO Configuration
        PD11     ------> SAI2_SD_A
        PD12     ------> SAI2_FS_A
        PD13     ------> SAI2_SCK_A
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13);

        HAL_DMA_DeInit(saiHandle->hdmatx);
    }
}

/**
 * @}
 */

/**
 * @}
 */
