/**
 ******************************************************************************
 * @file    usbd_audio.c
 * @author  MCD Application Team
 * @brief   This file provides the Audio core functions.
 *
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 * @verbatim
 *
 *          ===================================================================
 *                                AUDIO Class  Description
 *          ===================================================================
 *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
 *           Audio Devices V1.0 Mar 18, 98".
 *           This driver implements the following aspects of the specification:
 *             - Device descriptor management
 *             - Configuration descriptor management
 *             - Standard AC Interface Descriptor management
 *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
 *             - 1 Audio Streaming Endpoint
 *             - 1 Audio Terminal Input (1 channel)
 *             - Audio Class-Specific AC Interfaces
 *             - Audio Class-Specific AS Interfaces
 *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
 *             - Audio Feature Unit (limited to Mute control)
 *             - Audio Synchronization type: Asynchronous
 *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
 *          The current audio class version supports the following audio features:
 *             - Pulse Coded Modulation (PCM) format
 *             - sampling rate: 48KHz.
 *             - Bit resolution: 16
 *             - Number of channels: 2
 *             - No volume control
 *             - Mute/Unmute capability
 *             - Asynchronous Endpoints
 *
 * @note     In HS mode and when the DMA is used, all variables and data structures
 *           dealing with the DMA during the transaction process should be 32-bit aligned.
 *
 *
 *  @endverbatim
 ******************************************************************************
 */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}_audio.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"

#include "usbd_audio_if.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_AUDIO
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_Macros
 * @{
 */
#define AUDIO_SAMPLE_FREQ(frq) \
    (uint8_t) (frq), (uint8_t) ((frq >> 8)), (uint8_t) ((frq >> 16))

#define AUDIO_PACKET_SIZE(frq) \
    (uint8_t) (((frq * USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME_BYTES) / 1000U) & 0xFFU), (uint8_t) ((((frq * USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME_BYTES) / 1000U) >> 8) & 0xFFU)

#define AUDIO_PACKET_SIZE_MAX(frq)                                                                         \
    (uint8_t) (((frq / 1000U + AUDIO_PKT_EXT) * USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME_BYTES) & 0xFFU), \
        (uint8_t) ((((frq / 1000U + AUDIO_PKT_EXT) * USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME_BYTES) >> 8) & 0xFFU)

#ifdef USE_USBD_COMPOSITE
    #define AUDIO_PACKET_SZE_WORD(frq) (uint32_t) ((((frq) * USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME) / 1000U))
#endif /* USE_USBD_COMPOSITE  */
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
 * @{
 */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
#ifndef USE_USBD_COMPOSITE
static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t* length);
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t* length);
#endif /* USE_USBD_COMPOSITE  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef* pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef* pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef* pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef* pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void* USBD_AUDIO_GetAudioHeaderDesc(uint8_t* pConfDesc);

extern int8_t AUDIO_Mic_GetPacket(uint8_t* dst, uint16_t len);

extern uint8_t s_fb_ep;

/* FB送信 ACK カウンタ（if側から参照） */
extern uint8_t s_fb_busy; /* if側のフラグを参照 */
extern uint32_t g_fb_ack;
extern uint32_t g_fb_incomp;

extern volatile uint8_t s_last_arm_uf;
extern volatile uint8_t s_fb_arm_pending;
extern volatile uint32_t g_fb_tx_req, g_fb_tx_ok, g_fb_tx_busy;

extern volatile uint8_t s_fb_pkt[4];  // 3バイト送るが4バイト確保してアライン確保

static uint8_t mic_packet[AUDIO_IN_PACKET];
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_Variables
 * @{
 */

USBD_ClassTypeDef USBD_AUDIO =
    {
        USBD_AUDIO_Init,
        USBD_AUDIO_DeInit,
        USBD_AUDIO_Setup,
        USBD_AUDIO_EP0_TxReady,
        USBD_AUDIO_EP0_RxReady,
        USBD_AUDIO_DataIn,
        USBD_AUDIO_DataOut,
        USBD_AUDIO_SOF,
        USBD_AUDIO_IsoINIncomplete,
        USBD_AUDIO_IsoOutIncomplete,
#ifdef USE_USBD_COMPOSITE
        NULL,
        NULL,
        NULL,
        NULL,
#else
        USBD_AUDIO_GetCfgDesc,
        USBD_AUDIO_GetCfgDesc,
        USBD_AUDIO_GetCfgDesc,
        USBD_AUDIO_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
    {
        /* Configuration 1 */
        0x09,                              /* bLength */
        USB_DESC_TYPE_CONFIGURATION,       /* bDescriptorType */
        LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ), /* wTotalLength */
        HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
        0x03, /* bNumInterfaces */
        0x01, /* bConfigurationValue */
        0x00, /* iConfiguration */
    #if (USBD_SELF_POWERED == 1U)
        0xC0, /* bmAttributes: Bus Powered according to user configuration */
    #else
        0x80, /* bmAttributes: Bus Powered according to user configuration */
    #endif              /* USBD_SELF_POWERED */
        USBD_MAX_POWER, /* MaxPower (mA) */
        /* 09 byte(9)*/

        /* USB Speaker Standard interface descriptor */
        AUDIO_INTERFACE_DESC_SIZE,   /* bLength */
        USB_DESC_TYPE_INTERFACE,     /* bDescriptorType */
        0x00,                        /* bInterfaceNumber */
        0x00,                        /* bAlternateSetting */
        0x00,                        /* bNumEndpoints */
        USB_DEVICE_CLASS_AUDIO,      /* bInterfaceClass */
        AUDIO_SUBCLASS_AUDIOCONTROL, /* bInterfaceSubClass */
        AUDIO_PROTOCOL_UNDEFINED,    /* bInterfaceProtocol */
        0x00,                        /* iInterface */
        /* 09 byte(18)*/

        /* USB Speaker Class-specific AC Interface Descriptor */
        0x0A,                            /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_CONTROL_HEADER,            /* bDescriptorSubtype */
        0x00, /* 1.00 */                 /* bcdADC */
        0x01,
        0x48, /* wTotalLength */
        0x00,
        0x02, /* bInCollection */
        0x01, /* baInterfaceNr */
        0x02,
        /* 10 byte(28)*/

        /* USB Speaker Input Terminal Descriptor */
        AUDIO_INPUT_TERMINAL_DESC_SIZE,  /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_CONTROL_INPUT_TERMINAL,    /* bDescriptorSubtype */
        0x01,                            /* bTerminalID */
        0x01,                            /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
        0x01,
        0x00, /* bAssocTerminal */
        0x02, /* bNrChannels */
        0x03, /* wChannelConfig 0x0000  Mono */
        0x00,
        0x00, /* iChannelNames */
        0x00, /* iTerminal */
        /* 12 byte(40)*/

        /* USB Speaker Audio Feature Unit Descriptor */
        0x0A,                            /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_CONTROL_FEATURE_UNIT,      /* bDescriptorSubtype */
        AUDIO_OUT_STREAMING_CTRL,        /* bUnitID */
        0x01,                            /* bSourceID */
        0x01,                            /* bControlSize */
        AUDIO_CONTROL_MUTE,              /* bmaControls(0) */
        0,                               /* bmaControls(1) */
        0,                               /* bmaControls(2) */
        0x00,                            /* iTerminal */
        /* 10 byte(50)*/

        /* USB Speaker Output Terminal Descriptor */
        0x09,                            /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_CONTROL_OUTPUT_TERMINAL,   /* bDescriptorSubtype */
        0x03,                            /* bTerminalID */
        0x01,                            /* wTerminalType  0x0301 */
        0x03,
        0x00, /* bAssocTerminal */
        0x02, /* bSourceID */
        0x00, /* iTerminal */
        /* 09 byte(59) */

        /* ---- Record path: IT (LineIn 2ch) -> FU -> OT (USB Streaming) ---- */
        /* Input Terminal (LINE IN 2ch) [ID = 0x11] */
        0x0C, 0x24, 0x02,    /* bLength=12, CS_INTERFACE, INPUT_TERMINAL */
        0x11,                /* bTerminalID */
        0x02, 0x06,          /* wTerminalType = 0x0602 (Line Connector). MICなら 0x01,0x02 */
        0x00,                /* bAssocTerminal */
        USBD_AUDIO_CHANNELS, /* bNrChannels = 2 */
        0x03, 0x00,          /* wChannelConfig = L|R */
        0x00,                /* iChannelNames */
        0x00,                /* iTerminal */
                             /* 12 byte(71)*/

        /* Feature Unit (Record) [ID = 0x12, Source = IT(0x11)] */
        0x0A, 0x24, 0x06, /* bLength=10, CS_INTERFACE, FEATURE_UNIT */
        0x12,             /* bUnitID */
        0x11,             /* bSourceID */
        0x01,             /* bControlSize = 1 */
        0x01,             /* bmaControls(0): Mute on Master */
        0x00,             /* bmaControls(1) */
        0x00,             /* bmaControls(2) */
        0x00,             /* iFeature */
                          /* 10 byte(81)*/

        /* Output Terminal (USB Streaming) [ID = 0x13, Source = FU(0x12)] */
        0x09, 0x24, 0x03, /* bLength=9, CS_INTERFACE, OUTPUT_TERMINAL */
        0x13,             /* bTerminalID */
        0x01, 0x01,       /* wTerminalType = 0x0101 (USB Streaming) */
        0x00,             /* bAssocTerminal */
        0x12,             /* bSourceID */
        0x00,             /* iTerminal */
                          /* 09 byte(90)*/

        /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth */
        /* Interface 1, Alternate Setting 0                                              */
        AUDIO_INTERFACE_DESC_SIZE,     /* bLength */
        USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
        0x01,                          /* bInterfaceNumber */
        0x00,                          /* bAlternateSetting */
        0x00,                          /* bNumEndpoints */
        USB_DEVICE_CLASS_AUDIO,        /* bInterfaceClass */
        AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
        AUDIO_PROTOCOL_UNDEFINED,      /* bInterfaceProtocol */
        0x00,                          /* iInterface */
        /* 09 byte(99)*/

        /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
        /* Interface 1, Alternate Setting 1                                           */
        AUDIO_INTERFACE_DESC_SIZE,     /* bLength */
        USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
        0x01,                          /* bInterfaceNumber */
        0x01,                          /* bAlternateSetting */
        0x02,                          /* bNumEndpoints */
        USB_DEVICE_CLASS_AUDIO,        /* bInterfaceClass */
        AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
        AUDIO_PROTOCOL_UNDEFINED,      /* bInterfaceProtocol */
        0x00,                          /* iInterface */
        /* 09 byte(108)*/

        /* USB Speaker Audio Streaming Interface Descriptor */
        AUDIO_STREAMING_INTERFACE_DESC_SIZE, /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
        AUDIO_STREAMING_GENERAL,             /* bDescriptorSubtype */
        0x01,                                /* bTerminalLink */
        0x01,                                /* bDelay */
        0x01,                                /* wFormatTag AUDIO_FORMAT_PCM  0x0001 */
        0x00,
        /* 07 byte(115)*/

        /* USB Speaker Audio Type III Format Interface Descriptor */
        0x0B,                               /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE,    /* bDescriptorType */
        AUDIO_STREAMING_FORMAT_TYPE,        /* bDescriptorSubtype */
        AUDIO_FORMAT_TYPE_I,                /* bFormatType */
        USBD_AUDIO_CHANNELS,                /* bNrChannels */
        USBD_AUDIO_SUBFRAME_BYTES,          /* bSubFrameSize :  3 Bytes per frame (24bits) */
        USBD_AUDIO_RES_BITS,                /* bBitResolution (24-bits per sample) */
        0x01,                               /* bSamFreqType only one frequency supported */
        AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
                                            /* 11 byte(126)*/

        /* Endpoint 1 - Standard Descriptor */
        AUDIO_STANDARD_ENDPOINT_DESC_SIZE,      /* bLength */
        USB_DESC_TYPE_ENDPOINT,                 /* bDescriptorType */
        AUDIO_OUT_EP,                           /* bEndpointAddress 1 out endpoint */
        0x05,                                   /* bmAttributes */
        AUDIO_PACKET_SIZE_MAX(USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*3(HalfWord)) */
        AUDIO_HS_BINTERVAL,                     /* bInterval */
        0x00,                                   /* bRefresh */
        AUDIO_FB_EP,                            /* bSynchAddress */
        /* 09 byte(135)*/

        /* Endpoint - Audio Streaming Descriptor */
        AUDIO_STREAMING_ENDPOINT_DESC_SIZE, /* bLength */
        AUDIO_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType */
        AUDIO_ENDPOINT_GENERAL,             /* bDescriptor */
        0x00,                               /* bmAttributes */
        0x00,                               /* bLockDelayUnits */
        0x00,                               /* wLockDelay */
        0x00,
        /* 07 byte(142)*/

        /* Feedback IN Endpoint (Standard, 9B) */
        AUDIO_FEEDBACK_ENDPOINT_DESC_SIZE, /* bLength */
        USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType: ENDPOINT */
        AUDIO_FB_EP,                       /* bEndpointAddress: IN */
        0x11,                              /* bmAttributes: Isoch | Usage=Feedback */
        0x03,
        0x00,               /* wMaxPacketSize = 3 bytes (HS:10.14) */
        AUDIO_HS_BINTERVAL, /* bInterval: 1ms (=2^(4-1) µframes) */
        0x00,               /* bRefresh (未使用) */
        0x00,               /* bSynchAddress=0 */
                            /* 09 byte(151) */

        /* ---------------- AS(IN) Interface #2 ---------------- */
        /* Std AS Interface, alt 0 */
        0x09, 0x04,       /* INTERFACE */
        0x02,             /* bInterfaceNumber = 2 */
        0x00,             /* bAlternateSetting = 0 */
        0x00,             /* bNumEndpoints = 0 */
        0x01, 0x02, 0x00, /* AUDIO, AUDIO_STREAMING */
        0x00,
        /* 09 byte(160)*/

        /* Std AS Interface, alt 1 */
        0x09, 0x04,       /* INTERFACE */
        0x02,             /* bInterfaceNumber = 2 */
        0x01,             /* bAlternateSetting = 1 */
        0x01,             /* bNumEndpoints = 1 (IN) */
        0x01, 0x02, 0x00, /* AUDIO, AUDIO_STREAMING */
        0x00,
        /* 09 byte(169)*/

        /* CS AS General (link to OT USB Streaming = 0x13) */
        0x07, 0x24, 0x01, /* bLength=7, CS_INTERFACE, AS_GENERAL */
        0x13,             /* bTerminalLink = 0x13 */
        0x01,             /* bDelay */
        0x01, 0x00,       /* wFormatTag = PCM */
        /* 07 byte(176)*/

        /* Type I Format (2ch, 16bit, 48kHz) */
        0x0B, 0x24, 0x02,                   /* bLength=11, CS_INTERFACE, FORMAT_TYPE */
        0x01,                               /* FORMAT_TYPE_I */
        USBD_AUDIO_CHANNELS,                /* 2ch */
        USBD_AUDIO_SUBFRAME_BYTES,          /* 24-bit (3 bytes) */
        USBD_AUDIO_RES_BITS,                /* 24 bits */
        0x01,                               /* 1 discrete freq */
        AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
        /* 11 byte(187)*/

        /* Std ISO Endpoint (IN) 0x81, Async (0x05), 1ms, 288B */
        0x09, 0x05,                             /* ENDPOINT */
        AUDIO_IN_EP,                            /* bEndpointAddress = 0x81 (IN) */
        0x05,                                   /* bmAttributes = Isochronous | Asynchronous | Data */
        AUDIO_PACKET_SIZE_MAX(USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*3(Word)) */
        AUDIO_HS_BINTERVAL,                     /* bInterval */
        0x00,                                   /* bRefresh */
        0x00,                                   /* bSynchAddress */
        /* 09 byte(196)*/

        /* CS ISO Endpoint (IN) */
        0x07, 0x25, 0x01, /* CS_ENDPOINT, EP_GENERAL */
        0x00,             /* bmAttributes */
        0x00,             /* bLockDelayUnits */
        0x00, 0x00        /* wLockDelay */
                          /* 07 byte(203)*/
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
    {
        USB_LEN_DEV_QUALIFIER_DESC,
        USB_DESC_TYPE_DEVICE_QUALIFIER,
        0x00,
        0x02,
        0x00,
        0x00,
        0x00,
        0x40,
        0x01,
        0x00,
};
#endif /* USE_USBD_COMPOSITE  */

static uint8_t AUDIOOutEpAdd = AUDIO_OUT_EP;
static uint8_t AUDIOFbEpAdd  = AUDIO_FB_EP;
static uint8_t AUDIOInEpAdd  = AUDIO_IN_EP;
/**
 * @}
 */

/** @defgroup USBD_AUDIO_Private_Functions
 * @{
 */

/**
 * @brief  USBD_AUDIO_Init
 *         Initialize the AUDIO interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);
    USBD_AUDIO_HandleTypeDef* haudio;

    /* Allocate Audio structure */
    haudio = (USBD_AUDIO_HandleTypeDef*) USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

    if (haudio == NULL)
    {
        pdev->pClassDataCmsit[pdev->classId] = NULL;
        return (uint8_t) USBD_EMEM;
    }

    pdev->pClassDataCmsit[pdev->classId] = (void*) haudio;
    pdev->pClassData                     = pdev->pClassDataCmsit[pdev->classId];

#ifdef USE_USBD_COMPOSITE
    /* Get the Endpoints addresses allocated for this class instance */
    AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t) pdev->classId);
#endif /* USE_USBD_COMPOSITE */

    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
        pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
    }
    else /* LOW and FULL-speed endpoints */
    {
        pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
    }

    /* Open EP OUT */
    (void) USBD_LL_OpenEP(pdev, AUDIOOutEpAdd, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET);
    pdev->ep_out[AUDIOOutEpAdd & 0xFU].is_used = 1U;

    haudio->alt_setting = 0U;
    haudio->offset      = AUDIO_OFFSET_UNKNOWN;
    haudio->wr_ptr      = 0U;
    haudio->rd_ptr      = 0U;
    haudio->rd_enable   = 0U;
    haudio->mic_prime   = 0U;

    /* Initialize the Audio output Hardware layer */
    if (((USBD_AUDIO_ItfTypeDef*) pdev->pUserData[pdev->classId])->Init(USBD_AUDIO_FREQ, AUDIO_DEFAULT_VOLUME, 0U) != 0U)
    {
        return (uint8_t) USBD_FAIL;
    }

    /* Prepare Out endpoint to receive 1st packet */
    (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, haudio->buffer, AUDIO_OUT_PACKET);

    return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_AUDIO_Init
 *         DeInitialize the AUDIO layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
    /* Get the Endpoints addresses allocated for this class instance */
    AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t) pdev->classId);
#endif /* USE_USBD_COMPOSITE */

    /* Open EP OUT */
    (void) USBD_LL_CloseEP(pdev, AUDIOOutEpAdd);
    pdev->ep_out[AUDIOOutEpAdd & 0xFU].is_used   = 0U;
    pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = 0U;

    /* DeInit  physical Interface components */
    if (pdev->pClassDataCmsit[pdev->classId] != NULL)
    {
        ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData[pdev->classId])->DeInit(0U);
        (void) USBD_free(pdev->pClassDataCmsit[pdev->classId]);
        pdev->pClassDataCmsit[pdev->classId] = NULL;
        pdev->pClassData                     = NULL;
    }

    /* Open EP IN */
#ifdef USE_USBD_COMPOSITE
    AUDIOInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_ISOC, (uint8_t) pdev->classId);
#endif
    (void) USBD_LL_CloseEP(pdev, AUDIOInEpAdd);
    pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used   = 0U;
    pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = 0U;

    return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_AUDIO_Setup
 *         Handle the AUDIO specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
extern volatile uint8_t s_fb_opened;
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
    USBD_AUDIO_HandleTypeDef* haudio;
    uint16_t len;
    uint8_t* pbuf;
    uint16_t status_info   = 0U;
    USBD_StatusTypeDef ret = USBD_OK;

    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    if (haudio == NULL)
    {
        return (uint8_t) USBD_FAIL;
    }

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest)
        {
        case AUDIO_REQ_GET_CUR:
            AUDIO_REQ_GetCurrent(pdev, req);
            break;

        case AUDIO_REQ_SET_CUR:
            AUDIO_REQ_SetCurrent(pdev, req);
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest)
        {
        case USB_REQ_GET_STATUS:
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                (void) USBD_CtlSendData(pdev, (uint8_t*) &status_info, 2U);
            }
            else
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_GET_DESCRIPTOR:
            if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
            {
                pbuf = (uint8_t*) USBD_AUDIO_GetAudioHeaderDesc(pdev->pConfDesc);
                if (pbuf != NULL)
                {
                    len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);
                    (void) USBD_CtlSendData(pdev, pbuf, len);
                }
                else
                {
                    USBD_CtlError(pdev, req);
                    ret = USBD_FAIL;
                }
            }
            break;

        case USB_REQ_GET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                (void) USBD_CtlSendData(pdev, (uint8_t*) &haudio->alt_setting, 1U);
            }
            else
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
            break;

        case USB_REQ_SET_INTERFACE:
#if 0
        	if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                if ((uint8_t) (req->wValue) <= USBD_MAX_NUM_INTERFACES)
                {
                    haudio->alt_setting = (uint8_t) (req->wValue);
                }
                else
                {
                    /* Call the error management function (command will be NAKed */
                    USBD_CtlError(pdev, req);
                    ret = USBD_FAIL;
                }
            }
            else
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
#else
            if (pdev->dev_state != USBD_STATE_CONFIGURED)
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
                break;
            }

            uint8_t ifnum = (uint8_t) (req->wIndex & 0xFF);
            uint8_t alt   = (uint8_t) (req->wValue & 0xFF);

            haudio->alt_setting = alt;  // 既存の動作を踏襲

            /* ---- Speaker: Interface #1 (OUT) ---- */
            if (ifnum == 0x01)
            {
                if (alt == 1)
                {
                    /* 再生開始に備えてリングを初期化し、受信をプライム */
                    haudio->wr_ptr    = 0U;
                    haudio->rd_ptr    = 0U;
                    haudio->rd_enable = 0U;
                    haudio->offset    = AUDIO_OFFSET_UNKNOWN;
                    /* 受信EPをフラッシュしてから最初の受信を投入 */
                    USBD_LL_FlushEP(pdev, AUDIOOutEpAdd);
                    (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, &haudio->buffer[haudio->wr_ptr], AUDIO_OUT_PACKET);

                    if (pdev->dev_speed == USBD_SPEED_HIGH)
                        pdev->ep_in[AUDIOFbEpAdd & 0xF].bInterval = AUDIO_HS_BINTERVAL;
                    else
                        pdev->ep_in[AUDIOFbEpAdd & 0xF].bInterval = AUDIO_FS_BINTERVAL;

                    USBD_LL_OpenEP(pdev, AUDIOFbEpAdd, USBD_EP_TYPE_ISOC, 3);  // 3 bytes
                    pdev->ep_in[AUDIOFbEpAdd & 0xF].is_used   = 1U;
                    pdev->ep_in[AUDIOFbEpAdd & 0xF].maxpacket = 3U;

                    s_fb_busy   = 0; /* ★ busy解除 */
                    s_fb_opened = 1;

                    /* ★ 初回プライム：直近の値を用意して1発アーム */
                    AUDIO_FB_Task_1ms(); /* 値だけ用意 */
                    s_fb_arm_pending = 1;
                }
                else
                {
                    /* Alt=0 に戻る場合はEPをフラッシュ */
                    USBD_LL_FlushEP(pdev, AUDIOOutEpAdd);

                    USBD_LL_CloseEP(pdev, AUDIOFbEpAdd);
                    pdev->ep_in[AUDIOFbEpAdd & 0xF].is_used   = 0U;
                    pdev->ep_in[AUDIOFbEpAdd & 0xF].maxpacket = 0U;

                    s_fb_busy   = 0; /* ★ busy解除 */
                    s_fb_opened = 0;
                }
            }

            /* ---- Mic: Interface #2 ---- */
            if (ifnum == 0x02)
            {
    #ifdef USE_USBD_COMPOSITE
                AUDIOInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_ISOC, (uint8_t) pdev->classId);
    #endif

                if (alt == 1)
                {
                    /* IN EP open（Alt=1運用開始） */
                    if (pdev->dev_speed == USBD_SPEED_HIGH)
                    {
                        pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = AUDIO_HS_BINTERVAL;  // HSは通常 4 (=1ms)
                    }
                    else
                    {
                        pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = AUDIO_FS_BINTERVAL;  // FSは 1 (=1ms)
                    }
                    (void) USBD_LL_OpenEP(pdev, AUDIOInEpAdd, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
                    pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used = 1U;

                    haudio->mic_prime = 1;
                }
                else
                {
                    /* Alt=0: IN EP停止 */
                    (void) USBD_LL_CloseEP(pdev, AUDIOInEpAdd);
                    pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used   = 0U;
                    pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = 0U;
                }
            }

            /* 標準リクエストの応答 */
            (void) USBD_CtlSendStatus(pdev);
#endif
            break;

        case USB_REQ_CLEAR_FEATURE:
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
        }
        break;
    default:
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
    }

    return (uint8_t) ret;
}

#ifndef USE_USBD_COMPOSITE
/**
 * @brief  USBD_AUDIO_GetCfgDesc
 *         return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t* length)
{
    *length = (uint16_t) sizeof(USBD_AUDIO_CfgDesc);

    return USBD_AUDIO_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
 * @brief  USBD_AUDIO_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
    extern USBD_HandleTypeDef hUsbDeviceHS;
    USBD_EndpointTypeDef* ep = &hUsbDeviceHS.ep_in[s_fb_ep & 0xF];
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
        return (uint8_t) USBD_FAIL;
    if (!ep->is_used || ep->maxpacket == 0)
        return (uint8_t) USBD_FAIL;

    if ((epnum & 0x0F) == (AUDIOInEpAdd & 0x0F))
    {
        (void) AUDIO_Mic_GetPacket(mic_packet, AUDIO_IN_PACKET);
        (void) USBD_LL_Transmit(pdev, AUDIOInEpAdd, mic_packet, AUDIO_IN_PACKET);

        // printf("data in!\n");
    }

    /* epnum は EP番号(0..15)。アドレスではない点に注意 */
    if ((epnum & 0x0F) == (AUDIOFbEpAdd & 0x0F))
    {
        s_fb_busy = 0; /* ★ 完了で busy を確実に落とす */
        g_fb_ack++;    /* ★ ACK をカウント */
#if 0
        uint8_t uf_now       = USBD_GetMicroframeHS();
        uint8_t duf          = (uint8_t) ((uf_now - s_last_arm_uf) & 0x7);
        static uint32_t last = 0;
        uint32_t now         = HAL_GetTick();
        if ((now - last) >= 1000)
        {
            USB_OTG_INEndpointTypeDef* in =
                (USB_OTG_INEndpointTypeDef*) ((uint32_t) USB_OTG_HS + USB_OTG_IN_ENDPOINT_BASE);
            uint8_t idx = (uint8_t) (s_fb_ep & 0x0F);
            printf("[FB:ackUF] dUF=%u (arm=%u -> now=%u) DIEPCTL=0x%08lX DIEPTSIZ=0x%08lX\n", duf, s_last_arm_uf, uf_now, (unsigned long) in[idx].DIEPCTL, (unsigned long) in[idx].DIEPTSIZ);
            last = now;
        }
#endif
        /* ★ ACKが来たら、その場で次回分をアーム */
        s_fb_arm_pending = 1;
        return (uint8_t) USBD_OK;
    }

    /* Only OUT data are processed */
    return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_AUDIO_EP0_RxReady
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef* pdev)
{
    USBD_AUDIO_HandleTypeDef* haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    if (haudio == NULL)
    {
        return (uint8_t) USBD_FAIL;
    }

    if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
    {
        /* In this driver, to simplify code, only SET_CUR request is managed */

        if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
        {
            ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData[pdev->classId])->MuteCtl(haudio->control.data[0]);
            haudio->control.cmd = 0U;
            haudio->control.len = 0U;
        }
    }

    return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_EP0_TxReady
 *         handle EP0 TRx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef* pdev)
{
    UNUSED(pdev);

    /* Only OUT control data are processed */
    return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef* pdev)
{
    // UNUSED(pdev);

#if 1
    if (pdev && pdev->dev_state == USBD_STATE_CONFIGURED && s_fb_opened)
    {
        if (USBD_GetMicroframeHS() == 0)
        {
            AUDIO_FB_Task_1ms();
        }

        if (s_fb_arm_pending && !s_fb_busy && USBD_GetMicroframeHS() == 7)
        {
            // uint8_t idx = (uint8_t) (s_fb_ep & 0x0F);
            /* “次msの0”を狙うので even を指定（uF=7はodd） */
            // pdev->ep_in[idx].even_odd_frame = 1U;  // even
            /* ★ “次のms”に確実に乗るよう予約 → 送信 */
            // USBD_FB_ProgramNextMs(s_fb_ep);  // ← ここでだけ呼ぶ
            if (USBD_LL_Transmit(pdev, s_fb_ep, s_fb_pkt, 3) == USBD_OK)
            {
                s_fb_busy        = 1;
                s_fb_arm_pending = 0;  // 消費
                g_fb_tx_req++;
                g_fb_tx_ok++;

    #if 0
                static uint32_t last_ms = 0;
                const uint32_t now      = HAL_GetTick();
                if ((now - last_ms) >= 1000)
                {
        #if 0
                	// printf("[FB:arm]  st=OK  idx=%u mps=%lu pkt=3 pcnt(exp)=1 xlen=%lu xcnt=%lu evenodd=%u\n", idx, (unsigned long) ep->maxpacket, (unsigned long) ep->xfer_len, (unsigned long) ep->xfer_count, (unsigned) ep->even_odd_frame);

                    printf("[FB:rate] req=%lu ok=%lu ack=%lu incomp=%lu busy_skip=%lu ep=0x%02X\n", (unsigned long) g_fb_tx_req, (unsigned long) g_fb_tx_ok, (unsigned long) g_fb_ack, (unsigned long) g_fb_incomp, (unsigned long) g_fb_tx_busy, (unsigned) s_fb_ep);
                    g_fb_tx_req = g_fb_tx_ok = g_fb_ack = g_fb_incomp = g_fb_tx_busy = 0;
        #endif
                    AUDIO_Stats_On1sTick(); /* ← 1秒境界で確定 */
                    AUDIO_Stats st;
                    AUDIO_GetStats(&st);
                    printf("[AUDIO] cap=%u frm, level[now/min/max]=%u/%u/%u, "
                           "fps[in/out]=%u/%u, dLevel/s=%ld, "
                           "UR(ev=%u,frm=%u), OR(ev=%u,frm=%u), copy_us(last=%u,max=%u)\n",
                           st.rxq_capacity_frames, st.rxq_level_now, st.rxq_level_min, st.rxq_level_max, st.in_fps, st.out_fps, (long) st.dlevel_per_s, st.underrun_events, st.underrun_frames, st.overrun_events, st.overrun_frames, st.copy_us_last, st.copy_us_max);

                    last_ms = now;
                }
    #endif
            }
            else
            {
                g_fb_tx_busy++;
            }
        }
    }
#endif

    USBD_AUDIO_HandleTypeDef* haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];
    if (!haudio)
    {
        return (uint8_t) USBD_OK;
    }

    /* alt=1 が選択され、まだ初回送信していない？ */
    if (haudio->alt_setting == 1 && haudio->mic_prime)
    {
        /* いったんフラッシュして、フレーム境界(1ms)に揃えて送る */
        USBD_LL_FlushEP(pdev, AUDIOInEpAdd);
        AUDIO_Mic_GetPacket(mic_packet, AUDIO_IN_PACKET);
        USBD_LL_Transmit(pdev, AUDIOInEpAdd, mic_packet, AUDIO_IN_PACKET);
        haudio->mic_prime = 0;

        printf("first transmit.\n");
    }

    return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_AUDIO_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @param  offset: audio offset
 * @retval status
 */
void USBD_AUDIO_Sync(USBD_HandleTypeDef* pdev, AUDIO_OffsetTypeDef offset)
{
    USBD_AUDIO_HandleTypeDef* haudio;
    uint32_t BufferSize = AUDIO_TOTAL_BUF_SIZE / 2U;

    if (pdev->pClassDataCmsit[pdev->classId] == NULL)
    {
        return;
    }

    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    haudio->offset = offset;

    if (haudio->rd_enable == 1U)
    {
        haudio->rd_ptr += (uint16_t) BufferSize;

        if (haudio->rd_ptr == AUDIO_TOTAL_BUF_SIZE)
        {
            /* roll back */
            haudio->rd_ptr = 0U;
        }
    }

    if (haudio->rd_ptr > haudio->wr_ptr)
    {
        if ((haudio->rd_ptr - haudio->wr_ptr) < AUDIO_OUT_PACKET)
        {
            BufferSize += 4U;
        }
        else
        {
            if ((haudio->rd_ptr - haudio->wr_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
            {
                BufferSize -= 4U;
            }
        }
    }
    else
    {
        if ((haudio->wr_ptr - haudio->rd_ptr) < AUDIO_OUT_PACKET)
        {
            BufferSize -= 4U;
        }
        else
        {
            if ((haudio->wr_ptr - haudio->rd_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
            {
                BufferSize += 4U;
            }
        }
    }

    if (haudio->offset == AUDIO_OFFSET_FULL)
    {
        ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData[pdev->classId])->AudioCmd(&haudio->buffer[0], BufferSize, AUDIO_CMD_PLAY);
        haudio->offset = AUDIO_OFFSET_NONE;
    }
}

/**
 * @brief  USBD_AUDIO_IsoINIncomplete
 *         handle data ISO IN Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
    // UNUSED(pdev);
    // UNUSED(epnum);

    if ((epnum & 0x0F) == (AUDIOInEpAdd & 0x0F))
    {
        (void) USBD_LL_Transmit(pdev, AUDIOInEpAdd, mic_packet, AUDIO_IN_PACKET);

        // printf("incomplete...\n");
    }

    if ((epnum & 0x0F) == (AUDIOFbEpAdd & 0x0F))
    {
        s_fb_busy = 0; /* ★ 完了しなくても次回送れるように busy を解放 */
        g_fb_incomp++;
#if 0
        uint8_t uf_now = USBD_GetMicroframeHS();
        /* 差分（0..7）: uf_now が “アーム時”からどれだけ進んで失敗したか */
        uint8_t duf = (uint8_t) ((uf_now - s_last_arm_uf) & 0x7);
        /* 1秒1回だけでOKなら抑制して */
        static uint32_t last = 0;
        uint32_t now         = HAL_GetTick();
        if ((now - last) >= 1000)
        {
            /* DIEPCTL/SIZも参考ダンプ（hexで可） */
            USB_OTG_INEndpointTypeDef* in =
                (USB_OTG_INEndpointTypeDef*) ((uint32_t) USB_OTG_HS + USB_OTG_IN_ENDPOINT_BASE);
            uint8_t idx = (uint8_t) (s_fb_ep & 0x0F);
            printf("[FB:incUF] dUF=%u (arm=%u -> now=%u) DIEPCTL=0x%08lX DIEPTSIZ=0x%08lX\n", duf, s_last_arm_uf, uf_now, (unsigned long) in[idx].DIEPCTL, (unsigned long) in[idx].DIEPTSIZ);
            last = now;
        }
#endif
        /* ★ 未成立でも“すぐ次回分”をアーム（次のmsで間に合うよう先行） */
        s_fb_arm_pending = 1;

#if 0
        /* 1秒に1回だけ詳細 */
        static uint32_t s_inc_last = 0;
        uint32_t now               = HAL_GetTick();
        if ((now - s_inc_last) >= 1000)
        {
            extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
            uint8_t idx             = (uint8_t) (s_fb_ep & 0x0F);
            const PCD_EPTypeDef* ep = &hpcd_USB_OTG_HS.IN_ep[idx];
            printf("[FB:inc] idx=%u mps=%lu xlen=%lu xcnt=%lu evenodd=%u\n", idx, (unsigned long) ep->maxpacket, (unsigned long) ep->xfer_len, (unsigned long) ep->xfer_count, (unsigned) ep->even_odd_frame);
            s_inc_last = now;
        }
#endif
    }

    return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_IsoOutIncomplete
 *         handle data ISO OUT Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef* haudio;

    if (pdev->pClassDataCmsit[pdev->classId] == NULL)
    {
        return (uint8_t) USBD_FAIL;
    }

    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    /* Prepare Out endpoint to receive next audio packet */
    (void) USBD_LL_PrepareReceive(pdev, epnum, &haudio->buffer[haudio->wr_ptr], AUDIO_OUT_PACKET);

    return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint32_t s_rx_bytes_acc = 0;
static uint32_t s_rx_last_ms   = 0;
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
    uint16_t PacketSize;
    USBD_AUDIO_HandleTypeDef* haudio;

#ifdef USE_USBD_COMPOSITE
    /* Get the Endpoints addresses allocated for this class instance */
    AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t) pdev->classId);
#endif /* USE_USBD_COMPOSITE */

    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    if (haudio == NULL)
    {
        return (uint8_t) USBD_FAIL;
    }

    if (epnum == AUDIOOutEpAdd)
    {
        /* Get received data packet length */
        PacketSize = (uint16_t) USBD_LL_GetRxDataSize(pdev, epnum);

        /* Packet received Callback */
        ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData[pdev->classId])->PeriodicTC(&haudio->buffer[haudio->wr_ptr], PacketSize, AUDIO_OUT_TC);

        /* Increment the Buffer pointer or roll it back when all buffers are full */
        haudio->wr_ptr += PacketSize;

        if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE)
        {
            /* All buffers are full: roll back */
            haudio->wr_ptr = 0U;

            if (haudio->offset == AUDIO_OFFSET_UNKNOWN)
            {
                ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData[pdev->classId])->AudioCmd(&haudio->buffer[0], AUDIO_TOTAL_BUF_SIZE / 2U, AUDIO_CMD_START);
                haudio->offset = AUDIO_OFFSET_NONE;
            }
        }

        if (haudio->rd_enable == 0U)
        {
            if (haudio->wr_ptr == (AUDIO_TOTAL_BUF_SIZE / 2U))
            {
                haudio->rd_enable = 1U;
            }
        }

        /* Prepare Out endpoint to receive next audio packet */
        (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, &haudio->buffer[haudio->wr_ptr], AUDIO_OUT_PACKET);
    }

    return (uint8_t) USBD_OK;
}

/**
 * @brief  AUDIO_Req_GetCurrent
 *         Handles the GET_CUR Audio control request.
 * @param  pdev: device instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
    USBD_AUDIO_HandleTypeDef* haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    if (haudio == NULL)
    {
        return;
    }

    (void) USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);

    /* Send the current mute state */
    (void) USBD_CtlSendData(pdev, haudio->control.data, MIN(req->wLength, USB_MAX_EP0_SIZE));
}

/**
 * @brief  AUDIO_Req_SetCurrent
 *         Handles the SET_CUR Audio control request.
 * @param  pdev: device instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
    USBD_AUDIO_HandleTypeDef* haudio;
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    if (haudio == NULL)
    {
        return;
    }

    if (req->wLength != 0U)
    {
        haudio->control.cmd  = AUDIO_REQ_SET_CUR;                             /* Set the request value */
        haudio->control.len  = (uint8_t) MIN(req->wLength, USB_MAX_EP0_SIZE); /* Set the request data length */
        haudio->control.unit = HIBYTE(req->wIndex);                           /* Set the request target unit */

        /* Prepare the reception of the buffer over EP0 */
        (void) USBD_CtlPrepareRx(pdev, haudio->control.data, haudio->control.len);
    }
}

#ifndef USE_USBD_COMPOSITE
/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t* length)
{
    *length = (uint16_t) sizeof(USBD_AUDIO_DeviceQualifierDesc);

    return USBD_AUDIO_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
 * @brief  USBD_AUDIO_RegisterInterface
 * @param  pdev: device instance
 * @param  fops: Audio interface callback
 * @retval status
 */
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef* pdev, USBD_AUDIO_ItfTypeDef* fops)
{
    if (fops == NULL)
    {
        return (uint8_t) USBD_FAIL;
    }

    pdev->pUserData[pdev->classId] = fops;

    return (uint8_t) USBD_OK;
}

#ifdef USE_USBD_COMPOSITE
/**
 * @brief  USBD_AUDIO_GetEpPcktSze
 * @param  pdev: device instance (reserved for future use)
 * @param  If: Interface number (reserved for future use)
 * @param  Ep: Endpoint number (reserved for future use)
 * @retval status
 */
uint32_t USBD_AUDIO_GetEpPcktSze(USBD_HandleTypeDef* pdev, uint8_t If, uint8_t Ep)
{
    uint32_t mps;

    UNUSED(pdev);
    UNUSED(If);
    UNUSED(Ep);

    mps = AUDIO_PACKET_SZE_WORD(USBD_AUDIO_FREQ);

    /* Return the wMaxPacketSize value in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
    return mps;
}
#endif /* USE_USBD_COMPOSITE */

/**
 * @brief  USBD_AUDIO_GetAudioHeaderDesc
 *         This function return the Audio descriptor
 * @param  pdev: device instance
 * @param  pConfDesc:  pointer to Bos descriptor
 * @retval pointer to the Audio AC Header descriptor
 */
static void* USBD_AUDIO_GetAudioHeaderDesc(uint8_t* pConfDesc)
{
    USBD_ConfigDescTypeDef* desc  = (USBD_ConfigDescTypeDef*) (void*) pConfDesc;
    USBD_DescHeaderTypeDef* pdesc = (USBD_DescHeaderTypeDef*) (void*) pConfDesc;
    uint8_t* pAudioDesc           = NULL;
    uint16_t ptr;

    if (desc->wTotalLength > desc->bLength)
    {
        ptr = desc->bLength;

        while (ptr < desc->wTotalLength)
        {
            pdesc = USBD_GetNextDesc((uint8_t*) pdesc, &ptr);
            if ((pdesc->bDescriptorType == AUDIO_INTERFACE_DESCRIPTOR_TYPE) &&
                (pdesc->bDescriptorSubType == AUDIO_CONTROL_HEADER))
            {
                pAudioDesc = (uint8_t*) pdesc;
                break;
            }
        }
    }
    return pAudioDesc;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
