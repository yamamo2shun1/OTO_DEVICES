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

#define AUDIO_PACKET_SZE(frq) \
    (uint8_t) (((frq * 2U * 2U) / 1000U) & 0xFFU), (uint8_t) ((((frq * 2U * 2U) / 1000U) >> 8) & 0xFFU)

#ifdef USE_USBD_COMPOSITE
    #define AUDIO_PACKET_SZE_WORD(frq) (uint32_t) ((((frq) * 2U * 2U) / 1000U))
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

/**
 * @}
 */

static volatile uint8_t alt_as_out = 0;  // IF#1 の Alt
static volatile uint8_t alt_as_in  = 0;  // IF#2 の Alt

/* === USB loopback: OUT(1ms=192B) -> IN(次の1msで返す) === */
__attribute__((aligned(32))) __ALIGN_BEGIN static uint8_t s_last_out[AUDIO_IN_PACKET] __ALIGN_END; /* 192B */
__attribute__((aligned(32))) __ALIGN_BEGIN static uint8_t s_silence[AUDIO_IN_PACKET] __ALIGN_END;
static volatile uint8_t s_have_frame = 0; /* 直前OUTを保持しているか */
static volatile uint8_t in_busy      = 0; /* IN送信中か（完了はDataInで解除） */
static volatile uint8_t in_alt1      = 0; /* AS(IN)がAlt1(動作中)か */
static volatile uint8_t busy_ticks   = 0;

/* 受信側の1ms作業バッファ（既存PrepareReceiveに合わせるなら不要だが、明示的に用意） */
__attribute__((aligned(32))) __ALIGN_BEGIN static uint8_t s_out_frame[AUDIO_OUT_PACKET] __ALIGN_END;

USBD_AUDIO_LB_CTX g_audio_lb;

// 受信内容の簡易チェック用
static volatile uint32_t dbg_out_sum = 0;  // 直近OUT 1msのバイト合計
static volatile uint32_t dbg_out_nz  = 0;  // 連続“非ゼロ”観測フレーム数
static volatile uint32_t dbg_out_z   = 0;  // 連続“ゼロ”観測フレーム数

// テストトーン（1kHz相当の矩形：16bit/LR/48kHz → 1msで48サンプル）
__attribute__((aligned(32))) __ALIGN_BEGIN static uint8_t s_beep[AUDIO_IN_PACKET] __ALIGN_END;
static void fill_beep_1ms(void)
{
    // 簡単な矩形波（左右同じ）。16-bit little-endian
    static uint8_t phase = 0;  // 0..47
    int16_t hi = 12000, lo = -12000;
    int16_t v  = (phase < 24) ? hi : lo;
    uint8_t* p = s_beep;
    for (int i = 0; i < 48; ++i)
    {                                    // 48サンプル/1ms @48kHz
        int16_t s = (i < 24) ? hi : lo;  // 1kHz矩形（おおよそ）
        // L
        *p++ = (uint8_t) (s & 0xFF);
        *p++ = (uint8_t) ((s >> 8) & 0xFF);
        // R
        *p++ = (uint8_t) (s & 0xFF);
        *p++ = (uint8_t) ((s >> 8) & 0xFF);
    }
    phase = (phase + 1) % 48;
}

#define DCACHE_LINE 32U
static inline void dcache_invalidate(void* addr, size_t len)
{
    uintptr_t a = (uintptr_t) addr & ~(uintptr_t) (DCACHE_LINE - 1U);
    size_t l    = (len + DCACHE_LINE - 1U) & ~(size_t) (DCACHE_LINE - 1U);
    SCB_InvalidateDCache_by_Addr((uint32_t*) a, (int32_t) l);
}
static inline void dcache_clean(const void* addr, size_t len)
{
    uintptr_t a = (uintptr_t) addr & ~(uintptr_t) (DCACHE_LINE - 1U);
    size_t l    = (len + DCACHE_LINE - 1U) & ~(size_t) (DCACHE_LINE - 1U);
    SCB_CleanDCache_by_Addr((uint32_t*) a, (int32_t) l);
}
static inline void dcache_ciinv(void* addr, size_t len)
{  // Clean+Invalidate
    uintptr_t a = (uintptr_t) addr & ~(uintptr_t) (DCACHE_LINE - 1U);
    size_t l    = (len + DCACHE_LINE - 1U) & ~(size_t) (DCACHE_LINE - 1U);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*) a, (int32_t) l);
}

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
        0x0C, 0x24, 0x02, /* bLength=12, CS_INTERFACE, INPUT_TERMINAL */
        0x11,             /* bTerminalID */
        0x02, 0x06,       /* wTerminalType = 0x0602 (Line Connector). MICなら 0x01,0x02 */
        0x00,             /* bAssocTerminal */
        0x02,             /* bNrChannels = 2 */
        0x03, 0x00,       /* wChannelConfig = L|R */
        0x00,             /* iChannelNames */
        0x00,             /* iTerminal */
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
        0x01,                          /* bNumEndpoints */
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
        0x02,                               /* bNrChannels */
        0x02,                               /* bSubFrameSize :  2 Bytes per frame (16bits) */
        16,                                 /* bBitResolution (16-bits per sample) */
        0x01,                               /* bSamFreqType only one frequency supported */
        AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
        /* 11 byte(126)*/

        /* Endpoint 1 - Standard Descriptor */
        AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
        USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
        AUDIO_OUT_EP,                      /* bEndpointAddress 1 out endpoint */
        USBD_EP_TYPE_ISOC,                 /* bmAttributes */
        AUDIO_PACKET_SZE(USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
        AUDIO_HS_BINTERVAL,                /* bInterval */
        0x00,                              /* bRefresh */
        0x00,                              /* bSynchAddress */
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

        /* ---------------- AS(IN) Interface #2 ---------------- */
        /* Std AS Interface, alt 0 */
        0x09, 0x04,       /* INTERFACE */
        0x02,             /* bInterfaceNumber = 2 */
        0x00,             /* bAlternateSetting = 0 */
        0x00,             /* bNumEndpoints = 0 */
        0x01, 0x02, 0x00, /* AUDIO, AUDIO_STREAMING */
        0x00,
        /* 09 byte(151)*/

        /* Std AS Interface, alt 1 */
        0x09, 0x04,       /* INTERFACE */
        0x02,             /* bInterfaceNumber = 2 */
        0x01,             /* bAlternateSetting = 1 */
        0x01,             /* bNumEndpoints = 1 (IN) */
        0x01, 0x02, 0x00, /* AUDIO, AUDIO_STREAMING */
        0x00,
        /* 09 byte(160)*/

        /* CS AS General (link to OT USB Streaming = 0x13) */
        0x07, 0x24, 0x01, /* bLength=7, CS_INTERFACE, AS_GENERAL */
        0x13,             /* bTerminalLink = 0x13 */
        0x01,             /* bDelay */
        0x01, 0x00,       /* wFormatTag = PCM */
        /* 07 byte(167)*/

        /* Type I Format (2ch, 16bit, 48kHz) */
        0x0B, 0x24, 0x02,                   /* bLength=11, CS_INTERFACE, FORMAT_TYPE */
        0x01,                               /* FORMAT_TYPE_I */
        0x02,                               /* 2ch */
        0x02,                               /* 16-bit (2 bytes) */
        0x10,                               /* 16 bits */
        0x01,                               /* 1 discrete freq */
        AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
        /* 11 byte(178)*/

        /* Std ISO Endpoint (IN) 0x81, Async (0x05), 1ms, 192B */
        0x09, 0x05,                        /* ENDPOINT */
        0x81,                              /* bEndpointAddress = 0x81 (IN) */
        0x05,                              /* bmAttributes = Isochronous | Asynchronous | Data */
        AUDIO_PACKET_SZE(USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
        AUDIO_HS_BINTERVAL,                /* bInterval */
        0x00,                              /* bRefresh */
        0x00,                              /* bSynchAddress */
        /* 09 byte(187)*/

        /* CS ISO Endpoint (IN) */
        0x07, 0x25, 0x01, /* CS_ENDPOINT, EP_GENERAL */
        0x00,             /* bmAttributes */
        0x00,             /* bLockDelayUnits */
        0x00, 0x00        /* wLockDelay */
                          /* 07 byte(194)*/
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

    /* Open EP IN */
    //(void) USBD_LL_OpenEP(pdev, AUDIOInEpAdd, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
    // pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used = 1U;

    haudio->alt_setting = 0U;
    haudio->offset      = AUDIO_OFFSET_UNKNOWN;
    haudio->wr_ptr      = 0U;
    haudio->rd_ptr      = 0U;
    haudio->rd_enable   = 0U;

    /* Initialize the Audio output Hardware layer */
    if (((USBD_AUDIO_ItfTypeDef*) pdev->pUserData[pdev->classId])->Init(USBD_AUDIO_FREQ, AUDIO_DEFAULT_VOLUME, 0U) != 0U)
    {
        return (uint8_t) USBD_FAIL;
    }

    /* === Loopback 初期化 === */
    in_busy      = 0;
    in_alt1      = 0;
    s_have_frame = 0;
    memset(s_silence, 0, sizeof(s_silence));

    /* 念のため IN EP を開いておく（既に開いていればOK） */
    //(void) USBD_LL_OpenEP(pdev, AUDIOInEpAdd, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
    // pdev->ep_in[AUDIOInEpAdd & 0x0F].is_used = 1U;

    dcache_ciinv(s_out_frame, AUDIO_OUT_PACKET);
    /* 最初のOUT受信を明示アーム（テンプレにもあるが確実化） */
    (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, s_out_frame, AUDIO_OUT_PACKET);

    /* Prepare Out endpoint to receive 1st packet */
    //(void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, haudio->buffer, AUDIO_OUT_PACKET);

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

    (void) USBD_LL_CloseEP(pdev, AUDIOInEpAdd);
    pdev->ep_in[AUDIOInEpAdd & 0x0F].is_used = 0U;

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

    return (uint8_t) USBD_OK;
}

// 送出キック：キューに在庫があればそれ、無ければ直前OUT(s_last_out)、最後に無音
#if 0
static void AUDIO_KickIN(USBD_HandleTypeDef* pdev)
{
    // 旧: if (!g_audio_lb.in_alt1 || g_audio_lb.in_busy) return;
    if (!in_alt1 || in_busy)
        return;

    const uint8_t* src;
    // 旧: uint16_t len = AUDIO_PACKET_SZ;
    uint16_t len = AUDIO_IN_PACKET;

    if (g_audio_lb.count)
    {
        src           = g_audio_lb.buf[g_audio_lb.rd];
        len           = g_audio_lb.len[g_audio_lb.rd];
        g_audio_lb.rd = (g_audio_lb.rd + 1U) % LB_Q_DEPTH;
        g_audio_lb.count--;
    }
    else if (s_have_frame)
    {
        src = s_last_out;  // 直前のOUT 1msを返す
        // s_have_frame = 0;
    }
    else
    {
        memset(s_silence, 0, AUDIO_IN_PACKET);
        src = s_silence;
    }

    // 旧: USBD_LL_Transmit(pdev, AUDIO_IN_EP, ...)
    if (USBD_OK == USBD_LL_Transmit(pdev, AUDIOInEpAdd, (uint8_t*) src, len))
    {
        // 旧: g_audio_lb.in_busy = 1U;
        in_busy = 1U;
    }
}
#else
static void AUDIO_KickIN(USBD_HandleTypeDef* pdev)
{
    if (!in_alt1 || in_busy)
        return;

    const uint8_t* src = s_silence;

    if (s_have_frame)
    {
        // 直近OUTがある：まずはそれを使う
        src = s_last_out;
        // ただし “ここ数msずっとゼロ” ならテストトーンを出す
        if (dbg_out_nz == 0 && dbg_out_z > 10)
        {  // 10ms以上無音を観測
            fill_beep_1ms();
            src = s_beep;
        }
    }
    else
    {
        // まだOUT未到達なら、ビープで生存確認（無音よりわかりやすい）
        fill_beep_1ms();
        src = s_beep;
    }

    dcache_clean(src, AUDIO_IN_PACKET);  // ★CPUの書き込みをメモリへ押し出す
    if (USBD_OK == USBD_LL_Transmit(pdev, AUDIOInEpAdd, (uint8_t*) src, AUDIO_IN_PACKET))
    {
        in_busy = 1U;
    }
}

#endif

/**
 * @brief  USBD_AUDIO_Setup
 *         Handle the AUDIO specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
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
#if 0
        	if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                (void) USBD_CtlSendData(pdev, (uint8_t*) &haudio->alt_setting, 1U);
            }
            else
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
            }
#else
        {
            if (pdev->dev_state != USBD_STATE_CONFIGURED)
            {
                USBD_CtlError(pdev, req);
                ret = USBD_FAIL;
                break;
            }

            uint8_t ifn = (uint8_t) req->wIndex;
            uint8_t alt = 0;

            if (ifn == 1)
            {                      // AS(OUT)
                alt = alt_as_out;  // 互換として haudio->alt_setting でも良いが、明示的に
            }
            else if (ifn == 2)
            {  // AS(IN)
                alt = alt_as_in;
            }
            else
            {
                alt = 0;  // AC(IF#0) は Alt=0 固定
            }

            (void) USBD_CtlSendData(pdev, &alt, 1U);
            break;
        }
#endif

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
        {
            uint8_t ifn = (uint8_t) req->wIndex;
            uint8_t alt = (uint8_t) req->wValue;

            if (ifn == 1)
            {  // AS(OUT)
                alt_as_out          = alt;
                g_audio_lb.out_alt1 = (alt == 1);

                if (g_audio_lb.out_alt1)
                {
                    dcache_ciinv(s_out_frame, AUDIO_OUT_PACKET);
                    USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, s_out_frame, AUDIO_OUT_PACKET);
                }
                // 互換のため：従来の GET_INTERFACE が alt_setting を返す実装なので、
                // AS(OUT) に限っては haudio->alt_setting も更新しておく
                ((USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId])->alt_setting = alt;

                USBD_CtlSendStatus(pdev);
                return USBD_OK;
            }

            if (ifn == 2)
            {  // AS(IN)
                alt_as_in = alt;
                in_alt1   = (alt == 1);
                in_busy   = 0;

                if (in_alt1)
                {
                    (void) USBD_LL_OpenEP(pdev, AUDIOInEpAdd, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
                    pdev->ep_in[AUDIOInEpAdd & 0x0F].is_used = 1U;

                    // 初回キック（無音でもOK。次のOUTが来れば s_last_out に置き換わる）
                    if (!in_busy)
                    {
                        USBD_StatusTypeDef status = USBD_LL_Transmit(pdev, AUDIOInEpAdd, s_silence, AUDIO_IN_PACKET);
                        in_busy                   = 1;
                    }
                }
                else
                {
                    (void) USBD_LL_CloseEP(pdev, AUDIOInEpAdd);
                    pdev->ep_in[AUDIOInEpAdd & 0x0F].is_used = 0U;
                }

                USBD_CtlSendStatus(pdev);
                return USBD_OK;
            }
            break;
        }
#endif
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
    UNUSED(pdev);
    // UNUSED(epnum);

    if ((epnum & 0x0F) == (AUDIOInEpAdd & 0x0F))
    {
        in_busy    = 0U;
        busy_ticks = 0U;
        // （任意）完了直後に次フレームをキックすると更に安定：
        AUDIO_KickIN(pdev);
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

    if (in_alt1)
    {
        if (!in_busy)
        {
            AUDIO_KickIN(pdev);
        }
        else
        {
            if (++busy_ticks > 8)
            {  // ← 8ms以上返ってこない
                USBD_LL_CloseEP(pdev, AUDIOInEpAdd);
                pdev->ep_in[AUDIOInEpAdd & 0x0F].is_used = 0U;
                (void) USBD_LL_OpenEP(pdev, AUDIOInEpAdd, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
                pdev->ep_in[AUDIOInEpAdd & 0x0F].is_used = 1U;
                in_busy                                  = 0U;
                busy_ticks                               = 0U;
                AUDIO_KickIN(pdev);  // 再送開始
            }
        }
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
    UNUSED(pdev);
    UNUSED(epnum);

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
#if 0
	USBD_AUDIO_HandleTypeDef* haudio;

    if (pdev->pClassDataCmsit[pdev->classId] == NULL)
    {
        return (uint8_t) USBD_FAIL;
    }

    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    /* Prepare Out endpoint to receive next audio packet */
    (void) USBD_LL_PrepareReceive(pdev, epnum, &haudio->buffer[haudio->wr_ptr], AUDIO_OUT_PACKET);
#else
    // このIFで使うOUT EPだけを扱う
    if ((epnum & 0x0F) == (AUDIOOutEpAdd & 0x0F))
    {
        dcache_ciinv(s_out_frame, AUDIO_OUT_PACKET);  // ★受信直前の Clean+Invalidate
        (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, s_out_frame, AUDIO_OUT_PACKET);
    }
#endif
    return (uint8_t) USBD_OK;
}
/**
 * @brief  USBD_AUDIO_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
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

#if 0
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
#else
    #if 0
    if (epnum == (AUDIOOutEpAdd & 0x0F))
    {
        uint32_t rxlen = USBD_LL_GetRxDataSize(pdev, AUDIOOutEpAdd);
        if (rxlen > AUDIO_OUT_PACKET)
            rxlen = AUDIO_OUT_PACKET;

        /* 直前OUTフレームを保存（不足分はゼロ埋め） */
        memcpy(s_last_out, s_out_frame, rxlen);
        if (rxlen < AUDIO_OUT_PACKET)
        {
            memset(s_last_out + rxlen, 0, AUDIO_OUT_PACKET - rxlen);
        }
        s_have_frame = 1;

        /* 次のOUT受信を必ずアーム */
        (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, s_out_frame, AUDIO_OUT_PACKET);
    }
    #else
    if (epnum == (AUDIOOutEpAdd & 0x0F))
    {
        uint32_t rxlen = USBD_LL_GetRxDataSize(pdev, AUDIOOutEpAdd);
        if (rxlen > AUDIO_OUT_PACKET)
            rxlen = AUDIO_OUT_PACKET;

        dcache_invalidate(s_out_frame, AUDIO_OUT_PACKET);  // ★USBが書いた最新内容をメモリから読む
        // 直前OUTフレームを保存
        memcpy(s_last_out, s_out_frame, rxlen);
        if (rxlen < AUDIO_OUT_PACKET)
        {
            memset(s_last_out + rxlen, 0, AUDIO_OUT_PACKET - rxlen);
        }
        s_have_frame = 1;

        // ★簡易サム（0 なら無音の可能性が高い）
        uint32_t sum = 0;
        for (uint32_t i = 0; i < AUDIO_OUT_PACKET; ++i)
            sum += s_last_out[i];
        dbg_out_sum = sum;
        if (sum == 0)
        {
            dbg_out_z++;
            dbg_out_nz = 0;
        }
        else
        {
            dbg_out_nz++;
            dbg_out_z = 0;
        }

        dcache_ciinv(s_out_frame, AUDIO_OUT_PACKET);
        // 次のOUT受信をアーム
        (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, s_out_frame, AUDIO_OUT_PACKET);
    }
    #endif
#endif

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
