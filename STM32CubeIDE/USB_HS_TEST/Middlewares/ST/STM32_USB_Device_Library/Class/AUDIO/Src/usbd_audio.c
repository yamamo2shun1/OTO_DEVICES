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
    (uint8_t) (((frq * USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME_BYTES) / 1000U) & 0xFFU), (uint8_t) ((((frq * USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME_BYTES) / 1000U) >> 8) & 0xFFU)

#define PKT_BYTES(frq) \
    (uint16_t) (((frq) * USBD_AUDIO_CHANNELS * USBD_AUDIO_SUBFRAME_BYTES) / 1000U)

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

static uint8_t mic_packet[AUDIO_IN_PACKET_MAX];

#ifndef AUDIO_DEBUG
    #define AUDIO_DEBUG 1
#endif
#if AUDIO_DEBUG
    #define AUDDBG(...) printf(__VA_ARGS__)
#else
    #define AUDDBG(...)
#endif

/* 1秒ウィンドウでの統計用 */
static uint32_t g_dbg_win_start_ms = 0;
static uint32_t g_dbg_do_calls_1s  = 0; /* DataOut 呼び出し回数/秒 */
static uint32_t g_dbg_do_bytes_1s  = 0; /* 受信総バイト数/秒 */
static uint32_t g_dbg_do_badlen_1s = 0; /* 期待長と不一致の回数/秒 */
static uint32_t g_dbg_fb_done_1s   = 0; /* FB IN 完了回数/秒（=ホストが受理した回数） */

/* 呼び出し間隔（ms）の観測 */
static uint32_t g_dbg_do_last_ms    = 0;
static uint32_t g_dbg_do_int_min_ms = 0xFFFFFFFFu;
static uint32_t g_dbg_do_int_max_ms = 0;

/* 期待パケット長（Alt切替で更新）と現在のAltを覚えて出力に含める */
static uint16_t g_dbg_pkt_out_sz = 0;
static uint8_t g_dbg_cur_alt     = 0;

/* 追加: FB 送出試行 & Incomplete カウント */
static uint32_t g_dbg_fb_try_1s             = 0;
static uint32_t g_dbg_fb_inc_1s             = 0;
static uint32_t g_dbg_fb_rc_busy_1s         = 0;
static uint32_t g_dbg_fb_rc_fail_1s         = 0;
static uint32_t g_dbg_sof_1s                = 0;
static uint32_t g_dbg_fb_gate_1s            = 0;
static uint32_t g_dbg_fb_gate_block_alt_1s  = 0;
static uint32_t g_dbg_fb_gate_block_busy_1s = 0;
static uint32_t g_dbg_fb_gate_block_used_1s = 0;
/* busy張り付き監視 */
static uint32_t g_dbg_fb_busy_age_ms     = 0; /* 現在の張り付き継続ms */
static uint32_t g_dbg_fb_busy_kick_1s    = 0; /* セーフティ解除回数/秒 */
static uint32_t g_dbg_fb_busy_age_max_1s = 0; /* 1秒内の最大張り付きms */
/* 直近のFB完了(DataIn)時刻(ms)。平常時はこれが毎ms更新される */
static uint32_t g_fb_last_done_ms = 0;
/* ============================ DEBUG METRICS END ============================ */

/* HSのSOF(125us)で毎回送っていたFBを、1ms(=8 microframes)に絞るためのカウンタ */
static uint8_t s_fb_sof_mod8 = 0;
/* HS: FB送出の“早出し位相”。1=125usリード、2=250us… 最大3くらいまでが現実的 */
static uint8_t s_fb_phase_uf = 1; /* 1..3 を想定。0は当フレーム同発で間に合わないことが多い */
/* 初回プライム失敗時に SOF で再挑戦するためのフラグ */
static uint8_t s_fb_need_prime = 0;
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
        0x00,
        /* 1.00 */ /* bcdADC */
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
        0x0C,
        0x24,
        0x02, /* bLength=12, CS_INTERFACE, INPUT_TERMINAL */
        0x11, /* bTerminalID */
        0x02,
        0x06,                /* wTerminalType = 0x0602 (Line Connector). MICなら 0x01,0x02 */
        0x00,                /* bAssocTerminal */
        USBD_AUDIO_CHANNELS, /* bNrChannels = 2 */
        0x03,
        0x00, /* wChannelConfig = L|R */
        0x00, /* iChannelNames */
        0x00, /* iTerminal */
              /* 12 byte(71)*/

        /* Feature Unit (Record) [ID = 0x12, Source = IT(0x11)] */
        0x0A,
        0x24,
        0x06, /* bLength=10, CS_INTERFACE, FEATURE_UNIT */
        0x12, /* bUnitID */
        0x11, /* bSourceID */
        0x01, /* bControlSize = 1 */
        0x01, /* bmaControls(0): Mute on Master */
        0x00, /* bmaControls(1) */
        0x00, /* bmaControls(2) */
        0x00, /* iFeature */
              /* 10 byte(81)*/

        /* Output Terminal (USB Streaming) [ID = 0x13, Source = FU(0x12)] */
        0x09,
        0x24,
        0x03, /* bLength=9, CS_INTERFACE, OUTPUT_TERMINAL */
        0x13, /* bTerminalID */
        0x01,
        0x01, /* wTerminalType = 0x0101 (USB Streaming) */
        0x00, /* bAssocTerminal */
        0x12, /* bSourceID */
        0x00, /* iTerminal */
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
        USBD_AUDIO_SUBFRAME_BYTES,          /* bSubFrameSize :  4 Bytes per frame (32bits) */
        USBD_AUDIO_RES_BITS,                /* bBitResolution (32-bits per sample) */
        0x01,                               /* bSamFreqType only one frequency supported */
        AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
                                            /* 11 byte(126)*/

        /* Endpoint 1 - Standard Descriptor */
        AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
        USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
        AUDIO_OUT_EP,                      /* bEndpointAddress 1 out endpoint */
        0x05,                              /* bmAttributes */
        AUDIO_PACKET_SZE(USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*4(HalfWord)) */
        AUDIO_HS_BINTERVAL,                /* bInterval */
        0x00,                              /* bRefresh */
        AUDIO_FB_EP,                       /* bSynchAddress */
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
        0x00,               /* wMaxPacketSize = 4 bytes (HS:16.16) */
        AUDIO_HS_BINTERVAL, /* bInterval: 1ms (=2^(4-1) µframes) */
        0x00,               /* bRefresh (未使用) */
        0x00,               /* bSynchAddress=0 */

        /* Std AS Interface, alt 2 (96kHz) */
        0x09,
        0x04, /* INTERFACE */
        0x01, /* bInterfaceNumber = 1 */
        0x02, /* bAlternateSetting = 2 */
        0x02, /* bNumEndpoints = 1 (OUT) */
        0x01,
        0x02,
        0x00, /* AUDIO, AUDIO_STREAMING */
        0x00,

        /* CS AS General */
        0x07,
        0x24,
        0x01, /* AS_GENERAL */
        0x01, /* bTerminalLink = 0x01 */
        0x01, /* bDelay */
        0x01,
        0x00, /* PCM */

        /* Type I Format (2ch, 24/32-bit, 96kHz) */
        0x0B,
        0x24,
        0x02, /* FORMAT_TYPE */
        0x01, /* TYPE I */
        USBD_AUDIO_CHANNELS,
        USBD_AUDIO_SUBFRAME_BYTES,
        USBD_AUDIO_RES_BITS,
        0x01, /* 96kHz 1離散 */
        AUDIO_SAMPLE_FREQ(USBD_AUDIO_MAX_FREQ),

        /* Std ISO Endpoint (OUT), wMaxPacketSize=576B(96k*2ch*3B) */
        0x09,
        0x05,
        AUDIO_OUT_EP, /* 0x01 */
        0x05,         /* Iso | Adaptive/Data */
        AUDIO_PACKET_SZE(USBD_AUDIO_MAX_FREQ),
        AUDIO_HS_BINTERVAL, /* 1ms */
        0x00,
        AUDIO_FB_EP,

        /* CS ISO Endpoint */
        0x07,
        0x25,
        0x01,
        0x00,
        0x00,
        0x00,
        0x00,

        /* Feedback IN Endpoint (Standard, 9B) */
        AUDIO_FEEDBACK_ENDPOINT_DESC_SIZE, /* bLength */
        USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType: ENDPOINT */
        AUDIO_FB_EP,                       /* bEndpointAddress: IN */
        0x11,                              /* bmAttributes: Isoch | Usage=Feedback */
        0x03,
        0x00,               /* wMaxPacketSize = 4 bytes (HS:16.16) */
        AUDIO_HS_BINTERVAL, /* bInterval: 1ms (=2^(4-1) µframes) */
        0x00,               /* bRefresh (未使用) */
        0x00,               /* bSynchAddress=0 */

        /* ---------------- AS(IN) Interface #2 ---------------- */
        /* Std AS Interface, alt 0 */
        0x09,
        0x04, /* INTERFACE */
        0x02, /* bInterfaceNumber = 2 */
        0x00, /* bAlternateSetting = 0 */
        0x00, /* bNumEndpoints = 0 */
        0x01,
        0x02,
        0x00, /* AUDIO, AUDIO_STREAMING */
        0x00,
        /* 09 byte(151)*/

        /* Std AS Interface, alt 1 */
        0x09,
        0x04, /* INTERFACE */
        0x02, /* bInterfaceNumber = 2 */
        0x01, /* bAlternateSetting = 1 */
        0x01, /* bNumEndpoints = 1 (IN) */
        0x01,
        0x02,
        0x00, /* AUDIO, AUDIO_STREAMING */
        0x00,
        /* 09 byte(160)*/

        /* CS AS General (link to OT USB Streaming = 0x13) */
        0x07,
        0x24,
        0x01, /* bLength=7, CS_INTERFACE, AS_GENERAL */
        0x13, /* bTerminalLink = 0x13 */
        0x01, /* bDelay */
        0x01,
        0x00, /* wFormatTag = PCM */
        /* 07 byte(167)*/

        /* Type I Format (2ch, 16bit, 48kHz) */
        0x0B,
        0x24,
        0x02,                               /* bLength=11, CS_INTERFACE, FORMAT_TYPE */
        0x01,                               /* FORMAT_TYPE_I */
        USBD_AUDIO_CHANNELS,                /* 2ch */
        USBD_AUDIO_SUBFRAME_BYTES,          /* 32-bit (4 bytes) */
        USBD_AUDIO_RES_BITS,                /* 32 bits */
        0x01,                               /* 1 discrete freq */
        AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
        /* 11 byte(178)*/

        /* Std ISO Endpoint (IN) 0x81, Async (0x05), 1ms, 384B */
        0x09,
        0x05,                              /* ENDPOINT */
        AUDIO_IN_EP,                       /* bEndpointAddress = 0x81 (IN) */
        0x05,                              /* bmAttributes = Isochronous | Asynchronous | Data */
        AUDIO_PACKET_SZE(USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*4(Word)) */
        AUDIO_HS_BINTERVAL,                /* bInterval */
        0x00,                              /* bRefresh */
        0x00,                              /* bSynchAddress */
        /* 09 byte(187)*/

        /* CS ISO Endpoint (IN) */
        0x07,
        0x25,
        0x01, /* CS_ENDPOINT, EP_GENERAL */
        0x00, /* bmAttributes */
        0x00, /* bLockDelayUnits */
        0x00,
        0x00, /* wLockDelay */
        /* 07 byte(194)*/

        /* Std AS Interface, alt 2 (96kHz) */
        0x09,
        0x04, /* INTERFACE */
        0x02, /* bInterfaceNumber = 2 */
        0x02, /* bAlternateSetting = 2 */
        0x02, /* bNumEndpoints = 1 (IN) */
        0x01,
        0x02,
        0x00, /* AUDIO, AUDIO_STREAMING */
        0x00,
        /* CS AS General (link to OT USB Streaming = 0x13) */
        0x07,
        0x24,
        0x01,
        0x13,
        0x01,
        0x01,
        0x00,
        /* Type I Format (96kHz) 1離散 */
        0x0B,
        0x24,
        0x02,
        0x01,
        USBD_AUDIO_CHANNELS,
        USBD_AUDIO_SUBFRAME_BYTES,
        USBD_AUDIO_RES_BITS,
        0x01,
        AUDIO_SAMPLE_FREQ(USBD_AUDIO_MAX_FREQ),
        /* Std ISO Endpoint (IN), wMaxPacketSize=576B */
        0x09,
        0x05,
        AUDIO_IN_EP,
        0x05,
        AUDIO_PACKET_SZE(USBD_AUDIO_MAX_FREQ),
        AUDIO_HS_BINTERVAL,
        0x00,
        0x00,
        /* CS ISO Endpoint */
        0x07,
        0x25,
        0x01,
        0x00,
        0x00,
        0x00,
        0x00,
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

static uint8_t AUDIOOutEpAdd      = AUDIO_OUT_EP;
static uint8_t AUDIOInEpAdd       = AUDIO_IN_EP;
static uint8_t AUDIOFbEpAdd       = AUDIO_FB_EP;
static volatile uint8_t s_fb_busy = 0;

USBD_AUDIO_LB_CTX g_audio_lb;
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
    /* 既定=48kHz/288B で初期化（Alt切替時に Close→Open で更新） */
    haudio->out_srate  = USBD_AUDIO_FREQ;
    haudio->in_srate   = USBD_AUDIO_FREQ;
    haudio->pkt_out_sz = PKT_BYTES(USBD_AUDIO_FREQ);
    haudio->pkt_in_sz  = PKT_BYTES(USBD_AUDIO_FREQ);

    /* Open EP OUT （Alt0中でも HAL 的にはOpenしてOK。MPSは既定で開始） */
    (void) USBD_LL_OpenEP(pdev, AUDIOOutEpAdd, USBD_EP_TYPE_ISOC, haudio->pkt_out_sz);

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
    (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, &haudio->buffer[0], haudio->pkt_out_sz);

    AUDIO_RxQ_Flush();

#if AUDIO_DEBUG
    /* 計測の初期化 */
    g_dbg_win_start_ms  = HAL_GetTick();
    g_dbg_do_calls_1s   = 0;
    g_dbg_do_bytes_1s   = 0;
    g_dbg_do_badlen_1s  = 0;
    g_dbg_fb_done_1s    = 0;
    g_dbg_do_last_ms    = 0;
    g_dbg_do_int_min_ms = 0xFFFFFFFFu;
    g_dbg_do_int_max_ms = 0;
    g_dbg_pkt_out_sz    = 0;
    g_dbg_cur_alt       = 0;
    AUDDBG("[AUDIO] Init: dev_speed=%d (0:HS,1:FS)\n", pdev->dev_speed);
#endif

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

    AUDIO_RxQ_Flush();

    return (uint8_t) USBD_OK;
}

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

    /* デバッグ用に現在のAltを記録（1=48k, 2=96k, 0=停止）*/
    #if AUDIO_DEBUG
            g_dbg_cur_alt = alt;
    #endif

            haudio->alt_setting = alt;  // 既存の動作を踏襲

            /* ---- Speaker: Interface #1 (OUT) ---- */
            if (ifnum == 0x01)
            {
                if (alt == 1U || alt == 2U)
                {
                    /* Alt1=48kHz / Alt2=96kHz */
                    const uint16_t mps  = (alt == 2U) ? (AUDIO_OUT_PACKET * 2U) : AUDIO_OUT_PACKET;
                    g_audio_lb.out_alt1 = (alt == 1U);
                    g_audio_lb.out_alt  = alt;
                    haudio->pkt_out_sz  = mps;
                    haudio->out_srate   = (alt == 2U) ? 96000u : 48000u;

                    /* 計測：期待パケット長を更新＆ログ */
    #if AUDIO_DEBUG
                    g_dbg_pkt_out_sz = mps;
                    AUDDBG("[AUDIO] SET_INTERFACE if=%u OUT alt=%u pkt_out_sz=%u\n", (unsigned) ifnum, (unsigned) alt, (unsigned) g_dbg_pkt_out_sz);
    #endif

                    AUDIO_RxQ_Flush();

                    /* リング初期化 */
                    haudio->wr_ptr    = 0U;
                    haudio->rd_ptr    = 0U;
                    haudio->rd_enable = 0U;
                    haudio->offset    = AUDIO_OFFSET_UNKNOWN;

                    /* 1) 先に FB EP を確実に Open（ホストが先にFBを取りにきても対応できるように） */
                    (void) USBD_LL_OpenEP(pdev, AUDIOFbEpAdd, USBD_EP_TYPE_ISOC, 3);
                    pdev->ep_in[AUDIOFbEpAdd & 0x0FU].bInterval =
                        (pdev->dev_speed == USBD_SPEED_HIGH) ? AUDIO_HS_BINTERVAL : AUDIO_FS_BINTERVAL;
                    pdev->ep_in[AUDIOFbEpAdd & 0xFU].is_used = 1U;
                    /* ---- FB 初回プライム ---- */
                    {
                        uint32_t ff10_14 = AUDIO_GetFeedback_10_14(); /* 10.14 / 1ms */
                        uint8_t fb[3]    = {(uint8_t) ff10_14, (uint8_t) (ff10_14 >> 8), (uint8_t) (ff10_14 >> 16)};
                        if (USBD_LL_Transmit(pdev, AUDIOFbEpAdd, fb, 3) == USBD_OK)
                        {
                            s_fb_busy = 1;
    #if AUDIO_DEBUG
                            g_dbg_fb_try_1s++;
    #endif
                        }
                        else
                        {
                            /* 失敗したら SOF で再挑戦する */
                            s_fb_busy       = 0;
                            s_fb_need_prime = 1;
                        }
                        s_fb_sof_mod8 = 0; /* 位相初期化 */
                    }

                    /* 2) 次に OUT EP を Open→受信開始 */
                    USBD_LL_FlushEP(pdev, AUDIOOutEpAdd);
                    (void) USBD_LL_CloseEP(pdev, AUDIOOutEpAdd);
                    (void) USBD_LL_OpenEP(pdev, AUDIOOutEpAdd, USBD_EP_TYPE_ISOC, mps);
                    pdev->ep_out[AUDIOOutEpAdd & 0xFU].is_used = 1U;
                    (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, &haudio->buffer[0], mps);
                }
                else
                {
                    /* Alt=0 は停止（Flush のみ） */
                    g_audio_lb.out_alt = 0;
                    USBD_LL_FlushEP(pdev, AUDIOOutEpAdd);
                    AUDIO_RxQ_Flush();

                    (void) USBD_LL_CloseEP(pdev, AUDIOFbEpAdd);
                    pdev->ep_in[AUDIOFbEpAdd & 0x0FU].bInterval = 0;
                    pdev->ep_in[AUDIOFbEpAdd & 0xFU].is_used    = 0U;
                    s_fb_busy                                   = 0;
                }
            }

            /* ---- Mic: Interface #2 ---- */
            if (ifnum == 0x02)
            {
    #ifdef USE_USBD_COMPOSITE
                AUDIOInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_ISOC, (uint8_t) pdev->classId);
    #endif

                if (alt == 1U || alt == 2U)
                {
                    /* Alt1=48kHz / Alt2=96kHz（MPS=288/576B） */
                    const uint16_t mps = (alt == 2U) ? (AUDIO_IN_PACKET * 2U) : AUDIO_IN_PACKET;
                    g_audio_lb.in_alt1 = (alt == 1U);
                    g_audio_lb.in_alt  = alt;
                    haudio->pkt_in_sz  = mps;
                    haudio->in_srate   = (alt == 2U) ? 96000u : 48000u;
                    if (pdev->dev_speed == USBD_SPEED_HIGH)
                    {
                        pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
                    }
                    else
                    {
                        pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
                    }
                    USBD_LL_FlushEP(pdev, AUDIOInEpAdd);
                    (void) USBD_LL_CloseEP(pdev, AUDIOInEpAdd);
                    (void) USBD_LL_OpenEP(pdev, AUDIOInEpAdd, USBD_EP_TYPE_ISOC, mps);
                    pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used = 1U;
                    haudio->mic_prime                        = 1U; /* 初回送信は SOF 側で実行 */
                }
                else
                {
                    /* Alt=0: IN EP停止 */
                    g_audio_lb.in_alt = 0;
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
    USBD_AUDIO_HandleTypeDef* haudio =
        (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];

    if ((epnum & 0x0F) == (AUDIOInEpAdd & 0x0F))
    {
        (void) AUDIO_Mic_GetPacket(mic_packet, haudio->pkt_in_sz);
        (void) USBD_LL_Transmit(pdev, AUDIOInEpAdd, mic_packet, haudio->pkt_in_sz);

        // printf("data in!\n");
    }

    if ((epnum & 0x0F) == (AUDIOFbEpAdd & 0x0F))
    {
        /* ---- FB 完了：すぐ次の1発をキュー（先行キューでIncompleteを根絶） ---- */
        s_fb_busy = 0;
#if AUDIO_DEBUG
        g_dbg_fb_done_1s++; /* 1msごとに増えるのが理想 */
#endif
        g_fb_last_done_ms = HAL_GetTick(); /* ★ 直近完了時刻を記録 */
        s_fb_need_prime   = 0;             /* ★ 復帰済みフラグを下ろす */

        /* 直ちに次の値を送る（常に1個先行） */
        uint32_t ff10_14 = AUDIO_GetFeedback_10_14();
        uint8_t fb[3]    = {(uint8_t) ff10_14, (uint8_t) (ff10_14 >> 8), (uint8_t) (ff10_14 >> 16)};
        if (USBD_LL_Transmit(pdev, AUDIOFbEpAdd, fb, 3) == USBD_OK)
        {
            s_fb_busy = 1;
#if AUDIO_DEBUG
            g_dbg_fb_try_1s++;
#endif
        }
        else
        {
            /* BUSY/FAILなら、次のSOF 1msで再挑戦（SOF側でguard） */
        }
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
    USBD_AUDIO_HandleTypeDef* haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassDataCmsit[pdev->classId];
    if (!haudio)
    {
        return (uint8_t) USBD_OK;
    }

#if AUDIO_DEBUG
    g_dbg_sof_1s++;
#endif

    /* alt=1 が選択され、まだ初回送信していない？ */
    if (haudio->mic_prime)
    {
        /* いったんフラッシュして、フレーム境界(1ms)に揃えて送る */
        USBD_LL_FlushEP(pdev, AUDIOInEpAdd);
        AUDIO_Mic_GetPacket(mic_packet, haudio->pkt_in_sz);
        USBD_LL_Transmit(pdev, AUDIOInEpAdd, mic_packet, haudio->pkt_in_sz);
        haudio->mic_prime = 0;

        printf("first transmit.\n");
    }

    /* ---- FB フォールバック送出 ----
       ・「要プライム(s_fb_need_prime)」の間のみ送る
       ・HSは uSOF(125us) を mod8 カウントし、1msあたり厳密に 1 回だけ位相一致で送る
       ・平常時（DataInが回っている間）は完全に沈黙
    */
    if ((g_audio_lb.out_alt == 1U || g_audio_lb.out_alt == 2U) &&
        (pdev->ep_in[AUDIOFbEpAdd & 0x0FU].is_used != 0U))
    {
        /* ★要プライム時以外は SOF 側は何もしない */
        if (!(s_fb_need_prime && !s_fb_busy))
        {
            /* 平常時は完全に沈黙。busyは一切いじらない（保持） */
            g_dbg_fb_busy_age_ms = 0;
            return (uint8_t) USBD_OK;
        }

        /* ★HS: 1msに1回だけ（mod8一致時のみ）送る。phaseは“早出しuSOF数” */
        if (pdev->dev_speed == USBD_SPEED_HIGH)
        {
            s_fb_sof_mod8 = (uint8_t) ((s_fb_sof_mod8 + 1) & 0x07); /* 0..7 */
            /* 例: s_fb_phase_uf=2(=250usリード) → 一致条件は (s_fb_sof_mod8 == (8 - s_fb_phase_uf) & 7) */
            uint8_t gate_uf = (uint8_t) ((8u - s_fb_phase_uf) & 0x07u);
            if (s_fb_sof_mod8 != gate_uf)
            {
                return (uint8_t) USBD_OK; /* このuSOFでは送らない */
            }
        }

        if (!s_fb_busy)
        {
            /* 1msゲートを通過 */
#if AUDIO_DEBUG
            g_dbg_fb_gate_1s++;
#endif

            uint32_t ff10_14      = AUDIO_GetFeedback_10_14();
            uint8_t fb[3]         = {(uint8_t) ff10_14, (uint8_t) (ff10_14 >> 8), (uint8_t) (ff10_14 >> 16)};
            USBD_StatusTypeDef rc = USBD_LL_Transmit(pdev, AUDIOFbEpAdd, fb, 3);
            if (rc == USBD_OK)
            {
                s_fb_busy       = 1;
                s_fb_need_prime = 0;
#if AUDIO_DEBUG
                g_dbg_fb_try_1s++; /* 実送信成功のみ try を加算 */
#endif
            }
            /* BUSY/FAIL の場合は次の1ms（次の位相uSOF）で再試行（s_fb_need_prime維持） */
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
    uint32_t BufferSize        = AUDIO_TOTAL_BUF_SIZE / 2U;
    const uint32_t frame_bytes = (uint32_t) USBD_AUDIO_CHANNELS * (uint32_t) USBD_AUDIO_SUBFRAME_BYTES;

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
        if ((haudio->rd_ptr - haudio->wr_ptr) < haudio->pkt_out_sz)
        {
            // BufferSize += frame_bytes;
        }
        else
        {
            if ((haudio->rd_ptr - haudio->wr_ptr) > (AUDIO_TOTAL_BUF_SIZE - haudio->pkt_out_sz))
            {
                // BufferSize -= frame_bytes;
            }
        }
    }
    else
    {
        if ((haudio->wr_ptr - haudio->rd_ptr) < haudio->pkt_out_sz)
        {
            // BufferSize -= frame_bytes;
        }
        else
        {
            if ((haudio->wr_ptr - haudio->rd_ptr) > (AUDIO_TOTAL_BUF_SIZE - haudio->pkt_out_sz))
            {
                // BufferSize += frame_bytes;
            }
        }
    }

    if (BufferSize > (AUDIO_TOTAL_BUF_SIZE / 2U + haudio->pkt_out_sz))
    {
        BufferSize = AUDIO_TOTAL_BUF_SIZE / 2U + haudio->pkt_out_sz;
    }
    if (BufferSize < (AUDIO_TOTAL_BUF_SIZE / 2U - haudio->pkt_out_sz))
    {
        BufferSize = AUDIO_TOTAL_BUF_SIZE / 2U - haudio->pkt_out_sz;
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
        (void) USBD_LL_Transmit(pdev, AUDIOInEpAdd, mic_packet, ((USBD_AUDIO_HandleTypeDef*) pdev->pClassData)->pkt_in_sz);

        // printf("incomplete...\n");
    }
    else if ((epnum & 0x0F) == (AUDIOFbEpAdd & 0x0F))
    {
        /* FB Iso IN Incomplete:
           ★直ちに“要プライム”に遷移（SOF/uSOF側が125usおきに再試行） */
        s_fb_busy       = 0;
        s_fb_need_prime = 1;
    }

#if AUDIO_DEBUG
    if ((0x80U | epnum) == AUDIOFbEpAdd)
    {
        g_dbg_fb_inc_1s++;
    }
#endif

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
    (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, &haudio->buffer[haudio->wr_ptr], haudio->pkt_out_sz);

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

    if ((epnum & 0x0F) == (AUDIOOutEpAdd & 0x0F))
    {
        /* Get received data packet length */
        PacketSize = (uint16_t) USBD_LL_GetRxDataSize(pdev, epnum);

        /* ----------------------------- DEBUG START ------------------------------ */
#if AUDIO_DEBUG
        uint32_t now_ms = HAL_GetTick();
        /* 呼び出し間隔の min/max(ms) を更新 */
        if (g_dbg_do_last_ms != 0)
        {
            uint32_t dt = now_ms - g_dbg_do_last_ms;
            if (dt < g_dbg_do_int_min_ms)
                g_dbg_do_int_min_ms = dt;
            if (dt > g_dbg_do_int_max_ms)
                g_dbg_do_int_max_ms = dt;
        }
        g_dbg_do_last_ms = now_ms;

        /* 受信統計の更新 */
        g_dbg_do_calls_1s++;
        g_dbg_do_bytes_1s += PacketSize;

        /* 期待長（haudio->pkt_out_sz）と不一致なら記録＆即時ログ */
        if (haudio->pkt_out_sz != 0 && PacketSize != haudio->pkt_out_sz)
        {
            g_dbg_do_badlen_1s++;
            AUDDBG("[AUDIO][DataOut] len=%u (exp=%u) alt=%u\n", (unsigned) PacketSize, (unsigned) haudio->pkt_out_sz, (unsigned) g_dbg_cur_alt);
        }

        /* 1秒ごとにサマリを出力 */
        if ((now_ms - g_dbg_win_start_ms) >= 1000U)
        {
            /* 期待長は直近の haudio->pkt_out_sz（Alt変更に追従） */
            g_dbg_pkt_out_sz = haudio->pkt_out_sz;
            AUDDBG("[AUDIO][1s] DataOut calls=%lu bytes=%lu pkt=%u int[min/max]=%lums/%lums "
                   "badlen=%lu SOF=%lu FBgate=%lu FBtry=%lu FBinc=%lu FBdone=%lu rcBUSY=%lu rcFAIL=%lu "
                   "blk_alt=%lu blk_busy=%lu blk_used=%lu FBbusyKick=%lu FBbusyAgeMax=%lu alt=%u\n",
                   (unsigned long) g_dbg_do_calls_1s, (unsigned long) g_dbg_do_bytes_1s, (unsigned) g_dbg_pkt_out_sz, (unsigned long) (g_dbg_do_int_min_ms == 0xFFFFFFFFu ? 0 : g_dbg_do_int_min_ms), (unsigned long) g_dbg_do_int_max_ms, (unsigned long) g_dbg_do_badlen_1s, (unsigned long) g_dbg_sof_1s, (unsigned long) g_dbg_fb_gate_1s, (unsigned long) g_dbg_fb_try_1s, (unsigned long) g_dbg_fb_inc_1s, (unsigned long) g_dbg_fb_done_1s, (unsigned long) g_dbg_fb_rc_busy_1s, (unsigned long) g_dbg_fb_rc_fail_1s, (unsigned long) g_dbg_fb_gate_block_alt_1s, (unsigned long) g_dbg_fb_gate_block_busy_1s, (unsigned long) g_dbg_fb_gate_block_used_1s, (unsigned long) g_dbg_fb_busy_kick_1s, (unsigned long) g_dbg_fb_busy_age_max_1s, (unsigned) g_dbg_cur_alt);

            /* リセット */
            g_dbg_win_start_ms          = now_ms;
            g_dbg_do_calls_1s           = 0;
            g_dbg_do_bytes_1s           = 0;
            g_dbg_do_badlen_1s          = 0;
            g_dbg_sof_1s                = 0;
            g_dbg_fb_gate_1s            = 0;
            g_dbg_fb_try_1s             = 0;
            g_dbg_fb_inc_1s             = 0;
            g_dbg_fb_done_1s            = 0;
            g_dbg_fb_rc_busy_1s         = 0;
            g_dbg_fb_rc_fail_1s         = 0;
            g_dbg_fb_gate_block_alt_1s  = 0;
            g_dbg_fb_gate_block_busy_1s = 0;
            g_dbg_fb_gate_block_used_1s = 0;
            g_dbg_fb_busy_kick_1s       = 0;
            g_dbg_fb_busy_age_max_1s    = 0;
            g_dbg_do_int_min_ms         = 0xFFFFFFFFu;
            g_dbg_do_int_max_ms         = 0;

            /* ---- 自動位相調整（HSのみ） ----
               1秒内に FBinc が非常に多く、FBdone が 0 の場合は “さらに早出し” する。
               例：位相ズレで常時遅延しているときに有効。上限は 3（=375usリード） */
            if (pdev->dev_speed == USBD_SPEED_HIGH)
            {
                if ((g_dbg_fb_inc_1s > 800U) && (g_dbg_fb_done_1s == 0U) && (s_fb_phase_uf < 3U))
                {
                    s_fb_phase_uf++;
                    AUDDBG("[AUDIO][fb] shift phase earlier: %u uSOF lead\n", s_fb_phase_uf);
                }
            }
        }
#endif
        /* ------------------------------ DEBUG END ------------------------------- */

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

        /* Prepare Out endpoint to receive next audio packet (always use current pkt_out_sz) */
        (void) USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, &haudio->buffer[haudio->wr_ptr], haudio->pkt_out_sz);
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
