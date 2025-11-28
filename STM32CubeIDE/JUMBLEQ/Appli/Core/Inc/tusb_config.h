/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jerzy Kasenberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "usb_descriptors.h"

#include "my_log.h"

//--------------------------------------------------------------------+
// Board Specific Configuration
//--------------------------------------------------------------------+
#define CFG_TUSB_MCU              OPT_MCU_STM32H7RS
#define CFG_TUSB_OS               OPT_OS_NONE
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_HIGH_SPEED
#define BOARD_TUD_MAX_SPEED       OPT_MODE_HIGH_SPEED
#define BOARD_DEVICE_RHPORT_NUM   1
#define BOARD_TUD_RHPORT          1
#define CFG_TUSB_RHPORT1_MODE     (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#define CFG_TUSB_DEBUG            0
#define CFG_TUSB_DEBUG_PRINTF     my_printf

    // #define TUD_AUDIO_PREFER_RING_BUFFER 1
    // #define CFG_TUD_DWC2_SLAVE_ENABLE    0
    // #define CFG_TUD_DWC2_DMA_ENABLE      1
    // #define CFG_TUD_MEM_DCACHE_ENABLE    1
    // #define CFG_TUSB_MEM_SECTION         __attribute__((section(".rtt")))
    //  #define CFG_TUSB_MEM_ALIGN   __attribute__((aligned(32)))

#define USBD_AUDIO_FREQ     48000U
#define USBD_AUDIO_FREQ_96K 96000U

// RHPort number used for device can be defined by board.mk, default to port 0
#ifndef BOARD_TUD_RHPORT
    #define BOARD_TUD_RHPORT 0
#endif

// RHPort max operational speed can defined by board.mk
#ifndef BOARD_TUD_MAX_SPEED
    #define BOARD_TUD_MAX_SPEED OPT_MODE_DEFAULT_SPEED
#endif

//--------------------------------------------------------------------
// Common Configuration
//--------------------------------------------------------------------

// defined by compiler flags for flexibility
#ifndef CFG_TUSB_MCU
    #error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
    #define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
    #define CFG_TUSB_DEBUG 0
#endif

// Enable Device stack
#define CFG_TUD_ENABLED 1

// Default is max speed that hardware controller could support with on-chip PHY
#define CFG_TUD_MAX_SPEED BOARD_TUD_MAX_SPEED

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
    #define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
    #define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

    //--------------------------------------------------------------------
    // DEVICE CONFIGURATION
    //--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
    #define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC    0
#define CFG_TUD_MSC    0
#define CFG_TUD_HID    0
#define CFG_TUD_MIDI   0
#define CFG_TUD_AUDIO  1
#define CFG_TUD_VENDOR 0

//--------------------------------------------------------------------
// AUDIO CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------

// Allow volume controlled by on-baord button
#define CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP 1

#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN TUD_AUDIO_HEADSET_STEREO_DESC_LEN

// How many formats are used, need to adjust USB descriptor if changed
#define CFG_TUD_AUDIO_FUNC_1_N_FORMATS 1

// Audio format type I specifications
/* 24bit/48kHz is the best quality for headset or 24bit/96kHz for 2ch speaker,
   high-speed is needed beyond this */
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE_HS USBD_AUDIO_FREQ_96K
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX      2
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX      2

// 16bit in 16bit slots
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX 4
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_TX         24
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX 4
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX         24

// EP and buffer size - for isochronous EP´s, the buffer and EP size are equal (different sizes would not make sense)
#define CFG_TUD_AUDIO_ENABLE_EP_IN 1

#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_HS TUD_AUDIO_EP_SIZE(true, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE_HS, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)

#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX    CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_HS        // Maximum EP IN size for all AS alternate settings used
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ 32 * CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX  // Example read FIFO every 1ms, so it should be 8 times larger for HS device

// EP and buffer size - for isochronous EP´s, the buffer and EP size are equal (different sizes would not make sense)
#define CFG_TUD_AUDIO_ENABLE_EP_OUT 1

// Enable feedback EP
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP 1

#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_HS TUD_AUDIO_EP_SIZE(true, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE_HS, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)

#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX    CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_HS        // Maximum EP IN size for all AS alternate settings used
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ 32 * CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX  // Example read FIFO every 1ms, so it should be 8 times larger for HS device

// Size of control request buffer
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ 64

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
