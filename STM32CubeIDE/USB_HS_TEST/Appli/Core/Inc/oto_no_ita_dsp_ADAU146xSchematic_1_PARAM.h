/* 
 * File:	C:\Users\shuni\OTO_DEVICES\SigmaDSP\exported_code\oto_no_ita_dsp_ADAU146xSchematic_1_PARAM.h
 * Created:	Thursday, 06 November 2025 11:05 AM
 * Description:	ADAU146xSchematic_1 parameter RAM definitions.
 * 
 * This software is distributed in the hope that it will be useful, 
 * but is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR 
 * CONDITIONS OF ANY KIND, without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * 
 * This software may only be used to program products purchased from 
 * Analog Devices for incorporation by you into audio products that 
 * are intended for resale to audio product end users.This software 
 * may not be distributed whole or in any part to third parties. 
 * 
 * Copyright © 2025 Analog Devices, Inc. All rights reserved.
 */
#ifndef __OTO_NO_ITA_DSP_ADAU146XSCHEMATIC_1_PARAM_H__
#define __OTO_NO_ITA_DSP_ADAU146XSCHEMATIC_1_PARAM_H__

/* Module Safeload - Schematic Params */
#define MOD_SAFELOAD_COUNT                          8
#define MOD_SAFELOAD_SCHEMATIC                      "ADAU146xSchematic_1"
#define MOD_SAFELOAD_DATA_SAFELOAD0_ADDR            24576
#define MOD_SAFELOAD_DATA_SAFELOAD0_MEM_PAGE        0
#define MOD_SAFELOAD_DATA_SAFELOAD1_ADDR            24577
#define MOD_SAFELOAD_DATA_SAFELOAD1_MEM_PAGE        0
#define MOD_SAFELOAD_DATA_SAFELOAD2_ADDR            24578
#define MOD_SAFELOAD_DATA_SAFELOAD2_MEM_PAGE        0
#define MOD_SAFELOAD_DATA_SAFELOAD3_ADDR            24579
#define MOD_SAFELOAD_DATA_SAFELOAD3_MEM_PAGE        0
#define MOD_SAFELOAD_DATA_SAFELOAD4_ADDR            24580
#define MOD_SAFELOAD_DATA_SAFELOAD4_MEM_PAGE        0
#define MOD_SAFELOAD_ADDR_SAFELOAD_ADDR             24581
#define MOD_SAFELOAD_ADDR_SAFELOAD_MEM_PAGE         0
#define MOD_SAFELOAD_NUM_SAFELOAD_LOWER_ADDR        24582
#define MOD_SAFELOAD_NUM_SAFELOAD_LOWER_MEM_PAGE    0
#define MOD_SAFELOAD_NUM_SAFELOAD_UPPER_ADDR        24583
#define MOD_SAFELOAD_NUM_SAFELOAD_UPPER_MEM_PAGE    0




/* Module SingleVolumeControl_1 - Single Volume Control */
#define MOD_SINGLEVOLUMECONTROL_1_COUNT               1
#define MOD_SINGLEVOLUMECONTROL_1_SCHEMATIC           "ADAU146xSchematic_1"
#define MOD_SINGLEVOLUMECONTROL_1_GAIN_ADDR           22
#define MOD_SINGLEVOLUMECONTROL_1_GAIN_FIXPT          0x01000000
#define MOD_SINGLEVOLUMECONTROL_1_GAIN_VALUE          SIGMASTUDIOTYPE_8_24_CONVERT(1)
#define MOD_SINGLEVOLUMECONTROL_1_GAIN_TYPE           SIGMASTUDIOTYPE_8_24
#define MOD_SINGLEVOLUMECONTROL_1_GAIN_DATA_MEMORY    "DM0"
#define MOD_SINGLEVOLUMECONTROL_1_GAIN_MEM_PAGE       0
#define MOD_SINGLEVOLUMECONTROL_1_GAIN_DATA_MEMORY    "DM0"
#define MOD_SINGLEVOLUMECONTROL_1_GAIN_MEM_PAGE       0






/* Module SingleVolumeControl_2 - Single Volume Control */
#define MOD_SINGLEVOLUMECONTROL_2_COUNT               1
#define MOD_SINGLEVOLUMECONTROL_2_SCHEMATIC           "ADAU146xSchematic_1"
#define MOD_SINGLEVOLUMECONTROL_2_GAIN_ADDR           21
#define MOD_SINGLEVOLUMECONTROL_2_GAIN_FIXPT          0x01000000
#define MOD_SINGLEVOLUMECONTROL_2_GAIN_VALUE          SIGMASTUDIOTYPE_8_24_CONVERT(1)
#define MOD_SINGLEVOLUMECONTROL_2_GAIN_TYPE           SIGMASTUDIOTYPE_8_24
#define MOD_SINGLEVOLUMECONTROL_2_GAIN_DATA_MEMORY    "DM0"
#define MOD_SINGLEVOLUMECONTROL_2_GAIN_MEM_PAGE       0
#define MOD_SINGLEVOLUMECONTROL_2_GAIN_DATA_MEMORY    "DM0"
#define MOD_SINGLEVOLUMECONTROL_2_GAIN_MEM_PAGE       0

/* Module SingleSlewExtVol_0 - External Volume Control (Slew) */
#define MOD_SINGLESLEWEXTVOL_0_COUNT                    1
#define MOD_SINGLESLEWEXTVOL_0_SCHEMATIC                "ADAU146xSchematic_1"
#define MOD_SINGLESLEWEXTVOL_0_SLEW_MODE_ADDR           24585
#define MOD_SINGLESLEWEXTVOL_0_SLEW_MODE_FIXPT          0x00002080
#define MOD_SINGLESLEWEXTVOL_0_SLEW_MODE_VALUE          SIGMASTUDIOTYPE_INTEGER_CONVERT(8320)
#define MOD_SINGLESLEWEXTVOL_0_SLEW_MODE_TYPE           SIGMASTUDIOTYPE_INTEGER
#define MOD_SINGLESLEWEXTVOL_0_SLEW_MODE_DATA_MEMORY    "DM1"
#define MOD_SINGLESLEWEXTVOL_0_SLEW_MODE_MEM_PAGE       0
#define MOD_SINGLESLEWEXTVOL_0_SLEW_MODE_DATA_MEMORY    "DM1"
#define MOD_SINGLESLEWEXTVOL_0_SLEW_MODE_MEM_PAGE       0

/* Module DCInput_0 - DC Input */
#define MOD_DCINPUT_0_COUNT                  1
#define MOD_DCINPUT_0_SCHEMATIC              "ADAU146xSchematic_1"
#define MOD_DCINPUT_0_DCVALUE_ADDR           20
#define MOD_DCINPUT_0_DCVALUE_FIXPT          0x01000000
#define MOD_DCINPUT_0_DCVALUE_VALUE          SIGMASTUDIOTYPE_8_24_CONVERT(1)
#define MOD_DCINPUT_0_DCVALUE_TYPE           SIGMASTUDIOTYPE_8_24
#define MOD_DCINPUT_0_DCVALUE_DATA_MEMORY    "DM0"
#define MOD_DCINPUT_0_DCVALUE_MEM_PAGE       0
#define MOD_DCINPUT_0_DCVALUE_DATA_MEMORY    "DM0"
#define MOD_DCINPUT_0_DCVALUE_MEM_PAGE       0

#endif
