/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usbpd_pwr_if.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of usbpd_pw_if.h.
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

#ifndef __USBPD_PW_IF_H
#define __USBPD_PW_IF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"

/** @addtogroup STM32_USBPD_APPLICATION
  * @{
  */

/** @addtogroup STM32_USBPD_APPLICATION_POWER_IF
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/** @defgroup USBPD_USER_PWR_IF_Exported_Macros USBPD PWR IF Exported Macros
  * @{
  */

/* enumeration of the different power status available for VBUS */
typedef enum
{
  USBPD_PWR_BELOWVSAFE0V,
  USBPD_PWR_VSAFE5V,
  USBPD_PWR_SNKDETACH
} USBPD_VBUSPOWER_STATUS;

/* Enumeration of the different errors detected by power IF */
typedef enum{
  USBPD_PWR_IF_OTHER        = 0U,
  USBPD_PWR_IF_NMI          = 2U,
  USBPD_PWR_IF_HARD_FAULT   = 3U,
  USBPD_PWR_IF_OVER_CURRENT = 4U,
} USBPD_PWR_IF_ERROR;

/* Macros used to convert values into PDO representation */
#define PWR_V_20MV(_V_)        ((uint16_t)(( (_V_) * 1000) / 20))   /* From Volt to 20mV multiples      */
#define PWR_V_50MV(_V_)        ((uint16_t)(( (_V_) * 1000) / 50))   /* From Volt to 50mV multiples      */
#define PWR_V_100MV(_V_)       ((uint16_t)(( (_V_) * 1000) / 100))  /* From Volt to 100mV multiples     */
#define PWR_A_10MA(_A_)        ((uint16_t)(( (_A_) * 1000) / 10))   /* From Ampere to 10mA multiples    */
#define PWR_A_50MA(_A_)        ((uint16_t)(( (_A_) * 1000) / 50))   /* From Ampere to 50mA multiples    */
#define PWR_W(_W_)             ((uint16_t)(( (_W_) * 1000) / 250))  /* From Watt to 250mW multiples     */

/* Macros used to get values from PDO representation */
#define PWR_DECODE_50MV(_Value_)           ((uint16_t)(((_Value_) * 50)))     /* From 50mV multiples to mV        */
#define PWR_DECODE_100MV(_Value_)          ((uint16_t)(((_Value_) * 100)))    /* From 100mV multiples to mV       */
#define PWR_DECODE_10MA(_Value_)           ((uint16_t)(((_Value_) * 10)))     /* From 10mA multiples to mA        */
#define PWR_DECODE_50MA(_Value_)           ((uint16_t)(((_Value_) * 50)))     /* From 50mA multiples to mA        */
#define PWR_DECODE_MW(_Value_)             ((uint16_t)(((_Value_) * 250)))    /* From 250mW multiples to mW       */
/* Macros used to describe the parameters in a PDO list */
/* ---------------------------- FIXED --------------------------- */

/* Set voltage in mV in SRC Fixed PDO */
#define USBPD_PDO_SRC_FIXED_SET_VOLTAGE(_MV_)        (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)        \
                                                      & (USBPD_PDO_SRC_FIXED_VOLTAGE_Msk))

/* Set max current in mA in SRC Fixed PDO */
#define USBPD_PDO_SRC_FIXED_SET_MAX_CURRENT(_MA_)    (((PWR_A_10MA((_MA_)  / 1000.0))             \
                                                       << USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos)    \
                                                      & (USBPD_PDO_SRC_FIXED_MAX_CURRENT_Msk))

/* Set voltage in mV in SNK Fixed PDO */
#define USBPD_PDO_SNK_FIXED_SET_VOLTAGE(_MV_)        (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SNK_FIXED_VOLTAGE_Pos)        \
                                                      & (USBPD_PDO_SNK_FIXED_VOLTAGE_Msk))

/* Set operating current in mA in SNK Fixed PDO */
#define USBPD_PDO_SNK_FIXED_SET_OP_CURRENT(_MA_)     (((PWR_A_10MA((_MA_)  / 1000.0))             \
                                                       << USBPD_PDO_SNK_FIXED_OP_CURRENT_Pos)     \
                                                      & (USBPD_PDO_SNK_FIXED_OP_CURRENT_Msk))

/* ---------------------------- VARIABLE ------------------------ */

/* Set max voltage in mV in SRC VARIABLE PDO */
#define USBPD_PDO_SRC_VARIABLE_SET_MAX_VOLTAGE(_MV_) (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Pos) \
                                                      & (USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Msk))

/* Set min voltage in mV in SRC VARIABLE PDO */
#define USBPD_PDO_SRC_VARIABLE_SET_MIN_VOLTAGE(_MV_) (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Pos) \
                                                      & (USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Msk))

/* Set max current in mA in SRC VARIABLE PDO */
#define USBPD_PDO_SRC_VARIABLE_SET_MAX_CURRENT(_MA_) (((PWR_A_10MA((_MA_)  / 1000.0))             \
                                                       << USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Pos) \
                                                      & (USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Msk))

/* Set max voltage in mV in SNK VARIABLE PDO */
#define USBPD_PDO_SNK_VARIABLE_SET_MAX_VOLTAGE(_MV_) (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SNK_VARIABLE_MAX_VOLTAGE_Pos) \
                                                      & (USBPD_PDO_SNK_VARIABLE_MAX_VOLTAGE_Msk))

/* Set min voltage in mV in SNK VARIABLE PDO */
#define USBPD_PDO_SNK_VARIABLE_SET_MIN_VOLTAGE(_MV_) (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SNK_VARIABLE_MIN_VOLTAGE_Pos) \
                                                      & (USBPD_PDO_SNK_VARIABLE_MIN_VOLTAGE_Msk))

/* Set operating current in mA in SNK VARIABLE */
#define USBPD_PDO_SNK_VARIABLE_SET_OP_CURRENT(_MA_)  (((PWR_A_10MA((_MA_)  / 1000.0))             \
                                                       << USBPD_PDO_SNK_VARIABLE_OP_CURRENT_Pos)  \
                                                      & (USBPD_PDO_SNK_VARIABLE_OP_CURRENT_Msk))

/* ---------------------------- BATTERY ------------------------ */

/* Set max voltage in mV in SRC BATTERY PDO */
#define USBPD_PDO_SRC_BATTERY_SET_MAX_VOLTAGE(_MV_)  (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Pos)  \
                                                      & (USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Msk))

/* Set min voltage in mV in SRC BATTERY PDO */
#define USBPD_PDO_SRC_BATTERY_SET_MIN_VOLTAGE(_MV_)  (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Pos)  \
                                                      & (USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Msk))

/* Set max power in mW in SRC BATTERY PDO */
#define USBPD_PDO_SRC_BATTERY_SET_MAX_POWER(_MA_)    (((PWR_W((_MA_)       / 1000.0))             \
                                                       << USBPD_PDO_SRC_BATTERY_MAX_POWER_Pos)    \
                                                      & (USBPD_PDO_SRC_BATTERY_MAX_POWER_Msk))

/* Set max voltage in mV in SNK BATTERY PDO */
#define USBPD_PDO_SNK_BATTERY_SET_MAX_VOLTAGE(_MV_)  (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SNK_BATTERY_MAX_VOLTAGE_Pos)  \
                                                      & (USBPD_PDO_SNK_BATTERY_MAX_VOLTAGE_Msk))

/* Set min voltage in mV in SNK BATTERY PDO */
#define USBPD_PDO_SNK_BATTERY_SET_MIN_VOLTAGE(_MV_)  (((PWR_V_50MV((_MV_)  / 1000.0))             \
                                                       << USBPD_PDO_SNK_BATTERY_MIN_VOLTAGE_Pos)  \
                                                      & (USBPD_PDO_SNK_BATTERY_MIN_VOLTAGE_Msk))

/* Set operating power in mW in SNK BATTERY PDO */
#define USBPD_PDO_SNK_BATTERY_SET_OP_POWER(_MA_)     (((PWR_W((_MA_)       / 1000.0))             \
                                                       << USBPD_PDO_SNK_BATTERY_OP_POWER_Pos)     \
                                                      & (USBPD_PDO_SNK_BATTERY_OP_POWER_Msk))

/* ---------------------------- APDO ---------------------------- */

/* Set min voltage in mV in SRC APDO */
#define USBPD_PDO_SRC_APDO_SET_MIN_VOLTAGE(_MV_)     (((PWR_V_100MV((_MV_) / 1000.0))             \
                                                       << USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Pos)     \
                                                      & (USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Msk))

/* Set max voltage in mV in SRC APDO */
#define USBPD_PDO_SRC_APDO_SET_MAX_VOLTAGE(_MV_)     (((PWR_V_100MV((_MV_) / 1000.0))             \
                                                       << USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Pos)     \
                                                      & (USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Msk))

/* Set max current in mA in SRC APDO */
#define USBPD_PDO_SRC_APDO_SET_MAX_CURRENT(_MA_)     (((PWR_A_50MA((_MA_)  / 1000.0))             \
                                                       << USBPD_PDO_SRC_APDO_MAX_CURRENT_Pos)     \
                                                      & (USBPD_PDO_SRC_APDO_MAX_CURRENT_Msk))

/* Set min voltage in mV in SNK APDO */
#define USBPD_PDO_SNK_APDO_SET_MIN_VOLTAGE(_MV_)     (((PWR_V_100MV((_MV_) / 1000.0))             \
                                                       << USBPD_PDO_SNK_APDO_MIN_VOLTAGE_Pos)     \
                                                      & (USBPD_PDO_SNK_APDO_MIN_VOLTAGE_Msk))

/* Set max voltage in mV in SNK APDO */
#define USBPD_PDO_SNK_APDO_SET_MAX_VOLTAGE(_MV_)     (((PWR_V_100MV((_MV_) / 1000.0))             \
                                                       << USBPD_PDO_SNK_APDO_MAX_VOLTAGE_Pos)     \
                                                      & (USBPD_PDO_SNK_APDO_MAX_VOLTAGE_Msk))

/* Set max current in mA in SNK APDO */
#define USBPD_PDO_SNK_APDO_SET_MAX_CURRENT(_MA_)     (((PWR_A_50MA((_MA_)  / 1000.0))             \
                                                       << USBPD_PDO_SNK_APDO_MAX_CURRENT_Pos)     \
                                                      & (USBPD_PDO_SNK_APDO_MAX_CURRENT_Msk))
#define USBPD_PORT_IsValid(__Port__) ((__Port__) < (USBPD_PORT_COUNT))

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN variables */

/* USER CODE END variables */
/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32_USBPD_APPLICATION_POWER_IF_Exported_Functions USBPD PWR IF Exported Functions
  * @{
  */
/**
  * @brief  Initialize structures and variables related to power board profiles
  *         used by Sink and Source, for all available ports.
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_Init(void);

/**
  * @brief  Checks if the power on a specified port is ready
  * @param  PortNum Port number
  * @param  Vsafe   Vsafe status based on @ref USBPD_VSAFE_StatusTypeDef
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SupplyReady(uint8_t PortNum, USBPD_VSAFE_StatusTypeDef Vsafe);

/**
  * @brief  Reads the voltage and the current on a specified port
  * @param  PortNum Port number
  * @param  pVoltage The Voltage in mV
  * @param  pCurrent The Current in mA
  * @retval ENABLE or DISABLE
  */
USBPD_StatusTypeDef USBPD_PWR_IF_ReadVA(uint8_t PortNum, uint16_t *pVoltage, uint16_t *pCurrent);

/**
  * @brief  Enables the VConn on the port.
  * @param  PortNum Port number
  * @param  CC      Specifies the CCx to be selected based on @ref CCxPin_TypeDef structure
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_Enable_VConn(uint8_t PortNum, CCxPin_TypeDef CC);

/**
  * @brief  Disable the VConn on the port.
  * @param  PortNum Port number
  * @param  CC      Specifies the CCx to be selected based on @ref CCxPin_TypeDef structure
  * @retval USBPD Status
  */
USBPD_StatusTypeDef USBPD_PWR_IF_Disable_VConn(uint8_t PortNum, CCxPin_TypeDef CC);

/**
  * @brief  Allow PDO data reading from PWR_IF storage.
  * @param  PortNum Port number
  * @param  DataId Type of data to be read from PWR_IF
  *         This parameter can be one of the following values:
  *           @arg @ref USBPD_CORE_DATATYPE_SRC_PDO Source PDO reading requested
  *           @arg @ref USBPD_CORE_DATATYPE_SNK_PDO Sink PDO reading requested
  * @param  Ptr Pointer on address where PDO values should be written (u8 pointer)
  * @param  Size Pointer on nb of u32 written by PWR_IF (nb of PDOs)
  * @retval None
  */
void USBPD_PWR_IF_GetPortPDOs(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint8_t *Ptr, uint32_t *Size);

/**
  * @brief  Find out SRC PDO pointed out by a position provided in a Request DO (from Sink).
  * @param  PortNum Port number
  * @param  RdoPosition RDO Position in list of provided PDO
  * @param  Pdo Pointer on PDO value pointed out by RDO position (u32 pointer)
  * @retval Status of search
  *         USBPD_OK : Src PDO found for requested DO position (output Pdo parameter is set)
  *         USBPD_FAIL : Position is not compliant with current Src PDO for this port (no corresponding PDO value)
  */
USBPD_StatusTypeDef USBPD_PWR_IF_SearchRequestedPDO(uint8_t PortNum, uint32_t RdoPosition, uint32_t *Pdo);

/**
  * @brief  Function called in case of critical issue is detected to switch in safety mode.
  * @param  ErrorType Type of error detected by monitoring (based on @ref USBPD_PWR_IF_ERROR)
  * @retval None
  */
void USBPD_PWR_IF_AlarmType(USBPD_PWR_IF_ERROR ErrorType);

/**
  * @brief  Function called in case of critical issue is detected to switch in safety mode.
  * @retval None
  */
void USBPD_PWR_IF_Alarm(void);

/**
  * @brief Function is called to get VBUS power status.
  * @param PortNum Port number
  * @param PowerTypeStatus  Power type status based on @ref USBPD_VBUSPOWER_STATUS
  * @retval UBBPD_TRUE or USBPD_FALSE
  */
uint8_t USBPD_PWR_IF_GetVBUSStatus(uint8_t PortNum, USBPD_VBUSPOWER_STATUS PowerTypeStatus);

/**
  * @brief Function is called to set the VBUS threshold when a request has been accepted.
  * @param PortNum Port number
  * @retval None
  */
void USBPD_PWR_IF_UpdateVbusThreshold(uint8_t PortNum);

/**
  * @brief Function is called to reset the VBUS threshold when there is a power reset.
  * @param PortNum Port number
  * @retval None
  */
void USBPD_PWR_IF_ResetVbusThreshold(uint8_t PortNum);

/* USER CODE BEGIN Exported Functions */

/* USER CODE END Exported Functions */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PW_IF_H */
