/**
  ******************************************************************************
  * @file    stm32l4s5i_iot01_tsensor.h
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the temperature sensor
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4S5I_IOT01_TSENSOR_H
#define __STM32L4S5I_IOT01_TSENSOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4s5i_iot01.h"
#ifdef USE_LPS22HB_TEMP
#include "lps22hb.h"
#else /* USE_HTS221_TEMP */
#include "hts221.h"
#endif

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32L4S5I_IOT01
  * @{
  */

/** @addtogroup STM32L4S5I_IOT01_TEMPERATURE 
  * @{
  */
   
/** @defgroup STM32L4S5I_IOT01_TEMPERATURE_Exported_Types TEMPERATURE Exported Types
  * @{
  */
   
/** 
  * @brief  TSENSOR Status  
  */ 
typedef enum
{
  TSENSOR_OK = 0,
  TSENSOR_ERROR
}TSENSOR_Status_TypDef;

/**
  * @}
  */


/** @defgroup STM32L4S5I_IOT01_TEMPERATURE_Exported_Functions TEMPERATURE Exported Constants
  * @{
  */
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Sensor Configuration Functions */
uint32_t BSP_TSENSOR_Init(void);
float BSP_TSENSOR_ReadTemp(void);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

#endif /* __STM32L4S5I_IOT01_TSENSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/