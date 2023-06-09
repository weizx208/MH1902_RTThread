/************************ (C) COPYRIGHT Megahuntmicro *************************
 * @file                : mhscpu_rtc.h
 * @author              : Megahuntmicro
 * @version             : V1.0.0
 * @date                : 21-October-2014
 * @brief               : This file contains all the functions prototypes for the RTC firmware library
 *****************************************************************************/

#ifndef __MHSCPU_RTC_H
#define __MHSCPU_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"

typedef enum 
{
    SELECT_EXT32K = 0,
    SELECT_INC32K
} RTC_SOURCE_TypeDef;
#define IS_RTC_SOURCE(SRC)                      (((SRC) == SELECT_EXT32K) || \
                                                 ((SRC) == SELECT_INC32K))
	
FlagStatus RTC_IsReady(void);

void RTC_ResetCounter(void);
uint32_t RTC_GetCounter(void);

void RTC_SetRefRegister(uint32_t RefValue);
uint32_t RTC_GetRefRegister(void);

void RTC_SetAlarm(uint32_t AlarmValue);

uint32_t RTC_GetAttrackTime(void);

void RTC_ITConfig(FunctionalState NewState);
void RTC_ClearITPendingBit(void);
ITStatus RTC_GetITStatus(void);

void RTC_SourceSelect(RTC_SOURCE_TypeDef source);
	
#ifdef __cplusplus
}
#endif

#endif

/**************************      (C) COPYRIGHT Megahunt    *****END OF FILE****/
