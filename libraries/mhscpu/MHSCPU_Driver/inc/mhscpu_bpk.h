/************************ (C) COPYRIGHT Megahuntmicro *************************
 * @file                : mhscpu_bpk.h
 * @author              : Megahuntmicro
 * @version             : V1.0.0
 * @date                : 21-October-2014
 * @brief               : This file contains all the functions prototypes for the BPK firmware library
 *****************************************************************************/

#ifndef __MHSCPU_BPK_H
#define __MHSCPU_BPK_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"

	 
/** @defgroup BPK_Exported_Types
  * @{
  */ 
#define BPK_KEY_REGION_0                      ((uint32_t)0x0001)
#define BPK_KEY_REGION_1                      ((uint32_t)0x0002)
#define BPK_KEY_REGION_ALL                    ((uint32_t)0x0003)
#define IS_BPK_KEY_REGION(REGION)			  ((((REGION) & ~BPK_KEY_REGION_ALL) == 0x00) && ((REGION) != 0x00))

#define BPK_RDY_POR						      ((uint32_t)0x0002) 
#define BPK_RDY_READY				          ((uint32_t)0x0001) 

#define BPK_RR_RESET						  ((uint32_t)0x0001) 
	 
#define BPK_LR_LOCK_SELF				      ((uint32_t)0x0001) 
#define BPK_LR_LOCK_RESET				      ((uint32_t)0x0002) 
#define BPK_LR_LOCK_KEYWRITE				  ((uint32_t)0x0004) 
#define BPK_LR_LOCK_KEYREAD				      ((uint32_t)0x0008) 
#define BPK_LR_LOCK_KEYCLEAR				  ((uint32_t)0x0010)
#define BPK_LR_LOCK_SCRAMBER				  ((uint32_t)0x0020)   
#define BPK_LR_LOCK_ALL                       ((uint32_t)0x003F)
#define IS_BPK_LOCK(LOCK)					  ((((LOCK) & ~BPK_LR_LOCK_ALL) == 0x00) && ((LOCK) != 0x00))


FlagStatus BPK_IsReady(void);
ErrorStatus BPK_WriteKey(uint32_t *Key,uint32_t Key_Len, uint32_t Key_Offset);
ErrorStatus BPK_ReadKey(uint32_t *Key,uint32_t Key_Len, uint32_t Key_Offset);
void BPK_KeyWriteLock(uint16_t BPK_KEY_Region, FunctionalState NewState);
void BPK_KeyReadLock(uint16_t BPK_KEY_Region, FunctionalState NewState);
void BPK_KeyClear(uint16_t BPK_KEY_Region);
void BPK_Reset(void);
void BPK_SetScramber(uint32_t Scram);
void BPK_Lock(uint32_t BPK_LOCK, FunctionalState NewState);
void BPK_LockSelf(void);
FlagStatus BPK_GetLockStatus(uint32_t BPK_LOCK);


#ifdef __cplusplus
}
#endif

#endif 

/**************************      (C) COPYRIGHT Megahunt    *****END OF FILE****/
