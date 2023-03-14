#ifndef __MH_RSAEFC_H
#define __MH_RSAEFC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "mh_rsa.h"
    
    
    
    
uint32_t MHRSA_CrtEfc(MH_RSA_CALL *pCall);

uint32_t MHRSA_PubEfc(MH_RSA_CALL *pCall);

uint32_t MHRSA_PriEfc(MH_RSA_CALL *pCall);


#ifdef __cplusplus
}
#endif

#endif
