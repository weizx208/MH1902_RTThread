/* aes.h - header file for aes.c
 */

#ifndef __MH_AES_H
#define __MH_AES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mh_crypt.h"

#define MH_AES_OPT_MODE_Mask        (BIT(0))         
#define MH_AES_OPT_MODE_ENCRYPT     (0)
#define MH_AES_OPT_MODE_DECRYPT     (BIT(0))

#define MH_AES_OPT_BLK_Mask         (BIT(1)|BIT(2))
#define MH_AES_OPT_BLK_CBC          (BIT(1))
#define MH_AES_OPT_BLK_ECB          (BIT(2))
#define MH_AES_OPT_BLK_CBC_MAC      (BIT(1)|BIT(2))

#define MH_AES_OPT_KEY_LEN_Mask     (BIT(3)|BIT(4))
#define MH_AES_OPT_KEY_128          (BIT(3))
#define MH_AES_OPT_KEY_192          (BIT(4))
#define MH_AES_OPT_KEY_256          (BIT(3)|BIT(4))

#define MH_RET_AES_INIT                         (0x5471B9AA)
#define MH_RET_AES_PARAM_ERR                    (MH_RET_AES_INIT + 1)
#define MH_RET_AES_FAILURE                      (MH_RET_AES_INIT + 2)
#define MH_RET_AES_MODE_ERROR                   (MH_RET_AES_INIT + 3)
#define MH_RET_AES_INPUT_ERROR                  (MH_RET_AES_INIT + 4)
#define MH_RET_AES_PACK_MODE_ERROR              (MH_RET_AES_INIT + 5)
#define MH_RET_AES_OUTBUF_TOO_SMALL             (MH_RET_AES_INIT + 6)
#define MH_RET_AES_INPUT_SIZE_ERROR             (MH_RET_AES_INIT + 7)
#define MH_RET_AES_KEY_SIZE_ERROR               (MH_RET_AES_INIT + 8)
#define MH_RET_AES_ATTACKED                     (MH_RET_AES_INIT + 9)
#define MH_RET_AES_DFA_LOCK                     (MH_RET_AES_INIT + 10)
#define MH_RET_AES_SUCCESS                      (MH_RET_AES_INIT + 0x0100)

uint32_t MHAES_EncDec(MH_SYM_CRYPT_CALL *pCall);

#endif

#ifdef __cplusplus
}
#endif

