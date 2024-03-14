#ifndef __USBD_CONF_RTT_H
#define __USBD_CONF_RTT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h" 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USBD_MAX_NUM_INTERFACES                2U
#define USBD_MAX_NUM_CONFIGURATION             1U
#define USBD_MAX_STR_DESC_SIZ                  0x100U
#define USBD_SELF_POWERED                      1U
#define USBD_DEBUG_LEVEL                       0U

/* #define for FS and HS identification */
#define DEVICE_FS 		0
#define DEVICE_HS 		1

/* Memory management macros */
#define USBD_malloc               malloc
#define USBD_free                 free
#define USBD_memset               memset
#define USBD_memcpy               memcpy
#define USBD_Delay                HAL_Delay

/* DEBUG macros */
#if (USBD_DEBUG_LEVEL > 0U)
#define  USBD_UsrLog(...)   do { \
                            printf(__VA_ARGS__); \
                            printf("\n"); \
} while (0)
#else
#define USBD_UsrLog(...) do {} while (0)
#endif

#if (USBD_DEBUG_LEVEL > 1U)

#define  USBD_ErrLog(...) do { \
                            printf("ERROR: ") ; \
                            printf(__VA_ARGS__); \
                            printf("\n"); \
} while (0)
#else
#define USBD_ErrLog(...) do {} while (0)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#define  USBD_DbgLog(...)   do { \
                            printf("DEBUG : ") ; \
                            printf(__VA_ARGS__); \
                            printf("\n"); \
} while (0)
#else
#define USBD_DbgLog(...) do {} while (0)
#endif

#endif /* __USBD_CONF_RTT_H */
