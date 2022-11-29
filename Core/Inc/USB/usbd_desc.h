
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_DESC_TEMPLATE_H
#define __USBD_DESC_TEMPLATE_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define         DEVICE_ID1          (UID_BASE)
#define         DEVICE_ID2          (UID_BASE + 0x4U)
#define         DEVICE_ID3          (UID_BASE + 0x8U)

#define  USB_SIZ_STRING_SERIAL       0x1AU

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern USBD_DescriptorsTypeDef RTT_Desc;

extern uint8_t *USBD_RTT_DeviceQualifierDescriptor(uint16_t *length);
extern uint8_t *USBD_RTT_GetDeviceQualifierDesc(uint16_t *length);

#endif /* __USBD_DESC_TEMPLATE_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
