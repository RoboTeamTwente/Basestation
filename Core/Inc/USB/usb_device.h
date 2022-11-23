#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "usbd_def.h"


// USB function called by main.c to initialise the USB device
void MX_USB_DEVICE_Init(void);

#endif //__USB_DEVICE__H__