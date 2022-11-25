#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "usbd_def.h"
#include "usbd_RTT_class.h"


// USB function called by main.c to initialise the USB device
void MX_USB_DEVICE_Init(void);
// USB function called in basestation.c to register callback functions and starts the USB device after
void USB_Start_Class(USBD_RTT_Callbacks* callbacks);

#endif //__USB_DEVICE__H__