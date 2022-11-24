#ifndef __USB_RTT_CORE_H
#define __USB_RTT_CORE_H


/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

#include "usbd_def.h" // USB constants and definitions
#include "usbd_desc.h" // USB descriptors

// IN means host <-- device, OUT means host --> device
// EP 0x00U and 0x80U are reserved for command packets
#define RTT_HIGH_PRIO_IN_EP               0x81U
#define RTT_HIGH_PRIO_OUT_EP              0x01U
#define RTT_LOW_PRIO_IN_EP                0x82U
#define RTT_LOW_PRIO_OUT_EP               0x02U

#define USB_BULK_HS_MAX_PACKET_SIZE   USB_HS_MAX_PACKET_SIZE  // Max Bulk packet size in bytes for HS
#define USB_BULK_FS_MAX_PACKET_SIZE   USB_FS_MAX_PACKET_SIZE   // Max Bulk packet size in bytes for FS


#define USB_RTT_EP1_TYPE              USBD_EP_TYPE_BULK
#define USB_RTT_EP1_PACKET_SIZE       USB_BULK_HS_MAX_PACKET_SIZE
#define USB_RTT_EP2_TYPE              USBD_EP_TYPE_BULK
#define USB_RTT_EP2_PACKET_SIZE       USB_BULK_HS_MAX_PACKET_SIZE

// reference to the RTT USB class struct created in usbd_RTT_class.c, which contains all callback functions needed for usb
extern USBD_ClassTypeDef USBD_RTT_ClassDriver;
extern USBD_HandleTypeDef hUsbDeviceHS; // in usb_device.c


// Class specific data
typedef struct
{
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *HighPriorityRxBuffer; // (host --> device)
  uint8_t  *HighPriorityTxBuffer; // (host <-- device)
  uint8_t  *LowPriorityRxBuffer;  // (host --> device)
  uint8_t  *LowPriorityTxBuffer;  // (host <-- device)
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
} USBD_RTT_HandleTypeDef;

// User callable functions to transmit data
void USB_TransmitLowPriority(USBD_HandleTypeDef *pdev, uint8_t* buf, uint32_t len);
void USB_TransmitHighPriority(USBD_HandleTypeDef *pdev, uint8_t* buf, uint32_t len);

// Callback prototypes
typedef void USB_HighPriority_TX_cplt();
typedef void USB_HighPriority_RX_cplt(uint8_t* buf, uint32_t*len);
typedef void USB_LowPriority_TX_cplt();
typedef void USB_LowPriority_RX_cplt(uint8_t* buf, uint32_t*len);


// Callback functions needed to be implemented in basestation.c
typedef struct USBD_RTT_Callbacks{
  USB_HighPriority_TX_cplt* highprioTXcplt;
  USB_HighPriority_RX_cplt* highprioRXcplt;
  USB_LowPriority_TX_cplt* lowprioTXcplt;
  USB_LowPriority_RX_cplt* lowprioRXcplt;
} USBD_RTT_Callbacks;


#endif  /* __USB_RTT_CORE_H */
