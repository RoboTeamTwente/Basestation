#ifndef __USB_RTT_CORE_H
#define __USB_RTT_CORE_H


/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

#include "usbd_def.h" // USB constants and definitions

// IN means host <-- device, OUT means host --> device
// EP 0x00U and 0x80U are reserved for command packets
#define RTT_ROBOT_IN_EP               0x81U
#define RTT_ROBOT_OUT_EP              0x01U
#define RTT_BASESTATION_IN_EP         0x82U
#define RTT_BASESTATION_OUT_EP        0x02U

#define USB_BULK_HS_MAX_PACKET_SIZE   USB_HS_MAX_PACKET_SIZE  // Max Bulk packet size in bytes for HS
#define USB_BULK_FS_MAX_PACKET_SIZE   USB_FS_MAX_PACKET_SIZE   // Max Bulk packet size in bytes for FS

#define USB_RTT_CONFIG_DESC_SIZ       64U // config size in bytes, needs manual update if config is changed
#define USB_RTT_EP1_TYPE              USBD_EP_TYPE_BULK
#define USB_RTT_EP1_PACKET_SIZE       USB_BULK_HS_MAX_PACKET_SIZE
#define USB_RTT_EP2_TYPE              USBD_EP_TYPE_BULK
#define USB_RTT_EP2_PACKET_SIZE       USB_BULK_HS_MAX_PACKET_SIZE

// reference to the RTT USB class struct created in usbd_RTT_class.c, which contains all callback functions needed for usb
extern USBD_ClassTypeDef USBD_RTT_ClassDriver;

// Class specific data
typedef struct
{
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *BasestationRxBuffer;
  uint8_t  *BasestationTxBuffer;
  uint8_t  *RobotsRxBuffer;
  uint8_t  *RobotsTxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
} USBD_RTT_HandleTypeDef;


#endif  /* __USB_RTT_CORE_H */
