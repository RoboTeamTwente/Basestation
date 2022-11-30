#ifndef __USB_RTT_CORE_H
#define __USB_RTT_CORE_H


/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

#include "usbd_def.h" // USB constants and definitions
#include "usbd_desc.h" // USB descriptors

// RX Buffer Sizes
#define USB_HIGH_PRIO_RX_BUF_SIZE 1024
#define USB_LOW_PRIO_RX_BUF_SIZE 1024

// Endpoint addresses
// IN (0x80) means host <-- device, OUT (0x00) means host --> device
// EP 0x00U and 0x80U are reserved for command packets
#define RTT_HIGH_PRIO_IN_EP               0x81U
#define RTT_HIGH_PRIO_OUT_EP              0x01U
#define RTT_LOW_PRIO_IN_EP                0x82U
#define RTT_LOW_PRIO_OUT_EP               0x02U

// Packet Sizes
#define USB_BULK_HS_MAX_PACKET_SIZE   USB_HS_MAX_PACKET_SIZE  // Max Bulk packet size in bytes for HS
#define USB_BULK_FS_MAX_PACKET_SIZE   USB_FS_MAX_PACKET_SIZE   // Max Bulk packet size in bytes for FS

#define USB_INTR_HS_MAX_PACKET_SIZE    1024  // MAX Interupt packet size in HS
#define USB_INTR_FS_MAX_PACKET_SIZE    64    // MAX Interupt packet size in FS

// Endpoints and interfaces
#define USB_RTT_HIGH_PRIO_INTERFACE       0x00
#define USB_RTT_EP1_TYPE                  USBD_EP_TYPE_BULK
#define USB_RTT_EP1_MAX_PACKET_SIZE       USB_BULK_HS_MAX_PACKET_SIZE
#define USB_RTT_EP1_ALT_TYPE              USBD_EP_TYPE_INTR
#define USB_RTT_EP1_ALT_MAX_PACKET_SIZE   USB_INTR_HS_MAX_PACKET_SIZE

#define USB_RTT_LOW_PRIO_INTERFACE        0x01
#define USB_RTT_EP2_TYPE                  USBD_EP_TYPE_BULK
#define USB_RTT_EP2_MAX_PACKET_SIZE       USB_BULK_HS_MAX_PACKET_SIZE

// reference to the RTT USB class struct created in usbd_RTT_class.c, which contains all callback functions needed for usb
extern USBD_ClassTypeDef USBD_RTT_ClassDriver;
extern USBD_HandleTypeDef hUsbDeviceHS; // in usb_device.c

// Interface setting container
typedef struct{
  uint8_t intID;          // interface number
  uint8_t InAddress;      // IN ENDPOINT address
  uint8_t OutAddress;     // OUT ENDPOINT address
  uint8_t Type;           // Endpoint type
  uint16_t MaxPacketSize; // Max packet size that can be sent over this connection
  uint8_t* Rxbuffer;      // pointer to where packets can be stored
} InterfaceConfig;


// Class specific data
typedef struct{
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint32_t setup_data[USB_HS_MAX_PACKET_SIZE / 4U];
  
  // Interface variables
  uint8_t INT0AltSetting; // which alternative interface is selected, 0 is nomal, 1 is alt
  InterfaceConfig* INT0active; // pointer to active interface
  uint8_t INT1AltSetting;
  InterfaceConfig* INT1active; // pointer to active interface
  __IO uint32_t INT0TxState;
  __IO uint32_t INT1TxState;
  __IO uint32_t RxState;
} USBD_RTT_HandleTypeDef;

// User callable functions to transmit data
USBD_StatusTypeDef USB_TransmitLowPriority(uint8_t* buf, uint32_t len);
USBD_StatusTypeDef USB_TransmitHighPriority(uint8_t* buf, uint32_t len);

// Callback prototypes
typedef USBD_StatusTypeDef USB_Class_Setup_Requests(uint8_t cmd, uint8_t* pbuf, uint16_t length); // Make class implementation instead of user?
typedef void USB_HighPriority_TX_cplt(void);
typedef void USB_HighPriority_RX_cplt(uint8_t* buf, uint32_t len);
typedef void USB_LowPriority_TX_cplt(void);
typedef void USB_LowPriority_RX_cplt(uint8_t* buf, uint32_t len);


// Callback functions needed to be implemented in basestation.c
// NOTE: These functions are called from an interupt, so the implementation of these functions should be the bare minimum needed.
typedef struct USBD_RTT_Callbacks{
  USB_Class_Setup_Requests* usbControl;
  USB_HighPriority_TX_cplt* highprioTXcplt;
  USB_HighPriority_RX_cplt* highprioRXcplt;
  USB_LowPriority_TX_cplt* lowprioTXcplt;
  USB_LowPriority_RX_cplt* lowprioRXcplt;
} USBD_RTT_Callbacks;

#endif  /* __USB_RTT_CORE_H */
