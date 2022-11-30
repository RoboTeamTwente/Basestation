
/* Includes ------------------------------------------------------------------*/
#include "usbd_RTT_class.h"
#include "usbd_ctlreq.h"

// standard functions used by RTT class needed to implement for USBD library
static uint8_t USBD_RTT_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_RTT_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_RTT_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_RTT_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_RTT_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_RTT_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_RTT_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_RTT_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_RTT_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_RTT_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t *USBD_RTT_GetCfgDesc(uint16_t *length);
uint8_t *USBD_RTT_GetDeviceQualifierDesc(uint16_t *length);


// Helper functions
static void USBD_RTT_OpenInterface(USBD_HandleTypeDef *pdev, InterfaceConfig* interface);
static void USBD_RTT_CloseInterface(USBD_HandleTypeDef *pdev, InterfaceConfig* interface);
static void USBD_RTT_PrepareReceiveInterface(USBD_HandleTypeDef *pdev, InterfaceConfig* interface);

// USBD functions
USBD_ClassTypeDef USBD_RTT_ClassDriver =
{
  USBD_RTT_Init,
  USBD_RTT_DeInit,
  USBD_RTT_Setup,
  USBD_RTT_EP0_TxReady,     //not used currently
  USBD_RTT_EP0_RxReady,
  USBD_RTT_DataIn,
  USBD_RTT_DataOut,
  USBD_RTT_SOF,             //not used currently
  USBD_RTT_IsoINIncomplete, //not used currently
  USBD_RTT_IsoOutIncomplete,//not used currently
  USBD_RTT_GetCfgDesc,
  USBD_RTT_GetCfgDesc,
  USBD_RTT_GetCfgDesc,
  USBD_RTT_GetDeviceQualifierDesc,
};

// RX buffers
static uint8_t HighPriorityRxBuffer[USB_HIGH_PRIO_RX_BUF_SIZE] __attribute__ ((aligned (4)));
static uint8_t LowPriorityRxBuffer[USB_LOW_PRIO_RX_BUF_SIZE] __attribute__ ((aligned (4)));

// Interface configs
static InterfaceConfig int0 = {.InAddress = RTT_HIGH_PRIO_IN_EP, .OutAddress = RTT_HIGH_PRIO_OUT_EP, .MaxPacketSize = USB_RTT_EP1_MAX_PACKET_SIZE, .Type = USB_RTT_EP1_TYPE, .Rxbuffer = HighPriorityRxBuffer};
static InterfaceConfig int0ALT = {.InAddress = RTT_HIGH_PRIO_IN_EP, .OutAddress = RTT_HIGH_PRIO_OUT_EP, .MaxPacketSize = USB_RTT_EP1_ALT_MAX_PACKET_SIZE, .Type = USB_RTT_EP1_ALT_TYPE, .Rxbuffer = HighPriorityRxBuffer};
static InterfaceConfig int1 = {.InAddress = RTT_LOW_PRIO_IN_EP, .OutAddress = RTT_LOW_PRIO_OUT_EP, .MaxPacketSize = USB_RTT_EP2_MAX_PACKET_SIZE, .Type = USB_RTT_EP2_TYPE, .Rxbuffer = LowPriorityRxBuffer};

/* USB RTT device Configuration Descriptor */
// Contents are described in https://www.keil.com/pack/doc/mw/USB/html/_u_s_b__configuration__descriptor.html
// A layout example is described in https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-device-layout
#define USB_RTT_CONFIG_DESC_SIZ       78U // config size in bytes, needs manual update if config is changed
__ALIGN_BEGIN static uint8_t USBD_RTT_CfgDesc[] __ALIGN_END =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  LOBYTE(USB_RTT_CONFIG_DESC_SIZ),/* wTotalLength: Bytes returned */
  HIBYTE(USB_RTT_CONFIG_DESC_SIZ),
  0x02,         /*bNumInterfaces: 2 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of human readable string descriptor describing the configuration*/
  0b11000000,   /*bmAttributes: bus powered and Supports Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */

  /**********  Descriptor of RTT interface 0 Alternate setting 0 (HIGH PRIO channel BULK)**************/
  0x09, // blength 
  USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface
  USB_RTT_HIGH_PRIO_INTERFACE, //bInterfaceNumber
  0x00, //bAlternateSetting
  0x02, //bNumEndpoints
  0xFF, //bInterfaceClass - Custom class
  0x00, //bInterfaceSubClass
  0x00, //bInterfaceProtocol
  0x00, //Index to a human readable string describing this interface (0x00 - not available)

  /********************* Endpoints for interface 0, ALT 0*/
  // Endpoint HIGH PRIO IN
  0x07, //bLength
  USB_DESC_TYPE_ENDPOINT, //bDescriptorType: Endpoint
  RTT_HIGH_PRIO_IN_EP,    //bEndpointAddress
  USB_RTT_EP1_TYPE,       //bmAttributes, (ISO has more settings)
  LOBYTE(USB_RTT_EP1_MAX_PACKET_SIZE), // wMaxPacketSize
  HIBYTE(USB_RTT_EP1_MAX_PACKET_SIZE), // wMaxPacketSize
  0x00,                   //bInterval interval for polling (frame counts). Ignored for Bulk and Control
  
  // Endpoint HIGH PRIO OUT
  0x07, //bLength
  USB_DESC_TYPE_ENDPOINT, //bDescriptorType: Endpoint
  RTT_HIGH_PRIO_OUT_EP,    //bEndpointAddress
  USB_RTT_EP1_TYPE,       //bmAttributes, (ISO has more settings)
  LOBYTE(USB_RTT_EP1_MAX_PACKET_SIZE), // wMaxPacketSize
  HIBYTE(USB_RTT_EP1_MAX_PACKET_SIZE), // wMaxPacketSize
  0x00,                   //bInterval interval for polling (frame counts). Ignored for Bulk and Control
  
  /**********  Descriptor of RTT interface 0 Alternate setting 1 (HIGH PRIO channel Interupt)**************/
  0x09, // blength 
  USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface
  USB_RTT_HIGH_PRIO_INTERFACE, //bInterfaceNumber
  0x01, //bAlternateSetting
  0x02, //bNumEndpoints
  0xFF, //bInterfaceClass - Custom class
  0x00, //bInterfaceSubClass
  0x00, //bInterfaceProtocol
  0x00, //Index to a human readable string describing this interface (0x00 - not available)

  /********************* Endpoints for interface 0, ALT 0*/
  // Endpoint HIGH PRIO IN
  0x07, //bLength
  USB_DESC_TYPE_ENDPOINT, //bDescriptorType: Endpoint
  RTT_HIGH_PRIO_IN_EP,    //bEndpointAddress
  USB_RTT_EP1_ALT_TYPE,       //bmAttributes, (ISO has more settings)
  LOBYTE(USB_RTT_EP1_ALT_MAX_PACKET_SIZE), // wMaxPacketSize
  HIBYTE(USB_RTT_EP1_ALT_MAX_PACKET_SIZE), // wMaxPacketSize
  0x00,                   //bInterval interval for polling (frame counts). Ignored for Bulk and Control
  
  // Endpoint HIGH PRIO OUT
  0x07, //bLength
  USB_DESC_TYPE_ENDPOINT,   //bDescriptorType: Endpoint
  RTT_HIGH_PRIO_OUT_EP,     //bEndpointAddress
  USB_RTT_EP1_ALT_TYPE,     //bmAttributes, (ISO has more settings)
  LOBYTE(USB_RTT_EP1_ALT_MAX_PACKET_SIZE), // wMaxPacketSize
  HIBYTE(USB_RTT_EP1_ALT_MAX_PACKET_SIZE), // wMaxPacketSize
  0x00,                   //bInterval interval for polling (frame counts). Ignored for Bulk and Control

  /**********  Descriptor of RTT interface 1 Alternate setting 0 (LOW PRIO channel)**************/
  0x09, // blength 
  USB_DESC_TYPE_INTERFACE, //bDescriptorType: Interface
  USB_RTT_LOW_PRIO_INTERFACE, //bInterfaceNumber
  0x00, //bAlternateSetting
  0x02, //bNumEndpoints
  0xFF, //bInterfaceClass - Custom class
  0x00, //bInterfaceSubClass
  0x00, //bInterfaceProtocol
  0x00, //Index to a human readable string describing this interface (0x00 - not available)
  
  /********************* Endpoints for interface 1, ALT 0*/
  // Endpoint LOW PRIO IN
  0x07, //bLength
  USB_DESC_TYPE_ENDPOINT, //bDescriptorType: Endpoint
  RTT_LOW_PRIO_IN_EP,    //bEndpointAddress
  USB_RTT_EP2_TYPE,       //bmAttributes, (ISO has more settings)
  LOBYTE(USB_RTT_EP2_MAX_PACKET_SIZE), // wMaxPacketSize
  HIBYTE(USB_RTT_EP2_MAX_PACKET_SIZE), // wMaxPacketSize
  0x00,                   //bInterval interval for polling (frame counts). Ignored for Bulk and Control
  
  // Endpoint LOW PRIO OUT
  0x07, //bLength
  USB_DESC_TYPE_ENDPOINT, //bDescriptorType: Endpoint
  RTT_LOW_PRIO_OUT_EP,    //bEndpointAddress
  USB_RTT_EP2_TYPE,       //bmAttributes, (ISO has more settings)
  LOBYTE(USB_RTT_EP2_MAX_PACKET_SIZE), // wMaxPacketSize
  HIBYTE(USB_RTT_EP2_MAX_PACKET_SIZE), // wMaxPacketSize
  0x00,                   //bInterval interval for polling (frame counts). Ignored for Bulk and Control
  };
/**
  * @brief  USBD_RTT_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_RTT_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_RTT_CfgDesc);
  return USBD_RTT_CfgDesc;
}

/**
  * @brief  USBD_RTT_Init
  *         Initialize the RTT interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_RTT_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  // alloc RTTclass specific data struct
  USBD_RTT_HandleTypeDef *hcdc;
  hcdc = USBD_malloc(sizeof(USBD_RTT_HandleTypeDef));

  if (hcdc == NULL){
    pdev->pClassData = NULL;
    return (uint8_t)USBD_EMEM;
  }

  // link RTT class to device
  pdev->pClassData = (void *)hcdc;

  if (pdev->dev_speed != USBD_SPEED_HIGH){
    // Only HS allowed
    return (uint8_t)USBD_EMEM;
  }

  // Link active interfaces
  InterfaceConfig* i0 = &int0;
  InterfaceConfig* i1 = &int1;
  hcdc->INT0active = &int0;
  hcdc->INT1active = &int1;

  /* Open all interfaces IN */
  USBD_RTT_OpenInterface(pdev, i0);
  USBD_RTT_OpenInterface(pdev, i1);

  /* Init  physical Interface components */
  // TODO: Init user?
  // ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init();

  /* Init Xfer states */
  // TODO: Make enum for readability
  hcdc->INT0TxState = 0U;
  hcdc->INT1TxState = 0U;

  /* Prepare Out endpoints to receive next packet */
  USBD_RTT_PrepareReceiveInterface(pdev,i0);
  USBD_RTT_PrepareReceiveInterface(pdev,i1);
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_RTT_Init
  *         DeInitialize the RTT layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_RTT_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx){
  USBD_RTT_HandleTypeDef *hcdc = (USBD_RTT_HandleTypeDef *)pdev->pClassData;
  
  // Close all endpoints
  USBD_RTT_CloseInterface(pdev, hcdc->INT0active);
  USBD_RTT_CloseInterface(pdev, hcdc->INT1active);

  // remove RTT class
  if (pdev->pClassData != NULL)
  {
    // TODO: DeInit user?
    // ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->DeInit();
    (void)USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_RTT_Setup
  *         Handle the RTT specific setup requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_RTT_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req){
  // TODO: figure out setup
  USBD_RTT_HandleTypeDef *hcdc = (USBD_RTT_HandleTypeDef *)pdev->pClassData;
  USBD_RTT_Callbacks * callbacks = ((USBD_RTT_Callbacks *)pdev->pUserData);
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    // RTT Class specific requests
  case USB_REQ_TYPE_CLASS:
    // If meant to receive data, prepare to receive
    if ((req->bmRequest & 0x80U) != 0U){
      hcdc->CmdOpCode = req->bRequest;
      hcdc->CmdLength = (uint8_t)req->wLength;
      (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->setup_data, req->wLength);
    }else if(req->wLength != 0U){
      // Have to send data
      if(callbacks->usbControl){callbacks->usbControl(req->bRequest, (uint8_t *)hcdc->setup_data, req->wLength);}
      (void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->setup_data, req->wLength);

    }else{
      // just process request data
      if(callbacks->usbControl){callbacks->usbControl(req->bRequest, (uint8_t *)hcdc->setup_data, 0);}
    }
    break;

  // Standard requests. Should already be implemented by the library
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest){
      case USB_REQ_GET_STATUS:
        if (pdev->dev_state == USBD_STATE_CONFIGURED){
          uint16_t status_info = 0U;
          (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
        }else{
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
        }
        break;
      case USB_REQ_GET_INTERFACE:
        if (pdev->dev_state == USBD_STATE_CONFIGURED){
          // device has to be configured
          if(req->wIndex == USB_RTT_HIGH_PRIO_INTERFACE){
            // send active alt interface on interface 0
            (void)USBD_CtlSendData(pdev, &hcdc->INT0AltSetting, 1U);
          }else if(req->wIndex == USB_RTT_LOW_PRIO_INTERFACE){
            // send active alt interface on interface 1
            (void)USBD_CtlSendData(pdev, &hcdc->INT1AltSetting, 1U);
          }else{
            // interface not known
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
        }else{
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
        }
        break;
      case USB_REQ_SET_INTERFACE:
        if (pdev->dev_state != USBD_STATE_CONFIGURED){
          // device has to be configured
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
        }else{
          // Switch alternate interface (only interface 0 is implemented)
          if(req->wIndex == USB_RTT_HIGH_PRIO_INTERFACE){
            hcdc->INT0AltSetting = req->wValue;
            if(req->wValue == 0){
              USBD_RTT_CloseInterface(pdev,&int0);
              USBD_RTT_OpenInterface(pdev,&int0ALT);
              hcdc->INT0active = &int0ALT;
              hcdc->INT0TxState = 0U;
            }else{
              USBD_RTT_CloseInterface(pdev,&int0ALT);
              USBD_RTT_OpenInterface(pdev,&int0);
              hcdc->INT0active = &int0;
              hcdc->INT0TxState = 0U;
            }
          }else{
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
        }
        break;
      default:
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
    }
    break;
  // Unknown request
  default:
    USBD_CtlError(pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_RTT_DataIn
  *         handle data IN Stage (host <-- device), This is called when a message has been transmitted to the host
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_RTT_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_RTT_HandleTypeDef *hcdc = (USBD_RTT_HandleTypeDef *)pdev->pClassData;
  USBD_RTT_Callbacks * callbacks = ((USBD_RTT_Callbacks *)pdev->pUserData);
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if (hcdc == NULL){
    return (uint8_t)USBD_FAIL;
  }

  // When the complete message length fits perfectly in n packets, send a zero length packet (ZLP) to let the host know there is no more data
  if ((pdev->ep_in[epnum].total_length > 0U) && ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))  {
    /* Update the packet total length */
    pdev->ep_in[epnum].total_length = 0U;
    /* Send ZLP */
    (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
  }
  else
  {
    // TX done
    if(epnum == hcdc->INT0active->InAddress){
      hcdc->INT0TxState = 0U;
      if(callbacks->highprioTXcplt){
        callbacks->highprioTXcplt();
      }
    }else if(epnum == hcdc->INT1active->InAddress){
      hcdc->INT1TxState = 0U;
      if(callbacks->highprioTXcplt){
        callbacks->lowprioTXcplt();
      }
    }
  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_RTT_DataOut
  *         handle data OUT Stage (host --> device), At this point the data is already in the buffer
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_RTT_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_RTT_HandleTypeDef *hcdc = (USBD_RTT_HandleTypeDef *)pdev->pClassData;
  USBD_RTT_Callbacks * callbacks = ((USBD_RTT_Callbacks *)pdev->pUserData);

  if (pdev->pClassData == NULL){
    return (uint8_t)USBD_FAIL;
  }

  /* Get the received data length */
  uint32_t received_length = USBD_LL_GetRxDataSize(pdev, epnum);

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */
  if(epnum == hcdc->INT0active->OutAddress){
    // Let the user process data in the buffer
    if(callbacks->highprioRXcplt){
      callbacks->highprioRXcplt(hcdc->INT0active->Rxbuffer, received_length);
    }
    // Listen for new packets
    USBD_RTT_PrepareReceiveInterface(pdev, hcdc->INT0active);
  }else if( epnum == hcdc->INT1active->OutAddress){
    // Let the user process data in the buffer
    if(callbacks->lowprioRXcplt){
      callbacks->lowprioRXcplt(hcdc->INT1active->Rxbuffer, received_length);
    }
    // Listen for new packets
    USBD_RTT_PrepareReceiveInterface(pdev, hcdc->INT1active);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_RTT_EP0_RxReady
  *         handle EP0 Rx Ready event. This is called when data that was requested to be received on Ep0 is complete.
  *         Now process request. (USBD_RTT_Setup was called before this)
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_RTT_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_RTT_HandleTypeDef *hcdc = (USBD_RTT_HandleTypeDef *)pdev->pClassData;
  if ((pdev->pUserData != NULL) && (hcdc->CmdOpCode != 0xFFU)){
    ((USBD_RTT_Callbacks *)pdev->pUserData)->usbControl(hcdc->CmdOpCode, (uint8_t *)hcdc->setup_data, (uint16_t)hcdc->CmdLength);
    hcdc->CmdOpCode = 0xFFU;
  }
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_RTT_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_RTT_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  // Not needed
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_RTT_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_RTT_SOF(USBD_HandleTypeDef *pdev)
{
  //not needed
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_RTT_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_RTT_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  // not used
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_RTT_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_RTT_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  // Not used
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USB_TransmitLowPriority
  *         Transmits data on RTT_LOW_PRIO_IN_EP
  * @param  buf: buffer with data to send
  * @param  len: length of the data
  * @retval USB status
  */
static USBD_StatusTypeDef USB_TransmitLowPriority(uint8_t* buf, uint32_t len){
  USBD_RTT_HandleTypeDef *hcdc = (USBD_RTT_HandleTypeDef *)hUsbDeviceHS.pClassData;

  // Device needs to be configured before data can be sent
  if (hcdc == NULL || hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED){
    return (uint8_t)USBD_FAIL;
  }
  // can't send when still busy sending
  if(hcdc->INT1TxState == 1){
    return USBD_BUSY;
  }
  /* Tx Transfer in progress */
  hcdc->INT1TxState = 1U;
  /* Update the packet total length */
  hUsbDeviceHS.ep_in[hcdc->INT1active->InAddress & 0xFU].total_length = len;
  /* Transmit next packet */
  (void)USBD_LL_Transmit(&hUsbDeviceHS, hcdc->INT1active->InAddress, buf, len);

  return USBD_OK;
}

/**
  * @brief  USB_TransmitHighPriority
  *         Transmits data on RTT_HIGH_PRIO_IN_EP
  * @param  buf: buffer with data to send
  * @param  len: length of the data
  * @retval USB status
  */
static USBD_StatusTypeDef USB_TransmitHighPriority(uint8_t* buf, uint32_t len){
  USBD_RTT_HandleTypeDef *hcdc = (USBD_RTT_HandleTypeDef *)hUsbDeviceHS.pClassData;

  // Device needs to be configured before data can be sent
  if (hcdc == NULL || hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED){
    return (uint8_t)USBD_FAIL;
  }
  // can't send new packets when already sending
  if(hcdc->INT0TxState == 1){
    return USBD_BUSY;
  }
  /* Tx Transfer in progress */
  hcdc->INT0TxState = 1U;
  /* Update the packet total length */
  hUsbDeviceHS.ep_in[RTT_HIGH_PRIO_IN_EP & 0xFU].total_length = len;
  /* Transmit next packet */
  (void)USBD_LL_Transmit(&hUsbDeviceHS, RTT_HIGH_PRIO_IN_EP, buf, len);

  return USBD_OK;
}



// Helper functions
static void USBD_RTT_OpenInterface(USBD_HandleTypeDef *pdev, InterfaceConfig* interface){
  // Open IN Endpoint if defined
  if(interface->InAddress){
    (void)USBD_LL_OpenEP(pdev, interface->InAddress, interface->Type, interface->MaxPacketSize);
    pdev->ep_in[interface->InAddress & 0xFU].is_used = 1U;
  }
  // Open OUT Endpoint if defined
  if(interface->OutAddress){
    (void)USBD_LL_OpenEP(pdev, interface->OutAddress, interface->Type, interface->MaxPacketSize);
    pdev->ep_in[interface->OutAddress & 0xFU].is_used = 1U;
    (void)USBD_LL_PrepareReceive(pdev, interface->OutAddress, interface->Rxbuffer, interface->MaxPacketSize);
  }
}

static void USBD_RTT_CloseInterface(USBD_HandleTypeDef *pdev, InterfaceConfig* interface){
  // Close IN Endpoint if defined
  if(interface->InAddress){
    (void)USBD_LL_CloseEP(pdev, interface->InAddress);
    pdev->ep_in[interface->InAddress & 0xFU].is_used = 0U;
  }
  // Close OUT Endpoint if defined
  if(interface->OutAddress){
    (void)USBD_LL_CloseEP(pdev, interface->OutAddress);
    pdev->ep_in[interface->OutAddress & 0xFU].is_used = 0U;
  }
}

static void USBD_RTT_PrepareReceiveInterface(USBD_HandleTypeDef *pdev, InterfaceConfig* interface){
  (void)USBD_LL_PrepareReceive(pdev, interface->OutAddress, interface->Rxbuffer, interface->MaxPacketSize);
}