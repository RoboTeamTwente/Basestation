
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
static uint8_t *USBD_RTT_GetDeviceQualifierDesc(uint16_t *length);

USBD_ClassTypeDef USBD_RTT_ClassDriver =
{
  USBD_RTT_Init,
  USBD_RTT_DeInit,
  USBD_RTT_Setup,
  USBD_RTT_EP0_TxReady,
  USBD_RTT_EP0_RxReady,
  USBD_RTT_DataIn,
  USBD_RTT_DataOut,
  USBD_RTT_SOF,
  USBD_RTT_IsoINIncomplete,
  USBD_RTT_IsoOutIncomplete,
  USBD_RTT_GetCfgDesc,
  USBD_RTT_GetCfgDesc,
  USBD_RTT_GetCfgDesc,
  USBD_RTT_GetDeviceQualifierDesc,
};

/* USB RTT device Configuration Descriptor */
// Contents are described in https://www.keil.com/pack/doc/mw/USB/html/_u_s_b__configuration__descriptor.html
#define USB_RTT_CONFIG_DESC_SIZ       64U // config size in bytes, needs manual update if config is changed
__ALIGN_BEGIN static uint8_t USBD_RTT_CfgDesc[USB_RTT_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_RTT_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supports Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */

  /**********  Descriptor of RTT interface 0 Alternate setting 0 **************/

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

  if (hcdc == NULL)
  {
  pdev->pClassData = NULL;
  return (uint8_t)USBD_EMEM;
  }

  // link RTT class to device
  pdev->pClassData = (void *)hcdc;

  if (pdev->dev_speed != USBD_SPEED_HIGH){
    // Only HS allowed
    return (uint8_t)USBD_EMEM;
  }

  /* Open all EP IN */
  (void)USBD_LL_OpenEP(pdev, RTT_HIGH_PRIO_IN_EP, USB_RTT_EP1_TYPE, USB_RTT_EP1_PACKET_SIZE);
  pdev->ep_in[RTT_HIGH_PRIO_IN_EP & 0xFU].is_used = 1U;

  (void)USBD_LL_OpenEP(pdev, RTT_LOW_PRIO_IN_EP, USB_RTT_EP2_TYPE, USB_RTT_EP2_PACKET_SIZE);
  pdev->ep_in[RTT_LOW_PRIO_IN_EP & 0xFU].is_used = 1U;

  /* Open all EP OUT */
  (void)USBD_LL_OpenEP(pdev, RTT_HIGH_PRIO_OUT_EP, USB_RTT_EP1_TYPE, USB_RTT_EP1_PACKET_SIZE);
  pdev->ep_out[RTT_HIGH_PRIO_OUT_EP & 0xFU].is_used = 1U;

  (void)USBD_LL_OpenEP(pdev, RTT_LOW_PRIO_OUT_EP, USB_RTT_EP2_TYPE, USB_RTT_EP2_PACKET_SIZE);
  pdev->ep_out[RTT_LOW_PRIO_OUT_EP & 0xFU].is_used = 1U;

  /* Init  physical Interface components */
  // TODO: Init user?
  // ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init();

  /* Init Xfer states */
  // TODO: Make enum for readability
  hcdc->TxState = 0U;

  /* Prepare Out endpoints to receive next packet */
  (void)USBD_LL_PrepareReceive(pdev, RTT_HIGH_PRIO_OUT_EP, hcdc->HighPriorityRxBuffer, USB_RTT_EP1_PACKET_SIZE);
  (void)USBD_LL_PrepareReceive(pdev, RTT_LOW_PRIO_OUT_EP, hcdc->LowPriorityRxBuffer, USB_RTT_EP2_PACKET_SIZE);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_RTT_Init
  *         DeInitialize the RTT layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_RTT_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  // Close all endpoints
  (void)USBD_LL_CloseEP(pdev, RTT_HIGH_PRIO_IN_EP);
  pdev->ep_in[RTT_HIGH_PRIO_IN_EP & 0xFU].is_used = 0U;

  (void)USBD_LL_CloseEP(pdev, RTT_HIGH_PRIO_OUT_EP);
  pdev->ep_in[RTT_HIGH_PRIO_OUT_EP & 0xFU].is_used = 0U;

  (void)USBD_LL_CloseEP(pdev, RTT_LOW_PRIO_IN_EP);
  pdev->ep_in[RTT_LOW_PRIO_IN_EP & 0xFU].is_used = 0U;

  (void)USBD_LL_CloseEP(pdev, RTT_LOW_PRIO_IN_EP);
  pdev->ep_in[RTT_LOW_PRIO_IN_EP & 0xFU].is_used = 0U;

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
static uint8_t USBD_RTT_Setup(USBD_HandleTypeDef *pdev,
                                   USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  // TODO: figure out setup

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    switch (req->bRequest)
    {
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  default:
    USBD_CtlError(pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_RTT_DataIn
  *         handle data IN Stage (host <-- device), This is called when a transmit has completed. In the case the data transfer needs more transfers send a new packet
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_RTT_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_RTT_HandleTypeDef *hcdc;
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc = (USBD_RTT_HandleTypeDef *)pdev->pClassData;
  // keep sending data if not done
  if ((pdev->ep_in[epnum].total_length > 0U) &&
      ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))
  {
    /* Update the packet total length */
    pdev->ep_in[epnum].total_length = 0U;

    /* Send ZLP */
    (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
  }
  else
  {
    hcdc->TxState = 0U;
    if(epnum == RTT_HIGH_PRIO_IN_EP){
      ((USBD_RTT_Callbacks *)pdev->pUserData)->highprioTXcplt();
    }else if(epnum == RTT_LOW_PRIO_IN_EP){
      ((USBD_RTT_Callbacks *)pdev->pUserData)->lowprioTXcplt();
    }
    hcdc->TxState = 0; // TX done
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

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Get the received data length */
  uint32_t received_length = USBD_LL_GetRxDataSize(pdev, epnum);

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */
  if(epnum == RTT_HIGH_PRIO_OUT_EP){
    // Let the user process data in the buffer
    ((USBD_RTT_Callbacks *)pdev->pUserData)->highprioRXcplt(hcdc->HighPriorityRxBuffer, received_length);
    // Listen for new packets
    (void)USBD_LL_PrepareReceive(pdev, RTT_HIGH_PRIO_OUT_EP, hcdc->HighPriorityRxBuffer, received_length);
  }else if( epnum == RTT_LOW_PRIO_OUT_EP){
    // Let the user process data in the buffer
    ((USBD_RTT_Callbacks *)pdev->pUserData)->lowprioRXcplt(hcdc->LowPriorityRxBuffer, received_length);
    // Listen for new packets
    (void)USBD_LL_PrepareReceive(pdev, RTT_LOW_PRIO_OUT_EP, hcdc->LowPriorityRxBuffer, received_length);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_RTT_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_RTT_EP0_RxReady(USBD_HandleTypeDef *pdev)
{

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

  if (hcdc == NULL){
    return (uint8_t)USBD_FAIL;
  }
  // can't send when still busy sending
  if(hcdc->TxState == 1){
    return USBD_BUSY;
  }
  /* Tx Transfer in progress */
  hcdc->TxState = 1U;
  /* Update the packet total length */
  hUsbDeviceHS.ep_in[RTT_LOW_PRIO_IN_EP & 0xFU].total_length = len;
  /* Transmit next packet */
  (void)USBD_LL_Transmit(&hUsbDeviceHS, RTT_LOW_PRIO_IN_EP, hcdc->LowPriorityTxBuffer, len);

  return USBD_OK;
}

/**
  * @brief  USB_TransmitHighPriority
  *         Transmits data on RTT_HIGH_PRIO_IN_EP
  * @param  buf: buffer with data to send
  * @param  len: length of the data
  * @retval USB status
  */
static USBD_StatusTypeDef USB_TransmitHighPriority(USBD_HandleTypeDef *pdev, uint8_t* buf, uint32_t len){
  USBD_RTT_HandleTypeDef *hcdc = (USBD_RTT_HandleTypeDef *)hUsbDeviceHS.pClassData;

  if (hcdc == NULL){
    return (uint8_t)USBD_FAIL;
  }
  // can't send when still busy sending
  if(hcdc->TxState == 1){
    return USBD_BUSY;
  }
  /* Tx Transfer in progress */
  hcdc->TxState = 1U;
  /* Update the packet total length */
  hUsbDeviceHS.ep_in[RTT_HIGH_PRIO_IN_EP & 0xFU].total_length = len;
  /* Transmit next packet */
  (void)USBD_LL_Transmit(&hUsbDeviceHS, RTT_HIGH_PRIO_IN_EP, hcdc->HighPriorityTxBuffer, len);

  return USBD_OK;
}