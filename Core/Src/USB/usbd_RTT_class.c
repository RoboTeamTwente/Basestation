
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

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
/* USB RTT device Configuration Descriptor */
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

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_RTT_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


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
  (void)USBD_LL_OpenEP(pdev, RTT_ROBOT_IN_EP, USB_RTT_EP1_TYPE, USB_RTT_EP1_PACKET_SIZE);
  pdev->ep_in[RTT_ROBOT_IN_EP & 0xFU].is_used = 1U;

  (void)USBD_LL_OpenEP(pdev, RTT_BASESTATION_IN_EP, USB_RTT_EP2_TYPE, USB_RTT_EP2_PACKET_SIZE);
  pdev->ep_in[RTT_BASESTATION_IN_EP & 0xFU].is_used = 1U;

  /* Open all EP OUT */
  (void)USBD_LL_OpenEP(pdev, RTT_ROBOT_OUT_EP, USB_RTT_EP1_TYPE, USB_RTT_EP1_PACKET_SIZE);
  pdev->ep_out[RTT_ROBOT_OUT_EP & 0xFU].is_used = 1U;

  (void)USBD_LL_OpenEP(pdev, RTT_BASESTATION_OUT_EP, USB_RTT_EP2_TYPE, USB_RTT_EP2_PACKET_SIZE);
  pdev->ep_out[RTT_BASESTATION_OUT_EP & 0xFU].is_used = 1U;

  /* Init  physical Interface components */
  // TODO: Init user?
  // ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init();

  /* Init Xfer states */
  // TODO: Make enum for readability
  hcdc->TxState = 0U;
  hcdc->RxState = 0U;

  /* Prepare Out endpoints to receive next packet */
  (void)USBD_LL_PrepareReceive(pdev, RTT_ROBOT_OUT_EP, hcdc->RobotsRxBuffer, USB_RTT_EP1_PACKET_SIZE);
  (void)USBD_LL_PrepareReceive(pdev, RTT_BASESTATION_OUT_EP, hcdc->BasestationRxBuffer, USB_RTT_EP2_PACKET_SIZE);

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
  (void)USBD_LL_CloseEP(pdev, RTT_ROBOT_IN_EP);
  pdev->ep_in[RTT_ROBOT_IN_EP & 0xFU].is_used = 0U;

  (void)USBD_LL_CloseEP(pdev, RTT_ROBOT_OUT_EP);
  pdev->ep_in[RTT_ROBOT_OUT_EP & 0xFU].is_used = 0U;

  (void)USBD_LL_CloseEP(pdev, RTT_BASESTATION_IN_EP);
  pdev->ep_in[RTT_BASESTATION_IN_EP & 0xFU].is_used = 0U;

  (void)USBD_LL_CloseEP(pdev, RTT_BASESTATION_IN_EP);
  pdev->ep_in[RTT_BASESTATION_IN_EP & 0xFU].is_used = 0U;

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
  *         Handle the RTT specific requests
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
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t *USBD_RTT_DeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_RTT_DeviceQualifierDesc);
  return USBD_RTT_DeviceQualifierDesc;
}


/**
  * @brief  USBD_RTT_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_RTT_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{

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
  * @brief  USBD_RTT_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_RTT_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return (uint8_t)USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t *USBD_RTT_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_RTT_DeviceQualifierDesc);

  return USBD_RTT_DeviceQualifierDesc;
}
