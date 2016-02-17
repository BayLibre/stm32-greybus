/**
  ******************************************************************************
  * @file    usbd_greybus.h
  * @author  Neil Armstrong <narmstrong@baylibre.com>
  * @version V1
  * @date    2016
  * @brief   This file provides the high layer firmware functions to manage the
  * 	 	 Greybus protocol.
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT 2016 Neil Armstrong <narmstrong@baylibre.com>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_greybus.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_GREYBUS
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_GREYBUS_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_GREYBUS_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_GREYBUS_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_GREYBUS_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_GREYBUS_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx);

static uint8_t  USBD_GREYBUS_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx);

static uint8_t  USBD_GREYBUS_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_GREYBUS_DataIn (USBD_HandleTypeDef *pdev,
                                 uint8_t epnum);

static uint8_t  USBD_GREYBUS_DataOut (USBD_HandleTypeDef *pdev,
                                 uint8_t epnum);

static uint8_t  USBD_GREYBUS_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_GREYBUS_GetFSCfgDesc (uint16_t *length);

static uint8_t  *USBD_GREYBUS_GetHSCfgDesc (uint16_t *length);

static uint8_t  *USBD_GREYBUS_GetOtherSpeedCfgDesc (uint16_t *length);

static uint8_t  *USBD_GREYBUS_GetOtherSpeedCfgDesc (uint16_t *length);

uint8_t  *USBD_GREYBUS_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_GREYBUS_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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
  * @}
  */

/** @defgroup USBD_GREYBUS_Private_Variables
  * @{
  */


/* GREYBUS interface class callbacks structure */
USBD_ClassTypeDef  USBD_GREYBUS =
{
  USBD_GREYBUS_Init,
  USBD_GREYBUS_DeInit,
  USBD_GREYBUS_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_GREYBUS_EP0_RxReady,
  USBD_GREYBUS_DataIn,
  USBD_GREYBUS_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_GREYBUS_GetHSCfgDesc,
  USBD_GREYBUS_GetFSCfgDesc,
  USBD_GREYBUS_GetOtherSpeedCfgDesc,
  USBD_GREYBUS_GetDeviceQualifierDescriptor,
};

/* USB GREYBUS device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_GREYBUS_CfgHSDesc[USB_GREYBUS_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_GREYBUS_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: 1 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x04,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0xFF,   /* bInterfaceClass: */
  0xFF,   /* bInterfaceSubClass: */
  0xFF,   /* bInterfaceProtocol: */
  0x05,   /* iInterface: */

  /*CPORT IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  GREYBUS_CPORT_IN_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(GREYBUS_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(GREYBUS_DATA_HS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*CPORT OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  GREYBUS_CPORT_OUT_EP,              /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(GREYBUS_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(GREYBUS_DATA_HS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
} ;


/* USB GREYBUS device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_GREYBUS_CfgFSDesc[USB_GREYBUS_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_GREYBUS_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: 1 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x04,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0xFF,   /* bInterfaceClass: */
  0xFF,   /* bInterfaceSubClass: */
  0xFF,   /* bInterfaceProtocol: */
  0x05,   /* iInterface: */

  /*CPORT IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  GREYBUS_CPORT_IN_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(GREYBUS_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(GREYBUS_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*CPORT OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  GREYBUS_CPORT_OUT_EP,              /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(GREYBUS_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(GREYBUS_DATA_FS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
} ;

__ALIGN_BEGIN uint8_t USBD_GREYBUS_OtherSpeedCfgDesc[USB_GREYBUS_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_GREYBUS_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x01,   /* bNumInterfaces: 1 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x04,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0xFF,   /* bInterfaceClass: */
  0xFF,   /* bInterfaceSubClass: */
  0xFF,   /* bInterfaceProtocol: */
  0x05,   /* iInterface: */

  /*CPORT IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  GREYBUS_CPORT_IN_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(GREYBUS_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(GREYBUS_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*CPORT OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  GREYBUS_CPORT_OUT_EP,              /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(GREYBUS_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(GREYBUS_DATA_FS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
};

/**
  * @}
  */

/** @defgroup USBD_GREYBUS_Private_Functions
  * @{
  */

/**
  * @brief  USBD_GREYBUS_Init
  *         Initialize the GREYBUS interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_GREYBUS_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_GREYBUS_HandleTypeDef   *hgreybus;

  if(pdev->dev_speed == USBD_SPEED_HIGH  )
  {
    /* Open EP CPORT IN */
    USBD_LL_OpenEP(pdev,
                   GREYBUS_CPORT_IN_EP,
                   USBD_EP_TYPE_BULK,
                   GREYBUS_DATA_HS_OUT_PACKET_SIZE);

    /* Open EP CPORT OUT */
    USBD_LL_OpenEP(pdev,
                   GREYBUS_CPORT_OUT_EP,
                   USBD_EP_TYPE_BULK,
                   GREYBUS_DATA_HS_OUT_PACKET_SIZE);

  }
  else
  {
	  /* Open EP CPORT IN */
	  USBD_LL_OpenEP(pdev,
					 GREYBUS_CPORT_IN_EP,
					 USBD_EP_TYPE_BULK,
					 GREYBUS_DATA_FS_OUT_PACKET_SIZE);

	  /* Open EP CPORT OUT */
	  USBD_LL_OpenEP(pdev,
					 GREYBUS_CPORT_OUT_EP,
					 USBD_EP_TYPE_BULK,
					 GREYBUS_DATA_FS_OUT_PACKET_SIZE);
  }

  pdev->pClassData = USBD_malloc(sizeof (USBD_GREYBUS_HandleTypeDef));

  if(pdev->pClassData == NULL)
  {
    ret = 1;
  }
  else
  {
    hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

    /* Init  physical Interface components */
    ((USBD_GREYBUS_ItfTypeDef *)pdev->pUserData)->Init();

    /* Init Xfer states */
    hgreybus->TxState =0;
    hgreybus->RxState =0;

    if(pdev->dev_speed == USBD_SPEED_HIGH  )
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
    		  	  	  	     GREYBUS_CPORT_OUT_EP,
                             hgreybus->RxBuffer,
							 4 * 1024);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
    		  	  	  	  	 GREYBUS_CPORT_OUT_EP,
                             hgreybus->RxBuffer,
							 4 * 1024);
    }


  }
  return ret;
}

/**
  * @brief  USBD_GREYBUS_Init
  *         DeInitialize the GREYBUS layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_GREYBUS_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{
  uint8_t ret = 0;

  /* Open EP CPORT IN */
  USBD_LL_CloseEP(pdev,
              GREYBUS_CPORT_IN_EP);

  /* Open EP CPORT OUT */
  USBD_LL_CloseEP(pdev,
              GREYBUS_CPORT_OUT_EP);


  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_GREYBUS_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return ret;
}

/**
  * @brief  USBD_GREYBUS_Setup
  *         Handle the GREYBUS specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_GREYBUS_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;
  static uint8_t ifalt = 0;



  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_VENDOR :
  	printf("%s(USB_REQ_TYPE_VENDOR, wIndex=%d, wLength=%d, bmRequest=%x)\r\n", __func__,
  			req->wIndex, req->wLength, req->bmRequest);
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        ((USBD_GREYBUS_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                          (uint8_t *)hgreybus->data,
														  req->wIndex,
                                                          req->wLength);
          printf("%s() send data %d [%02x, %02x]\r\n", __func__, req->wLength,
        		  ((uint8_t *)hgreybus->data)[0], ((uint8_t *)hgreybus->data)[1]);
          USBD_CtlSendData (pdev,
                            (uint8_t *)hgreybus->data,
                            req->wLength);
      }
      else
      {
        hgreybus->CmdOpCode = req->bRequest;
        hgreybus->CmdIndex = req->wIndex;
        hgreybus->CmdLength = req->wLength;

        printf("%s() prepare rx data %d\r\n", __func__, req->wLength);

        USBD_CtlPrepareRx (pdev,
                           (uint8_t *)hgreybus->data,
                           req->wLength);
      }

    }
    else
    {
      ((USBD_GREYBUS_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                        (uint8_t*)req,
														req->wIndex,
                                                        0);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_INTERFACE :
    	printf("%s(USB_REQ_GET_INTERFACE)\r\n");
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;

    case USB_REQ_SET_INTERFACE :
    	printf("%s(USB_REQ_SET_INTERFACE)\r\n");
    	ifalt = (uint8_t)(req->wValue);
      break;
    }

  default:
  	printf("%s(USB_REQ_TYPE_STANDARD)=%d\r\n", req->bRequest);
    break;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_GREYBUS_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_GREYBUS_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

  if(pdev->pClassData != NULL)
  {
	  // Force zlp if multiple of 64
	  if (hgreybus->TxLength >= 64 && hgreybus->TxLength % 64 == 0) {
		  hgreybus->TxLength = 0;

		  /* Transmit next packet */
		  USBD_LL_Transmit(pdev,
	                         GREYBUS_CPORT_IN_EP,
	                         hgreybus->TxBuffer,
	                         0);
	  }
	  else
		  hgreybus->TxState = 0;

	  return USBD_OK;
  }
  else
  {
	  return USBD_FAIL;
  }
}

/**
  * @brief  USBD_GREYBUS_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_GREYBUS_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

  /* Get the received data length */
  hgreybus->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {
    ((USBD_GREYBUS_ItfTypeDef *)pdev->pUserData)->Receive(hgreybus->RxBuffer, &hgreybus->RxLength);

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}



/**
  * @brief  USBD_GREYBUS_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_GREYBUS_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

  if((pdev->pUserData != NULL) && (hgreybus->CmdOpCode != 0xFF))
  {
    ((USBD_GREYBUS_ItfTypeDef *)pdev->pUserData)->Control(hgreybus->CmdOpCode,
                                                      (uint8_t *)hgreybus->data,
													  hgreybus->CmdIndex,
                                                      hgreybus->CmdLength);
      hgreybus->CmdOpCode = 0xFF;

  }
  return USBD_OK;
}

/**
  * @brief  USBD_GREYBUS_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_GREYBUS_GetFSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_GREYBUS_CfgFSDesc);
  printf("%s()=%d\r\n", __func__, *length);
  return USBD_GREYBUS_CfgFSDesc;
}

/**
  * @brief  USBD_GREYBUS_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_GREYBUS_GetHSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_GREYBUS_CfgHSDesc);
  printf("%s()=%d\r\n", __func__, *length);
  return USBD_GREYBUS_CfgHSDesc;
}

/**
  * @brief  USBD_GREYBUS_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_GREYBUS_GetOtherSpeedCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_GREYBUS_OtherSpeedCfgDesc);
  printf("%s()=%d\r\n", __func__, *length);
  return USBD_GREYBUS_OtherSpeedCfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_GREYBUS_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_GREYBUS_DeviceQualifierDesc);
  printf("%s()=%d\r\n", __func__, *length);
  return USBD_GREYBUS_DeviceQualifierDesc;
}

/**
* @brief  USBD_GREYBUS_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_GREYBUS_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                      USBD_GREYBUS_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  printf("%s()\r\n",__func__);

  if(fops != NULL)
  {
    pdev->pUserData= fops;
    ret = USBD_OK;
  }

  return ret;
}

/**
  * @brief  USBD_GREYBUS_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_GREYBUS_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

  hgreybus->TxBuffer = pbuff;
  hgreybus->TxLength = length;

  return USBD_OK;
}


/**
  * @brief  USBD_GREYBUS_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_GREYBUS_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

  hgreybus->RxBuffer = pbuff;

  return USBD_OK;
}

/**
  * @brief  USBD_GREYBUS_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_GREYBUS_GetTransmitState(USBD_HandleTypeDef *pdev)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

  if(pdev->pClassData != NULL)
  {
    if(hgreybus->TxState == 0)
    {
      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_GREYBUS_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_GREYBUS_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

  if(pdev->pClassData != NULL)
  {
    if(hgreybus->TxState == 0)
    {
      /* Tx Transfer in progress */
      hgreybus->TxState = 1;

      /* Transmit next packet */
      USBD_LL_Transmit(pdev,
                       GREYBUS_CPORT_IN_EP,
                       hgreybus->TxBuffer,
                       hgreybus->TxLength);

      return USBD_OK;
    }
    else
    {
    	printf("TX Busy !!!!!!\r\n");
    	return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}


/**
  * @brief  USBD_GREYBUS_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_GREYBUS_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_GREYBUS_HandleTypeDef   *hgreybus = (USBD_GREYBUS_HandleTypeDef*) pdev->pClassData;

  /* Suspend or Resume USB Out process */
  if(pdev->pClassData != NULL)
  {
    if(pdev->dev_speed == USBD_SPEED_HIGH  )
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             GREYBUS_CPORT_OUT_EP,
                             hgreybus->RxBuffer,
                             4 * 1204);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             GREYBUS_CPORT_OUT_EP,
                             hgreybus->RxBuffer,
							 4 * 1204);
    }
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
