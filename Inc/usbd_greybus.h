/**
  ******************************************************************************
  * @file    usbd_greybus.h
  * @author  Neil Armstrong <narmstrong@baylibre.com>
  * @version V1
  * @date    2016
  * @brief   header file for the usbd_greybus.c file.
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT 2016 Neil Armstrong <narmstrong@baylibre.com>
  *
  ******************************************************************************
  */
#ifndef INC_USBD_GREYBUS_H_
#define INC_USBD_GREYBUS_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_cdc
  * @brief This file is the Header file for usbd_cdc.c
  * @{
  */


/** @defgroup usbd_cdc_Exported_Defines
  * @{
  */
#define GREYBUS_CPORT_IN_EP                                 0x81  /* EP1 for CPORT IN */
#define GREYBUS_CPORT_OUT_EP                                0x01  /* EP1 for CPORT OUT */

/* GREYBUS Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define GREYBUS_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define GREYBUS_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */

#define USB_GREYBUS_CONFIG_DESC_SIZ                     32
#define GREYBUS_DATA_HS_IN_PACKET_SIZE                  GREYBUS_DATA_HS_MAX_PACKET_SIZE
#define GREYBUS_DATA_HS_OUT_PACKET_SIZE                 GREYBUS_DATA_HS_MAX_PACKET_SIZE

#define GREYBUS_DATA_FS_IN_PACKET_SIZE                  GREYBUS_DATA_FS_MAX_PACKET_SIZE
#define GREYBUS_DATA_FS_OUT_PACKET_SIZE                 GREYBUS_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  GREYBUS definitions                                                    */
/*---------------------------------------------------------------------*/
#define GREYBUS_SEND_ENCAPSULATED_COMMAND               0x00
#define GREYBUS_GET_ENCAPSULATED_RESPONSE               0x01
#define GREYBUS_SET_COMM_FEATURE                        0x02
#define GREYBUS_GET_COMM_FEATURE                        0x03
#define GREYBUS_CLEAR_COMM_FEATURE                      0x04
#define GREYBUS_SET_LINE_CODING                         0x20
#define GREYBUS_GET_LINE_CODING                         0x21
#define GREYBUS_SET_CONTROL_LINE_STATE                  0x22
#define GREYBUS_SEND_BREAK                              0x23

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */

typedef struct _USBD_GREYBUS_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t, uint16_t);
  int8_t (* Receive)       (uint8_t *, uint32_t *);

}USBD_GREYBUS_ItfTypeDef;


typedef struct
{
  uint32_t data[GREYBUS_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdIndex;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
}
USBD_GREYBUS_HandleTypeDef;



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_GREYBUS;
#define USBD_GREYBUS_CLASS    &USBD_GREYBUS
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_GREYBUS_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                      USBD_GREYBUS_ItfTypeDef *fops);

uint8_t  USBD_GREYBUS_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_GREYBUS_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);

uint8_t  USBD_GREYBUS_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_GREYBUS_TransmitPacket     (USBD_HandleTypeDef *pdev);

uint8_t  USBD_GREYBUS_GetTransmitState   (USBD_HandleTypeDef *pdev);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* INC_USBD_GREYBUS_H_ */
