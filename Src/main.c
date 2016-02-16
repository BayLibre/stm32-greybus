/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_greybus.h"
#include "greybus/greybus.h"
#include "endian.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int verbose = 1;
struct gbsim_interface interface;

/* Sample manifest */
unsigned char examples_IID1_simple_i2c_module_mnfb[] = {
  0x4c, 0x00, 0x00, 0x01, 0x08, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x00,
  0x14, 0x00, 0x02, 0x00, 0x0b, 0x01, 0x50, 0x72, 0x6f, 0x6a, 0x65, 0x63,
  0x74, 0x20, 0x41, 0x72, 0x61, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x02, 0x00,
  0x14, 0x02, 0x53, 0x69, 0x6d, 0x70, 0x6c, 0x65, 0x20, 0x49, 0x32, 0x43,
  0x20, 0x49, 0x6e, 0x74, 0x65, 0x72, 0x66, 0x61, 0x63, 0x65, 0x00, 0x00,
  0x08, 0x00, 0x04, 0x00, 0x01, 0x00, 0x01, 0x03, 0x08, 0x00, 0x03, 0x00,
  0x01, 0x03, 0x00, 0x00
};
unsigned int examples_IID1_simple_i2c_module_mnfb_len = 76;

int manifest_startup = 0;
int svc_comm_startup = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t GREYBUS_Init_FS     (void);
static int8_t GREYBUS_DeInit_FS   (void);
static int8_t GREYBUS_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t index, uint16_t length);
static int8_t GREYBUS_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN 0 */
USBD_GREYBUS_ItfTypeDef USBD_Interface_fops_FS =
{
  GREYBUS_Init_FS,
  GREYBUS_DeInit_FS,
  GREYBUS_Control_FS,
  GREYBUS_Receive_FS
};

#define ES1_MSG_SIZE	(4 * 1024)
static uint8_t cport_rbuf[ES1_MSG_SIZE];
uint8_t cport_tbuf[ES1_MSG_SIZE];

static int8_t GREYBUS_Init_FS     (void)
{
	fprintf(stderr, "%s\r\n", __func__);
	USBD_GREYBUS_SetRxBuffer(&hUsbDeviceFS, cport_rbuf);
	USBD_GREYBUS_SetTxBuffer(&hUsbDeviceFS, cport_tbuf, ES1_MSG_SIZE);
}

static int8_t GREYBUS_DeInit_FS   (void)
{
	fprintf(stderr, "%s\r\n", __func__);
}

static int dump_control_msg(uint8_t *buf, uint16_t count)
{
	if (1) {
		printf("AP->SVC message:\r\n");
		for (int i = 0; i < count; i++)
			fprintf(stdout, "%02x ", buf[i]);
		fprintf(stdout, "\n");
	}

	return count;
}

/* vendor request APB1 log */
#define REQUEST_LOG		0x02
/* vendor request to map a cport to bulk in and bulk out endpoints */
#define REQUEST_EP_MAPPING	0x03
/* vendor request to get the number of cports available */
#define REQUEST_CPORT_COUNT	0x04
/* vendor request to reset a cport state */
#define REQUEST_RESET_CPORT	0x05
/* vendor request to time the latency of messages on a given cport */
#define REQUEST_LATENCY_TAG_EN	0x06
#define REQUEST_LATENCY_TAG_DIS	0x07

static int8_t GREYBUS_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t index, uint16_t length)
{
	uint16_t *u16val = (uint16_t*)pbuf;

	fprintf(stderr, "%s\r\n", __func__);
	printf("AP->AP Bridge setup message:\r\n");
	printf("  bRequestType = %02x\r\n", USB_REQ_TYPE_VENDOR);
	printf("  bRequest     = %02x\r\n", cmd);
	printf("  wValue       = %p\r\n", pbuf);
	printf("  wIndex       = %04x\r\n", index);
	printf("  wLength      = %04x\r\n", length);

	switch (cmd) {
	case REQUEST_LOG:
		printf("log request, nothing to do\r\n");
		break;
	case REQUEST_EP_MAPPING:
		dump_control_msg(pbuf, length);
		printf("ep_mapping request, nothing to do\r\n");
		break;
	case REQUEST_CPORT_COUNT:
		*u16val = htole16(16);
		printf("cport count request, reply 16\r\n");

		svc_comm_startup  = 1;
		break;
	case REQUEST_RESET_CPORT:
		dump_control_msg(pbuf, length);
		printf("reset_cport request for cport: %04x\r\n",
				le16toh(*u16val));
		break;
	case REQUEST_LATENCY_TAG_EN:
		dump_control_msg(pbuf, length);
		printf("latency_tag_en request for cport: %04x\r\n",
				le16toh(*u16val));
		break;
	case REQUEST_LATENCY_TAG_DIS:
		dump_control_msg(pbuf, length);
		printf("latency_tag_dis request for cport: %04x\r\n",
				le16toh(*u16val));
		break;
	default:
		printf("Invalid request type %02x\r\n", cmd);
	}
}

int GREYBUS_Transmit_FS (uint8_t* pbuf, uint32_t Len)
{
	while(USBD_GREYBUS_GetTransmitState(&hUsbDeviceFS) != USBD_OK);

	memcpy(cport_tbuf, pbuf, Len);

	USBD_GREYBUS_SetTxBuffer(&hUsbDeviceFS, cport_tbuf, Len);
	USBD_GREYBUS_TransmitPacket(&hUsbDeviceFS);

	return 0;
}

static int8_t GREYBUS_Receive_FS  (uint8_t* pbuf, uint32_t *Len)
{
	fprintf(stderr, "%s(%d)\n", __func__, *Len);

	recv_handler(pbuf, *Len);

	/* Restart OUT EP */
	USBD_GREYBUS_ReceivePacket(&hUsbDeviceFS);
}

static int get_interface_id(char *fname)
{
	char *iid_str;
	int iid = 0;
	char tmp[256];

	strcpy(tmp, fname);
	iid_str = strtok(tmp, "-");
	if (!strncmp(iid_str, "IID", 3))
		iid = strtol(iid_str+3, NULL, 0);

	return iid;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  printf("Hello World GREYBUS!\r\n");

  TAILQ_INIT(&interface.connections);

  svc_init();

  USBD_GREYBUS_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);

  USBD_Start(&hUsbDeviceFS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  HAL_Delay(10);

	  if (svc_comm_startup) {
		  printf("SVC Hello Startup\r\n");
		  /*
		   * Start communication with the AP in following sequence:
		   * - Send a svc protocol version request
		   * - For a valid response, send the 'hello' message.
		   */
		  svc_request_send(GB_REQUEST_TYPE_PROTOCOL_VERSION, AP_INTF_ID);

		  svc_comm_startup = 0;
	  }

	  if (manifest_startup) {
		  printf("Manifest Startup\r\n");
		  interface.manifest = examples_IID1_simple_i2c_module_mnfb;
		  interface.manifest_size = examples_IID1_simple_i2c_module_mnfb_len;
		  manifest_parse(examples_IID1_simple_i2c_module_mnfb, examples_IID1_simple_i2c_module_mnfb_len);
		  int iid = get_interface_id("IID1");
		  printf("Manifest iid %d\r\n", iid);
		  if (iid > 0) {
			  svc_request_send(GB_SVC_TYPE_INTF_HOTPLUG, iid);
		  }
		  manifest_startup = 0;
	  }

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
