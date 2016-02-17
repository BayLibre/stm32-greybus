
/*
 * Greybus Simulator
 *
 * Copyright 2014 Google Inc.
 * Copyright 2014 Linaro Ltd.
 *
 * Provided under the three clause BSD license found in the LICENSE file.
 */

#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include "endian.h"

#include "greybus/greybus.h"
#include "gpio.h"

#define GPIO_COUNT	23

static struct _gpio_map {
	unsigned pin;
	unsigned bank;
	GPIO_InitTypeDef GPIO_InitStruct;
} gpio_map[GPIO_COUNT] =
{
		{ GPIO_PIN_2, GPIOE},
		{ GPIO_PIN_4, GPIOE},
		{ GPIO_PIN_5, GPIOE},
		{ GPIO_PIN_6, GPIOE},
		{ GPIO_PIN_7, GPIOE},
		{ GPIO_PIN_8, GPIOE},
		{ GPIO_PIN_9, GPIOE},
		{ GPIO_PIN_10, GPIOE},
		{ GPIO_PIN_11, GPIOE},
		{ GPIO_PIN_12, GPIOE},
		{ GPIO_PIN_13, GPIOE},
		{ GPIO_PIN_14, GPIOE},
		{ GPIO_PIN_15, GPIOE},
		{ GPIO_PIN_0, GPIOD},
		{ GPIO_PIN_1, GPIOD},
		{ GPIO_PIN_2, GPIOD},
		{ GPIO_PIN_3, GPIOD},
		{ GPIO_PIN_6, GPIOD},
		{ GPIO_PIN_7, GPIOD},
		{ GPIO_PIN_8, GPIOD},
		{ GPIO_PIN_9, GPIOD},
		{ GPIO_PIN_10, GPIOD},
		{ GPIO_PIN_11, GPIOD},
};

int gpio_handler(struct gbsim_connection *connection, void *rbuf,
		 size_t rsize, void *tbuf, size_t tsize)
{
	struct gb_operation_msg_hdr *oph;
	struct op_msg *op_req = rbuf;
	struct op_msg *op_rsp;
	size_t payload_size;
	ssize_t nbytes;
	uint16_t message_size;
	uint16_t hd_cport_id = connection->hd_cport_id;
	unsigned gpiopin = 0;
	int status = PROTOCOL_STATUS_SUCCESS;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Store connection for report
	gpio_connection = connection;

	op_rsp = (struct op_msg *)tbuf;
	oph = (struct gb_operation_msg_hdr *)&op_req->header;

	switch (oph->type) {
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		payload_size = sizeof(struct gb_protocol_version_response);
		op_rsp->pv_rsp.major = GREYBUS_VERSION_MAJOR;
		op_rsp->pv_rsp.minor = GREYBUS_VERSION_MINOR;
		break;
	case GB_GPIO_TYPE_LINE_COUNT:
		payload_size = sizeof(struct gb_gpio_line_count_response);
		op_rsp->gpio_lc_rsp.count = GPIO_COUNT-1; /* Something arbitrary, but useful */
		break;
	case GB_GPIO_TYPE_ACTIVATE:
		payload_size = 0;
		gbsim_debug("GPIO %d activate request\n  ",
			    op_req->gpio_act_req.which);
		break;
	case GB_GPIO_TYPE_DEACTIVATE:
		payload_size = 0;
		gbsim_debug("GPIO %d deactivate request\n  ",
			    op_req->gpio_deact_req.which);
		break;
	case GB_GPIO_TYPE_GET_DIRECTION:
		payload_size = sizeof(struct gb_gpio_get_direction_response);
		gpiopin = op_req->gpio_dir_output_req.which;
		if (gpiopin > GPIO_COUNT) {
			status = PROTOCOL_STATUS_INVALID;
			break;
		}

		if (gpio_map[gpiopin].GPIO_InitStruct.Mode == GPIO_MODE_INPUT)
			op_rsp->gpio_get_dir_rsp.direction = 0;
		else
			op_rsp->gpio_get_dir_rsp.direction = 1;

		gbsim_debug("GPIO %d get direction (%d) response\n  ",
			    op_req->gpio_get_dir_req.which, op_rsp->gpio_get_dir_rsp.direction);
		break;
	case GB_GPIO_TYPE_DIRECTION_IN:
		gpiopin = op_req->gpio_dir_output_req.which;
		if (gpiopin > GPIO_COUNT) {
			status = PROTOCOL_STATUS_INVALID;
			break;
		}
		payload_size = 0;
		gbsim_debug("GPIO %d direction input request\n  ",
			    op_req->gpio_dir_input_req.which);

		gpio_map[gpiopin].GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_Init(gpio_map[gpiopin].bank, &gpio_map[gpiopin].GPIO_InitStruct);
		break;
	case GB_GPIO_TYPE_DIRECTION_OUT:
		gpiopin = op_req->gpio_dir_output_req.which;
		if (gpiopin > GPIO_COUNT) {
			status = PROTOCOL_STATUS_INVALID;
			break;
		}
		payload_size = 0;
		gbsim_debug("GPIO %d direction output request\n  ",
			    op_req->gpio_dir_output_req.which);

		gpio_map[gpiopin].GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(gpio_map[gpiopin].bank, &gpio_map[gpiopin].GPIO_InitStruct);
		break;
	case GB_GPIO_TYPE_GET_VALUE:
		gpiopin = op_req->gpio_dir_output_req.which;
		if (gpiopin > GPIO_COUNT) {
			status = PROTOCOL_STATUS_INVALID;
			break;
		}
		payload_size = sizeof(struct gb_gpio_get_value_response);

		op_rsp->gpio_get_val_rsp.value = HAL_GPIO_ReadPin(gpio_map[gpiopin].bank, gpio_map[gpiopin].pin);
		gbsim_debug("GPIO %d get value (%d) response\n  ",
			    op_req->gpio_get_val_req.which, op_rsp->gpio_get_val_rsp.value);
		break;
	case GB_GPIO_TYPE_SET_VALUE:
		gpiopin = op_req->gpio_dir_output_req.which;
		if (gpiopin > GPIO_COUNT) {
			status = PROTOCOL_STATUS_INVALID;
			break;
		}
		payload_size = 0;
		gbsim_debug("GPIO %d set value (%d) request\n  ",
			    op_req->gpio_set_val_req.which, op_req->gpio_set_val_req.value);
		HAL_GPIO_WritePin(gpio_map[gpiopin].bank, gpio_map[gpiopin].pin, (op_req->gpio_set_val_req.value?GPIO_PIN_SET:0));
		break;
	case GB_GPIO_TYPE_SET_DEBOUNCE:
		payload_size = 0;
		gbsim_debug("GPIO %d set debounce (%d us) request\n  ",
			    op_req->gpio_set_db_req.which, op_req->gpio_set_db_req.usec);
		break;
	case GB_GPIO_TYPE_IRQ_TYPE:
		payload_size = 0;
		gbsim_debug("GPIO protocol IRQ type %d request\n  ",
			    op_req->gpio_irq_type_req.type);
		break;
	case GB_GPIO_TYPE_IRQ_MASK:
		payload_size = 0;
		break;
	case GB_GPIO_TYPE_IRQ_UNMASK:
		payload_size = 0;
		break;
	default:
		return -EINVAL;
	}

	message_size = sizeof(struct gb_operation_msg_hdr) + payload_size;
	nbytes = send_response(hd_cport_id, op_rsp, message_size,
				oph->operation_id, oph->type,
				status);
	if (nbytes)
		return nbytes;

	return 0;
}

char *gpio_get_operation(uint8_t type)
{
	switch (type) {
	case GB_REQUEST_TYPE_INVALID:
		return "GB_GPIO_TYPE_INVALID";
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		return "GB_GPIO_TYPE_PROTOCOL_VERSION";
	case GB_GPIO_TYPE_LINE_COUNT:
		return "GB_GPIO_TYPE_LINE_COUNT";
	case GB_GPIO_TYPE_ACTIVATE:
		return "GB_GPIO_TYPE_ACTIVATE";
	case GB_GPIO_TYPE_DEACTIVATE:
		return "GB_GPIO_TYPE_DEACTIVATE";
	case GB_GPIO_TYPE_GET_DIRECTION:
		return "GB_GPIO_TYPE_GET_DIRECTION";
	case GB_GPIO_TYPE_DIRECTION_IN:
		return "GB_GPIO_TYPE_DIRECTION_IN";
	case GB_GPIO_TYPE_DIRECTION_OUT:
		return "GB_GPIO_TYPE_DIRECTION_OUT";
	case GB_GPIO_TYPE_GET_VALUE:
		return "GB_GPIO_TYPE_GET_VALUE";
	case GB_GPIO_TYPE_SET_VALUE:
		return "GB_GPIO_TYPE_SET_VALUE";
	case GB_GPIO_TYPE_SET_DEBOUNCE:
		return "GB_GPIO_TYPE_SET_DEBOUNCE";
	case GB_GPIO_TYPE_IRQ_TYPE:
		return "GB_GPIO_TYPE_IRQ_TYPE";
	case GB_GPIO_TYPE_IRQ_MASK:
		return "GB_GPIO_TYPE_IRQ_MASK";
	case GB_GPIO_TYPE_IRQ_UNMASK:
		return "GB_GPIO_TYPE_IRQ_UNMASK";
	case GB_GPIO_TYPE_IRQ_EVENT:
		return "GB_GPIO_TYPE_IRQ_EVENT";
	default:
		return "(Unknown operation)";
	}
}

void gpio_init(void)
{
	int i;

	for (i = 0 ; i < GPIO_COUNT ; i++) {
		gpio_map[i].GPIO_InitStruct.Pin = gpio_map[i].pin;
		gpio_map[i].GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		gpio_map[i].GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(gpio_map[i].bank, &gpio_map[i].GPIO_InitStruct);
	}
}
