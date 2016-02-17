
/*
 * Greybus Simulator
 *
 * Copyright 2014 Google Inc.
 * Copyright 2014 Linaro Ltd.
 *
 * Provided under the three clause BSD license found in the LICENSE file.
 */

#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

#include "greybus/greybus.h"
#include "endian.h"

#include "tim.h"

TIM_OC_InitTypeDef sConfigOC;
unsigned pwm_on = 0;

#define TIM3_CLK_MHz 72	//MHz
#define TIM3_CLK_NS(t)	((72 * (t)) / 1000)

void pwm_init(void)
{
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.Period = 0;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	//HAL_TIM_PWM_Init(&htim3);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	//HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}

int pwm_handler(struct gbsim_connection *connection, void *rbuf,
		size_t rsize, void *tbuf, size_t tsize)
{
	struct gb_operation_msg_hdr *oph;
	struct op_msg *op_req = rbuf;
	struct op_msg *op_rsp;
	__u32 duty;
	__u32 period;
	size_t payload_size;
	uint16_t message_size;
	uint16_t hd_cport_id = connection->hd_cport_id;
	uint8_t result = PROTOCOL_STATUS_SUCCESS;

	op_rsp = (struct op_msg *)tbuf;
	oph = (struct gb_operation_msg_hdr *)&op_req->header;

	switch (oph->type) {
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		payload_size = sizeof(struct gb_protocol_version_response);
		op_rsp->pv_rsp.major = GREYBUS_VERSION_MAJOR;
		op_rsp->pv_rsp.minor = GREYBUS_VERSION_MINOR;
		break;
	case GB_PWM_TYPE_PWM_COUNT:
		payload_size = sizeof(struct gb_pwm_count_response);
		op_rsp->pwm_cnt_rsp.count = 0; /* Something arbitrary, but useful */
		break;
	case GB_PWM_TYPE_ACTIVATE:
		payload_size = 0;
		gbsim_debug("PWM %d activate request\n  ",
			    op_req->pwm_act_req.which);
		break;
	case GB_PWM_TYPE_DEACTIVATE:
		payload_size = 0;
		gbsim_debug("PWM %d deactivate request\n  ",
			    op_req->pwm_deact_req.which);
		break;
	case GB_PWM_TYPE_CONFIG:
		payload_size = 0;
		duty = le32toh(op_req->pwm_cfg_req.duty);
		period = le32toh(op_req->pwm_cfg_req.period);
		htim3.Init.Period = TIM3_CLK_NS(period);
		sConfigOC.Pulse = TIM3_CLK_NS(duty);
		gbsim_debug("PWM %d config (%dns/%dns) request\n  ",
			    op_req->pwm_cfg_req.which, duty, period);
		break;
	case GB_PWM_TYPE_POLARITY:
		payload_size = 0;
		if (pwm_on) {
			result = PROTOCOL_STATUS_BUSY;
		} else {
			if (op_req->pwm_pol_req.polarity)
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			else
				sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
		}
		gbsim_debug("PWM %d polarity (%s) request\n  ",
			    op_req->pwm_cfg_req.which,
			    op_req->pwm_pol_req.polarity ? "inverse" : "normal");
		break;
	case GB_PWM_TYPE_ENABLE:
		payload_size = 0;
		pwm_on = 1;
		HAL_TIM_PWM_MspInit(&htim3);
		gbsim_debug("PWM %d enable request\n  ",
			    op_req->pwm_enb_req.which);
		break;
	case GB_PWM_TYPE_DISABLE:
		payload_size = 0;
		pwm_on = 0;
		HAL_TIM_PWM_MspDeInit(&htim3);
		gbsim_debug("PWM %d disable request\n  ",
			    op_req->pwm_dis_req.which);
		break;
	default:
		gbsim_error("pwm operation type %02x not supported\n", oph->type);
		return -EINVAL;
	}

	message_size = sizeof(struct gb_operation_msg_hdr) + payload_size;
	return send_response(hd_cport_id, op_rsp, message_size,
				oph->operation_id, oph->type, result);
}

char *pwm_get_operation(uint8_t type)
{
	switch (type) {
	case GB_REQUEST_TYPE_INVALID:
		return "GB_PWM_TYPE_INVALID";
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		return "GB_PWM_TYPE_PROTOCOL_VERSION";
	case GB_PWM_TYPE_PWM_COUNT:
		return "GB_PWM_TYPE_PWM_COUNT";
	case GB_PWM_TYPE_ACTIVATE:
		return "GB_PWM_TYPE_ACTIVATE";
	case GB_PWM_TYPE_DEACTIVATE:
		return "GB_PWM_TYPE_DEACTIVATE";
	case GB_PWM_TYPE_CONFIG:
		return "GB_PWM_TYPE_CONFIG";
	case GB_PWM_TYPE_POLARITY:
		return "GB_PWM_TYPE_POLARITY";
	case GB_PWM_TYPE_ENABLE:
		return "GB_PWM_TYPE_ENABLE";
	case GB_PWM_TYPE_DISABLE:
		return "GB_PWM_TYPE_DISABLE";
	default:
		return "(Unknown operation)";
	}
}
