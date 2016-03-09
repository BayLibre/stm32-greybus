
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

/*
#define HID_REPORT_LENGTH	28
static char hid_report[] = {
	    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	    0x09, 0x06,                    // USAGE (Keyboard)
	    0xa1, 0x01,                    // COLLECTION (Application)
	    0x09, 0x07,                    //   USAGE (Keypad)
	    0xa1, 0x00,                    //   COLLECTION (Physical)
	    0x05, 0x09,                    //     USAGE_PAGE (Button)
	    0x19, 0x00,                    //     USAGE_MINIMUM (No Buttons Pressed)
	    0x29, 0x01,                    //     USAGE_MAXIMUM (Button 1)
	    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
	    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
	    0x95, 0x01,                    //     REPORT_COUNT (1)
	    0x75, 0x01,                    //     REPORT_SIZE (1)
	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	    0xc0,                          //   END_COLLECTION
	    0xc0                           // END_COLLECTION
};
 */

#define HID_REPORT_LENGTH	48
static char hid_report[] = {
	    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	    0x09, 0x05,                    // USAGE (Game Pad)
	    0xa1, 0x01,                    // COLLECTION (Application)
	    0xa1, 0x00,                    //   COLLECTION (Physical)
	    0x05, 0x09,                    //     USAGE_PAGE (Button)
	    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
	    0x29, 0x02,                    //     USAGE_MAXIMUM (Button 2)
	    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
	    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
	    0x95, 0x02,                    //     REPORT_COUNT (2)
	    0x75, 0x01,                    //     REPORT_SIZE (1)
	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	    0x95, 0x01,                    //     REPORT_COUNT (1)
	    0x75, 0x06,                    //     REPORT_SIZE (6)
	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
	    0x09, 0x30,                    //     USAGE (X)
	    0x09, 0x31,                    //     USAGE (Y)
	    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
	    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
	    0x75, 0x08,                    //     REPORT_SIZE (8)
	    0x95, 0x02,                    //     REPORT_COUNT (2)
	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	    0xc0,                          //     END_COLLECTION
	    0xc0                           // END_COLLECTION
};

uint8_t hid_status[3];

#define HID_MSG_SIZE	(sizeof(struct gb_operation_msg_hdr)+2)
uint8_t hid_tbuf[HID_MSG_SIZE];

int hid_report_button(struct gbsim_connection *connection, uint8_t *values)
{
	void *tbuf = &hid_tbuf[0];
	size_t tsize = sizeof(hid_tbuf);
	struct op_msg *msg = (struct op_msg *)tbuf;
	struct gb_operation_msg_hdr *oph = &msg->header;
	uint16_t message_size = sizeof(*oph);
	size_t payload_size;

	//gbsim_info("hid_report_button(%d,%d,%d)\r\n", values[0],values[1],values[2]);
	memcpy(hid_status, values, 3);

	memset(tbuf, 0, tsize);	/* Zero buffer before use */

	oph->type = GB_HID_TYPE_IRQ_EVENT;

	payload_size = 3;
	memcpy(&msg->hid_input_report_req.report[0], values, 3);

	message_size = sizeof(struct gb_operation_msg_hdr) + payload_size;
	return send_request(connection->hd_cport_id, msg, message_size, 0,
			oph->type);
}

#define VERSION_BCD(Major, Minor, Revision) \
		htole16( ((Major & 0xFF) << 8) | \
				((Minor & 0x0F) << 4) | \
				(Revision & 0x0F) )

int hid_handler(struct gbsim_connection *connection, void *rbuf,
		size_t rsize, void *tbuf, size_t tsize)
{
	struct gb_operation_msg_hdr *oph;
	struct op_msg *op_req = rbuf;
	struct op_msg *op_rsp;
	size_t payload_size = 0;
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
	case GB_HID_TYPE_GET_DESC:
		payload_size = sizeof(struct gb_hid_desc_response);
		op_rsp->hid_desc_rsp.bLength = sizeof(struct gb_hid_desc_response);
		op_rsp->hid_desc_rsp.wReportDescLength = htole16(HID_REPORT_LENGTH);
		op_rsp->hid_desc_rsp.bcdHID = VERSION_BCD(1,1,1);
		op_rsp->hid_desc_rsp.wProductID = 0xFADA;
		op_rsp->hid_desc_rsp.wVendorID = 0xBABA;
		op_rsp->hid_desc_rsp.bCountryCode = 0x00;
		break;
	case GB_HID_TYPE_GET_REPORT_DESC:
		payload_size = HID_REPORT_LENGTH;
		memcpy(&op_rsp->i2c_xfer_rsp.data[0], hid_report, HID_REPORT_LENGTH);
		break;
	case GB_HID_TYPE_PWR_ON:
		payload_size = 0;
		// Store connection for button report
		hid_connection = connection;
		break;
	case GB_HID_TYPE_PWR_OFF:
		payload_size = 0;
		// disable button reporting
		hid_connection = NULL;
		break;
	case GB_HID_TYPE_GET_REPORT:
		payload_size = 3;
		memcpy(&op_rsp->hid_input_report_req.report[0], hid_status, 3);
		break;
	case GB_HID_TYPE_SET_REPORT:
		// Nothing to do... yet
		break;
	case (OP_RESPONSE | GB_HID_TYPE_IRQ_EVENT):
		return 0;
	default:
		return -EINVAL;
	}

	message_size = sizeof(struct gb_operation_msg_hdr) + payload_size;
	return send_response(hd_cport_id, op_rsp, message_size,
				oph->operation_id, oph->type, result);
}

char *hid_get_operation(uint8_t type)
{
	switch (type) {
	case GB_HID_TYPE_GET_DESC:
		return "GB_HID_TYPE_GET_DESC";
	case GB_HID_TYPE_GET_REPORT_DESC:
		return "GB_HID_TYPE_GET_REPORT_DESC";
	case GB_HID_TYPE_PWR_ON:
		return "GB_HID_TYPE_PWR_ON";
	case GB_HID_TYPE_PWR_OFF:
		return "GB_HID_TYPE_PWR_OFF";
	case GB_HID_TYPE_GET_REPORT:
		return "GB_HID_TYPE_GET_REPORT";
	case GB_HID_TYPE_SET_REPORT:
		return "GB_HID_TYPE_SET_REPORT";
	case GB_HID_TYPE_IRQ_EVENT:
		return "GB_HID_TYPE_IRQ_EVENT";
	default:
		return "(Unknown operation)";
	}
}
