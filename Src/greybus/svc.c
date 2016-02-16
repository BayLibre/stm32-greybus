/*
 * Greybus Simulator: SVC CPort protocol
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Provided under the three clause BSD license found in the LICENSE file.
 */

#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include "endian.h"

#include "greybus/greybus.h"

static int svc_handler_request(uint16_t cport_id, uint16_t hd_cport_id,
			       void *rbuf, size_t rsize, void *tbuf,
			       size_t tsize)
{
	struct op_msg *op_req = rbuf;
	struct op_msg *op_rsp = tbuf;
	struct gb_operation_msg_hdr *oph = &op_req->header;
	struct gb_svc_intf_device_id_request *svc_dev_id;
	struct gb_svc_conn_create_request *svc_conn_create;
	struct gb_svc_conn_destroy_request *svc_conn_destroy;
	struct gb_svc_dme_peer_get_request *dme_get_request;
	struct gb_svc_dme_peer_get_response *dme_get_response;
	struct gb_svc_dme_peer_set_request *dme_set_request;
	struct gb_svc_dme_peer_set_response *dme_set_response;
	struct gb_svc_route_create_request *svc_route_create;
	struct gb_svc_route_destroy_request *svc_route_destroy;
	uint16_t message_size = sizeof(*oph);
	size_t payload_size = 0;

	switch (oph->type) {
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		gbsim_error("%s: Protocol Version request not supported\r\n",
			    __func__);
		break;
	case GB_SVC_TYPE_INTF_DEVICE_ID:
		svc_dev_id = &op_req->svc_intf_device_id_request;

		gbsim_debug("SVC assign device id (%hhu %hhu) response\r\n",
			    svc_dev_id->intf_id, svc_dev_id->device_id);
		break;
	case GB_SVC_TYPE_CONN_CREATE:
		svc_conn_create = &op_req->svc_conn_create_request;

		gbsim_debug("SVC connection create request (%hhu %hu):(%hhu %hu) response\r\n",
			    svc_conn_create->intf1_id, svc_conn_create->cport1_id,
			    svc_conn_create->intf2_id, svc_conn_create->cport2_id);
		break;
	case GB_SVC_TYPE_CONN_DESTROY:
		svc_conn_destroy = &op_req->svc_conn_destroy_request;

		gbsim_debug("SVC connection destroy request (%hhu %hu):(%hhu %hu) response\r\n",
			    svc_conn_destroy->intf1_id, svc_conn_destroy->cport1_id,
			    svc_conn_destroy->intf2_id, svc_conn_destroy->cport2_id);
		break;
	case GB_SVC_TYPE_DME_PEER_GET:
		payload_size = sizeof(*dme_get_response);
		dme_get_request = &op_req->svc_dme_peer_get_request;
		dme_get_response = &op_rsp->svc_dme_peer_get_response;
		dme_get_response->result_code = 0;
		dme_get_response->attr_value = 1;

		gbsim_debug("SVC dme peer get (%hhu %hu %hu) request\r\n",
			    dme_get_request->intf_id, dme_get_request->attr,
			    dme_get_request->selector);
		gbsim_debug("SVC dme peer get (%hu %u) response\r\n",
			    dme_get_response->result_code,
			    dme_get_response->attr_value);
		break;
	case GB_SVC_TYPE_DME_PEER_SET:
		payload_size = sizeof(*dme_set_response);
		dme_set_request = &op_req->svc_dme_peer_set_request;
		dme_set_response = &op_rsp->svc_dme_peer_set_response;
		dme_set_response->result_code = 0;

		gbsim_debug("SVC dme peer set (%hhu %hu %hu %u) request\r\n",
			    dme_set_request->intf_id, dme_set_request->attr,
			    dme_set_request->selector, dme_set_request->value);
		gbsim_debug("SVC dme peer set (%hu) response\r\n",
			    dme_set_response->result_code);
		break;
	case GB_SVC_TYPE_ROUTE_CREATE:
		svc_route_create = &op_req->svc_route_create_request;

		gbsim_debug("SVC route create request (%hhu %hu):(%hhu %hu) response\r\n",
			    svc_route_create->intf1_id, svc_route_create->dev1_id,
			    svc_route_create->intf2_id, svc_route_create->dev2_id);
		break;
	case GB_SVC_TYPE_ROUTE_DESTROY:
		svc_route_destroy = &op_req->svc_route_destroy_request;

		gbsim_debug("SVC route destroy request (%hhu:%hhu) response\r\n",
			    svc_route_destroy->intf1_id,
			    svc_route_destroy->intf2_id);
		break;
	case GB_SVC_TYPE_PING:
		gbsim_debug("SVC ping request response\r\n");
		break;
	case GB_SVC_TYPE_INTF_HOTPLUG:
	case GB_SVC_TYPE_INTF_HOT_UNPLUG:
	case GB_SVC_TYPE_INTF_RESET:
	default:
		gbsim_error("%s: Request not supported (%d)\r\n", __func__,
			    oph->type);
		return -EINVAL;
	}

	message_size += payload_size;
	return send_response(hd_cport_id, op_rsp, message_size,
				oph->operation_id, oph->type,
				PROTOCOL_STATUS_SUCCESS);
}

static int svc_handler_response(uint16_t cport_id, uint16_t hd_cport_id,
				void *rbuf, size_t rsize)
{
	struct op_msg *op_rsp = rbuf;
	struct gb_operation_msg_hdr *oph = &op_rsp->header;
	int ret;

	/* Must be AP's svc protocol's cport */
	if (cport_id != GB_SVC_CPORT_ID || cport_id != hd_cport_id) {
		gbsim_error("%s: Error: cport-id-mismatch (%d %d %d)\r\n", __func__,
			    cport_id, hd_cport_id, GB_SVC_CPORT_ID);
		return -EINVAL;
	}

	switch (oph->type & ~OP_RESPONSE) {
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		gbsim_debug("%s: Version major-%d minor-%d\r\n", __func__,
			    op_rsp->pv_rsp.major, op_rsp->pv_rsp.minor);

		/* Version request successful, send hello msg */
		ret = svc_request_send(GB_SVC_TYPE_SVC_HELLO, AP_INTF_ID);
		if (ret) {
			gbsim_error("%s: Failed to send svc hello request (%d)\r\n",
				    __func__, ret);
			return ret;
		}
		break;
	case GB_SVC_TYPE_SVC_HELLO:
		/*
		 * AP's SVC cport is ready now, start scanning for module
		 * hotplug.
		 */
#if 0
		ret = inotify_start(hotplug_basedir);
		if (ret < 0)
			gbsim_error("Failed to start inotify thread\r\n");
#endif
		gbsim_info("Here it started inotify...\r\n");
		manifest_startup = 1;
		break;
	case GB_SVC_TYPE_INTF_HOT_UNPLUG:
		free_connections();
		break;
	case GB_SVC_TYPE_INTF_HOTPLUG:
	case GB_SVC_TYPE_INTF_RESET:
		break;
	default:
		gbsim_error("%s: Response not supported (%d)\r\n", __func__,
			    oph->type & ~OP_RESPONSE);
		return -EINVAL;
	}

	return 0;
}

int svc_handler(struct gbsim_connection *connection, void *rbuf,
		    size_t rsize, void *tbuf, size_t tsize)
{
	struct op_msg *op = rbuf;
	struct gb_operation_msg_hdr *oph = &op->header;
	uint16_t cport_id = connection->cport_id;
	uint16_t hd_cport_id = connection->hd_cport_id;

	if (oph->type & OP_RESPONSE)
		return svc_handler_response(cport_id, hd_cport_id, rbuf, rsize);
	else
		return svc_handler_request(cport_id, hd_cport_id, rbuf, rsize,
					   tbuf, tsize);
}

char *svc_get_operation(uint8_t type)
{
	switch (type) {
	case GB_REQUEST_TYPE_INVALID:
		return "GB_SVC_TYPE_INVALID";
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		return "GB_SVC_TYPE_PROTOCOL_VERSION";
	case GB_SVC_TYPE_SVC_HELLO:
		return "GB_SVC_TYPE_SVC_HELLO";
	case GB_SVC_TYPE_INTF_DEVICE_ID:
		return "GB_SVC_TYPE_INTF_DEVICE_ID";
	case GB_SVC_TYPE_INTF_HOTPLUG:
		return "GB_SVC_TYPE_INTF_HOTPLUG";
	case GB_SVC_TYPE_INTF_HOT_UNPLUG:
		return "GB_SVC_TYPE_INTF_HOT_UNPLUG";
	case GB_SVC_TYPE_INTF_RESET:
		return "GB_SVC_TYPE_INTF_RESET";
	case GB_SVC_TYPE_CONN_CREATE:
		return "GB_SVC_TYPE_CONN_CREATE";
	case GB_SVC_TYPE_CONN_DESTROY:
		return "GB_SVC_TYPE_CONN_DESTROY";
	case GB_SVC_TYPE_DME_PEER_GET:
		return "GB_SVC_TYPE_DME_PEER_GET";
	case GB_SVC_TYPE_DME_PEER_SET:
		return "GB_SVC_TYPE_DME_PEER_SET";
	case GB_SVC_TYPE_ROUTE_CREATE:
		return "GB_SVC_TYPE_ROUTE_CREATE";
	case GB_SVC_TYPE_ROUTE_DESTROY:
		return "GB_SVC_TYPE_ROUTE_DESTROY";
	case GB_SVC_TYPE_PING:
		return "GB_SVC_TYPE_PING";
	default:
		return "(Unknown operation)";
	}
}

int svc_request_send(uint8_t type, uint8_t intf_id)
{
	struct op_msg msg = { };
	struct gb_operation_msg_hdr *oph = &msg.header;
	struct gb_protocol_version_response *version_request;
	struct gb_svc_hello_request *hello_request;
	struct gb_svc_intf_hotplug_request *hotplug;
	struct gb_svc_intf_hot_unplug_request *hotunplug;
	struct gb_svc_intf_reset_request *reset;
	uint16_t message_size = sizeof(*oph);
	size_t payload_size;

	switch (type) {
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		payload_size = sizeof(*version_request);
		version_request = &msg.svc_version_request;
		version_request->major = GB_SVC_VERSION_MAJOR;
		version_request->minor = GB_SVC_VERSION_MINOR;
		break;
	case GB_SVC_TYPE_SVC_HELLO:
		payload_size = sizeof(*hello_request);
		hello_request = &msg.hello_request;

		hello_request->endo_id = htole16(ENDO_ID);
		hello_request->interface_id = AP_INTF_ID;
		break;
	case GB_SVC_TYPE_INTF_HOTPLUG:
		payload_size = sizeof(*hotplug);
		hotplug = &msg.svc_intf_hotplug_request;

		hotplug->intf_id = intf_id;

		//FIXME: Use some real version numbers here ?
		hotplug->data.ddbl1_mfr_id = htole32(1);
		hotplug->data.ddbl1_prod_id = htole32(1);
		hotplug->data.ara_vend_id = htole32(1);
		hotplug->data.ara_prod_id = htole32(1);
		hotplug->data.serial_number = htole64(0x0000000000001234);
		break;
	case GB_SVC_TYPE_INTF_HOT_UNPLUG:
		payload_size = sizeof(*hotunplug);
		hotunplug = &msg.svc_intf_hot_unplug_request;
		hotunplug->intf_id = intf_id;
		break;
	case GB_SVC_TYPE_INTF_RESET:
		payload_size = sizeof(*reset);
		reset = &msg.svc_intf_reset_request;
		reset->intf_id = intf_id;

		break;
	default:
		gbsim_error("svc operation type %02x not supported\r\n", type);
		return -EINVAL;
	}

	message_size += payload_size;
	return send_request(GB_SVC_CPORT_ID, &msg, message_size, 1, type);
}

void svc_init(void)
{
	/* Allocate cport for svc protocol between AP and SVC */
	allocate_connection(GB_SVC_CPORT_ID, GB_SVC_CPORT_ID, GREYBUS_PROTOCOL_SVC);
}

void svc_exit(void)
{
	free_connection(connection_find(GB_SVC_CPORT_ID));
}
