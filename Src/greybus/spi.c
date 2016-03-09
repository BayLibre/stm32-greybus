/*
 * Greybus Simulator
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Provided under the three clause BSD license found in the LICENSE file.
 */

#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include "greybus/greybus.h"
#include "endian.h"

#include "spi.h"

#define SPI_BPW_MASK(bits) BIT((bits) - 1)
#define SPIDEV_TYPE	0x00

/*
 * this control the number of devices that will be created, for even chipselect
 * spidev for odd spinor
 */
#define SPI_NUM_CS	1

struct gb_spi_dev_config {
	uint16_t	mode;
	uint32_t	bits_per_word;
	uint32_t	max_speed_hz;
	uint8_t		device_type;
	uint8_t		name[32];
};

struct gb_spi_dev {
	uint8_t	cs;
	uint8_t	*buf;
	size_t	buf_size;
	uint8_t	*buf_resp;
	uint8_t	cmd_resp[6];
	size_t	resp_size;
	int	(*xfer_req_recv)(struct gb_spi_dev *dev,
				 struct gb_spi_transfer *xfer,
				 uint8_t *xfer_data, bool last);
	struct gb_spi_dev_config *conf;
};

struct gb_spi_master {
	uint16_t		mode;
	uint8_t			flags;
	uint32_t		bpwm;
	uint32_t		min_speed_hz;
	uint32_t		max_speed_hz;
	uint8_t			num_chipselect;
	struct gb_spi_dev	*devices;
};

static struct gb_spi_master *master;

static struct gb_spi_dev_config spidev_config = {
	.mode		= GB_SPI_MODE_MODE_3,
	.bits_per_word	= 8,
	.max_speed_hz	= 10000000,
	.name		= "dev",
	.device_type	= GB_SPI_SPI_DEV,
};


static int spidev_xfer_req_recv(struct gb_spi_dev *dev,
				struct gb_spi_transfer *xfer,
				uint8_t *xfer_data, bool last)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	if (xfer->rdwr == GB_SPI_XFER_WRITE)
		HAL_SPI_Transmit(&hspi2, xfer_data, xfer->len, HAL_MAX_DELAY);
	else if (xfer->rdwr == GB_SPI_XFER_READ)
		HAL_SPI_Receive(&hspi2, dev->buf_resp, xfer->len, HAL_MAX_DELAY);
	else if (xfer->rdwr == (GB_SPI_XFER_READ|GB_SPI_XFER_WRITE))
		HAL_SPI_TransmitReceive(&hspi2, xfer_data, dev->buf_resp, xfer->len, HAL_MAX_DELAY);

	if (xfer->cs_change && !last)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	else if (!xfer->cs_change && last)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	if (xfer->rdwr &= GB_SPI_XFER_READ)
		dev->buf_resp += xfer->len;

	dev->resp_size += xfer->len;

	return 0;
}

static int spi_set_device(uint8_t cs, int spi_type)
{
	struct gb_spi_dev *spi_dev = &master->devices[cs];

	switch (spi_type) {
	case SPIDEV_TYPE:
		spi_dev->xfer_req_recv = spidev_xfer_req_recv;
		spi_dev->buf_size = 0;
		spi_dev->conf = &spidev_config;
		break;
	}

	spi_dev->cs = cs;

	if (!spi_dev->buf_size)
		return 0;

	spi_dev->buf = calloc(1, spi_dev->buf_size);
	if (!spi_dev->buf) {
		free(spi_dev);
		return -ENOMEM;
	}

	return 0;
}

static int spi_master_setup(void)
{
	int i;

	master = calloc(1, sizeof(struct gb_spi_master));
	if (!master)
		return -ENOMEM;

	master->mode = GB_SPI_MODE_MODE_3;
	master->flags = 0;
	master->bpwm = SPI_BPW_MASK(8);
	master->min_speed_hz = 400000;
	master->max_speed_hz = 48000000;
	master->num_chipselect = SPI_NUM_CS;

	master->devices = calloc(master->num_chipselect,
				 sizeof(struct gb_spi_dev));
	if (!master->devices)
		return -ENOMEM;

	for (i = 0; i < SPI_NUM_CS; i++)
		spi_set_device(i, SPIDEV_TYPE);

	return 0;
}

int spi_handler(struct gbsim_connection *connection, void *rbuf,
		   size_t rsize, void *tbuf, size_t tsize)
{
	struct gb_operation_msg_hdr *oph;
	struct op_msg *op_req = rbuf;
	struct op_msg *op_rsp;
	size_t payload_size = 0;
	uint16_t message_size;
	uint16_t hd_cport_id = connection->hd_cport_id;
	struct gb_spi_transfer *xfer;
	struct gb_spi_dev *spi_dev;
	struct gb_spi_dev_config *conf;
	void *xfer_data;
	int xfer_cs, cs;
	int xfer_count;
	int xfer_rx = 0;
	int ret;
	int i;

	op_rsp = (struct op_msg *)tbuf;
	oph = (struct gb_operation_msg_hdr *)&op_req->header;

	switch (oph->type) {
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		payload_size = sizeof(struct gb_protocol_version_response);
		op_rsp->pv_rsp.major = GB_SPI_VERSION_MAJOR;
		op_rsp->pv_rsp.minor = GB_SPI_VERSION_MINOR;
		spi_master_setup();
		break;
	case GB_SPI_TYPE_MASTER_CONFIG:
		payload_size = sizeof(struct gb_spi_master_config_response);

		op_rsp->spi_mc_rsp.mode = htole16(master->mode);
		op_rsp->spi_mc_rsp.flags = htole16(master->flags);
		op_rsp->spi_mc_rsp.bits_per_word_mask = htole32(master->bpwm);
		op_rsp->spi_mc_rsp.num_chipselect = htole16(master->num_chipselect);
		op_rsp->spi_mc_rsp.min_speed_hz = htole32(master->min_speed_hz);
		op_rsp->spi_mc_rsp.max_speed_hz = htole32(master->max_speed_hz);
		break;
	case GB_SPI_TYPE_DEVICE_CONFIG:
		payload_size = sizeof(struct gb_spi_device_config_response);

		cs = op_req->spi_dc_req.chip_select;
		spi_dev = &master->devices[cs];
		conf = spi_dev->conf;

		op_rsp->spi_dc_rsp.mode = htole16(conf->mode);
		op_rsp->spi_dc_rsp.bits_per_word = conf->bits_per_word;
		op_rsp->spi_dc_rsp.max_speed_hz = htole32(conf->max_speed_hz);
		op_rsp->spi_dc_rsp.device_type = conf->device_type;
	        memcpy(op_rsp->spi_dc_rsp.name, conf->name, sizeof(conf->name));
		break;
	case GB_SPI_TYPE_TRANSFER:
		xfer_cs = op_req->spi_xfer_req.chip_select;
		xfer_count = op_req->spi_xfer_req.count;

		xfer = &op_req->spi_xfer_req.transfers[0];
		xfer_data = xfer + xfer_count;

		spi_dev = &master->devices[xfer_cs];

		spi_dev->buf_resp = op_rsp->spi_xfer_rsp.data;

		for (i = 0; i < xfer_count; i++, xfer++) {
			spi_dev->xfer_req_recv(spi_dev, xfer, xfer_data,
					(i == (xfer_count-1)));
			/* we only increment if transfer is write */
			if (xfer->rdwr & GB_SPI_XFER_WRITE)
				xfer_data += xfer->len;
			if (xfer->rdwr & GB_SPI_XFER_READ)
				xfer_rx += xfer->len;
		}

		payload_size = sizeof(struct gb_spi_transfer_response) + xfer_rx;
		break;
	default:
		return -EINVAL;
	}

	message_size = sizeof(struct gb_operation_msg_hdr) + payload_size;
	ret = send_response(hd_cport_id, op_rsp, message_size,
			    oph->operation_id, oph->type,
			    PROTOCOL_STATUS_SUCCESS);
	return ret;
}

char *spi_get_operation(uint8_t type)
{
	switch (type) {
	case GB_REQUEST_TYPE_INVALID:
		return "GB_SPI_TYPE_INVALID";
	case GB_REQUEST_TYPE_PROTOCOL_VERSION:
		return "GB_SPI_TYPE_PROTOCOL_VERSION";
	case GB_SPI_TYPE_MASTER_CONFIG:
		return "GB_SPI_TYPE_MASTER_CONFIG";
	case GB_SPI_TYPE_DEVICE_CONFIG:
		return "GB_SPI_TYPE_DEVICE_CONFIG";
	case GB_SPI_TYPE_TRANSFER:
		return "GB_SPI_TYPE_TRANSFER";
	default:
		return "(Unknown operation)";
	}
}

void spi_init(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}
