/*
 * Greybus interface manifest parsing
 *
 * Copyright 2014 Google Inc.
 * Copyright 2014 Linaro Ltd.
 *
 * Provided under the three clause BSD license found in the LICENSE file.
 */

#include <errno.h>

#include <stdbool.h>
#include <stdlib.h>
#include "endian.h"

#include "greybus/greybus.h"
/*
 * Greybus kernel assigns hd-cport-ids to cports in the order they are present
 * in manifest. To match that here, we can just use a simple counter.
 */
static uint16_t hd_cport_id_counter;
static int control_done;

static uint16_t allocate_hd_cport_id(void)
{
	/*
	 * AP's hd_cport_id GB_SVC_CPORT_ID is reserved and must not be used for
	 * other protocols.
	 */
	if (hd_cport_id_counter == GB_SVC_CPORT_ID)
		++hd_cport_id_counter;

	return hd_cport_id_counter++;
}

void reset_hd_cport_id(void)
{
	hd_cport_id_counter = 0;
}

/*
 * Validate the given descriptor.  Its reported size must fit within
 * the number of bytes reamining, and it must have a recognized
 * type.  Check that the reported size is at least as big as what
 * we expect to see.  (It could be bigger, perhaps for a new version
 * of the format.)
 *
 * Returns the number of bytes consumed by the descriptor, or a
 * negative errno.
 */
static int identify_descriptor(struct greybus_descriptor *desc, size_t size)
{
	struct greybus_descriptor_header *desc_header = &desc->header;
	size_t expected_size;
	size_t desc_size;

	if (size < sizeof(*desc_header)) {
		gbsim_error("manifest too small\n");
		return -EINVAL;		/* Must at least have header */
	}

	desc_size = (int)le16toh(desc_header->size);
	if ((size_t)desc_size > size) {
		gbsim_error("descriptor too big\n");
		return -EINVAL;
	}

	/* Descriptor needs to at least have a header */
	expected_size = sizeof(*desc_header);

	switch (desc_header->type) {
	case GREYBUS_TYPE_STRING:
		expected_size += sizeof(struct greybus_descriptor_string);
		expected_size += desc->string.length;

		/* String descriptors are padded to 4 byte boundaries */
		expected_size = ALIGN(expected_size);
		break;
	case GREYBUS_TYPE_INTERFACE:
		expected_size += sizeof(struct greybus_descriptor_interface);
		break;
	case GREYBUS_TYPE_BUNDLE:
		expected_size += sizeof(struct greybus_descriptor_bundle);
		break;
	case GREYBUS_TYPE_CPORT:
		expected_size += sizeof(struct greybus_descriptor_cport);

		/*
		 * Module's control protocol's node might not be present in
		 * manifest, and the first allocated cport should be for control
		 * protocol.
		 */
		if (!control_done &&
			(le16toh(desc->cport.id) != GB_CONTROL_CPORT_ID)) {
			allocate_connection(GB_CONTROL_CPORT_ID,
					allocate_hd_cport_id(),
					GREYBUS_PROTOCOL_CONTROL);
		}

		control_done = 1;
		allocate_connection(le16toh(desc->cport.id), allocate_hd_cport_id(),
				desc->cport.protocol_id);
		break;
	case GREYBUS_TYPE_INVALID:
	default:
		gbsim_error("invalid descriptor type (%hhu)\n", desc_header->type);
		return -EINVAL;
	}

	if (desc_size < expected_size) {
		gbsim_error("%d descriptor too small (%zu < %zu)\n",
		       desc_header->type, desc_size, expected_size);
		return -EINVAL;
	}

	/* Warn if there is a size mismatch */
	if (desc_size != expected_size) {
		gbsim_error("%d descriptor size mismatch, expected - %zu, actual - %zu)\n",
			    desc_header->type, expected_size, desc_size);
	}

	return desc_size;
}

/*
 * Parse a buffer containing a Interface manifest.
 *
 * If we find anything wrong with the content/format of the buffer
 * we reject it.
 *
 * The first requirement is that the manifest's version is
 * one we can parse.
 *
 * We make an initial pass through the buffer and identify all of
 * the descriptors it contains, keeping track for each its type
 * and the location size of its data in the buffer.
 *
 * Next we scan the descriptors, looking for a interface descriptor;
 * there must be exactly one of those.  When found, we record the
 * information it contains, and then remove that descriptor (and any
 * string descriptors it refers to) from further consideration.
 *
 * After that we look for the interface's bundles--there must be at
 * least one of those.
 *
 * Returns true if parsing was successful, false otherwise.
 */
bool manifest_parse(void *data, size_t size)
{
	struct greybus_manifest *manifest;
	struct greybus_manifest_header *header;
	struct greybus_descriptor *desc;
	uint16_t manifest_size;

	/* we have to have at _least_ the manifest header */
	if (size <= sizeof(manifest->header)) {
		gbsim_error("short manifest (%zu)\r\n", size);
		return false;
	}

	/* Make sure the size is right */
	manifest = data;
	header = &manifest->header;
	manifest_size = le16toh(header->size);
	if (manifest_size != size) {
		gbsim_error("manifest size mismatch %zu != %hu\r\n",
			size, manifest_size);
		return false;
	}

	/* Validate major/minor number */
	if (header->version_major > GREYBUS_VERSION_MAJOR) {
		gbsim_error("manifest version too new (%hhu.%hhu > %hhu.%hhu)\r\n",
			header->version_major, header->version_minor,
			GREYBUS_VERSION_MAJOR, GREYBUS_VERSION_MINOR);
		return false;
	}

	/* OK, find all the descriptors */
	desc = (struct greybus_descriptor *)(header + 1);
	size -= sizeof(*header);

	/* Reset control protocol's counter */
	control_done = 0;

	while (size) {
		int desc_size;

		desc_size = identify_descriptor(desc, size);
		if (desc_size < 0)
			return false;

		desc = (struct greybus_descriptor *)((char *)desc + desc_size);
		size -= desc_size;
	}

	return true;
}
