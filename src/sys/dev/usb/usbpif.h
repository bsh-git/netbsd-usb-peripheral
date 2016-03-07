/*	$NetBSD$ */

/*-
 * Copyright (c) 2016 Genetec Corporation.  All rights reserved.
 * Written by Hiroyuki Bessho for Genetec Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#ifndef _USBPIF_H
#define _USBPIF_H

/*
 * information used to build USB device descriptor.
 */
struct usbp_device_info {
	int16_t class_id;	/* can be USBP_ID_UNSPECIFIED */
	uint8_t subclass_id;
	uint8_t protocol;
	int vendor_id;		/* can be USBP_ID_UNSPECIFIED */
	uint16_t product_id;
	uint16_t bcd_device;		/* device release number in BCD */
	const char *manufacturer_name;
	const char *product_name;
	const char *serial;	/* device's serial number */
};

#define	USBP_ID_UNSPECIFIED	(-1)

/*
 * requirements of an endpoint used by the interface
 */
struct usbp_endpoint_request {
	uint8_t address;	/* (UE_DIR_IN or UE_DIR_OUT) + index.
				   Normallffy, set index to 0 to let the client controller
				   select it.
				   Alternatively, you can specify the address explictly
				   but not recommended. */
	uint8_t attributes;	/* Transfer type: UE_ISOCHROMOUS, UE_BULK, UE_INTERRUPT */
#ifdef notyet
	bool optional;	/* true if the interface can work without this endpoint */
#endif
	u_int packetsize;

	/* need more for isochronous */
};


struct usbp_interface_spec {
	uByte class_id;
	uByte subclass_id;
	uByte protocol;
	enum USBP_PIPE0_USAGE {
		USBP_PIPE0_NOTUSED, /* this interface doesn't use pipe #0 */
		USBP_PIPE0_SHARED,  /* pipe#0 is shared among interfaces,
				       by means of interface number in the packets */
		USBP_PIPE0_EXCLUSIVE /* this interface requires an exclusive use of pipe#0 */
	} pipe0_usage;
	const char *description;
	uint8_t num_endpoints;	/* the number of endpoints used by this interface excluding ep0. */
	struct usbp_endpoint_request endpoints[];
};


enum USBP_INTERFACE_STATE {
	USBP_INTERFACE_CONFIGURED,
	USBP_INTERFACE_UNCONFIGURED };
#endif	/* _USBPIF_H */
