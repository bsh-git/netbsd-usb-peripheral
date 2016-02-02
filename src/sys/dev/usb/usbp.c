/*	$NetBSD$ */
/*	$OpenBSD: usbf.c,v 1.16 2013/11/18 20:21:51 deraadt Exp $	*/

/*-
 * Copyright (c) 2015 Genetec Corporation.  All rights reserved.
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

/*
 * Copyright (c) 2006 Uwe Stuehler <uwe@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Peripheral-side USB support.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/kmem.h>
#include <sys/proc.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>

#include <dev/usb/usbp.h>

struct usbp_port {
	struct usbd_port usbd;
};

struct usbp_interface {
	struct usbd_interface usbd;

	struct usbp_config *config;
	SIMPLEQ_ENTRY(usbp_interface) next;
	SIMPLEQ_HEAD(, usbp_endpoint) endpoint_head;
};

struct usbp_config {
	struct usbp_device *uc_device;
	usb_config_descriptor_t *uc_cdesc;
	size_t uc_cdesc_size;
	int uc_closed;
	SIMPLEQ_HEAD(, usbp_interface) iface_head;
	SIMPLEQ_ENTRY(usbp_config) next;
};

struct usbp_device {
	struct usbd_device usbd;

	uByte	 string_id;	/* next string id */
	SIMPLEQ_HEAD(, usbp_config) configs;
	usb_status_t		 status;	/* device status */

	struct usbd_xfer	*default_xfer;	/* device request xfer */
	struct usbd_xfer	*data_xfer;	/* request response xfer */

	usb_device_request_t	 def_req;	/* device request buffer */

	struct usbp_config *config;
	struct usbp_function *function;

	usb_string_descriptor_t *sdesc;		/* string descriptors */
	size_t			 sdesc_size;	/* size of ud_sdesc */
	
};


#define USBP_EMPTY_STRING_ID		(USB_LANGUAGE_TABLE+1)
#define USBP_STRING_ID_MIN		(USB_LANGUAGE_TABLE+2)
#define USBP_STRING_ID_MAX		255


struct usbp_softc {
	device_t sc_dev;
	struct usbp_bus  *sc_bus;
#if 0
	lwp_t sc_thread;	/* task thread */
	TAILQ_HEAD(,usbp_task)	 sc_tskq;	/* task queue head */
#endif
	bool sc_dying;
	u_int8_t		*sc_hs_config;
};

static const char *usbrev_str[] = USBREV_STR;


#ifndef	UC_BUS_POWERED
#define	UC_BUS_POWERED	UC_ATTR_MBO
#endif

#define DEVNAME(sc)	device_xname((sc)->sc_dev)
static usbd_status usbp_softintr_establish(struct usbp_softc *);
static usbd_status
usbp_new_device(device_t parent, struct usbp_bus *bus, int speed
#if 0
		, int depth, int port, struct usbp_port *up
#endif
	);
static usbd_status usbp_setup_pipe(struct usbp_device *dev, struct usbp_interface *iface,
				   struct usbp_endpoint *ep, int ival, struct usbd_pipe **pipe);

static void usbp_setup_default_xfer(struct usbd_xfer *xfer, struct usbd_pipe *pipe,
    void *priv, usb_device_request_t *req, u_int16_t flags,
				    u_int32_t timeout, usbd_callback callback);
static void usbp_do_request(struct usbd_xfer *xfer, void *priv, usbd_status err);
static usbd_status usbp_transfer(struct usbd_xfer *xfer);
static usbd_status usbp_probe_and_attach(struct device *parent, struct usbp_device *dev/*, int port*/);
static void usbp_remove_device(struct usbp_device *dev/*, struct usbp_port *up*/);
static void usbp_free_xfer(struct usbd_xfer *xfer);
static void usbp_close_pipe(struct usbd_pipe *pipe);
static void usbp_set_address(struct usbp_device *dev, u_int8_t address);
static int usbp_match(device_t, cfdata_t, void *);
static void usbp_attach(device_t, device_t, void *);
static usbd_status usbp_set_config(struct usbp_device *dev, u_int8_t new);

usb_config_descriptor_t *usbp_config_descriptor(struct usbp_device *dev, u_int8_t index);



extern struct cfdriver usbf_cd;

CFATTACH_DECL3_NEW(usbp, sizeof(struct usbp_softc),
		   usbp_match, usbp_attach, NULL, NULL, NULL, NULL, 0);


#ifndef USBP_DEBUG
#define DPRINTF(l, x)	do {} while (0)
#else
int usbpdebug = 10;
#define DPRINTF(l, x)	if ((l) <= usbpdebug) printf x; else
#endif

static int
usbp_match(device_t parent, cfdata_t match, void *aux)
{
	struct usbp_bus_attach_args *uaa = aux;

	if (strcmp(uaa->busname, "usbp") == 0)
		return 1;
	return 0;
}

static void
usbp_attach(device_t parent, device_t self, void *aux)
{
	struct usbp_softc *sc = device_private(self);
	struct usbp_bus_attach_args *uaa = aux;
	u_int usbrev, speed;
	usbd_status err;

	sc->sc_dev = self;
	sc->sc_bus = uaa->bus;
	sc->sc_bus->usbd.usbctl = self;

	usbrev = sc->sc_bus->usbd.usbrev;
	aprint_normal(": USB revision %s", usbrev_str[usbrev]);
	aprint_naive("\n");

	switch (usbrev) {
	case USBREV_2_0:
		speed = USB_SPEED_HIGH;
		break;
	case USBREV_1_1:
	case USBREV_1_0:
		speed = USB_SPEED_FULL;
		break;
	default:
		aprint_error(", not supported\n");
		sc->sc_dying = 1;
		return;
	}
	aprint_normal("\n");

#if 0
	/* Initialize the usbf struct. */
	TAILQ_INIT(&sc->sc_tskq);
#endif

	/* Establish the software interrupt. */
	if (usbp_softintr_establish(sc)) {
		printf("%s: can't establish softintr\n", DEVNAME(sc));
		sc->sc_dying = true;
		return;
	}

	/* Attach the function driver. */
	err = usbp_new_device(self, sc->sc_bus, speed/*, 0, 0, &sc->sc_port*/);
	if (err) {
		printf("%s: usbp_new_device failed, %s\n", DEVNAME(sc),
		    usbd_errstr(err));
		sc->sc_dying = true;
		return;
	}

#if 0
	/* Create a process context for asynchronous tasks. */
	if (kthread_create(PRI_NONE, 0, NULL, usbf_task_thread, sc,
			   &sc->sc_proc, "%s", device_xname(self)) ) {
		aprint_normal_dev(self, "unable to create event thread for USB client\n");
	}
#endif
	
}




/*
 * Software interrupts
 */

static usbd_status
usbp_softintr_establish(struct usbp_softc *sc)
{
	struct usbp_bus *bus = sc->sc_bus;

	KASSERT(bus->usbd.soft == NULL);

	bus->usbd.soft = softint_establish(IPL_SOFTUSB,
	    bus->usbd.methods->soft_intr, bus);
	if (bus->usbd.soft == NULL)
		return USBD_INVAL;

	return USBD_NORMAL_COMPLETION;
}


/*
 * Bus event handling
 */

void
usbp_host_reset(struct usbp_bus *bus)
{
	struct usbp_device *dev = bus->device;

	DPRINTF(0,("%s\n", __func__));

	/* Change device state from any state backe to Default. */
	(void)usbp_set_config(dev, USB_UNCONFIG_NO);
	dev->usbd.address = 0;

	bus->ep0state = EP0_IDLE;

	DPRINTF(0,("%s %d\n", __func__, __LINE__));

#if 0
	SIMPLEQ_INIT(&dev->default_pipe);
#else
	while (!SIMPLEQ_EMPTY(&dev->usbd.default_pipe->queue)) {
		DPRINTF(0, ("%s: dequeue xfer %p", __func__, SIMPLEQ_FIRST(&dev->usbd.default_pipe->queue)));
		SIMPLEQ_REMOVE_HEAD(&dev->usbd.default_pipe->queue, next);
	}
#endif		
	usbp_setup_default_xfer(dev->default_xfer, dev->usbd.default_pipe,
	    NULL, &dev->def_req, 0, 0, usbp_do_request);
	usbp_transfer(dev->default_xfer);
}


/*
 *  USB device
 */
static usbd_status
usbp_new_device(device_t parent, struct usbp_bus *bus, int speed
#if 0
		, int depth,
		int port, struct usbp_port *up
#endif
	)
{
	struct usbp_device *dev = NULL;
	usb_device_descriptor_t *ud;
	usbd_status err;
	struct usbd_pipe *default_pipe = NULL;

#if 0
#ifdef DIAGNOSTIC
	KASSERT(up->device == NULL);
#endif
#endif

	DPRINTF(0,("%s: %s: rev=%x\n", __func__, device_xname(parent), bus->usbd.usbrev));

	dev = malloc(sizeof(*dev), M_USB, M_NOWAIT | M_ZERO);
	if (dev == NULL)
		return USBD_NOMEM;

	dev->usbd.bus = &bus->usbd;                   // XXX
	dev->string_id = USBP_STRING_ID_MIN;
	SIMPLEQ_INIT(&dev->configs);

	/* Initialize device status. */
	USETW(dev->status.wStatus, UDS_SELF_POWERED);

	/*
	 * Initialize device descriptor.  The function driver for this
	 * device (attached below) must complete the device descriptor.
	 */
	ud = &dev->usbd.ddesc;
	ud->bLength = USB_DEVICE_DESCRIPTOR_SIZE;
	ud->bDescriptorType = UDESC_DEVICE;
	ud->bMaxPacketSize = bus->ep0_maxp;
	if (bus->usbd.usbrev >= USBREV_2_0)
		USETW(ud->bcdUSB, 0x0200);
	else
		USETW(ud->bcdUSB, 0x0101);

	/* Set up the default endpoint handle and descriptor. */
	dev->usbd.def_ep.edesc = &dev->usbd.def_ep_desc;
	dev->usbd.def_ep_desc.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE;
	dev->usbd.def_ep_desc.bDescriptorType = UDESC_ENDPOINT;
	dev->usbd.def_ep_desc.bEndpointAddress = USB_CONTROL_ENDPOINT;
	dev->usbd.def_ep_desc.bmAttributes = UE_CONTROL;
	USETW(dev->usbd.def_ep_desc.wMaxPacketSize, ud->bMaxPacketSize);
	dev->usbd.def_ep_desc.bInterval = 0;

	/* Establish the default pipe. */
	err = usbp_setup_pipe(dev, NULL, (struct usbp_endpoint *)&dev->usbd.def_ep, 0, &default_pipe);
	if (err)
		goto bad;

	dev->usbd.default_pipe = default_pipe;

	/* Preallocate xfers for default pipe. */
	dev->default_xfer = usbp_alloc_xfer(dev);
	dev->data_xfer = usbp_alloc_xfer(dev);
	if (dev->default_xfer == NULL || dev->data_xfer == NULL)
		goto bad;

	/* Insert device request xfer. */
	usbp_setup_default_xfer(dev->default_xfer, default_pipe, NULL,
				&dev->def_req, 0, 0, usbp_do_request);
	err = usbp_transfer(dev->default_xfer);
	if (err && err != USBD_IN_PROGRESS)
		goto bad;

#if 0
	/* Associate the upstream port with the device. */
	bzero(up, sizeof *up);
	up->portno = port;
	up->device = dev;
#endif

	bus->device = dev;

	/* Attach function driver. */
	err = usbp_probe_and_attach(parent, dev /* , port*/);
	if (err) {
		DPRINTF(0, ("%s: %s: usbf_probe_and_attach failed. err=%d\n",
			     device_xname(parent),
			     __func__, err));
		usbp_remove_device(dev/*, up*/);

	}
	return err;

bad:
	if (dev != NULL) {
		if (dev->default_xfer)
			usbp_free_xfer(dev->default_xfer);
		if (dev->data_xfer)
			usbp_free_xfer(dev->data_xfer);
		if (default_pipe)
			usbp_close_pipe(default_pipe);
		free(dev, M_USB);
	}
	return err;
			
}

/*
 * Should be called by the function driver in its attach routine to change
 * the default device identification according to the particular function.
 */
void
usbp_devinfo_setup(struct usbp_device *dev, u_int8_t devclass,
    u_int8_t subclass, u_int8_t proto, u_int16_t vendor, u_int16_t product,
    u_int16_t device, const char *manf, const char *prod, const char *ser)
{
	usb_device_descriptor_t *dd;

	dd = usbp_device_descriptor(dev);
	dd->bDeviceClass = devclass;
	dd->bDeviceSubClass = subclass;
	dd->bDeviceProtocol = proto;
	if (vendor != 0)
		USETW(dd->idVendor, vendor);
	if (product != 0)
		USETW(dd->idProduct, product);
	if (device != 0)
		USETW(dd->bcdDevice, device);
	if (manf != NULL)
		dd->iManufacturer = usbp_add_string(dev, manf);
	if (prod != NULL)
		dd->iProduct = usbp_add_string(dev, prod);
	if (ser != NULL)
		dd->iSerialNumber = usbp_add_string(dev, ser);
}

usbd_status
usbp_setup_pipe(struct usbp_device *dev, struct usbp_interface *iface,
    struct usbp_endpoint *ep, int ival, struct usbd_pipe **pipe)
{
	struct usbd_pipe *p;
	usbd_status err;

	p = malloc(dev->usbd.bus->pipe_size, M_USB, M_NOWAIT|M_ZERO);
	if (p == NULL)
		return USBD_NOMEM;

	p->device = &dev->usbd;
	p->iface = &iface->usbd;
	p->endpoint = &ep->usbd;
	ep->usbd.refcnt++;
	p->aborting = 0;
	p->running = 0;
	p->refcnt = 1;
	p->repeat = 0;
	p->interval = ival;
	p->methods = NULL;	/* set by bus driver in open_pipe() */
	SIMPLEQ_INIT(&p->queue);
	err = dev->usbd.bus->methods->open_pipe(p);
	if (err) {
		free(p, M_USB);
		return err;
	}
	*pipe = p;
	return USBD_NORMAL_COMPLETION;
}

void
usbp_setup_xfer(struct usbd_xfer *xfer, struct usbd_pipe *pipe,
    void *priv, void *buffer, u_int32_t length,
		u_int16_t flags, u_int32_t timeout, usbd_callback callback);

void
usbp_setup_xfer(struct usbd_xfer *xfer, struct usbd_pipe *pipe,
    void *priv, void *buffer, u_int32_t length,
    u_int16_t flags, u_int32_t timeout, usbd_callback callback)
{
	xfer->pipe = pipe;
	xfer->priv = priv;
	xfer->buffer = buffer;
	xfer->length = length;
	xfer->actlen = 0;
	xfer->flags = flags;
	xfer->timeout = timeout;
	xfer->status = USBD_NOT_STARTED;
	xfer->callback = callback;
	xfer->rqflags &= ~URQ_REQUEST;
}


void
usbp_setup_default_xfer(struct usbd_xfer *xfer, struct usbd_pipe *pipe,
    void *priv, usb_device_request_t *req, u_int16_t flags,
    u_int32_t timeout, usbd_callback callback)
{
	xfer->pipe = pipe;
	xfer->priv = priv;
	xfer->buffer = req;
	xfer->length = sizeof *req;
	xfer->actlen = 0;
	xfer->flags = flags;
	xfer->timeout = timeout;
	xfer->status = USBD_NOT_STARTED;
	xfer->callback = callback;
	xfer->rqflags |= URQ_REQUEST;
}


/*
 * Change device state from Default to Address, or change the device address
 * if the device is not currently in the Default state.
 */
static void
usbp_set_address(struct usbp_device *dev, u_int8_t address)
{
	DPRINTF(0,("usbf_set_address: dev=%p, %u -> %u\n", dev,
	    dev->usbd.address, address));
	dev->usbd.address = address;
}

/*
 * If the device was in the Addressed state (dev->config == NULL) before, it
 * will be in the Configured state upon successful return from this routine.
 */
static usbd_status
usbp_set_config(struct usbp_device *dev, u_int8_t new)
{
	usbd_status err = USBD_NORMAL_COMPLETION;
	struct usbp_config *cfg = dev->config;
	u_int8_t old = cfg ? cfg->uc_cdesc->bConfigurationValue :
	    USB_UNCONFIG_NO;

	DPRINTF(0,("%s: dev=%p, %u -> %u\n", __func__, dev, old, new));

	if (old == new)
		return USBD_NORMAL_COMPLETION;

	struct usbp_function *fun = dev->function;


	/*
	 * Resetting the device state to Unconfigured must always succeed.
	 * This happens typically when the host resets the bus.
	 */
	if (new == USB_UNCONFIG_NO) {
		if (fun->methods->set_config)
			err = fun->methods->set_config(fun, NULL);
		if (err) {
			DPRINTF(0,("usbp_set_config: %s\n", usbd_errstr(err)));
		}
		dev->config = NULL;
		return USBD_NORMAL_COMPLETION;
	}

	/*
	 * Changing the device configuration may fail.  The function
	 * may decline to set the new configuration.
	 */
	SIMPLEQ_FOREACH(cfg, &dev->configs, next) {
		if (cfg->uc_cdesc->bConfigurationValue == new) {
			if (dev->function->methods->set_config)
				err = fun->methods->set_config(fun, cfg);
			if (!err)
				dev->config = cfg;
			return err;
		}
	}
	return USBD_INVAL;
}

static usb_string_descriptor_t *
usbp_string_descriptor(struct usbp_device *dev, u_int8_t id)
{
	static usb_string_descriptor_t sd0;
	static usb_string_descriptor_t sd1;
	usb_string_descriptor_t *sd;

	/* handle the special string ids */
	switch (id) {
	case USB_LANGUAGE_TABLE:
		sd0.bLength = 4;
		sd0.bDescriptorType = UDESC_STRING;
		USETW(sd0.bString[0], 0x0409 /* en_US */);
		return &sd0;

	case USBP_EMPTY_STRING_ID:
		sd1.bLength = 2;
		sd1.bDescriptorType = UDESC_STRING;
		return &sd1;
	}

	/* check if the string id is valid */
	if (id > dev->string_id)
		return NULL;

	/* seek and return the descriptor of a non-empty string */
	id -= USBP_STRING_ID_MIN;
	sd = dev->sdesc;
	while (id-- > 0)
		sd = (usb_string_descriptor_t *)((char *)sd + sd->bLength);
	return sd;
}



/*
 * Device request handling
 */

static usbd_status
usbp_get_descriptor(struct usbp_device *dev, usb_device_request_t *req,
    void **data)
{
	u_int8_t type = UGETW(req->wValue) >> 8;
	u_int8_t index = UGETW(req->wValue) & 0xff;
	usb_device_descriptor_t *dd;
	usb_config_descriptor_t *cd;
	usb_string_descriptor_t *sd;
	struct usbp_softc *sc;

	printf("%s: type=%d index=%d\n", __func__, type, index);
	switch (type) {
	case UDESC_DEVICE:
		dd = &dev->usbd.ddesc;
		*data = dd;
		USETW(req->wLength, MIN(UGETW(req->wLength), dd->bLength));
		return USBD_NORMAL_COMPLETION;

	case UDESC_DEVICE_QUALIFIER: {
		static usb_device_qualifier_t dq;

		dd = &dev->usbd.ddesc;
		bzero(&dq, sizeof dq);
		dq.bLength = USB_DEVICE_QUALIFIER_SIZE;
		dq.bDescriptorType = UDESC_DEVICE_QUALIFIER;
		USETW(dq.bcdUSB, 0x0200);
		dq.bDeviceClass = dd->bDeviceClass;
		dq.bDeviceSubClass = dd->bDeviceSubClass;
		dq.bDeviceProtocol = dd->bDeviceProtocol;
		dq.bMaxPacketSize0 = dd->bMaxPacketSize;
		dq.bNumConfigurations = dd->bNumConfigurations;
		*data = &dq;
		USETW(req->wLength, MIN(UGETW(req->wLength), dq.bLength));
		return USBD_NORMAL_COMPLETION;
	}

	case UDESC_CONFIG:
		cd = usbp_config_descriptor(dev, index);
		if (cd == NULL)
			return USBD_INVAL;
		*data = cd;
		USETW(req->wLength, MIN(UGETW(req->wLength),
		    UGETW(cd->wTotalLength)));
		return USBD_NORMAL_COMPLETION;

	/* XXX */
	case UDESC_OTHER_SPEED_CONFIGURATION: {
		struct usbp_bus *bus = (struct usbp_bus *)dev->usbd.bus;
		sc = device_private(bus->usbd.usbctl);

		cd = usbp_config_descriptor(dev, index);
		if (cd == NULL)
			return USBD_INVAL;
		if (sc->sc_hs_config == NULL) {
			/* XXX should allocate more dynamically */
			sc->sc_hs_config =
			    (u_int8_t *)malloc(65536, M_USB, M_NOWAIT);
		}
		if (sc->sc_hs_config == NULL)
			return USBD_INVAL;
		bcopy(cd, sc->sc_hs_config, UGETW(cd->wTotalLength));
		*data = sc->sc_hs_config;
		((usb_config_descriptor_t *)sc->sc_hs_config)->bDescriptorType =
		    UDESC_OTHER_SPEED_CONFIGURATION;
		USETW(req->wLength, MIN(UGETW(req->wLength),
		    UGETW(cd->wTotalLength)));
		return USBD_NORMAL_COMPLETION;
	}

	case UDESC_STRING:
		sd = usbp_string_descriptor(dev, index);
		if (sd == NULL)
			return USBD_INVAL;
		*data = sd;
		USETW(req->wLength, MIN(UGETW(req->wLength), sd->bLength));
		return USBD_NORMAL_COMPLETION;

	default:
		DPRINTF(0,("usbf_get_descriptor: unknown descriptor type=%u\n",
		    type));
		return USBD_INVAL;
	}
}

static struct usbp_endpoint *
usbp_iface_endpoint(struct usbp_interface *iface, u_int8_t address)
{
	struct usbp_endpoint *ep;

	SIMPLEQ_FOREACH(ep, &iface->endpoint_head, next) {
		if (ep->usbd.edesc->bEndpointAddress == address)
			return ep;
	}
	return NULL;
}


static struct usbp_endpoint *
usbp_config_endpoint(struct usbp_config *cfg, u_int8_t address)
{
	struct usbp_interface *iface;
	struct usbp_endpoint *ep;

	SIMPLEQ_FOREACH(iface, &cfg->iface_head, next) {
		SIMPLEQ_FOREACH(ep, &iface->endpoint_head, next) {
			if (ep->usbd.edesc->bEndpointAddress == address)
				return ep;
		}
	}
	return NULL;
}


static void
usbp_set_endpoint_halt(struct usbp_endpoint *endpoint)
{
}

static void
usbp_clear_endpoint_halt(struct usbp_endpoint *endpoint)
{
}

static void
usbp_stall_pipe(struct usbd_pipe *pipe)
{
	DPRINTF(0,("usbf_stall_pipe not implemented\n"));
}



static usbd_status
usbp_set_endpoint_feature(struct usbp_config *cfg, u_int8_t address,
    u_int16_t value)
{
	struct usbp_endpoint *ep;

	DPRINTF(0,("usbf_set_endpoint_feature: cfg=%p address=%#x"
	    " value=%#x\n", cfg, address, value));

	ep = usbp_config_endpoint(cfg, address);
	if (ep == NULL)
		return USBD_BAD_ADDRESS;

	switch (value) {
	case UF_ENDPOINT_HALT:
		usbp_set_endpoint_halt(ep);
		return USBD_NORMAL_COMPLETION;
	default:
		/* unsupported feature, send STALL in data/status phase */
		return USBD_STALLED;
	}
}

static usbd_status
usbp_clear_endpoint_feature(struct usbp_config *cfg, u_int8_t address,
    u_int16_t value)
{
	struct usbp_endpoint *ep;

	DPRINTF(0,("usbf_clear_endpoint_feature: cfg=%p address=%#x"
	    " value=%#x\n", cfg, address, value));

	ep = usbp_config_endpoint(cfg, address);
	if (ep == NULL)
		return USBD_BAD_ADDRESS;

	switch (value) {
	case UF_ENDPOINT_HALT:
		usbp_clear_endpoint_halt(ep);
		return USBD_NORMAL_COMPLETION;
	default:
		/* unsupported feature, send STALL in data/status phase */
		return USBD_STALLED;
	}
}

/*
 * Handle device requests coming in via endpoint 0 pipe.
 */
static void
usbp_do_request(struct usbd_xfer *xfer, void *priv,
    usbd_status err)
{
	struct usbp_device *dev = (struct usbp_device*)(xfer->pipe->device);
	usb_device_request_t *req = xfer->buffer;
	struct usbp_config *cfg;
	void *data = NULL;
	u_int16_t value;
	u_int16_t index;
	struct usbp_bus *bus = (struct usbp_bus *)(dev->usbd.bus);

	/* XXX */
	if (bus->ep0state == EP0_END_XFER &&
	    err == USBD_SHORT_XFER) {

		bus->ep0state = EP0_IDLE;
		goto next;
	}

	if (err) {
		DPRINTF(0,("usbp_do_request: receive failed, %s\n",
		    usbd_errstr(err)));
		return;
	}

#ifdef USBP_DEBUG
	if (usbpdebug >= 20)
		usbp_dump_request(dev, req);
#endif

#define C(x,y) ((x) | ((y) << 8))
	switch (C(req->bRequest, req->bmRequestType)) {
		
	case C(UR_SET_ADDRESS, UT_WRITE_DEVICE):
		/* Change device state from Default to Address. */
		usbp_set_address(dev, UGETW(req->wValue));
		break;

	case C(UR_SET_CONFIG, UT_WRITE_DEVICE):
		/* Change device state from Address to Configured. */
		printf("set config activated\n");
		err = usbp_set_config(dev, UGETW(req->wValue) & 0xff);
		break;

	case C(UR_GET_CONFIG, UT_READ_DEVICE):
	{			/* XXX */
		if ((cfg = dev->config) == NULL) {
			static u_int8_t zero = 0;
			data = &zero;
		} else
			data = &cfg->uc_cdesc->bConfigurationValue;
		USETW(req->wLength, MIN(UGETW(req->wLength), 1));
	}
		break;

	case C(UR_GET_DESCRIPTOR, UT_READ_DEVICE):
		err = usbp_get_descriptor(dev, req, &data);
		break;

	case C(UR_GET_STATUS, UT_READ_DEVICE):
		DPRINTF(1,("%s: UR_GET_STATUS %d\n", __func__,
			    UGETW(req->wLength)));
		data = &dev->status;
		USETW(req->wLength, MIN(UGETW(req->wLength),
		    sizeof dev->status));
		break;

	case C(UR_GET_STATUS, UT_READ_ENDPOINT): {
		//u_int8_t addr = UGETW(req->wIndex) & 0xff;
		static u_int16_t status = 0;

		data = &status;
		USETW(req->wLength, MIN(UGETW(req->wLength), sizeof status));
		break;
	}

	case C(UR_SET_FEATURE, UT_WRITE_ENDPOINT):
		value = UGETW(req->wValue);
		index = UGETW(req->wIndex);
		if ((cfg = dev->config) == NULL)
			err = USBD_STALLED;
		else
			err = usbp_set_endpoint_feature(cfg, index, value);
		break;

	case C(UR_CLEAR_FEATURE, UT_WRITE_ENDPOINT):
		value = UGETW(req->wValue);
		index = UGETW(req->wIndex);
		if ((cfg = dev->config) == NULL)
			err = USBD_STALLED;
		else
			err = usbp_clear_endpoint_feature(cfg, index, value);
		break;

	/* Alternate settings for interfaces are unsupported. */
	case C(UR_SET_INTERFACE, UT_WRITE_INTERFACE):
		if (UGETW(req->wValue) != 0)
			err = USBD_STALLED;
		break;
	case C(UR_GET_INTERFACE, UT_READ_INTERFACE): {
		static u_int8_t zero = 0;
		data = &zero;
		USETW(req->wLength, MIN(UGETW(req->wLength), 1));
		break;
	}

	default: {
		struct usbp_function *fun = dev->function;
		
		DPRINTF(5,("usbf_do_request: xfer=%p dev=%p, fun=%p methods=%p %p %p\n",
			   xfer,
			   dev,
			   fun, fun->methods,
			   fun->methods ? fun->methods->set_config : NULL,
			   fun->methods ? fun->methods->do_request : NULL));

		if (fun == NULL)
			err = USBD_STALLED;
		else {
			/* XXX change prototype for this method to remove
			 * XXX the data argument. */
			DPRINTF(0, ("calling do_request %p\n", fun->methods->do_request));
			err = fun->methods->do_request(fun, req, &data);
			DPRINTF(0, ("return from do_request err=%d\n", err));
		}
	}
	}

	DPRINTF(5,("usbf_do_request: %d err=%d\n", __LINE__, err));

	if (err) {
		DPRINTF(0,("usbf_do_request: request=%#x, type=%#x "
		    "failed, %s\n", req->bRequest, req->bmRequestType,
		    usbd_errstr(err)));
		usbp_stall_pipe(dev->usbd.default_pipe);
	} else if (UGETW(req->wLength) > 0) {
		if (data == NULL) {
			DPRINTF(0,("usbf_do_request: no data, "
			    "sending ZLP\n"));
			USETW(req->wLength, 0);
		}
		printf("%s: reply IN data length=%d\n", __func__, UGETW(req->wLength));
		/* Transfer IN data in response to the request. */
		usbp_setup_xfer(dev->data_xfer, dev->usbd.default_pipe,
		    NULL, data, UGETW(req->wLength), 0, 0, NULL);
		err = usbp_transfer(dev->data_xfer);
		if (err && err != USBD_IN_PROGRESS) {
			DPRINTF(0,("usbf_do_request: data xfer=%p, %s\n",
			    xfer, usbd_errstr(err)));
		}
	}

next:
	/* Schedule another request transfer. */
	usbp_setup_default_xfer(dev->default_xfer, dev->usbd.default_pipe,
	    NULL, &dev->def_req, 0, 0, usbp_do_request);
	err = usbp_transfer(dev->default_xfer);
	if (err && err != USBD_IN_PROGRESS) {
		DPRINTF(0,("usbf_do_request: ctrl xfer=%p, %s\n", xfer,
		    usbd_errstr(err)));
	}

	DPRINTF(5,("usbf_do_request: done\n"));
}


static usbd_status
usbp_transfer(struct usbd_xfer *xfer)
{
	struct usbd_pipe *pipe = xfer->pipe;
	usbd_status err;

	err = pipe->methods->transfer(xfer);
	if (err != USBD_IN_PROGRESS && err) {
		if (xfer->rqflags & URQ_AUTO_DMABUF) {
			usbd_free_buffer(xfer);
			xfer->rqflags &= ~URQ_AUTO_DMABUF;
		}
	}
	return err;
}



/*
 * Attach a function driver.
 */
static usbd_status
usbp_probe_and_attach(struct device *parent, struct usbp_device *dev/*, int port*/)
{
	struct usbp_function_attach_args uaa;
	device_t dv;

	KASSERT(dev->function == NULL);

	bzero(&uaa, sizeof uaa);
	uaa.device = dev;

	/*
	 * The softc structure of a USB function driver must begin with a
	 * "struct usbf_function" member (instead of USBBASEDEV), which must
	 * be initialized in the function driver's attach routine.  Also, it
	 * should use usbf_devinfo_setup() to set the device identification.
	 */
	dv = config_found(parent, &uaa, NULL);
	if (dv != NULL) {
		dev->function = (struct usbp_function *)device_private(dv);
		return USBD_NORMAL_COMPLETION;
	}

	/*
	 * We failed to attach a function driver for this device, but the
	 * device can still function as a generic USB device without any
	 * interfaces.
	 */
	return USBD_NORMAL_COMPLETION;
}

static void
usbp_remove_device(struct usbp_device *dev/*, struct usbp_port *up*/)
{
	KASSERT(dev != NULL /*&& dev == up->device*/);

	if (dev->function != NULL)
		config_detach((struct device *)dev->function, DETACH_FORCE);
	if (dev->usbd.default_pipe != NULL)
		usbp_close_pipe(dev->usbd.default_pipe);
#if 0
	up->device = NULL;
#endif
	free(dev, M_USB);
}


static void
usbp_free_xfer(struct usbd_xfer *xfer)
{
	DPRINTF(999,("usbf_free_xfer: %p\n", xfer));
	if (xfer->rqflags & (URQ_DEV_DMABUF | URQ_AUTO_DMABUF))
		usbd_free_buffer(xfer);
	xfer->device->bus->methods->freex(xfer->device->bus, xfer);
}


/* Dequeue all pipe operations. */
void
usbp_abort_pipe(struct usbd_pipe *pipe);
void
usbp_abort_pipe(struct usbd_pipe *pipe)
{
	struct usbd_xfer *xfer;
	int s;

	s = splusb();
	pipe->repeat = 0;
	pipe->aborting = 1;

	while ((xfer = SIMPLEQ_FIRST(&pipe->queue)) != NULL) {
		DPRINTF(0,("usbf_abort_pipe: pipe=%p, xfer=%p\n", pipe,
		    xfer));
		/* Make the DC abort it (and invoke the callback). */
		pipe->methods->abort(xfer);
	}

	pipe->aborting = 0;
	splx(s);
}

/* Abort all pipe operations and close the pipe. */
static void
usbp_close_pipe(struct usbd_pipe *pipe)
{
	usbp_abort_pipe(pipe);
	pipe->methods->close(pipe);
	pipe->endpoint->refcnt--;
	free(pipe, M_USB);
}



usb_device_descriptor_t *
usbp_device_descriptor(struct usbp_device *dev)
{
	return &dev->usbd.ddesc;
}


usb_config_descriptor_t *
usbp_config_descriptor(struct usbp_device *dev, u_int8_t index)
{
	struct usbp_config *uc;

	SIMPLEQ_FOREACH(uc, &dev->configs, next) {
		if (index-- == 0)
			return uc->uc_cdesc;
	}
	return NULL;
}



struct usbd_xfer *
usbp_alloc_xfer(struct usbp_device *dev)
{
	struct usbd_xfer *xfer;

	/* allocate zero-filled buffer */
	xfer = dev->usbd.bus->methods->allocx(dev->usbd.bus);
	if (xfer == NULL)
		return NULL;
	xfer->device = &dev->usbd;
	callout_init(&xfer->timeout_handle, 0);
	DPRINTF(999,("usbf_alloc_xfer() = %p\n", xfer));
	return xfer;
}



/*
 * Add a new device configuration to an existing USB logical device.
 * The new configuration initially has zero interfaces.
 */
usbd_status
usbp_add_config(struct usbp_device *dev, struct usbp_config **ucp)
{
	struct usbp_config *uc;
	usb_config_descriptor_t *cd;

	uc = malloc(sizeof *uc, M_USB, M_NOWAIT | M_ZERO);
	if (uc == NULL)
		return USBD_NOMEM;

	cd = malloc(sizeof *cd, M_USB, M_NOWAIT | M_ZERO);
	if (cd == NULL) {
		free(uc, M_USB);
		return USBD_NOMEM;
	}

	uc->uc_device = dev;
	uc->uc_cdesc = cd;
	uc->uc_cdesc_size = sizeof *cd;
	SIMPLEQ_INIT(&uc->iface_head);

	cd->bLength = USB_CONFIG_DESCRIPTOR_SIZE;
	cd->bDescriptorType = UDESC_CONFIG;
	USETW(cd->wTotalLength, USB_CONFIG_DESCRIPTOR_SIZE);
	cd->bConfigurationValue = USB_UNCONFIG_NO + 1 +
	    dev->usbd.ddesc.bNumConfigurations;
	cd->iConfiguration = 0;
	cd->bmAttributes = UC_BUS_POWERED | UC_SELF_POWERED;
#if 0
	cd->bMaxPower = 100 / UC_POWER_FACTOR; /* 100 mA */
#else
	cd->bMaxPower = 0; /* XXX 0 mA */
#endif

	SIMPLEQ_INSERT_TAIL(&dev->configs, uc, next);
	dev->usbd.ddesc.bNumConfigurations++;

	if (ucp != NULL)
		*ucp = uc;
	return USBD_NORMAL_COMPLETION;
}


/*
 * Allocate memory for a new descriptor at the end of the existing
 * device configuration descriptor.
 */
usbd_status
usbp_add_config_desc(struct usbp_config *uc, usb_descriptor_t *d,
    usb_descriptor_t **dp)
{
    	usb_config_descriptor_t *cd;
	size_t oldsize;
	size_t newsize;

	oldsize = uc->uc_cdesc_size;
	newsize = oldsize + d->bLength;
	if (d->bLength < sizeof(usb_descriptor_t) || newsize > 65535)
		return USBD_INVAL;

	cd = realloc(uc->uc_cdesc, newsize, M_USBDEV, M_NOWAIT);
	if (cd == NULL)
		return USBD_NOMEM;
	printf("%s: oldsize=%zd newsize=%zd olddesc=%p newptr=%p\n",
	    __func__,
	    oldsize, newsize, uc->uc_cdesc, cd);


	uc->uc_cdesc = cd;
	uc->uc_cdesc_size = newsize;

	bcopy(d, (char *)cd + oldsize, d->bLength);
	USETW(cd->wTotalLength, newsize);
	if (dp != NULL)
		*dp = (usb_descriptor_t *)((char *)cd + oldsize);
	return USBD_NORMAL_COMPLETION;
}

usbd_status
usbp_add_interface(struct usbp_config *uc, u_int8_t bInterfaceClass,
    u_int8_t bInterfaceSubClass, u_int8_t bInterfaceProtocol,
    const char *string, struct usbp_interface **uip)
{
	struct usbp_interface *ui;
	usb_interface_descriptor_t *id;

	if (uc->uc_closed)
		return USBD_INVAL;

	ui = malloc(sizeof *ui, M_USB, M_NOWAIT | M_ZERO);
	if (ui == NULL)
		return USBD_NOMEM;

	id = malloc(sizeof *id, M_USB, M_NOWAIT | M_ZERO);
	if (id == NULL) {
		free(ui, M_USB);
		return USBD_NOMEM;
	}

	ui->config = uc;
	ui->usbd.idesc = id;
	LIST_INIT(&ui->usbd.pipes);
	SIMPLEQ_INIT(&ui->endpoint_head);
	ui->usbd.endpoints = malloc(sizeof (struct usbp_endpoint) * 16,     // XXX
	    M_USB, M_NOWAIT | M_ZERO);
	if (ui->usbd.endpoints == NULL) {
		free(id, M_USB);
		free(ui, M_USB);
		return USBD_NOMEM;
	}

	id->bLength = USB_INTERFACE_DESCRIPTOR_SIZE;
	id->bDescriptorType = UDESC_INTERFACE;
	id->bInterfaceNumber = uc->uc_cdesc->bNumInterface;
	id->bInterfaceClass = bInterfaceClass;
	id->bInterfaceSubClass = bInterfaceSubClass;
	id->bInterfaceProtocol = bInterfaceProtocol;
	id->iInterface = 0; /*usbf_add_string(uc->uc_device, string);*/ /* XXX */

	SIMPLEQ_INSERT_TAIL(&uc->iface_head, ui, next);
	uc->uc_cdesc->bNumInterface++;

	*uip = ui;
	return USBD_NORMAL_COMPLETION;
}

usbd_status
usbp_add_endpoint(struct usbp_interface *ui, u_int8_t bEndpointAddress,
    u_int8_t bmAttributes, u_int16_t wMaxPacketSize, u_int8_t bInterval,
    struct usbp_endpoint **uep)
{
	struct usbp_endpoint *ue;
	usb_endpoint_descriptor_t *ed;
	int num_ep;

	if (ui->config->uc_closed)
		return USBD_INVAL;

	num_ep = ui->usbd.idesc->bNumEndpoints;

	ue = malloc(sizeof *ue, M_USB, M_NOWAIT | M_ZERO);
	if (ue == NULL)
		return USBD_NOMEM;
	ed = malloc(sizeof *ed, M_USB, M_NOWAIT | M_ZERO);
//	ed = kmem_zalloc(sizeof *ed, KM_NOSLEEP);
	if (ed == NULL) {
		free(ue, M_USB);
		return USBD_NOMEM;
	}

	ue->iface = ui;
	ue->usbd.edesc = ed;

	ed->bLength = USB_ENDPOINT_DESCRIPTOR_SIZE;
	ed->bDescriptorType = UDESC_ENDPOINT;
	ed->bEndpointAddress = bEndpointAddress;
	ed->bmAttributes = bmAttributes;
	USETW(ed->wMaxPacketSize, wMaxPacketSize);
	ed->bInterval = bInterval;

	SIMPLEQ_INSERT_TAIL(&ui->endpoint_head, ue, next);

	ui->usbd.endpoints[num_ep++] = &ue->usbd;
	ui->usbd.idesc->bNumEndpoints = num_ep;

	*uep = ue;
	return USBD_NORMAL_COMPLETION;
}

/*
 * Close the configuration, thereby combining all descriptors and creating
 * the real USB configuration descriptor that can be sent to the USB host.
 */
usbd_status
usbp_end_config(struct usbp_config *uc)
{
	struct usbp_interface *ui;
	struct usbp_endpoint *ue;
//	usb_descriptor_t *d;
	usbd_status err = USBD_NORMAL_COMPLETION;
	size_t newsize, idx;
	unsigned char *newdesc;

	if (uc->uc_closed)
		return USBD_INVAL;

	
	newsize = uc->uc_cdesc_size;
	SIMPLEQ_FOREACH(ui, &uc->iface_head, next) {
		newsize += ui->usbd.idesc->bLength;
		SIMPLEQ_FOREACH(ue, &ui->endpoint_head, next) {
			newsize += ue->usbd.edesc->bLength;
		}
	}

	newdesc = realloc(uc->uc_cdesc, newsize, M_USB, M_NOWAIT | M_ZERO);
		
	idx = uc->uc_cdesc_size;
	SIMPLEQ_FOREACH(ui, &uc->iface_head, next) {
		size_t len;

		len = ui->usbd.idesc->bLength;
		memcpy(newdesc + idx, ui->usbd.idesc, len);
		free(ui->usbd.idesc, M_USB);
		ui->usbd.idesc = (usb_interface_descriptor_t *)(newdesc + idx);
		idx += len;

		SIMPLEQ_FOREACH(ue, &ui->endpoint_head, next) {
			len = ue->usbd.edesc->bLength;
			memcpy(newdesc + idx, ue->usbd.edesc, len);
			free(ue->usbd.edesc, M_USB);
			ue->usbd.edesc = (usb_endpoint_descriptor_t *)(newdesc + idx);
			idx += len;
		}
	}


	uc->uc_cdesc = (usb_config_descriptor_t *)newdesc;
	uc->uc_cdesc_size = newsize;
	USETW(uc->uc_cdesc->wTotalLength, newsize);
	uc->uc_closed = 1;
	return err;
}












int
usbp_interface_number(struct usbp_interface *iface)
{
	return iface->usbd.idesc->bInterfaceNumber;
}

u_int8_t
usbd_endpoint_address(struct usbd_endpoint *endpoint)
{
	return endpoint->edesc->bEndpointAddress;
}

usbd_status
usbp_open_pipe(struct usbp_interface *iface, u_int8_t address,
    struct usbd_pipe **pipe)
{
	return usbp_open_pipe_ival(iface, address, pipe, 0);
}

usbd_status
usbp_open_pipe_ival(struct usbp_interface *iface, u_int8_t address,
    struct usbd_pipe **pipe, int ival)
{
	struct usbp_endpoint *ep;
	struct usbd_pipe *p;
	usbd_status err;

	ep = usbp_iface_endpoint(iface, address);
	if (ep == NULL)
		return USBD_BAD_ADDRESS;

	err = usbp_setup_pipe(iface->config->uc_device, iface, ep,
	    ival, &p);
	if (err)
		return err;
	LIST_INSERT_HEAD(&iface->usbd.pipes, p, next);
	*pipe = p;
	return USBD_NORMAL_COMPLETION;
}



/* Called at splusb() */
void
usbp_transfer_complete(struct usbd_xfer *xfer)
{
	struct usbd_pipe *pipe = xfer->pipe;
	int repeat = pipe->repeat;

	//SPLUSBCHECK;
	DPRINTF(1,("usbp_transfer_complete: xfer=%s pipe=%p running=%d callback=%p\n",
		   usbp_describe_xfer(xfer), pipe, pipe->running, xfer->callback));
#ifdef USBP_DEBUG
	if (usbpdebug > 20)
		usbp_dump_buffer(xfer);
#endif

	if (!repeat) {
		/* Remove request from queue. */
		KASSERT(SIMPLEQ_FIRST(&pipe->queue) == xfer);
		SIMPLEQ_REMOVE_HEAD(&pipe->queue, next);
	}

	if (xfer->status == USBD_NORMAL_COMPLETION &&
	    xfer->actlen < xfer->length &&
	    !(xfer->flags & USBD_SHORT_XFER_OK)) {
		DPRINTF(0,("usbf_transfer_complete: short xfer=%p %u<%u\n",
		    xfer, xfer->actlen, xfer->length));
		xfer->status = USBD_SHORT_XFER;
	}

	if (xfer->callback != NULL)
		xfer->callback(xfer, xfer->priv, xfer->status);

	pipe->methods->done(xfer);

	if (xfer->flags & USBD_SYNCHRONOUS)
		wakeup(xfer);

	if (!repeat) {
		if (xfer->status != USBD_NORMAL_COMPLETION &&
		    pipe->iface != NULL) /* not control pipe */
			pipe->running = 0;
		else
			usbp_start_next(pipe);
	}
}


usbd_status
usbp_insert_transfer(struct usbd_xfer *xfer)
{
	struct usbd_pipe *pipe = xfer->pipe;
	usbd_status err;
	int s;


	DPRINTF(1,("%s: xfer=%s pipe=%p running=%d callback=%p\n", __func__,
		   usbp_describe_xfer(xfer),
		   pipe, pipe->running, xfer->callback));

	s = splusb();
	SIMPLEQ_INSERT_TAIL(&pipe->queue, xfer, next);
	if (pipe->running)
		err = USBD_IN_PROGRESS;
	else {
		pipe->running = 1;
		err = USBD_NORMAL_COMPLETION;
	}
	splx(s);
	return err;
}

void
usbp_start_next(struct usbd_pipe *pipe)
{
	struct usbd_xfer *xfer;
	usbd_status err;

	DPRINTF(1,("%s: pipe=%p running=%d first xfer=%p\n", __func__, pipe, pipe->running,
		SIMPLEQ_FIRST(&pipe->queue)));
		
	//SPLUSBCHECK;

	/* Get next request in queue. */
	xfer = SIMPLEQ_FIRST(&pipe->queue);
	if (xfer == NULL)
		pipe->running = 0;
	else {
		err = pipe->methods->start(xfer);
		if (err != USBD_IN_PROGRESS) {
			printf("usbf_start_next: %s\n", usbd_errstr(err));
			pipe->running = 0;
			/* XXX do what? */
		}
	}
}


/*
 * Add a string descriptor to a logical device and return the string's id.
 *
 * If there is not enough memory available for the new string descriptor, or
 * if there is no unused string id left, return the id of the empty string
 * instead of failing.
 */
u_int8_t
usbp_add_string(struct usbp_device *dev, const char *string)
{
	usb_string_descriptor_t *sd;
	size_t oldsize;
	size_t newsize;
	size_t len, i;
	u_int8_t id;

	if (string == NULL || *string == '\0' ||
	    dev->string_id == USBP_STRING_ID_MAX)
		return USBP_EMPTY_STRING_ID;

	if ((len = strlen(string)) >= USB_MAX_STRING_LEN)
		len = USB_MAX_STRING_LEN - 1;

	oldsize = dev->sdesc_size;
	newsize = oldsize + 2 + 2 * len;

	sd = realloc(dev->sdesc, newsize, M_USBDEV, M_NOWAIT);
	if (sd == NULL)
		return USBP_EMPTY_STRING_ID;
	dev->sdesc = sd;
	dev->sdesc_size = newsize;

	sd = (usb_string_descriptor_t *)((char *)sd + oldsize);
	sd->bLength = newsize - oldsize;
	sd->bDescriptorType = UDESC_STRING;
	for (i = 0; string[i] != '\0' && i < len; i++)
		USETW(sd->bString[i], string[i]);

	id = dev->string_id++;
	return id;
}


#ifdef USBP_DEBUG
struct usb_enum_str {
	int code;
	const char * const str;
};

static const struct usb_enum_str usb_request_str[] = {
	{ UR_GET_STATUS,		"GET STATUS"             },
	{ UR_CLEAR_FEATURE,		"CLEAR FEATURE"          },
	{ UR_SET_FEATURE,		"SET FEATURE"            },
	{ UR_SET_ADDRESS,		"SET ADDRESS"            },
	{ UR_GET_DESCRIPTOR,		"GET DESCRIPTOR"         },
	{ UR_SET_DESCRIPTOR,		"SET DESCRIPTOR"         },
	{ UR_GET_CONFIG,		"GET CONFIG"             },
	{ UR_SET_CONFIG,		"SET CONFIG"             },
	{ UR_GET_INTERFACE,		"GET INTERFACE"          },
	{ UR_SET_INTERFACE,		"SET INTERFACE"          },
	{ UR_SYNCH_FRAME,		"SYNCH FRAME"            },
	{ 0, NULL }
};

static const struct usb_enum_str usb_request_type_str[] = {
	{ UT_READ_DEVICE,		"Read Device"            },
	{ UT_READ_INTERFACE,		"Read Interface"         },
	{ UT_READ_ENDPOINT,		"Read Endpoint"          },
	{ UT_WRITE_DEVICE,		"Write Device"           },
	{ UT_WRITE_INTERFACE,		"Write Interface"        },
	{ UT_WRITE_ENDPOINT,		"Write Endpoint"         },
	{ UT_READ_CLASS_DEVICE,		"Read Class Device"      },
	{ UT_READ_CLASS_INTERFACE,	"Read Class Interface"   },
	{ UT_READ_CLASS_OTHER,		"Read Class Other"       },
	{ UT_READ_CLASS_ENDPOINT,	"Read Class Endpoint"    },
	{ UT_WRITE_CLASS_DEVICE,	"Write Class Device"     },
	{ UT_WRITE_CLASS_INTERFACE,	"Write Class Interface"  },
	{ UT_WRITE_CLASS_OTHER,		"Write Class Other"      },
	{ UT_WRITE_CLASS_ENDPOINT,	"Write Class Endpoint"   },
	{ UT_READ_VENDOR_DEVICE,	"Read Vendor Device"     },
	{ UT_READ_VENDOR_INTERFACE,	"Read Vendor Interface"  },
	{ UT_READ_VENDOR_OTHER,		"Read Vendor Other"      },
	{ UT_READ_VENDOR_ENDPOINT,	"Read Vendor Endpoint"   },
	{ UT_WRITE_VENDOR_DEVICE,	"Write Vendor Device"    },
	{ UT_WRITE_VENDOR_INTERFACE,	"Write Vendor Interface" },
	{ UT_WRITE_VENDOR_OTHER,	"Write Vendor Other"     },
	{ UT_WRITE_VENDOR_ENDPOINT,	"Write Vendor Endpoint"  },
	{ 0, NULL }
};

static const struct usb_enum_str usb_request_desc_str[] = {
	{ UDESC_DEVICE,			"Device"                       },
	{ UDESC_CONFIG,			"Configuration"                },
	{ UDESC_STRING,			"String"                       },
	{ UDESC_INTERFACE,		"Interface"                    },
	{ UDESC_ENDPOINT,		"Endpoint"                     },
	{ UDESC_DEVICE_QUALIFIER,	"Device Qualifier"             },
	{ UDESC_OTHER_SPEED_CONFIGURATION, "Other Speed Configuration" },
	{ UDESC_INTERFACE_POWER,	"Interface Power"              },
	{ UDESC_OTG,			"OTG"                          },
	{ UDESC_CS_DEVICE,		"Class-specific Device"        },
	{ UDESC_CS_CONFIG,		"Class-specific Configuration" },
	{ UDESC_CS_STRING,		"Class-specific String"        },
	{ UDESC_CS_INTERFACE,		"Class-specific Interface"     },
	{ UDESC_CS_ENDPOINT,		"Class-specific Endpoint"      },
	{ UDESC_HUB,			"Hub"                          },
	{ 0, NULL }
};

static const char *
usb_enum_string(const struct usb_enum_str *tab, int code)
{
	static char buf[16];

	while (tab->str != NULL) {
		if (tab->code == code)
			return tab->str;
		tab++;
	}

	(void)snprintf(buf, sizeof buf, "0x%02x", code);
	return buf;
}

static const char *
usbp_request_code_string(usb_device_request_t *req)
{
	static char buf[32];

	(void)snprintf(buf, sizeof buf, "%s",
	    usb_enum_string(usb_request_str, req->bRequest));
	return buf;
}

static const char *
usbp_request_type_string(usb_device_request_t *req)
{
	static char buf[32];

	(void)snprintf(buf, sizeof buf, "%s",
	    usb_enum_string(usb_request_type_str, req->bmRequestType));
	return buf;
}

static const char *
usbp_request_desc_string(usb_device_request_t *req)
{
	static char buf[32];
	u_int8_t type = UGETW(req->wValue) >> 8;
	u_int8_t index = UGETW(req->wValue) & 0xff;

	(void)snprintf(buf, sizeof buf, "%s/%u",
	    usb_enum_string(usb_request_desc_str, type), index);
	return buf;
}

void
usbp_dump_request(struct usbp_device *dev, usb_device_request_t *req)
{
	struct usbp_softc *sc = device_private(dev->usbd.bus->usbctl);

	printf("%s: %s request %s\n",
	    DEVNAME(sc), usbp_request_type_string(req),
	    usbp_request_code_string(req));

	if (req->bRequest == UR_GET_DESCRIPTOR)
		printf("%s:    VALUE:  0x%04x (%s)\n", DEVNAME(sc),
		    UGETW(req->wValue), usbp_request_desc_string(req));
	else
		printf("%s:    VALUE:  0x%04x\n", DEVNAME(sc),
		    UGETW(req->wValue));

	printf("%s:    INDEX:  0x%04x\n", DEVNAME(sc), UGETW(req->wIndex));
	printf("%s:    LENGTH: 0x%04x\n", DEVNAME(sc), UGETW(req->wLength));
}

void
usbp_dump_buffer(struct usbd_xfer *xfer)
{
	device_t dev = xfer->pipe->device->bus->usbctl;
	struct usbd_endpoint *ep = xfer->pipe->endpoint;
	int index = usbd_endpoint_index(ep);
	int dir = usbd_endpoint_dir(ep);
	u_char *p = xfer->buffer;
	u_int i;

	printf("%s: ep%d-%s, actlen=%u length=%u, %s", device_xname(dev), index,
	    (xfer->rqflags & URQ_REQUEST) ? "setup" :
	    (index == 0 ? "in" : (dir == UE_DIR_IN ? "in" : "out")),
	       xfer->actlen, xfer->length, usbd_errstr(xfer->status));

	for (i = 0; i < xfer->length; i++) {
		if ((i % 16) == 0)
			printf("\n%4x:", i);
		else if ((i % 8) == 0)
			printf(" ");
		printf(" %02x", p[i]);
	}
	printf("\n");
}


#endif

char *
usbp_describe_xfer(struct usbd_xfer *xfer)
{
	static char buf[100];
	struct usbd_endpoint *ep = xfer->pipe->endpoint;
	int index = usbd_endpoint_index(ep);
	int dir = usbd_endpoint_dir(ep);

	snprintf(buf, sizeof buf, "%p:ep%d-%s len=%d,actlen=%d,pipe=%p,ep=%p(0x%x)",
		 xfer,
		 index,
		 (xfer->rqflags & URQ_REQUEST) ? "setup" :
		 (index == 0 ? "in" : (dir == UE_DIR_IN ? "in" : "out")),
	    xfer->length, xfer->actlen, xfer->pipe, xfer->pipe->endpoint, usbd_endpoint_address(ep));
		 
	return buf;
}
