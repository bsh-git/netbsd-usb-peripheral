/*	$NetBSD$ */
/*	$OpenBSD: usbf.c,v 1.16 2013/11/18 20:21:51 deraadt Exp $	*/

/*-
 * Copyright (c) 2015, 2016 Genetec Corporation.  All rights reserved.
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

/* Partly derived from OpenBSD's usbf.c */
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
#include <sys/bitops.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>

#include <dev/usb/usbp.h>

#ifndef	USBP_DEFAULT_VENDOR_ID
#define	USBP_DEFAULT_VENDOR_ID 0x0001		/* XXX */
#endif
#ifndef	USBP_DEFAULT_PRODUCT_ID
#define	USBP_DEFAULT_PRODUCT_ID	0x0001
#endif
#ifndef	USBP_DEFAULT_DEVICE_BCD
#define	USBP_DEFAULT_DEVICE_BCD	0x0100
#endif
#ifndef	USBP_DEFAULT_MANUFACTURER
#define	USBP_DEFAULT_MANUFACTURER	"NetBSD.org"
#endif
#ifndef	USBP_DEFAULT_PRODUCT_NAME
#define	USBP_DEFAULT_PRODUCT_NAME	"Some USB device"
#endif
#ifndef	USBP_DEFAULT_SERIAL
#define	USBP_DEFAULT_SERIAL	"1.00"
#endif

struct usbp_port {
	struct usbd_port usbd;
};

#define USBP_EMPTY_STRING_ID		(USB_LANGUAGE_TABLE+1)
#define USBP_STRING_ID_MIN		(USB_LANGUAGE_TABLE+2)
#define USBP_STRING_ID_MAX		255

struct usbp_device {
	struct usbd_device usbd;

	device_t	usbp;	/* USBP logical device */

	usb_config_descriptor_t *cdesc;
	size_t	cdesc_size;
	usb_status_t		 status;	/* device status */
	uByte current_config;

	struct usbd_xfer	*default_xfer;	/* device request xfer */
	struct usbd_xfer	*data_xfer;	/* request response xfer */

	usb_device_request_t	 def_req;	/* device request buffer */

	enum USBP_DEVICE_STATE {
		USBP_DEV_INIT,
		USBP_DEV_NO_INTERFACE,	/* no interfaces attached */
		USBP_DEV_ASSEMBLED,	/* has interfaces, not connected to a host */
		USBP_DEV_READY,	        /* waiting for host to connect */
		USBP_DEV_CONNECTED,
		USBP_DEV_ADDRESS,	/* after set address */
		USBP_DEV_CONFIGURED
	} dev_state;

	bool connect_auto;	/* you may fire when ready */

	int	n_interfaces;
	SIMPLEQ_HEAD(, usbp_interface)  interface_list;

	/* USB interfaces actually configured in this device. */
	int n_picked_interfaces;
	struct usbp_interface **picked_interfaces;

	/* strings in configuration */
	SIMPLEQ_HEAD(, usbp_string) string_list;
	__BITMAP_TYPE(usbp_string_id_map, uint32_t, USBP_STRING_ID_MAX+1) string_id_map;
	struct usbp_string *empty_string;
	usb_string_descriptor_t *sdesc;		/* string descriptors */
	size_t			 sdesc_size;	/* size of ud_sdesc */
	/* scratch area to send string descriptor back to the host */
	usb_string_descriptor_t	string_descriptor;


};


struct usbp_string {
	SIMPLEQ_ENTRY (usbp_string) next;
	const char *str;
	int refcount;
	uByte id;

};

struct usbp_string *usbp_intern_string(struct usbp_device *, const char *);
void usbp_release_string(struct usbp_device *, struct usbp_string *);
#define	usbp_string_id(s)	((s)==NULL ? 0 : (s)->id)

struct usbp_softc {
	device_t sc_dev;
	struct usbp_bus  *sc_bus;
	struct usbp_device *sc_usbdev;
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
static void usbp_do_request(struct usbd_xfer *xfer, void *priv, usbd_status err);
static usbd_status usbp_transfer(struct usbd_xfer *xfer);
static usbd_status usbp_search_interfaces(struct device *parent, struct usbp_device *dev/*, int port*/);
static void usbp_remove_device(struct usbp_device *dev/*, struct usbp_port *up*/);
//static void usbp_free_xfer(struct usbd_xfer *xfer);
static void usbp_close_pipe(struct usbd_pipe *pipe);
static void usbp_set_address(struct usbp_device *dev, u_int8_t address);
static int usbp_match(device_t, cfdata_t, void *);
static void usbp_attach(device_t, device_t, void *);
static void usbp_child_detached(device_t, device_t);
static usbd_status usbp_set_config(struct usbp_device *dev, u_int8_t new);
static void start_usbp(device_t);
static usbd_status reassemble_interfaces(struct usbp_device *);
static const usb_string_descriptor_t *get_string_descriptor(struct usbp_device *, int);
static usbd_status pass_request_to_iface(struct usbp_device *, usb_device_request_t *, void **);

usb_config_descriptor_t *usbp_config_descriptor(struct usbp_device *dev, u_int8_t index);


extern struct cfdriver usbf_cd;

CFATTACH_DECL3_NEW(usbp, sizeof(struct usbp_softc),
		   usbp_match, usbp_attach, NULL, NULL, NULL, usbp_child_detached, 0);


#define	USBP_DEBUG_TRACE	(1<<0)
#define	USBP_DEBUG_MISC 	(1<<1)
#define	USBP_DEBUG_CONTROLLER 	(1<<4)
#define	USBP_DEBUG_DESCRIPTOR	(1<<5)
#define	USBP_DEBUG_STRING	(1<<8)
#ifndef USBP_DEBUG
#define DPRINTF(l, x)	do {} while (0)
#else
int usbpdebug = 0xffffffff;
#define DPRINTF(l, x)	if ((l) & usbpdebug) printf x; else
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
	sc->sc_bus->usbd.pipe_size = sizeof (struct usbd_pipe);

	sc->sc_bus->usbd.methods->get_lock(&sc->sc_bus->usbd,
	    &sc->sc_bus->usbd.lock);

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

	/* initialize USB device. there is only one device per client controller. */
	err = usbp_new_device(self, sc->sc_bus, speed/*, 0, 0, &sc->sc_port*/);
	if (err) {
		printf("%s: usbp_new_device failed, %s\n", DEVNAME(sc),
		    usbd_errstr(err));
		sc->sc_dying = true;
		return;
	}

	config_defer(self, start_usbp);
#if 0
	/* Create a process context for asynchronous tasks. */
	if (kthread_create(PRI_NONE, 0, NULL, usbf_task_thread, sc,
			   &sc->sc_proc, "%s", device_xname(self)) ) {
		aprint_normal_dev(self, "unable to create event thread for USB client\n");
	}
#endif
	
}


static void
start_usbp(device_t self)
{
	struct usbp_softc *sc = device_private(self);
	struct usbp_device *dev = sc->sc_usbdev;

	DPRINTF(USBP_DEBUG_TRACE, ("%s: assemble interfaces\n", __func__));
	reassemble_interfaces(dev);

}


struct iface_assembly {
	struct usbp_interface *iface;
	int n_endpoints;
	struct usbp_endpoint_request epspec[USB_MAX_ENDPOINTS];
};

/*
 * select interfaces that can be configured in the device according to
 * endpoints restrictions.
 */
static int
select_interfaces(struct usbp_device *device, struct iface_assembly *ifa)
{
	struct usbp_interface *iface;
	u_int epmap = 0, tmpmap;
	int n_if = 0;
	enum USBP_PIPE0_USAGE pipe0 = USBP_PIPE0_NOTUSED;
	struct usbp_bus *bus = (struct usbp_bus *)(device->usbd.bus);
	
	SIMPLEQ_FOREACH(iface, &device->interface_list, next) {
		const struct usbp_interface_spec *ispec = iface->ispec;
		int i;
		
		switch (pipe0) {
		case USBP_PIPE0_NOTUSED:
			pipe0 = ispec->pipe0_usage;
			break;
		case USBP_PIPE0_SHARED:
			if (ispec->pipe0_usage == USBP_PIPE0_EXCLUSIVE)
				goto not_used;
			break;
		case USBP_PIPE0_EXCLUSIVE:
			if (ispec->pipe0_usage != USBP_PIPE0_NOTUSED)
				goto not_used;
			break;
		}
			
		tmpmap = epmap;
		for (i=0; i < ispec->num_endpoints; ++i) {
			struct usbp_endpoint_request *ep = &ifa[n_if].epspec[i];
			usbd_status st;
			*ep = ispec->endpoints[i];
			st = bus->usbp_methods->select_endpoint(bus, ep, tmpmap);
			DPRINTF(USBP_DEBUG_CONTROLLER,
			    ("%s: client controller returned %d\n", __func__, st));
			if (st) {
				/* can't assing an endpoint.  This
				 * interface is not selected for the
				 * device */
				goto not_used;
			}
			tmpmap |= (1<<UE_GET_ADDR(ep->address));
		}
		epmap = tmpmap;
		ifa[n_if].n_endpoints = ispec->num_endpoints;
		ifa[n_if].iface = iface;
		n_if++;
		continue;
	not_used: ;
	}

	return n_if;
}

static bool
compare_interfaces(struct iface_assembly *newifs, int n_new,
    struct usbp_interface *curifs[], int n_cur)
{
	int i;
	
	if (n_new != n_cur)
		return false;

	for (i=0; i < n_cur; ++i) {
		if (newifs[i].iface != curifs[i])
			return false;
	}

	return true;
}

#if 0
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
#endif


static void
build_device_descriptor(struct usbp_device *usbdev, struct iface_assembly *ifa, int n_if)
{
	usb_device_descriptor_t *dd = &usbdev->usbd.ddesc;
	uint16_t vendor, product, bcddev;
	struct usbp_interface *iface;
	struct usbp_bus *bus = (struct usbp_bus *)usbdev->usbd.bus;
	int i;


	/*
	 * Initialize device descriptor.  The function driver for this
	 * device (attached below) must complete the device descriptor.
	 */
	dd->bLength = USB_DEVICE_DESCRIPTOR_SIZE;
	dd->bDescriptorType = UDESC_DEVICE;
	dd->bMaxPacketSize = bus->ep0_maxp;
	dd->bNumConfigurations = 1;
	if (bus->usbd.usbrev >= USBREV_2_0)
		USETW(dd->bcdUSB, 0x0200);
	else
		USETW(dd->bcdUSB, 0x0101);

	for (i=0; i < n_if; ++i) {
		if (ifa[i].iface->devinfo.class_id_valid)
			break;
	}
	if (i >= n_if) {
		dd->bDeviceClass = UDCLASS_IN_INTERFACE;
		dd->bDeviceSubClass = 0;
		dd->bDeviceProtocol = 0;
	}
	else {
		dd->bDeviceClass = ifa[i].iface->devinfo.class_id;
		dd->bDeviceSubClass = ifa[i].iface->devinfo.subclass_id;
		dd->bDeviceProtocol = ifa[i].iface->devinfo.protocol;
	}
		
	for (i=0; i < n_if; ++i) {
		if (ifa[i].iface->devinfo.vendor_id_valid)
			break;
	}
	if (i >= n_if) {
		struct usbp_string *s;
		
		vendor = USBP_DEFAULT_VENDOR_ID;
		product = USBP_DEFAULT_PRODUCT_ID;
		bcddev = USBP_DEFAULT_DEVICE_BCD;

		s = usbp_intern_string(usbdev, USBP_DEFAULT_MANUFACTURER);
		dd->iManufacturer = usbp_string_id(s);
		s = usbp_intern_string(usbdev, USBP_DEFAULT_PRODUCT_NAME);
		dd->iProduct = usbp_string_id(s);
		s = usbp_intern_string(usbdev, USBP_DEFAULT_SERIAL);
		dd->iSerialNumber = usbp_string_id(s);
		
	}
	else {
		iface = ifa[i].iface;
		
		vendor = iface->devinfo.vendor_id;
		product = iface->devinfo.product_id;
		bcddev = iface->devinfo.bcd_device;

		dd->iManufacturer = usbp_string_id(iface->string[USBP_IF_STR_MANUFACTURER]);
		dd->iProduct = usbp_string_id(iface->string[USBP_IF_STR_PRODUCT_NAME]);
		dd->iSerialNumber = usbp_string_id(iface->string[USBP_IF_STR_SERIAL]);
	}
	USETW(dd->idVendor, vendor);
	USETW(dd->idProduct, product);
	USETW(dd->bcdDevice, bcddev);

}

/*
 * build configuration descriptor which consists of interface
 * descriptors and endpoint descriptors.
 *
 * also, fixup the interface.
 */
static usbd_status
build_config_descriptor(struct usbp_device *usbdev, struct iface_assembly *ifa, int n_if)
{
	size_t total_size;
	uint8_t *area, *ap;
	usb_config_descriptor_t *cd;
	int i, j;

	DPRINTF(USBP_DEBUG_TRACE, ("%s\n", __func__));

	if (usbdev->cdesc) {
		kmem_free(usbdev->cdesc, usbdev->cdesc_size);
		usbdev->cdesc = NULL;
	}

	/* calculate the size of config descriptor. */
	total_size = USB_CONFIG_DESCRIPTOR_SIZE;
	total_size += USB_INTERFACE_DESCRIPTOR_SIZE * n_if;
	for (i=0; i < n_if; ++i) {
		total_size += ifa[i].iface->additional_idesc_size;
		total_size += USB_ENDPOINT_DESCRIPTOR_SIZE *
		    ifa[i].iface->ispec->num_endpoints;
	}

	DPRINTF(USBP_DEBUG_DESCRIPTOR,
	    ("%s: config_descriptor_size=%zd\n", __func__, total_size));

	area = kmem_alloc(total_size, KM_SLEEP);
	if (area == NULL)
		return USBD_NOMEM;
	usbdev->cdesc = cd = (usb_config_descriptor_t *)area;
	usbdev->cdesc_size = total_size;

	cd->bLength = USB_CONFIG_DESCRIPTOR_SIZE;
	cd->bDescriptorType = UDESC_CONFIG;
	
	USETW(cd->wTotalLength, total_size);
	cd->bNumInterface = n_if;
	cd->bConfigurationValue = 1;
	cd->iConfiguration = 0;
	cd->bmAttributes = UC_BUS_POWERED | UC_SELF_POWERED;
#if 0
	cd->bMaxPower = 100 / UC_POWER_FACTOR; /* 100 mA */
#else
	cd->bMaxPower = 0; /* XXX 0 mA */
#endif
	ap = area + USB_CONFIG_DESCRIPTOR_SIZE;
	for (i=0; i < n_if; ++i) {
		usb_interface_descriptor_t *id =
		    (usb_interface_descriptor_t *)ap;
		struct usbp_interface *iface = ifa[i].iface;

		id->bLength = USB_INTERFACE_DESCRIPTOR_SIZE;
		id->bDescriptorType = UDESC_INTERFACE;
		id->bInterfaceNumber = i + 1;
		id->bAlternateSetting = 0;
		id->bInterfaceClass = iface->ispec->class_id;
		id->bInterfaceSubClass = iface->ispec->subclass_id;
		id->bInterfaceProtocol = iface->ispec->protocol;
		id->iInterface = usbp_string_id(iface->string[USBP_IF_STR_DESCRIPTION]);
		id->bNumEndpoints = iface->ispec->num_endpoints;

		ap += USB_INTERFACE_DESCRIPTOR_SIZE;

		if (iface->usbd.endpoints != NULL) {
			/* XXX */
		}
		iface->usbd.endpoints = kmem_alloc(sizeof (struct usbd_endpoint) *
		    iface->ispec->num_endpoints,
		    KM_NOSLEEP);
		if (iface->usbd.endpoints == NULL)
			return USBD_NOMEM;

		for (j=0; j < iface->ispec->num_endpoints; ++j) {
			usb_endpoint_descriptor_t *ed =
			    (usb_endpoint_descriptor_t *)ap;
			struct usbd_endpoint *ep = &iface->usbd.endpoints[j];

			ed->bLength = USB_ENDPOINT_DESCRIPTOR_SIZE;
			ed->bDescriptorType = UDESC_ENDPOINT;

			ed->bEndpointAddress = ifa[i].epspec[j].address;
			ed->bmAttributes = ifa[i].epspec[j].attributes;
			USETW(ed->wMaxPacketSize, ifa[i].epspec[j].packetsize);

			ep->edesc = ed;
			ep->refcnt = 0;
			ep->datatoggle = 0;
			
			printf("%s: interface=%p endpoitns=%p endpoint[%d]=0x%x (%p)\n",
			    __func__,
			    iface, iface->usbd.endpoints,
			    j, ep->edesc->bEndpointAddress, ed);
			ap += USB_ENDPOINT_DESCRIPTOR_SIZE;
		}

		if (iface->additional_idesc_size > 0) {
			memcpy(ap, iface->additional_idesc, iface->additional_idesc_size);
			ap += iface->additional_idesc_size;
		}

		if (iface->methods->fixup_idesc)
			iface->methods->fixup_idesc(iface, id);

		iface->usbd.idesc = id;
		printf("%s: interface=%p idesc=%p numEndpoints=%d\n",
		    __func__, iface, id, id->bNumEndpoints);
	}


	return USBD_NORMAL_COMPLETION;
}

static inline bool
is_vbus_on(struct usbp_bus *bus)
{
	if (bus->usbp_methods->is_connected == NULL) {
		/* platform provides the method. assume we are
		 * connected to the host. */
		return true;
	}
	return bus->usbp_methods->is_connected(bus);
}

static usbd_status
activate_interface(struct usbp_interface *iface)
{
	iface->methods->configured(iface);
	return USBD_NORMAL_COMPLETION;
}

static usbd_status
deactivate_interface(struct usbp_interface *iface)
{
	iface->methods->unconfigured(iface);
	return USBD_NORMAL_COMPLETION;
}

static usbd_status
reassemble_interfaces(struct usbp_device *device)
{
	struct iface_assembly *ifa = alloca(sizeof (struct iface_assembly) *
	    device->n_interfaces);
	int n_new;
	enum USBP_DEVICE_STATE old_state = device->dev_state;
	struct usbp_bus *bus = (struct usbp_bus *)device->usbd.bus;

	n_new = select_interfaces(device, ifa);

	DPRINTF(USBP_DEBUG_TRACE,
	    ("%s: number of active interface state=%d %d->%d\n", __func__,
		device->dev_state,
		device->n_picked_interfaces, n_new));
	do {
		int i, j;
		for (i=0; i < n_new; ++i) {
			printf("if=%p, %d endpoints: ", ifa[i].iface, ifa[i].n_endpoints);
			for(j=0; j < ifa[i].n_endpoints; ++j) {
				printf("0x%x ", ifa[i].epspec[j].address);
			}
			printf("\n");
		}
	} while (0);
	
	if (!compare_interfaces(ifa, n_new, device->picked_interfaces,
		device->n_picked_interfaces)) {
		/* set of active interfaces has changed. */
		if (device->dev_state >= USBP_DEV_READY) {
			if (bus->usbp_methods->pullup_control)
				bus->usbp_methods->pullup_control(bus, false);
			
			bus->usbp_methods->enable(bus, false);
		}

		if (device->dev_state != USBP_DEV_INIT &&
		    device->n_picked_interfaces > 0) {
			int i;
			DPRINTF(USBP_DEBUG_TRACE,
			    ("%s: unconfigure\n", __func__));
			// unconfigure
			for (i=0; i < device->n_picked_interfaces; ++i) {
				deactivate_interface(device->picked_interfaces[i]);
			}

			kmem_free(device->picked_interfaces,
			    device->n_picked_interfaces	* sizeof device->picked_interfaces[0]);
			device->picked_interfaces = NULL;
			device->n_picked_interfaces = 0;
			DPRINTF(USBP_DEBUG_TRACE,
			    ("%s: unconfigure END\n", __func__));
		}

		if (n_new <= 0) {
			device->dev_state = USBP_DEV_NO_INTERFACE;
		}
		else {
			int i;

			build_device_descriptor(device, ifa, n_new);
			build_config_descriptor(device, ifa, n_new);
			// configure

			device->picked_interfaces =
			    kmem_alloc(n_new * sizeof device->picked_interfaces[0],
				KM_NOSLEEP);

			if (device->picked_interfaces == NULL)
				return USBD_NOMEM;

			for (i=0; i < n_new; ++i) {
				struct usbp_interface *iface = ifa[i].iface;
				activate_interface(iface);
				device->picked_interfaces[i] = iface;
			}

			device->dev_state = USBP_DEV_ASSEMBLED;
		}
		device->n_picked_interfaces = n_new;

	}

	if (device->dev_state == USBP_DEV_ASSEMBLED &&
	    (old_state >= USBP_DEV_READY || device->connect_auto)) {
		if (bus->usbp_methods->enable)
			bus->usbp_methods->enable(bus, true);
		if (bus->usbp_methods->pullup_control)
			bus->usbp_methods->pullup_control(bus, true);
		device->dev_state = USBP_DEV_READY;

		if (is_vbus_on(bus)) 
			device->dev_state = USBP_DEV_CONNECTED;
	}

	return USBD_NORMAL_COMPLETION;
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

	DPRINTF(USBP_DEBUG_TRACE, ("%s\n", __func__));

	/* Change device state from any state backe to Default. */
	(void)usbp_set_config(dev, USB_UNCONFIG_NO);
	dev->usbd.address = 0;

	bus->ep0state = EP0_IDLE;

	DPRINTF(USBP_DEBUG_TRACE, ("%s %d\n", __func__, __LINE__));

#if 0
	SIMPLEQ_INIT(&dev->default_pipe);
#else
	while (!SIMPLEQ_EMPTY(&dev->usbd.default_pipe->queue)) {
		DPRINTF(USBP_DEBUG_TRACE, ("%s: dequeue xfer %p", __func__, SIMPLEQ_FIRST(&dev->usbd.default_pipe->queue)));
		SIMPLEQ_REMOVE_HEAD(&dev->usbd.default_pipe->queue, next);
	}
#endif		
	usbd_setup_default_xfer(dev->default_xfer, &dev->usbd,
	    NULL,
	    0, /* timeout */
	    NULL, /* req */
	    &dev->def_req,	/* buffer */
	    sizeof (dev->def_req),	/* length */
	    0,	/* flags */
	    usbp_do_request);
	usbp_transfer(dev->default_xfer);
}


/*
 *  USB device
 */
static usbd_status
usbp_new_device(device_t parent, struct usbp_bus *bus, int speed)
{
	struct usbp_device *dev = NULL;
	usbd_status err;
	struct usbd_pipe *default_pipe = NULL;
	struct usbp_softc *sc = device_private(parent);
	int i;

#if 0
#ifdef DIAGNOSTIC
	KASSERT(up->device == NULL);
#endif
#endif

	DPRINTF(USBP_DEBUG_TRACE,
	    ("%s: %s: rev=%x\n", __func__, device_xname(parent), bus->usbd.usbrev));

	dev = malloc(sizeof(*dev), M_USB, M_NOWAIT | M_ZERO);
	if (dev == NULL)
		return USBD_NOMEM;

	dev->usbd.bus = &bus->usbd;                   // XXX
	dev->usbp = parent;
//	SIMPLEQ_INIT(&dev->configs);
	dev->current_config = USB_UNCONFIG_NO;
	SIMPLEQ_INIT(&dev->interface_list);
#if 0
	/* these are zero-ed by malloc */
	dev->n_interfaces = 0;
	dev->picked_interfaces = NULL;
	dev->n_picked_interfaces = 0;
	dev->cdesc = NULL;
	dev->cdesc_size = 0;
#endif	

	dev->dev_state = USBP_DEV_INIT;
	dev->connect_auto = true;          // XXX

	/* initialize string pool */
	/* __BITMAP_ZERO(...) */
	for (i=0; i < USBP_EMPTY_STRING_ID; ++i)
		__BITMAP_SET(i, &dev->string_id_map);
	dev->empty_string = usbp_intern_string(dev, "");
	DPRINTF(USBP_DEBUG_STRING,
	    ("%s: dev=%p empty_string=%p\n", __func__, dev, dev->empty_string));

	/* Initialize device status. */
	USETW(dev->status.wStatus, UDS_SELF_POWERED);


	/* Set up the default endpoint handle and descriptor. */
	dev->usbd.def_ep.edesc = &dev->usbd.def_ep_desc;
	dev->usbd.def_ep_desc.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE;
	dev->usbd.def_ep_desc.bDescriptorType = UDESC_ENDPOINT;
	dev->usbd.def_ep_desc.bEndpointAddress = USB_CONTROL_ENDPOINT;
	dev->usbd.def_ep_desc.bmAttributes = UE_CONTROL;
	USETW(dev->usbd.def_ep_desc.wMaxPacketSize, bus->ep0_maxp);
	dev->usbd.def_ep_desc.bInterval = 0;

	/* Establish the default pipe. */
	err = usbd_setup_pipe_flags(&dev->usbd, NULL, &dev->usbd.def_ep, 0, &default_pipe,
	    USBD_PERIPHERAL);
	if (err)
		goto bad;

	dev->usbd.default_pipe = default_pipe;

	/* Preallocate xfers for default pipe. */
	dev->default_xfer = usbp_alloc_xfer(dev);
	dev->data_xfer = usbp_alloc_xfer(dev);
	if (dev->default_xfer == NULL || dev->data_xfer == NULL)
		goto bad;

	/* Insert device request xfer. */
	usbd_setup_default_xfer(dev->default_xfer, &dev->usbd, NULL,
	    0, NULL, &dev->def_req, sizeof (dev->def_req), 0, usbp_do_request);
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
	sc->sc_usbdev = dev;
	

	/* Attach interface drivers. */
	err = usbp_search_interfaces(parent, dev /* , port*/);
	if (err) {
		/* doesn't happen for now */
		DPRINTF(USBP_DEBUG_TRACE, ("%s: %s: usbp_search_interfaces failed. err=%d\n",
			     device_xname(parent),
			     __func__, err));
		usbp_remove_device(dev/*, up*/);

	}
	return err;

bad:
	if (dev != NULL) {
		if (dev->default_xfer)
			usbd_free_xfer(dev->default_xfer);
		if (dev->data_xfer)
			usbd_free_xfer(dev->data_xfer);
		if (default_pipe)
			usbp_close_pipe(default_pipe);
		free(dev, M_USB);
	}
	return err;
			
}

/*
 * index: the index number in endpoint request
 */
usbd_status
usbp_open_pipe(struct usbp_interface *iface, int index, int flags, struct usbd_pipe **pipe)
{
	struct usbd_endpoint *ep = usbp_iface_endpoint(iface, index);

	if (ep == NULL)
		return USBD_INVAL;

	return usbd_open_pipe(&iface->usbd, ep->edesc->bEndpointAddress,
	    flags | USBD_PERIPHERAL, pipe);
}


struct usbd_endpoint *
usbp_iface_endpoint(struct usbp_interface *iface, int index)
{
	if (index < 0 || iface->ispec->num_endpoints <= index) {
		aprint_error("%s: endpoint index out of range (%d)\n",
		    __func__, index);
		return NULL;
	}

	return &iface->usbd.endpoints[index];
}

/*
 * Change device state from Default to Address, or change the device address
 * if the device is not currently in the Default state.
 */
static void
usbp_set_address(struct usbp_device *dev, u_int8_t address)
{
	DPRINTF(USBP_DEBUG_TRACE,
	    ("%s: dev=%p, %u -> %u\n", __func__, dev,
	    dev->usbd.address, address));
	dev->usbd.address = address;
}

#if 1
/*
 * Currently, only one configuration is allowed.
 */
static usbd_status
usbp_set_config(struct usbp_device *dev, u_int8_t new)
{
	if (new != USB_UNCONFIG_NO && new != 1) {
		return USBD_INVAL;
	}

	if (new == USB_UNCONFIG_NO) {
		/* move to unconfigured state. */
		// XXX
	}

	return USBD_NORMAL_COMPLETION;
}
#else
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

	DPRINTF(USBP_DEBUG_TRACE, ("%s: dev=%p, %u -> %u\n", __func__, dev, old, new));

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
			DPRINTF(USBP_DEBUG_TRACE,
			    ("usbp_set_config: %s\n", usbd_errstr(err)));
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
#endif


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
	const usb_string_descriptor_t *sd;
	struct usbp_softc *sc;

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
		sd = get_string_descriptor(dev, index);
		if (sd == NULL)
			return USBD_INVAL;
		*data = __UNCONST(sd);
		USETW(req->wLength, MIN(UGETW(req->wLength), sd->bLength));
		return USBD_NORMAL_COMPLETION;
	default:
		DPRINTF(USBP_DEBUG_TRACE,
		    ("usbf_get_descriptor: unknown descriptor type=%u\n",
		    type));
		return USBD_INVAL;
	}
}

#if 0
static struct usbd_endpoint *
usbp_iface_endpoint(struct usbp_interface *iface, u_int8_t address)
{
	struct usbd_endpoint *ep;
	int i;

	for (i = 0; i < iface->usbd.idesc->bNumEndpoints; i++) {
		ep = &iface->usbd.endpoints[i];

		if (ep->edesc == NULL)
			continue;	// XXX error
		if (ep->edesc->bEndpointAddress == address)
			return ep;
	}
	return NULL;
}
#endif


static struct usbd_endpoint *
usbp_device_endpoint(struct usbp_device *dev, u_int8_t address)
{
#if 0
	struct usbp_interface *iface;
	struct usbd_endpoint *ep;

	SIMPLEQ_FOREACH(iface, &cfg->iface_head, next) {
		ep = usbp_iface_endpoint(iface, address);
		if (ep)
			return ep;
	}
#endif
	return NULL;
}

static void
usbp_set_endpoint_halt(struct usbd_endpoint *endpoint)
{
}

static void
usbp_clear_endpoint_halt(struct usbd_endpoint *endpoint)
{
}

static void
usbp_stall_pipe(struct usbd_pipe *pipe)
{
	DPRINTF(USBP_DEBUG_TRACE, ("%s\n", __func__));
}



static usbd_status
usbp_set_endpoint_feature(struct usbp_device *dev, u_int8_t address,
    u_int16_t value)
{
	struct usbd_endpoint *ep;

	DPRINTF(USBP_DEBUG_TRACE,
	    ("usbf_set_endpoint_feature: dev=%p address=%#x"
	    " value=%#x\n", dev, address, value));

	ep = usbp_device_endpoint(dev, address);
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
usbp_clear_endpoint_feature(struct usbp_device *dev, u_int8_t address,
    u_int16_t value)
{
	struct usbd_endpoint *ep;

	DPRINTF(USBP_DEBUG_TRACE,
	    ("usbf_clear_endpoint_feature: device=%p address=%#x"
	    " value=%#x\n", dev, address, value));

	ep = usbp_device_endpoint(dev, address);
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
	//struct usbp_config *cfg;
	void *data = NULL;
	u_int16_t value;
	u_int16_t index;
	struct usbp_bus *bus = (struct usbp_bus *)(dev->usbd.bus);

	DPRINTF(USBP_DEBUG_TRACE,
	    ("%s: xfer=%p ep0state=%d  bRequest=0x%x bmRequestType=0x%x\n",
		__func__, xfer, bus->ep0state, req->bRequest, req->bmRequestType));

	/* XXX */
	if (bus->ep0state == EP0_END_XFER &&
	    err == USBD_SHORT_XFER) {

		bus->ep0state = EP0_IDLE;
		goto next;
	}

	if (err) {
		DPRINTF(USBP_DEBUG_TRACE,
		    ("usbp_do_request: receive failed, %s\n",
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
		data = &dev->current_config;
		USETW(req->wLength, MIN(UGETW(req->wLength), 1));
		break;

	case C(UR_GET_DESCRIPTOR, UT_READ_DEVICE):
		err = usbp_get_descriptor(dev, req, &data);
		break;

	case C(UR_GET_STATUS, UT_READ_DEVICE):
		DPRINTF(USBP_DEBUG_TRACE,("%s: UR_GET_STATUS %d\n", __func__,
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
		/* check if device is configured */
		err = usbp_set_endpoint_feature(dev, index, value);
		break;

	case C(UR_CLEAR_FEATURE, UT_WRITE_ENDPOINT):
		value = UGETW(req->wValue);
		index = UGETW(req->wIndex);
		err = usbp_clear_endpoint_feature(dev, index, value);
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

	default:
		err = pass_request_to_iface(dev, req, &data);
	}

	DPRINTF(USBP_DEBUG_TRACE,
	    ("usbp_do_request: %d err=%d wLength=%d\n",
		__LINE__, err, UGETW(req->wLength)));

	if (err) {
		DPRINTF(USBP_DEBUG_TRACE,
		    ("usbp_do_request: request=%#x, type=%#x "
			"failed, %s\n", req->bRequest, req->bmRequestType,
			usbd_errstr(err)));
		usbp_stall_pipe(dev->usbd.default_pipe);
	} else if (UGETW(req->wLength) > 0) {
		if (data == NULL) {
			DPRINTF(USBP_DEBUG_TRACE, ("usbp_do_request: no data, "
				"sending ZLP\n"));
			USETW(req->wLength, 0);
		}
		printf("%s: reply IN data length=%d\n", __func__, UGETW(req->wLength));
		/* Transfer IN data in response to the request. */
		usbd_setup_xfer(dev->data_xfer, dev->usbd.default_pipe,
		    NULL, data, UGETW(req->wLength), 0, 0, NULL);
		err = usbp_transfer(dev->data_xfer);
		if (err && err != USBD_IN_PROGRESS) {
			DPRINTF(USBP_DEBUG_TRACE,("usbp_do_request: data xfer=%p, %s\n",
			    xfer, usbd_errstr(err)));
		}
	}

next:
	/* Schedule another request transfer. */
	usbd_setup_default_xfer(dev->default_xfer, &dev->usbd, NULL,
	    0, NULL, &dev->def_req, sizeof (dev->def_req), 0, usbp_do_request);
	err = usbp_transfer(dev->default_xfer);
	if (err && err != USBD_IN_PROGRESS) {
		DPRINTF(USBP_DEBUG_TRACE,("usbp_do_request: ctrl xfer=%p, %s\n", xfer,
		    usbd_errstr(err)));
	}

	DPRINTF(USBP_DEBUG_TRACE,("usbp_do_request: done\n"));
}


static usbd_status
pass_request_to_iface(struct usbp_device *dev, usb_device_request_t *req, void **data)
{
	int i;
	usbd_status err = USBD_STALLED;
	struct usbp_interface *iface;

	DPRINTF(USBP_DEBUG_TRACE, ("%s: picked_interfaces=%d\n", __func__, dev->n_picked_interfaces));
	for (i=0; i < dev->n_picked_interfaces; ++i) {
		iface = dev->picked_interfaces[i];

		DPRINTF(USBP_DEBUG_TRACE, ("%s: iface=%p handler=%p\n", __func__,
			iface, iface->methods->handle_device_request));

		if (iface->methods->handle_device_request == NULL)
			continue;
		err = iface->methods->handle_device_request(iface, req, data);

		if (err != USBD_NOT_FOR_US)
			return err;
	}

	/* no interface can handle the request */
	return USBD_STALLED;
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
 * Attach USB interface drivers.
 */
static usbd_status
usbp_search_interfaces(struct device *parent, struct usbp_device *dev/*, int port*/)
{
	struct usbp_interface_attach_args uaa;
	cfdata_t cfdata;

	bzero(&uaa, sizeof uaa);
	uaa.device = dev;

	/*
	 * find all USB interface drivers attached to this device.
	 */
	while ((cfdata = config_search_ia(NULL, parent, "usbp", &uaa)) != NULL) {
		printf("%s: cfdata=%p\n", __func__, cfdata);
		config_attach(parent, cfdata, &uaa, NULL);
	}

	return USBD_NORMAL_COMPLETION;
}

static void
usbp_remove_device(struct usbp_device *dev/*, struct usbp_port *up*/)
{
	KASSERT(dev != NULL /*&& dev == up->device*/);

//	if (dev->function != NULL)
//		config_detach((struct device *)dev->function, DETACH_FORCE);
	if (dev->usbd.default_pipe != NULL)
		usbp_close_pipe(dev->usbd.default_pipe);
#if 0
	up->device = NULL;
#endif
	free(dev, M_USB);
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
		DPRINTF(USBP_DEBUG_TRACE,("usbf_abort_pipe: pipe=%p, xfer=%p\n", pipe,
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
#if 1

	/*
	 * Currently, we don't support multiple configurations
	 */
	return dev->cdesc;
#else
	struct usbp_config *uc;
	SIMPLEQ_FOREACH(uc, &dev->configs, next) {
		if (index-- == 0)
			return uc->uc_cdesc;
	}
	return NULL;
#endif	
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

#if 1
/* Called at splusb() */
void
usbp_transfer_complete(struct usbd_xfer *xfer)

{
	struct usbd_pipe *pipe = xfer->pipe;
	int repeat = pipe->repeat;

	//SPLUSBCHECK;
	DPRINTF(USBP_DEBUG_TRACE,("usbp_transfer_complete: xfer=%s pipe=%p running=%d callback=%p\n",
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
		DPRINTF(USBP_DEBUG_TRACE,("usbf_transfer_complete: short xfer=%p %u<%u\n",
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

#endif

usbd_status
usbp_insert_transfer(struct usbd_xfer *xfer)
{
	struct usbd_pipe *pipe = xfer->pipe;
	usbd_status err;
	int s;


	DPRINTF(USBP_DEBUG_TRACE,("%s: xfer=%s pipe=%p running=%d callback=%p\n", __func__,
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

	DPRINTF(USBP_DEBUG_TRACE,("%s: pipe=%p running=%d first xfer=%p\n", __func__, pipe, pipe->running,
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


/* New APIs */
usbd_status
usbp_add_interface(struct usbp_device *dev,
    struct usbp_interface *iface,
    const struct usbp_device_info *devinfo,
    const struct usbp_interface_spec *ispec,
    const struct usbp_interface_methods *methods,
    const void *additional_idesc,
    size_t additional_idesc_size)
{
	size_t ispec_size = sizeof *ispec +
	    ispec->num_endpoints * sizeof (struct usbp_endpoint_request);
	struct usbp_interface_spec *ispec_copy;


	DPRINTF(USBP_DEBUG_TRACE, ("%s: dev=%p empty_string=%p\n", __func__, dev, dev->empty_string));

	usbp_init_interface(dev, iface);

	ispec_copy = kmem_alloc(ispec_size, KM_SLEEP);
	if (ispec_copy == NULL)
		return USBD_NOMEM;

	memcpy(ispec_copy, ispec, ispec_size);
	iface->ispec = ispec_copy;
	iface->methods = methods;

	iface->devinfo.class_id_valid = (devinfo->class_id != USBP_ID_UNSPECIFIED);
	if (iface->devinfo.class_id_valid) {
		iface->devinfo.class_id_valid = true;
		iface->devinfo.class_id = devinfo->class_id;
		iface->devinfo.subclass_id = devinfo->subclass_id;
		iface->devinfo.protocol = devinfo->protocol;
	}

	iface->devinfo.vendor_id_valid = (devinfo->vendor_id != USBP_ID_UNSPECIFIED);
	if (iface->devinfo.vendor_id_valid) {
		iface->devinfo.vendor_id = devinfo->vendor_id;
		iface->devinfo.product_id = devinfo->product_id;
		iface->devinfo.bcd_device = devinfo->bcd_device;
	}

	iface->num_strings = 4;
	iface->string = kmem_alloc(sizeof (struct usbp_string *) * iface->num_strings, KM_SLEEP);
	if (iface->string == NULL) {
		kmem_free(__UNCONST(iface->ispec), ispec_size);
		iface->ispec = NULL;
		return USBD_NOMEM;
	}
	

	iface->string[USBP_IF_STR_MANUFACTURER] = 
	    usbp_intern_string(dev, devinfo->manufacturer_name);
	iface->string[USBP_IF_STR_PRODUCT_NAME] = 
	    usbp_intern_string(dev, devinfo->product_name);
	iface->string[USBP_IF_STR_SERIAL] = 
	    usbp_intern_string(dev, devinfo->serial);
	iface->string[USBP_IF_STR_DESCRIPTION] = 
	    usbp_intern_string(dev, ispec->description);

	if (additional_idesc == NULL || additional_idesc_size == 0) {
		iface->additional_idesc = NULL;
		iface->additional_idesc_size = 0;
	}
	else {
		iface->additional_idesc = additional_idesc;
		iface->additional_idesc_size = additional_idesc_size;
	}

	SIMPLEQ_INSERT_HEAD(&dev->interface_list, iface, next);
	dev->n_interfaces++;

	if (!cold) {
		// reconfigure
	}
		
	
	return USBD_NORMAL_COMPLETION;
}

usbd_status
usbp_delete_interface(struct usbp_interface *iface)
{
	struct usbp_device *dev = usbp_interface_to_device(iface);
	usbd_status err = USBD_NORMAL_COMPLETION;

	KASSERT(dev != NULL);
	
	SIMPLEQ_REMOVE(&dev->interface_list, iface, usbp_interface, next);
	dev->n_interfaces--;

	if (dev->dev_state != USBP_DEV_INIT) {
		// reconfigure
		err = reassemble_interfaces(dev);
		if (err != USBD_NORMAL_COMPLETION)
			return err;
	}
		
	if (iface->ispec) {
		size_t ispec_size = sizeof *iface->ispec +
		    iface->ispec->num_endpoints * sizeof (struct usbp_endpoint_request);
		kmem_free(__UNCONST(iface->ispec), ispec_size);
		iface->ispec = NULL;
	}
		

	if (iface->string) {
		int i;

		DPRINTF(USBP_DEBUG_STRING,
		    ("%s: relasing strings for iface\n", __func__));
		for (i=0; i < iface->num_strings; ++i) {
			if (iface->string[i]) {
				usbp_release_string(dev, iface->string[i]);
			}
		}


		DPRINTF(USBP_DEBUG_STRING,
		    ("%s: free strings for iface\n", __func__));
		kmem_free(iface->string, sizeof (struct usbp_string *) * iface->num_strings);
		iface->string = NULL;
		iface->num_strings = 0;
	}

	iface->usbd.device = NULL;
	return err;
}

/*
 * manage strings used in the USB device configuration.
 */
struct usbp_string *
usbp_intern_string(struct usbp_device *usbdev, const char *string)
{
	struct usbp_string *s;
	size_t len;
	int id;
	char *tmp;
	
	DPRINTF(USBP_DEBUG_TRACE|USBP_DEBUG_STRING,
	    ("%s %s\n", __func__, string));
	
	if (string == NULL) {
#if 1
		return NULL;
#else
		s = usbdev->empty_string;
		DPRINTF(USBP_DEBUG_TRACE, ("%s empty=%p\n", __func__, s));
		s->refcount++;
		return s;
#endif
	}

	len = strlen(string);

	SIMPLEQ_FOREACH(s, &usbdev->string_list, next) {
		if (0 == strcmp(string, s->str)) {
			s->refcount++;
			return s;
		}
	}

	/* XXX: need faster way */
	for (id = USBP_STRING_ID_MIN;
	    id <= USBP_STRING_ID_MAX &&
	    __BITMAP_ISSET(id, &usbdev->string_id_map);
		++id) {
	}

	DPRINTF(USBP_DEBUG_STRING, ("%s new id=%d\n", __func__, id));
	
	if (id > USBP_STRING_ID_MAX) {
		aprint_error_dev(usbdev->usbp, "Too many strings");
		s = usbdev->empty_string;
		s->refcount++;
		return s;
	}
	    
	s = kmem_alloc(sizeof *s + len + 1, KM_SLEEP);
	if (s == NULL) {
		aprint_error_dev(usbdev->usbp, "out of memory for a string");
		s = usbdev->empty_string;
		DPRINTF(USBP_DEBUG_STRING, ("%s empty_string=%p\n", __func__, s));
		s->refcount++;
		return s;
	}

	DPRINTF(USBP_DEBUG_STRING, ("%s new entry\n", __func__));

	tmp = (uint8_t *)s + sizeof (*s);
	memcpy(tmp, string, len+1);
	s->str = tmp;
	s->refcount = 1;
	s->id = id;
	__BITMAP_SET(id, &usbdev->string_id_map);
	SIMPLEQ_INSERT_HEAD(&usbdev->string_list, s, next);

	return s;
}

void
usbp_release_string(struct usbp_device *usbdev, struct usbp_string *str)
{
	if (--(str->refcount) <= 0) {
		__BITMAP_CLR(str->id, &usbdev->string_id_map);
		SIMPLEQ_REMOVE(&usbdev->string_list, str, usbp_string, next);
		kmem_free(str, sizeof *str + strlen(str->str) + 1);
	}
}

#if 0
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
#endif

static size_t
make_utf16(usb_string_descriptor_t *dest, const char *s)
{
	int i;

	/* TODO: UTF-8 -> UTF-16 */
	for (i=0; s[i] && i < __arraycount(dest->bString); ++i) {
		USETW(dest->bString[i], s[i]);
	}
	return 2 * i;
}

static const usb_string_descriptor_t *
get_string_descriptor(struct usbp_device *dev, int index)
{
	const static struct {
		uByte bLength;
		uByte bDescriptorType;
		uWord bString[1];
	} sd_langtbl = {
		.bLength = 4,
		.bDescriptorType = UDESC_STRING,
		.bString = {{0x09, 0x04}}, /* en_US */
	}, sd_empty = {
		.bLength = 2,
		.bDescriptorType = UDESC_STRING
	};
	const usb_string_descriptor_t *sdp = NULL;
	struct usbp_string *s;

	/* handle the special string ids */
	switch (index) {
	case USB_LANGUAGE_TABLE:
		sdp = (const usb_string_descriptor_t *)&sd_langtbl;
		break;

	case USBP_EMPTY_STRING_ID:
		sdp = (const usb_string_descriptor_t *)&sd_empty;
		break;
	default:		
		sdp = NULL;
		SIMPLEQ_FOREACH(s, &dev->string_list, next) {
			usb_string_descriptor_t *p;
			if (s->id == index) {
				p = &dev->string_descriptor;
				p->bLength = 2 + make_utf16(p, s->str);
				p->bDescriptorType = UDESC_STRING;

				sdp = p;
				break;
			}
		}
		if (sdp == NULL) {
			aprint_error_dev(dev->usbp,
			    "Unkown string descriptor requested (%d)\n", index);
			return NULL;
		}
	}

	return sdp;
}



void
usbp_init_interface(struct usbp_device *dev, struct usbp_interface *iface)
{
	memset(iface, 0, sizeof *iface);
	iface->usbd.device = &dev->usbd;
}

static void
usbp_child_detached(device_t self, device_t child)
{
	DPRINTF(USBP_DEBUG_TRACE, ("%s\n", __func__));

	/* children call usbp_delete_interface by themselves.
	   nothing to do here.
	   this function is necessary for "drvctl -d" to work  */
}
