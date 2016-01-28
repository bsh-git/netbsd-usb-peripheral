/*	$NetBSD$ */

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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/conf.h>
#include <sys/tty.h>

#include <dev/usb/usb.h>

#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdevs.h>
#include <dev/usb/usbf.h>

#include <dev/usb/ucomvar.h>

#include <dev/usb/uftdireg.h>

#define UPFTDI_DEBUG

#ifdef UPFTDI_DEBUG
#define DPRINTF(x)	if (upftdidebug) printf x
#define DPRINTFN(n,x)	if (upftdidebug>(n)) printf x
int upftdidebug = 0;
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif


/*
 * These are the default number of bytes transferred per frame if the
 * endpoint doesn't tell us.  The output buffer size is a hard limit
 * for devices that use a 6-bit size encoding.
 */
#define UFTDIIBUFSIZE 64
#define UFTDIOBUFSIZE 64


#define DEVNAME(sc)	device_xname((sc)->sc_fun.dev)

#define	MAXCOMDEVICES  4

struct upftdi_softc {
	struct usbf_function	sc_fun;
	struct usbf_config	*sc_config;
	int			sc_rxeof_errors;

	int sc_ncomdevs;
	struct upftdi_comdev {
		struct usbf_interface	*iface;
		struct usbf_endpoint	*ep_in;
		struct usbf_endpoint	*ep_out;
		struct usbf_pipe	*pipe_in;
		struct usbf_pipe	*pipe_out;
		struct usbf_xfer	*xfer_in;
		struct usbf_xfer	*xfer_out;
		void			*buffer_in;
		void			*buffer_out;

		device_t ucomdev;

	} sc_comdev[MAXCOMDEVICES];
};

static int	upftdi_match(device_t, cfdata_t, void *);
static void	upftdi_attach(device_t, device_t, void *);

static usbf_status	upftdi_do_request(struct usbf_function *,
				 usb_device_request_t *, void **);

void		upftdi_start(struct ifnet *);

void		upftdi_txeof(struct usbf_xfer *, void *,
			    usbf_status);
static void	upftdi_rxeof(struct usbf_xfer *, void *,
			    usbf_status);
int		upftdi_ioctl(struct ifnet *ifp, u_long command, void *data);
void		upftdi_watchdog(struct ifnet *ifp);
void		upftdi_stop(struct upftdi_softc *);
void		upftdi_start_timeout (void *);

static usbf_status upftdi_set_config(struct usbf_function *, struct usbf_config *);
static void upftdi_attach_ucom(device_t, struct usbf_device *, int);


CFATTACH_DECL_NEW(upftdi, sizeof (struct upftdi_softc),
    upftdi_match, upftdi_attach, NULL, NULL);

struct usbf_function_methods upftdi_methods = {
	upftdi_set_config,		/* set_config */
	upftdi_do_request
};

static struct ucom_methods upftdi_ucom_methods = {
};

/*
 * USB function match/attach/detach
 */
static int
upftdi_match(device_t parent, cfdata_t cf, void *aux)
{
	return UMATCH_GENERIC;
}

#define	UPFTDI_VENDOR_STRING	"fake"
#define	UPFTDI_PRODUCT_STRING	"emulate FTDI 230x"
#define	UPFTDI_SERIAL_STRING	"1.00"

#define	UPFTDI_DEVICE_CODE	0x0000	/* BCD device release number */
#define UPFTDI_BUFSZ	        256


static void
upftdi_attach(device_t parent, device_t self, void *aux)
{
	struct upftdi_softc *sc = device_private(self);
	struct usbf_attach_arg *uaa = aux;
	struct usbf_device *dev = uaa->device;
	usbf_status err;
	int n_comdevs = 1;
	int idx;
	//int s;
	extern void usbf_debug(struct usbf_device *);


	DPRINTF(("%s  dev=%p sc=%p upftdi_methods=%p\n", __func__, dev,
		sc,
		&upftdi_methods));
	usbf_debug(dev);
	
	/* Set the device identification according to the function. */
	usbf_devinfo_setup(dev, UDCLASS_IN_INTERFACE, 0, 0, USB_VENDOR_FTDI,
	    USB_PRODUCT_FTDI_SERIAL_230X, UPFTDI_DEVICE_CODE, UPFTDI_VENDOR_STRING,
	    UPFTDI_PRODUCT_STRING, UPFTDI_SERIAL_STRING);

	/* Fill in the fields needed by the parent device. */
	sc->sc_fun.dev = self;
	sc->sc_fun.methods = &upftdi_methods;

	/* timeout to start delayed transfers */
//XXX	timeout_set(&sc->start_to, upftdi_start_timeout, sc);

	/*
	 * Build descriptors according to the device class specification.
	 */
	err = usbf_add_config(dev, &sc->sc_config);
	if (err) {
		printf(": usbf_add_config failed\n");
		return;
	}

	for (idx=0; idx < n_comdevs; ++idx) {
		struct usbf_interface *iface;
		struct usbf_endpoint *ep_in, *ep_out;

		err = usbf_add_interface(sc->sc_config, UICLASS_VENDOR,
		    UICLASS_VENDOR, 0, NULL, &iface);
		if (err) {
			printf(": usbf_add_interface failed\n");
			break;
		}
		/* XXX don't use hard-coded values 128 and 16. */
		err = usbf_add_endpoint(iface, UE_DIR_IN | 1, UE_BULK,         //XXX
		    64, 16, &ep_in) ||
		    usbf_add_endpoint(iface, UE_DIR_OUT | 2, UE_BULK,          //XXX
			64, 16, &ep_out);
		if (err) {
			printf(": usbf_add_endpoint failed\n");
			break;
		}


		sc->sc_comdev[idx].iface = iface;
		sc->sc_comdev[idx].ep_in = ep_in;
		sc->sc_comdev[idx].ep_out = ep_out;

	}
	n_comdevs = idx;

#if 0
	usb_cdc_union_descriptor_t udesc;
	/* Append a CDC union descriptor. */
	bzero(&udesc, sizeof udesc);
	udesc.bLength = sizeof udesc;
	udesc.bDescriptorType = UDESC_CS_INTERFACE;
	udesc.bDescriptorSubtype = UDESCSUB_CDC_UNION;
	udesc.bSlaveInterface[0] = usbf_interface_number(sc->sc_iface);
	err = usbf_add_config_desc(sc->sc_config,
	    (usb_descriptor_t *)&udesc, NULL);
	if (err) {
		printf(": usbf_add_config_desc failed\n");
		return;
	}
#endif

	/*
	 * Close the configuration and build permanent descriptors.
	 */
	err = usbf_end_config(sc->sc_config);
	if (err) {
		printf(": usbf_end_config failed\n");
		return;
	}

	sc->sc_ncomdevs = 0;

	for (idx=0; idx < n_comdevs; ++idx) {
		struct upftdi_comdev *com;

		com = &sc->sc_comdev[idx];
		/* Preallocate xfers and data buffers. */
		com->xfer_in = usbf_alloc_xfer(dev);
		com->xfer_out = usbf_alloc_xfer(dev);


		com->buffer_in = usbf_alloc_buffer(com->xfer_in, UPFTDI_BUFSZ);
		com->buffer_out = usbf_alloc_buffer(com->xfer_out, UPFTDI_BUFSZ);
		if (com->buffer_in == NULL || com->buffer_out == NULL) {
			printf(": usbf_alloc_buffer failed\n");
			break;
		}

		/* Open the bulk pipes. */
		err = usbf_open_pipe(com->iface,
		    usbf_endpoint_address(com->ep_out), &com->pipe_out) ||
		    usbf_open_pipe(com->iface,
			usbf_endpoint_address(com->ep_in), &com->pipe_in);
		if (err) {
			printf(": usbf_open_pipe failed\n");
			return;
		}

		upftdi_attach_ucom(self, dev, idx);

		/* Get ready to receive packets. */
		usbf_setup_xfer(com->xfer_out, com->pipe_out, sc,
		    com->buffer_out, UPFTDI_BUFSZ, USBD_SHORT_XFER_OK, 0, upftdi_rxeof);
		err = usbf_transfer(com->xfer_out);
		if (err && err != USBF_IN_PROGRESS) {
			printf(": usbf_transfer failed\n");

			config_detach(com->ucomdev, DETACH_FORCE);
			/* XXX: release other resources */
			break;
		}
		
		++sc->sc_ncomdevs;
	}

#if 0
	s = splnet();


	splx(s);
#endif
}


static void
upftdi_attach_ucom(device_t self, struct usbf_device *dev, int idx)
{
	struct ucom_attach_args uca;
	struct upftdi_softc *sc = device_private(self);

	uca.portno = FTDI_PIT_SIOA + idx;
	/* bulkin, bulkout set above */
	uca.ibufsize = UFTDIIBUFSIZE;
	uca.ibufsizepad = uca.ibufsize;
	uca.obufsize = UFTDIOBUFSIZE;
	uca.opkthdrlen = 0; // sc->sc_hdrlen;
	uca.device = NULL;
	uca.iface =  NULL;
	uca.pdevice = dev;
	uca.piface = NULL; 		/* XXX */
	uca.methods = &upftdi_ucom_methods;
	uca.arg = self;
	uca.info = NULL;

	sc->sc_comdev[idx].ucomdev = config_found_sm_loc(self, "ucombus", NULL,
	    &uca, ucomprint, ucomsubmatch);

}

static usbf_status
upftdi_req_reset(struct usbf_function *fun, usb_device_request_t *req)
{
	int portno = UGETW(req->wIndex);

	printf("%s: portno=%d\n", __func__, portno);
	
	return USBF_NORMAL_COMPLETION;;
}

static usbf_status
upftdi_req_flow(struct usbf_function *fun, usb_device_request_t *req)
{
	printf("%s: wIndex=%x\n", __func__,
	    UGETW(req->wIndex));
	return USBF_NORMAL_COMPLETION;
}

static usbf_status
upftdi_req_bitmode(struct usbf_function *fun, usb_device_request_t *req)
{
	printf("%s: portno=%d mode=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBF_NORMAL_COMPLETION;
}

static usbf_status
upftdi_req_baudrate(struct usbf_function *fun, usb_device_request_t *req)
{
	printf("%s: portno=%d rate=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBF_NORMAL_COMPLETION;
}

static usbf_status
upftdi_req_setdata(struct usbf_function *fun, usb_device_request_t *req)
{
	printf("%s: portno=%d data=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBF_NORMAL_COMPLETION;
}

static usbf_status
upftdi_req_modemctrl(struct usbf_function *fun, usb_device_request_t *req)
{
	printf("%s: portno=%d ctrl=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBF_NORMAL_COMPLETION;
}

static usbf_status
upftdi_do_vendor_write(struct usbf_function *fun, usb_device_request_t *req, void **data)
{
	usbf_status err = USBF_STALLED;
	
	switch(req->bRequest) {
	case FTDI_SIO_RESET:
		 err = upftdi_req_reset(fun, req);
		 break;
	case FTDI_SIO_SET_FLOW_CTRL:
		err = upftdi_req_flow(fun, req);
	case FTDI_SIO_SET_BITMODE:
		err = upftdi_req_bitmode(fun, req);
		break;
	case FTDI_SIO_SET_BAUD_RATE:
		err = upftdi_req_baudrate(fun, req);
		break;
	case FTDI_SIO_SET_DATA:
		err = upftdi_req_setdata(fun, req);
		break;
	case FTDI_SIO_MODEM_CTRL:
		err = upftdi_req_modemctrl(fun, req);
		break;
	default:
		printf("%s: unknown request 0x%x\n", __func__, req->bRequest);
		err = USBF_STALLED;
	}

	if (err == USBF_NORMAL_COMPLETION) {
		static uint16_t v = 0;
		USETW(req->wLength, 2);
		*data = (void *)&v;
		return err;
	}

	return USBF_STALLED;
}

/*
 * Handle non-standard request received on the control pipe.
 */
static usbf_status
upftdi_do_request(struct usbf_function *fun, usb_device_request_t *req, void **data)
{
#define C(x,y) ((x) | ((y) << 8))
	if (req->bmRequestType == UT_WRITE_VENDOR_DEVICE)
		return upftdi_do_vendor_write(fun, req, data);

	printf("requestType=%x request=%x\n",
	    req->bmRequestType,
	    req->bRequest);
	return USBF_STALLED;

#undef C
}


static void
upftdi_rxeof(struct usbf_xfer *xfer, void *priv,
    usbf_status status)
{
#if 0
	struct upftdi_softc	*sc = priv;
	int total_len = 0;
//	int s;

	DPRINTF(("upftdi_rxeof: xfer=%p, priv=%p, %s\n", xfer, priv,
		 usbf_errstr(status)));

	if (status != USBF_NORMAL_COMPLETION) {
		if (status == USBF_NOT_STARTED || status == USBF_CANCELLED)	
			return;
		if (sc->sc_rxeof_errors == 0)
			printf("%s: usb error on rx: %s\n",
			    DEVNAME(sc), usbf_errstr(status));
		/* XXX - no stalls on client */
		if (sc->sc_rxeof_errors++ > 10) {
			printf("%s: too many errors, disabling\n",
			    DEVNAME(sc));
		}
		goto done;
	}
	sc->sc_rxeof_errors = 0;

#if 0
	/* upon first incoming packet we know the host is listening */
	if (sc->sc_listening == 0) {
		sc->sc_listening = 1;
	}
#endif


	usbf_get_xfer_status(xfer, NULL, NULL, &total_len, NULL);

	if (total_len <= 1)
		goto done;

done:
	/* Setup another xfer. */
	usbf_setup_xfer(xfer, sc->sc_pipe_out, sc, sc->sc_buffer_out,
	    UPFTDI_BUFSZ, USBD_SHORT_XFER_OK, 0, upftdi_rxeof);

	status = usbf_transfer(xfer);
	if (status && status != USBF_IN_PROGRESS) {
		printf("%s: usbf_transfer failed\n", DEVNAME(sc));
		return;
	}
#endif
}


static usbf_status
upftdi_set_config(struct usbf_function *fun, struct usbf_config *config)
{
	DPRINTF(("%s: fun=%p\n", __func__, fun));
	return USBF_NORMAL_COMPLETION;
}
