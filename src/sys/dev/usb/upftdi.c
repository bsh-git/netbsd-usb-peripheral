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
#include <dev/usb/usbdivar.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdevs.h>
#include <dev/usb/usbp.h>

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
	struct usbp_function	sc_fun;
	struct usbp_config	*sc_config;
	int			sc_rxeof_errors;

	bool sc_dying;

	int sc_ncomdevs;
	struct upftdi_comdev {
		struct usbp_interface	*iface;
		struct usbd_endpoint	*ep_in;
		struct usbd_endpoint	*ep_out;
		struct usbd_pipe	*pipe_in;
		struct usbd_pipe	*pipe_out;
		struct usbd_xfer	*xfer_in;
		struct usbd_xfer	*xfer_out;
		void			*buffer_in;
		void			*buffer_out;

		device_t ucomdev;

		struct upftdi_softc *softc;

	} sc_comdev[MAXCOMDEVICES];
};

static int	upftdi_match(device_t, cfdata_t, void *);
static void	upftdi_attach(device_t, device_t, void *);

static usbd_status	upftdi_do_request(struct usbp_function *,
				 usb_device_request_t *, void **);

void		upftdi_start(struct ifnet *);

void		upftdi_txeof(struct usbd_xfer *, void *,
			    usbd_status);
#if 0
static void	upftdi_rxeof(struct usbd_xfer *, void *,
			    usbd_status);
#endif
int		upftdi_ioctl(struct ifnet *ifp, u_long command, void *data);
void		upftdi_watchdog(struct ifnet *ifp);
void		upftdi_stop(struct upftdi_softc *);
void		upftdi_start_timeout (void *);

static usbd_status upftdi_set_config(struct usbp_function *, struct usbp_config *);
static void upftdi_attach_ucom(device_t, struct usbp_device *, int);


CFATTACH_DECL_NEW(upftdi, sizeof (struct upftdi_softc),
    upftdi_match, upftdi_attach, NULL, NULL);

struct usbp_function_methods upftdi_methods = {
	upftdi_set_config,		/* set_config */
	upftdi_do_request
};

//static int upftdi_open(void *vsc, int portno);
static void upftdi_read(void *vsc, int portno, u_char **ptr, u_int32_t *count);
static void upftdi_write(void *vsc, int portno, u_char *to, u_char *from, u_int32_t *count);

static struct ucom_methods upftdi_ucom_methods = {
//	.ucom_open = upftdi_open,
	.ucom_read = upftdi_read,
	.ucom_write = upftdi_write,
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
	struct usbp_function_attach_args *uaa = aux;
	struct usbp_device *dev = uaa->device;
	usbd_status err;
	int n_comdevs = 1;
	int idx;
	//int s;


	DPRINTF(("%s  dev=%p sc=%p upftdi_methods=%p\n", __func__, dev,
		sc,
		&upftdi_methods));
	
	/* Set the device identification according to the function. */
	usbp_devinfo_setup(dev, UDCLASS_IN_INTERFACE, 0, 0, USB_VENDOR_FTDI,
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
	err = usbp_add_config(dev, &sc->sc_config);
	if (err) {
		printf(": usbp_add_config failed\n");
		return;
	}

	for (idx=0; idx < n_comdevs; ++idx) {
		struct usbp_interface *iface;
		struct usbd_endpoint *ep_in, *ep_out;

		err = usbp_add_interface(sc->sc_config, UICLASS_VENDOR,
		    UICLASS_VENDOR, 0, NULL, &iface);
		if (err) {
			printf(": usbp_add_interface failed\n");
			break;
		}
		/* XXX don't use hard-coded values 128 and 16. */
		err = usbp_add_endpoint(iface, UE_DIR_IN | 1, UE_BULK,         //XXX
		    64, 16, &ep_in) ||
		    usbp_add_endpoint(iface, UE_DIR_OUT | 2, UE_BULK,          //XXX
			64, 16, &ep_out);
		if (err) {
			printf(": usbf_add_endpoint failed\n");
			break;
		}


		sc->sc_comdev[idx].iface = iface;
		sc->sc_comdev[idx].ep_in = ep_in;
		sc->sc_comdev[idx].ep_out = ep_out;
		sc->sc_comdev[idx].softc = sc;

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
	err = usbp_end_config(sc->sc_config);
	if (err) {
		printf(": usbf_end_config failed\n");
		return;
	}

	sc->sc_ncomdevs = 0;

	for (idx=0; idx < n_comdevs; ++idx) {
		struct upftdi_comdev *com;

		com = &sc->sc_comdev[idx];
		/* Preallocate xfers and data buffers. */
		com->xfer_in = usbp_alloc_xfer(dev);
		com->xfer_out = usbp_alloc_xfer(dev);


		com->buffer_in = usbd_alloc_buffer(com->xfer_in, UPFTDI_BUFSZ);
		com->buffer_out = usbd_alloc_buffer(com->xfer_out, UPFTDI_BUFSZ);
		if (com->buffer_in == NULL || com->buffer_out == NULL) {
			printf(": usbf_alloc_buffer failed\n");
			break;
		}

#if 0
		/* Open the bulk pipes. */
		err = usbp_open_pipe(com->iface,
		    usbd_endpoint_address(com->ep_out), &com->pipe_out) ||
		    usbp_open_pipe(com->iface,
			usbd_endpoint_address(com->ep_in), &com->pipe_in);
		if (err) {
			printf(": usbf_open_pipe failed\n");
			return;
		}
#endif

		upftdi_attach_ucom(self, dev, idx);

#if 0
		/* Get ready to receive packets. */
		usbd_setup_xfer(com->xfer_out, com->pipe_out, com,
		    com->buffer_out, UPFTDI_BUFSZ, USBD_SHORT_XFER_OK, 0, upftdi_rxeof);
		err = usbd_transfer(com->xfer_out);
		if (err && err != USBD_IN_PROGRESS) {
			printf(": usbf_transfer failed err=%d\n", err);

#if 0			/* FixME: this crashes the kernel */
			config_detach(com->ucomdev, DETACH_FORCE);
#endif
			/* XXX: release other resources */
			break;
		}
#endif
		
		++sc->sc_ncomdevs;
	}

#if 0
	s = splnet();


	splx(s);
#endif
}


static void
upftdi_attach_ucom(device_t self, struct usbp_device *dev, int idx)
{
	struct ucom_attach_args uca;
	struct upftdi_softc *sc = device_private(self);

	memset(&uca, 0, sizeof uca);
	uca.portno = FTDI_PIT_SIOA + idx;
	/* bulkin, bulkout set above */
	uca.ibufsize = UFTDIIBUFSIZE;
	uca.ibufsizepad = uca.ibufsize;
	uca.obufsize = UFTDIOBUFSIZE;
	uca.opkthdrlen = 0; // sc->sc_hdrlen;
	uca.device = (struct usbd_device *)dev;
	uca.iface =  (struct usbd_interface *)sc->sc_comdev[idx].iface;
	uca.bulkin = usbd_endpoint_address(sc->sc_comdev[idx].ep_in);
	uca.bulkout = usbd_endpoint_address(sc->sc_comdev[idx].ep_out);
	uca.methods = &upftdi_ucom_methods;
	uca.arg = self;
	uca.info = NULL;
	uca.portno = idx;

	sc->sc_comdev[idx].ucomdev = config_found_sm_loc(self, "ucombus", NULL,
	    &uca, ucomprint, ucomsubmatch);

}

static usbd_status
upftdi_req_reset(struct usbp_function *fun, usb_device_request_t *req)
{
	int portno = UGETW(req->wIndex);

	printf("%s: portno=%d\n", __func__, portno);
	
	return USBD_NORMAL_COMPLETION;;
}

static usbd_status
upftdi_req_flow(struct usbp_function *fun, usb_device_request_t *req)
{
	printf("%s: wIndex=%x\n", __func__,
	    UGETW(req->wIndex));
	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_req_bitmode(struct usbp_function *fun, usb_device_request_t *req)
{
	printf("%s: portno=%d mode=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_req_baudrate(struct usbp_function *fun, usb_device_request_t *req)
{
	printf("%s: portno=%d rate=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_req_setdata(struct usbp_function *fun, usb_device_request_t *req)
{
	printf("%s: portno=%d data=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_req_modemctrl(struct usbp_function *fun, usb_device_request_t *req)
{
	printf("%s: portno=%d ctrl=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_do_vendor_write(struct usbp_function *fun, usb_device_request_t *req, void **data)
{
	usbd_status err = USBD_STALLED;
	
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
		err = USBD_STALLED;
	}

	if (err == USBD_NORMAL_COMPLETION) {
		static uint16_t v = 0;
		USETW(req->wLength, 2);
		*data = (void *)&v;
		return err;
	}

	return USBD_STALLED;
}

/*
 * Handle non-standard request received on the control pipe.
 */
static usbd_status
upftdi_do_request(struct usbp_function *fun, usb_device_request_t *req, void **data)
{
#define C(x,y) ((x) | ((y) << 8))
	if (req->bmRequestType == UT_WRITE_VENDOR_DEVICE)
		return upftdi_do_vendor_write(fun, req, data);

	printf("requestType=%x request=%x\n",
	    req->bmRequestType,
	    req->bRequest);
	return USBD_STALLED;

#undef C
}


#if 0
static void
upftdi_rxeof(struct usbd_xfer *xfer, void *priv,
    usbd_status status)
{
	struct upftdi_comdev *com = priv;
	struct upftdi_softc  *sc = com->softc;
	int total_len = 0;
//	int s;

	DPRINTF(("upftdi_rxeof: xfer=%p, priv=%p, %s\n", xfer, priv,
		 usbd_errstr(status)));

	if (status != USBD_NORMAL_COMPLETION) {
		if (status == USBD_NOT_STARTED || status == USBD_CANCELLED)	
			return;
		if (sc->sc_rxeof_errors == 0)
			printf("%s: usb error on rx: %s\n",
			    DEVNAME(sc), usbd_errstr(status));
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

	
	usbd_get_xfer_status(xfer, NULL, NULL, &total_len, NULL);

	if (total_len <= 1)
		goto done;

done:
	/* Setup another xfer. */
	usbd_setup_xfer(xfer, com->pipe_out, com, com->buffer_out,
	    UPFTDI_BUFSZ, USBD_SHORT_XFER_OK, 0, upftdi_rxeof);

	status = usbd_transfer(xfer);
	if (status && status != USBD_IN_PROGRESS) {
		printf("%s: usbf_transfer failed\n", DEVNAME(sc));
		return;
	}
}
#endif


static usbd_status
upftdi_set_config(struct usbp_function *fun, struct usbp_config *config)
{
	DPRINTF(("%s: fun=%p\n", __func__, fun));
	return USBD_NORMAL_COMPLETION;
}



#if 0
static int
upftdi_open(void *vsc, int portno)
{
	struct upftdi_softc *sc = vsc;
	usb_device_request_t req;
	usbd_status err;
	struct termios t;

	DPRINTF(("uftdi_open: sc=%p\n", sc));

	if (sc->sc_dying)
		return (EIO);

	/* Perform a full reset on the device */
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_RESET;
	USETW(req.wValue, FTDI_SIO_RESET_SIO);
	USETW(req.wIndex, portno);
	USETW(req.wLength, 0);
	err = usbd_do_request(sc->sc_udev, &req, NULL);
	if (err)
		return (EIO);

	/* Set 9600 baud, 2 stop bits, no parity, 8 bits */
	t.c_ospeed = 9600;
	t.c_cflag = CSTOPB | CS8;
	(void)uftdi_param(sc, portno, &t);

	/* Turn on RTS/CTS flow control */
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = FTDI_SIO_SET_FLOW_CTRL;
	USETW(req.wValue, 0);
	USETW2(req.wIndex, FTDI_SIO_RTS_CTS_HS, portno);
	USETW(req.wLength, 0);
	err = usbd_do_request(sc->sc_udev, &req, NULL);
	if (err)
		return (EIO);

	return (0);
}
#endif

static void
upftdi_read(void *vsc, int portno, u_char **ptr, u_int32_t *count)
{
	struct upftdi_softc *sc = vsc;
//	u_char msr, lsr;

	DPRINTFN(15,("uftdi_read: sc=%p, port=%d count=%d\n", sc, portno,
		     *count));

#if 0
	msr = FTDI_GET_MSR(*ptr);
	lsr = FTDI_GET_LSR(*ptr);

#ifdef UFTDI_DEBUG
	if (*count != 2)
		DPRINTFN(10,("uftdi_read: sc=%p, port=%d count=%d data[0]="
			    "0x%02x\n", sc, portno, *count, (*ptr)[2]));
#endif

	if (sc->sc_msr != msr ||
	    (sc->sc_lsr & FTDI_LSR_MASK) != (lsr & FTDI_LSR_MASK)) {
		DPRINTF(("uftdi_read: status change msr=0x%02x(0x%02x) "
			 "lsr=0x%02x(0x%02x)\n", msr, sc->sc_msr,
			 lsr, sc->sc_lsr));
		sc->sc_msr = msr;
		sc->sc_lsr = lsr;
		ucom_status_change(device_private(sc->sc_subdev[portno-1]));
	}

	/* Adjust buffer pointer to skip status prefix */
	*ptr += 2;

#endif
}

static void
upftdi_write(void *vsc, int portno, u_char *to, u_char *from, u_int32_t *count)
{
//	struct uftdi_softc *sc = vsc;

	DPRINTFN(10,("uftdi_write: sc=%p, port=%d count=%u data[0]=0x%02x\n",
		     vsc, portno, *count, from[0]));

#if 0
	/* Make length tag and copy data */
	if (sc->sc_hdrlen > 0)
		*to = FTDI_OUT_TAG(*count, portno);

	memcpy(to + sc->sc_hdrlen, from, *count);
	*count += sc->sc_hdrlen;
#endif
}
