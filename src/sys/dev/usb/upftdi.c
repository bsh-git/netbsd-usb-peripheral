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
#include <sys/kmem.h>

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
int upftdidebug = 20;
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
	device_t	sc_dev;
	struct usbp_interface	sc_iface;
	int			sc_rxeof_errors;

	bool sc_dying;

	int sc_ncomdevs_requested;
	int sc_ncomdevs_active;

	struct upftdi_comdev {
		device_t ucomdev;
		struct usbd_endpoint	*ep_in;
		struct usbd_endpoint	*ep_out;
		struct usbd_pipe	*pipe_in;
		struct usbd_pipe	*pipe_out;
		struct usbd_xfer	*xfer_in;
		struct usbd_xfer	*xfer_out;
		void			*buffer_in;
		void			*buffer_out;

	} sc_comdev[MAXCOMDEVICES];
};


static int	upftdi_match(device_t, cfdata_t, void *);
static void	upftdi_attach(device_t, device_t, void *);
static int	upftdi_detach(device_t, int);

static usbd_status	upftdi_do_request(struct usbp_interface *,
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

static void upftdi_attach_ucom(device_t, struct usbp_device *, int,
    struct usbd_endpoint *, struct usbd_endpoint *);


static usbd_status upftdi_configured(struct usbp_interface *);
static usbd_status upftdi_unconfigured(struct usbp_interface *);

const struct usbp_interface_methods upftdi_if_methods = {
	upftdi_configured,
	upftdi_unconfigured,
	upftdi_do_request,
	NULL
};

CFATTACH_DECL_NEW(upftdi, sizeof (struct upftdi_softc),
    upftdi_match, upftdi_attach, upftdi_detach, NULL);

//static int upftdi_open(void *vsc, int portno);
//static void upftdi_read(void *vsc, int portno, u_char **ptr, u_int32_t *count);
static void upftdi_write(void *vsc, int portno, u_char *to, u_char *from, u_int32_t *count);

static struct ucom_methods upftdi_ucom_methods = {
//	.ucom_open = upftdi_open,
//	.ucom_read = upftdi_read,
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
	struct usbp_interface_attach_args *uaa = aux;
	struct usbp_device *dev = uaa->device;
	usbd_status err;
	int n_comdevs = 1;
	//int s;


	static const struct usbp_device_info devdata = {
		.class_id = UDCLASS_IN_INTERFACE,
		.subclass_id = 0,
		.protocol = 0,
		.vendor_id = USB_VENDOR_FTDI,
		.product_id = USB_PRODUCT_FTDI_SERIAL_230X,
		.bcd_device = UPFTDI_DEVICE_CODE,
		.manufacturer_name = UPFTDI_VENDOR_STRING,
		.product_name = UPFTDI_PRODUCT_STRING,
		.serial = UPFTDI_SERIAL_STRING
	};
#if 0
	struct usbp_interface_spec *ispec = alloca(sizeof (struct usbp_interface_spec) +
	    n_comdevs * 2 * sizeof (struct usbp_endpoint_request));

	static const struct usbp_endpoint_request
	    in_spec ={.address = UE_DIR_IN | 0,
		      .attributes = UE_BULK,
		      .packetsize = 64},
	    out_spec = {.address = UE_DIR_OUT | 0,
			.attributes = UE_BULK,
			.packetsize = 64};


	ispec->class_id = UICLASS_VENDOR;
	ispec->subclass_id = UICLASS_VENDOR;
	ispec->protocol = 0;
	ispec->pipe0_usage = USBP_PIPE0_OCCUPIED;
	ispec->description = NULL;
	ispec->num_endpoints = n_comdevs * 2;

	for (idx=0; idx < n_comdevs; ++idx) {
		ispec->endpoints[idx * 2] = in_spec;
		ispec->endpoints[idx * 2 + 1] = out_spec;
	}
#else
	static struct usbp_interface_spec _ispec = {
		.class_id = UICLASS_VENDOR,
		.subclass_id = UICLASS_VENDOR,
		.protocol = 0,
		.pipe0_usage = USBP_PIPE0_EXCLUSIVE,
		.description = NULL,
		.num_endpoints = 2,
		.endpoints = {
			{.address = UE_DIR_IN | 0, .attributes = UE_BULK, .packetsize = 64},
			{.address= UE_DIR_OUT | 0, .attributes = UE_BULK, .packetsize = 64},
		}
	};
	struct usbp_interface_spec *ispec = &_ispec;
#endif

	aprint_normal(": enulates FTDI USB serial\n");
	aprint_naive("\n");

	DPRINTF(("%s  dev=%p sc=%p upftdi_if_methods=%p\n", __func__, dev,
		sc,
		&upftdi_if_methods));
	
	err = usbp_add_interface(dev, &sc->sc_iface, &devdata, ispec,
	    &upftdi_if_methods, NULL, 0);
	if (err != USBD_NORMAL_COMPLETION) {
		aprint_error_dev(self, "%s: usbp_add_interface failed (%d)\n",
		    __func__, err);
		return;
	}

	sc->sc_iface.usbd.priv = sc;
	sc->sc_dev = self;
	sc->sc_ncomdevs_requested = n_comdevs;

	
}


static void
upftdi_attach_ucom(device_t self, struct usbp_device *udev, int idx,
    struct usbd_endpoint *bulk_in,
    struct usbd_endpoint *bulk_out)
{
	struct ucom_attach_args uca;
	struct upftdi_softc *sc = device_private(self);

	printf("%s: bulkin=%p,0x%x buldout=%p,0x%x\n",
	    __func__,
	    bulk_in, usbd_endpoint_address(bulk_in),
	    bulk_out, usbd_endpoint_address(bulk_out));
	
	memset(&uca, 0, sizeof uca);
	uca.portno = FTDI_PIT_SIOA + idx;
	/* bulkin, bulkout set above */
	uca.ibufsize = UFTDIIBUFSIZE;
	uca.ibufsizepad = uca.ibufsize;
	uca.obufsize = UFTDIOBUFSIZE;
	uca.opkthdrlen = 0; // sc->sc_hdrlen;
	uca.device = (struct usbd_device *)udev;
	uca.iface =  &sc->sc_iface.usbd;
	/* we give bulk_in and bulk_out inside-out here.
	   because OUT is host-to-device, and it is input for us */
	uca.bulkin = usbd_endpoint_address(bulk_out);
	uca.bulkout = usbd_endpoint_address(bulk_in);
	uca.methods = &upftdi_ucom_methods;
	uca.arg = self;
	uca.info = NULL;
	uca.portno = idx;

	sc->sc_comdev[idx].ucomdev = config_found_sm_loc(self, "ucombus", NULL,
	    &uca, ucomprint, ucomsubmatch);

}

static usbd_status
upftdi_req_reset(struct usbp_interface *iface, usb_device_request_t *req)
{
	int portno = UGETW(req->wIndex);

	printf("%s: portno=%d\n", __func__, portno);
	
	return USBD_NORMAL_COMPLETION;;
}

static usbd_status
upftdi_req_flow(struct usbp_interface *iface, usb_device_request_t *req)
{
	printf("%s: wIndex=%x\n", __func__,
	    UGETW(req->wIndex));
	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_req_bitmode(struct usbp_interface *iface, usb_device_request_t *req)
{
	printf("%s: portno=%d mode=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_req_baudrate(struct usbp_interface *iface, usb_device_request_t *req)
{
	printf("%s: portno=%d rate=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_req_setdata(struct usbp_interface *iface, usb_device_request_t *req)
{
	printf("%s: portno=%d data=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_req_modemctrl(struct usbp_interface *iface, usb_device_request_t *req)
{
	printf("%s: portno=%d ctrl=%x\n", __func__,
	    UGETW(req->wIndex),
	    UGETW(req->wValue));

	return USBD_NORMAL_COMPLETION;
}

static usbd_status
upftdi_do_vendor_write(struct usbp_interface *iface, usb_device_request_t *req, void **data)
{
	usbd_status err = USBD_STALLED;
	
	switch(req->bRequest) {
	case FTDI_SIO_RESET:
		 err = upftdi_req_reset(iface, req);
		 break;
	case FTDI_SIO_SET_FLOW_CTRL:
		err = upftdi_req_flow(iface, req);
	case FTDI_SIO_SET_BITMODE:
		err = upftdi_req_bitmode(iface, req);
		break;
	case FTDI_SIO_SET_BAUD_RATE:
		err = upftdi_req_baudrate(iface, req);
		break;
	case FTDI_SIO_SET_DATA:
		err = upftdi_req_setdata(iface, req);
		break;
	case FTDI_SIO_MODEM_CTRL:
		err = upftdi_req_modemctrl(iface, req);
		break;
	default:
		printf("%s: unknown request 0x%x\n", __func__, req->bRequest);
		err = USBD_STALLED;
	}

	if (err == USBD_NORMAL_COMPLETION) {
		static uint16_t zero = 0;
		USETW(req->wLength, 2);
		*data = (void *)&zero;
		return err;
	}

	return USBD_STALLED;
}

/*
 * Handle non-standard request received on the control pipe.
 */
static usbd_status
upftdi_do_request(struct usbp_interface *iface, usb_device_request_t *req, void **data)
{
	printf("%s: requestType=%x request=%x\n",
	    __func__,
	    req->bmRequestType,
	    req->bRequest);

#define C(x,y) ((x) | ((y) << 8))
	if (req->bmRequestType == UT_WRITE_VENDOR_DEVICE)
		return upftdi_do_vendor_write(iface, req, data);

	return USBD_NOT_FOR_US;

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

Static void
upftdi_write(void *vsc, int portno, u_char *to, u_char *from, u_int32_t *count)
{
//	struct upftdi_softc *sc = vsc;

	DPRINTFN(10,("uftdi_write: sc=%p, port=%d count=%u data[0]=0x%02x\n",
		     vsc, portno, *count, from[0]));

	
	/* XXX: RSLD, RI, DSR, CTS, portno */
	to[0] = portno;
	to[1] = 0x60;	/* XXX: LSR */

	memcpy(to + 2, from, *count);
	*count += 2;
}


static usbd_status
upftdi_configured(struct usbp_interface *iface)
{
	int idx;
	struct upftdi_softc *sc = iface->usbd.priv;
	struct usbp_device *udev = (struct usbp_device *)iface->usbd.device;
	int n_comdevs = MIN(sc->sc_ncomdevs_requested,
	    iface->usbd.idesc->bNumEndpoints / 2);
	
	sc->sc_ncomdevs_active = 0;

	for (idx=0; idx < n_comdevs; ++idx) {
		upftdi_attach_ucom(sc->sc_dev, udev, idx,
		    &iface->usbd.endpoints[idx * 2],
		    &iface->usbd.endpoints[idx * 2 + 1]);

		++sc->sc_ncomdevs_active;
	}

#if 0
	s = splnet();


	splx(s);
#endif

	return USBD_NORMAL_COMPLETION;
}

/* detach ucoms */
static void
detach_ucoms(struct upftdi_softc *sc, int flags)
{
	int i, err;
	
	DPRINTF(("%s: n_ucoms=%d\n", __func__, sc->sc_ncomdevs_active));

	for (i=0; i < sc->sc_ncomdevs_active; ++i) {
		if (sc->sc_comdev[i].ucomdev == NULL) {
			continue;
		}
		err = config_detach(sc->sc_comdev[i].ucomdev, flags);
		if (err) {
			aprint_error_dev(sc->sc_dev,
			    "can't detach ucom [%d] err=%d\n", i, err);
			/* XXX: what should we do? */
		}
		sc->sc_comdev[i].ucomdev = NULL;
	}
	sc->sc_ncomdevs_active = 0;
}


static usbd_status
upftdi_unconfigured(struct usbp_interface *iface)
{
	struct upftdi_softc *sc = iface->usbd.priv;

	DPRINTF(("%s\n", __func__));
	detach_ucoms(sc, 0);

	return USBD_NORMAL_COMPLETION;
}


static int
upftdi_detach(device_t self, int flags)
{
	struct upftdi_softc *sc = device_private(self);
	struct usbp_interface *iface = &sc->sc_iface;
	usbd_status uerr;
	int err;

	detach_ucoms(sc, flags);

	uerr = usbp_delete_interface(iface);

	if (uerr != USBD_NORMAL_COMPLETION)
		err = EINVAL;	/* XXX */

	return err;
}


