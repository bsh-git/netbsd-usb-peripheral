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

/*
 * Userland interface for Peripheral-side USB support.
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
#include <sys/conf.h>
#include <sys/fcntl.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>

#include <dev/usb/usbp.h>
#include <dev/usb/usbpvar.h>
#include <dev/usb/usbpif.h>

struct usbpu_softc {
	device_t	sc_dev;
	bool sc_dying;
};


static int usbpu_match(device_t, cfdata_t, void *);
static void usbpu_attach(device_t, device_t, void *);
static int usbpu_device_ioctl(struct usbpu_softc *, u_long, void *, int, struct lwp *);
static int usbpu_endpoint_ioctl(struct usbpu_softc *, u_long, void *, int, struct lwp *);

extern struct cfdriver usbpusr_cd;

CFATTACH_DECL3_NEW(usbpusr, sizeof(struct usbpu_softc),
		   usbpu_match, usbpu_attach, NULL, NULL, NULL, NULL, 0);


dev_type_open(usbpuopen);
dev_type_close(usbpuclose);
//dev_type_read(usbread);
dev_type_ioctl(usbpuioctl);
//dev_type_poll(usbpoll);
//dev_type_kqfilter(usbkqfilter);

const struct cdevsw usbp_cdevsw = {
	.d_open = usbpuopen,
	.d_close = usbpuclose,
	.d_read = noread,
	.d_write = nowrite,
	.d_ioctl = usbpuioctl,
	.d_stop = nostop,
	.d_tty = notty,
	.d_poll = nopoll,
	.d_mmap = nommap,
	.d_kqfilter = nokqfilter,
	.d_discard = nodiscard,
	.d_flag = D_OTHER
};


#define	USBPU_DEBUG_TRACE	(1<<0)
#ifndef USBPU_DEBUG
#define DPRINTF(l, x)	do {} while (0)
#else
int usbpudebug = 0xffffffff;
#define DPRINTF(l, x)	if ((l) & usbpdebug) printf x; else
#endif


#define	USBPU_ENDPOINT(minor)	((minor) & 0x0f)
#define	USBPU_ISDEVICE(minor)	((minor) & 0x80)
#define	USBPU_UNIT(minor)	((minor) >> 8 & 0xff)

static int
usbpu_match(device_t parent, cfdata_t match, void *aux)
{
	struct usbp_interface_attach_args *uia = aux;

//	printf("%s: cf_name=%s cf_atname=%s name=%s\n", __func__, match->cf_name, match->cf_atname, uia->busname);
	if (strcmp(uia->busname, "usbpu") != 0)
		return 0;
	return 1;
}

static void
usbpu_attach(device_t parent, device_t self, void *aux)
{
	struct usbpu_softc *sc = device_private(self);

	aprint_normal(": USB Peripheral interface\n");
	aprint_naive("\n");
	
	sc->sc_dev = self;
	sc->sc_dying = false;
}


int
usbpuopen(dev_t dev, int flag, int mode, struct lwp *l)
{
	int mnr = minor(dev);
	int unit = (mnr >> 8) & 0xff;
	struct usbpu_softc *sc;

	sc = device_lookup_private(&usbpusr_cd, unit);
	if (!sc)
		return (ENXIO);

	if (sc->sc_dying)
		return (EIO);

	return (0);
}

#if 0
int
usbpuread(dev_t dev, struct uio *uio, int flag)
{
	struct usb_event *ue;
#ifdef COMPAT_30
	struct usb_event_old *ueo = NULL;	/* XXXGCC */
	int useold = 0;
#endif
	int error, n;

	if (minor(dev) != USB_DEV_MINOR)
		return (ENXIO);

	switch (uio->uio_resid) {
#ifdef COMPAT_30
	case sizeof(struct usb_event_old):
		ueo = malloc(sizeof(struct usb_event_old), M_USBDEV,
			     M_WAITOK|M_ZERO);
		useold = 1;
		/* FALLTHRU */
#endif
	case sizeof(struct usb_event):
		ue = usb_alloc_event();
		break;
	default:
		return (EINVAL);
	}

	error = 0;
	mutex_enter(&usb_event_lock);
	for (;;) {
		n = usb_get_next_event(ue);
		if (n != 0)
			break;
		if (flag & IO_NDELAY) {
			error = EWOULDBLOCK;
			break;
		}
		error = cv_wait_sig(&usb_event_cv, &usb_event_lock);
		if (error)
			break;
	}
	mutex_exit(&usb_event_lock);
	if (!error) {
#ifdef COMPAT_30
		if (useold) { /* copy fields to old struct */
			ueo->ue_type = ue->ue_type;
			memcpy(&ueo->ue_time, &ue->ue_time,
			      sizeof(struct timespec));
			switch (ue->ue_type) {
				case USB_EVENT_DEVICE_ATTACH:
				case USB_EVENT_DEVICE_DETACH:
					usb_copy_old_devinfo(&ueo->u.ue_device, &ue->u.ue_device);
					break;

				case USB_EVENT_CTRLR_ATTACH:
				case USB_EVENT_CTRLR_DETACH:
					ueo->u.ue_ctrlr.ue_bus=ue->u.ue_ctrlr.ue_bus;
					break;

				case USB_EVENT_DRIVER_ATTACH:
				case USB_EVENT_DRIVER_DETACH:
					ueo->u.ue_driver.ue_cookie=ue->u.ue_driver.ue_cookie;
					memcpy(ueo->u.ue_driver.ue_devname,
					       ue->u.ue_driver.ue_devname,
					       sizeof(ue->u.ue_driver.ue_devname));
					break;
				default:
					;
			}

			error = uiomove((void *)ueo, sizeof *ueo, uio);
		} else
#endif
			error = uiomove((void *)ue, sizeof *ue, uio);
	}
	usb_free_event(ue);
#ifdef COMPAT_30
	if (useold)
		free(ueo, M_USBDEV);
#endif

	return (error);
}
#endif

int
usbpuclose(dev_t dev, int flag, int mode,
    struct lwp *l)
{
#if 0
	int unit = minor(dev);

	if (unit == USB_DEV_MINOR) {
		mutex_enter(proc_lock);
		usb_async_proc = 0;
		mutex_exit(proc_lock);
		usb_dev_open = 0;
	}

#endif
	return (0);
}

int
usbpuioctl(dev_t devt, u_long cmd, void *data, int flag, struct lwp *lwp)
{
	struct usbpu_softc *sc;
	int mnr = minor(devt);
	int unit = USBPU_UNIT(mnr);

	sc = device_lookup_private(&usbpusr_cd, unit);

	if (!sc)
		return ENXIO;
	if (sc->sc_dying)
		return EINVAL;

	if (USBPU_ISDEVICE(mnr))
		return usbpu_device_ioctl(sc, cmd, data, flag, lwp);
	return usbpu_endpoint_ioctl(sc, cmd, data, flag, lwp);
}
		

static int
usbpu_device_ioctl(struct usbpu_softc *sc, u_long cmd, void *data, int flag, struct lwp *lwp)
{
	device_t usbp = device_parent(sc->sc_dev);

	switch(cmd) {
	case USBP_IOC_ADDIFACE:
		return usbp_add_iface(usbp, (struct usbp_add_iface *)data, flag, lwp);
	case USBP_IOC_SETPULLDOWN:
		return usbp_set_pulldown(usbp, *(int *)data);
	}
	return EINVAL;
}

static int
usbpu_endpoint_ioctl(struct usbpu_softc *sc, u_long cmd, void *data, int flag, struct lwp *lwp)
{
	return EINVAL;
}


#if 0
if (unit == USB_DEV_MINOR) {
		switch (cmd) {
		case FIONBIO:
			/* All handled in the upper FS layer. */
			return (0);

		case FIOASYNC:
			mutex_enter(proc_lock);
			if (*(int *)data)
				usb_async_proc = l->l_proc;
			else
				usb_async_proc = 0;
			mutex_exit(proc_lock);
			return (0);

		default:
			return (EINVAL);
		}
	}

	sc = device_lookup_private(&usb_cd, unit);

	if (sc->sc_dying)
		return (EIO);

	switch (cmd) {
#ifdef USB_DEBUG
	case USB_SETDEBUG:
		if (!(flag & FWRITE))
			return (EBADF);
		usbdebug  = ((*(int *)data) & 0x000000ff);
		break;
#endif /* USB_DEBUG */
	case USB_REQUEST:
	{
		struct usb_ctl_request *ur = (void *)data;
		int len = UGETW(ur->ucr_request.wLength);
		struct iovec iov;
		struct uio uio;
		void *ptr = 0;
		int addr = ur->ucr_addr;
		usbd_status err;
		int error = 0;

		if (!(flag & FWRITE))
			return (EBADF);

		DPRINTF(("usbioctl: USB_REQUEST addr=%d len=%d\n", addr, len));
		if (len < 0 || len > 32768)
			return (EINVAL);
		if (addr < 0 || addr >= USB_MAX_DEVICES ||
		    sc->sc_bus->devices[addr] == NULL)
			return (EINVAL);
		if (len != 0) {
			iov.iov_base = (void *)ur->ucr_data;
			iov.iov_len = len;
			uio.uio_iov = &iov;
			uio.uio_iovcnt = 1;
			uio.uio_resid = len;
			uio.uio_offset = 0;
			uio.uio_rw =
				ur->ucr_request.bmRequestType & UT_READ ?
				UIO_READ : UIO_WRITE;
			uio.uio_vmspace = l->l_proc->p_vmspace;
			ptr = malloc(len, M_TEMP, M_WAITOK);
			if (uio.uio_rw == UIO_WRITE) {
				error = uiomove(ptr, len, &uio);
				if (error)
					goto ret;
			}
		}
		err = usbd_do_request_flags(sc->sc_bus->devices[addr],
			  &ur->ucr_request, ptr, ur->ucr_flags, &ur->ucr_actlen,
			  USBD_DEFAULT_TIMEOUT);
		if (err) {
			error = EIO;
			goto ret;
		}
		if (len > ur->ucr_actlen)
			len = ur->ucr_actlen;
		if (len != 0) {
			if (uio.uio_rw == UIO_READ) {
				error = uiomove(ptr, len, &uio);
				if (error)
					goto ret;
			}
		}
	ret:
		if (ptr)
			free(ptr, M_TEMP);
		return (error);
	}

	case USB_DEVICEINFO:
	{
		usbd_device_handle dev;
		struct usb_device_info *di = (void *)data;
		int addr = di->udi_addr;

		if (addr < 0 || addr >= USB_MAX_DEVICES)
			return EINVAL;
		if ((dev = sc->sc_bus->devices[addr]) == NULL)
			return ENXIO;
		usbd_fill_deviceinfo(dev, di, 1);
		break;
	}

#ifdef COMPAT_30
	case USB_DEVICEINFO_OLD:
	{
		usbd_device_handle dev;
		struct usb_device_info_old *di = (void *)data;
		int addr = di->udi_addr;

		if (addr < 1 || addr >= USB_MAX_DEVICES)
			return EINVAL;
		if ((dev = sc->sc_bus->devices[addr]) == NULL)
			return ENXIO;
		usbd_fill_deviceinfo_old(dev, di, 1);
		break;
	}
#endif

	case USB_DEVICESTATS:
		*(struct usb_device_stats *)data = sc->sc_bus->stats;
		break;

	default:
		return (EINVAL);
	}
	return (0);
}
#endif

