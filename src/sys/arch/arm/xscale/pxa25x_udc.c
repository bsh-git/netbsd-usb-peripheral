/*	$NetBSD$ */
/* Copyright (c) 2015, 2016 Hiroyuki Bessho <bsh@netbsd.org>
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

/* based on pxa27x_udc.c from OpenBSD */
/*	$OpenBSD: pxa27x_udc.c,v 1.31 2015/05/15 13:32:08 jsg Exp $ */

/*
 * Copyright (c) 2007 Dale Rahn <drahn@openbsd.org>
 * Copyright (c) 2006 Uwe Stuehler <uwe@openbsd.org>
 * Copyright (c) 2005 David Gwynne <dlg@openbsd.org>
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/callout.h>

#include <machine/intr.h>
#include <sys/bus.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>
#include <dev/usb/usbp.h>

#include <arm/xscale/pxa2x0reg.h>
#include <arm/xscale/pxa2x0var.h>
#include <arm/xscale/pxa2x0_gpio.h>

#include <arm/xscale/pxa2x0_udc.h>

#include "usbp.h"

//#define	DEBUG_RX
//#define	DEBUG_TX
#define PXAUDC_DEBUG

struct pxaudc_xfer {
	struct usbd_xfer	 xfer;
	u_int16_t		 frmlen;
};

void		 pxa25xudc_enable(struct pxaudc_softc *);
void		 pxa25xudc_disable(struct pxaudc_softc *);

void		 pxa25xudc_intr1(struct pxaudc_softc *);
void		 pxa25xudc_ep0_intr(struct pxaudc_softc *);
void		 pxa25xudc_epN_intr(struct pxaudc_softc *sc, int ep, int isr);

usbd_status	 pxaudc_open(struct usbd_pipe *);
void		 pxaudc_softintr(void *);
usbd_status	 pxaudc_allocm(struct usbd_bus *, usb_dma_t *, u_int32_t);
void		 pxaudc_freem(struct usbd_bus *, usb_dma_t *);
struct usbd_xfer *pxaudc_allocx(struct usbd_bus *);
void		 pxaudc_freex(struct usbd_bus *, struct usbd_xfer *);

usbd_status	 pxaudc_ctrl_transfer(struct usbd_xfer *);
usbd_status	 pxaudc_ctrl_start(struct usbd_xfer *);
void		 pxaudc_ctrl_abort(struct usbd_xfer *);
void		 pxaudc_ctrl_done(struct usbd_xfer *);
void		 pxaudc_ctrl_close(struct usbd_pipe *);

usbd_status	 pxaudc_bulk_transfer(struct usbd_xfer *);
usbd_status	 pxaudc_bulk_start(struct usbd_xfer *);
void		 pxaudc_bulk_abort(struct usbd_xfer *);
void		 pxaudc_bulk_done(struct usbd_xfer *);
void		 pxaudc_bulk_close(struct usbd_pipe *);


static void pxaudc_read_ep0(struct pxaudc_softc *, struct usbd_xfer *);
static void pxaudc_read_epN(struct pxaudc_softc *, int);
static void pxaudc_write_ep0(struct pxaudc_softc *, struct usbd_xfer *);
static void pxaudc_write(struct pxaudc_softc *, struct usbd_xfer *);
static void pxaudc_write_epN(struct pxaudc_softc *sc, int ep);
static int pxaudc_intr(void *);
static void pxaudc_intr1(struct pxaudc_softc *);
static void pxaudc_ep0_intr(struct pxaudc_softc *);
static void pxaudc_epN_intr(struct pxaudc_softc *, int);
static void pxaudc_get_lock(struct usbd_bus *bus, kmutex_t **mutex);
static usbd_status pxaudc_new_device(device_t dev, usbd_bus_handle bus, int a,
    int b, int c, struct usbd_port *port);
static 	usbd_status pxaudc_select_endpoint(struct usbp_bus *, struct usbp_endpoint_request *, u_int);
static usbd_status pxaudc_enable_bus(struct usbp_bus *, bool);


#if NUSBP > 0

struct usbd_bus_methods pxaudc_bus_methods = {
	pxaudc_open,
	pxaudc_softintr,
	NULL,		// do_poll
	pxaudc_allocm,
	pxaudc_freem,
	pxaudc_allocx,
	pxaudc_freex,
	pxaudc_get_lock,
	pxaudc_new_device
};

struct usbd_pipe_methods pxaudc_ctrl_methods = {
	pxaudc_ctrl_transfer,
	pxaudc_ctrl_start,
	pxaudc_ctrl_abort,
	pxaudc_ctrl_close,
	NULL,		// cleartoggle
	pxaudc_ctrl_done
};

struct usbd_pipe_methods pxaudc_bulk_methods = {
	pxaudc_bulk_transfer,
	pxaudc_bulk_start,
	pxaudc_bulk_abort,
	pxaudc_bulk_close,
	NULL, // cleartoggle
	pxaudc_bulk_done
};

#endif /* NUSBP > 0 */

#define DEVNAME(sc)	device_xname((sc)->sc_dev)

#define CSR_READ_4(sc, reg) \
	bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, (reg))
#define CSR_READ_1(sc, reg) \
	bus_space_read_1((sc)->sc_iot, (sc)->sc_ioh, (reg))
#define CSR_WRITE_4(sc, reg, val) \
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, (reg), (val))
#define CSR_WRITE_1(sc, reg, val) \
	bus_space_write_1((sc)->sc_iot, (sc)->sc_ioh, (reg), (val))
#define CSR_SET_4(sc, reg, val) \
	CSR_WRITE_4((sc), (reg), CSR_READ_4((sc), (reg)) | (val))
#define CSR_CLR_4(sc, reg, val) \
	CSR_WRITE_4((sc), (reg), CSR_READ_4((sc), (reg)) & ~(val))

#ifndef PXAUDC_DEBUG
#define DPRINTF(l, x)	do {} while (0)
#else
int pxaudcdebug = 10;
#define DPRINTF(l, x)	if ((l) <= pxaudcdebug) printf x; else {}
#endif

static const struct {
	bus_size_t data;
	bus_size_t count;
	int fifo_length;

} udc_ep_regs[16] = {
	{ USBDC_UDDR0, 0, 16},
	{ USBDC_UDDR1, 0, 64 },
	{ USBDC_UDDR2, USBDC_UBCR2, 64 },
	{ USBDC_UDDR3, 0, 256 },
	{ USBDC_UDDR4, USBDC_UBCR4, 256 },
	{ USBDC_UDDR5, 0, 8 },
	{ USBDC_UDDR6, 0, 64 },
	{ USBDC_UDDR7, USBDC_UBCR7, 64 },
	{ USBDC_UDDR8, 0, 256 },
	{ USBDC_UDDR9, USBDC_UBCR9, 256 },
	{ USBDC_UDDR10, 0, 8 },
	{ USBDC_UDDR11, 0, 64 },
	{ USBDC_UDDR12, USBDC_UBCR12, 64 },
	{ USBDC_UDDR13, 0, 256 },
	{ USBDC_UDDR14, USBDC_UBCR14, 256 },
	{ USBDC_UDDR15, 0, 8 }
};

#if 0
int
pxaudc_match(void)
{
#if 0
	if ((cputype & ~CPU_ID_XSCALE_COREREV_MASK) != CPU_ID_PXA27X)
		return (0);
#endif
	return (1);
}

void
pxaudc_attach(device_t parent, device_t self, void *aux)
{
	struct pxaip_attach_args	*pxa = aux;

	aprint_normal(": USB Device controller\n");
	aprint_naive("\n");

	pxaudc_attach_sub(self, pxa);
}
#endif

int
pxaudc_attach_sub(device_t self, struct pxaip_attach_args *pxa,
    const struct usbp_bus_methods *platform_bus_methods)
{
	struct pxaudc_softc *sc = device_private(self);
#if NUSBP > 0
	struct usbp_bus_attach_args uaa;
#endif

	sc->sc_dev = self;
	memset(&sc->sc_bus, 0, sizeof sc->sc_bus);
	sc->sc_bus.usbd.hci_private = sc;
	sc->sc_iot = pxa->pxa_iot;

	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_SOFTUSB);

	if (bus_space_map(sc->sc_iot, pxa->pxa_addr, pxa->pxa_size, 0,
	    &sc->sc_ioh)) {
		printf(": cannot map mem space\n");
		return -1;
	}
	sc->sc_size = pxa->pxa_size;
	bus_space_barrier(sc->sc_iot, sc->sc_ioh, 0, sc->sc_size,
	    BUS_SPACE_BARRIER_READ|BUS_SPACE_BARRIER_WRITE);

	callout_init(&sc->sc_callout, 0);

	/* Set up GPIO pins and disable the controller. */
	pxa25xudc_disable(sc);


#if NUSBP > 0
	/* Establish USB device interrupt. */
	sc->sc_ih = pxa2x0_intr_establish(PXA2X0_INT_USB, IPL_USB,
	    pxaudc_intr, sc);
	if (sc->sc_ih == NULL) {
		printf(": unable to establish interrupt\n");
		bus_space_unmap(sc->sc_iot, sc->sc_ioh, sc->sc_size);
		sc->sc_size = 0;
		return -1;
	}


	/* Set up the bus struct. */
	sc->sc_bus.usbd.methods = &pxaudc_bus_methods;
	sc->usbp_bus_methods = *platform_bus_methods;
	sc->sc_bus.usbp_methods = &sc->usbp_bus_methods;
	if (sc->usbp_bus_methods.select_endpoint == NULL)
		sc->usbp_bus_methods.select_endpoint = pxaudc_select_endpoint;
	if (sc->usbp_bus_methods.enable == NULL)
		sc->usbp_bus_methods.enable = pxaudc_enable_bus;
#ifdef	DIAGNOSTIC
	if (sc->sc_bus.usbp_methods->is_connected == NULL) {
		aprint_error_dev(self,
		    "platform dependent code doesn't provide "
		    "a method to detect a connection to a host.");
	}
#endif

	sc->sc_bus.usbd.pipe_size = sizeof(struct usbd_pipe);
	sc->sc_bus.ep0_maxp = PXAUDC_EP0MAXP;
	sc->sc_bus.usbd.usbrev = USBREV_1_1;
	sc->sc_bus.usbd.dmatag = pxa->pxa_dmat;
	sc->sc_bus.usbd.lock = &sc->sc_lock;
	sc->sc_npipe = 0;	/* ep0 is always there. */

	uaa.busname = "usbp";
	uaa.bus = &sc->sc_bus;

	/* Attach USB interfaces */
	(void)config_found(self, &uaa, NULL);

#if 0
	/* Enable the controller unless we're now acting as a host. */
	if (!pxaudc_is_host(sc))
		pxa25xudc_enable(sc);
#endif
#endif	/* NUSBP > 0 */

	return 0;
}

#if 0
int
pxaudc_detach(struct pxaudc_softc *sc, int flags)
{
	if (sc->sc_conn_ih != NULL)
		pxa2x0_gpio_intr_disestablish(sc->sc_conn_ih);

	if (sc->sc_ih != NULL)
		pxa2x0_intr_disestablish(sc->sc_ih);

	if (sc->sc_size) {
		bus_space_unmap(sc->sc_iot, sc->sc_ioh, sc->sc_size);
		sc->sc_size = 0;
	}

	callout_halt(&sc->callout, &sc->sc_lock);
	//usb_delay_ms(&sc->sc_bus, 300); /* XXX let stray task complete */
	callout_destroy(&sc->sc_callout);


	mutex_destroy(&sc->sc_lock);

	return (0);
}

int
pxaudc_activate(struct pxaudc_softc *self, int act)
{
	struct pxaudc_softc *sc = (struct pxaudc_softc *)self;

	switch (act) {
	case DVACT_SUSPEND:
		pxa25xudc_disable(sc);
		break;
	case DVACT_RESUME:
		pxa25xudc_enable(sc);
		break;
	}
	return 0;
}
#endif

/*
 * Register manipulation
 */

#if 0
static void
pxaudc_dump_regs(struct pxaudc_softc *sc)
{
	printf("UDCCR\t%b\n", CSR_READ_4(sc, USBDC_UDCCR),
	    USBDC_UDCCR_BITS);
	printf("UDCICR0\t%b\n", CSR_READ_4(sc, USBDC_UDCICR0),
	    USBDC_UDCISR0_BITS);
	printf("UDCICR1\t%b\n", CSR_READ_4(sc, USBDC_UDCICR1),
	    USBDC_UDCISR1_BITS);
	printf("OTGICR\t%b\n", CSR_READ_4(sc, USBDC_UDCOTGICR),
	    USBDC_UDCOTGISR_BITS);
}
#endif

#if 0
static void
check(void *arg)
{
	struct pxaudc_softc *sc = arg;

	printf("check: udccs0=%x usirr0=%x\n",
	    CSR_READ_4(sc, USBDC_UDCCS(0)),
	    CSR_READ_4(sc, USBDC_USIR0));

	callout_schedule(&sc->callout, hz/10);
}
#endif

void
pxa25xudc_enable(struct pxaudc_softc *sc)
{
	int i;
	uint32_t icr, ccr;

	DPRINTF(10,("pxaudc_enable\n"));

	/* Start the clocks. */
	pxa2x0_clkman_config(CKEN_USBDC, 1);

	icr = 0xfffe;

	for (i = 1; i < sc->sc_npipe; i++) {
		if (sc->sc_pipe[i] != NULL)  {
			struct usbd_endpoint *ep = sc->sc_pipe[i]->endpoint;
			int dir = usbd_endpoint_dir(ep);

			icr &= ~(1 << i);

			/* clear old status, flush TX Fifo */
			if (dir == UE_DIR_IN) 
				CSR_WRITE_4(sc, USBDC_UDCCS(i),
					    USBDC_UDCCS_TUR|USBDC_UDCCS_TPC|
					    USBDC_UDCCS_SST|USBDC_UDCCS_FTF);
			else
				CSR_WRITE_4(sc, USBDC_UDCCS(i),
					    USBDC_UDCCS_RPC|USBDC_UDCCS_SST);

			DPRINTF(10,("%s: ep%d-%s\n", __func__,
				i, dir == UE_DIR_IN ? "in" : "out"));
		}
	}

	CSR_WRITE_4(sc, USBDC_USIR0, 0xff); /* clear all */
	CSR_WRITE_4(sc, USBDC_USIR1, 0xff); /* clear all */
	CSR_SET_4(sc, USBDC_UDCCFR, USBDC_UDCCFR_ACM);

	DPRINTF(10,("%s: interrupt mask=%x\n", __func__, icr));

	/* Enable interrupts for configured endpoints. */
	CSR_WRITE_4(sc, USBDC_UICR0, icr & 0xff);
	CSR_WRITE_4(sc, USBDC_UICR1, (icr >> 8) & 0xff);

	/* Enable the controller. */
	ccr = CSR_READ_4(sc, USBDC_UDCCR);
	ccr &= ~(USBDC_UDCCR_REM|USBDC_UDCCR_SRM);  /* enable reset/resume/suspend interrupt */
	ccr |= USBDC_UDCCR_UDE;		/* enable the controller */
	CSR_WRITE_4(sc, USBDC_UDCCR, ccr);

	DPRINTF(10, ("%s: UDCCR=%x\n", __func__, CSR_READ_4(sc, USBDC_UDCCR)));

	sc->sc_enabled = true;

//	callout_reset(&sc->callout, hz/10, check, sc);

}

void
pxa25xudc_disable(struct pxaudc_softc *sc)
{
	DPRINTF(10,("%s\n", __func__));

	/* disable the controller, disable all interrupts */
	CSR_WRITE_4(sc, USBDC_UDCCR, USBDC_UDCCR_SRM|USBDC_UDCCR_REM);
	CSR_WRITE_4(sc, USBDC_UICR0, 0xff);
	CSR_WRITE_4(sc, USBDC_UICR1, 0xff);

	/* Stop the clocks. */
	pxa2x0_clkman_config(CKEN_USBDC, 0);

	sc->sc_enabled = false;
	DPRINTF(10, ("%s: UDCCR=%x\n", __func__, CSR_READ_4(sc, USBDC_UDCCR)));
}

#if NUSBP > 0

/*
 * Endpoint FIFO handling
 */

static void
pxaudc_read_ep0(struct pxaudc_softc *sc, struct usbd_xfer *xfer)
{
	ssize_t len;
	u_int8_t *p;
	int count = 0;
	uint32_t ccs;

	if (xfer == NULL) {
		len = 0;
		p = NULL;
	}
	else {
		len = xfer->length;
		p = xfer->buffer;
	}

	while ((ccs = CSR_READ_4(sc, USBDC_UDCCS(0))) & USBDC_UDCCS_RNE) {
		u_int8_t v = CSR_READ_1(sc, USBDC_UDDR0);
		//printf("[%d]=%02x\n", count, v);
		if (count < len) 
			*p++ = v;
		++count;
	}

	CSR_WRITE_4(sc, USBDC_UDCCS(0), USBDC_UDCCS0_SA | USBDC_UDCCS0_OPR);
	printf("%s: csr0=%x\n", __func__, CSR_READ_4(sc, USBDC_UDCCS(0)));

	if (xfer != NULL) {
		xfer->actlen = count;
		xfer->status = USBD_NORMAL_COMPLETION;
		usbp_transfer_complete(xfer);
	}

	printf("read_ep0 done. count=%d (%p)\n", count, xfer);

}

static void
pxaudc_read_ep0_errata(struct pxaudc_softc *sc, struct usbd_xfer *xfer)
{
	if (xfer) {
		bus_space_read_multi_1(sc->sc_iot, sc->sc_ioh, USBDC_UDDR0,
		    xfer->buffer, 8);
	}

	CSR_WRITE_4(sc, USBDC_UDCCS(0), USBDC_UDCCS0_SA | USBDC_UDCCS0_OPR);
	printf("%s: csr0=%x\n", __func__, CSR_READ_4(sc, USBDC_UDCCS(0)));

	/* XXX: Check if correct control packet */

	if (xfer != NULL) {
		xfer->actlen = 8;
		xfer->status = USBD_NORMAL_COMPLETION;
		usbp_transfer_complete(xfer);
	}

	printf("read_ep0_errata done. (%p)\n", xfer);

}

static size_t
pxaudc_read_sub(struct pxaudc_softc *sc, int ep, uint32_t csr, uint8_t *buf, size_t buflen)
{
	uint32_t count;
	size_t len;
	uint32_t datareg = udc_ep_regs[ep].data, countreg = udc_ep_regs[ep].count;
	
	count = CSR_READ_4(sc, countreg) + 1;
	len = MIN(count, buflen);

#ifdef DEBUG_RX
	printf("reading data from ep%d len %zd csr %x\n", ep, len, csr);
#endif
	bus_space_read_multi_1(sc->sc_iot, sc->sc_ioh, datareg, buf, len);
	return len;
}

static void
pxaudc_read_epN(struct pxaudc_softc *sc, int ep)
{
	size_t len;
	struct usbd_pipe *pipe = NULL;
	struct usbd_xfer *xfer = NULL;
	u_int32_t csr;
	uint32_t countreg = udc_ep_regs[ep].count;

	DPRINTF(10, ("read_ep%d ppipe=%p\n", ep, sc->sc_pipe[ep]));
	if (countreg == 0) {
		aprint_error("%s: %s: EP%d is not OUT endpoint\n",
			     DEVNAME(sc), __func__, ep);
		return;
	}

	pipe = sc->sc_pipe[ep];
	if (pipe == NULL) {
#ifdef DEBUG_RX
		printf("%s: read no pipe (ep%d)\n", __func__, ep);
#endif
		return;
	}

	do {

		xfer = SIMPLEQ_FIRST(&pipe->queue);

		if (xfer == NULL) {
			aprint_error("pxaudc_read_epN: ep%d, no xfer\n", ep);
			return;
		}

		csr = CSR_READ_4(sc, USBDC_UDCCS(ep));
#ifdef DEBUG_RX
		printf("%s: csr%d=%x count=%d\n", __func__, ep, csr, 1+CSR_READ_4(sc,countreg));
#endif

		if ((csr & (USBDC_UDCCS_RNE|USBDC_UDCCS_RSP)) == USBDC_UDCCS_RSP) {
#ifdef DEBUG_RX
			printf("trans1 complete (zero-length packet)\n");
#endif
			xfer->status = USBD_NORMAL_COMPLETION;
			usbp_transfer_complete(xfer);
			CSR_SET_4(sc, USBDC_UDCCS(ep), USBDC_UDCCS_RPC);
			return;
		}

#if 1
		if ((csr & USBDC_UDCCS_RNE) == 0) {
			/* no data in FIFO */
			CSR_SET_4(sc, USBDC_UDCCS(ep), USBDC_UDCCS_RPC);
			return;
		}
#endif

		len = pxaudc_read_sub(sc, ep, csr,
		    (uint8_t *)(xfer->buffer) + xfer->actlen,
		    xfer->length - xfer->actlen);

		xfer->actlen += len;
		
		if (xfer->length == xfer->actlen || (len == 0 && xfer->actlen != 0) 
		    || (csr & USBDC_UDCCS_RSP) ) {
#ifdef DEBUG_RX
			printf("trans2 complete\n");
#endif
			xfer->status = USBD_NORMAL_COMPLETION;
			usbp_transfer_complete(xfer);
		}
		csr = CSR_READ_4(sc, USBDC_UDCCS(ep));
#ifdef DEBUG_RX
		printf("csr now %x len %d\n",
		       csr, CSR_READ_4(sc, countreg));
#endif
	} while (csr & USBDC_UDCCS_RFS);

#ifdef DEBUG_RX
	printf("complete reading.\n");
#endif
	CSR_SET_4(sc, USBDC_UDCCS(ep), USBDC_UDCCS_RPC);
}

static void
pxaudc_clear_epN(struct pxaudc_softc *sc, int ep)
{
	u_int32_t csr;
	uint8_t buf[1024];	//XXX

	if (udc_ep_regs[ep].count == 0) {
		aprint_error("%s: %s: EP%d is not OUT endpoint\n",
			     DEVNAME(sc), __func__, ep);
		return;
	}

	csr = CSR_READ_4(sc, USBDC_UDCCS(ep));

	if ((csr & USBDC_UDCCS_RNE) == 0) {
		/* no data in FIFO */
		return;
	}
		
	pxaudc_read_sub(sc, ep, csr, buf, sizeof buf);
}

void
pxaudc_write_ep0(struct pxaudc_softc *sc, struct usbd_xfer *xfer)
{
	struct pxaudc_xfer *lxfer = (struct pxaudc_xfer *)xfer;
	u_int32_t len;
	u_int8_t *p;

	DPRINTF(10,("%s: %s: xfer=%s frmlen=%u\n",
		DEVNAME(sc), __func__, usbp_describe_xfer(xfer), lxfer->frmlen));

	if (lxfer->frmlen > 0) {
		xfer->actlen += lxfer->frmlen;
		lxfer->frmlen = 0;
	}


	if (xfer->actlen >= xfer->length) {
		sc->sc_bus.ep0state = EP0_END_XFER;
		printf("write_ep0 done\n");
		usbp_transfer_complete(xfer);
		return;
	}

	sc->sc_bus.ep0state = EP0_IN_DATA_PHASE;

	p = (u_char *)xfer->buffer + xfer->actlen;
	len = xfer->length - xfer->actlen;
	len = MIN(len, PXAUDC_EP0MAXP);
	lxfer->frmlen = len;

	bus_space_write_multi_1(sc->sc_iot, sc->sc_ioh, USBDC_UDDR0, p, len);

	/* (12.6.7) Set IPR only for short packets. */
	if (lxfer->frmlen < PXAUDC_EP0MAXP)
		CSR_WRITE_4(sc, USBDC_UDCCS(0), USBDC_UDCCS0_IPR);
	DPRINTF(10,("%s: %s: wrote %d bytes\n",
		DEVNAME(sc), __func__, lxfer->frmlen));
}

void
pxaudc_write(struct pxaudc_softc *sc, struct usbd_xfer *xfer)
{
	u_int8_t *p;
	struct usbd_endpoint *endpoint = xfer->pipe->endpoint;
	int epidx = usbd_endpoint_index(endpoint);
	int maxp = UGETW(endpoint->edesc->wMaxPacketSize);
	int fifo_length = udc_ep_regs[epidx].fifo_length;
	u_int32_t csr, csr_o;
	bus_size_t datareg = udc_ep_regs[epidx].data;
#ifdef DEBUG_TX
	int tlen = 0;
#endif

	fifo_length = MIN(fifo_length, maxp);

#ifdef DEBUG_TX_PKT
	if (xfer->actlen == 0)
		printf("new packet len %x\n", xfer->length);
#endif

#ifdef DEBUG_TX
	printf("%s: writing data to endpoint %x, xlen %d xact %d\n",
	    __func__,
	    epidx, xfer->length, xfer->actlen);
#endif


	if (xfer->actlen == xfer->length) {
		/*
		 * If the packet size is wMaxPacketSize byte multiple
		 * send a zero packet to indicate termiation.
		 */
		if ((xfer->actlen % maxp) == 0 &&
		    xfer->status != USBD_NORMAL_COMPLETION &&
		    xfer->flags & USBD_FORCE_SHORT_XFER) {
			if (CSR_READ_4(sc, USBDC_UDCCS(epidx))
			    & USBDC_UDCCS_TFS) {
				CSR_SET_4(sc, USBDC_UDCCS(epidx),
				    USBDC_UDCCS_TSP);
				/*
				 * if we send a zero packet, we are 'done', but
				 * dont to usbp_transfer_complete() just yet
				 * because the short packet will cause another
				 * interrupt.
				 */
				xfer->status = USBD_NORMAL_COMPLETION;
				return;
			} else  {
				aprint_error("%s: %s: fifo full when trying to set short packet\n",
					     DEVNAME(sc), __func__);
			}
		}
		xfer->status = USBD_NORMAL_COMPLETION;
#ifdef DEBUG_TX
		printf("%s: packet complete %x\n", __func__, xfer->actlen);
#endif
		usbp_transfer_complete(xfer);
		return;
	}

	p = (uint8_t *)xfer->buffer + xfer->actlen;


	csr = CSR_READ_4(sc, USBDC_UDCCS(epidx));
	csr_o = csr & (USBDC_UDCCS_TPC|USBDC_UDCCS_SST|USBDC_UDCCS_TUR);
	if (csr_o != 0)
		CSR_WRITE_4(sc, USBDC_UDCCS(epidx), csr_o);


	while (xfer->actlen < xfer->length &&
	       CSR_READ_4(sc, USBDC_UDCCS(epidx)) & USBDC_UDCCS_TFS) {
		int len = xfer->length - xfer->actlen;
		len = MIN(len, fifo_length);

		bus_space_write_multi_1(sc->sc_iot, sc->sc_ioh, datareg, p, len);

		p += len;
		xfer->actlen += len;

#ifdef	DEBUG_TX
		tlen += len;
#endif
	}

#ifdef DEBUG_TX
	printf(" wrote %d %d maxp=%d flags=%x cr=%x uicr0=%x uicr1=%x\n",
	    tlen, xfer->actlen, maxp, xfer->flags,
	    CSR_READ_4(sc, USBDC_UDCCR),
	    CSR_READ_4(sc, USBDC_UICR0),
	    CSR_READ_4(sc, USBDC_UICR1));
	if (xfer->actlen == 0) {
		printf("whoa, write_ep called, but no free space\n");
	}
#endif
	if (xfer->actlen >= xfer->length) {
		if ((xfer->actlen % maxp) != 0) {
			if (1 /*xfer->flags & USBD_FORCE_SHORT_XFER*/) {
				CSR_SET_4(sc, USBDC_UDCCS(epidx), USBDC_UDCCS_TSP);
#ifdef DEBUG_TX
				printf("setting short packet on %d csr=%x\n", epidx,
				    CSR_READ_4(sc, USBDC_UDCCS(epidx)));
#endif
			} else {
				/* fill buffer to maxpacket size??? */
			}
		}
	}
}

/*
 * Interrupt handling
 */

#if 0
int
pxa25xudc_connect_intr(void *v)
{
	struct pxaudc_softc *sc = v;

	/* XXX only set a flag here */
	if (pxaudc_is_host(sc)) {
		DPRINTF(10, ("%s:switching to host\n", sc->sc_bus.bdev.dv_xname));
		pxa25xudc_disable(sc);
	} else {
		DPRINTF(10, ("%s:switching to client\n", sc->sc_bus.bdev.dv_xname));
		pxa25xudc_enable(sc);
	}

	/* Claim this interrupt. */
	return 1;
}
#endif

static int
pxaudc_intr(void *v)
{
	struct pxaudc_softc *sc = v;
	u_int32_t isr0, isr1, cr;

	isr0 = CSR_READ_4(sc, USBDC_USIR0);
	isr1 = CSR_READ_4(sc, USBDC_USIR1);
	cr = CSR_READ_4(sc, USBDC_UDCCR);

	CSR_SET_4(sc, USBDC_UDCCR, cr &
	    (USBDC_UDCCR_RESIR|USBDC_UDCCR_SUSIR|USBDC_UDCCR_RSTIR));

	CSR_WRITE_4(sc, USBDC_USIR0, isr0);
	CSR_WRITE_4(sc, USBDC_USIR1, isr1);


	DPRINTF(10,("%s: pxaudc_intr: isr0=%x, isr1=%x cr=%x\n", DEVNAME(sc), isr0, isr1, cr));

	sc->sc_isr |= isr0 | (isr1 << 8);
	if (cr & USBDC_UDCCR_RESIR)
		sc->sc_isr |= INTR_RESUME;
	if (cr & USBDC_UDCCR_SUSIR)
		sc->sc_isr |= INTR_SUSPEND;
	if (cr & USBDC_UDCCR_RSTIR)
		sc->sc_isr |= INTR_RESET;

//	if (sc->sc_isr == 0)
//		break;

	//usbf_schedsoftintr(&sc->sc_bus);
	pxaudc_intr1(sc); /* XXX */

	DPRINTF(10, ("%s: end: uicr0=%x isr0=%x udccr=%x udccs0=%x\n", __func__,
	    CSR_READ_4(sc, USBDC_UICR0),
	    CSR_READ_4(sc, USBDC_USIR0),
	    CSR_READ_4(sc, USBDC_UDCCR),
		CSR_READ_4(sc, USBDC_UDCCS(0))	    ));	

	/* Claim this interrupt. */
	return 1;
}

static void
pxaudc_intr1(struct pxaudc_softc *sc)
{
	u_int32_t isr;
	int i;
	int s;


	s = splhardusb();
	isr = sc->sc_isr;
	sc->sc_isr = 0;
	sc->sc_bus.intr_context++;
	splx(s);


	DPRINTF(10, ("%s: isr=%x\n", __func__, isr));

	/* Handle USB RESET condition. */
	if (isr & INTR_RESET) {
		usbp_host_reset(&sc->sc_bus);
		/* Discard all other interrupts. */
		goto ret;
	}

	/* Service control pipe interrupts. */
	if (isr & (1<<0))
		pxaudc_ep0_intr(sc);

	for (i = 1; i < 16; i++) {
		if (isr & (1 << i))
			pxaudc_epN_intr(sc, i);
	}

	if (isr & INTR_SUSPEND) {
		/* suspend ?? */
		printf("%s: suspend\n", DEVNAME(sc));
	}

	if (isr & INTR_RESUME) {
		/* resume ?? */
		printf("%s: resume\n", DEVNAME(sc));
	}

ret:
	sc->sc_bus.intr_context--;
}

static void
pxaudc_epN_intr(struct pxaudc_softc *sc, int ep)
{
	struct usbd_pipe *pipe;
	struct usbd_endpoint *endpoint;
	int dir;

	DPRINTF(10, ("%s: ep%d intr ppipe=%p\n", __func__, ep, sc->sc_pipe[ep]));
	    
	pipe = sc->sc_pipe[ep];
	if (pipe == NULL) {
		if (udc_ep_regs[ep].count) {
			/* this is OUT endpoint and no pipe is opened. */
			DPRINTF(0, ("%s: ep%d interrupt while no pipe is opened.\n", __func__, ep));
			pxaudc_clear_epN(sc, ep);
		}
		return;
	}

	endpoint = pipe->endpoint;
	dir = usbd_endpoint_dir(endpoint);

	if (dir == UE_DIR_IN) {
		uint32_t r = CSR_READ_4(sc, USBDC_UDCCS(ep));
		DPRINTF(10, ("udccs%d=%x\n", ep, r));
		/* clear TPC bit */
		CSR_WRITE_4(sc, USBDC_UDCCS(ep), r & USBDC_UDCCS_TPC);
		pxaudc_write_epN(sc, ep);
	} else {
		pxaudc_read_epN(sc, ep);
	}

}

void
pxaudc_write_epN(struct pxaudc_softc *sc, int ep)
{
	struct usbd_pipe *pipe = NULL;
	struct usbd_xfer *xfer = NULL;

	DPRINTF(10, ("%s: write ep%d pipe=%p xfer=%p\n", __func__,
		ep, sc->sc_pipe[ep],
		sc->sc_pipe[ep] ? SIMPLEQ_FIRST(&sc->sc_pipe[ep]->queue) : NULL));

	pipe = sc->sc_pipe[ep];

	if (pipe == NULL) {
		return;
	}

	xfer = SIMPLEQ_FIRST(&pipe->queue);
	if (xfer != NULL)
		pxaudc_write(sc, xfer);

}

static void
pxaudc_ep0_intr(struct pxaudc_softc *sc)
{
	struct usbd_pipe *pipe = NULL;
	struct usbd_xfer *xfer = NULL;
	u_int32_t csr0;
	bool write_done = FALSE;

	csr0 = CSR_READ_4(sc, USBDC_UDCCS(0));
	DPRINTF(10,("%s: start: pxaudc_ep0_intr: csr0=%x ep0state=%d\n", DEVNAME(sc), csr0, sc->sc_bus.ep0state));
	delay (25);			/* ??? */

	pipe = sc->sc_pipe[0];


	if (csr0 & USBDC_UDCCS_SST)
		CSR_WRITE_4(sc, USBDC_UDCCS(0), USBDC_UDCCS_SST);

	if (sc->sc_bus.ep0state == EP0_IN_DATA_PHASE &&
	    (csr0 & USBDC_UDCCS0_IPR) == 0  && pipe != NULL) {

		xfer = SIMPLEQ_FIRST(&pipe->queue);
		if (xfer != NULL)
			pxaudc_write_ep0(sc, xfer);

		write_done = TRUE;

//		csr0 = CSR_READ_4(sc, USBDC_UDCCS(0));
		return;
	}
	

	if ((csr0 & (USBDC_UDCCS0_OPR|USBDC_UDCCS0_SA/*|USBDC_UDCCS_RNE*/)) ==
	    (USBDC_UDCCS0_OPR|USBDC_UDCCS0_SA/*|USBDC_UDCCS_RNE*/) ) {
		/* Setup */

		if (sc->sc_bus.ep0state != EP0_IDLE) {
			printf("%s: Setup stage restarted.\n",
			    DEVNAME(sc));
			/* XXX: remove remaining xfers? */
		}
		sc->sc_bus.ep0state = EP0_IDLE;

		if (pipe)
			xfer = SIMPLEQ_FIRST(&pipe->queue);

		if ((csr0 & USBDC_UDCCS_RNE) != 0) {
			pxaudc_read_ep0(sc, xfer);
		}
		else {
			/* Errata #131 */
			pxaudc_read_ep0_errata(sc, xfer);
		}
		return;
	}


	if ((sc->sc_bus.ep0state == EP0_END_XFER ||
		sc->sc_bus.ep0state == EP0_IN_DATA_PHASE) &&
	    (csr0 & (USBDC_UDCCS0_OPR|USBDC_UDCCS0_SA|USBDC_UDCCS_RNE)) ==
	    USBDC_UDCCS0_OPR) {


		/* STATUS OUT stage */
		printf("STATUS OUT??  csr0=%x ep0state=%d\n", csr0, sc->sc_bus.ep0state);

		if (sc->sc_bus.ep0state == EP0_IN_DATA_PHASE) {
			/* Premature Status Stage */
			/* XXX */
		}
		
		CSR_WRITE_4(sc, USBDC_UDCCS(0), USBDC_UDCCS0_OPR);

		sc->sc_bus.ep0state = EP0_IDLE;
		return;
	}
	    
	if (!write_done) 
		aprint_error("%s: %s: unexpected state: ep0state=%d csr0=%x\n",
		    DEVNAME(sc), __func__, sc->sc_bus.ep0state, csr0);
}

/*
 * Bus methods
 */

usbd_status
pxaudc_open(struct usbd_pipe *pipe)
{
	struct pxaudc_softc *sc = (struct pxaudc_softc *)pipe->device->bus;
	struct usbd_endpoint *endpoint = pipe->endpoint;
	int ep_idx;
	int s;

	ep_idx = usbd_endpoint_index(endpoint);
	if (ep_idx >= PXAUDC_NEP)
		return USBD_BAD_ADDRESS;

	DPRINTF(10,("pxaudc_open  ep%d sc=%p npipe=%d\n", ep_idx, sc, sc->sc_npipe));
	s = splhardusb();

	switch (usbd_endpoint_type(endpoint)) {
	case UE_CONTROL:
		pipe->methods = &pxaudc_ctrl_methods;
		break;

	case UE_BULK:
		pipe->methods = &pxaudc_bulk_methods;
		break;

	case UE_ISOCHRONOUS:
	case UE_INTERRUPT:
	default:
		/* XXX */
		splx(s);
		return USBD_BAD_ADDRESS;
	}
	
	sc->sc_pipe[ep_idx] = pipe;
	sc->sc_npipe++;

	
	if (sc->sc_enabled) {
		/* Enable interrupts for the endpoint */
		u_int icrreg = USBDC_UICR0;
		if (ep_idx >= 8)
			icrreg = USBDC_UICR1;

		CSR_CLR_4(sc, icrreg, (1 << (ep_idx % 8)));
	}

	splx(s);
	return USBD_NORMAL_COMPLETION;
}

void
pxaudc_softintr(void *v)
{
	struct pxaudc_softc *sc = v;

	pxaudc_intr1(sc);
}

usbd_status
pxaudc_allocm(struct usbd_bus *bus, usb_dma_t *dmap, u_int32_t size)
{
	return usb_allocmem(bus, size, 0, dmap);
}

void
pxaudc_freem(struct usbd_bus *bus, usb_dma_t *dmap)
{
	usb_freemem(bus, dmap);
}

struct usbd_xfer *
pxaudc_allocx(struct usbd_bus *_bus)
{
	struct pxaudc_softc *sc = _bus->hci_private;
	struct usbd_xfer *xfer;

	xfer = SIMPLEQ_FIRST(&sc->sc_free_xfers);
	if (xfer != NULL)
		SIMPLEQ_REMOVE_HEAD(&sc->sc_free_xfers, next);
	else
		xfer = malloc(sizeof(struct pxaudc_xfer), M_USB, M_NOWAIT);
	if (xfer != NULL)
		bzero(xfer, sizeof(struct pxaudc_xfer));
	return xfer;
}

void
pxaudc_freex(struct usbd_bus *bus, struct usbd_xfer *xfer)
{
	struct pxaudc_softc *sc = bus->hci_private;

	SIMPLEQ_INSERT_HEAD(&sc->sc_free_xfers, xfer, next);
}

/*
 * Control pipe methods
 */

usbd_status
pxaudc_ctrl_transfer(struct usbd_xfer *xfer)
{
	usbd_status err;
	struct pxaudc_xfer *pxfer = (struct pxaudc_xfer *)xfer;
	struct pxaudc_softc *sc = xfer->pipe->device->bus->hci_private;

	printf("%s xfer=%p sc=%p\n", __func__, xfer, sc);

	pxfer->frmlen = 0;

	/* Insert last in queue. */
#if 1
	err = usbp_insert_transfer(xfer);
#else
	mutex_enter(&sc->sc_lock);
	err = usb_insert_transfer(xfer);
	mutex_exit(&sc->sc_lock);
#endif
	if (err)
		return err;

	/*
	 * Pipe isn't running (otherwise err would be USBD_IN_PROGRESS),
	 * so start first.
	 */
	return pxaudc_ctrl_start(SIMPLEQ_FIRST(&xfer->pipe->queue));
}

usbd_status
pxaudc_ctrl_start(struct usbd_xfer *xfer)
{
	struct usbd_pipe *pipe = xfer->pipe;
	struct pxaudc_softc *sc = pipe->device->bus->hci_private;
	int iswrite = !(xfer->rqflags & URQ_REQUEST);
	int s;

	s = splusb();
	xfer->status = USBD_IN_PROGRESS;
	if (iswrite)
		pxaudc_write_ep0(sc, xfer);
	else {
		/* XXX boring message, this case is normally reached if
		 * XXX the xfer for a device request is being queued. */
		DPRINTF(10,("(%s): ep[%x] ctrl-out, xfer=%p, len=%u, "
			"actlen=%u\n", DEVNAME(sc),
			usbd_endpoint_address(pipe->endpoint),
			xfer, xfer->length,
			xfer->actlen));
	}
	splx(s);
	return USBD_IN_PROGRESS;
}

/* (also used by bulk pipes) */
void
pxaudc_ctrl_abort(struct usbd_xfer *xfer)
{
	int s;
#ifdef PXAUDC_DEBUG
	struct usbd_pipe *pipe = xfer->pipe;
	struct pxaudc_softc *sc = pipe->device->bus->hci_private;
	int index = usbd_endpoint_index(pipe->endpoint);
	int dir = usbd_endpoint_dir(pipe->endpoint);
	int type = usbd_endpoint_type(pipe->endpoint);
#endif

	DPRINTF(10,("%s: ep%d %s-%s abort, xfer=%p\n", DEVNAME(sc), index,
	    type == UE_CONTROL ? "ctrl" : "bulk", dir == UE_DIR_IN ?
	    "in" : "out", xfer));

	/*
	 * Step 1: Make soft interrupt routine and hardware ignore the xfer.
	 */
	s = splusb();
	xfer->status = USBD_CANCELLED;
	callout_stop(&xfer->timeout_handle);
	splx(s);

	/*
	 * Step 2: Make sure hardware has finished any possible use of the
	 * xfer and the soft interrupt routine has run.
	 */
	s = splusb();
	/* XXX this does not seem right, what if there
	 * XXX are two xfers in the FIFO and we only want to
	 * XXX ignore one? */
#ifdef notyet
	pxaudc_flush(sc, usbd_endpoint_address(pipe->endpoint));
#endif
	/* XXX we're not doing DMA and the soft interrupt routine does not
	   XXX need to clean up anything. */
	splx(s);

	/*
	 * Step 3: Execute callback.
	 */
	s = splusb();
	usbp_transfer_complete(xfer);
//	usbd_transfer_complete(xfer);
	splx(s);
}

void
pxaudc_ctrl_done(struct usbd_xfer *xfer)
{
}

void
pxaudc_ctrl_close(struct usbd_pipe *pipe)
{
}

/*
 * Bulk pipe methods
 */

usbd_status
pxaudc_bulk_transfer(struct usbd_xfer *xfer)
{
	usbd_status err;
	struct pxaudc_softc *sc = xfer->pipe->device->bus->hci_private;

	DPRINTF(5, ("%s: xfer=%p sc=%p\n", __func__, xfer, sc));

	/* Insert last in queue. */
#if 1
	err = usbp_insert_transfer(xfer);
#else
	mutex_enter(&sc->sc_lock);
	err = usb_insert_transfer(xfer);
	mutex_exit(&sc->sc_lock);
#endif
	if (err)
		return err;

	/*
	 * Pipe isn't running (otherwise err would be USBD_IN_PROGRESS),
	 * so start first.
	 */
	return pxaudc_bulk_start(SIMPLEQ_FIRST(&xfer->pipe->queue));
}

usbd_status
pxaudc_bulk_start(struct usbd_xfer *xfer)
{
	struct usbd_pipe *pipe = xfer->pipe;
	struct pxaudc_softc *sc = pipe->device->bus->hci_private;
	int iswrite = (usbd_endpoint_dir(pipe->endpoint) == UE_DIR_IN);
	int s;

	DPRINTF(0,("%s: %s: ep%d bulk-%s start, xfer=%p, len=%u\n", DEVNAME(sc), __func__,
	    usbd_endpoint_index(pipe->endpoint), iswrite ? "in" : "out",
	    xfer, xfer->length));

	s = splusb();
	xfer->status = USBD_IN_PROGRESS;
	if (iswrite)
		pxaudc_write(sc, xfer);
	else {
		/* enable interrupt */
	}
	splx(s);
	return USBD_IN_PROGRESS;
}

void
pxaudc_bulk_abort(struct usbd_xfer *xfer)
{
	pxaudc_ctrl_abort(xfer);
}

void
pxaudc_bulk_done(struct usbd_xfer *xfer)
{
#ifdef PXAUDC_DEBUG
	int ep = usbd_endpoint_index(xfer->pipe->endpoint);
	struct usbd_pipe *pipe = xfer->pipe;
	struct pxaudc_softc *sc = pipe->device->bus->hci_private;
	int iswrite = (usbd_endpoint_dir(pipe->endpoint) == UE_DIR_IN);

	DPRINTF(0,("%s: %s: ep%d bulk-%s done, xfer=%p, len=%u\n", DEVNAME(sc), __func__,
		ep, iswrite ? "in" : "out",
		xfer, xfer->length));
#endif
}

void
pxaudc_bulk_close(struct usbd_pipe *pipe)
{
	struct pxaudc_softc *sc = (struct pxaudc_softc *)pipe->device->bus;
	struct usbd_endpoint *endpoint = pipe->endpoint;
	int ep_idx;
	int s;
	u_int icrreg;
	
	ep_idx = usbd_endpoint_index(endpoint);
	DPRINTF(10,("pxaudc_bulk_close  ep%d sc=%p npipe=%d\n", ep_idx, sc, sc->sc_npipe));

	if (ep_idx >= PXAUDC_NEP)
		return;


	s = splhardusb();

	sc->sc_pipe[ep_idx] = NULL;
	sc->sc_npipe--;

	
	/* Mask interrupts for the endpoint */
	icrreg = USBDC_UICR0;
	if (ep_idx >= 8)
		icrreg = USBDC_UICR1;

	CSR_SET_4(sc, icrreg, (1 << (ep_idx % 8)));

	/* clear read buffer */
	if (udc_ep_regs[ep_idx].count != 0)
		CSR_SET_4(sc, USBDC_UDCCS(ep_idx), USBDC_UDCCS_RPC);

	splx(s);
}

#endif /* NUSBP > 0 */


/*
 * from Intel PXA255 Processor Developer's Manual.
 * EP0 Control Read
 *
 *  Host              Client                   Client driver
 *                                      {EP0_IDLE}
 *   SETUP command  -->
 *                       EP0 interrupt -->
 *                       UDCCS0[SA] = 1, UDCCS0[OPR] = 1
 *                                         read data from EP0 while
 *                                           UDCCS0[RNE] == 1
 * ---------------------  repeat { ------------------------------------------
 *                                      <-- load IN data
 *                                        {EP0_IN_DATA_PHASE}
 *                                        clear UDCCS0[SA and UDCCS0[OPR]
 *                                        set UDCCS0[IPR] if shor packet
 *                                        clear UDC EP0 inpurrupt
 *   IN packet      -->
 *                  <-- data
 *                  <-- ACK
 *                        UDCCS0[IPR] = 0
 *                            EP0 interrupt -->
 * ---------------------- } until (all data are transmitted) --------------
 *
 *                                     if the last packet was not short packet
 *                                     set UDCCS0[IPR] to send zero-length packet
 *                                      {EP0_END_XFER}
 *   STATUS OUT stage
 *     zero-length OUT  -->
 *                        EP0 interrupt -->
 *                        UDCCS0[OPR]=1 UDCCS0[SA]=0
 *                                         clear UDCCS0[OPR]
 *                                      {EP0_IDLE}
 *
 * * Premature Status Stage: Host may send zero-length OUT packet and
 *   enters to STATUS OUT stage before receiving all data from the client.
 */



static void
pxaudc_get_lock(struct usbd_bus *bus, kmutex_t **mutex)
{
	struct pxaudc_softc *sc = bus->hci_private;

	*mutex = &sc->sc_lock;
}

static usbd_status
pxaudc_new_device(device_t dev, usbd_bus_handle bus, int a,
					    int b, int c, struct usbd_port *port)
{
	printf("!!! new_device called\n");
	return USBD_NORMAL_COMPLETION;
}

static 	usbd_status
pxaudc_select_endpoint(struct usbp_bus *bus, struct usbp_endpoint_request *epreq, u_int map)
{
	static const uint8_t epspec[][4][4] = {
		/* OUT */
		{
			[UE_CONTROL] = {0},
			[UE_ISOCHRONOUS] = {4, 9, 14, 0},
			[UE_BULK] = {2, 7, 12, 0},
			[UE_INTERRUPT] = {0}
		},
		/* IN */
		{
			[UE_CONTROL] = {0},
			[UE_ISOCHRONOUS] = {3, 8, 13, 0},
			[UE_BULK] = {1, 6, 11, 0},
			[UE_INTERRUPT] = {5, 10, 15, 0}
		}
	};
	static const int type_to_packetsize [] = {
		[UE_CONTROL] = 16,
		[UE_ISOCHRONOUS] = 256,
		[UE_BULK] = 64,
		[UE_INTERRUPT] = 8
	};
	const uint8_t *bp;
	int dir = epreq->dir;
	int index = epreq->epnum;
	int xfrtype = UE_GET_XFERTYPE(epreq->attributes);
	int i;

	if ((dir != UE_DIR_OUT && dir != UE_DIR_IN) ||
	    (index < 0 || USB_MAX_ENDPOINTS <= index ))
		return USBD_BAD_ADDRESS;

	bp = epspec[dir == UE_DIR_OUT ? 0: 1][xfrtype];
	DPRINTF(0, ("%s: dir=%x xfrtype=%x index=%d bp[0]=%d\n", __func__,
		dir, xfrtype, index, bp[0]));
	if (index != 0) {
		/* Endpoint address is specified explicitly.
		   check if transfer type matches */
		for (i=0; bp[i]; ++i) {
			if (index != bp[i])
				continue;
			/*XXX check packetsize */
		}
		if (bp[i] == 0) {
			/* doesn't match */
			return USBD_BAD_ADDRESS;
		}
	}
	else {
		/* find a suitable endpoint */
		for (i=0; bp[i]; ++i) {
			DPRINTF(0, ("dir=%x xfrtype=%x i=%d bp[i]=%d\n", dir, xfrtype, i, bp[i]));
			if ((map & (1 << bp[i])) != 0)
				continue;
			/* XXX check packetsize */
			break;
		}
		if (bp[i] == 0)
			return USBD_NO_ADDR;
		index = bp[i];
	}

	epreq->epnum = index;
	epreq->packetsize = type_to_packetsize[xfrtype];
	return USBD_NORMAL_COMPLETION;
}


static usbd_status
pxaudc_enable_bus(struct usbp_bus *bus, bool on)
{
	struct pxaudc_softc *sc = bus->usbd.hci_private;


	if (on) {
		pxa25xudc_enable(sc);
	}
	else {
		pxa25xudc_disable(sc);
	}

	return USBD_NORMAL_COMPLETION;
}
