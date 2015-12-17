/*	$NetBSD$ */
/* Copyright (c) 2015 Hiroyuki Bessho <bsh@netbsd.org>
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
#include <dev/usb/usbf.h>
#include <dev/usb/usbfvar.h>

#include <arm/xscale/pxa2x0reg.h>
#include <arm/xscale/pxa2x0var.h>
#include <arm/xscale/pxa2x0_gpio.h>

#include <arm/xscale/pxa2x0_udc.h>

#include "usbf.h"

//#define	DEBUG_RX
//#define	DEBUG_TX
//#define PXAUDC_DEBUG

struct pxaudc_xfer {
	struct usbf_xfer	 xfer;
	u_int16_t		 frmlen;
};

struct pxaudc_pipe {
	struct usbf_pipe	 pipe;
//	LIST_ENTRY(pxaudc_pipe)	 list;
};

void		 pxa25xudc_enable(struct pxaudc_softc *);
void		 pxa25xudc_disable(struct pxaudc_softc *);

void		 pxa25xudc_intr1(struct pxaudc_softc *);
void		 pxa25xudc_ep0_intr(struct pxaudc_softc *);
void		 pxa25xudc_epN_intr(struct pxaudc_softc *sc, int ep, int isr);

usbf_status	 pxaudc_open(struct usbf_pipe *);
void		 pxaudc_softintr(void *);
usbf_status	 pxaudc_allocm(struct usbf_bus *, usb_dma_t *, u_int32_t);
void		 pxaudc_freem(struct usbf_bus *, usb_dma_t *);
struct usbf_xfer *pxaudc_allocx(struct usbf_bus *);
void		 pxaudc_freex(struct usbf_bus *, struct usbf_xfer *);

usbf_status	 pxaudc_ctrl_transfer(struct usbf_xfer *);
usbf_status	 pxaudc_ctrl_start(struct usbf_xfer *);
void		 pxaudc_ctrl_abort(struct usbf_xfer *);
void		 pxaudc_ctrl_done(struct usbf_xfer *);
void		 pxaudc_ctrl_close(struct usbf_pipe *);

usbf_status	 pxaudc_bulk_transfer(struct usbf_xfer *);
usbf_status	 pxaudc_bulk_start(struct usbf_xfer *);
void		 pxaudc_bulk_abort(struct usbf_xfer *);
void		 pxaudc_bulk_done(struct usbf_xfer *);
void		 pxaudc_bulk_close(struct usbf_pipe *);


static void pxaudc_read_ep0(struct pxaudc_softc *, struct usbf_xfer *);
static void pxaudc_read_epN(struct pxaudc_softc *, int);
static void pxaudc_write_ep0(struct pxaudc_softc *, struct usbf_xfer *);
static void pxaudc_write(struct pxaudc_softc *, struct usbf_xfer *);
static void pxaudc_write_epN(struct pxaudc_softc *sc, int ep);
static int pxaudc_intr(void *);
static void pxaudc_intr1(struct pxaudc_softc *);
static void pxaudc_ep0_intr(struct pxaudc_softc *);
static void pxaudc_epN_intr(struct pxaudc_softc *, int);


#if NUSBF > 0

struct usbf_bus_methods pxaudc_bus_methods = {
	pxaudc_open,
	pxaudc_softintr,
	pxaudc_allocm,
	pxaudc_freem,
	pxaudc_allocx,
	pxaudc_freex
};

struct usbf_pipe_methods pxaudc_ctrl_methods = {
	pxaudc_ctrl_transfer,
	pxaudc_ctrl_start,
	pxaudc_ctrl_abort,
	pxaudc_ctrl_done,
	pxaudc_ctrl_close
};

struct usbf_pipe_methods pxaudc_bulk_methods = {
	pxaudc_bulk_transfer,
	pxaudc_bulk_start,
	pxaudc_bulk_abort,
	pxaudc_bulk_done,
	pxaudc_bulk_close
};

#endif /* NUSBF > 0 */

#define DEVNAME(sc)	device_xname((sc)->sc_bus.bdev)

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
pxaudc_attach_sub(device_t self, struct pxaip_attach_args *pxa)
{
	struct pxaudc_softc *sc = device_private(self);
	struct usbdev_attach_args uaa;

	sc->sc_bus.bdev = self;
	sc->sc_iot = pxa->pxa_iot;

	if (bus_space_map(sc->sc_iot, pxa->pxa_addr, pxa->pxa_size, 0,
	    &sc->sc_ioh)) {
		printf(": cannot map mem space\n");
		return -1;
	}
	sc->sc_size = pxa->pxa_size;

	bus_space_barrier(sc->sc_iot, sc->sc_ioh, 0, sc->sc_size,
	    BUS_SPACE_BARRIER_READ|BUS_SPACE_BARRIER_WRITE);

	callout_init(&sc->callout, 0);

	/* Set up GPIO pins and disable the controller. */
	pxa25xudc_disable(sc);


#if NUSBF > 0
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
	sc->sc_bus.methods = &pxaudc_bus_methods;
	sc->sc_bus.pipe_size = sizeof(struct pxaudc_pipe);
	sc->sc_bus.ep0_maxp = PXAUDC_EP0MAXP;
	sc->sc_bus.usbrev = USBREV_1_1;
	sc->sc_bus.dmatag = pxa->pxa_dmat;
	sc->sc_npipe = 0;	/* ep0 is always there. */

	uaa.uaa_busname = "usbdev";
	uaa.uaa_bus = &sc->sc_bus;

	/* Attach logical device and function. */
	(void)config_found(self, &uaa, NULL);

	/* Enable the controller unless we're now acting as a host. */
	if (!pxaudc_is_host(sc))
		pxa25xudc_enable(sc);

#endif	/* NUSBF > 0 */

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
			struct usbf_endpoint *ep =
			    sc->sc_pipe[i]->pipe.endpoint;
			int dir = usbf_endpoint_dir(ep);

			icr &= ~(1 << i);

			/* clear old status, flush TX Fifo */
			if (dir == UE_DIR_IN) 
				CSR_WRITE_4(sc, USBDC_UDCCS(i),
					    USBDC_UDCCS_TUR|USBDC_UDCCS_TPC|
					    USBDC_UDCCS_SST|USBDC_UDCCS_FTF);
			else
				CSR_WRITE_4(sc, USBDC_UDCCS(i),
					    USBDC_UDCCS_RPC|USBDC_UDCCS_SST);

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

	DPRINTF(10, ("%s: UDCCR=%x\n", __func__, CSR_READ_4(sc, USBDC_UDCCR)));
}

#if NUSBF > 0

/*
 * Endpoint FIFO handling
 */

static void
pxaudc_read_ep0(struct pxaudc_softc *sc, struct usbf_xfer *xfer)
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
		xfer->status = USBF_NORMAL_COMPLETION;
		usbf_transfer_complete(xfer);
	}

	printf("read_ep0 done. count=%d (%p)\n", count, xfer);

}

static void
pxaudc_read_ep0_errata(struct pxaudc_softc *sc, struct usbf_xfer *xfer)
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
		xfer->status = USBF_NORMAL_COMPLETION;
		usbf_transfer_complete(xfer);
	}

	printf("read_ep0_errata done. (%p)\n", xfer);

}

static void
pxaudc_read_epN(struct pxaudc_softc *sc, int ep)
{
	size_t len, tlen;
	u_int8_t *p;
	struct pxaudc_pipe *ppipe;
	struct usbf_pipe *pipe = NULL;
	struct usbf_xfer *xfer = NULL;
	int count;
	u_int32_t csr;
	uint32_t datareg = udc_ep_regs[ep].data, countreg = udc_ep_regs[ep].count;

	DPRINTF(10, ("read_ep%d ppipe=%p\n", ep, sc->sc_pipe[ep]));
	if (countreg == 0) {
		aprint_error("%s: %s: EP%d is not OUT endpoint\n",
			     DEVNAME(sc), __func__, ep);
		return;
	}

	ppipe = sc->sc_pipe[ep];

	if (ppipe == NULL) {
		return;
	}
	pipe = &ppipe->pipe;


	do {

		xfer = SIMPLEQ_FIRST(&pipe->queue);

		if (xfer == NULL) {
			printf("pxaudc_read_epN: ep %d, no xfer\n", ep);
			return;
		}

		csr = CSR_READ_4(sc, USBDC_UDCCS(ep));

		if ((csr & (USBDC_UDCCS_RNE|USBDC_UDCCS_RSP)) == USBDC_UDCCS_RSP) {
#ifdef DEBUG_RX
			printf("trans1 complete (zero-length packet)\n");
#endif
			xfer->status = USBF_NORMAL_COMPLETION;
			usbf_transfer_complete(xfer);
			CSR_SET_4(sc, USBDC_UDCCS(ep), USBDC_UDCCS_RPC);
			return;
		}

		if ((csr & USBDC_UDCCS_RNE) == 0) {
			/* no data in FIFO */
			return;
		}
		
		count = CSR_READ_4(sc, countreg) + 1;
		tlen = len = MIN(count, xfer->length - xfer->actlen);
		p = (uint8_t *)(xfer->buffer) + xfer->actlen;

#ifdef DEBUG_RX
		printf("reading data from endpoint %x, len %x csr %x\n",
		       ep, count, csr);
#endif

	
		bus_space_read_multi_1(sc->sc_iot, sc->sc_ioh, datareg, p, count);
		CSR_SET_4(sc, USBDC_UDCCS(ep), USBDC_UDCCS_RPC);
		xfer->actlen += count;

		if (xfer->length == xfer->actlen || (tlen == 0 && xfer->actlen != 0) 
		    || (csr & USBDC_UDCCS_RSP) ) {
#ifdef DEBUG_RX
			printf("trans2 complete\n");
#endif
			xfer->status = USBF_NORMAL_COMPLETION;
			usbf_transfer_complete(xfer);
		}
		csr = CSR_READ_4(sc, USBDC_UDCCS(ep));
#ifdef DEBUG_RX
		printf("csr now %x len %x\n",
		       csr, CSR_READ_4(sc, countreg));
#endif
	} while (csr & USBDC_UDCCS_RFS);
}

void
pxaudc_write_ep0(struct pxaudc_softc *sc, struct usbf_xfer *xfer)
{
	struct pxaudc_xfer *lxfer = (struct pxaudc_xfer *)xfer;
	u_int32_t len;
	u_int8_t *p;

	DPRINTF(10,("%s: %s: xfer=%s frmlen=%u\n",
		DEVNAME(sc), __func__, usbf_describe_xfer(xfer), lxfer->frmlen));

	if (lxfer->frmlen > 0) {
		xfer->actlen += lxfer->frmlen;
		lxfer->frmlen = 0;
	}


	if (xfer->actlen >= xfer->length) {
		sc->sc_bus.ep0state = EP0_END_XFER;
		printf("write_ep0 done\n");
		usbf_transfer_complete(xfer);
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
pxaudc_write(struct pxaudc_softc *sc, struct usbf_xfer *xfer)
{
	u_int8_t *p;
	int ep = usbf_endpoint_index(xfer->pipe->endpoint);
	int maxp = UGETW(xfer->pipe->endpoint->edesc->wMaxPacketSize);
	int fifo_length = udc_ep_regs[ep].fifo_length;
	u_int32_t csr, csr_o;
	bus_size_t datareg = udc_ep_regs[ep].data;
#ifdef DEBUG_TX
	int tlen = 0;
#endif

	fifo_length = MIN(fifo_length, maxp);

#ifdef DEBUG_TX_PKT
	if (xfer->actlen == 0)
		printf("new packet len %x\n", xfer->length);
#endif

#ifdef DEBUG_TX
	printf("writing data to endpoint %x, xlen %x xact %x\n",
		ep, xfer->length, xfer->actlen);
#endif


	if (xfer->actlen == xfer->length) {
		/*
		 * If the packet size is wMaxPacketSize byte multiple
		 * send a zero packet to indicate termiation.
		 */
		if ((xfer->actlen % maxp) == 0 &&
		    xfer->status != USBF_NORMAL_COMPLETION &&
		    xfer->flags & USBD_FORCE_SHORT_XFER) {
			if (CSR_READ_4(sc, USBDC_UDCCS(ep))
			    & USBDC_UDCCS_TFS) {
				CSR_SET_4(sc, USBDC_UDCCS(ep),
				    USBDC_UDCCS_TSP);
				/*
				 * if we send a zero packet, we are 'done', but
				 * dont to usbf_transfer_complete() just yet
				 * because the short packet will cause another
				 * interrupt.
				 */
				xfer->status = USBF_NORMAL_COMPLETION;
				return;
			} else  {
				aprint_error("%s: %s: fifo full when trying to set short packet\n",
					     DEVNAME(sc), __func__);
			}
		}
		xfer->status = USBF_NORMAL_COMPLETION;
#ifdef DEBUG_TX_PKT
		printf("packet complete %x\n", xfer->actlen);
#endif
		usbf_transfer_complete(xfer);
		return;
	}

	p = (uint8_t *)xfer->buffer + xfer->actlen;


	csr = CSR_READ_4(sc, USBDC_UDCCS(ep));
	csr_o = csr & (USBDC_UDCCS_TPC|USBDC_UDCCS_SST|USBDC_UDCCS_TUR);
	if (csr_o != 0)
		CSR_WRITE_4(sc, USBDC_UDCCS(ep), csr_o);


	while (xfer->actlen < xfer->length &&
	       CSR_READ_4(sc, USBDC_UDCCS(ep)) & USBDC_UDCCS_TFS) {
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
	printf(" wrote %d %d\n", tlen, xfer->actlen);
	if (xfer->actlen == 0) {
		printf("whoa, write_ep called, but no free space\n");
	}
#endif
	if (xfer->actlen >= xfer->length) {
		if ((xfer->actlen % maxp) != 0) {
			if (xfer->flags & USBD_FORCE_SHORT_XFER) {
				CSR_SET_4(sc, USBDC_UDCCS(ep), USBDC_UDCCS_TSP);
#ifdef DEBUG_TX
				printf("setting short packet on %d csr=%x\n", ep,
				    CSR_READ_4(sc, USBDC_UDCCS(ep)));
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

//u_int32_t csr1, csr2;

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


	/* Handle USB RESET condition. */
	if (isr & INTR_RESET) {
		usbf_host_reset(&sc->sc_bus);
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
	struct pxaudc_pipe *ppipe;
	struct usbf_pipe *pipe;
	int dir;

	DPRINTF(10, ("ep%d intr ppipe=%p\n", ep, sc->sc_pipe[ep]));
	    
	/* faster method of determining direction? */
	ppipe = sc->sc_pipe[ep];

	if (ppipe == NULL)
		return;
	pipe = &ppipe->pipe;
	dir = usbf_endpoint_dir(pipe->endpoint);

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
	struct pxaudc_pipe *ppipe;
	struct usbf_pipe *pipe = NULL;
	struct usbf_xfer *xfer = NULL;

	DPRINTF(10, ("write ep%d ppipe=%p\n", ep, sc->sc_pipe[ep]));

	ppipe = sc->sc_pipe[ep];

	if (ppipe == NULL) {
		return;
	}
	pipe = &ppipe->pipe;
	xfer = SIMPLEQ_FIRST(&pipe->queue);
	if (xfer != NULL)
		pxaudc_write(sc, xfer);

}

static void
pxaudc_ep0_intr(struct pxaudc_softc *sc)
{
	struct pxaudc_pipe *ppipe;
	struct usbf_pipe *pipe = NULL;
	struct usbf_xfer *xfer = NULL;
	u_int32_t csr0;
	bool write_done = FALSE;

	csr0 = CSR_READ_4(sc, USBDC_UDCCS(0));
	DPRINTF(10,("%s: start: pxaudc_ep0_intr: csr0=%x ep0state=%d\n", DEVNAME(sc), csr0, sc->sc_bus.ep0state));
	delay (25);			/* ??? */

	ppipe = sc->sc_pipe[0];
	if (ppipe != NULL) {
		pipe = &ppipe->pipe;
	}


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
		printf("STATUS OUT??  csr0=%x\n", csr0);

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

usbf_status
pxaudc_open(struct usbf_pipe *pipe)
{
	struct pxaudc_softc *sc = (struct pxaudc_softc *)pipe->device->bus;
	struct pxaudc_pipe *ppipe = (struct pxaudc_pipe *)pipe;
	int ep_idx;
	int s;

	ep_idx = usbf_endpoint_index(pipe->endpoint);
	if (ep_idx >= PXAUDC_NEP)
		return USBF_BAD_ADDRESS;

	DPRINTF(10,("pxaudc_open  ep%d sc=%p npipe=%d\n", ep_idx, sc, sc->sc_npipe));
	s = splhardusb();

	switch (usbf_endpoint_type(pipe->endpoint)) {
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
		return USBF_BAD_ADDRESS;
	}
	
	sc->sc_pipe[ep_idx] = ppipe;
	sc->sc_npipe++;

	splx(s);
	return USBF_NORMAL_COMPLETION;
}

void
pxaudc_softintr(void *v)
{
	struct pxaudc_softc *sc = v;

	pxaudc_intr1(sc);
}

usbf_status
pxaudc_allocm(struct usbf_bus *bus, usb_dma_t *dmap, u_int32_t size)
{
	return usbf_allocmem(bus, size, 0, dmap);
}

void
pxaudc_freem(struct usbf_bus *bus, usb_dma_t *dmap)
{
	usbf_freemem(bus, dmap);
}

struct usbf_xfer *
pxaudc_allocx(struct usbf_bus *bus)
{
	struct pxaudc_softc *sc = (struct pxaudc_softc *)bus;
	struct usbf_xfer *xfer;

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
pxaudc_freex(struct usbf_bus *bus, struct usbf_xfer *xfer)
{
	struct pxaudc_softc *sc = (struct pxaudc_softc *)bus;

	SIMPLEQ_INSERT_HEAD(&sc->sc_free_xfers, xfer, next);
}

/*
 * Control pipe methods
 */

usbf_status
pxaudc_ctrl_transfer(struct usbf_xfer *xfer)
{
	usbf_status err;
	struct pxaudc_xfer *pxfer = (struct pxaudc_xfer *)xfer;

	printf("%s xfer=%p\n", __func__, xfer);

	pxfer->frmlen = 0;

	/* Insert last in queue. */
	err = usbf_insert_transfer(xfer);
	if (err)
		return err;

	/*
	 * Pipe isn't running (otherwise err would be USBF_IN_PROGRESS),
	 * so start first.
	 */
	return pxaudc_ctrl_start(SIMPLEQ_FIRST(&xfer->pipe->queue));
}

usbf_status
pxaudc_ctrl_start(struct usbf_xfer *xfer)
{
	struct usbf_pipe *pipe = xfer->pipe;
	struct pxaudc_softc *sc = (struct pxaudc_softc *)pipe->device->bus;
	int iswrite = !(xfer->rqflags & URQ_REQUEST);
	int s;

	s = splusb();
	xfer->status = USBF_IN_PROGRESS;
	if (iswrite)
		pxaudc_write_ep0(sc, xfer);
	else {
		/* XXX boring message, this case is normally reached if
		 * XXX the xfer for a device request is being queued. */
		DPRINTF(10,("(%s): ep[%x] ctrl-out, xfer=%p, len=%u, "
		    "actlen=%u\n", DEVNAME(sc),
		    usbf_endpoint_address(xfer->pipe->endpoint),
		    xfer, xfer->length,
		    xfer->actlen));
	}
	splx(s);
	return USBF_IN_PROGRESS;
}

/* (also used by bulk pipes) */
void
pxaudc_ctrl_abort(struct usbf_xfer *xfer)
{
	int s;
#ifdef PXAUDC_DEBUG
	struct usbf_pipe *pipe = xfer->pipe;
	struct pxaudc_softc *sc = (struct pxaudc_softc *)pipe->device->bus;
	int index = usbf_endpoint_index(pipe->endpoint);
	int dir = usbf_endpoint_dir(pipe->endpoint);
	int type = usbf_endpoint_type(pipe->endpoint);
#endif

	DPRINTF(10,("%s: ep%d %s-%s abort, xfer=%p\n", DEVNAME(sc), index,
	    type == UE_CONTROL ? "ctrl" : "bulk", dir == UE_DIR_IN ?
	    "in" : "out", xfer));

	/*
	 * Step 1: Make soft interrupt routine and hardware ignore the xfer.
	 */
	s = splusb();
	xfer->status = USBF_CANCELLED;
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
	pxaudc_flush(sc, usbf_endpoint_address(pipe->endpoint));
#endif
	/* XXX we're not doing DMA and the soft interrupt routine does not
	   XXX need to clean up anything. */
	splx(s);

	/*
	 * Step 3: Execute callback.
	 */
	s = splusb();
	usbf_transfer_complete(xfer);
	splx(s);
}

void
pxaudc_ctrl_done(struct usbf_xfer *xfer)
{
}

void
pxaudc_ctrl_close(struct usbf_pipe *pipe)
{
	/* XXX */
}

/*
 * Bulk pipe methods
 */

usbf_status
pxaudc_bulk_transfer(struct usbf_xfer *xfer)
{
	usbf_status err;

	/* Insert last in queue. */
	err = usbf_insert_transfer(xfer);
	if (err)
		return err;

	/*
	 * Pipe isn't running (otherwise err would be USBF_IN_PROGRESS),
	 * so start first.
	 */
	return pxaudc_bulk_start(SIMPLEQ_FIRST(&xfer->pipe->queue));
}

usbf_status
pxaudc_bulk_start(struct usbf_xfer *xfer)
{
	struct usbf_pipe *pipe = xfer->pipe;
	struct pxaudc_softc *sc = (struct pxaudc_softc *)pipe->device->bus;
	int iswrite = (usbf_endpoint_dir(pipe->endpoint) == UE_DIR_IN);
	int s;

	DPRINTF(0,("%s: ep%d bulk-%s start, xfer=%p, len=%u\n", DEVNAME(sc),
	    usbf_endpoint_index(pipe->endpoint), iswrite ? "in" : "out",
	    xfer, xfer->length));

	s = splusb();
	xfer->status = USBF_IN_PROGRESS;
	if (iswrite)
		pxaudc_write(sc, xfer);
	else {
		/* enable interrupt */
	}
	splx(s);
	return USBF_IN_PROGRESS;
}

void
pxaudc_bulk_abort(struct usbf_xfer *xfer)
{
	pxaudc_ctrl_abort(xfer);
}

void
pxaudc_bulk_done(struct usbf_xfer *xfer)
{
#ifdef PXAUDC_DEBUG
	int ep = usbf_endpoint_index(xfer->pipe->endpoint);
	struct usbf_pipe *pipe = xfer->pipe;
	struct pxaudc_softc *sc = (struct pxaudc_softc *)pipe->device->bus;
	int iswrite = (usbf_endpoint_dir(pipe->endpoint) == UE_DIR_IN);

	DPRINTF(0,("%s: ep%d bulk-%s start, xfer=%p, len=%u\n", DEVNAME(sc),
		ep, iswrite ? "in" : "out",
		xfer, xfer->length));
#endif
}

void
pxaudc_bulk_close(struct usbf_pipe *pipe)
{
	/* XXX */
}

#endif /* NUSBF > 0 */


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
