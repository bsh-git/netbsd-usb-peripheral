/*	$NetBSD$ */
/*	$OpenBSD: pxa27x_udc.h,v 1.4 2013/10/24 22:40:10 aalm Exp $	*/
/*
 * Copyright (c) 2009 Marek Vasut <marex@openbsd.org>
 *
 * Moved from pxa27x_udc.c:
 *
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

//#include <arm/xscale/pxa2x0reg.h>

/*
 * following files should be included before this file.
 * <dev/usb/usb.h>
 * <dev/usb/usbdi.h>
 * <dev/usb/usbdivar.h>
 * <dev/usb/usbp.h>
 */

#define PXAUDC_EP0MAXP	16	/* XXX */
#ifdef	CPU_XSCALE_PXA270
#define PXAUDC_NEP	24	/* total number of endpoints */
#else
#define PXAUDC_NEP	16	/* total number of endpoints */
#endif

struct pxaudc_softc {
	struct usbp_bus		 sc_bus;
	device_t    		 sc_dev;
	bus_space_tag_t		 sc_iot;
	bus_space_handle_t	 sc_ioh;
	bus_size_t		 sc_size;
	void			*sc_ih;
	void			*sc_conn_ih;
	SIMPLEQ_HEAD(,usbd_xfer) sc_free_xfers;	/* recycled xfers */
	u_int32_t		 sc_icr0;	/* enabled EP interrupts */
	u_int32_t		 sc_icr1;	/* enabled EP interrupts */
#if 0
	enum {
		EP0_SETUP,
		EP0_IN
	}			 sc_ep0state;
#endif
//	struct pxaudc_pipe	*sc_pipe[PXAUDC_NEP];
	usbd_pipe_handle	sc_pipe[PXAUDC_NEP];
	int			 sc_npipe;

#ifdef	CPU_XSCALE_PXA250
	unsigned int	sc_isr;		/* 0..15: endpoint interrupts */

#define		INTR_RESUME	(1<<25)
#define	  	INTR_SUSPEND	(1<<26)
#define  	INTR_RESET	(1<<27)
#endif
#ifdef	CPU_XSCALE_PXA270
	u_int32_t		 sc_isr0;	/* XXX deferred interrupts */
	u_int32_t		 sc_isr1;	/* XXX deferred interrupts */
	u_int32_t		 sc_otgisr;	/* XXX deferred interrupts */

	int			 sc_cn;
	int			 sc_in;
	int			 sc_isn;
	int8_t			 sc_ep_map[16];
#endif

#if 0
	int			sc_gpio_detect;
	int			sc_gpio_detect_inv;

	int			sc_gpio_pullup;
	int			sc_gpio_pullup_inv;
#endif

#if defined(CPU_XSCALE_PXA270)
	int			(*sc_is_host)(void);
#define	pxaudc_is_host(sc)	(sc->sc_is_host())	
#else
#define	pxaudc_is_host(sc)	0
#endif

	callout_t	sc_callout;
	kmutex_t	sc_lock;
	bool		sc_enabled;

	struct usbp_bus_methods usbp_bus_methods;
};

#if 0
int		 pxaudc_match(void);
void		 pxaudc_attach(struct pxaudc_softc *, void *);
int		 pxaudc_detach(struct pxaudc_softc *, int);
int		 pxaudc_activate(struct pxaudc_softc *, int);
#endif

/* for other attachment than to pxaip */
int pxaudc_attach_sub(device_t, struct pxaip_attach_args *, const struct usbp_bus_methods *);
