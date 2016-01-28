/*	$NetBSD: g42xxeb_mci.c,v 1.3 2012/01/21 19:44:28 nonaka Exp $ */

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
__KERNEL_RCSID(0, "$NetBSD: g42xxeb_mci.c,v 1.3 2012/01/21 19:44:28 nonaka Exp $");

#include <sys/param.h>
#include <sys/device.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/pmf.h>

#include <machine/intr.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usbp.h>

#include <arm/xscale/pxa2x0cpu.h>
#include <arm/xscale/pxa2x0reg.h>
#include <arm/xscale/pxa2x0var.h>
#include <arm/xscale/pxa2x0_gpio.h>
#include <arm/xscale/pxa2x0_udc.h>

#include <arm/xscale/pxa2x0var.h>
#include <evbarm/g42xxeb/g42xxeb_var.h>

#include "locators.h"

struct g42xxeb_udc_softc {
	struct pxaudc_softc sc_udc;

	bus_space_tag_t sc_obio_iot;
	bus_space_handle_t sc_obio_ioh;

	void *sc_detect_ih;
};

static int pxaudc_match(device_t, cfdata_t, void *);
static void pxaudc_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(pxaudc_obio, sizeof(struct g42xxeb_udc_softc),
    pxaudc_match, pxaudc_attach, NULL, NULL);

static int g42xxeb_udc_intr(void *arg);
static int g42xxeb_udc_is_connected(struct g42xxeb_udc_softc *sc);

static int
pxaudc_match(device_t parent, cfdata_t cf, void *aux)
{

	if (strcmp(cf->cf_name, "pxaudc") == 0)
		return 1;
	return 0;
}

struct pxa2x0_gpioconf g42xxeb_pxaudc_gpioconf[] = {
	{  21, GPIO_SET | GPIO_OUT },	/* USB_CONTROL */
#if 0
	{  6, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCLK */
	{  8, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCS0 */
	{  9, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCS1 */
#endif
	{  -1 }
};


static void
pxaudc_attach(device_t parent, device_t self, void *aux)
{
	struct g42xxeb_udc_softc *sc = device_private(self);
	struct obio_attach_args *oba = aux;
	struct obio_softc *osc = device_private(parent);
	struct pxaip_attach_args paa;
	struct pxa2x0_gpioconf *gpioconf[] = {
		g42xxeb_pxaudc_gpioconf,
		NULL
	};

	aprint_normal(": USB device controller\n");
	aprint_naive("\n");

	bus_space_tag_t iot = oba->oba_iot;

	pxa2x0_gpio_config(gpioconf);

	memset(&paa, 0, sizeof paa);
	paa.pxa_iot = iot;  /* oba and pxaip use the same bus_space */
	paa.pxa_dmat = &pxa2x0_bus_dma_tag;		/* XXX */
	if (oba->oba_addr == OBIOCF_ADDR_DEFAULT)
		paa.pxa_addr = PXA2X0_USBDC_BASE;
	else
		paa.pxa_addr = oba->oba_addr;
	paa.pxa_size = PXA250_USBDC_SIZE;
	paa.pxa_intr = 0;
				   
	if (pxaudc_attach_sub(self, &paa)) {
		aprint_error_dev(self,
		    "unable to attach UDC\n");
		return;
	}

	if (!pmf_device_register(self, NULL, NULL)) {
		aprint_error_dev(self,
		    "couldn't establish power handler\n");
	}

	sc->sc_obio_iot = iot;
	sc->sc_obio_ioh = osc->sc_obioreg_ioh;

	/* Establish card detect interrupt */
	sc->sc_detect_ih = obio_intr_establish(osc, G42XXEB_INT_USB,
	    IPL_BIO, IST_EDGE_BOTH, g42xxeb_udc_intr, sc);
	if (sc->sc_detect_ih == NULL) {
		aprint_error_dev(self,
		    "unable to establish SD detect interrupt\n");
		return;
	}

	return;

#if 0
free_intr:
	obio_intr_disestablish(osc, G42XXEB_INT_USB, sc->sc_detect_ih);
	sc->sc_detect_ih = NULL;
#endif
}

static int
g42xxeb_udc_intr(void *arg)
{
	struct g42xxeb_udc_softc *sc = (struct g42xxeb_udc_softc *)arg;
	int connected = g42xxeb_udc_is_connected(sc);

	printf("%s: %s\n", device_xname(sc->sc_udc.sc_dev),
	    connected ? "connected" : "not connected");

	return 1;
}

/*
 * Return non-zero if the card is currently inserted.
 */
static int
g42xxeb_udc_is_connected(struct g42xxeb_udc_softc *sc)
{
	uint16_t reg;

	reg = bus_space_read_2(sc->sc_obio_iot, sc->sc_obio_ioh,
	    G42XXEB_INTSTS2);

	return !(reg & (1<<G42XXEB_INT_USB));
}

