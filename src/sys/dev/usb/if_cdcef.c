/*	$OpenBSD: if_cdcef.c,v 1.37 2015/06/24 09:40:54 mpi Exp $	*/

/*
 * Copyright (c) 2007 Dale Rahn <drahn@openbsd.org>
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
 * USB Communication Device Class Ethernet Emulation Model function driver
 * (counterpart of the host-side cdce(4) driver)
 */
#ifndef	__NetBSD__
#include <bpfilter.h>
#endif

#include <sys/param.h>
#include <sys/device.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/mbuf.h>

#ifndef	__NetBSD__
#include <sys/timeout.h>
#else
#include <net/if_ether.h>
#include <netinet/if_inarp.h>	/* arp_ifinit */
#endif

#include <net/if.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usbp.h>
#include <dev/usb/usbcdc.h>

#if NBPFILTER > 0
#include <net/bpf.h>
#endif

#include <netinet/in.h>
#include <net/if_ether.h>


#define CDCEF_VENDOR_ID		0x0001
#define CDCEF_PRODUCT_ID	0x0001
#define CDCEF_DEVICE_CODE	0x0100		/* BCD Device release number */
#define CDCEF_VENDOR_STRING	"OpenBSD.org"
#define CDCEF_PRODUCT_STRING	"CDC Ethernet Emulation"
#define CDCEF_SERIAL_STRING	"1.00"

#define CDCEF_BUFSZ		1600


struct cdcef_softc {
	struct usbp_function	sc_dev;
	struct usbp_config	*sc_config;
	struct usbp_interface	*sc_iface;
	struct usbp_endpoint	*sc_ep_in;
	struct usbp_endpoint	*sc_ep_out;
	struct usbd_pipe	*sc_pipe_in;
	struct usbd_pipe	*sc_pipe_out;
	struct usbd_xfer	*sc_xfer_in;
	struct usbd_xfer	*sc_xfer_out;
	void			*sc_buffer_in;
	void			*sc_buffer_out;

//XXX	struct timeout		start_to;

	struct mbuf		*sc_xmit_mbuf;

#ifdef	__NetBSD__
	struct ethercom  	sc_ethcom;
#define GET_IFP(sc) (&(sc)->sc_ethcom.ec_if)
	uint8_t 	sc_ether_addr[ETHER_ADDR_LEN];	/* ?? */
#else
	struct arpcom           sc_arpcom;
#define GET_IFP(sc) (&(sc)->sc_arpcom.ac_if)
#endif

	int			sc_rxeof_errors;
	int			sc_listening;
};

int		cdcef_match(device_t, cfdata_t, void *);
void		cdcef_attach(device_t, device_t, void *);

usbd_status	cdcef_do_request(struct usbp_function *,
				 usb_device_request_t *, void **);

void		cdcef_start(struct ifnet *);

void		cdcef_txeof(struct usbd_xfer *, void *,
			    usbd_status);
void		cdcef_rxeof(struct usbd_xfer *, void *,
			    usbd_status);
int		cdcef_ioctl(struct ifnet *ifp, u_long command, void *data);
void		cdcef_watchdog(struct ifnet *ifp);
void		cdcef_stop(struct cdcef_softc *);
int		cdcef_encap(struct cdcef_softc *sc, struct mbuf *m, int idx);
struct mbuf *	cdcef_newbuf(void);
void		cdcef_start_timeout (void *);

static int cdcef_init(struct ifnet *);


CFATTACH_DECL_NEW(cdcef, sizeof (struct cdcef_softc),
    cdcef_match, cdcef_attach, NULL, NULL);

struct usbp_function_methods cdcef_methods = {
	NULL,			/* set_config */
	cdcef_do_request
};

#ifndef CDCEF_DEBUG
#define DPRINTF(l,x)	do {} while (0)
#else
int cdcefdebug = 9;
#define DPRINTF(l,x)	if ((l) <= cdcefdebug) printf x; else 
#endif

#define DEVNAME(sc)	device_xname((sc)->sc_dev.dev)

extern int ticks;

/*
 * USB function match/attach/detach
 */

int
cdcef_match(device_t parent, cfdata_t cf, void *aux)
{
	return UMATCH_GENERIC;
}

void
cdcef_attach(device_t parent, device_t self, void *aux)
{
	struct cdcef_softc *sc = device_private(self);
	struct usbp_function_attach_args *uaa = aux;
	struct usbp_device *dev = uaa->device;
	struct ifnet *ifp;
	usbd_status err;
	usb_cdc_union_descriptor_t udesc;
	int s;
	u_int16_t macaddr_hi;


	DPRINTF(10, ("%s\n", __func__));
	
	/* Set the device identification according to the function. */
	usbp_devinfo_setup(dev, UDCLASS_IN_INTERFACE, 0, 0, CDCEF_VENDOR_ID,
	    CDCEF_PRODUCT_ID, CDCEF_DEVICE_CODE, CDCEF_VENDOR_STRING,
	    CDCEF_PRODUCT_STRING, CDCEF_SERIAL_STRING);

	/* Fill in the fields needed by the parent device. */
	sc->sc_dev.dev = self;
	sc->sc_dev.methods = &cdcef_methods;

	/* timeout to start delayed transfers */
//XXX	timeout_set(&sc->start_to, cdcef_start_timeout, sc);

	/*
	 * Build descriptors according to the device class specification.
	 */
	err = usbp_add_config(dev, &sc->sc_config);
	if (err) {
		printf(": usbp_add_config failed\n");
		return;
	}

	err = usbp_add_interface(sc->sc_config, UICLASS_CDC,
	    UISUBCLASS_ETHERNET_NETWORKING_CONTROL_MODEL, 0, NULL,
	    &sc->sc_iface);
	if (err) {
		printf(": usbp_add_interface failed\n");
		return;
	}

	/* XXX don't use hard-coded values 128 and 16. */
	err = usbp_add_endpoint(sc->sc_iface, UE_DIR_IN | 1, UE_BULK,
	    64, 16, &sc->sc_ep_in) ||
	    usbp_add_endpoint(sc->sc_iface, UE_DIR_OUT | 2, UE_BULK,
	    64, 16, &sc->sc_ep_out);
	if (err) {
		printf(": usbf_add_endpoint failed\n");
		return;
	}

	/* Append a CDC union descriptor. */
	bzero(&udesc, sizeof udesc);
	udesc.bLength = sizeof udesc;
	udesc.bDescriptorType = UDESC_CS_INTERFACE;
	udesc.bDescriptorSubtype = UDESCSUB_CDC_UNION;
	udesc.bSlaveInterface[0] = usbp_interface_number(sc->sc_iface);
	err = usbp_add_config_desc(sc->sc_config,
	    (usb_descriptor_t *)&udesc, NULL);
	if (err) {
		printf(": usbf_add_config_desc failed\n");
		return;
	}

	/*
	 * Close the configuration and build permanent descriptors.
	 */
	err = usbp_end_config(sc->sc_config);
	if (err) {
		printf(": usbf_end_config failed\n");
		return;
	}

	/* Preallocate xfers and data buffers. */
	sc->sc_xfer_in = usbp_alloc_xfer(dev);
	sc->sc_xfer_out = usbp_alloc_xfer(dev);


	sc->sc_buffer_in = usbd_alloc_buffer(sc->sc_xfer_in,
	    CDCEF_BUFSZ);

	sc->sc_buffer_out = usbd_alloc_buffer(sc->sc_xfer_out,
	    CDCEF_BUFSZ);
	if (sc->sc_buffer_in == NULL || sc->sc_buffer_out == NULL) {
		printf(": usbf_alloc_buffer failed\n");
		return;
	}

	/* Open the bulk pipes. */
	err = usbp_open_pipe(sc->sc_iface,
	    usbp_endpoint_address(sc->sc_ep_out), &sc->sc_pipe_out) ||
	    usbp_open_pipe(sc->sc_iface,
	    usbp_endpoint_address(sc->sc_ep_in), &sc->sc_pipe_in);
	if (err) {
		printf(": usbf_open_pipe failed\n");
		return;
	}

	/* Get ready to receive packets. */
	usbd_setup_xfer(sc->sc_xfer_out, sc->sc_pipe_out, sc,
	    sc->sc_buffer_out, CDCEF_BUFSZ, USBD_SHORT_XFER_OK, 0, cdcef_rxeof);
	err = usbd_transfer(sc->sc_xfer_out);
	if (err && err != USBD_IN_PROGRESS) {
		printf(": usbd_transfer failed\n");
		return;
	}

	s = splnet();

	macaddr_hi = htons(0x2acb);
#ifdef	__NetBSD__
	do {
		union {
			uint8_t b[ETHER_ADDR_LEN];
			uint16_t h[ETHER_ADDR_LEN/sizeof (uint16_t)];
		} ea;

		ea.h[0] = macaddr_hi;
		ea.h[1] = time_uptime & 0xffff;
		ea.h[2] = (time_uptime >> 16) & 0xffff;
		ea.b[5] = device_unit(sc->sc_dev.dev);

		bcopy(&ea, sc->sc_ether_addr, ETHER_ADDR_LEN);
	} while (0);
				

	printf(": address %s\n", ether_sprintf(sc->sc_ether_addr));

#else
	bcopy(&macaddr_hi, &sc->sc_arpcom.ac_enaddr[0], sizeof(u_int16_t));
	bcopy(&ticks, &sc->sc_arpcom.ac_enaddr[2], sizeof(u_int32_t));
	sc->sc_arpcom.ac_enaddr[5] = (u_int8_t)(sc->sc_dev.bdev.dv_unit);

	printf(": address %s\n", ether_sprintf(sc->sc_arpcom.ac_enaddr));
#endif

	ifp = GET_IFP(sc);
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = cdcef_ioctl;
	ifp->if_start = cdcef_start;
	ifp->if_watchdog = cdcef_watchdog;
	ifp->if_init = cdcef_init;
	strlcpy(ifp->if_xname, DEVNAME(sc), IFNAMSIZ);


	IFQ_SET_READY(&ifp->if_snd);

	if_attach(ifp);
	ether_ifattach(ifp, sc->sc_ether_addr);
	splx(s);

}

usbd_status
cdcef_do_request(struct usbp_function *fun, usb_device_request_t *req,
    void **data)
{
	printf("cdcef_do_request\n");
	return USBD_STALLED;
}

void
cdcef_start(struct ifnet *ifp)
{
	struct cdcef_softc	*sc = ifp->if_softc;
	struct mbuf		*m_head = NULL;

	DPRINTF(1, ("%s\n", __func__));
	if(ifp->if_flags & IFF_OACTIVE)
		return;

	IFQ_POLL(&ifp->if_snd, m_head);
	if (m_head == NULL) {
		return;
	}

	if (sc->sc_listening == 0 || m_head->m_pkthdr.len > CDCEF_BUFSZ) {
		/*
		 * drop packet because receiver is not listening,
		 * or if packet is larger than xmit buffer
		 */
		IFQ_DEQUEUE(&ifp->if_snd, m_head);
		m_freem(m_head);
		return;
	}

	if (cdcef_encap(sc, m_head, 0)) {
		ifp->if_flags |= IFF_OACTIVE;
		return;
	}

	IFQ_DEQUEUE(&ifp->if_snd, m_head);

#if NBPFILTER > 0
	if (ifp->if_bpf)
		bpf_mtap(ifp->if_bpf, m_head, BPF_DIRECTION_OUT);
#endif
					
	ifp->if_flags |= IFF_OACTIVE;

	ifp->if_timer = 6;
}

void
cdcef_txeof(struct usbd_xfer *xfer, void *priv,
    usbd_status err)
{
	struct cdcef_softc *sc = priv;
	struct ifnet *ifp = GET_IFP(sc);
	int s;

	s = splnet();
#if 0
	printf("cdcef_txeof: xfer=%p, priv=%p, %s\n", xfer, priv,
	    usbf_errstr(err));
#endif

	ifp->if_timer = 0;
	ifp->if_flags &= ~IFF_OACTIVE;

	if (sc->sc_xmit_mbuf != NULL) {
		m_freem(sc->sc_xmit_mbuf);
		sc->sc_xmit_mbuf = NULL;
	}

	if (err)
		ifp->if_oerrors++;
	else
		ifp->if_opackets++;

#if 0 /*XXX*/
	if (IFQ_IS_EMPTY(&ifp->if_snd) == 0)
		timeout_add(&sc->start_to, 1); /* XXX  */
#endif

	splx(s);
}
void
cdcef_start_timeout (void *v)
{
	struct cdcef_softc *sc = v;
	struct ifnet *ifp = GET_IFP(sc);
	int s;

	s = splnet();
	cdcef_start(ifp);
	splx(s);
}


void
cdcef_rxeof(struct usbd_xfer *xfer, void *priv,
    usbd_status status)
{
	struct cdcef_softc	*sc = priv;
	int total_len = 0;
	struct ifnet		*ifp = GET_IFP(sc);
	struct mbuf		*m = NULL;
#ifndef	__NetBSD__
	struct mbuf_list	ml = MBUF_LIST_INITIALIZER();
#endif


	int s;

	DPRINTF(5, ("cdcef_rxeof: xfer=%p, priv=%p, %s\n", xfer, priv,
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

	/* upon first incoming packet we know the host is listening */
	if (sc->sc_listening == 0) {
		sc->sc_listening = 1;
	}


	usbd_get_xfer_status(xfer, NULL, NULL, &total_len, NULL);

	/* total_len -= 4; Strip off CRC added for Zaurus - XXX*/
	if (total_len <= 1)
		goto done;

	if (total_len < sizeof(struct ether_header)) {
		ifp->if_ierrors++;
		goto done;
	}

	if (ifp->if_flags & IFF_RUNNING) {
		m = cdcef_newbuf();
		if (m == NULL) {
			/* message? */
			ifp->if_ierrors++;
			goto done;
		}

		m->m_pkthdr.rcvif = ifp;
		m->m_pkthdr.len = m->m_len = total_len;
		bcopy(sc->sc_buffer_out, mtod(m, char *), total_len);

#ifndef __NetBSD__
		ml_enqueue(&ml, m);
#endif
	}

#ifdef	__NetBSD__
	if (m != NULL) {
		s = splnet();
		DPRINTF(5, ("cdcef_rxeof: calling if_input ifp=%p\n", ifp));
		ifp->if_input(ifp, m);
		DPRINTF(5, ("cdcef_rxeof: back from if_input\n"));
		splx(s);
	}
#else
	s = splnet();
	if_input(ifp, &ml);
	splx(s);
#endif
done:
	/* Setup another xfer. */
	usbd_setup_xfer(xfer, sc->sc_pipe_out, sc, sc->sc_buffer_out,
	    CDCEF_BUFSZ, USBD_SHORT_XFER_OK, 0, cdcef_rxeof);

	status = usbd_transfer(xfer);
	if (status && status != USBD_IN_PROGRESS) {
		printf("%s: usbd_transfer failed\n", DEVNAME(sc));
		return;
	}
}

struct mbuf *
cdcef_newbuf(void)
{
	struct mbuf		*m;

	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == NULL)
		return (NULL);

	MCLGET(m, M_DONTWAIT);
	if (!(m->m_flags & M_EXT)) {
		m_freem(m);
		return (NULL);
	}

	m->m_len = m->m_pkthdr.len = MCLBYTES;
#if 0
	m_adj(m, ETHER_ALIGN);
#else
	m_adj(m, 2);
#endif
	

	return (m);
}

int
cdcef_ioctl(struct ifnet *ifp, u_long command, void *data)
{
	struct cdcef_softc	*sc = ifp->if_softc;
	struct ifaddr		*ifa = (struct ifaddr *)data;
	struct ifreq		*ifr = (struct ifreq *)data;
	int			 s, error = 0;

	s = splnet();

	DPRINTF(10, ("cmd=%lx ifp=%p ifa=%p ifa->ifa_addr=%p ifp->if_init=%p\n", command, ifp, ifa, ifa ? ifa->ifa_addr : NULL, ifp->if_init));

	switch (command) {
	case SIOCSIFADDR:
		ifp->if_flags |= IFF_UP;
		if (!(ifp->if_flags & IFF_RUNNING))
			cdcef_init(ifp);
#ifdef __NetBSD__
		if (ifa->ifa_addr->sa_family == AF_INET)
			arp_ifinit(ifp, ifa);
#else	
		if (ifa->ifa_addr->sa_family == AF_INET)
			arp_ifinit(&sc->sc_arpcom, ifa);
#endif
		break;

	case SIOCSIFMTU:
		if (ifr->ifr_mtu < ETHERMIN || ifr->ifr_mtu > ETHERMTU)
			error = EINVAL;
		else if ((error = ifioctl_common(ifp, command, data)) == ENETRESET)
			error = 0;
		break;

	case SIOCSIFFLAGS:
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_flags & IFF_RUNNING)
				error = ENETRESET;
			else
				cdcef_init(ifp);
		} else {
			if (ifp->if_flags & IFF_RUNNING)
				cdcef_stop(sc);
		}
		break;

	default:
#ifdef __NetBSD__
		error = ether_ioctl(ifp, command, data);
#else
		error = ether_ioctl(ifp, &sc->sc_arpcom, command, data);
#endif
	}

	if (error == ENETRESET)
		error = 0;

	splx(s);
	return (error);
}

void
cdcef_watchdog(struct ifnet *ifp)
{
	struct cdcef_softc	*sc = ifp->if_softc;
	int s;

	ifp->if_oerrors++;
	printf("%s: watchdog timeout\n", DEVNAME(sc));

	s = splusb();
	ifp->if_timer = 0;
	ifp->if_flags &= ~IFF_OACTIVE;

	/* cancel receive pipe? */
	usbd_abort_pipe(sc->sc_pipe_in); /* in is tx pipe */
	splx(s);
}

static int
cdcef_init(struct ifnet *ifp)
{
	int s;

	s = splnet();

	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;

	splx(s);

	return 0;
}

int
cdcef_encap(struct cdcef_softc *sc, struct mbuf *m, int idx)
{
	usbd_status err;

	DPRINTF(1, ("%s: idx=%d xfer=%p pipe=%p\n", __func__, idx, sc->sc_xfer_in, sc->sc_pipe_in));

	m_copydata(m, 0, m->m_pkthdr.len, sc->sc_buffer_in);
	/* NO CRC */

	usbd_setup_xfer(sc->sc_xfer_in, sc->sc_pipe_in, sc, sc->sc_buffer_in,
	    m->m_pkthdr.len, USBD_FORCE_SHORT_XFER | USBD_NO_COPY,
	    10000, cdcef_txeof);

	printf("%s: %s\n", __func__, usbp_describe_xfer(sc->sc_xfer_in));

	err = usbd_transfer(sc->sc_xfer_in);
	if (err && err != USBD_IN_PROGRESS) {
		printf("encap error\n");
		cdcef_stop(sc);
		return (EIO);
	}
	sc->sc_xmit_mbuf = m;

	return (0);
}


void
cdcef_stop(struct cdcef_softc *sc)
{
	struct ifnet    *ifp = GET_IFP(sc);

	ifp->if_timer = 0;
	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);

	/* cancel receive pipe? */

	if (sc->sc_xmit_mbuf != NULL) {
		m_freem(sc->sc_xmit_mbuf);
		sc->sc_xmit_mbuf = NULL;
	}
}
