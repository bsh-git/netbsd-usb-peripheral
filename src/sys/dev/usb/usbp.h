/*	$NetBSD$ */

/*
 * you need to include <dev/usb/usbdivar.h> before this.
 */

#ifndef _USBP_H
#define _USBP_H

struct usbp_function;
struct usbp_config;
struct usbp_interface;
struct usbp_endpoint;
struct usbp_bus;
struct usbp_device;


/*
 * Attach USB function at the logical device.
 */
struct usbp_function_attach_args {
	struct usbp_device *device;
};

/*
 * Attach USBP logical device at USB client controller.
 */

struct usbp_bus_attach_args {
	const char *busname;
	struct usbp_bus *bus;
};

struct usbp_function_methods {
	usbd_status (*set_config)(struct usbp_function *, struct usbp_config *);
	usbd_status (*do_request)(struct usbp_function *,
	    usb_device_request_t *req, void **data);
};

struct usbp_function {
	device_t      dev; 		/* base device */
	//struct device bdev;
	/* filled in by function driver */
	struct usbp_function_methods *methods;
};


struct usbp_bus {
	struct usbd_bus	usbd;

	u_int8_t ep0_maxp;	/* packet size for EP0 */

	enum {
		EP0_IDLE,	/* waiting for SETUP */
		EP0_IN_DATA_PHASE,	/* sending IN packets to the host */
		EP0_END_XFER		/* all data is sent. */
	}	 ep0state;
	
	int			 intr_context;

	struct usbp_device *device;	/* USB device we are acting as on this bus */
//	struct usbp_softc *usbfctl;	// XXX use usbd_bus.usbctl
};


struct usbp_endpoint {
	struct usbd_endpoint usbd;

	struct usbp_interface *iface;
	SIMPLEQ_ENTRY(usbp_endpoint) next;	// XXX do we really need this?
};


void usbp_devinfo_setup(struct usbp_device *dev, u_int8_t devclass,
			u_int8_t subclass, u_int8_t proto, u_int16_t vendor, u_int16_t product,
			u_int16_t device, const char *manf, const char *prod, const char *ser);

usbd_status usbp_add_config_desc(struct usbp_config *uc, usb_descriptor_t *d,
				 usb_descriptor_t **dp);
usbd_status usbp_add_config(struct usbp_device *dev, struct usbp_config **ucp);
usbd_status usbp_end_config(struct usbp_config *uc);

usbd_status usbp_add_interface(struct usbp_config *uc, u_int8_t bInterfaceClass,
			       u_int8_t bInterfaceSubClass, u_int8_t bInterfaceProtocol,
			       const char *string, struct usbp_interface **uip);
usbd_status
usbp_add_endpoint(struct usbp_interface *ui, u_int8_t bEndpointAddress,
		  u_int8_t bmAttributes, u_int16_t wMaxPacketSize, u_int8_t bInterval,
		  struct usbp_endpoint **uep);

int usbp_interface_number(struct usbp_interface *iface);
u_int8_t usbd_endpoint_address(struct usbd_endpoint *);
#define usbd_endpoint_index(e)	UE_GET_ADDR(usbd_endpoint_address((e)))
#define usbd_endpoint_dir(e)	UE_GET_DIR(usbd_endpoint_address((e)))
#define	usbd_endpoint_type(e)	UE_GET_XFERTYPE(usbd_endpoint_attributes(e))

#define	usbp_endpoint_address(e)	usbd_endpoint_address(&e->usbd)
#define	usbp_endpoint_index(e)	usbd_endpoint_index(&e->usbd)
#define	usbp_endpoint_dir(e)	usbd_endpoint_dir(&e->usbd)
#define usbp_endpoint_type(e)	usbd_endpoint_type(&e->usbd)

static inline u_int8_t
usbd_endpoint_attributes(struct usbd_endpoint *endpoint)
{
	return endpoint->edesc->bmAttributes;
}

#define	usbp_endpoint_attributes(endpoint)	usbd_endpoint_attributes(&endpoint->usbd)

usbd_status usbp_open_pipe(struct usbp_interface *iface, u_int8_t address,
			   struct usbd_pipe **pipe);

usbd_status usbp_open_pipe_ival(struct usbp_interface *iface, u_int8_t address,
				struct usbd_pipe **pipe, int ival);


u_int8_t usbp_add_string(struct usbp_device *dev, const char *string);

/* XXX usbd_alloc_xfer() ? */
struct usbd_xfer * usbp_alloc_xfer(struct usbp_device *dev);
/* XXX usbd_alloc_buffer() ? */
void * usbp_alloc_buffer(struct usbd_xfer *xfer, u_int32_t size);

void usbp_setup_xfer(struct usbd_xfer *xfer, struct usbd_pipe *pipe,
		     void *priv, void *buffer, u_int32_t length,
		     u_int16_t flags, u_int32_t timeout, usbd_callback callback);

usb_device_descriptor_t *usbp_device_descriptor(struct usbp_device *dev);


usbd_status usbp_insert_transfer(struct usbd_xfer *xfer);
void usbp_transfer_complete(struct usbd_xfer *xfer);
void usbp_start_next(struct usbd_pipe *pipe);

void	    usbp_host_reset(struct usbp_bus *);


/* debug */
void usbp_dump_request(struct usbp_device *dev, usb_device_request_t *req);
char *usbp_describe_xfer(struct usbd_xfer *xfer);
void usbp_dump_buffer(struct usbd_xfer *xfer);


#endif	/* _USBP_H */
