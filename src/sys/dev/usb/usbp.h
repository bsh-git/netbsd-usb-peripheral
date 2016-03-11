/*	$NetBSD$ */

/*
 * you need to include <dev/usb/usbdivar.h> before this.
 */

#include <dev/usb/usbpif.h>
#include <dev/usb/usbdi.h>

#ifndef _USBP_H
#define _USBP_H

struct usbp_interface;
struct usbp_endpoint;
struct usbp_bus;
struct usbp_device;


/*
 * Attach USB function at the logical device.
 */
struct usbp_interface_attach_args {
	const char *busname;
	struct usbp_device *device;
};

/*
 * Attach USBP logical device at USB client controller.
 */

struct usbp_bus_attach_args {
	const char *busname;
	struct usbp_bus *bus;
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

	struct usbp_device *device;	/* USB device we are acting */
	struct usbp_bus_methods *usbp_methods;	/* additional bus methods for
						   peripheral-side*/

};


struct usbp_bus_methods {
	usbd_status (* select_endpoint)(
		struct usbp_bus *, struct usbp_endpoint_request *, u_int);
	usbd_status (* enable)(struct usbp_bus *, bool);
	void (* pullup_control)(struct usbp_bus *, bool);
	bool (* is_connected)(struct usbp_bus *);
};

struct usbp_interface_methods {
	usbd_status (* configured)(struct usbp_interface *);
	usbd_status (* unconfigured)(struct usbp_interface *);
	usbd_status (* handle_device_request)(
		struct usbp_interface *, usb_device_request_t *, void **);
	usbd_status (* fixup_idesc)(
		struct usbp_interface *, usb_interface_descriptor_t *);
};


/* strings in configuration */
enum USBP_IF_STRINGS {
	USBP_IF_STR_MANUFACTURER,
	USBP_IF_STR_PRODUCT_NAME,
	USBP_IF_STR_SERIAL,
	USBP_IF_STR_DESCRIPTION };

struct usbp_interface {
	struct usbd_interface usbd;

	/* device information.  used if this is the primay interface of the device*/
	struct {
		bool class_id_valid;
		uint8_t class_id;
		uint8_t subclass_id;
		uint8_t protocol;

		bool vendor_id_valid;
		uint16_t vendor_id;
		uint16_t product_id;
		uint16_t bcd_device;
	} devinfo;
	
	/* interface spec */
	const struct usbp_interface_spec *ispec;
	const struct usbp_endpoint_request *epreq;

	int num_strings;
	struct usbp_string **string;

	const void *additional_idesc;
	size_t additional_idesc_size;

	const struct usbp_interface_methods *methods;

	size_t alloc_size;
	SIMPLEQ_ENTRY(usbp_interface) next;	// list of interfaces in a device.
};

void usbp_init_interface(struct usbp_device *, struct usbp_interface *);
#define	usbp_interface_to_device(iface)	((struct usbp_device *)iface->usbd.device)

int usbp_interface_number(struct usbp_interface *iface);
u_int8_t usbd_endpoint_address(struct usbd_endpoint *);
#define usbd_endpoint_index(e)	UE_GET_ADDR(usbd_endpoint_address((e)))
#define usbd_endpoint_dir(e)	UE_GET_DIR(usbd_endpoint_address((e)))
#define	usbd_endpoint_type(e)	UE_GET_XFERTYPE(usbd_endpoint_attributes(e))

#if 0
#define	usbp_endpoint_address(e)	usbd_endpoint_address(&e->usbd)
#define	usbp_endpoint_index(e)	usbd_endpoint_index(&e->usbd)
#define	usbp_endpoint_dir(e)	usbd_endpoint_dir(&e->usbd)
#define usbp_endpoint_type(e)	usbd_endpoint_type(&e->usbd)
#endif

struct usbd_endpoint *usbp_iface_endpoint(struct usbp_interface *, int);

static inline u_int8_t
usbd_endpoint_attributes(struct usbd_endpoint *endpoint)
{
	return endpoint->edesc->bmAttributes;
}

usbd_status usbp_open_pipe(struct usbp_interface *iface, int index, int flags, struct usbd_pipe **pipe);

#if 0
usbd_status usbp_open_pipe_ival(struct usbp_interface *iface, u_int8_t address,
				struct usbd_pipe **pipe, int ival);
#endif

u_int8_t usbp_add_string(struct usbp_device *dev, const char *string);

static inline struct usbd_xfer *usbp_alloc_xfer(struct usbp_device *dev)
{
	return usbd_alloc_xfer((struct usbd_device *)dev);
}

usb_device_descriptor_t *usbp_device_descriptor(struct usbp_device *dev);


usbd_status usbp_insert_transfer(struct usbd_xfer *xfer);
void usbp_transfer_complete(struct usbd_xfer *xfer);
void usbp_start_next(struct usbd_pipe *pipe);

void	    usbp_host_reset(struct usbp_bus *);


/* debug */
void usbp_dump_request(struct usbp_device *dev, usb_device_request_t *req);
char *usbp_describe_xfer(struct usbd_xfer *xfer);
void usbp_dump_buffer(struct usbd_xfer *xfer);


/* New APIs */
usbd_status usbp_add_interface(struct usbp_device *,
    const struct usbp_add_iface_request *,
    const struct usbp_interface_methods *,
    const void *,         /* additional_interface_descriptor */
    size_t,  		/* additional_interface_descriptor_size */
    struct usbp_interface **	/* returns the created interface object */
);
    
usbd_status usbp_delete_interface(struct usbp_interface *);

#endif	/* _USBP_H */
