/*	$NetBSD: usb_subr.c,v 1.196 2014/02/17 07:34:21 skrll Exp $	*/
/*	$FreeBSD: src/sys/dev/usb/usb_subr.c,v 1.18 1999/11/17 22:33:47 n_hibma Exp $	*/

/*
 * Copyright (c) 1998, 2004 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: usb_subr.c,v 1.196 2014/02/17 07:34:21 skrll Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/kthread.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/once.h>
#include <sys/atomic.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>


#ifdef USB_DEBUG
#define DPRINTF(x)	if (usbdebug) printf x
#define DPRINTFN(n,x)	if (usbdebug>(n)) printf x
int usbdebug = 11;
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif

Static void	usb_task_thread(void *);


Static const char * const usbd_error_strs[] = {
	"NORMAL_COMPLETION",
	"IN_PROGRESS",
	"PENDING_REQUESTS",
	"NOT_STARTED",
	"INVAL",
	"NOMEM",
	"CANCELLED",
	"BAD_ADDRESS",
	"IN_USE",
	"NO_ADDR",
	"SET_ADDR_FAILED",
	"NO_POWER",
	"TOO_DEEP",
	"IOERROR",
	"NOT_CONFIGURED",
	"TIMEOUT",
	"SHORT_XFER",
	"STALLED",
	"INTERRUPTED",
	"NOT_FOR_US",
	"XXX",
};

const char *
usbd_errstr(usbd_status err)
{
	static char buffer[5];

	if (err < USBD_ERROR_MAX) {
		return usbd_error_strs[err];
	} else {
		snprintf(buffer, sizeof buffer, "%d", err);
		return buffer;
	}
}

usbd_status
usbd_setup_pipe(usbd_device_handle dev, usbd_interface_handle iface,
		struct usbd_endpoint *ep, int ival, usbd_pipe_handle *pipe)
{
	return usbd_setup_pipe_flags(dev, iface, ep, ival, pipe, 0);
}

usbd_status
usbd_setup_pipe_flags(usbd_device_handle dev, usbd_interface_handle iface,
    struct usbd_endpoint *ep, int ival, usbd_pipe_handle *pipe, uint8_t flags)
{
	usbd_pipe_handle p;
	usbd_status err;

	DPRINTFN(1,("usbd_setup_pipe: dev=%p iface=%p ep=%p\n",
		    dev, iface, ep));
	p = malloc(dev->bus->pipe_size, M_USB, M_NOWAIT);
	if (p == NULL)
		return (USBD_NOMEM);
	DPRINTFN(1,("usbd_setup_pipe: pipe=%p\n", p));

	p->device = dev;
	p->iface = iface;
	p->endpoint = ep;
	ep->refcnt++;
	p->refcnt = 1;
	p->intrxfer = NULL;
	p->running = 0;
	p->aborting = 0;
	p->repeat = 0;
	p->interval = ival;
	p->flags = flags;
	SIMPLEQ_INIT(&p->queue);
	err = dev->bus->methods->open_pipe(p);
	if (err) {
		DPRINTFN(-1,("usbd_setup_pipe: endpoint=0x%x failed, error="
			 "%s\n",
			 ep->edesc->bEndpointAddress, usbd_errstr(err)));
		free(p, M_USB);
		return (err);
	}
	usb_init_task(&p->async_task, usbd_clear_endpoint_stall_task, p,
	    USB_TASKQ_MPSAFE);
	*pipe = p;
	return (USBD_NORMAL_COMPLETION);
}


/*
 * USB task
 */

struct usb_taskq {
	TAILQ_HEAD(, usb_task) tasks;
	kmutex_t lock;
	kcondvar_t cv;
	struct lwp *task_thread_lwp;
	const char *name;
};

static struct usb_taskq usb_taskq[USB_NUM_TASKQS];

static const char *taskq_names[] = USB_TASKQ_NAMES;


static int
usb_task_once_init(void)
{
	struct usb_taskq *taskq;
	int i;

	for (i = 0; i < USB_NUM_TASKQS; i++) {
		taskq = &usb_taskq[i];

		TAILQ_INIT(&taskq->tasks);
		/*
		 * Since USB task methods usb_{add,rem}_task are callable
		 * from any context, we have to make this lock a spinlock.
		 */
		mutex_init(&taskq->lock, MUTEX_DEFAULT, IPL_USB);
		cv_init(&taskq->cv, "usbtsk");
		taskq->name = taskq_names[i];
		if (kthread_create(PRI_NONE, KTHREAD_MPSAFE, NULL,
		    usb_task_thread, taskq, &taskq->task_thread_lwp,
		    "%s", taskq->name)) {
			printf("unable to create task thread: %s\n", taskq->name);
			panic("usb_create_event_thread task");
		}
		/*
		 * XXX we should make sure these threads are alive before
		 * end up using them in usb_doattach().
		 */
	}

	return 0;
}

void
usb_task_init(void)
{
	static ONCE_DECL(init_control);

	RUN_ONCE(&init_control, usb_task_once_init);
}

void
usb_task_thread(void *arg)
{
	struct usb_task *task;
	struct usb_taskq *taskq;
	bool mpsafe;

	taskq = arg;
	DPRINTF(("usb_task_thread: start taskq %s\n", taskq->name));

	mutex_enter(&taskq->lock);
	for (;;) {
		task = TAILQ_FIRST(&taskq->tasks);
		if (task == NULL) {
			cv_wait(&taskq->cv, &taskq->lock);
			task = TAILQ_FIRST(&taskq->tasks);
		}
		DPRINTFN(2,("usb_task_thread: woke up task=%p\n", task));
		if (task != NULL) {
			mpsafe = ISSET(task->flags, USB_TASKQ_MPSAFE);
			TAILQ_REMOVE(&taskq->tasks, task, next);
			task->queue = USB_NUM_TASKQS;
			mutex_exit(&taskq->lock);

			if (!mpsafe)
				KERNEL_LOCK(1, curlwp);
			task->fun(task->arg);
			/* Can't dereference task after this point.  */
			if (!mpsafe)
				KERNEL_UNLOCK_ONE(curlwp);

			mutex_enter(&taskq->lock);
		}
	}
	mutex_exit(&taskq->lock);
}


/*
 * Add a task to be performed by the task thread.  This function can be
 * called from any context and the task will be executed in a process
 * context ASAP.
 */
void
usb_add_task(usbd_device_handle dev, struct usb_task *task, int queue)
{
	struct usb_taskq *taskq;

	KASSERT(0 <= queue);
	KASSERT(queue < USB_NUM_TASKQS);
	taskq = &usb_taskq[queue];
	mutex_enter(&taskq->lock);
	if (atomic_cas_uint(&task->queue, USB_NUM_TASKQS, queue) ==
	    USB_NUM_TASKQS) {
		DPRINTFN(2,("usb_add_task: task=%p\n", task));
		TAILQ_INSERT_TAIL(&taskq->tasks, task, next);
		cv_signal(&taskq->cv);
	} else {
		DPRINTFN(3,("usb_add_task: task=%p on q\n", task));
	}
	mutex_exit(&taskq->lock);
}

/*
 * XXX This does not wait for completion!  Most uses need such an
 * operation.  Urgh...
 */
void
usb_rem_task(usbd_device_handle dev, struct usb_task *task)
{
	unsigned queue;

	while ((queue = task->queue) != USB_NUM_TASKQS) {
		struct usb_taskq *taskq = &usb_taskq[queue];
		mutex_enter(&taskq->lock);
		if (__predict_true(task->queue == queue)) {
			TAILQ_REMOVE(&taskq->tasks, task, next);
			task->queue = USB_NUM_TASKQS;
			mutex_exit(&taskq->lock);
			break;
		}
		mutex_exit(&taskq->lock);
	}
}

