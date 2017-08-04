/*-
 * Copyright (c) 2017 Ilya Bakulin.  All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/sysctl.h>

#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/cam_queue.h>
#include <cam/cam_periph.h>
#include <cam/cam_xpt.h>
#include <cam/cam_xpt_periph.h>
#include <cam/cam_xpt_internal.h> /* for cam_path */
#include <cam/cam_debug.h>

#include <cam/mmc/sdio_ivars.h>

#include "sdio_subr.h"
#include "opt_cam.h"

struct sdio_softc {
	device_t dev;
	device_t card_dev;
#define SDIO_STATE_INIT  (0x1  << 1)
#define SDIO_STATE_READY (0x1 << 2)
	uint8_t state;

	/*
	 * softc is used both in CAM and newbus, so we need to keep track
	 * which part is still alive.
	 * XXX Convert to a simple refcount field?
	 */
	uint8_t is_cam_attached;
	uint8_t is_newbus_attached;
	struct task start_init_task;
	struct card_info cinfo;
};

/* Peripheral driver methods */
static	periph_init_t	sdioinit;
static	periph_deinit_t	sdiodeinit;

/* Peripheral device methods */
static	periph_ctor_t	sdioregister;
static	periph_dtor_t	sdiocleanup;
static	periph_start_t	sdiostart;
static	periph_oninv_t	sdiooninvalidate;

static void sdio_identify(driver_t *, device_t);
static void sdio_real_identify(driver_t *driver,
			       device_t parent,
			       struct sdio_softc *sc);
static int  sdio_probe(device_t);
static int  sdio_attach(device_t);
static int  sdio_detach(device_t);
static int  sdio_read_ivar(device_t dev, device_t child, int index,
			  uintptr_t *result);

static void  sdioasync(void *callback_arg, u_int32_t code,
		       struct cam_path *path, void *arg);

static void  sdio_start_init_task(void *context, int pending);

static struct periph_driver sdiodriver =
{
	sdioinit, "sdio",
	TAILQ_HEAD_INITIALIZER(sdiodriver.units),
	/* generation */ 0,
	/* flags */ 0,
	sdiodeinit
};

PERIPHDRIVER_DECLARE(sdio, sdiodriver);
static MALLOC_DEFINE(M_SDIO, "sdio", "sdio buffers");

static device_method_t sdio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,      sdio_identify),
	DEVMETHOD(device_probe,         sdio_probe),
	DEVMETHOD(device_attach,        sdio_attach),
	DEVMETHOD(device_detach,        sdio_detach),

	/* bus interface */
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,	bus_generic_adjust_resource),
	DEVMETHOD(bus_alloc_resource,	bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_get_resource,	bus_generic_rl_get_resource),
	DEVMETHOD(bus_release_resource, bus_generic_rl_release_resource),
	DEVMETHOD(bus_set_resource,	bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource_list, bus_generic_get_resource_list),
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_add_child,	bus_generic_add_child),
	DEVMETHOD(bus_read_ivar,	sdio_read_ivar),
	DEVMETHOD(bus_write_ivar,	bus_generic_write_ivar),

	DEVMETHOD_END
};

driver_t sdio_driver = {
	"sdio",
	sdio_methods,
	sizeof(struct sdio_softc),
};

static void
sdioinit(void)
{
	cam_status status;
	/*
	 * Install a global async callback.  This callback will
	 * receive async callbacks like "new device found".
	 */
	status = xpt_register_async(AC_FOUND_DEVICE, sdioasync, NULL, NULL);

	if (status != CAM_REQ_CMP) {
		printf("sdio: Failed to attach master async callback "
		       "due to status 0x%x!\n", status);
	}
}

/* This function should just exist to allow unloading the KLD */
static int
sdiodeinit(void)
{
	struct cam_periph *periph, *periph_temp;

	printf("CAM is calling sdiodeinit()\n");
	/* Walk through all instances and invalidate them manually */
	TAILQ_FOREACH_SAFE(periph, &sdiodriver.units, unit_links, periph_temp) {
		cam_periph_lock(periph);
		printf("Invalidating %s\n", periph->periph_name);
		cam_periph_invalidate(periph);
	}
	return (0);
}

static cam_status
sdioregister(struct cam_periph *periph, void *arg)
{
	struct sdio_softc *softc;
	struct ccb_getdev *cgd;

	CAM_DEBUG(periph->path, CAM_DEBUG_TRACE, ("sdioregister\n"));
	cgd = (struct ccb_getdev *)arg;
	if (cgd == NULL) {
		printf("sdioregister: no getdev CCB, can't register device\n");
		return(CAM_REQ_CMP_ERR);
	}

	softc = (struct sdio_softc *)malloc(sizeof(*softc), M_DEVBUF,
					    M_NOWAIT|M_ZERO);

	if (softc == NULL) {
		printf("sdioregister: Unable to probe new device. "
		       "Unable to allocate softc\n");
		return (CAM_REQ_CMP_ERR);
	}

	softc->state = SDIO_STATE_INIT;
	softc->is_cam_attached = 1;

	TASK_INIT(&softc->start_init_task, 0, sdio_start_init_task, periph);

	periph->softc = softc;

	xpt_schedule(periph, CAM_PRIORITY_XPT);
	return (CAM_REQ_CMP);
}

static void
sdioasync(void *callback_arg, u_int32_t code,
	struct cam_path *path, void *arg)
{
	struct cam_periph *periph;
	__unused struct sdio_softc *softc;

	periph = (struct cam_periph *)callback_arg;
	CAM_DEBUG(path, CAM_DEBUG_TRACE, ("sdioasync(code=%d)\n", code));
	switch (code) {
	case AC_FOUND_DEVICE: {
		struct ccb_getdev *cgd;
		cam_status status;

		cgd = (struct ccb_getdev *)arg;
		if (cgd == NULL)
			break;

		if (cgd->protocol != PROTO_MMCSD)
			break;

		/* We support only SDIO cards without memory portion */
		if ((path->device->mmc_ident_data.card_features & CARD_FEATURE_MEMORY)) {
			CAM_DEBUG(path, CAM_DEBUG_TRACE, ("Memory card, not interested\n"));
			break;
		}

		/*
		 * Allocate a peripheral instance for
		 * this device and start the probe
		 * process.
		 */
		status = cam_periph_alloc(sdioregister, sdiooninvalidate,
					  sdiocleanup, sdiostart,
					  "sdio", CAM_PERIPH_BIO,
					  path, sdioasync,
					  AC_FOUND_DEVICE, cgd);

		if (status != CAM_REQ_CMP
		    && status != CAM_REQ_INPROG)
			CAM_DEBUG(path, CAM_DEBUG_PERIPH, ("sdioasync: Unable to attach to new device due to status 0x%x\n", status));
		break;
	}
	default:
		CAM_DEBUG(path, CAM_DEBUG_PERIPH, ("Cannot handle async code 0x%02x\n", code));
		cam_periph_async(periph, code, path, arg);
		break;
	}
}

static void
sdiooninvalidate(struct cam_periph *periph)
{
	struct sdio_softc *softc;

	softc = (struct sdio_softc *)periph->softc;

	CAM_DEBUG(periph->path, CAM_DEBUG_TRACE, ("sdiooninvalidate\n"));

	/*
	 * De-register any async callbacks.
	 */
	xpt_register_async(0, sdioasync, periph, periph->path);
}

static void
sdiocleanup(struct cam_periph *periph)
{
	struct sdio_softc *softc;

	CAM_DEBUG(periph->path, CAM_DEBUG_TRACE, ("sdiocleanup\n"));
	softc = (struct sdio_softc *)periph->softc;
	/*
	 * If newbus deallocation code has already run, destroy softc
	 * Otherwise just mark CAM as detached so that newbus detach
	 * is allowed to release the memory.
	 */
	if (!softc->is_newbus_attached)
		free(softc, M_DEVBUF);
	else
		softc->is_cam_attached = 0;
	cam_periph_unlock(periph);
}

static void
sdio_start_init_task(void *context, int pending) {
	union ccb *new_ccb;
	struct cam_periph *periph;
	struct sdio_softc *softc;

	periph = (struct cam_periph *)context;
	softc = (struct sdio_softc *)periph->softc;
	CAM_DEBUG(periph->path, CAM_DEBUG_TRACE, ("sdio_start_init_task\n"));
	/* periph was held for us when this task was enqueued */
	if ((periph->flags & CAM_PERIPH_INVALID) != 0) {
		cam_periph_release(periph);
		return;
	}
	new_ccb = xpt_alloc_ccb();
	xpt_setup_ccb(&new_ccb->ccb_h, periph->path,
		      CAM_PRIORITY_NONE);

	cam_periph_lock(periph);

	/*
	 * Read CCCR and FBR of each function, get manufacturer and device IDs,
	 * max block size, possibly more information like this that might be useful
	 */
	get_sdio_card_info(new_ccb, &softc->cinfo);

	softc->state = SDIO_STATE_READY;
	cam_periph_unlock(periph);
	xpt_free_ccb(new_ccb);

	/*
	 * Now CAM portion of the driver has been initialized and
	 * we know VID/PID of all the functions on the card.
	 * Time to hook into the newbus.
	 */
	sdio_real_identify(&sdio_driver, periph->sim->parent_dev, softc);
}

static void
sdiostart(struct cam_periph *periph, union ccb *start_ccb)
{
	struct sdio_softc *softc = (struct sdio_softc *)periph->softc;
	CAM_DEBUG(periph->path, CAM_DEBUG_TRACE, ("sdiostart\n"));

	if (softc->state == SDIO_STATE_INIT) {
		/* Make initialization */
		taskqueue_enqueue(taskqueue_thread, &softc->start_init_task);
		xpt_release_ccb(start_ccb);
	} else
		xpt_release_ccb(start_ccb);
	return;
}

/*
 * This is normally called by the parent bus.
 * Thus, mmcnull calls us here. At this point we cannot really
 * do anything, since even the card might not have been inserted yet.
 *
 * So the real work is done by sdio_real_identify().
 * It's called by CAM peripheral device "sdio" when it has scanned
 * the SDIO card registers and knows what functions are available
 * on the card and their VIDs/PIDs.
 * Since sdio_real_identify is called not during loading/unloading
 * modules or newbus-related events, we have to initiate device
 * probe/attach ourselves.
 */
static void
sdio_identify(driver_t *driver, device_t parent)
{
	device_printf(parent, "sdio_identify() called\n");
}

static void
sdio_real_identify(driver_t *driver, device_t parent, struct sdio_softc *sc) {
	device_t child;
	int ret;

	if (resource_disabled("sdio", 0))
		return;

	device_printf(parent, "sdio_real_identify() called\n");
	/* Avoid duplicates. */
	if (device_find_child(parent, "sdio", -1)) {
		device_printf(parent, "sdio_identify(): there is already a child\n");
		return;
	}

	child = BUS_ADD_CHILD(parent, 20, "sdio", 0);
	if (child == NULL) {
		device_printf(parent, "add SDIO child failed\n");
		return;
	}

	device_printf(parent, "BUS_ADD_CHILD() finished, child=%s\n",
		      device_get_nameunit(child));

	device_set_desc(child, "SDIO bus");

	/*
	 * Newbus stuff needs to be Giant-locked!
	 */
	mtx_lock(&Giant);
	ret = device_probe_and_attach(child);
	mtx_unlock(&Giant);
	if (ret != 0)
		device_printf(child, "attach() failed, ret=%d", ret);
	/*
	 * Now attach() has returned, so we can fill in the softc.
	 * We use device_set_softc() which sets a flag DF_EXTERNALSOFTC
	 * so that softc is not freed when the device detaches.
	 * This is just what we need since we manage softc from within
	 * the CAM code.
	 */
	device_set_softc(child, sc);
	sc->is_newbus_attached = 1;
	sc->card_dev = bus_generic_add_child(child, 0, NULL, -1);
	/*
	 * XXX If the brcmwl.ko was loaded before this code executes,
	 * it will not be probed/attached. But if it's loaded after,
	 * it is probed and attached just fine.
	 * Why?
	 */
//	bus_generic_attach(dev);
}

static int
sdio_probe(device_t dev)
{
	device_printf(dev, "SDIO probe() called\n");
	device_set_desc(dev, "SDIO bus");
	return (BUS_PROBE_DEFAULT);
}

static int
sdio_attach(device_t dev)
{
	device_printf(dev, "attached OK\n");
	return (0);
}

static int
sdio_detach(device_t dev)
{
	struct sdio_softc *sc;
	int ret;

	sc = device_get_softc(dev);
	ret = device_delete_child(dev, sc->card_dev);
	if (ret != 0) {
		device_printf(dev, "Cannot detach child device %s: %d",
			      device_get_nameunit(sc->card_dev), ret);
	}
	sc->is_newbus_attached = 0;
	if (!sc->is_cam_attached)
		free(sc, M_DEVBUF);
	device_printf(dev, "detached OK\n");
	return (0);

	/*
	 * This is not enough to delete the device from the tree!
	 * devinfo:
	 *
	 * isa0
	 *   vga0
	 *   mmcnull0
	 *
	 * But devinfo -v:
	 * isa0
	 *  sc0
	 *  vga0
	 *  fdc0
	 *  ppc0
	 *  mmcnull0
	 *    sdio0
	 */
}

static int
sdio_read_ivar(device_t dev, device_t child, int index,
	       uintptr_t *result) {

	device_printf(dev, "sdio_read_ivar(me, %s, %d)\n",
		      device_get_nameunit(child),
		      index);
	switch (index) {
	case SDIO_IVAR_VENDOR_ID:
		*result = (int) 0xDEADBEEF;
		break;
	}
	return (ENOENT);
}

devclass_t sdio_devclass;

MODULE_VERSION(sdio, 1);

/*
 * As possible parent devclass we should also list sdhci.
 * mmcnull doesn't extend sdhci so it stays on its own.
 */
DRIVER_MODULE(sdio, mmcnull, sdio_driver, sdio_devclass, 0, 0);
