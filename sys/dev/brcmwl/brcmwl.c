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

#include <cam/mmc/sdio_ivars.h>

#define SDIO_VENDOR_BROADCOM 0x02d0

struct brcmwl_softc {
	device_t dev;
};

static int	brcmwl_probe(device_t dev);
static int	brcmwl_attach(device_t dev);
static int	brcmwl_detach(device_t dev);

/*
 * XXX Do we need to implement device_identify()?
 */
static int
brcmwl_probe(device_t dev)
{
	int vendor_id;

	device_printf(dev, "probe() called\n");
	vendor_id = sdio_get_vendor_id(dev);
	device_printf(dev, "vendor_id=%d\n", vendor_id);
	if (vendor_id != SDIO_VENDOR_BROADCOM) {
		device_printf(dev, "Non-Broadcom card\n");
		return (-1);
	}
	return (BUS_PROBE_GENERIC);
}

static int
brcmwl_attach(device_t dev)
{
	device_printf(dev, "attach() called\n");
	device_set_desc(dev, "Imaginary SDIO WiFi driver");
	return (0);
}

static int
brcmwl_detach(device_t dev)
{
	return (0);
}

static device_method_t brcmwl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		brcmwl_probe),
	DEVMETHOD(device_attach,	brcmwl_attach),
	DEVMETHOD(device_detach,	brcmwl_detach),

	DEVMETHOD_END
};

static driver_t brcmwl_driver = {
	"brcmwl",
	brcmwl_methods,
	sizeof(struct brcmwl_softc),
};

static devclass_t brcmwl_devclass;
DRIVER_MODULE(brcmwl, sdio, brcmwl_driver, brcmwl_devclass, NULL, NULL);
