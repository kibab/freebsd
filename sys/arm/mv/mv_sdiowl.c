/*-
 * Copyright (c) 2013 Ilya Bakulin.  All rights reserved.
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
 * Portions of this software may have been developed with reference to
 * the SD Simplified Specification.  The following disclaimer may apply:
 *
 * The following conditions apply to the release of the simplified
 * specification ("Simplified Specification") by the SD Card Association and
 * the SD Group. The Simplified Specification is a subset of the complete SD
 * Specification which is owned by the SD Card Association and the SD
 * Group. This Simplified Specification is provided on a non-confidential
 * basis subject to the disclaimers below. Any implementation of the
 * Simplified Specification may require a license from the SD Card
 * Association, SD Group, SD-3C LLC or other third parties.
 *
 * Disclaimers:
 *
 * The information contained in the Simplified Specification is presented only
 * as a standard specification for SD Cards and SD Host/Ancillary products and
 * is provided "AS-IS" without any representations or warranties of any
 * kind. No responsibility is assumed by the SD Group, SD-3C LLC or the SD
 * Card Association for any damages, any infringements of patents or other
 * right of the SD Group, SD-3C LLC, the SD Card Association or any third
 * parties, which may result from its use. No license is granted by
 * implication, estoppel or otherwise under any patent or other rights of the
 * SD Group, SD-3C LLC, the SD Card Association or any third party. Nothing
 * herein shall be construed as an obligation by the SD Group, the SD-3C LLC
 * or the SD Card Association to disclose or distribute any technical
 * information, know-how or other confidential information to any third party.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcvar.h>
#include <dev/mmc/mmcioreg.h>
#include <dev/mmc/mmciovar.h>

#include "mmcbus_if.h"

struct sdiowl_softc {
	device_t dev;
	struct mtx sc_mtx;
	int running;
};

/* bus entry points */
static int sdiowl_attach(device_t dev);
static int sdiowl_detach(device_t dev);
static int sdiowl_probe(device_t dev);

#define SDIOWL_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	SDIOWL_UNLOCK(_sc)	mtx_unlock(&(_sc)->sc_mtx)
#define SDIOWL_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->dev), \
	    "sdiowl", MTX_DEF)
#define SDIOWL_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);
#define SDIOWL_ASSERT_LOCKED(_sc) mtx_assert(&_sc->sc_mtx, MA_OWNED);
#define SDIOWL_ASSERT_UNLOCKED(_sc) mtx_assert(&_sc->sc_mtx, MA_NOTOWNED);

static int
sdiowl_probe(device_t dev)
{
	uint32_t media_size, sdio_vendor, sdio_product;

//	device_quiet(dev);
	device_set_desc(dev, "Marvell dummy SDIO WLAN driver");

	if(BUS_READ_IVAR(device_get_parent(dev), dev, MMC_IVAR_MEDIA_SIZE,
			 &media_size)) {
		device_printf(dev, "Cannot get media size from the bus!\n");
		return (-1);
	}
	if (media_size > 0)
		return(-1);

	if(BUS_READ_IVAR(device_get_parent(dev), dev, MMC_IVAR_SDIO_VENDOR,
			 &sdio_vendor) ||
	   BUS_READ_IVAR(device_get_parent(dev), dev, MMC_IVAR_SDIO_PRODUCT,
			 &sdio_product)) {
		device_printf(dev, "Cannot get vendor/product from the bus!\n");
		return (-1);
	}
	if (sdio_vendor == 0x02DF && sdio_product == 0x9119)
		return (0);
	else
		return (-1);
}

static int
sdiowl_attach(device_t dev)
{
	struct sdiowl_softc *sc;
//	char unit;

	sc = device_get_softc(dev);
	sc->dev = dev;
	SDIOWL_LOCK_INIT(sc);

	sc->running = 1;

	return (0);
}

static int
sdiowl_detach(device_t dev)
{
	/* Not implemented yet */
	return (-1);
}

static device_method_t sdiowl_methods[] = {
	DEVMETHOD(device_probe, sdiowl_probe),
	DEVMETHOD(device_attach, sdiowl_attach),
	DEVMETHOD(device_detach, sdiowl_detach),
	DEVMETHOD_END
};

static driver_t sdiowl_driver = {
	"sdiowl",
	sdiowl_methods,
	sizeof(struct sdiowl_softc),
};
static devclass_t sdiowl_devclass;

DRIVER_MODULE(sdiowl, mmc, sdiowl_driver, sdiowl_devclass, NULL, NULL);
