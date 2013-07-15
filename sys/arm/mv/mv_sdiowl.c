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
#include <sys/linker.h>
#include <sys/firmware.h>

#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcvar.h>
#include <dev/mmc/mmcioreg.h>
#include <dev/mmc/mmciovar.h>

#include "mmcbus_if.h"

#define DN_LD_CARD_RDY		(1u << 0)
#define CARD_IO_READY		(1u << 3)

/* Host Control Registers */
/* Host Control Registers : Host interrupt RSR */
#define HOST_INT_RSR_REG	0x01
#define SDIO_INT_MASK		0x3F

/* Host Control Registers : I/O port 0 */
#define IO_PORT_0_REG		0x78
/* Host Control Registers : I/O port 1 */
#define IO_PORT_1_REG		0x79
/* Host Control Registers : I/O port 2 */
#define IO_PORT_2_REG		0x7A

/* Host F1 read base 0 */
#define HOST_F1_RD_BASE_0	0x0040
/* Host F1 read base 1 */
#define HOST_F1_RD_BASE_1	0x0041

/* Card Control Registers : Card status register */
#define CARD_STATUS_REG		0x30
/* Card Control Registers : Miscellaneous Configuration Register */
#define CARD_MISC_CFG_REG	0x6C

/* Misc. Config Register : Auto Re-enable interrupts */
#define AUTO_RE_ENABLE_INT	1 << 4

/* SD block size can not bigger than 64 due to buf size limit in firmware */
/* define SD block size for data Tx/Rx */
#define SDIO_BLOCK_SIZE		64


struct sdiowl_softc {
	device_t dev;
	struct mtx sc_mtx;
	int running;
	int sdio_function;
	uint32_t ioport;
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
		device_printf(dev, "Cannot get vendor/product/function from the bus!\n");
		return (-1);
	}

	if (sdio_vendor == 0x02DF && sdio_product == 0x9119)
		return (0);
	else
		return (-1);
}

static int
sdiowl_read_1(struct sdiowl_softc *sc, uint32_t reg, uint8_t *val) {
	return MMCBUS_IO_READ_1(device_get_parent(sc->dev), sc->dev,
	    reg, val);
}

static int
sdiowl_write_1(struct sdiowl_softc *sc, uint32_t reg, uint8_t val) {
	return MMCBUS_IO_WRITE_1(device_get_parent(sc->dev), sc->dev,
	    reg, val);
}

static int
sdiowl_attach(device_t dev)
{
	struct sdiowl_softc *sc;
	const struct firmware *fw;
	uint8_t funcs_enabled;
//	char unit;

	sc = device_get_softc(dev);
	sc->dev = dev;
	SDIOWL_LOCK_INIT(sc);
	sc->running = 1;
	BUS_READ_IVAR(device_get_parent(dev), dev, MMC_IVAR_SDIO_FUNCTION,
	    &sc->sdio_function);

	device_printf(dev, "Trying to load firmware...\n");
	fw = firmware_get("sdiowl_fw");
	if (!fw) {
		device_printf(dev, "Fail to load fw");
		return (-1);
	}

	device_printf(dev, "OK. name=%s data=%08X, size=%d, ver=%d\n",
		      fw->name, (uint32_t )fw->data, fw->datasize, fw->version);

	if (MMCBUS_IO_F0_READ_1(device_get_parent(dev), dev,
	    SD_IO_CCCR_FN_ENABLE, &funcs_enabled))
		return (-1);
	if (funcs_enabled | sc->sdio_function)
		device_printf(dev, "My function is enabled, good!\n");
	else {
		/* XXX Enable function from here? */
		device_printf(dev, "My func is NOT enabled, bad!\n");
		return (-1);
	}

	/* Try to set block size */
	device_printf(dev, "Setting block size to %d bytes\n", SDIO_BLOCK_SIZE);
	if(MMCBUS_IO_SET_BLOCK_SIZE(device_get_parent(dev), dev, SDIO_BLOCK_SIZE))
		return (-1);

	/* Get IO Port */
	uint8_t reg;
	if (sdiowl_read_1(sc, IO_PORT_0_REG, &reg))
		return (-1);
	sc->ioport = reg;
	if (sdiowl_read_1(sc, IO_PORT_1_REG, &reg))
		return (-1);
	sc->ioport = reg << 8;
	if (sdiowl_read_1(sc, IO_PORT_2_REG, &reg))
		return (-1);
	sc->ioport = reg << 16;
	device_printf(dev, "IO Port: 0x%08X\n", sc->ioport);

	/* Set Host interrupt reset to read to clear */
	if (!sdiowl_read_1(sc, HOST_INT_RSR_REG, &reg))
		sdiowl_write_1(sc, HOST_INT_RSR_REG, reg | SDIO_INT_MASK);
	else
		return (-1);

	/* Dnld/Upld ready set to auto reset */
	if (!sdiowl_read_1(sc, CARD_MISC_CFG_REG, &reg))
		sdiowl_write_1(sc, CARD_MISC_CFG_REG, reg | AUTO_RE_ENABLE_INT);
	else
		return (-1);

	char data[256]; /* = 4 blocks */
	memset(data, 0, 256);
	if (MMCBUS_IO_READ_MULTI(device_get_parent(dev), dev, 0,
				 data, 256, 4)) {
		device_printf(dev, "Cannot read-multi\n");
	} else hexdump(data, 256, NULL, 0);

	/* Stop init here for now */
	return (-1);


	/* Now upload FW to the card */
	uint8_t status, base0, base1, tries;
	uint32_t len, txlen, tx_blocks, offset;

	offset = len = 0;
	do {
		for (tries = 0; tries < 200; tries ++) {
			/* Ask card about its status */
			if (sdiowl_read_1(sc, CARD_STATUS_REG, &status))
				return (-1);
			device_printf(dev, "Card status: %d\n", status);
			if (status | CARD_IO_READY | DN_LD_CARD_RDY)
				device_printf(dev, "Card ready to accept FW\n");
			else {
				device_printf(dev, "Card NOT ready to accept FW\n");
				return (-1);
			}

			/* XXX: This doesn't work and returns 0's :( */
			if (sdiowl_read_1(sc, HOST_F1_RD_BASE_0, &base0) > 0 ||
			    sdiowl_read_1(sc, HOST_F1_RD_BASE_1, &base1) > 0) {
				device_printf(dev, "Err while reading BASEx\n");
				return (-1);
			}

			len = (((base1 & 0xff) << 8) | (base0 & 0xff));
			device_printf(dev, "len = %d [base0=%d, base1=%d]\n", len, base0, base1);
			if (len)
				break;
			/* Sleep here */
		}

		if (!len)
			break;
		txlen = len;

		if (fw->datasize - offset < txlen)
			txlen = fw->datasize - offset;

		tx_blocks = (txlen + SDIO_BLOCK_SIZE - 1)
			/ SDIO_BLOCK_SIZE;

/*
		ret = mwifiex_write_data_sync(adapter, fwbuf, tx_blocks *
					      MWIFIEX_SDIO_BLOCK_SIZE,
					      adapter->ioport);

This uses FIFO write -- need to implement the bus method for this.
*/
	} while (true);


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
