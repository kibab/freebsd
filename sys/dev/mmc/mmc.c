/*-
 * Copyright (c) 2006 Bernd Walter.  All rights reserved.
 * Copyright (c) 2006 M. Warner Losh.  All rights reserved.
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
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/sysctl.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmcvar.h>
#include <dev/mmc/mmcioreg.h>
#include <dev/mmc/mmciovar.h>

#include "mmcbr_if.h"
#include "mmcbus_if.h"

struct mmc_softc {
	device_t dev;
	struct mtx sc_mtx;
	struct intr_config_hook config_intrhook;
	device_t owner;
	uint32_t __sdio_rca; /* XXX Temp; for testng only */
	uint32_t last_rca;
	uint8_t sdio_nfunc;
	u_char sdio_bus_width;
	uint8_t sdio_support_hs;
	struct resource	*sdio_irq_res;
	void *sdio_irq_cookiep;
	driver_intr_t *sdio_isrs[7];
	void *sdio_isr_args[7];
	struct sdio_function sdio_func0;
	STAILQ_HEAD(, sdio_function) sdiof_head;
};

/*
 * Per-card data
 */
struct mmc_ivars {
	uint32_t raw_cid[4];	/* Raw bits of the CID */
	uint32_t raw_csd[4];	/* Raw bits of the CSD */
	uint32_t raw_scr[2];	/* Raw bits of the SCR */
	uint8_t raw_ext_csd[512];	/* Raw bits of the EXT_CSD */
	uint32_t raw_sd_status[16];	/* Raw bits of the SD_STATUS */
	uint16_t rca;
	enum mmc_card_mode mode;
	struct mmc_cid cid;	/* cid decoded */
	struct mmc_csd csd;	/* csd decoded */
	struct mmc_scr scr;	/* scr decoded */
	struct mmc_sd_status sd_status;	/* SD_STATUS decoded */
	u_char read_only;	/* True when the device is read-only */
	u_char bus_width;	/* Bus width to use */
	u_char timing;		/* Bus timing support */
	u_char high_cap;	/* High Capacity card (block addressed) */
	uint32_t sec_count;	/* Card capacity in 512byte blocks */
	uint32_t tran_speed;	/* Max speed in normal mode */
	uint32_t hs_tran_speed;	/* Max speed in high speed mode */
	uint32_t erase_sector;	/* Card native erase sector size */
	char card_id_string[64];/* Formatted CID info (serial, MFG, etc) */
	struct sdio_function *sdiof;
};

#define CMD_RETRIES	3

#define	CARD_ID_FREQUENCY 400000 /* Spec requires 400kHz max during ID phase. */

static SYSCTL_NODE(_hw, OID_AUTO, mmc, CTLFLAG_RD, NULL, "mmc driver");

static int mmc_debug;
SYSCTL_INT(_hw_mmc, OID_AUTO, debug, CTLFLAG_RW, &mmc_debug, 0, "Debug level");

/* bus entry points */
static int mmc_acquire_bus(device_t busdev, device_t dev);
static int mmc_attach(device_t dev);
static int mmc_child_location_str(device_t dev, device_t child, char *buf,
    size_t buflen);
static int mmc_detach(device_t dev);
static int mmc_probe(device_t dev);
static int mmc_read_ivar(device_t bus, device_t child, int which,
    uintptr_t *result);
static int mmc_release_bus(device_t busdev, device_t dev);
static int mmc_resume(device_t dev);
static int mmc_suspend(device_t dev);
static int mmc_wait_for_request(device_t brdev, device_t reqdev,
    struct mmc_request *req);
static int mmc_write_ivar(device_t bus, device_t child, int which,
    uintptr_t value);

static struct resource * mmc_alloc_resource(device_t dev, device_t child,
					    int type, int *rid,
					    u_long start, u_long end,
					    u_long count, u_int flags);

static int mmc_setup_intr(device_t dev, device_t child, struct resource *irq,
			  int flags, driver_filter_t *filt,
			  driver_intr_t *function, void *argument,
			  void **cookiep);

#define MMC_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	MMC_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define MMC_LOCK_INIT(_sc)					\
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->dev),	\
	    "mmc", MTX_DEF)
#define MMC_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);
#define MMC_ASSERT_LOCKED(_sc)	mtx_assert(&_sc->sc_mtx, MA_OWNED);
#define MMC_ASSERT_UNLOCKED(_sc) mtx_assert(&_sc->sc_mtx, MA_NOTOWNED);

static int mmc_all_send_cid(struct mmc_softc *sc, uint32_t *rawcid);
static void mmc_app_decode_scr(uint32_t *raw_scr, struct mmc_scr *scr);
static void mmc_app_decode_sd_status(uint32_t *raw_sd_status,
    struct mmc_sd_status *sd_status);
static int mmc_app_sd_status(struct mmc_softc *sc, uint16_t rca,
    uint32_t *rawsdstatus);
static int mmc_app_send_scr(struct mmc_softc *sc, uint16_t rca,
    uint32_t *rawscr);
static int mmc_calculate_clock(struct mmc_softc *sc);
static void mmc_decode_cid_mmc(uint32_t *raw_cid, struct mmc_cid *cid);
static void mmc_decode_cid_sd(uint32_t *raw_cid, struct mmc_cid *cid);
static void mmc_decode_csd_mmc(uint32_t *raw_csd, struct mmc_csd *csd);
static void mmc_decode_csd_sd(uint32_t *raw_csd, struct mmc_csd *csd);
static void mmc_delayed_attach(void *xsc);
static int mmc_delete_cards(struct mmc_softc *sc);
static void mmc_discover_cards(struct mmc_softc *sc);
static void mmc_format_card_id_string(struct mmc_ivars *ivar);
static void mmc_go_discovery(struct mmc_softc *sc);
static uint32_t mmc_get_bits(uint32_t *bits, int bit_len, int start,
    int size);
static int mmc_highest_voltage(uint32_t ocr);
static void mmc_idle_cards(struct mmc_softc *sc);
static int mmc_io_func_enable(struct mmc_softc *sc, uint32_t fn);
static uint8_t mmc_io_read_1(struct mmc_softc *sc, uint32_t fn, uint32_t adr);
static int mmc_io_rw_direct(struct mmc_softc *sc, int wr, uint32_t fn,
    uint32_t adr, uint8_t *data);
static void mmc_ms_delay(int ms);
static void mmc_log_card(device_t dev, struct mmc_ivars *ivar, int newcard);
static void mmc_power_down(struct mmc_softc *sc);
static void mmc_power_up(struct mmc_softc *sc);
static int mmc_probe_sdio(struct mmc_softc *sc, uint32_t ocr, uint32_t *rocr,
    uint8_t *nfunc, uint8_t *mem_present);
static void mmc_rescan_cards(struct mmc_softc *sc);
static void mmc_scan(struct mmc_softc *sc);
static int mmc_sd_switch(struct mmc_softc *sc, uint8_t mode, uint8_t grp,
    uint8_t value, uint8_t *res);
static int mmc_select_card(struct mmc_softc *sc, uint16_t rca);
static uint32_t mmc_select_vdd(struct mmc_softc *sc, uint32_t ocr);
static int mmc_send_app_op_cond(struct mmc_softc *sc, uint32_t ocr,
    uint32_t *rocr);
static int mmc_send_csd(struct mmc_softc *sc, uint16_t rca, uint32_t *rawcsd);
static int mmc_send_ext_csd(struct mmc_softc *sc, uint8_t *rawextcsd);
static int mmc_send_if_cond(struct mmc_softc *sc, uint8_t vhs);
static int mmc_send_op_cond(struct mmc_softc *sc, uint32_t ocr,
    uint32_t *rocr);
static int mmc_send_relative_addr(struct mmc_softc *sc, uint32_t *resp);
static int mmc_send_status(struct mmc_softc *sc, uint16_t rca,
    uint32_t *status);
static int mmc_set_blocklen(struct mmc_softc *sc, uint32_t len);
static int mmc_set_card_bus_width(struct mmc_softc *sc, uint16_t rca,
    int width);
static int mmc_set_relative_addr(struct mmc_softc *sc, uint16_t resp);
static int mmc_set_timing(struct mmc_softc *sc, int timing);
static int mmc_switch(struct mmc_softc *sc, uint8_t set, uint8_t index,
    uint8_t value);
static int mmc_test_bus_width(struct mmc_softc *sc);
static int mmc_wait_for_app_cmd(struct mmc_softc *sc, uint32_t rca,
    struct mmc_command *cmd, int retries);
static int mmc_wait_for_cmd(struct mmc_softc *sc, struct mmc_command *cmd,
    int retries);
static int mmc_wait_for_command(struct mmc_softc *sc, uint32_t opcode,
    uint32_t arg, uint32_t flags, uint32_t *resp, int retries);
static int mmc_wait_for_req(struct mmc_softc *sc, struct mmc_request *req);
static void mmc_wakeup(struct mmc_request *req);

static void
mmc_ms_delay(int ms)
{

	DELAY(1000 * ms);	/* XXX BAD */
}

static int
mmc_probe(device_t dev)
{

	device_set_desc(dev, "MMC/SD bus");
	return (0);
}

static int
mmc_attach(device_t dev)
{
	struct mmc_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	MMC_LOCK_INIT(sc);

	STAILQ_INIT(&sc->sdiof_head);

	/* We'll probe and attach our children later, but before / mount */
	sc->config_intrhook.ich_func = mmc_delayed_attach;
	sc->config_intrhook.ich_arg = sc;
	if (config_intrhook_establish(&sc->config_intrhook) != 0)
		device_printf(dev, "config_intrhook_establish failed\n");
	return (0);
}

static int
mmc_detach(device_t dev)
{
	struct mmc_softc *sc = device_get_softc(dev);
	int err;

	if ((err = mmc_delete_cards(sc)) != 0)
		return (err);
	mmc_power_down(sc);
	MMC_LOCK_DESTROY(sc);

	return (0);
}

static int
mmc_suspend(device_t dev)
{
	struct mmc_softc *sc = device_get_softc(dev);
	int err;

	err = bus_generic_suspend(dev);
	if (err)
	        return (err);
	mmc_power_down(sc);
	return (0);
}

static int
mmc_resume(device_t dev)
{
	struct mmc_softc *sc = device_get_softc(dev);

	mmc_scan(sc);
	return (bus_generic_resume(dev));
}

static int
mmc_acquire_bus(device_t busdev, device_t dev)
{
	struct mmc_softc *sc;
	struct mmc_ivars *ivar;
	int err;
	int rca;

	err = MMCBR_ACQUIRE_HOST(device_get_parent(busdev), busdev);
	if (err)
		return (err);
	sc = device_get_softc(busdev);
	MMC_LOCK(sc);
	if (sc->owner)
		panic("mmc: host bridge didn't serialize us.");
	sc->owner = dev;
	MMC_UNLOCK(sc);

	if (busdev != dev) {
		/*
		 * Keep track of the last rca that we've selected.  If
		 * we're asked to do it again, don't.  We never
		 * unselect unless the bus code itself wants the mmc
		 * bus, and constantly reselecting causes problems.
		 */
		rca = mmc_get_rca(dev);
		if (sc->last_rca != rca) {
			mmc_select_card(sc, rca);
			sc->last_rca = rca;
			/* Prepare bus width for the new card. */
			ivar = device_get_ivars(dev);
			if (bootverbose || mmc_debug) {
				device_printf(busdev,
				    "setting bus width to %d bits\n",
				    (ivar->bus_width == bus_width_4) ? 4 :
				    (ivar->bus_width == bus_width_8) ? 8 : 1);
			}
			mmc_set_card_bus_width(sc, rca, ivar->bus_width);
			mmcbr_set_bus_width(busdev, ivar->bus_width);
			mmcbr_update_ios(busdev);
		}
	} else {
		/*
		 * If there's a card selected, stand down.
		 */
		if (sc->last_rca != 0) {
			mmc_select_card(sc, 0);
			sc->last_rca = 0;
		}
	}

	return (0);
}

static int
mmc_release_bus(device_t busdev, device_t dev)
{
	struct mmc_softc *sc;
	int err;

	sc = device_get_softc(busdev);

	MMC_LOCK(sc);
	if (!sc->owner)
		panic("mmc: releasing unowned bus.");
	if (sc->owner != dev)
		panic("mmc: you don't own the bus.  game over.");
	MMC_UNLOCK(sc);
	err = MMCBR_RELEASE_HOST(device_get_parent(busdev), busdev);
	if (err)
		return (err);
	MMC_LOCK(sc);
	sc->owner = NULL;
	MMC_UNLOCK(sc);
	return (0);
}

static uint32_t
mmc_select_vdd(struct mmc_softc *sc, uint32_t ocr)
{

	return (ocr & MMC_OCR_VOLTAGE);
}

static int
mmc_highest_voltage(uint32_t ocr)
{
	int i;

	for (i = MMC_OCR_MAX_VOLTAGE_SHIFT;
	    i >= MMC_OCR_MIN_VOLTAGE_SHIFT; i--)
		if (ocr & (1 << i))
			return (i);
	return (-1);
}

static void
mmc_wakeup(struct mmc_request *req)
{
	struct mmc_softc *sc;

	sc = (struct mmc_softc *)req->done_data;
	MMC_LOCK(sc);
	req->flags |= MMC_REQ_DONE;
	MMC_UNLOCK(sc);
	wakeup(req);
}

static int
mmc_wait_for_req(struct mmc_softc *sc, struct mmc_request *req)
{

	req->done = mmc_wakeup;
	req->done_data = sc;
	if (mmc_debug > 1) {
		device_printf(sc->dev, "REQUEST: CMD%d arg %#x flags %#x",
		    req->cmd->opcode, req->cmd->arg, req->cmd->flags);
		if (req->cmd->data) {
			printf(" data %d\n", (int)req->cmd->data->len);
		} else
			printf("\n");
	}
	MMCBR_REQUEST(device_get_parent(sc->dev), sc->dev, req);
	MMC_LOCK(sc);
	while ((req->flags & MMC_REQ_DONE) == 0)
		msleep(req, &sc->sc_mtx, 0, "mmcreq", 0);
	MMC_UNLOCK(sc);
	if (mmc_debug > 2 || (mmc_debug > 0 && req->cmd->error != MMC_ERR_NONE))
		device_printf(sc->dev, "CMD%d RESULT: %d\n", 
		    req->cmd->opcode, req->cmd->error);
	return (0);
}

static int
mmc_wait_for_request(device_t brdev, device_t reqdev, struct mmc_request *req)
{
	struct mmc_softc *sc = device_get_softc(brdev);

	return (mmc_wait_for_req(sc, req));
}

static int
mmc_wait_for_cmd(struct mmc_softc *sc, struct mmc_command *cmd, int retries)
{
	struct mmc_request mreq;
	int err;

	do {
		memset(&mreq, 0, sizeof(mreq));
		memset(cmd->resp, 0, sizeof(cmd->resp));
		cmd->retries = 0; /* Retries done here, not in hardware. */
		cmd->mrq = &mreq;
		mreq.cmd = cmd;
		if (mmc_wait_for_req(sc, &mreq) != 0)
			err = MMC_ERR_FAILED;
		else
			err = cmd->error;
	} while (err != MMC_ERR_NONE && retries-- > 0);

	return (err);
}

static int
mmc_wait_for_app_cmd(struct mmc_softc *sc, uint32_t rca,
    struct mmc_command *cmd, int retries)
{
	struct mmc_command appcmd;
	int err;

	do {
		memset(&appcmd, 0, sizeof(appcmd));
		appcmd.opcode = MMC_APP_CMD;
		appcmd.arg = rca << 16;
		appcmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
		appcmd.data = NULL;
		if (mmc_wait_for_cmd(sc, &appcmd, 0) != 0)
			err = MMC_ERR_FAILED;
		else
			err = appcmd.error;
		if (err == MMC_ERR_NONE) {
			if (!(appcmd.resp[0] & R1_APP_CMD))
				err = MMC_ERR_FAILED;
			else if (mmc_wait_for_cmd(sc, cmd, 0) != 0)
				err = MMC_ERR_FAILED;
			else
				err = cmd->error;
		}
	} while (err != MMC_ERR_NONE && retries-- > 0);

	return (err);
}

static int
mmc_wait_for_command(struct mmc_softc *sc, uint32_t opcode,
    uint32_t arg, uint32_t flags, uint32_t *resp, int retries)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = flags;
	cmd.data = NULL;
	err = mmc_wait_for_cmd(sc, &cmd, retries);
	if (err)
		return (err);
	if (resp) {
		if (flags & MMC_RSP_136)
			memcpy(resp, cmd.resp, 4 * sizeof(uint32_t));
		else
			*resp = cmd.resp[0];
	}
	return (0);
}

/* CMD0 */
static void
mmc_idle_cards(struct mmc_softc *sc)
{
	device_t dev;
	struct mmc_command cmd;
	
	dev = sc->dev;
	mmcbr_set_chip_select(dev, cs_high);
	mmcbr_update_ios(dev);
	mmc_ms_delay(1);

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_GO_IDLE_STATE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_BC;
	cmd.data = NULL;
	mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	mmc_ms_delay(1);

	mmcbr_set_chip_select(dev, cs_dontcare);
	mmcbr_update_ios(dev);
	mmc_ms_delay(1);
}

/* CMD41 -> CMD55 */
static int
mmc_send_app_op_cond(struct mmc_softc *sc, uint32_t ocr, uint32_t *rocr)
{
	struct mmc_command cmd;
	int err = MMC_ERR_NONE, i;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = ACMD_SD_SEND_OP_COND;
	cmd.arg = ocr;
	cmd.flags = MMC_RSP_R3 | MMC_CMD_BCR;
	cmd.data = NULL;

	for (i = 0; i < 1000; i++) {
		err = mmc_wait_for_app_cmd(sc, 0, &cmd, CMD_RETRIES);
		if (err != MMC_ERR_NONE)
			break;
		if ((cmd.resp[0] & MMC_OCR_CARD_BUSY) ||
		    (ocr & MMC_OCR_VOLTAGE) == 0)
			break;
		err = MMC_ERR_TIMEOUT;
		mmc_ms_delay(10);
	}
	if (rocr && err == MMC_ERR_NONE)
		*rocr = cmd.resp[0];
	return (err);
}

/* CMD1 */
static int
mmc_send_op_cond(struct mmc_softc *sc, uint32_t ocr, uint32_t *rocr)
{
	struct mmc_command cmd;
	int err = MMC_ERR_NONE, i;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_SEND_OP_COND;
	cmd.arg = ocr;
	cmd.flags = MMC_RSP_R3 | MMC_CMD_BCR;
	cmd.data = NULL;

	for (i = 0; i < 1000; i++) {
		err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
		if (err != MMC_ERR_NONE)
			break;
		if ((cmd.resp[0] & MMC_OCR_CARD_BUSY) ||
		    (ocr & MMC_OCR_VOLTAGE) == 0)
			break;
		err = MMC_ERR_TIMEOUT;
		mmc_ms_delay(10);
	}
	if (rocr && err == MMC_ERR_NONE)
		*rocr = cmd.resp[0];
	return (err);
}

/* CMD8 */
static int
mmc_send_if_cond(struct mmc_softc *sc, uint8_t vhs)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = SD_SEND_IF_COND;
	cmd.arg = (vhs << 8) + 0xAA;
	cmd.flags = MMC_RSP_R7 | MMC_CMD_BCR;
	cmd.data = NULL;

	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	return (err);
}

static void
mmc_power_up(struct mmc_softc *sc)
{
	device_t dev;

	dev = sc->dev;
	mmcbr_set_vdd(dev, mmc_highest_voltage(mmcbr_get_host_ocr(dev)));
	mmcbr_set_bus_mode(dev, opendrain);
	mmcbr_set_chip_select(dev, cs_dontcare);
	mmcbr_set_bus_width(dev, bus_width_1);
	mmcbr_set_power_mode(dev, power_up);
	mmcbr_set_clock(dev, 0);
	mmcbr_update_ios(dev);
	mmc_ms_delay(1);

	mmcbr_set_clock(dev, CARD_ID_FREQUENCY);
	mmcbr_set_timing(dev, bus_timing_normal);
	mmcbr_set_power_mode(dev, power_on);
	mmcbr_update_ios(dev);
	mmc_ms_delay(2);
}

static void
mmc_power_down(struct mmc_softc *sc)
{
	device_t dev = sc->dev;

	mmcbr_set_bus_mode(dev, opendrain);
	mmcbr_set_chip_select(dev, cs_dontcare);
	mmcbr_set_bus_width(dev, bus_width_1);
	mmcbr_set_power_mode(dev, power_off);
	mmcbr_set_clock(dev, 0);
	mmcbr_set_timing(dev, bus_timing_normal);
	mmcbr_update_ios(dev);
}

/* CMD7 */
static int
mmc_select_card(struct mmc_softc *sc, uint16_t rca)
{
	int flags;

	flags = (rca ? MMC_RSP_R1B : MMC_RSP_NONE) | MMC_CMD_AC;
	return (mmc_wait_for_command(sc, MMC_SELECT_CARD, (uint32_t)rca << 16,
	    flags, NULL, CMD_RETRIES));
}

static int
mmc_switch(struct mmc_softc *sc, uint8_t set, uint8_t index, uint8_t value)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_SWITCH_FUNC;
	cmd.arg = (MMC_SWITCH_FUNC_WR << 24) |
	    (index << 16) |
	    (value << 8) |
	    set;
	cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	cmd.data = NULL;
	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	return (err);
}

static int
mmc_sd_switch(struct mmc_softc *sc, uint8_t mode, uint8_t grp, uint8_t value,
    uint8_t *res)
{
	int err;
	struct mmc_command cmd;
	struct mmc_data data;

	memset(&cmd, 0, sizeof(cmd));
	memset(&data, 0, sizeof(data));
	memset(res, 0, 64);

	cmd.opcode = SD_SWITCH_FUNC;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	cmd.arg = mode << 31;			/* 0 - check, 1 - set */
	cmd.arg |= 0x00FFFFFF;
	cmd.arg &= ~(0xF << (grp * 4));
	cmd.arg |= value << (grp * 4);
	cmd.data = &data;

	data.data = res;
	data.len = 64;
	data.flags = MMC_DATA_READ;

	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	return (err);
}

static int
mmc_set_card_bus_width(struct mmc_softc *sc, uint16_t rca, int width)
{
	struct mmc_command cmd;
	int err;
	uint8_t	value;

	if (mmcbr_get_mode(sc->dev) == mode_sd) {
		memset(&cmd, 0, sizeof(cmd));
		cmd.opcode = ACMD_SET_CLR_CARD_DETECT;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
		cmd.arg = SD_CLR_CARD_DETECT;
		err = mmc_wait_for_app_cmd(sc, rca, &cmd, CMD_RETRIES);
		if (err != 0)
			return (err);
		memset(&cmd, 0, sizeof(cmd));
		cmd.opcode = ACMD_SET_BUS_WIDTH;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
		switch (width) {
		case bus_width_1:
			cmd.arg = SD_BUS_WIDTH_1;
			break;
		case bus_width_4:
			cmd.arg = SD_BUS_WIDTH_4;
			break;
		default:
			return (MMC_ERR_INVALID);
		}
		err = mmc_wait_for_app_cmd(sc, rca, &cmd, CMD_RETRIES);
	} else {
		switch (width) {
		case bus_width_1:
			value = EXT_CSD_BUS_WIDTH_1;
			break;
		case bus_width_4:
			value = EXT_CSD_BUS_WIDTH_4;
			break;
		case bus_width_8:
			value = EXT_CSD_BUS_WIDTH_8;
			break;
		default:
			return (MMC_ERR_INVALID);
		}
		err = mmc_switch(sc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_BUS_WIDTH,
		    value);
	}
	return (err);
}

static int
mmc_set_timing(struct mmc_softc *sc, int timing)
{
	int err;
	uint8_t	value;
	u_char switch_res[64];

	switch (timing) {
	case bus_timing_normal:
		value = 0;
		break;
	case bus_timing_hs:
		value = 1;
		break;
	default:
		return (MMC_ERR_INVALID);
	}
	if (mmcbr_get_mode(sc->dev) == mode_sd)
		err = mmc_sd_switch(sc, SD_SWITCH_MODE_SET, SD_SWITCH_GROUP1,
		    value, switch_res);
	else
		err = mmc_switch(sc, EXT_CSD_CMD_SET_NORMAL,
		    EXT_CSD_HS_TIMING, value);
	return (err);
}

static int
mmc_test_bus_width(struct mmc_softc *sc)
{
	struct mmc_command cmd;
	struct mmc_data data;
	int err;
	uint8_t buf[8];
	uint8_t	p8[8] =   { 0x55, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t	p8ok[8] = { 0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint8_t	p4[4] =   { 0x5A, 0x00, 0x00, 0x00, };
	uint8_t	p4ok[4] = { 0xA5, 0x00, 0x00, 0x00, };

	if (mmcbr_get_caps(sc->dev) & MMC_CAP_8_BIT_DATA) {
		mmcbr_set_bus_width(sc->dev, bus_width_8);
		mmcbr_update_ios(sc->dev);

		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));
		cmd.opcode = MMC_BUSTEST_W;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.data = &data;

		data.data = p8;
		data.len = 8;
		data.flags = MMC_DATA_WRITE;
		mmc_wait_for_cmd(sc, &cmd, 0);
		
		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));
		cmd.opcode = MMC_BUSTEST_R;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.data = &data;

		data.data = buf;
		data.len = 8;
		data.flags = MMC_DATA_READ;
		err = mmc_wait_for_cmd(sc, &cmd, 0);
		
		mmcbr_set_bus_width(sc->dev, bus_width_1);
		mmcbr_update_ios(sc->dev);

		if (err == MMC_ERR_NONE && memcmp(buf, p8ok, 8) == 0)
			return (bus_width_8);
	}

	if (mmcbr_get_caps(sc->dev) & MMC_CAP_4_BIT_DATA) {
		mmcbr_set_bus_width(sc->dev, bus_width_4);
		mmcbr_update_ios(sc->dev);

		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));
		cmd.opcode = MMC_BUSTEST_W;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.data = &data;

		data.data = p4;
		data.len = 4;
		data.flags = MMC_DATA_WRITE;
		mmc_wait_for_cmd(sc, &cmd, 0);
		
		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));
		cmd.opcode = MMC_BUSTEST_R;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.data = &data;

		data.data = buf;
		data.len = 4;
		data.flags = MMC_DATA_READ;
		err = mmc_wait_for_cmd(sc, &cmd, 0);

		mmcbr_set_bus_width(sc->dev, bus_width_1);
		mmcbr_update_ios(sc->dev);

		if (err == MMC_ERR_NONE && memcmp(buf, p4ok, 4) == 0)
			return (bus_width_4);
	}
	return (bus_width_1);
}

static uint32_t
mmc_get_bits(uint32_t *bits, int bit_len, int start, int size)
{
	const int i = (bit_len / 32) - (start / 32) - 1;
	const int shift = start & 31;
	uint32_t retval = bits[i] >> shift;
	if (size + shift > 32)
		retval |= bits[i - 1] << (32 - shift);
	return (retval & ((1llu << size) - 1));
}

static void
mmc_decode_cid_sd(uint32_t *raw_cid, struct mmc_cid *cid)
{
	int i;

	/* There's no version info, so we take it on faith */
	memset(cid, 0, sizeof(*cid));
	cid->mid = mmc_get_bits(raw_cid, 128, 120, 8);
	cid->oid = mmc_get_bits(raw_cid, 128, 104, 16);
	for (i = 0; i < 5; i++)
		cid->pnm[i] = mmc_get_bits(raw_cid, 128, 96 - i * 8, 8);
	cid->pnm[5] = 0;
	cid->prv = mmc_get_bits(raw_cid, 128, 56, 8);
	cid->psn = mmc_get_bits(raw_cid, 128, 24, 32);
	cid->mdt_year = mmc_get_bits(raw_cid, 128, 12, 8) + 2000;
	cid->mdt_month = mmc_get_bits(raw_cid, 128, 8, 4);
}

static void
mmc_decode_cid_mmc(uint32_t *raw_cid, struct mmc_cid *cid)
{
	int i;

	/* There's no version info, so we take it on faith */
	memset(cid, 0, sizeof(*cid));
	cid->mid = mmc_get_bits(raw_cid, 128, 120, 8);
	cid->oid = mmc_get_bits(raw_cid, 128, 104, 8);
	for (i = 0; i < 6; i++)
		cid->pnm[i] = mmc_get_bits(raw_cid, 128, 96 - i * 8, 8);
	cid->pnm[6] = 0;
	cid->prv = mmc_get_bits(raw_cid, 128, 48, 8);
	cid->psn = mmc_get_bits(raw_cid, 128, 16, 32);
	cid->mdt_month = mmc_get_bits(raw_cid, 128, 12, 4);
	cid->mdt_year = mmc_get_bits(raw_cid, 128, 8, 4) + 1997;
}

static void
mmc_format_card_id_string(struct mmc_ivars *ivar)
{
	char oidstr[8];
	uint8_t c1;
	uint8_t c2;

	/*
	 * Format a card ID string for use by the mmcsd driver, it's what
	 * appears between the <> in the following:
	 * mmcsd0: 968MB <SD SD01G 8.0 SN 2686905 Mfg 08/2008 by 3 TN> at mmc0
	 * 22.5MHz/4bit/128-block
	 *
	 * The card_id_string in mmc_ivars is currently allocated as 64 bytes,
	 * and our max formatted length is currently 55 bytes if every field
	 * contains the largest value.
	 *
	 * Sometimes the oid is two printable ascii chars; when it's not,
	 * format it as 0xnnnn instead.
	 */
	c1 = (ivar->cid.oid >> 8) & 0x0ff;
	c2 = ivar->cid.oid & 0x0ff;
	if (c1 > 0x1f && c1 < 0x7f && c2 > 0x1f && c2 < 0x7f)
		snprintf(oidstr, sizeof(oidstr), "%c%c", c1, c2);
	else
		snprintf(oidstr, sizeof(oidstr), "0x%04x", ivar->cid.oid);
	snprintf(ivar->card_id_string, sizeof(ivar->card_id_string),
	    "%s%s %s %d.%d SN %u MFG %02d/%04d by %d %s",
	    ivar->mode == mode_sd ? "SD" : "MMC", ivar->high_cap ? "HC" : "",
	    ivar->cid.pnm, ivar->cid.prv >> 4, ivar->cid.prv & 0x0f,
	    ivar->cid.psn, ivar->cid.mdt_month, ivar->cid.mdt_year,
	    ivar->cid.mid, oidstr);
}

static const int exp[8] = {
	1, 10, 100, 1000, 10000, 100000, 1000000, 10000000
};

static const int mant[16] = {
	0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80
};

static const int cur_min[8] = {
	500, 1000, 5000, 10000, 25000, 35000, 60000, 100000
};

static const int cur_max[8] = {
	1000, 5000, 10000, 25000, 35000, 45000, 800000, 200000
};

static void
mmc_decode_csd_sd(uint32_t *raw_csd, struct mmc_csd *csd)
{
	int v;
	int m;
	int e;

	memset(csd, 0, sizeof(*csd));
	csd->csd_structure = v = mmc_get_bits(raw_csd, 128, 126, 2);
	if (v == 0) {
		m = mmc_get_bits(raw_csd, 128, 115, 4);
		e = mmc_get_bits(raw_csd, 128, 112, 3);
		csd->tacc = (exp[e] * mant[m] + 9) / 10;
		csd->nsac = mmc_get_bits(raw_csd, 128, 104, 8) * 100;
		m = mmc_get_bits(raw_csd, 128, 99, 4);
		e = mmc_get_bits(raw_csd, 128, 96, 3);
		csd->tran_speed = exp[e] * 10000 * mant[m];
		csd->ccc = mmc_get_bits(raw_csd, 128, 84, 12);
		csd->read_bl_len = 1 << mmc_get_bits(raw_csd, 128, 80, 4);
		csd->read_bl_partial = mmc_get_bits(raw_csd, 128, 79, 1);
		csd->write_blk_misalign = mmc_get_bits(raw_csd, 128, 78, 1);
		csd->read_blk_misalign = mmc_get_bits(raw_csd, 128, 77, 1);
		csd->dsr_imp = mmc_get_bits(raw_csd, 128, 76, 1);
		csd->vdd_r_curr_min = cur_min[mmc_get_bits(raw_csd, 128, 59, 3)];
		csd->vdd_r_curr_max = cur_max[mmc_get_bits(raw_csd, 128, 56, 3)];
		csd->vdd_w_curr_min = cur_min[mmc_get_bits(raw_csd, 128, 53, 3)];
		csd->vdd_w_curr_max = cur_max[mmc_get_bits(raw_csd, 128, 50, 3)];
		m = mmc_get_bits(raw_csd, 128, 62, 12);
		e = mmc_get_bits(raw_csd, 128, 47, 3);
		csd->capacity = ((1 + m) << (e + 2)) * csd->read_bl_len;
		csd->erase_blk_en = mmc_get_bits(raw_csd, 128, 46, 1);
		csd->erase_sector = mmc_get_bits(raw_csd, 128, 39, 7) + 1;
		csd->wp_grp_size = mmc_get_bits(raw_csd, 128, 32, 7);
		csd->wp_grp_enable = mmc_get_bits(raw_csd, 128, 31, 1);
		csd->r2w_factor = 1 << mmc_get_bits(raw_csd, 128, 26, 3);
		csd->write_bl_len = 1 << mmc_get_bits(raw_csd, 128, 22, 4);
		csd->write_bl_partial = mmc_get_bits(raw_csd, 128, 21, 1);
	} else if (v == 1) {
		m = mmc_get_bits(raw_csd, 128, 115, 4);
		e = mmc_get_bits(raw_csd, 128, 112, 3);
		csd->tacc = (exp[e] * mant[m] + 9) / 10;
		csd->nsac = mmc_get_bits(raw_csd, 128, 104, 8) * 100;
		m = mmc_get_bits(raw_csd, 128, 99, 4);
		e = mmc_get_bits(raw_csd, 128, 96, 3);
		csd->tran_speed = exp[e] * 10000 * mant[m];
		csd->ccc = mmc_get_bits(raw_csd, 128, 84, 12);
		csd->read_bl_len = 1 << mmc_get_bits(raw_csd, 128, 80, 4);
		csd->read_bl_partial = mmc_get_bits(raw_csd, 128, 79, 1);
		csd->write_blk_misalign = mmc_get_bits(raw_csd, 128, 78, 1);
		csd->read_blk_misalign = mmc_get_bits(raw_csd, 128, 77, 1);
		csd->dsr_imp = mmc_get_bits(raw_csd, 128, 76, 1);
		csd->capacity = ((uint64_t)mmc_get_bits(raw_csd, 128, 48, 22) + 1) *
		    512 * 1024;
		csd->erase_blk_en = mmc_get_bits(raw_csd, 128, 46, 1);
		csd->erase_sector = mmc_get_bits(raw_csd, 128, 39, 7) + 1;
		csd->wp_grp_size = mmc_get_bits(raw_csd, 128, 32, 7);
		csd->wp_grp_enable = mmc_get_bits(raw_csd, 128, 31, 1);
		csd->r2w_factor = 1 << mmc_get_bits(raw_csd, 128, 26, 3);
		csd->write_bl_len = 1 << mmc_get_bits(raw_csd, 128, 22, 4);
		csd->write_bl_partial = mmc_get_bits(raw_csd, 128, 21, 1);
	} else 
		panic("unknown SD CSD version");
}

static void
mmc_decode_csd_mmc(uint32_t *raw_csd, struct mmc_csd *csd)
{
	int m;
	int e;

	memset(csd, 0, sizeof(*csd));
	csd->csd_structure = mmc_get_bits(raw_csd, 128, 126, 2);
	csd->spec_vers = mmc_get_bits(raw_csd, 128, 122, 4);
	m = mmc_get_bits(raw_csd, 128, 115, 4);
	e = mmc_get_bits(raw_csd, 128, 112, 3);
	csd->tacc = exp[e] * mant[m] + 9 / 10;
	csd->nsac = mmc_get_bits(raw_csd, 128, 104, 8) * 100;
	m = mmc_get_bits(raw_csd, 128, 99, 4);
	e = mmc_get_bits(raw_csd, 128, 96, 3);
	csd->tran_speed = exp[e] * 10000 * mant[m];
	csd->ccc = mmc_get_bits(raw_csd, 128, 84, 12);
	csd->read_bl_len = 1 << mmc_get_bits(raw_csd, 128, 80, 4);
	csd->read_bl_partial = mmc_get_bits(raw_csd, 128, 79, 1);
	csd->write_blk_misalign = mmc_get_bits(raw_csd, 128, 78, 1);
	csd->read_blk_misalign = mmc_get_bits(raw_csd, 128, 77, 1);
	csd->dsr_imp = mmc_get_bits(raw_csd, 128, 76, 1);
	csd->vdd_r_curr_min = cur_min[mmc_get_bits(raw_csd, 128, 59, 3)];
	csd->vdd_r_curr_max = cur_max[mmc_get_bits(raw_csd, 128, 56, 3)];
	csd->vdd_w_curr_min = cur_min[mmc_get_bits(raw_csd, 128, 53, 3)];
	csd->vdd_w_curr_max = cur_max[mmc_get_bits(raw_csd, 128, 50, 3)];
	m = mmc_get_bits(raw_csd, 128, 62, 12);
	e = mmc_get_bits(raw_csd, 128, 47, 3);
	csd->capacity = ((1 + m) << (e + 2)) * csd->read_bl_len;
	csd->erase_blk_en = 0;
	csd->erase_sector = (mmc_get_bits(raw_csd, 128, 42, 5) + 1) *
	    (mmc_get_bits(raw_csd, 128, 37, 5) + 1);
	csd->wp_grp_size = mmc_get_bits(raw_csd, 128, 32, 5);
	csd->wp_grp_enable = mmc_get_bits(raw_csd, 128, 31, 1);
	csd->r2w_factor = 1 << mmc_get_bits(raw_csd, 128, 26, 3);
	csd->write_bl_len = 1 << mmc_get_bits(raw_csd, 128, 22, 4);
	csd->write_bl_partial = mmc_get_bits(raw_csd, 128, 21, 1);
}

static void
mmc_app_decode_scr(uint32_t *raw_scr, struct mmc_scr *scr)
{
	unsigned int scr_struct;

	memset(scr, 0, sizeof(*scr));

	scr_struct = mmc_get_bits(raw_scr, 64, 60, 4);
	if (scr_struct != 0) {
		printf("Unrecognised SCR structure version %d\n",
		    scr_struct);
		return;
	}
	scr->sda_vsn = mmc_get_bits(raw_scr, 64, 56, 4);
	scr->bus_widths = mmc_get_bits(raw_scr, 64, 48, 4);
}

static void
mmc_app_decode_sd_status(uint32_t *raw_sd_status,
    struct mmc_sd_status *sd_status)
{

	memset(sd_status, 0, sizeof(*sd_status));

	sd_status->bus_width = mmc_get_bits(raw_sd_status, 512, 510, 2);
	sd_status->secured_mode = mmc_get_bits(raw_sd_status, 512, 509, 1);
	sd_status->card_type = mmc_get_bits(raw_sd_status, 512, 480, 16);
	sd_status->prot_area = mmc_get_bits(raw_sd_status, 512, 448, 12);
	sd_status->speed_class = mmc_get_bits(raw_sd_status, 512, 440, 8);
	sd_status->perf_move = mmc_get_bits(raw_sd_status, 512, 432, 8);
	sd_status->au_size = mmc_get_bits(raw_sd_status, 512, 428, 4);
	sd_status->erase_size = mmc_get_bits(raw_sd_status, 512, 408, 16);
	sd_status->erase_timeout = mmc_get_bits(raw_sd_status, 512, 402, 6);
	sd_status->erase_offset = mmc_get_bits(raw_sd_status, 512, 400, 2);
}

/* CMD2 */
static int
mmc_all_send_cid(struct mmc_softc *sc, uint32_t *rawcid)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_ALL_SEND_CID;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_BCR;
	cmd.data = NULL;
	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	memcpy(rawcid, cmd.resp, 4 * sizeof(uint32_t));
	return (err);
}

static int
mmc_send_csd(struct mmc_softc *sc, uint16_t rca, uint32_t *rawcsd)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_SEND_CSD;
	cmd.arg = rca << 16;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_BCR;
	cmd.data = NULL;
	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	memcpy(rawcsd, cmd.resp, 4 * sizeof(uint32_t));
	return (err);
}

static int
mmc_app_send_scr(struct mmc_softc *sc, uint16_t rca, uint32_t *rawscr)
{
	int err;
	struct mmc_command cmd;
	struct mmc_data data;

	memset(&cmd, 0, sizeof(cmd));
	memset(&data, 0, sizeof(data));

	memset(rawscr, 0, 8);
	cmd.opcode = ACMD_SEND_SCR;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	cmd.arg = 0;
	cmd.data = &data;

	data.data = rawscr;
	data.len = 8;
	data.flags = MMC_DATA_READ;

	err = mmc_wait_for_app_cmd(sc, rca, &cmd, CMD_RETRIES);
	rawscr[0] = be32toh(rawscr[0]);
	rawscr[1] = be32toh(rawscr[1]);
	return (err);
}

static int
mmc_send_ext_csd(struct mmc_softc *sc, uint8_t *rawextcsd)
{
	int err;
	struct mmc_command cmd;
	struct mmc_data data;

	memset(&cmd, 0, sizeof(cmd));
	memset(&data, 0, sizeof(data));

	memset(rawextcsd, 0, 512);
	cmd.opcode = MMC_SEND_EXT_CSD;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	cmd.arg = 0;
	cmd.data = &data;

	data.data = rawextcsd;
	data.len = 512;
	data.flags = MMC_DATA_READ;

	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	return (err);
}

static int
mmc_app_sd_status(struct mmc_softc *sc, uint16_t rca, uint32_t *rawsdstatus)
{
	int err, i;
	struct mmc_command cmd;
	struct mmc_data data;

	memset(&cmd, 0, sizeof(cmd));
	memset(&data, 0, sizeof(data));

	memset(rawsdstatus, 0, 64);
	cmd.opcode = ACMD_SD_STATUS;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	cmd.arg = 0;
	cmd.data = &data;

	data.data = rawsdstatus;
	data.len = 64;
	data.flags = MMC_DATA_READ;

	err = mmc_wait_for_app_cmd(sc, rca, &cmd, CMD_RETRIES);
	for (i = 0; i < 16; i++)
	    rawsdstatus[i] = be32toh(rawsdstatus[i]);
	return (err);
}

static int
mmc_set_relative_addr(struct mmc_softc *sc, uint16_t resp)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_SET_RELATIVE_ADDR;
	cmd.arg = resp << 16;
	cmd.flags = MMC_RSP_R6 | MMC_CMD_BCR;
	cmd.data = NULL;
	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	return (err);
}

/* CMD3 */
static int
mmc_send_relative_addr(struct mmc_softc *sc, uint32_t *resp)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = SD_SEND_RELATIVE_ADDR;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R6 | MMC_CMD_BCR;
	cmd.data = NULL;
	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	*resp = cmd.resp[0];
	return (err);
}

/* CMD13 */
static int
mmc_send_status(struct mmc_softc *sc, uint16_t rca, uint32_t *status)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	cmd.data = NULL;
	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	*status = cmd.resp[0];
	return (err);
}

static int
mmc_set_blocklen(struct mmc_softc *sc, uint32_t len)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = len;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	cmd.data = NULL;
	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	return (err);
}

static void
mmc_log_card(device_t dev, struct mmc_ivars *ivar, int newcard)
{
	device_printf(dev, "Card at relative address 0x%04x%s:\n",
	    ivar->rca, newcard ? " added" : "");
	device_printf(dev, " card: %s\n", ivar->card_id_string);
	device_printf(dev, " bus: %ubit, %uMHz%s\n",
	    (ivar->bus_width == bus_width_1 ? 1 :
	    (ivar->bus_width == bus_width_4 ? 4 : 8)),
	    (ivar->timing == bus_timing_hs ?
		ivar->hs_tran_speed : ivar->tran_speed) / 1000000,
	    ivar->timing == bus_timing_hs ? ", high speed timing" : "");
	device_printf(dev, " memory: %u blocks, erase sector %u blocks%s\n",
	    ivar->sec_count, ivar->erase_sector,
	    ivar->read_only ? ", read-only" : "");
}

/*
 * Enables the given function on SDIO card.
 */
static int
mmc_io_func_enable(struct mmc_softc *sc, uint32_t fn)
{
	int err, i;
	uint8_t funcs;

	if (fn > sc->sdio_nfunc) {
		device_printf(sc->dev, "Invalid function to enable: %d\n", fn);
		return (MMC_ERR_INVALID);
	}

	funcs = mmc_io_read_1(sc, 0, SD_IO_CCCR_FN_READY);

	funcs |= 1 << fn;
	err = mmc_io_rw_direct(sc, 1, 0, SD_IO_CCCR_FN_ENABLE, &funcs);
	if (err != MMC_ERR_NONE) {
		device_printf(sc->dev, "Error writing SDIO func enable %d\n", err);
		return (err);
	}

	funcs = 0;
	for(i=0; i < 10; i++) {
		funcs = mmc_io_read_1(sc, 0, SD_IO_CCCR_FN_READY);

		if (funcs & (1 << fn))
			return 0;
		mmc_ms_delay(10);
	}

	device_printf(sc->dev, "Cannot enable function %d!\n", fn);
	return (MMC_ERR_FAILED);
}

/*
 * Enables or disables interrupt generation
 * for the given function on SDIO card.
 */
static int
mmc_io_set_intr(struct mmc_softc *sc, uint32_t fn, char enable)
{
	int err;
	uint8_t intrs;

	if (fn > sc->sdio_nfunc) {
		device_printf(sc->dev, "Invalid function to manage interrupt: %d\n", fn);
		return (MMC_ERR_INVALID);
	}

	intrs = mmc_io_read_1(sc, 0, SD_IO_CCCR_INT_ENABLE);

	if (enable) {
		intrs |= (1 << fn);
		intrs |= 1; /* Master interrupt enable */
	} else {
		intrs &= ~ (1 << fn);
		if (intrs == 0x01)
			intrs = 0x0; /* No interrupts enabled -> disable on card */
	}

	device_printf(sc->dev, "mmc_io_set_intr(): writing %d to INT_ENABLE\n", intrs);
	err = mmc_io_rw_direct(sc, 1, 0, SD_IO_CCCR_INT_ENABLE, &intrs);
	if (err != MMC_ERR_NONE) {
		device_printf(sc->dev, "Error writing SDIO intr enable %d\n", err);
		return (err);
	}
	return (0);
}

/* CMD52 */
static int
mmc_io_rw_direct(struct mmc_softc *sc, int wr, uint32_t fn, uint32_t adr,
		      uint8_t *data)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = SD_IO_RW_DIRECT;
	cmd.arg = SD_IO_RW_FUNC(fn) | SD_IO_RW_ADR(adr);
	if (wr)
		cmd.arg |= SD_IO_RW_WR | SD_IO_RW_RAW | SD_IO_RW_DAT(*data);
	cmd.flags = MMC_RSP_R5 | MMC_CMD_AC;
	cmd.data = NULL;

	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
	if (err)
		return (err);
	if (cmd.error)
		return (cmd.error);

	if (cmd.resp[0] & R5_COM_CRC_ERROR)
		return (MMC_ERR_BADCRC);
	if (cmd.resp[0] & (R5_ILLEGAL_COMMAND | R5_FUNCTION_NUMBER))
		return (MMC_ERR_INVALID);
	if (cmd.resp[0] & R5_OUT_OF_RANGE)
		return (MMC_ERR_FAILED);

	/* Just for information... */
	if (R5_IO_CURRENT_STATE(cmd.resp[0]) != 1)
		printf("!!! SDIO state %d\n", R5_IO_CURRENT_STATE(cmd.resp[0]));

	if (cmd.resp[0] & R5_ERROR)
		printf("An error was detected!\n");

	if (cmd.resp[0] & R5_COM_CRC_ERROR)
		printf("A CRC error was detected!\n");

	*data = (uint8_t) (cmd.resp[0] & 0xff);
	return (MMC_ERR_NONE);
}

/*
 * CMD53
 * incr = 1 with non-zero block count does not make any sense!
*/
static int
mmc_io_rw_extended(struct mmc_softc *sc, int wr, uint32_t fn, uint32_t adr,
		 uint8_t *datap, size_t datalen, uint8_t incr, uint8_t blks)
{
	int err;
	struct mmc_command cmd;
	struct mmc_data data;

	memset(&cmd, 0, sizeof(cmd));
	memset(&data, 0, sizeof(data));

	cmd.opcode = SD_IO_RW_EXTENDED;
	cmd.flags = MMC_RSP_R5 | MMC_CMD_AC;
	cmd.arg = SD_IO_RW_FUNC(fn);
	cmd.arg |= SD_IO_RW_ADR(adr);
	if (blks)
		cmd.arg |= SD_IOE_RW_BLK | SD_IOE_RW_LEN(blks);
	else
		cmd.arg |= SD_IOE_RW_LEN(datalen);
	if (wr)
		cmd.arg |= SD_IO_RW_WR;
	else
		memset(datap, 0, datalen);

	if (incr)
		cmd.arg |= SD_IO_RW_INCR;
	cmd.data = &data;

	data.data = datap;
	data.len = datalen;
	data.flags = wr ? MMC_DATA_WRITE : MMC_DATA_READ;
	if (blks > 1) {
		data.flags |= MMC_DATA_MULTI;
		data.blocksz = data.len / blks;
	}

	err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);

	if (err)
		return (err);
	if (cmd.error)
		return (cmd.error);

	if (cmd.resp[0] & R5_COM_CRC_ERROR)
		return (MMC_ERR_BADCRC);
	if (cmd.resp[0] & (R5_ILLEGAL_COMMAND | R5_FUNCTION_NUMBER))
		return (MMC_ERR_INVALID);
	if (cmd.resp[0] & R5_OUT_OF_RANGE)
		return (MMC_ERR_FAILED);

	return (MMC_ERR_NONE);
}

static uint8_t
mmc_io_read_1(struct mmc_softc *sc, uint32_t fn, uint32_t adr)
{
	int err;
	uint8_t val = 0;

	err = mmc_io_rw_direct(sc, 0, fn, adr, &val);
	if (err) {
		device_printf(sc->dev, "Err reading FN %d addr 0x%08X: %d",
			      fn, adr, err);
		return (0xff);
	}
	return val;
}

/*
 * Parse Card Information Structure of the SDIO card.
 * Both Function 0 CIS and Function 1-7 CIS are supported.
 */
static int
mmc_io_parse_cis(struct mmc_softc *sc, uint8_t func, uint32_t cisptr, struct sdio_function *sdio_func)
{
	uint32_t tmp;

	uint8_t tuple_id, tuple_len, func_id;
	uint32_t addr, maninfo_p;

	char *cis1_info[4];
	int start, i, ch, count;
	char cis1_info_buf[256];

	sdio_func->number  = func;

	cis1_info[0] = NULL;
	cis1_info[1] = NULL;
	cis1_info[2] = NULL;
	cis1_info[3] = NULL;
	memset(cis1_info_buf, 0, 256);

	tmp = 0;
	addr = cisptr;

	/*
	 * XXX Some parts of this code are taken
	 * from sys/dev/pccard/pccard_cis.c.
	 * Need to think about making it more abstract.
	 */
	do {
		tuple_id = mmc_io_read_1(sc, 0, addr++);
		if (tuple_id == SD_IO_CISTPL_END)
			break;
		tuple_len = mmc_io_read_1(sc, 0, addr++);
		if (tuple_len == 0 && tuple_id != 0x00) {
			device_printf(sc->dev,
			    "Parse error: 0-length tuple %02X\n", tuple_id);
			break;
		}

		switch (tuple_id) {
		case SD_IO_CISTPL_VERS_1:
			maninfo_p = addr;

			sdio_func->cis1_major = mmc_io_read_1(sc, 0, maninfo_p);
			sdio_func->cis1_minor = mmc_io_read_1(sc, 0, maninfo_p + 1);

			for (count = 0, start = 0, i = 0;
			    (count < 4) && ((i + 4) < 256); i++) {
				ch = mmc_io_read_1(sc, 0, maninfo_p + 2 + i);
				if (ch == 0xff)
					break;
				cis1_info_buf[i] = ch;
				if (ch == 0) {
					cis1_info[count] =
					    cis1_info_buf + start;
					start = i + 1;
					count++;
				}
			}
			for (i=0; i<4; i++)
				if (cis1_info[i])
					device_printf(sc->dev,
						      "%s ",
						      cis1_info[i]);
			device_printf(sc->dev, "\n");
			break;

		case SD_IO_CISTPL_MANFID:
			if (tuple_len < 4) {
				device_printf(sc->dev, "MANFID is too short\n");
				break;
			}
			sdio_func->manufacturer = mmc_io_read_1(sc, 0, addr);
			sdio_func->manufacturer |= mmc_io_read_1(sc, 0, addr + 1) << 8;

			sdio_func->product = mmc_io_read_1(sc, 0, addr + 2);
			sdio_func->product |= mmc_io_read_1(sc, 0, addr + 3) << 8;
			break;

		case SD_IO_CISTPL_FUNCID:
			/* Function ID for SDIO devices is always 0x0C */
			if (tuple_len < 1) {
				device_printf(sc->dev, "FUNCID is too short\n");
				break;
			}
			func_id = mmc_io_read_1(sc, 0, addr);
			if (func_id != 0x0C)
				device_printf(sc->dev, "func_id non-std: %d\n", func_id);
			break;

		case SD_IO_CISTPL_FUNCE:
			if (tuple_len < 4) {
				device_printf(sc->dev, "FUNCE is too short\n");
				break;
			}
			uint8_t ext_data_type = mmc_io_read_1(sc, 0, addr);

			if (func == 0) {
				if (ext_data_type != 0x0)
					device_printf(sc->dev,
					    "funce for func 0 non-std: %d\n",
					    ext_data_type);
				sdio_func->max_blksize =
				    mmc_io_read_1(sc, 0, addr + 1);
				sdio_func->max_blksize |=
				    mmc_io_read_1(sc, 0, addr + 2) << 8;
				sdio_func->max_tran_speed =
				    mmc_io_read_1(sc, 0, addr + 3);
				uint8_t max_tran_rate =
				    sdio_func->max_tran_speed & 0x7;
				uint8_t timecode =
				    (sdio_func->max_tran_speed >> 3) & 0xF;

				device_printf(sc->dev,
				    "*** Max tran speed: %02X (unit %d, time value code %d)\n",
				    sdio_func->max_tran_speed, max_tran_rate, timecode);
			} else {
				if (ext_data_type != 0x1)
					device_printf(sc->dev,
					    "funce for func 0 non-std: %d\n",
					    ext_data_type);
				sdio_func->max_blksize =
				    mmc_io_read_1(sc, 0, addr + 0x0c);
				sdio_func->max_blksize |=
				    mmc_io_read_1(sc, 0, addr + 0x0d) << 8;

			}

			break;

		default:
			device_printf(sc->dev,
			    "*** Skipping tuple ID %02X len %02X\n",
			    tuple_id, tuple_len);
			break;
		}

		addr += tuple_len;
		tmp++;
	} while (tuple_id != SD_IO_CISTPL_END && tmp < 10);

	return 0;
}

/*
 * Parse Card Common Control Register of the SDIO card
 */
static int
mmc_io_parse_cccr(struct mmc_softc *sc)
{
	uint32_t cisptr = 0;

	cisptr =  mmc_io_read_1(sc, 0, SD_IO_CCCR_CISPTR);
	cisptr |= mmc_io_read_1(sc, 0, SD_IO_CCCR_CISPTR + 1) << 8;
	cisptr |= mmc_io_read_1(sc, 0, SD_IO_CCCR_CISPTR + 2) << 16;

	if (cisptr < SD_IO_CIS_START ||
	    cisptr > SD_IO_CIS_START +  SD_IO_CIS_SIZE) {
		device_printf(sc->dev, "Bad CIS pointer in CCCR: %08X\n", cisptr);
		return (-1);
	}

	return mmc_io_parse_cis(sc, 0, cisptr, &sc->sdio_func0);
}

/*
 * Parse Function Basic Register of the given function
 */
static int
mmc_io_parse_fbr(struct mmc_softc *sc, uint8_t func)
{
	uint32_t fbr_addr, cisptr;

	fbr_addr = SD_IO_FBR_START * func + 0x9;
	cisptr =  mmc_io_read_1(sc, 0, fbr_addr);
	cisptr |= mmc_io_read_1(sc, 0, fbr_addr + 1) << 8;
	cisptr |= mmc_io_read_1(sc, 0, fbr_addr + 2) << 16;

	if (cisptr < SD_IO_CIS_START ||
	    cisptr > SD_IO_CIS_START +  SD_IO_CIS_SIZE) {
		device_printf(sc->dev, "Bad CIS pointer in FBR: %08X\n", cisptr);
		return (-1);
	}

	struct sdio_function *f = malloc(sizeof(struct sdio_function), M_DEVBUF, M_WAITOK);
	STAILQ_INSERT_TAIL(&sc->sdiof_head, f, sdiof_list);

	return mmc_io_parse_cis(sc, func, cisptr, f);
}

static void
mmc_io_get_info(struct mmc_softc *sc)
{
	sc->sdio_bus_width = bus_width_1;
	sc->sdio_support_hs = 0;

	uint8_t cardcap = mmc_io_read_1(sc, 0, SD_IO_CCCR_CARDCAP);
	uint8_t hs_info = mmc_io_read_1(sc, 0, SD_IO_CCCR_CISPTR + 0x13);

	/*
	 * If the card is a full-speed card, it supports 4-bit bus width.
	 * If it is low-speed, we check 4BLS to determine if it
	 * supports 4-bit width
	 */
	if (((cardcap & (1 << 6)) && (cardcap & (1 << 7))) ||
	    ((cardcap & (1 << 6)) == 0))
		sc->sdio_bus_width = bus_width_4;

	sc->sdio_support_hs = hs_info & (1 << 0);
}

/* Set bus width for SDIO card */
static int
mmc_io_set_bus_width(struct mmc_softc *sc, int width)
{
	uint8_t busctrl = mmc_io_read_1(sc, 0, SD_IO_CCCR_BUS_WIDTH);

	busctrl |= width == bus_width_4 ? CCCR_BUS_WIDTH_4 : 0;

	if (mmc_debug)
		device_printf(sc->dev, "Setting SDIO bus width to %d bits\n",
		    width == bus_width_4 ? 4 : 1);

	return mmc_io_rw_direct(sc, 1, 0, SD_IO_CCCR_BUS_WIDTH, &busctrl);
}

static int
mmc_io_set_block_size(struct mmc_softc *sc, struct sdio_function *sdiof,
    uint16_t bs)
{
	uint32_t addr;
	uint8_t val;
	int err;

	addr = SD_IO_FBR_START * sdiof->number + 0x10;
	val = bs & 0xFF;

	err = mmc_io_rw_direct(sc, 1, 0,
	    addr++, &val);
	if (err) {
		device_printf(sc->dev, "mmc_io_set_block_size: Err %d\n", err);
		return (err);
	}

	val = (bs >> 8) & 0xFF;
	err = mmc_io_rw_direct(sc, 1, 0,
	    addr++, &val);
	if (err) {
		device_printf(sc->dev, "mmc_io_set_block_size: Err %d\n", err);
		return (err);
	}
	sdiof->blksize = bs;

	return (err);
}

static void
mmc_discover_cards(struct mmc_softc *sc)
{
	struct mmc_ivars *ivar = NULL;
	device_t *devlist;
	int err, i, devcount, newcard;
	uint32_t raw_cid[4], resp, sec_count, status;
	device_t child;
	uint16_t rca = 2;
	u_char switch_res[64];
	uint8_t nfunc, mem_present;

	if (bootverbose || mmc_debug)
		device_printf(sc->dev, "Probing cards\n");
	while (1) {
		/*
		 * Probe SDIO first, because SDIO cards don't have
		 * a CID register and won't respond to the CMD2
		 */
		mmc_idle_cards(sc);
		err = mmc_probe_sdio(sc, 0, NULL, &nfunc, &mem_present);
		sc->sdio_nfunc = nfunc;
		if (err != MMC_ERR_NONE && err != MMC_ERR_TIMEOUT) {
			device_printf(sc->dev, "Error probing SDIO %d\n", err);
			break;
		}

		/* The card answered OK -> SDIO */
		if (err == MMC_ERR_NONE) {
			device_printf(sc->dev, "Detected SDIO card\n");
			mmc_send_relative_addr(sc, &resp); /* CMD3 */
			uint16_t rca = resp >> 16;
			err = mmc_select_card(sc, rca); /* CMD7 */
			sc->__sdio_rca = rca; /* XXX Temp; for testing only */
			if (err != MMC_ERR_NONE) {
				device_printf(sc->dev, "Error selecting SDIO %d\n", err);
				break;
			}

			device_printf(sc->dev, "Get card info\n");
			mmc_io_parse_cccr(sc);
			mmc_io_get_info(sc);
			for(i=1; i <= nfunc; i++) {
				device_printf(sc->dev,
				    "Get info for function %d\n", i);
				mmc_io_parse_fbr(sc, i);
				mmc_io_func_enable(sc, i);
			}

			device_printf(sc->dev, "=== Functions ===\n");
			struct sdio_function *f;

			STAILQ_FOREACH(f, &sc->sdiof_head, sdiof_list)
				device_printf(sc->dev,
				    "FN %d, vendor %04X, product %04X; blksize %02X\n",
				    f->number, f->manufacturer, f->product, f->max_blksize);

			/*
			 * Only set 4-bit width if both the host and the card
			 * support it.
			 * The card starts in 1-bit mode by default.
			 */
			if (mmcbr_get_caps(sc->dev) & MMC_CAP_4_BIT_DATA &&
			    sc->sdio_bus_width == bus_width_4) {
				mmc_io_set_bus_width(sc, sc->sdio_bus_width);
				mmcbr_set_bus_width(sc->dev, sc->sdio_bus_width);
			}

			u_char sdio_timing;
			uint32_t sdio_tran_speed;
			/* Set high speed mode if host and card support it */
			if (mmcbr_get_caps(sc->dev) & MMC_CAP_HSPEED &&
			    sc->sdio_support_hs) {
				device_printf(sc->dev, "Activating high-speed mode");
				uint8_t hs_info = 1;
				err = mmc_io_rw_direct(sc, 1, 0,
				    SD_IO_CCCR_CISPTR + 0x13, &hs_info);
				if (err != MMC_ERR_NONE) {
					device_printf(sc->dev, "Error setting HS mode%d\n", err);
					return;
				}
				sdio_timing = bus_timing_hs;
				sdio_tran_speed = 50 * 1000 * 1000;
			} else {
				sdio_tran_speed = 25 * 1000 * 1000;
				sdio_timing = bus_timing_normal;
			}

			/* Attach children */
			STAILQ_FOREACH(f, &sc->sdiof_head, sdiof_list) {
				ivar = malloc(sizeof(struct mmc_ivars), M_DEVBUF,
				    M_WAITOK | M_ZERO);
				ivar->sec_count = 0;
				ivar->sdiof = f;
				ivar->rca = rca;
				ivar->timing = sdio_timing;
				ivar->tran_speed =
					ivar->hs_tran_speed = sdio_tran_speed;
				child = device_add_child(sc->dev, NULL, -1);
				device_set_ivars(child, ivar);
			}
			if (!mem_present)
				return;
		}

		err = mmc_all_send_cid(sc, raw_cid); /* Command 2 */
		if (err == MMC_ERR_TIMEOUT)
			break;
		if (err != MMC_ERR_NONE) {
			device_printf(sc->dev, "Error reading CID %d\n", err);
			break;
		}
		newcard = 1;
		if ((err = device_get_children(sc->dev, &devlist, &devcount)) != 0)
			return;
		for (i = 0; i < devcount; i++) {
			ivar = device_get_ivars(devlist[i]);
			if (memcmp(ivar->raw_cid, raw_cid, sizeof(raw_cid)) == 0) {
				newcard = 0;
				break;
			}
		}
		free(devlist, M_TEMP);
		if (bootverbose || mmc_debug) {
			device_printf(sc->dev, "%sard detected (CID %08x%08x%08x%08x)\n",
			    newcard ? "New c" : "C",
			    raw_cid[0], raw_cid[1], raw_cid[2], raw_cid[3]);
		}
		if (newcard) {
			ivar = malloc(sizeof(struct mmc_ivars), M_DEVBUF,
			    M_WAITOK | M_ZERO);
			memcpy(ivar->raw_cid, raw_cid, sizeof(raw_cid));
		}
		if (mmcbr_get_ro(sc->dev))
			ivar->read_only = 1;
		ivar->bus_width = bus_width_1;
		ivar->timing = bus_timing_normal;
		ivar->mode = mmcbr_get_mode(sc->dev);
		if (ivar->mode == mode_sd) {
			mmc_decode_cid_sd(ivar->raw_cid, &ivar->cid);
			mmc_send_relative_addr(sc, &resp);
			ivar->rca = resp >> 16;
			/* Get card CSD. */
			mmc_send_csd(sc, ivar->rca, ivar->raw_csd);
			if (bootverbose || mmc_debug)
				device_printf(sc->dev,
				    "%sard detected (CSD %08x%08x%08x%08x)\n",
				    newcard ? "New c" : "C", ivar->raw_csd[0],
				    ivar->raw_csd[1], ivar->raw_csd[2],
				    ivar->raw_csd[3]);
			mmc_decode_csd_sd(ivar->raw_csd, &ivar->csd);
			ivar->sec_count = ivar->csd.capacity / MMC_SECTOR_SIZE;
			if (ivar->csd.csd_structure > 0)
				ivar->high_cap = 1;
			ivar->tran_speed = ivar->csd.tran_speed;
			ivar->erase_sector = ivar->csd.erase_sector * 
			    ivar->csd.write_bl_len / MMC_SECTOR_SIZE;
			
			err = mmc_send_status(sc, ivar->rca, &status);
			if (err != MMC_ERR_NONE) {
				device_printf(sc->dev,
				    "Error reading card status %d\n", err);
				break;
			}
			if ((status & R1_CARD_IS_LOCKED) != 0) {
				device_printf(sc->dev,
				    "Card is password protected, skipping.\n");
				break;
			}

			/* Get card SCR. Card must be selected to fetch it. */
			mmc_select_card(sc, ivar->rca);
			mmc_app_send_scr(sc, ivar->rca, ivar->raw_scr);
			mmc_app_decode_scr(ivar->raw_scr, &ivar->scr);
			/* Get card switch capabilities (command class 10). */
			if ((ivar->scr.sda_vsn >= 1) &&
			    (ivar->csd.ccc & (1<<10))) {
				mmc_sd_switch(sc, SD_SWITCH_MODE_CHECK,
				    SD_SWITCH_GROUP1, SD_SWITCH_NOCHANGE,
				    switch_res);
				if (switch_res[13] & 2) {
					ivar->timing = bus_timing_hs;
					ivar->hs_tran_speed = SD_MAX_HS;
				}
			}
			mmc_app_sd_status(sc, ivar->rca, ivar->raw_sd_status);
			mmc_app_decode_sd_status(ivar->raw_sd_status,
			    &ivar->sd_status);
			if (ivar->sd_status.au_size != 0) {
				ivar->erase_sector =
				    16 << ivar->sd_status.au_size;
			}
			mmc_select_card(sc, 0);
			/* Find max supported bus width. */
			if ((mmcbr_get_caps(sc->dev) & MMC_CAP_4_BIT_DATA) &&
			    (ivar->scr.bus_widths & SD_SCR_BUS_WIDTH_4))
				ivar->bus_width = bus_width_4;

			/*
			 * Some cards that report maximum I/O block sizes
			 * greater than 512 require the block length to be
			 * set to 512, even though that is supposed to be
			 * the default.  Example:
			 *
			 * Transcend 2GB SDSC card, CID:
			 * mid=0x1b oid=0x534d pnm="00000" prv=1.0 mdt=00.2000
			 */
			if (ivar->csd.read_bl_len != MMC_SECTOR_SIZE ||
			    ivar->csd.write_bl_len != MMC_SECTOR_SIZE)
				mmc_set_blocklen(sc, MMC_SECTOR_SIZE);

			mmc_format_card_id_string(ivar);

			if (bootverbose || mmc_debug)
				mmc_log_card(sc->dev, ivar, newcard);
			if (newcard) {
				/* Add device. */
				child = device_add_child(sc->dev, NULL, -1);
				device_set_ivars(child, ivar);
			}
			return;
		}
		mmc_decode_cid_mmc(ivar->raw_cid, &ivar->cid);
		ivar->rca = rca++;
		mmc_set_relative_addr(sc, ivar->rca);
		/* Get card CSD. */
		mmc_send_csd(sc, ivar->rca, ivar->raw_csd);
		if (bootverbose || mmc_debug)
			device_printf(sc->dev,
			    "%sard detected (CSD %08x%08x%08x%08x)\n",
			    newcard ? "New c" : "C", ivar->raw_csd[0],
			    ivar->raw_csd[1], ivar->raw_csd[2],
			    ivar->raw_csd[3]);

		mmc_decode_csd_mmc(ivar->raw_csd, &ivar->csd);
		ivar->sec_count = ivar->csd.capacity / MMC_SECTOR_SIZE;
		ivar->tran_speed = ivar->csd.tran_speed;
		ivar->erase_sector = ivar->csd.erase_sector * 
		    ivar->csd.write_bl_len / MMC_SECTOR_SIZE;

		err = mmc_send_status(sc, ivar->rca, &status);
		if (err != MMC_ERR_NONE) {
			device_printf(sc->dev,
			    "Error reading card status %d\n", err);
			break;
		}
		if ((status & R1_CARD_IS_LOCKED) != 0) {
			device_printf(sc->dev,
			    "Card is password protected, skipping.\n");
			break;
		}

		/* Only MMC >= 4.x cards support EXT_CSD. */
		if (ivar->csd.spec_vers >= 4) {
			/* Card must be selected to fetch EXT_CSD. */
			mmc_select_card(sc, ivar->rca);
			mmc_send_ext_csd(sc, ivar->raw_ext_csd);
			/* Handle extended capacity from EXT_CSD */
			sec_count = ivar->raw_ext_csd[EXT_CSD_SEC_CNT] +
			    (ivar->raw_ext_csd[EXT_CSD_SEC_CNT + 1] << 8) +
			    (ivar->raw_ext_csd[EXT_CSD_SEC_CNT + 2] << 16) +
			    (ivar->raw_ext_csd[EXT_CSD_SEC_CNT + 3] << 24);
			if (sec_count != 0) {
				ivar->sec_count = sec_count;
				ivar->high_cap = 1;
			}
			/* Get card speed in high speed mode. */
			ivar->timing = bus_timing_hs;
			if (ivar->raw_ext_csd[EXT_CSD_CARD_TYPE]
			    & EXT_CSD_CARD_TYPE_52)
				ivar->hs_tran_speed = MMC_TYPE_52_MAX_HS;
			else if (ivar->raw_ext_csd[EXT_CSD_CARD_TYPE]
			    & EXT_CSD_CARD_TYPE_26)
				ivar->hs_tran_speed = MMC_TYPE_26_MAX_HS;
			else
				ivar->hs_tran_speed = ivar->tran_speed;
			/* Find max supported bus width. */
			ivar->bus_width = mmc_test_bus_width(sc);
			mmc_select_card(sc, 0);
			/* Handle HC erase sector size. */
			if (ivar->raw_ext_csd[EXT_CSD_ERASE_GRP_SIZE] != 0) {
				ivar->erase_sector = 1024 *
				    ivar->raw_ext_csd[EXT_CSD_ERASE_GRP_SIZE];
				mmc_switch(sc, EXT_CSD_CMD_SET_NORMAL,
				    EXT_CSD_ERASE_GRP_DEF, 1);
			}
		} else {
			ivar->bus_width = bus_width_1;
			ivar->timing = bus_timing_normal;
		}

		/*
		 * Some cards that report maximum I/O block sizes greater
		 * than 512 require the block length to be set to 512, even
		 * though that is supposed to be the default.  Example:
		 *
		 * Transcend 2GB SDSC card, CID:
		 * mid=0x1b oid=0x534d pnm="00000" prv=1.0 mdt=00.2000
		 */
		if (ivar->csd.read_bl_len != MMC_SECTOR_SIZE ||
		    ivar->csd.write_bl_len != MMC_SECTOR_SIZE)
			mmc_set_blocklen(sc, MMC_SECTOR_SIZE);

		mmc_format_card_id_string(ivar);

		if (bootverbose || mmc_debug)
			mmc_log_card(sc->dev, ivar, newcard);
		if (newcard) {
			/* Add device. */
			child = device_add_child(sc->dev, NULL, -1);
			device_set_ivars(child, ivar);
		}
	}
}

static void
mmc_rescan_cards(struct mmc_softc *sc)
{
	struct mmc_ivars *ivar = NULL;
	device_t *devlist;
	int err, i, devcount;

	if ((err = device_get_children(sc->dev, &devlist, &devcount)) != 0)
		return;
	for (i = 0; i < devcount; i++) {
		ivar = device_get_ivars(devlist[i]);
		if (mmc_select_card(sc, ivar->rca)) {
			if (bootverbose || mmc_debug)
				device_printf(sc->dev, "Card at relative address %d lost.\n",
				    ivar->rca);
			device_delete_child(sc->dev, devlist[i]);
			free(ivar, M_DEVBUF);
		}
	}
	free(devlist, M_TEMP);
	mmc_select_card(sc, 0);
}

static int
mmc_delete_cards(struct mmc_softc *sc)
{
	struct mmc_ivars *ivar;
	device_t *devlist;
	int err, i, devcount;

	if ((err = device_get_children(sc->dev, &devlist, &devcount)) != 0)
		return (err);
	for (i = 0; i < devcount; i++) {
		ivar = device_get_ivars(devlist[i]);
		if (bootverbose || mmc_debug)
			device_printf(sc->dev, "Card at relative address %d deleted.\n",
			    ivar->rca);
		device_delete_child(sc->dev, devlist[i]);
		free(ivar, M_DEVBUF);
	}
	free(devlist, M_TEMP);
	return (0);
}

/* CMD 5 */
static int
mmc_probe_sdio(struct mmc_softc *sc, uint32_t ocr, uint32_t *rocr, uint8_t *nfunc, uint8_t *mem_present) {
	struct mmc_command cmd;
	int err = MMC_ERR_NONE, i;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = IO_SEND_OP_COND;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R4;
	cmd.data = NULL;

	for (i = 0; i < 1000; i++) {
		err = mmc_wait_for_cmd(sc, &cmd, CMD_RETRIES);
		if (err != MMC_ERR_NONE)
			break;
		if ((cmd.resp[0] & MMC_OCR_CARD_BUSY) ||
		    (ocr & MMC_OCR_VOLTAGE) == 0)
			break;
		err = MMC_ERR_TIMEOUT;
		mmc_ms_delay(10);
	}

	if (err == MMC_ERR_NONE) {
		if (rocr)
			*rocr = cmd.resp[0];
		if (nfunc)
			*nfunc = SD_IO_OCR_NUM_FUNCTIONS(cmd.resp[0]);
		if (mem_present)
			*mem_present = cmd.resp[0] >> 27 & 0x1;
	}

	return (err);
}

static void
mmc_go_discovery(struct mmc_softc *sc)
{
	uint8_t nfunc, mem_present;
	uint32_t ocr;
	device_t dev;
	int err;

	dev = sc->dev;
	if (mmcbr_get_power_mode(dev) != power_on) {
		/*
		 * First, try SD modes
		 */
		mmcbr_set_mode(dev, mode_sd);
		mmc_power_up(sc);
		mmcbr_set_bus_mode(dev, pushpull);
		if (bootverbose || mmc_debug)
			device_printf(sc->dev, "Probing bus\n");
		mmc_idle_cards(sc);
		err = mmc_send_if_cond(sc, 1);  /* SD_SEND_IF_COND = 8 */
		if ((bootverbose || mmc_debug) && err == 0)
			device_printf(sc->dev, "SD 2.0 interface conditions: OK\n");
		if (mmc_probe_sdio(sc, 0, &ocr, &nfunc, &mem_present) == MMC_ERR_NONE) {
			device_printf(dev, "SDIO probe OK (OCR: 0x%08x, %d functions, memory: %d)\n", ocr, nfunc, mem_present);
			if (nfunc > 0 && mem_present) {
				device_printf(sc->dev, "SDIO combo cards are not supported yet");
				return;
			}
		} else
		if (mmc_send_app_op_cond(sc, 0, &ocr) != MMC_ERR_NONE) { /* retry 55 -> then 41 */
			if (bootverbose || mmc_debug)
				device_printf(sc->dev, "SD probe: failed\n");
			/*
			 * Failed, try MMC
			 */
			mmcbr_set_mode(dev, mode_mmc);
			if (mmc_send_op_cond(sc, 0, &ocr) != MMC_ERR_NONE) { /* command 1 */
				if (bootverbose || mmc_debug)
					device_printf(sc->dev, "MMC probe: failed\n");
				ocr = 0; /* Failed both, powerdown. */
			} else if (bootverbose || mmc_debug)
				device_printf(sc->dev,
				    "MMC probe: OK (OCR: 0x%08x)\n", ocr);
		} else if (bootverbose || mmc_debug)
			device_printf(sc->dev, "SD probe: OK (OCR: 0x%08x)\n", ocr);

		mmcbr_set_ocr(dev, mmc_select_vdd(sc, ocr));
		if (mmcbr_get_ocr(dev) != 0)
			mmc_idle_cards(sc);
	} else {
		mmcbr_set_bus_mode(dev, opendrain);
		mmcbr_set_clock(dev, CARD_ID_FREQUENCY);
		mmcbr_update_ios(dev);
		/* XXX recompute vdd based on new cards? */
	}
	/*
	 * Make sure that we have a mutually agreeable voltage to at least
	 * one card on the bus.
	 */
	if (bootverbose || mmc_debug)
		device_printf(sc->dev, "Current OCR: 0x%08x\n", mmcbr_get_ocr(dev));
	if (mmcbr_get_ocr(dev) == 0) {
		mmc_delete_cards(sc);
		mmc_power_down(sc);
		return;
	}
	/*
	 * Reselect the cards after we've idled them above.
	 */
	if (mmcbr_get_mode(dev) == mode_sd) {
		if (mem_present) {
			err = mmc_send_if_cond(sc, 1); /* CMD 8 */
			mmc_send_app_op_cond(sc, /* 41 -> 55 */
			    (err ? 0 : MMC_OCR_CCS) | mmcbr_get_ocr(dev), NULL);
		}
	} else
		mmc_send_op_cond(sc, mmcbr_get_ocr(dev), NULL);
	mmc_discover_cards(sc);
	mmc_rescan_cards(sc);

	mmcbr_set_bus_mode(dev, pushpull);
	mmcbr_update_ios(dev);
	mmc_calculate_clock(sc);

	mmc_select_card(sc, sc->__sdio_rca);

	bus_generic_attach(dev);
/*	mmc_update_children_sysctl(dev);*/
}

static int
mmc_calculate_clock(struct mmc_softc *sc)
{
	int max_dtr, max_hs_dtr, max_timing;
	int nkid, i, f_max;
	device_t *kids;
	struct mmc_ivars *ivar;

	f_max = mmcbr_get_f_max(sc->dev);
	max_dtr = max_hs_dtr = f_max;
	if ((mmcbr_get_caps(sc->dev) & MMC_CAP_HSPEED))
		max_timing = bus_timing_hs;
	else
		max_timing = bus_timing_normal;

	if (device_get_children(sc->dev, &kids, &nkid) != 0)
		panic("can't get children");
	for (i = 0; i < nkid; i++) {
		ivar = device_get_ivars(kids[i]);
		if (ivar->timing < max_timing)
			max_timing = ivar->timing;
		if (ivar->tran_speed < max_dtr)
			max_dtr = ivar->tran_speed;
		if (ivar->hs_tran_speed < max_hs_dtr)
			max_hs_dtr = ivar->hs_tran_speed;
	}
	for (i = 0; i < nkid; i++) {
		ivar = device_get_ivars(kids[i]);
		if (ivar->timing == bus_timing_normal)
			continue;
		mmc_select_card(sc, ivar->rca);
		mmc_set_timing(sc, max_timing);
	}
	mmc_select_card(sc, 0);
	free(kids, M_TEMP);
	if (max_timing == bus_timing_hs)
		max_dtr = max_hs_dtr;
	if (bootverbose || mmc_debug) {
		device_printf(sc->dev,
		    "setting transfer rate to %d.%03dMHz%s\n",
		    max_dtr / 1000000, (max_dtr / 1000) % 1000,
		    max_timing == bus_timing_hs ? " (high speed timing)" : "");
	}
	mmcbr_set_timing(sc->dev, max_timing);
	mmcbr_set_clock(sc->dev, max_dtr);
	mmcbr_update_ios(sc->dev);
	return max_dtr;
}

static void
mmc_scan(struct mmc_softc *sc)
{
	device_t dev = sc->dev;

	mmc_acquire_bus(dev, dev);
	mmc_go_discovery(sc);
	mmc_release_bus(dev, dev);
}

static int
mmc_read_ivar(device_t bus, device_t child, int which, uintptr_t *result)
{
	struct mmc_ivars *ivar = device_get_ivars(child);

	switch (which) {
	default:
		return (EINVAL);
	case MMC_IVAR_DSR_IMP:
		*result = ivar->csd.dsr_imp;
		break;
	case MMC_IVAR_MEDIA_SIZE:
		*result = ivar->sec_count;
		break;
	case MMC_IVAR_RCA:
		*result = ivar->rca;
		break;
	case MMC_IVAR_SECTOR_SIZE:
		*result = MMC_SECTOR_SIZE;
		break;
	case MMC_IVAR_TRAN_SPEED:
		*result = mmcbr_get_clock(bus);
		break;
	case MMC_IVAR_READ_ONLY:
		*result = ivar->read_only;
		break;
	case MMC_IVAR_HIGH_CAP:
		*result = ivar->high_cap;
		break;
	case MMC_IVAR_CARD_TYPE:
		*result = ivar->mode;
		break;
	case MMC_IVAR_BUS_WIDTH:
		*result = ivar->bus_width;
		break;
	case MMC_IVAR_ERASE_SECTOR:
		*result = ivar->erase_sector;
		break;
	case MMC_IVAR_MAX_DATA:
		*result = mmcbr_get_max_data(bus);
		break;
	case MMC_IVAR_SDIO_VENDOR:
		*result = ivar->sdiof ? ivar->sdiof->manufacturer : 0;
		break;
	case MMC_IVAR_SDIO_PRODUCT:
		*result = ivar->sdiof ? ivar->sdiof->product : 0;
		break;
	case MMC_IVAR_SDIO_FUNCTION:
		*result = ivar->sdiof ? ivar->sdiof->number : 0;
		break;
	case MMC_IVAR_CARD_ID_STRING:
		*(char **)result = ivar->card_id_string;
		break;
	}
	return (0);
}

static int
mmc_write_ivar(device_t bus, device_t child, int which, uintptr_t value)
{
	/*
	 * None are writable ATM
	 */
	return (EINVAL);
}

static void
mmc_sdio_intr(void *xsc)
{
	struct mmc_softc *sc;
	sc = xsc;

	/* XXX This is a hack. See below */
	if (sc->sdio_isrs[1])
		(sc->sdio_isrs[1])(sc->sdio_isr_args[1]);

	/*
	 * TODO: we should read "Int pending" register in CCCR to figure out
	 * which function did the interrupt, then call its interrupt handler.
	 * Linux has some nice optimization -- if there was only one function
	 * that set up the interrupt, call the handler without looking into CCCR.
	 *
	 * Currently, reading SDIO register here results in kernel panic.
	 panic: _mtx_lock_sleep: recursed on non-recursive mutex sdio0 @ /usr/home/kibab/repos/fbsd-dreamplug/freebsd/sys/arm/mv/mv_sdio.c:579
	 * It would be better to create a new kernel thread to handle
	 * interrupts.
	 */
/*	uint8_t irqs_pending = mmc_io_read_1(sc, 0, SD_IO_CCCR_INT_PENDING);
	uint8_t i;

	for (i=1; i<= 7; i++) {
		if (irqs_pending & (1 << i)) {
			if (sc->sdio_isrs[i])
				(sc->sdio_isrs[i])(sc->sdio_isr_args[i]);
			else
				device_printf(sc->dev,
					      "Stray SDIO interrupt for func %d",
					      i);

		}
	}

*/
}

static void
mmc_delayed_attach(void *xsc)
{
	struct mmc_softc *sc = xsc;
	int irq_id = 0;

	/*
	 * TODO: Implement a new read-only ivar for mmcbr, to be able to ask
	 * the SDIO controller if it supports SDIO interrupts.
	 * If not, then don't even try to request IRQ resource.
	 * If yes, then failure to allocate resource and setup the interrupt
	 * handler will be fatal.
	 */
	sc->sdio_irq_res = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ, &irq_id,
	    RF_ACTIVE);
	if (sc->sdio_irq_res == NULL) {
		device_printf(sc->dev, "could not allocate IRQ for SDIO\n");
	} else if (bus_setup_intr(sc->dev, sc->sdio_irq_res,
	    INTR_MPSAFE | INTR_ENTROPY,
	    NULL, mmc_sdio_intr, sc, &sc->sdio_irq_cookiep)) {
		device_printf(sc->dev, "could not setup SDIO interrupt\n");
	}

	mmc_scan(sc);
	config_intrhook_disestablish(&sc->config_intrhook);
}

static int
mmc_child_location_str(device_t dev, device_t child, char *buf,
    size_t buflen)
{

	snprintf(buf, buflen, "rca=0x%04x", mmc_get_rca(child));
	return (0);
}

static struct resource *
mmc_alloc_resource(device_t dev, device_t child, int type, int *rid,
    u_long start, u_long end, u_long count, u_int flags)
{
	struct mmc_softc *sc;

	sc = device_get_softc(dev);

	KASSERT(type == SYS_RES_IRQ,
	    ("illegal resource request (type %u).", type));
	return (sc->sdio_irq_res);
}


static int
mmc_setup_intr(device_t dev, device_t child, struct resource *irq, int flags,
    driver_filter_t *filt, driver_intr_t *function, void *argument,
    void **cookiep)
{
	struct mmc_softc *sc;
	struct mmc_ivars *ivar = device_get_ivars(child);

	device_printf(dev, "mmc_setup_intr() called\n");
	sc = device_get_softc(dev);

	if (filt != NULL) {
		device_printf(dev, "filter interrupts are not supported.\n");
		return (EINVAL);
	}

	sc->sdio_isrs[ivar->sdiof->number] = function;
	sc->sdio_isr_args[ivar->sdiof->number] = argument;
	*cookiep = sc;

	/* Now enable interrupts on the card */
	if (mmc_io_set_intr(device_get_softc(dev), ivar->sdiof->number, 1)) {
		device_printf(dev,
			      "Cannot enable interrupts for the function %d\n",
			      ivar->sdiof->number);
	}

	return (0);
}

/* SDIO-related MMC bus methods */
static int
mmcb_io_set_block_size(device_t dev, device_t child, uint16_t bs)
{
	struct mmc_ivars *ivar = device_get_ivars(child);

	return mmc_io_set_block_size(device_get_softc(dev), ivar->sdiof, bs);
}

static int
mmcb_io_f0_read_1(device_t dev, device_t child, uint32_t adr, uint8_t *val)
{
	int err;
	err = mmc_io_rw_direct(device_get_softc(dev), 0, 0, adr, val);
	if (err)
		device_printf(dev, "mmc_io_f0_read_1: Err %d", err);
	return (err);
}

static int
mmcb_io_read_1(device_t dev, device_t child, uint32_t adr, uint8_t *val)
{
	int err;
	struct mmc_ivars *ivar = device_get_ivars(child);

	err = mmc_io_rw_direct(device_get_softc(dev), 0, ivar->sdiof->number,
	    adr, val);
	if (err)
		device_printf(dev, "mmc_io_read_1: Err %d", err);
	return (err);
}

static int
mmcb_io_write_1(device_t dev, device_t child, uint32_t adr, uint8_t val)
{
	int err;
	struct mmc_ivars *ivar = device_get_ivars(child);

	err = mmc_io_rw_direct(device_get_softc(dev), 1, ivar->sdiof->number,
	    adr, &val);
	if (err)
		device_printf(dev, "mmc_io_write_1: Err %d", err);
	return (err);
}

static int
mmcb_io_write_multi(device_t dev, device_t child, uint32_t adr,
		    uint8_t *datap, size_t datalen, uint16_t nblocks)
{
	int err;
	struct mmc_ivars *ivar = device_get_ivars(child);

	err = mmc_io_rw_extended(device_get_softc(dev), 1, ivar->sdiof->number,
	    adr, datap, datalen, 1, nblocks);
	if (err)
		device_printf(dev, "mmcb_io_write_multi: Err %d", err);
	return (err);
}

static int
mmcb_io_read_multi(device_t dev, device_t child, uint32_t adr,
		   uint8_t *datap, size_t datalen, uint16_t nblocks)
{
	int err;
	struct mmc_ivars *ivar = device_get_ivars(child);

	err = mmc_io_rw_extended(device_get_softc(dev), 0, ivar->sdiof->number,
	    adr, datap, datalen, 1, nblocks);

	if (err)
		device_printf(dev, "mmc_io_read_multi: Err %d\n", err);
	return (err);
}

static int
mmcb_io_write_fifo(device_t dev, device_t child, uint32_t adr,
		   uint8_t *datap, size_t datalen)
{
	int err;
	uint32_t b_written = 0;
	uint32_t b_to_write = datalen;
	uint32_t nblocks = 0;
	struct mmc_ivars *ivar = device_get_ivars(child);

//	device_printf(dev, "mmcb_io_write_fifo: func %d, adr=0x%04X, datap=0x%04X, len %d\n", ivar->sdiof->number, adr, (unsigned int) datap, datalen);
	err = 0;

/*
 * Linux code takes into account the max block size that is allowed
 * by the host controller. Would be nice to be able to get such information.
 */
	if (datalen >= ivar->sdiof->blksize) {
		nblocks = datalen / ivar->sdiof->blksize;
		b_to_write = ivar->sdiof->blksize * nblocks;
		err = mmc_io_rw_extended(device_get_softc(dev), 1,
		    ivar->sdiof->number, adr, datap, b_to_write, 0, nblocks);
		if (err) {
			device_printf(dev, "mmcb_io_write_fifo: Err %d\n", err);
			return (err);
		}

		b_written = b_to_write;
	}

	if (datalen - b_written > 0) {
		b_to_write = datalen - b_written;
		err = mmc_io_rw_extended(device_get_softc(dev), 1,
		    ivar->sdiof->number, adr, datap + b_written, b_to_write, 0, 0);
		if (err)
			device_printf(dev, "mmcb_io_write_fifo: Err %d\n", err);
	}
	return (err);
}

static int
mmcb_io_read_fifo(device_t dev, device_t child, uint32_t adr,
		   uint8_t *datap, size_t datalen)
{
	int err;
	uint32_t b_written = 0;
	uint32_t b_to_write = datalen;
	uint32_t nblocks = 0;
	struct mmc_ivars *ivar = device_get_ivars(child);

	device_printf(dev, "mmcb_io_read_fifo: func %d, adr=0x%04X, datap=0x%04X, len %d\n", ivar->sdiof->number, adr, (unsigned int) datap, datalen);
	err = 0;

/*
 * Linux code takes into account the max block size that is allowed
 * by the host controller. Would be nice to be able to get such information.
 */
	if (datalen >= ivar->sdiof->blksize) {
		nblocks = datalen / ivar->sdiof->blksize;
		b_to_write = ivar->sdiof->blksize * nblocks;
		err = mmc_io_rw_extended(device_get_softc(dev), 0,
		    ivar->sdiof->number, adr, datap, b_to_write, 0, nblocks);
		if (err) {
			device_printf(dev, "mmcb_io_read_fifo: Err %d\n", err);
			return (err);
		}

		b_written = b_to_write;
	}

	if (datalen - b_written > 0) {
		b_to_write = datalen - b_written;
		err = mmc_io_rw_extended(device_get_softc(dev), 0,
		    ivar->sdiof->number, adr, datap + b_written, b_to_write, 0, 0);
		if (err)
			device_printf(dev, "mmcb_io_read_fifo: Err %d\n", err);
	}
	return (err);
}

static device_method_t mmc_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, mmc_probe),
	DEVMETHOD(device_attach, mmc_attach),
	DEVMETHOD(device_detach, mmc_detach),
	DEVMETHOD(device_suspend, mmc_suspend),
	DEVMETHOD(device_resume, mmc_resume),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar, mmc_read_ivar),
	DEVMETHOD(bus_write_ivar, mmc_write_ivar),
	DEVMETHOD(bus_child_location_str, mmc_child_location_str),
	DEVMETHOD(bus_alloc_resource, mmc_alloc_resource),
	DEVMETHOD(bus_setup_intr, mmc_setup_intr),

	/* MMC Bus interface */
	DEVMETHOD(mmcbus_wait_for_request, mmc_wait_for_request),
	DEVMETHOD(mmcbus_io_set_block_size, mmcb_io_set_block_size),
	DEVMETHOD(mmcbus_io_f0_read_1, mmcb_io_f0_read_1),
	DEVMETHOD(mmcbus_io_read_1, mmcb_io_read_1),
	DEVMETHOD(mmcbus_io_read_multi, mmcb_io_read_multi),
	DEVMETHOD(mmcbus_io_write_1, mmcb_io_write_1),
	DEVMETHOD(mmcbus_io_write_multi, mmcb_io_write_multi),
	DEVMETHOD(mmcbus_io_read_fifo, mmcb_io_read_fifo),
	DEVMETHOD(mmcbus_io_write_fifo, mmcb_io_write_fifo),
	DEVMETHOD(mmcbus_acquire_bus, mmc_acquire_bus),
	DEVMETHOD(mmcbus_release_bus, mmc_release_bus),

	DEVMETHOD_END
};

static driver_t mmc_driver = {
	"mmc",
	mmc_methods,
	sizeof(struct mmc_softc),
};
static devclass_t mmc_devclass;

DRIVER_MODULE(mmc, ti_mmchs, mmc_driver, mmc_devclass, NULL, NULL);
DRIVER_MODULE(mmc, at91_mci, mmc_driver, mmc_devclass, NULL, NULL);
DRIVER_MODULE(mmc, sdhci_pci, mmc_driver, mmc_devclass, NULL, NULL);
DRIVER_MODULE(mmc, sdhci_bcm, mmc_driver, mmc_devclass, NULL, NULL);
DRIVER_MODULE(mmc, sdhci_fdt, mmc_driver, mmc_devclass, NULL, NULL);
DRIVER_MODULE(mmc, sdhci_ti, mmc_driver, mmc_devclass, NULL, NULL);
DRIVER_MODULE(mmc, sdio, mmc_driver, mmc_devclass, NULL, NULL);
