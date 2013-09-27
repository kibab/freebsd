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

#include "opt_inet.h"
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/linker.h>
#include <sys/firmware.h>
#include <sys/taskqueue.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_llc.h>

#include <net/bpf.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/if_ether.h>
#endif /* INET */

#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcvar.h>
#include <dev/mmc/mmcioreg.h>
#include <dev/mmc/mmciovar.h>

MALLOC_DECLARE(M_MVSDIOWL);
MALLOC_DEFINE(M_MVSDIOWL, "mv_sdiowl", "Buffers of Marvell SDIO WLAN Driver");

#include "mmcbus_if.h"

#define DN_LD_CARD_RDY		(1u << 0)
#define CARD_IO_READY		(1u << 3)

/* Host Control Registers */
/* Host Control Registers : Host interrupt mask */
#define HOST_INT_MASK_REG	0x02
/* Host Control Registers : Upload host interrupt mask */
#define UP_LD_HOST_INT_MASK	(0x1U)
/* Host Control Registers : Download host interrupt mask */
#define DN_LD_HOST_INT_MASK	(0x2U)
/* Enable Host interrupt mask */
#define HOST_INT_ENABLE		(UP_LD_HOST_INT_MASK | DN_LD_HOST_INT_MASK)
/* Disable Host interrupt mask */
#define HOST_INT_DISABLE	0xff

/* The following defines the layout of CTRL register (0) */
/* Host Control Registers : Host interrupt status */
#define HOST_INTSTATUS_REG	0x03
/* Host Control Registers : Upload host interrupt status */
#define UP_LD_HOST_INT_STATUS	(0x1U)
/* Host Control Registers : Download host interrupt status */
#define DN_LD_HOST_INT_STATUS	(0x2U)

/* Host Control Registers : R/W bitmaps */
#define RD_BITMAP_L		0x04
#define RD_BITMAP_U		0x05
#define WR_BITMAP_L		0x06
#define WR_BITMAP_U		0x07
#define RD_LEN_P0_L		0x08
#define RD_LEN_P0_U		0x09

#define CTRL_PORT		0
#define CTRL_PORT_MASK		0x0001

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

/* Firmware status 0 register */
#define CARD_FW_STATUS0_REG	0x60
/* Firmware status 1 register */
#define CARD_FW_STATUS1_REG	0x61

/* Misc. Config Register : Auto Re-enable interrupts */
#define AUTO_RE_ENABLE_INT	1 << 4

enum sdiowl_bss_type {
	MWIFIEX_BSS_TYPE_STA = 0,
	MWIFIEX_BSS_TYPE_UAP = 1,
	MWIFIEX_BSS_TYPE_P2P = 2,
	MWIFIEX_BSS_TYPE_ANY = 0xff,
};

/* Firmware commands */
#define HostCmd_CMD_FUNC_INIT	0x00a9
#define HostCmd_CMD_GET_HW_SPEC 0x0003

/* SD block size can not bigger than 64 due to buf size limit in firmware */
/* define SD block size for data Tx/Rx */
#define SDIO_BLOCK_SIZE		256

#define FIRMWARE_READY_SDIO	0xfedc

#define BUF_ALIGN 512

/* struct host_cmd_ds_gen */
struct sdiowl_cmd_hdr {
	uint16_t command;
	uint16_t size;
	uint16_t seq_num;
	uint16_t result;
};

struct host_cmd_ds_get_hw_spec {
	uint16_t hw_if_version;
	uint16_t version;
	uint16_t reserved;
	uint16_t num_of_mcast_adr;
	uint8_t permanent_addr[6];
	uint16_t region_code;
	uint16_t number_of_antenna;
	uint32_t fw_release_number;
	uint32_t reserved_1;
	uint32_t reserved_2;
	uint32_t reserved_3;
	uint32_t fw_cap_info;
	uint32_t dot_11n_dev_cap;
	uint8_t dev_mcs_support;
	uint16_t mp_end_port;     /* SDIO only, reserved for other interfacces */
	uint16_t mgmt_buf_count;  /* mgmt IE buffer count */
	uint32_t reserved_5;
	uint32_t reserved_6;
	uint32_t dot_11ac_dev_cap;
	uint32_t dot_11ac_mcs_support;
} __packed;

/* struct host_cmd_ds_command */
/* This is also what we receive from the firmware */
struct sdiowl_cmd {
	uint16_t __sdio_pkt_len;	/* SDIO-specific header */
	uint16_t __sdio_cmd_type;	/* SDIO-specific header */
	uint16_t command;
	uint16_t size;
	uint16_t seq_num;
	uint16_t result;
	union {
		struct host_cmd_ds_get_hw_spec hw_spec;
	} params;
} __packed;

#define HDR_SIZE sizeof(struct sdiowl_cmd_hdr)

struct sdiowl_softc {
	device_t dev;
	struct mtx sc_mtx;
	int running;
	int sdio_function;
	uint32_t ioport;
	uint8_t irqstatus;
	uint16_t rd_bitmap;
	uint16_t wr_bitmap;
	struct ifnet *sc_ifp;
	struct taskqueue *sc_tq;
	struct task sc_cmdtask;
	struct cv sc_cmdtask_finished;
	uint8_t bss_type;
	uint8_t cmd_pending;
	uint32_t lastseq;
	struct sdiowl_cmd *cmd; /* XXX: should be a FIFO-type queue */
	struct sdiowl_cmd resp;

	/* Adapter HW information */
	uint8_t bssid[6];
	uint16_t reg_code;
	uint16_t number_of_antenna;
	uint16_t mp_end_port;     /* SDIO only */

};

/* bus entry points */
static int sdiowl_attach(device_t dev);
static int sdiowl_detach(device_t dev);
static int sdiowl_probe(device_t dev);

static int sdiowl_read_1(struct sdiowl_softc *sc, uint32_t reg, uint8_t *val);
static int sdiowl_write_1(struct sdiowl_softc *sc, uint32_t reg, uint8_t val);

#define SDIOWL_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	SDIOWL_UNLOCK(_sc)	mtx_unlock(&(_sc)->sc_mtx)
#define SDIOWL_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->dev), \
	    "sdiowl", MTX_DEF)
#define SDIOWL_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);
#define SDIOWL_ASSERT_LOCKED(_sc) mtx_assert(&_sc->sc_mtx, MA_OWNED);
#define SDIOWL_ASSERT_UNLOCKED(_sc) mtx_assert(&_sc->sc_mtx, MA_NOTOWNED);

#define HostCmd_SET_SEQ_NO_BSS_INFO(seq, num, type) {   \
		(((seq) & 0x00ff) |			\
		 (((num) & 0x000f) << 8)) |		\
		(((type) & 0x000f) << 12);  }

/* This is executed in the separate task queue */
static void
sdiowl_send_cmd(void *arg, int npending)
{
	struct sdiowl_softc *sc = arg;
	struct sdiowl_cmd *cmd = sc->cmd;
	int ret;
	size_t blockcnt;

	device_printf(sc->dev, "Starting async command execution\n");

	SDIOWL_LOCK(sc);
	sc->lastseq++;
	sc->cmd_pending = 1;
	cmd->seq_num = (HostCmd_SET_SEQ_NO_BSS_INFO(sc->lastseq, 0, MWIFIEX_BSS_TYPE_STA));
	cmd->result = 0;
	cmd->__sdio_pkt_len = sizeof(struct sdiowl_cmd);
	cmd->__sdio_cmd_type = 1; /* MWIFIEX_TYPE_DATA = 0, MWIFIEX_TYPE_CMD = 1, MWIFIEX_TYPE_EVENT = 3 */
	blockcnt = (cmd->__sdio_pkt_len + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE;

	device_printf(sc->dev, "sdiowl_send_cmd(): I WILL SEND THE COMMAND, pkt len=%d, write len=%d\n", cmd->__sdio_pkt_len, blockcnt * SDIO_BLOCK_SIZE);

	hexdump(cmd, blockcnt * SDIO_BLOCK_SIZE, NULL, 0);
	ret = MMCBUS_IO_WRITE_FIFO(device_get_parent(sc->dev), sc->dev,
				   sc->ioport /* CTRL_PORT */, (uint8_t *) cmd, blockcnt * SDIO_BLOCK_SIZE);
	if (ret) {
		device_printf(sc->dev, "Error when writing to FIFO!\n");
		/* XXX Think about handling errors in the main thread... */
	}

	cv_signal(&sc->sc_cmdtask_finished);
	SDIOWL_UNLOCK(sc);
}

static int
sdiowl_check_mp_regs(struct sdiowl_softc *sc)
{
	char sdiowl_mpregs[64];
	int ret;
	memset(sdiowl_mpregs, 0, 64);

	/* Linux: REG_PORT | MWIFIEX_SDIO_BYTE_MODE_MASK, which is 0 finally */
	ret = MMCBUS_IO_READ_FIFO(device_get_parent(sc->dev), sc->dev,
				  0, (uint8_t *) sdiowl_mpregs, 64);
	if (ret) {
		device_printf(sc->dev, "Error when reading from FIFO 0!\n");
		return -1;
	}

	sc->irqstatus = sdiowl_mpregs[HOST_INTSTATUS_REG];
	sc->rd_bitmap = ( sdiowl_mpregs[RD_BITMAP_U] << 8)
		| sdiowl_mpregs[RD_BITMAP_L];

	sc->wr_bitmap = ( sdiowl_mpregs[WR_BITMAP_U] << 8)
		| sdiowl_mpregs[WR_BITMAP_L];

	device_printf(sc->dev, "IntStatus Reg in MP regs = %d\n",
		      sc->irqstatus);
	hexdump(sdiowl_mpregs, 64, NULL, 0);

	uint8_t rd_port;
	if (sc->rd_bitmap & CTRL_PORT_MASK) {
		sc->rd_bitmap &= (uint16_t) (~CTRL_PORT_MASK);
		rd_port = CTRL_PORT;
	} else panic("rd_bitmap & CTRL_PORT_MASK is FALSE -- cannot handle");

	uint8_t len_reg_l, len_reg_u;
	uint16_t rx_len;
	len_reg_l = RD_LEN_P0_L + (rd_port << 1);
	len_reg_u = RD_LEN_P0_U + (rd_port << 1);

	rx_len = ((uint16_t) sdiowl_mpregs[len_reg_u]) << 8;
	rx_len |= (uint16_t) sdiowl_mpregs[len_reg_l];

	device_printf(sc->dev, "RX len: %d from port %05x\n", rx_len, rd_port);

	uint8_t rcvbuf[256];

	KASSERT(rx_len <= 256, ("rx_len too large"));

	if (MMCBUS_IO_READ_FIFO(device_get_parent(sc->dev), sc->dev,
				sc->ioport + rd_port, rcvbuf, rx_len)) {
		device_printf(sc->dev, "FAIL to read %d bytes from ioport %d",
			      rx_len, sc->ioport + rd_port);
		return (-1);
	}

	memcpy(&sc->resp, rcvbuf, sizeof(sc->resp));
	hexdump(&sc->resp, rx_len, NULL, 0);
	if (sc->resp.seq_num != sc->lastseq) {
		device_printf(sc->dev, "Unexpected command seqnum!\n");
		return (-1);
	}
	sc->cmd_pending = 0;

	return (0);
}

static int
sdiowl_send_cmd_sync(struct sdiowl_softc *sc, struct sdiowl_cmd *cmd)
{
	device_printf(sc->dev, "Starting sync command execution\n");

	SDIOWL_LOCK(sc);
	sc->cmd_pending = 1;
	sc->cmd = cmd;
	taskqueue_enqueue(sc->sc_tq, &sc->sc_cmdtask);

	device_printf(sc->dev, "Waiting for command to be sent...\n");
	cv_wait_sig(&sc->sc_cmdtask_finished, &sc->sc_mtx);
	SDIOWL_UNLOCK(sc);

	device_printf(sc->dev, "Executing command finished!\n");

	/*
	 * XXX: I assume that interrupt status should already  be set
	 * at this point. I cannot set up an interrupt handler yet.
	*/
	pause("sdiowl", 100);
	device_printf(sc->dev, "... ok, lets see what we have here...\n");

	sdiowl_check_mp_regs(sc);
	return 0;
}

static int
sdiowl_send_init(struct sdiowl_softc *sc) {
	struct sdiowl_cmd *cmd;

	cmd = malloc(256, M_MVSDIOWL, M_WAITOK);
	memset(cmd, 0, 256);

	cmd->command = HostCmd_CMD_FUNC_INIT;
	cmd->size = HDR_SIZE;

	sdiowl_send_cmd_sync(sc, cmd);
	device_printf(sc->dev, "Device Init completed!\n");

	free(cmd, M_MVSDIOWL);

	return 0;
}

static int
sdiowl_get_hw_spec(struct sdiowl_softc *sc)
{
	struct sdiowl_cmd *cmd;

	device_printf(sc->dev, "Try to HW params\n");

	cmd = malloc(256, M_MVSDIOWL, M_WAITOK);
	memset(cmd, 0, 256);

	struct host_cmd_ds_get_hw_spec *payload = &cmd->params.hw_spec;

	cmd->command = HostCmd_CMD_GET_HW_SPEC;
	cmd->size = HDR_SIZE + sizeof(struct host_cmd_ds_get_hw_spec);

	memset(payload, 0, sizeof(struct host_cmd_ds_get_hw_spec));
	memset(payload->permanent_addr, 0xff, 6);

	sdiowl_send_cmd_sync(sc, cmd);
	payload = &sc->resp.params.hw_spec;

	memcpy(sc->bssid, payload->permanent_addr, 6);
	sc->number_of_antenna = payload->number_of_antenna;
	sc->reg_code = payload->region_code;
	sc->mp_end_port = payload->mp_end_port;

	device_printf(sc->dev, "MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
		      sc->bssid[0],
		      sc->bssid[1],
		      sc->bssid[2],
		      sc->bssid[3],
		      sc->bssid[4],
		      sc->bssid[5]);
	device_printf(sc->dev, "Antennas: %d\n", sc->number_of_antenna);

	free(cmd, M_MVSDIOWL);

	return 0;
}

static int
sdiowl_probe(device_t dev)
{
	size_t sdio_vendor, sdio_product;

//	device_quiet(dev);
	device_set_desc(dev, "Marvell SD8787 WLAN driver");

	if(BUS_READ_IVAR(device_get_parent(dev), dev, MMC_IVAR_SDIO_VENDOR,
			 &sdio_vendor) ||
	   BUS_READ_IVAR(device_get_parent(dev), dev, MMC_IVAR_SDIO_PRODUCT,
			 &sdio_product)) {
		device_printf(dev, "Cannot get vendor/product/function from the bus!\n");
		return (ENXIO);
	}

	/*
	 * XXX: Extend and make a table here.
	 * firmware name should be determined here.
	 */
	if (sdio_vendor == 0x02DF && sdio_product == 0x9119)
		return (BUS_PROBE_DEFAULT);
	else
		return (ENXIO);
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
sdiowl_get_fw_status(struct sdiowl_softc *sc, uint16_t *s) {
	uint8_t s0, s1;

	if (sdiowl_read_1(sc, CARD_FW_STATUS0_REG, &s0) ||
	    sdiowl_read_1(sc, CARD_FW_STATUS1_REG, &s1))
		return (-1);

	*s = ((s1 << 8) | s0);

	return 0;
}

static int
sdiowl_check_fw_status(struct sdiowl_softc *sc) {
	uint16_t status, status_ok;
	int i, ret;

	status_ok = 0;
	ret = -1;

	for (i=0; i < 200; i++) {
		ret = sdiowl_get_fw_status(sc, &status);
		if (ret)
			continue;
		if (status == FIRMWARE_READY_SDIO) {
			status_ok = status;
			ret = 0;
			break;
		} else {
			pause("sdiowl", 10);
			ret = -1;
		}
	}

	return (ret);
}

static int
sdiowl_disable_host_int(struct sdiowl_softc *sc) {
	uint8_t host_int_mask;

	if (sdiowl_read_1(sc, HOST_INT_MASK_REG, &host_int_mask))
		return (-1);
	host_int_mask &= (uint8_t) ~HOST_INT_DISABLE;
	if (sdiowl_write_1(sc, HOST_INT_MASK_REG, host_int_mask)) {
		device_printf(sc->dev, "Disabling host interrupt failed!\n");
		return (-1);
	}

	if (MMCBUS_IO_SET_INTR(device_get_parent(sc->dev), sc->dev, NULL)) {
		device_printf(sc->dev, "Bus call to disable interrupt!\n");
		return (-1);
	}

	return 0;
}

static int
sdiowl_enable_host_int(struct sdiowl_softc *sc) {
	size_t handler = 0xdeadbeef;

	if (MMCBUS_IO_SET_INTR(device_get_parent(sc->dev), sc->dev, &handler)) {
		device_printf(sc->dev, "Bus call to enable interrupt!\n");
		return (-1);
	}

	if (sdiowl_write_1(sc, HOST_INT_MASK_REG, HOST_INT_ENABLE)) {
		device_printf(sc->dev, "Enabling host interrupt failed!\n");
		return (-1);
	}

	return 0;
}

static int
sdiowl_attach(device_t dev)
{
	struct sdiowl_softc *sc;
	struct ifnet *ifp;
	struct ieee80211com *ic;

	const struct firmware *fw;
	uint8_t funcs_enabled;
	uint8_t sdio_irq;
	size_t sdiofunc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	SDIOWL_LOCK_INIT(sc);
	sc->running = 1;
	sc->lastseq = 0;
	sc->bss_type = MWIFIEX_BSS_TYPE_STA;
	sc->irqstatus = 0;
	sc->rd_bitmap = 0;
	sc->wr_bitmap = 0;

	BUS_READ_IVAR(device_get_parent(dev), dev, MMC_IVAR_SDIO_FUNCTION,
	    &sdiofunc);
	sc->sdio_function = sdiofunc;

	/* Call FW loader from FreeBSD firmware framework */
	fw = firmware_get("sdiowl_fw");
	if (!fw) {
		device_printf(dev, "Firmware request failed!\n");
		return (-1);
	}

	if (MMCBUS_IO_F0_READ_1(device_get_parent(dev), dev,
	    SD_IO_CCCR_FN_ENABLE, &funcs_enabled))
		return (-1);

	/*
	 * XXX: We probably should just enable function from the driver code
	 * and not rely on MMC/SDIO stack to do this!
	 */
	if (funcs_enabled & (1 << sc->sdio_function))
		device_printf(dev, "My function is enabled, good!\n");
	else {
		device_printf(dev, "My func is NOT enabled, bad (mask = %d, func=%d)!\n", funcs_enabled, sc->sdio_function);
		return (-1);
	}

	/* Set block size */
	device_printf(dev, "Setting block size to %d bytes\n", SDIO_BLOCK_SIZE);
	if(MMCBUS_IO_SET_BLOCK_SIZE(device_get_parent(dev), dev, SDIO_BLOCK_SIZE))
		return (-1);

	/* ACK the first interrupt from bootloader, disable host intr mask */
	sdio_irq = 0;
	if(sdiowl_read_1(sc, HOST_INTSTATUS_REG, &sdio_irq))
		return (-1);

	/* Disable host interrupts */
	sdiowl_disable_host_int(sc);

	/* Get IO Port */
	uint8_t reg;
	if (sdiowl_read_1(sc, IO_PORT_0_REG, &reg))
		return (-1);
	sc->ioport = reg;
	if (sdiowl_read_1(sc, IO_PORT_1_REG, &reg))
		return (-1);
	sc->ioport |= reg << 8;
	if (sdiowl_read_1(sc, IO_PORT_2_REG, &reg))
		return (-1);
	sc->ioport |= reg << 16;
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


	/* XXX No idea but without this FW upload doesn't work :-( */
	char data[256];
	memset(data, 0, 256);
	if (MMCBUS_IO_READ_MULTI(device_get_parent(dev), dev, 0,
				 data, 256, 1)) {
		device_printf(dev, "Cannot read-multi\n");
	}

	/* Now upload FW to the card */
	uint8_t status, base0, base1, tries;
	uint32_t len, txlen, tx_blocks, offset;

	uint8_t *fwbuf_ = malloc(2048 + BUF_ALIGN, M_MVSDIOWL, M_WAITOK);

	size_t fwbuf_a = (size_t) fwbuf_;
	fwbuf_a &= ~ (BUF_ALIGN - 1) ;
	fwbuf_a += BUF_ALIGN;
	uint8_t *fwbuf = (uint8_t *) fwbuf_a;
	offset = len = 0;
	do {
		for (tries = 0; tries < 200; tries ++) {
			status = 0;
			/* Ask card about its status */
			if (sdiowl_read_1(sc, CARD_STATUS_REG, &status))
				return (-1);
			if (!(status | CARD_IO_READY | DN_LD_CARD_RDY)) {
				device_printf(dev,
				    "Card NOT ready to accept FW (status %d)\n",
				    status);
				return (-1);
			}

			if (sdiowl_read_1(sc, HOST_F1_RD_BASE_0, &base0) > 0 ||
			    sdiowl_read_1(sc, HOST_F1_RD_BASE_1, &base1) > 0) {
				device_printf(dev, "Err while reading BASEx\n");
				return (-1);
			}

			len = (((base1 & 0xff) << 8) | (base0 & 0xff));
			if (len)
				break;
			pause("sdiowl", 10);
		}

		txlen = len;

		if (!len)
			break;
		if (len & 0x1) {
			device_printf(sc->dev, "CRC error?! len=0x%04X, txlen=%d\n", len, txlen);
			break;
		}

		if (fw->datasize - offset < txlen)
			txlen = fw->datasize - offset;

		tx_blocks = (txlen + SDIO_BLOCK_SIZE - 1)
			/ SDIO_BLOCK_SIZE;
		/* Copy payload to buffer */
		memset(fwbuf_, 0, 2048 + BUF_ALIGN);
		memmove(fwbuf, (uint8_t *)((size_t)fw->data + offset), txlen);

		if (MMCBUS_IO_WRITE_FIFO(device_get_parent(dev), dev, sc->ioport,
		    (uint8_t *) fwbuf,
		    tx_blocks * SDIO_BLOCK_SIZE)) {
			device_printf(dev, "Cannot write-fifo\n");
			break;
		}

		offset += txlen;

		if (offset >= fw->datasize)
			break;
		pause("sdiowl", 2);

	} while (true);

	device_printf(sc->dev, "FW download over, size %d bytes\n", offset);

	/* Check FW status */
	if (sdiowl_check_fw_status(sc)) {
		device_printf(sc->dev, "FW did not come up in time\n");
		return (-1);
	} else device_printf(sc->dev, "FW READY\n");

	/* Enable host interrupts */
	sdiowl_enable_host_int(sc);

	/* Now create the interface */
	ifp = sc->sc_ifp = if_alloc(IFT_IEEE80211);
	if (ifp == NULL) {
		device_printf(sc->dev, "cannot if_alloc()\n");
		return ENOSPC;
	}
	ic = ifp->if_l2com;

	/* set these up early for if_printf use */
	if_initname(ifp, device_get_name(sc->dev),
		    device_get_unit(sc->dev));

	sc->sc_tq = taskqueue_create("mv_sdiowl_taskq", M_NOWAIT,
		taskqueue_thread_enqueue, &sc->sc_tq);
	taskqueue_start_threads(&sc->sc_tq, 1, PI_NET,
		"%s taskq", ifp->if_xname);

	TASK_INIT(&sc->sc_cmdtask, 0, sdiowl_send_cmd, sc);
	cv_init(&sc->sc_cmdtask_finished, "sdiowl task finish marker");

	sdiowl_send_init(sc);
	sdiowl_get_hw_spec(sc);

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
