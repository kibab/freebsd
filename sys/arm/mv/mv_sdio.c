/*-
 * Copyright (c) 2009 Semihalf, Rafal Czubak
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Driver for Marvell Integrated SDIO Host Controller.
 * Works stable in DMA mode. PIO mode has problems with large data transfers
 * (timeouts).
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include <arm/mv/mvreg.h>
#include <arm/mv/mvvar.h>

#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcvar.h>
#include <dev/mmc/mmcbrvar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "mmcbr_if.h"

#include "mv_sdio.h"

/* Minimum DMA segment size. */
#define MV_SDIO_DMA_SEGMENT_SIZE	4096

/* Transferred block size. */
#define MV_SDIO_BLOCK_SIZE  512

/* Maximum number of blocks the controller can handle. */
#define MV_SDIO_BLOCKS_MAX		65535

/* Halfword bit masks used for command response extraction. */
#define MV_SDIO_RSP48_BM2	0x0003	/* Lower 2 bits. */
#define MV_SDIO_RSP48_BM6	0x003f	/* Lower 6 bits. */
#define MV_SDIO_RSP48_BM16	0xffff	/* 16 bits */

/* SDIO aggregated command interrupts */
#define MV_SDIO_IRQS_CMD (MV_SDIO_IRQ_CMD | MV_SDIO_IRQ_UNEXPECTED_RSP)
#define MV_SDIO_EIRQS_CMD (MV_SDIO_EIRQ_CMD_TMO | MV_SDIO_EIRQ_CMD_CRC7 | \
    MV_SDIO_EIRQ_CMD_ENDBIT | MV_SDIO_EIRQ_CMD_INDEX | \
    MV_SDIO_EIRQ_CMD_STARTBIT | MV_SDIO_EIRQ_RSP_TBIT)

/* SDIO aggregated data interrupts */
#define MV_SDIO_IRQS_DATA (MV_SDIO_IRQ_XFER | MV_SDIO_IRQ_TX_EMPTY | \
    MV_SDIO_IRQ_RX_FULL | MV_SDIO_IRQ_DMA | MV_SDIO_IRQ_AUTOCMD12)
#define MV_SDIO_EIRQS_DATA (MV_SDIO_EIRQ_DATA_TMO | \
    MV_SDIO_EIRQ_DATA_CRC16 | MV_SDIO_EIRQ_DATA_ENDBIT | \
    MV_SDIO_EIRQ_AUTOCMD12 | MV_SDIO_EIRQ_XFER_SIZE | \
    MV_SDIO_EIRQ_CRC_ENDBIT | MV_SDIO_EIRQ_CRC_STARTBIT | \
    MV_SDIO_EIRQ_CRC_STAT)

/*
 * Timing configuration.
 */

/* SDIO controller base clock frequency. */
#define MV_SDIO_F_BASE			100000000		/* 200 MHz */

/* Maximum SD clock frequency. */
#define MV_SDIO_F_MAX			(MV_SDIO_F_BASE / 2)	/* 50 MHz */

/* Maximum timeout value. */
#define MV_SDIO_TMO_MAX			0xf

/* Reset delay in microseconds. */
#define MV_SDIO_RESET_DELAY		10000	/* 10 ms */

/* Empty FIFO polling delay. */
#define MV_SDIO_FIFO_EMPTY_DELAY	1000	/* 1 ms */

/* Delays between operations on multiple blocks. */
#define MV_SDIO_RD_DELAY		50 /*50*/	/* Read access time. */
#define MV_SDIO_WR_DELAY		10 /*10*/	/* Write access time. */

/* Maximum clock divider value. */
#define MV_SDIO_CLK_DIV_MAX		0x7ff

struct mv_sdio_softc {
	device_t		sc_dev;
	device_t		sc_child;

	bus_space_handle_t	sc_bsh;
	bus_space_tag_t		sc_bst;

	int			sc_use_dma;
	bus_dma_tag_t		sc_dmatag;
	bus_dmamap_t		sc_dmamap;
	uint8_t			*sc_dmamem;
	bus_addr_t		sc_physaddr;
	int			sc_mapped;
	size_t			sc_dma_size;

	struct resource		*sc_mem_res;
	int			sc_mem_rid;

	struct resource		*sc_irq_res;
	int			sc_irq_rid;
	void			*sc_ihl;

	struct resource		*sc_cd_irq_res;
	int			sc_cd_irq_rid;
	void			*sc_cd_ihl;

	uint32_t		sc_irq_mask;
	uint32_t		sc_eirq_mask;

	struct task		sc_card_task;
	struct callout		sc_card_callout;

	struct mtx		sc_mtx;

	int			sc_bus_busy;
	int			sc_card_present;
	struct mmc_host		sc_host;
	struct mmc_request	*sc_req;
	struct mmc_command	*sc_curcmd;

	uint32_t		sc_data_offset;
};

/* Read/write data from/to registers.*/
static uint32_t MV_SDIO_RD4(struct mv_sdio_softc *, bus_size_t);
static void MV_SDIO_WR4(struct mv_sdio_softc *, bus_size_t, uint32_t);

static int mv_sdio_probe(device_t);
static int mv_sdio_attach(device_t);

static int mv_sdio_read_ivar(device_t, device_t, int, uintptr_t *);
static int mv_sdio_write_ivar(device_t, device_t, int, uintptr_t);

static int mv_sdio_update_ios(device_t, device_t);
static int mv_sdio_request(device_t, device_t, struct mmc_request *);
static int mv_sdio_get_ro(device_t, device_t);
static int mv_sdio_acquire_host(device_t, device_t);
static int mv_sdio_release_host(device_t, device_t);

/* Finalizes active MMC request. */
static void mv_sdio_finalize_request(struct mv_sdio_softc *);

/* Initializes controller's registers. */
static void mv_sdio_init(device_t);

/* Initializes host structure. */
static void mv_sdio_init_host(struct mv_sdio_softc *);

/* Used to add and handle sysctls. */
static void mv_sdio_add_sysctls(struct mv_sdio_softc *);
static int mv_sdio_sysctl_use_dma(SYSCTL_HANDLER_ARGS);

/* DMA initialization and cleanup functions. */
static int mv_sdio_dma_init(struct mv_sdio_softc *);
static void mv_sdio_dma_finish(struct mv_sdio_softc *);

/* DMA map load callback. */
static void mv_sdio_getaddr(void *, bus_dma_segment_t *, int, int);

/* Prepare command/data before transaction. */
static int mv_sdio_start_command(struct mv_sdio_softc *, struct
    mmc_command *);
static int mv_sdio_start_data(struct mv_sdio_softc *, struct mmc_data *);

/* Finish command after transaction. */
static void mv_sdio_finish_command(struct mv_sdio_softc *);

/* Response handling. */
static void mv_sdio_handle_136bit_resp(struct mv_sdio_softc *);
static void mv_sdio_handle_48bit_resp(struct mv_sdio_softc *,
    struct mmc_command *);

/* Interrupt handler and interrupt helper functions. */
static void mv_sdio_intr(void *);
static void mv_sdio_cmd_intr(struct mv_sdio_softc *, uint32_t, uint32_t);
static void mv_sdio_data_intr(struct mv_sdio_softc *, uint32_t, uint32_t);
static void mv_sdio_disable_intr(struct mv_sdio_softc *);

/* Used after card detect interrupt has been handled. */
static void mv_sdio_card_task(void *, int);

/* Read/write data from FIFO in PIO mode. */
static uint32_t mv_sdio_read_fifo(struct mv_sdio_softc *);
static void mv_sdio_write_fifo(struct mv_sdio_softc *, uint32_t);

/*
 * PIO mode handling.
 *
 * Inspired by sdhci(4) driver routines.
 */
static void mv_sdio_transfer_pio(struct mv_sdio_softc *);
static void mv_sdio_read_block_pio(struct mv_sdio_softc *);
static void mv_sdio_write_block_pio(struct mv_sdio_softc *);


static device_method_t mv_sdio_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, mv_sdio_probe),
	DEVMETHOD(device_attach, mv_sdio_attach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar, mv_sdio_read_ivar),
	DEVMETHOD(bus_write_ivar, mv_sdio_write_ivar),

	/* mmcbr_if */
	DEVMETHOD(mmcbr_update_ios, mv_sdio_update_ios),
	DEVMETHOD(mmcbr_request, mv_sdio_request),
	DEVMETHOD(mmcbr_get_ro, mv_sdio_get_ro),
	DEVMETHOD(mmcbr_acquire_host, mv_sdio_acquire_host),
	DEVMETHOD(mmcbr_release_host, mv_sdio_release_host),

	{0, 0},
};

static driver_t mv_sdio_driver = {
	"sdio",
	mv_sdio_methods,
	sizeof(struct mv_sdio_softc),
};
static devclass_t mv_sdio_devclass;

DRIVER_MODULE( sdio, simplebus, mv_sdio_driver, mv_sdio_devclass, 0, 0);


static __inline uint32_t
MV_SDIO_RD4(struct mv_sdio_softc *sc, bus_size_t off)
{

	return (bus_read_4(sc->sc_mem_res, off));
}

static __inline void
MV_SDIO_WR4(struct mv_sdio_softc *sc, bus_size_t off, uint32_t val)
{

	bus_write_4(sc->sc_mem_res, off, val);
}

static int platform_sdio_slot_signal( int signal )
{
  switch( signal )
  {
    case MV_SDIO_SIG_CD:
    {
      return -1;
      break;
    }
    case MV_SDIO_SIG_WP:
      return 0;
      break;
    default:
      return -1;
      break;
  }
  
  return 0;
}

static int
mv_sdio_probe(device_t dev)
{
	uint32_t device, revision;
  
	if (!ofw_bus_is_compatible(dev, "mrvl,sdio"))
		return (ENXIO);


	soc_id(&device, &revision);

	switch (device) {
	case MV_DEV_88F6281:
		break;
	default:
		return (ENXIO);
	}

	device_set_desc(dev, "Marvell Integrated SDIO Host Controller");

	return (BUS_PROBE_SPECIFIC);
}

static int
mv_sdio_attach(device_t dev)
{
	struct mv_sdio_softc *sc;
	int task_initialized = 0;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Allocate memory and interrupt resources. */
	sc->sc_mem_rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_mem_rid, RF_ACTIVE);

	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "Could not allocate memory!\n");
		goto fail;
	}

	sc->sc_irq_rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE);

	if (sc->sc_irq_res == NULL) {
		device_printf(dev, "Could not allocate IRQ!\n");
		goto fail;
	}

	sc->sc_bst = rman_get_bustag(sc->sc_mem_res);
	sc->sc_bsh = rman_get_bushandle(sc->sc_mem_res);

  
	/* Initialize host controller's registers. */
	mv_sdio_init(dev);

	/* Try to setup DMA. */
	sc->sc_mapped = 0;	/* No DMA buffer is mapped. */
	sc->sc_use_dma = 1;	/* DMA mode is preferred to PIO mode. */

	if (mv_sdio_dma_init(sc) < 0) {
		device_printf(dev, "Falling back to PIO mode.\n");
		sc->sc_use_dma = 0;
	}

	/* Add sysctls. */
	mv_sdio_add_sysctls(sc);

	if (platform_sdio_slot_signal(MV_SDIO_SIG_CD) != -1) {
		/* Check if card is present in the slot. */
		if (platform_sdio_slot_signal(MV_SDIO_SIG_CD) == 1)
			sc->sc_card_present = 1;
	}

	TASK_INIT(&sc->sc_card_task, 0, mv_sdio_card_task, sc);
	callout_init(&sc->sc_card_callout, 1);
	task_initialized = 1;

	/* Setup interrupt. */
	if (bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC |
	    INTR_MPSAFE, NULL, mv_sdio_intr, sc, &sc->sc_ihl) != 0) {
		device_printf(dev, "Could not setup interrupt!\n");
		goto fail;
	}

	/* Host can be acquired. */
	sc->sc_bus_busy = 0;

	/*
	 * Attach MMC bus only if the card is in the slot or card detect is
	 * not supported on the platform.
	 */
	if ((platform_sdio_slot_signal(MV_SDIO_SIG_CD) == -1) ||
	    sc->sc_card_present) {
		sc->sc_child = device_add_child(dev, "mmc", -1);

		if (sc->sc_child == NULL) {
			device_printf(dev, "Could not add MMC bus!\n");
			goto fail;
		}

		/* Initialize host structure for MMC bus. */
		mv_sdio_init_host(sc);

		device_set_ivars(sc->sc_child, &sc->sc_host);
	}

	return (bus_generic_attach(dev));

fail:
	mv_sdio_dma_finish(sc);
	if (task_initialized) {
		callout_drain(&sc->sc_card_callout);
		taskqueue_drain(taskqueue_swi, &sc->sc_card_task);
	}
	if (sc->sc_ihl != NULL)
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_ihl);
	if (sc->sc_cd_ihl != NULL)
		bus_teardown_intr(dev, sc->sc_cd_irq_res, sc->sc_cd_ihl);
	if (sc->sc_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_res);
	if (sc->sc_cd_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_cd_irq_rid,
		    sc->sc_cd_irq_res);
	if (sc->sc_mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_mem_rid,
		    sc->sc_mem_res);
	mtx_destroy(&sc->sc_mtx);
	return (ENXIO);
}

static int
mv_sdio_update_ios(device_t brdev, device_t reqdev)
{
	struct mv_sdio_softc *sc;
	struct mmc_host *host;
	struct mmc_ios *ios;
	uint32_t xfer, clk_div, host_cr;

	sc = device_get_softc(brdev);
	host = device_get_ivars(reqdev);
	ios = &host->ios;

	mtx_lock(&sc->sc_mtx);

	if (ios->power_mode == power_off)
		/* Re-initialize the controller. */
		mv_sdio_init(brdev);

	xfer = MV_SDIO_RD4(sc, MV_SDIO_XFER);

	if (ios->clock == 0) {
		/* Disable clock. */
		xfer |= MV_SDIO_XFER_STOP_CLK;
		MV_SDIO_WR4(sc, MV_SDIO_XFER, xfer);

		/* Set maximum clock divider. */
		MV_SDIO_WR4(sc, MV_SDIO_CLK_DIV, MV_SDIO_CLK_DIV_MAX);
	} else {
		/*
		 * Calculate and set clock divider.
		 * Clock rate value is:
		 *     clock = MV_SDIO_F_BASE / (clk_div + 1)
		 * Thus we calculate the divider value as:
		 *     clk_div = (MV_SDIO_F_BASE / clock) - 1
		 */
		clk_div = (MV_SDIO_F_BASE / ios->clock) - 1;
		if (clk_div > MV_SDIO_CLK_DIV_MAX)
			clk_div = MV_SDIO_CLK_DIV_MAX;
		MV_SDIO_WR4(sc, MV_SDIO_CLK_DIV, clk_div);
    
		/* Enable clock. */
		xfer &= ~MV_SDIO_XFER_STOP_CLK;
		MV_SDIO_WR4(sc, MV_SDIO_XFER, xfer);
	}

	host_cr = MV_SDIO_RD4(sc, MV_SDIO_HOST_CR);

	/* Set card type. */
	if (host->mode == mode_mmc)
		host_cr |= MV_SDIO_HOST_CR_MMC;		/* MMC card. */
	else
		host_cr &= ~MV_SDIO_HOST_CR_MMC;	/* SD card. */

	/* Set bus width. */
	if (ios->bus_width == bus_width_4)
		host_cr |= MV_SDIO_HOST_CR_4BIT;	/* 4-bit bus width */
	else
		host_cr &= ~MV_SDIO_HOST_CR_4BIT;	/* 1-bit bus width */

	/* Set high/normal speed mode. */
#if 0 /* Some cards have problems with the highspeed-mode 
       * Not selecting High-Speed mode enables all cards to work
       */
  
	if ((ios->timing == bus_timing_hs ) && ( 1 == 0 ) )
		host_cr |= MV_SDIO_HOST_CR_HIGHSPEED;
	else
#endif
		host_cr &= ~MV_SDIO_HOST_CR_HIGHSPEED;

	MV_SDIO_WR4(sc, MV_SDIO_HOST_CR, host_cr);

	mtx_unlock(&sc->sc_mtx);

	return (0);
}

static int
mv_sdio_request(device_t brdev, device_t reqdev, struct mmc_request *req)
{
	struct mv_sdio_softc *sc;
	int rv;

	sc = device_get_softc(brdev);
	rv = EBUSY;

	mtx_lock(&sc->sc_mtx);

	if (sc->sc_req != NULL) {
		mtx_unlock(&sc->sc_mtx);
		return (rv);
	}

	sc->sc_req = req;
/*
  device_printf(sc->sc_dev, "cmd %d (hw state 0x%04x)\n",
          req->cmd->opcode , MV_SDIO_RD4( sc, MV_SDIO_HOST_SR ) );
*/
	rv = mv_sdio_start_command(sc, req->cmd);

	mtx_unlock(&sc->sc_mtx);

	return (rv);
}

static int
mv_sdio_get_ro(device_t brdev, device_t reqdev)
{
	int rv;

	/* Check if card is read only. */
	rv = platform_sdio_slot_signal(MV_SDIO_SIG_WP);

	/*
	 * Assume that card is not write protected, when platform doesn't
	 * support WP signal.
	 */
	if (rv < 0)
		rv = 0;

	return (rv);
}

static int
mv_sdio_acquire_host(device_t brdev, device_t reqdev)
{
	struct mv_sdio_softc *sc;
	int rv;

	sc = device_get_softc(brdev);
	rv = 0;

	mtx_lock(&sc->sc_mtx);
	while (sc->sc_bus_busy)
		rv = mtx_sleep(sc, &sc->sc_mtx, PZERO, "sdioah", 0);
	sc->sc_bus_busy++;
	mtx_unlock(&sc->sc_mtx);

	return (rv);
}

static int
mv_sdio_release_host(device_t brdev, device_t reqdev)
{
	struct mv_sdio_softc *sc;

	sc = device_get_softc(brdev);

	mtx_lock(&sc->sc_mtx);
	sc->sc_bus_busy--;
	wakeup(sc);
	mtx_unlock(&sc->sc_mtx);

	return (0);
}

static void
mv_sdio_finalize_request(struct mv_sdio_softc *sc)
{
	struct mmc_request *req;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	req = sc->sc_req;

	if (req) {
		/* Finalize active request. */
    /*device_printf(sc->sc_dev, "Finalize request %i\n",req->cmd->opcode);*/
		sc->sc_req = NULL;
		sc->sc_curcmd = NULL;
		req->done(req);
    
    
	} else
		device_printf(sc->sc_dev, "No active request to finalize!\n");
}

static void
mv_sdio_init(device_t dev)
{
	struct mv_sdio_softc *sc;
	uint32_t host_cr;

	sc = device_get_softc(dev);

	/* Disable interrupts. */
	sc->sc_irq_mask = 0;
	sc->sc_eirq_mask = 0;
	MV_SDIO_WR4(sc, MV_SDIO_IRQ_EN, sc->sc_irq_mask);
	MV_SDIO_WR4(sc, MV_SDIO_EIRQ_EN, sc->sc_eirq_mask);

	/* Clear interrupt status registers. */
	MV_SDIO_WR4(sc, MV_SDIO_IRQ_SR, MV_SDIO_IRQ_ALL);
	MV_SDIO_WR4(sc, MV_SDIO_EIRQ_SR, MV_SDIO_EIRQ_ALL);

	/* Enable interrupt status registers. */
	MV_SDIO_WR4(sc, MV_SDIO_IRQ_SR_EN, MV_SDIO_IRQ_ALL);
	MV_SDIO_WR4(sc, MV_SDIO_EIRQ_SR_EN, MV_SDIO_EIRQ_ALL);

	/* Initialize Host Control Register. */
	host_cr = (MV_SDIO_HOST_CR_PUSHPULL | MV_SDIO_HOST_CR_BE |
             MV_SDIO_HOST_CR_TMOVAL(MV_SDIO_TMO_MAX) | MV_SDIO_HOST_CR_TMO);

	MV_SDIO_WR4(sc, MV_SDIO_HOST_CR, host_cr);

	/* Stop clock and reset Transfer Mode Register. */
	MV_SDIO_WR4(sc, MV_SDIO_XFER, MV_SDIO_XFER_STOP_CLK);

	/* Set maximum clock divider value. */
	MV_SDIO_WR4(sc, MV_SDIO_CLK_DIV, MV_SDIO_CLK_DIV_MAX);

	/* Reset status, state machine and FIFOs synchronously. */
	MV_SDIO_WR4(sc, MV_SDIO_SW_RESET, MV_SDIO_SW_RESET_ALL);
	    DELAY(MV_SDIO_RESET_DELAY);
}

static void
mv_sdio_init_host(struct mv_sdio_softc *sc)
{
	struct mmc_host *host;

	host = &sc->sc_host;

	/* Clear host structure. */
	bzero(host, sizeof(struct mmc_host));

	/* Calculate minimum and maximum operating frequencies. */
	host->f_min = MV_SDIO_F_BASE / (MV_SDIO_CLK_DIV_MAX + 1);
	host->f_max = MV_SDIO_F_MAX;
  
	/* Set operation conditions (voltage). */
	host->host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;

	/* Set additional host controller capabilities. */
	host->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_HSPEED;
}

static void
mv_sdio_add_sysctls(struct mv_sdio_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *children;
	struct sysctl_oid *tree;

	ctx = device_get_sysctl_ctx(sc->sc_dev);
	children = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->sc_dev));
	tree = SYSCTL_ADD_NODE(ctx, children, OID_AUTO, "params",
	    CTLFLAG_RD, 0, "Driver parameters");
	children = SYSCTL_CHILDREN(tree);

	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "use_dma",
	    CTLTYPE_UINT | CTLFLAG_RW, sc, 0, mv_sdio_sysctl_use_dma,
	    "I", "Use DMA for data transfers (0-1)");
}

/*
 * This sysctl allows switching between DMA and PIO modes for data transfers:
 *
 * dev.mv_sdio.<unit>.params.use_dma
 *
 * Values:
 *
 * - 1 sets DMA mode
 * - 0 sets PIO mode
 *
 * Driver uses DMA mode by default.
 */
static int
mv_sdio_sysctl_use_dma(SYSCTL_HANDLER_ARGS)
{
	struct mv_sdio_softc *sc;
	uint32_t use_dma;
	int error;

	sc = (struct mv_sdio_softc *)arg1;

	use_dma = sc->sc_use_dma;

	error = sysctl_handle_int(oidp, &use_dma, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (use_dma > 1)
		return (EINVAL);

	mtx_lock(&sc->sc_mtx);

	/* Check if requested mode is already being used. */
	if (sc->sc_use_dma == use_dma) {
		mtx_unlock(&sc->sc_mtx);
		return (EPERM);
	}

	if (!(sc->sc_mapped)) {
		device_printf(sc->sc_dev, "DMA not initialized!\n");
		mtx_unlock(&sc->sc_mtx);
		return (ENOMEM);
	}

	/* Set new mode. */
	sc->sc_use_dma = use_dma;

	mtx_unlock(&sc->sc_mtx);

	return (0);
}

static void
mv_sdio_getaddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error != 0)
		return;

	/* Get first segment's physical address. */
	*(bus_addr_t *)arg = segs->ds_addr;
}

static int
mv_sdio_dma_init(struct mv_sdio_softc *sc)
{
	device_t dev;
	bus_size_t dmabuf_size;

	dev = sc->sc_dev;
	dmabuf_size = MAXPHYS;

	/* Create DMA tag. */
	if (bus_dma_tag_create(bus_get_dma_tag(dev),	/* parent */
	    MV_SDIO_DMA_SEGMENT_SIZE, 0,	/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,		/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filtfunc, filtfuncarg */
	    MAXPHYS, 1,				/* maxsize, nsegments */
	    MAXPHYS, BUS_DMA_ALLOCNOW,		/* maxsegsz, flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &sc->sc_dmatag) != 0) {
		device_printf(dev, "Could not create DMA tag!\n");
		return (-1);
	}

	/* Allocate DMA memory. */
	if (bus_dmamem_alloc(sc->sc_dmatag, (void **)&sc->sc_dmamem,
	    BUS_DMA_NOWAIT, &sc->sc_dmamap) != 0) {
		device_printf(dev, "Could not allocate DMA memory!\n");
		mv_sdio_dma_finish(sc);
		return (-1);
	}

	/* Find the biggest available DMA buffer size. */
	while (bus_dmamap_load(sc->sc_dmatag, sc->sc_dmamap,
	    (void *)sc->sc_dmamem, dmabuf_size, mv_sdio_getaddr,
	    &sc->sc_physaddr, 0) != 0) {
		dmabuf_size >>= 1;
		if (dmabuf_size < MV_SDIO_BLOCK_SIZE) {
			device_printf(dev, "Could not load DMA map!\n");
			mv_sdio_dma_finish(sc);
			return (-1);
		}
	}

	sc->sc_mapped++;
	sc->sc_dma_size = dmabuf_size;

	return (0);
}

static void
mv_sdio_dma_finish(struct mv_sdio_softc *sc)
{

	/* Free DMA resources. */
	if (sc->sc_mapped) {
		bus_dmamap_unload(sc->sc_dmatag, sc->sc_dmamap);
		sc->sc_mapped--;
	}
	if (sc->sc_dmamem != NULL)
		bus_dmamem_free(sc->sc_dmatag, sc->sc_dmamem, sc->sc_dmamap);
	if (sc->sc_dmamap != NULL)
		bus_dmamap_destroy(sc->sc_dmatag, sc->sc_dmamap);
	if (sc->sc_dmatag != NULL)
		bus_dma_tag_destroy(sc->sc_dmatag);
}

static int
mv_sdio_start_command(struct mv_sdio_softc *sc, struct mmc_command *cmd)
{
	struct mmc_request *req;
	uint32_t cmdreg;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	req = sc->sc_req;

	sc->sc_curcmd = cmd;

	cmd->error = MMC_ERR_NONE;

	/* Check if card is in the slot. */
	if ((platform_sdio_slot_signal(MV_SDIO_SIG_CD) != -1) &&
	    (sc->sc_card_present == 0)) {
		cmd->error = MMC_ERR_FAILED;
		mv_sdio_finalize_request(sc);
		return (-1);
	}

	/* Check if clock is enabled. */
	if (MV_SDIO_RD4(sc, MV_SDIO_XFER) & MV_SDIO_XFER_STOP_CLK) {
		cmd->error = MMC_ERR_FAILED;
		mv_sdio_finalize_request(sc);
		return (-1);
	}

	/* Write command argument. */
	MV_SDIO_WR4(sc, MV_SDIO_CMD_ARGL, cmd->arg & 0xffff);
	MV_SDIO_WR4(sc, MV_SDIO_CMD_ARGH, cmd->arg >> 16);

	/* Determine response type. */
	if (cmd->flags & MMC_RSP_136)
		cmdreg = MV_SDIO_CMD_RSP_136;
	else if (cmd->flags & MMC_RSP_BUSY)
		cmdreg = MV_SDIO_CMD_RSP_48_BUSY;
	else if (cmd->flags & MMC_RSP_PRESENT)
		cmdreg = MV_SDIO_CMD_RSP_48;
	else {
		/* No response. */
		cmdreg = MV_SDIO_CMD_RSP_NONE;
		/* Enable host to detect unexpected response. */
		cmdreg |= MV_SDIO_CMD_UNEXPECTED_RSP;
		sc->sc_irq_mask |= MV_SDIO_CMD_UNEXPECTED_RSP;
	}

	/* Check command checksum if needed. */
	if (cmd->flags & MMC_RSP_CRC)
		cmdreg |= MV_SDIO_CMD_CRC7;
	/* Check command opcode if needed. */
	if (cmd->flags & MMC_RSP_OPCODE)
		cmdreg |= MV_SDIO_CMD_INDEX_CHECK;

	/* Set commannd opcode. */
	cmdreg |= MV_SDIO_CMD_INDEX(cmd->opcode);

	/* Setup interrupts. */
	sc->sc_irq_mask = MV_SDIO_IRQ_CMD;
	sc->sc_eirq_mask = MV_SDIO_EIRQ_ALL;

	/* Prepare data transfer. */
	if (cmd->data) {
		cmdreg |= (MV_SDIO_CMD_DATA_PRESENT | MV_SDIO_CMD_DATA_CRC16);
		if (mv_sdio_start_data(sc, cmd->data) < 0) {
			cmd->error = MMC_ERR_FAILED;
      printf("mv_sdio_start_data() failed!\n");
			mv_sdio_finalize_request(sc);
			return (-1);
		}
	}

	/* Write command register. */
	MV_SDIO_WR4(sc, MV_SDIO_CMD, cmdreg);

	/* Clear interrupt status. */
	MV_SDIO_WR4(sc, MV_SDIO_IRQ_SR, ~MV_SDIO_IRQ_CARD_EVENT /*MV_SDIO_IRQ_ALL*/);
	MV_SDIO_WR4(sc, MV_SDIO_EIRQ_SR, 0xffff /*MV_SDIO_EIRQ_ALL*/);

	/* Update interrupt/error interrupt enable registers. */
	MV_SDIO_WR4(sc, MV_SDIO_IRQ_EN, sc->sc_irq_mask);
	MV_SDIO_WR4(sc, MV_SDIO_EIRQ_EN, sc->sc_eirq_mask);

	/* Do not complete request, interrupt handler will do this. */
	return (0);
}

static void
mv_sdio_finish_command(struct mv_sdio_softc *sc)
{
	struct mmc_command *cmd;
	struct mmc_data *data;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	cmd = sc->sc_curcmd;
	data = cmd->data;

	/* Get response. */
	if (cmd->flags & MMC_RSP_PRESENT) {
		if(cmd->flags & MMC_RSP_136)
			/* 136-bit response. */
			mv_sdio_handle_136bit_resp(sc);
		else
			/* 48-bit response. */
			mv_sdio_handle_48bit_resp(sc, NULL);
	}

	if (data) {
		/*
		 * Disable command complete interrupt. It has already been
		 * handled.
		 */
		sc->sc_irq_mask &= ~MV_SDIO_IRQ_CMD;

		/* Enable XFER interrupt. */
		sc->sc_irq_mask |= MV_SDIO_IRQ_XFER;

		/* Check which data interrupts we need to activate. */
		if (sc->sc_use_dma)
			/* DMA transaction. */
			sc->sc_irq_mask |= MV_SDIO_IRQ_DMA;
		else if (data->flags & MMC_DATA_READ)
			/* Read transaction in PIO mode. */
			sc->sc_irq_mask |= MV_SDIO_IRQ_RX_FULL;
		else
			/* Write transaction in PIO mode. */
			sc->sc_irq_mask |= MV_SDIO_IRQ_TX_EMPTY;

		/* Check if Auto-CMD12 interrupt will be needed. */
		if (sc->sc_req->stop)
			sc->sc_irq_mask |= MV_SDIO_IRQ_AUTOCMD12;

		/* Update interrupt enable register. */
		MV_SDIO_WR4(sc, MV_SDIO_IRQ_EN, sc->sc_irq_mask);
	} else {
		/* We're done. Disable interrupts and finalize request. */
		mv_sdio_disable_intr(sc);
		mv_sdio_finalize_request(sc);
	}
}

static int
mv_sdio_start_data(struct mv_sdio_softc *sc, struct mmc_data *data)
{
	struct mmc_command *stop;
	uint32_t autocmd12reg, xfer, host_sr;
	size_t blk_size, blk_count;
	int retries;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	/*
	 * No transfer can be started when FIFO_EMPTY bit in MV_SDIO_HOST_SR
	 * is not set. This bit is sometimes not set instantly after XFER
	 * interrupt has been asserted.
	 */
	host_sr = MV_SDIO_RD4(sc, MV_SDIO_HOST_SR);

	retries = 10;
	while (!(host_sr & MV_SDIO_HOST_SR_FIFO_EMPTY)) {
		if (retries == 0)
			return (-1);
		retries--;
		DELAY(MV_SDIO_FIFO_EMPTY_DELAY);
		host_sr = MV_SDIO_RD4(sc, MV_SDIO_HOST_SR);
	}

	/* Clear data offset. */
	sc->sc_data_offset = 0;

	/*
	 * Set block size. It can be less than or equal to MV_SDIO_BLOCK_SIZE
	 * bytes.
	 */
	blk_size = (data->len < MV_SDIO_BLOCK_SIZE) ? data->len :
	    MV_SDIO_BLOCK_SIZE;
	if (data->flags & MMC_DATA_MULTI)
		blk_size = data->blocksz > MV_SDIO_BLOCK_SIZE ?
		    MV_SDIO_BLOCK_SIZE : data->blocksz;
	MV_SDIO_WR4(sc, MV_SDIO_BLK_SIZE, blk_size);

	/* Set block count. */
	blk_count = (data->len + MV_SDIO_BLOCK_SIZE - 1) / MV_SDIO_BLOCK_SIZE;
	if (data->flags & MMC_DATA_MULTI)
		blk_count = data->len / blk_size;
	MV_SDIO_WR4(sc, MV_SDIO_BLK_COUNT, blk_count);
	device_printf(sc->sc_dev, "BLK SIZE: %d, COUNT: %d\n", blk_size, blk_count);

	/* We want to initiate transfer by software. */
	xfer = MV_SDIO_XFER_SW_WR_EN;

	if (sc->sc_use_dma) {
		/* Synchronize before DMA transfer. */
		if (data->flags & MMC_DATA_READ)
			bus_dmamap_sync(sc->sc_dmatag, sc->sc_dmamap,
			    BUS_DMASYNC_PREREAD);
		else {
			memcpy(sc->sc_dmamem, data->data, data->len);
			bus_dmamap_sync(sc->sc_dmatag, sc->sc_dmamap,
			    BUS_DMASYNC_PREWRITE);
		}

		/* Write DMA buffer address register. */
		MV_SDIO_WR4(sc, MV_SDIO_DMA_ADDRL, sc->sc_physaddr & 0xffff);
		MV_SDIO_WR4(sc, MV_SDIO_DMA_ADDRH, sc->sc_physaddr >> 16);
	} else
		/* Set PIO transfer mode. */
		xfer |= MV_SDIO_XFER_PIO;

	/*
	 * Prepare Auto-CMD12. This command is automatically sent to the card
	 * by the host controller to stop multiple-block data transaction.
	 */
	if (sc->sc_req->stop) {
		stop = sc->sc_req->stop;

		/* Set Auto-CMD12 argument. */
		MV_SDIO_WR4(sc, MV_SDIO_AUTOCMD12_ARGL, stop->arg & 0xffff);
		MV_SDIO_WR4(sc, MV_SDIO_AUTOCMD12_ARGH, stop->arg >> 16);

		/* Set Auto-CMD12 opcode. */
		autocmd12reg = MV_SDIO_AUTOCMD12_INDEX(stop->opcode);

		/* Check busy signal if needed. */
		if (stop->flags & MMC_RSP_BUSY)
			autocmd12reg |= MV_SDIO_AUTOCMD12_BUSY_CHECK;
		/* Check Auto-CMD12 index. */
		if (stop->flags & MMC_RSP_OPCODE)
			autocmd12reg |= MV_SDIO_AUTOCMD12_INDEX_CHECK;

		MV_SDIO_WR4(sc, MV_SDIO_AUTOCMD12, autocmd12reg);

		xfer |= MV_SDIO_XFER_AUTOCMD12;
	}

	/* Change data direction. */
	if (data->flags & MMC_DATA_READ)
		xfer |= MV_SDIO_XFER_TO_HOST;

	/* Write transfer mode register. */
	MV_SDIO_WR4(sc, MV_SDIO_XFER, xfer);

	return (0);
}

static void
mv_sdio_handle_136bit_resp(struct mv_sdio_softc *sc)
{
	struct mmc_command *cmd;
	uint32_t resp[8];
	uint32_t base, extra;
	int i, j, off;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	cmd = sc->sc_curcmd;

	/* Collect raw response from the controller. */
	for (i = 0; i < 8; i++)
		resp[i] = MV_SDIO_RD4(sc, MV_SDIO_RSP(i));

	/* Response passed to MMC bus is shifted by one byte. */
	extra = 0;
	for (i = 0, j = 7; i < 4; i++, j -= 2) {
		off = (i ? 0 : 2);
		base = resp[j] | (resp[j - 1] << (16 - off));
		cmd->resp[3 - i] = (base << (6 + off)) + extra;
		extra = base >> (26 - off);
	}
}

static void
mv_sdio_handle_48bit_resp(struct mv_sdio_softc *sc, struct mmc_command *stop)
{
	struct mmc_command *cmd;
	uint32_t resp[3], word;
	uint8_t *rp;
	int i;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	if (stop == NULL)
		cmd = sc->sc_curcmd;
	else
		cmd = stop;

	/* Collect raw response from the controller. */
	for (i = 0; i < 3; i++) {
		if (stop == NULL)
			resp[i] = MV_SDIO_RD4(sc, MV_SDIO_RSP(i));
		else
			resp[i] = MV_SDIO_RD4(sc, MV_SDIO_AUTOCMD12_RSP(i));
	}

	/* Clear MMC bus response buffer. */
	bzero(&cmd->resp[0], 4 * sizeof(uint32_t));

	/*
	 * Fill MMC bus response buffer.
	 */

	rp = (uint8_t *)&cmd->resp[0];

	/* Response bits [45:14] */
	word = (resp[1] & MV_SDIO_RSP48_BM16) |
	    ((resp[0] & MV_SDIO_RSP48_BM16) << 16);

	/* Response bits [15:14] and [13:8] */
	*rp++ = (resp[2] & MV_SDIO_RSP48_BM6) |
	    ((word & MV_SDIO_RSP48_BM2) << 6);

	/* Response bits [15:14] are already included. */
	word >>= 2;

	/* Response bits [45:16] */
	memcpy(rp, &word, sizeof(uint32_t));
}

static void
mv_sdio_intr(void *arg)
{
	struct mv_sdio_softc *sc;
	uint32_t irq_stat, eirq_stat;

  sc = (struct mv_sdio_softc *)arg;
#if 1
	device_printf(sc->sc_dev,"intr 0x%04x intr_en 0x%04x hw_state 0x%04x\n",
                MV_SDIO_RD4( sc, MV_SDIO_IRQ_SR ) ,  
                MV_SDIO_RD4( sc, MV_SDIO_IRQ_EN ),
                MV_SDIO_RD4( sc, MV_SDIO_HOST_SR ));
#endif
  
  
	mtx_lock(&sc->sc_mtx);

  

	irq_stat = MV_SDIO_RD4(sc, MV_SDIO_IRQ_SR) & sc->sc_irq_mask;
	eirq_stat = MV_SDIO_RD4(sc, MV_SDIO_EIRQ_SR) & sc->sc_eirq_mask;

	/*
	 * In case of error interrupt, interrupt cause will be identified by
	 * checking bits in error interrupt status register.
	 */
	irq_stat &= ~MV_SDIO_IRQ_ERR;

	/* Handle command interrupts. */
	if ((irq_stat & MV_SDIO_IRQS_CMD) ||
	    (eirq_stat & MV_SDIO_EIRQS_CMD)) {
		MV_SDIO_WR4(sc, MV_SDIO_IRQ_SR, irq_stat);
		MV_SDIO_WR4(sc, MV_SDIO_EIRQ_SR, eirq_stat);
		mv_sdio_cmd_intr(sc, irq_stat, eirq_stat);
		irq_stat &= ~MV_SDIO_IRQS_CMD;
		eirq_stat &= ~MV_SDIO_EIRQS_CMD;
	}

	/* Handle data interrupts. */
	if ((irq_stat & MV_SDIO_IRQS_DATA) ||
	    (eirq_stat & MV_SDIO_EIRQS_DATA)) {
		MV_SDIO_WR4(sc, MV_SDIO_IRQ_SR, irq_stat);
		MV_SDIO_WR4(sc, MV_SDIO_EIRQ_SR, eirq_stat);
		mv_sdio_data_intr(sc, irq_stat, eirq_stat);
		irq_stat &= ~MV_SDIO_IRQS_DATA;
		eirq_stat &= ~MV_SDIO_EIRQS_DATA;
	}

	/* Handle unexpected interrupts. */
	if (irq_stat) {
		device_printf(sc->sc_dev, "Unexpected interrupt(s)! "
		    "IRQ SR = 0x%08x\n", irq_stat);
		/* Clear interrupt status. */
		MV_SDIO_WR4(sc, MV_SDIO_IRQ_SR, irq_stat);
	}
	if (eirq_stat) {
		device_printf(sc->sc_dev, "Unexpected error interrupt(s)! "
		    "EIRQ SR = 0x%08x\n", eirq_stat);
		/* Clear error interrupt status. */
		MV_SDIO_WR4(sc, MV_SDIO_EIRQ_SR, eirq_stat);
	}

	mtx_unlock(&sc->sc_mtx);
}

static void
mv_sdio_cmd_intr(struct mv_sdio_softc *sc, uint32_t irq, uint32_t eirq)
{

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	if (!sc->sc_curcmd) {
		device_printf(sc->sc_dev, "Got command interrupt, but there "
		    "is no active command!\n");
		return;
	}

	/* Handle unexpected response error. */
	if (irq & MV_SDIO_IRQ_UNEXPECTED_RSP) {
		sc->sc_curcmd->error = MMC_ERR_FAILED;
		device_printf(sc->sc_dev, "Unexpected response!\n");
	}

	/* Handle errors. */
	if (eirq & MV_SDIO_EIRQ_CMD_TMO) {
		sc->sc_curcmd->error = MMC_ERR_TIMEOUT;
		device_printf(sc->sc_dev, "Error - command %d timeout!\n",
		    sc->sc_curcmd->opcode);
	} else if (eirq & MV_SDIO_EIRQ_CMD_CRC7) {
		sc->sc_curcmd->error = MMC_ERR_BADCRC;
		device_printf(sc->sc_dev, "Error - bad command %d "
		    "checksum!\n", sc->sc_curcmd->opcode);
	} else if (eirq & MV_SDIO_EIRQ_DATA_CRC16) {
		sc->sc_curcmd->error = MMC_ERR_BADCRC;
		device_printf(sc->sc_dev, "Error - bad command %d "
		    "DATA checksum!\n", sc->sc_curcmd->opcode);
	} else if (eirq & MV_SDIO_EIRQ_DATA_ENDBIT) {
		sc->sc_curcmd->error = MMC_ERR_BADCRC;
		device_printf(sc->sc_dev, "Error - bad command %d "
		    "end bit error!\n", sc->sc_curcmd->opcode);
	} else if (eirq) {
		sc->sc_curcmd->error = MMC_ERR_FAILED;
		device_printf(sc->sc_dev, "Command %d error eirq=%d!\n",
		    sc->sc_curcmd->opcode, eirq);
	}

	if (sc->sc_curcmd->error != MMC_ERR_NONE) {
		/* Error. Disable interrupts and finalize request. */
		mv_sdio_disable_intr(sc);
		mv_sdio_finalize_request(sc);
		return;
	}

	if (irq & MV_SDIO_IRQ_CMD)
		mv_sdio_finish_command(sc);
}

static void
mv_sdio_data_intr(struct mv_sdio_softc *sc, uint32_t irq, uint32_t eirq)
{
	struct mmc_command *stop;

	mtx_assert(&sc->sc_mtx, MA_OWNED);

	if (!sc->sc_curcmd) {
		device_printf(sc->sc_dev, "Got data interrupt, but there is "
		    "no active command.\n");
		return;
	}
	if ((!sc->sc_curcmd->data) && ((sc->sc_curcmd->flags &
	    MMC_RSP_BUSY) == 0)) {
		device_printf(sc->sc_dev, "Got data interrupt, but there is "
		    "no active data transaction.n\n");
		sc->sc_curcmd->error = MMC_ERR_FAILED;
		return;
	}

	/* Handle errors. */
	if(eirq & MV_SDIO_EIRQ_DATA_TMO) {
		sc->sc_curcmd->error = MMC_ERR_TIMEOUT;
		device_printf(sc->sc_dev, "Data %s timeout!\n",
		    (sc->sc_curcmd->data->flags & MMC_DATA_READ) ? "read" :
		    "write");
	} else if (eirq & (MV_SDIO_EIRQ_DATA_CRC16 |
	    MV_SDIO_EIRQ_DATA_ENDBIT)) {
		sc->sc_curcmd->error = MMC_ERR_BADCRC;
		device_printf(sc->sc_dev, "Bad data checksum!\n");
	} else if (eirq) {
		sc->sc_curcmd->error = MMC_ERR_FAILED;
		device_printf(sc->sc_dev, "Data error!: 0x%04X \n",
      eirq);

    if( 0 != ( eirq & MV_SDIO_EIRQ_CRC_STAT ) )
    {
      device_printf(sc->sc_dev, "MV_SDIO_EIRQ_CRC_STAT\n");
    }
	}

	/* Handle Auto-CMD12 error. */
	if (eirq & MV_SDIO_EIRQ_AUTOCMD12) {
		sc->sc_req->stop->error = MMC_ERR_FAILED;
		sc->sc_curcmd->error = MMC_ERR_FAILED;
		device_printf(sc->sc_dev, "Auto-CMD12 error!\n");
	}

	if (sc->sc_curcmd->error != MMC_ERR_NONE) {
		/* Error. Disable interrupts and finalize request. */
		mv_sdio_disable_intr(sc);
		mv_sdio_finalize_request(sc);
		return;
	}

	/* Handle PIO interrupt. */
	if (irq & (MV_SDIO_IRQ_TX_EMPTY | MV_SDIO_IRQ_RX_FULL))
		mv_sdio_transfer_pio(sc);

	/* Handle DMA interrupt. */
	if (irq & (MV_SDIO_IRQ_DMA)) {
		/* Synchronize DMA buffer. */
		if (MV_SDIO_RD4(sc, MV_SDIO_XFER) & MV_SDIO_XFER_TO_HOST) {
			bus_dmamap_sync(sc->sc_dmatag, sc->sc_dmamap,
			    BUS_DMASYNC_POSTWRITE);
			memcpy(sc->sc_curcmd->data->data, sc->sc_dmamem,
			    sc->sc_curcmd->data->len);
		} else
			bus_dmamap_sync(sc->sc_dmatag, sc->sc_dmamap,
			    BUS_DMASYNC_POSTREAD);

		/* Disable DMA interrupt. */
		sc->sc_irq_mask &= ~MV_SDIO_IRQ_DMA;
		MV_SDIO_WR4(sc, MV_SDIO_IRQ_EN, sc->sc_irq_mask);
	}

	/* Handle Auto-CMD12 interrupt. */
	if (irq & (MV_SDIO_IRQ_AUTOCMD12)) {
		stop = sc->sc_req->stop;
		/* Get 48-bit response. */
		mv_sdio_handle_48bit_resp(sc, stop);

		/* Disable Auto-CMD12 interrupt. */
		sc->sc_irq_mask &= ~MV_SDIO_IRQ_AUTOCMD12;
		MV_SDIO_WR4(sc, MV_SDIO_IRQ_EN, sc->sc_irq_mask);
	}

	/* Transfer finished. Disable interrupts and finalize request. */
	if (irq & (MV_SDIO_IRQ_XFER)) {
		mv_sdio_disable_intr(sc);
		mv_sdio_finalize_request(sc);
	}
}

static void
mv_sdio_disable_intr(struct mv_sdio_softc *sc)
{

	/* Disable interrupts that were enabled. */
	sc->sc_irq_mask &= ~(sc->sc_irq_mask);
	sc->sc_eirq_mask &= ~(sc->sc_eirq_mask);
	MV_SDIO_WR4(sc, MV_SDIO_IRQ_EN, sc->sc_irq_mask);
	MV_SDIO_WR4(sc, MV_SDIO_EIRQ_EN, sc->sc_eirq_mask);
}

static void
mv_sdio_card_task(void *arg, int pending)
{
	struct mv_sdio_softc *sc;

	int device_probe_and_attach_ret_val = 0;

	sc = (struct mv_sdio_softc *)arg;

	mtx_lock(&sc->sc_mtx);

	if (sc->sc_card_present) {
		if (sc->sc_child) {
			mtx_unlock(&sc->sc_mtx);
			return;
		}

		/* Initialize host controller's registers. */
		mv_sdio_init(sc->sc_dev);

		sc->sc_child = device_add_child(sc->sc_dev, "mmc", -1);
		if (sc->sc_child == NULL) {
			device_printf(sc->sc_dev, "Could not add MMC bus!\n");
			mtx_unlock(&sc->sc_mtx);
			return;
		}
            
		/* Initialize host structure for MMC bus. */
		mv_sdio_init_host(sc);

		device_set_ivars(sc->sc_child, &sc->sc_host);

		mtx_unlock(&sc->sc_mtx);

		device_probe_and_attach_ret_val = device_probe_and_attach(sc->sc_child);
        
		if( 0 != device_probe_and_attach_ret_val ) {
			device_printf(sc->sc_dev, "MMC bus failed on probe "
			"and attach! %i\n",device_probe_and_attach_ret_val);
			device_delete_child(sc->sc_dev, sc->sc_child);
			sc->sc_child = NULL;
		}
	} else {
		if (sc->sc_child == NULL) {
			mtx_unlock(&sc->sc_mtx);
			return;
		}

		mtx_unlock(&sc->sc_mtx);
		if (device_delete_child(sc->sc_dev, sc->sc_child) != 0) {
			device_printf(sc->sc_dev, "Could not delete MMC "
			    "bus!\n");
		}
		sc->sc_child = NULL;
	}
}

static uint32_t
mv_sdio_read_fifo(struct mv_sdio_softc *sc)
{
	uint32_t data;
	device_printf(sc->sc_dev, "This is not tested, yet MV_SDIO_FIFO not ensured\n ");
  
 	while (!(MV_SDIO_RD4(sc, MV_SDIO_IRQ_SR) & MV_SDIO_IRQ_RX_FULL));
	data = MV_SDIO_RD4(sc, MV_SDIO_FIFO);
	while (!(MV_SDIO_RD4(sc, MV_SDIO_IRQ_SR) & MV_SDIO_IRQ_RX_FULL));
	data |= (MV_SDIO_RD4(sc, MV_SDIO_FIFO) << 16);
	return data;
}

static void
mv_sdio_write_fifo(struct mv_sdio_softc *sc, uint32_t val)
{
	while (!(MV_SDIO_RD4(sc, MV_SDIO_IRQ_SR) & MV_SDIO_IRQ_TX_EMPTY));
	MV_SDIO_WR4(sc, MV_SDIO_FIFO, val & 0xffff);
	while (!(MV_SDIO_RD4(sc, MV_SDIO_IRQ_SR) & MV_SDIO_IRQ_TX_EMPTY));
	MV_SDIO_WR4(sc, MV_SDIO_FIFO, val >> 16);
}

static void
mv_sdio_transfer_pio(struct mv_sdio_softc *sc)
{
	struct mmc_command *cmd;

	cmd = sc->sc_curcmd;

	if (cmd->data->flags & MMC_DATA_READ) {
		while (MV_SDIO_RD4(sc, MV_SDIO_IRQ_SR) &
		    MV_SDIO_IRQ_RX_FULL) {
			mv_sdio_read_block_pio(sc);
			/*
			 * Assert delay after each block transfer to meet read
			 * access timing constraint.
			 */
			DELAY(MV_SDIO_RD_DELAY);
			if (sc->sc_data_offset >= cmd->data->len)
				break;
		}
		/* All blocks read in PIO mode. Disable interrupt. */
		sc->sc_irq_mask &= ~MV_SDIO_IRQ_RX_FULL;
		MV_SDIO_WR4(sc, MV_SDIO_IRQ_EN, sc->sc_irq_mask);
	} else {
		while (MV_SDIO_RD4(sc, MV_SDIO_IRQ_SR) &
		    MV_SDIO_IRQ_TX_EMPTY) {
			mv_sdio_write_block_pio(sc);
			/* Wait while card is programming the memory. */
			while ((MV_SDIO_RD4(sc, MV_SDIO_HOST_SR) &
              MV_SDIO_HOST_SR_CARD_BUSY));
			/*
			 * Assert delay after each block transfer to meet
			 * write access timing constraint.
			 */
			DELAY(MV_SDIO_WR_DELAY);

			if (sc->sc_data_offset >= cmd->data->len)
				break;
		}
		/* All blocks written in PIO mode. Disable interrupt. */
		sc->sc_irq_mask &= ~MV_SDIO_IRQ_TX_EMPTY;
		MV_SDIO_WR4(sc, MV_SDIO_IRQ_EN, sc->sc_irq_mask);
	}
}

static void
mv_sdio_read_block_pio(struct mv_sdio_softc *sc)
{
	uint32_t data;
	char *buffer;
	size_t left;

	buffer = sc->sc_curcmd->data->data;
	buffer += sc->sc_data_offset;
	/* Transfer one block at a time. */
	left = min(MV_SDIO_BLOCK_SIZE, sc->sc_curcmd->data->len -
	    sc->sc_data_offset);
	sc->sc_data_offset += left;

	/* Handle unaligned and aligned buffer cases. */
	if ((intptr_t)buffer & 3) {
		while (left > 3) {
			data = mv_sdio_read_fifo(sc);
			buffer[0] = data;
			buffer[1] = (data >> 8);
			buffer[2] = (data >> 16);
			buffer[3] = (data >> 24);
			buffer += 4;
			left -= 4;
		}
	} else {
		while (left > 3) {
			data = mv_sdio_read_fifo(sc);
			*((uint32_t *)buffer) = data;
			buffer += 4;
			left -= 4;
		}
	}
	/* Handle uneven size case. */
	if (left > 0) {
		data = mv_sdio_read_fifo(sc);
		while (left > 0) {
			*(buffer++) = data;
			data >>= 8;
			left--;
		}
	}
}

static void
mv_sdio_write_block_pio(struct mv_sdio_softc *sc)
{
	uint32_t data = 0;
	char *buffer;
	size_t left;

	buffer = sc->sc_curcmd->data->data;
	buffer += sc->sc_data_offset;
	/* Transfer one block at a time. */
	left = min(MV_SDIO_BLOCK_SIZE, sc->sc_curcmd->data->len -
	    sc->sc_data_offset);
	sc->sc_data_offset += left;

	/* Handle unaligned and aligned buffer cases. */
	if ((intptr_t)buffer & 3) {
		while (left > 3) {
			data = buffer[0] +
			    (buffer[1] << 8) +
			    (buffer[2] << 16) +
			    (buffer[3] << 24);
			left -= 4;
			buffer += 4;
			mv_sdio_write_fifo(sc, data);
		}
	} else {
		while (left > 3) {
			data = *((uint32_t *)buffer);
			left -= 4;
			buffer += 4;
			mv_sdio_write_fifo(sc, data);
		}
	}
	/* Handle uneven size case. */
	if (left > 0) {
		data = 0;
		while (left > 0) {
			data <<= 8;
			data += *(buffer++);
			left--;
		}
		mv_sdio_write_fifo(sc, data);
	}
}

static int
mv_sdio_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	struct mv_sdio_softc *sc;
	struct mmc_host *host;

	sc = device_get_softc(dev);
	host = device_get_ivars(child);

	switch (index) {
	case MMCBR_IVAR_BUS_MODE:
		*(int *)result = host->ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*(int *)result = host->ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*(int *)result = host->ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*(int *)result = host->ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*(int *)result = host->f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*(int *)result = host->f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*(int *)result = host->host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*(int *)result = host->mode;
		break;
	case MMCBR_IVAR_OCR:
		*(int *)result = host->ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*(int *)result = host->ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*(int *)result = host->ios.vdd;
		break;
	case MMCBR_IVAR_CAPS:
		*(int *)result = host->caps;
		break;
	case MMCBR_IVAR_TIMING:
		*(int *)result = host->ios.timing;
		break;
	case MMCBR_IVAR_MAX_DATA:
		mtx_lock(&sc->sc_mtx);
		/* Return maximum number of blocks the driver can handle. */
		if (sc->sc_use_dma)
			*(int *)result = (sc->sc_dma_size /
			    MV_SDIO_BLOCK_SIZE);
		else
			*(int *)result = MV_SDIO_BLOCKS_MAX;
		mtx_unlock(&sc->sc_mtx);
		break;
	default:
		return (EINVAL);
	}

	return (0);
}

static int
mv_sdio_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	struct mmc_host *host;

	host = device_get_ivars(child);

	switch (index) {
	case MMCBR_IVAR_BUS_MODE:
		host->ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		host->ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		host->ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		host->ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		host->mode = value;
		break;
	case MMCBR_IVAR_OCR:
		host->ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		host->ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		host->ios.vdd = value;
		break;
	case MMCBR_IVAR_TIMING:
		host->ios.timing = value;
		break;
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
	default:
		/* Instance variable not writable. */
		return (EINVAL);
	}

	return (0);
}

