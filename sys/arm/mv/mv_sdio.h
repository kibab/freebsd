/*
 *  Copyright (C) 2008 Marvell Semiconductors, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _MVSDMMC_INCLUDE
#define _MVSDMMC_INCLUDE


#define MVSDMMC_DMA_SIZE			65536



/*
 * The base MMC clock rate
 */

#define MVSDMMC_CLOCKRATE_MIN			100000
#define MVSDMMC_CLOCKRATE_MAX			50000000

#define MVSDMMC_BASE_FAST_CLOCK			200000000


/*
 * SDIO register
 */

#define MV_SDIO_DMA_ADDRL       0x000 
#define MV_SDIO_DMA_ADDRH       0x004
#define MV_SDIO_BLK_SIZE				0x008
#define MV_SDIO_BLK_COUNT				0x00c
#define MV_SDIO_CMD             0x01c
#define MV_SDIO_CMD_ARGL        0x010
#define MV_SDIO_CMD_ARGH        0x014
#define MV_SDIO_XFER    				0x018
#define MV_SDIO_HOST_SR         0x048
#define MV_SDIO_HOST_CR         0x050
#define MV_SDIO_SW_RESET        0x05c
#define MV_SDIO_IRQ_SR		      0x060
#define MV_SDIO_EIRQ_SR         0x064
#define MV_SDIO_IRQ_SR_EN       0x068
#define MV_SDIO_EIRQ_SR_EN      0x06c
#define MV_SDIO_IRQ_EN          0x070
#define MV_SDIO_EIRQ_EN         0x074
#define MV_SDIO_AUTOCMD12_ARGL	0x084
#define MV_SDIO_AUTOCMD12_ARGH	0x088
#define MV_SDIO_AUTOCMD12			  0x08c
#define MV_SDIO_CLK_DIV				  0x128
#define MV_SDIO_FIFO     0xa2100 /* FIXME!!! */

#define MV_SDIO_RSP(i)			  (0x020 + ((i)<<2))
#define MV_SDIO_AUTOCMD12_RSP(i)	(0x090 + ((i)<<2))

/*
 * SDIO Status-Register
 */
#define MV_SDIO_HOST_SR_CARD_BUSY (1<<1)
#define MV_SDIO_HOST_SR_FIFO_EMPTY (1<<13)



/*
 * SDIO_CMD
 */
#define MV_SDIO_CMD_RSP_NONE			(0 << 0)
#define MV_SDIO_CMD_RSP_136			(1 << 0)
#define MV_SDIO_CMD_RSP_48				(2 << 0)
#define MV_SDIO_CMD_RSP_48_BUSY			(3 << 0)
#define MV_SDIO_CMD_DATA_CRC16 (1<<2)
#define MV_SDIO_CMD_CRC7 (1<<3)
#define MV_SDIO_CMD_INDEX_CHECK (1<<4)
#define MV_SDIO_CMD_DATA_PRESENT (1<<5)
#define MV_SDIO_CMD_UNEXPECTED_RSP (1<<7)
#define MV_SDIO_CMD_INDEX(x) ( (x) << 8 )


/*
 * SDIO_XFER_MODE
 */
#define MV_SDIO_XFER_SW_WR_EN			(1 << 1)
#define MV_SDIO_XFER_AUTOCMD12			(1 << 2)
#define MV_SDIO_XFER_MODE_INT_CHK_EN		(1 << 3)
#define MV_SDIO_XFER_TO_HOST			(1 << 4)
#define MV_SDIO_XFER_STOP_CLK			(1 << 5)
#define MV_SDIO_XFER_PIO			(1 << 6)


/*
 * SDIO_HOST_CTRL
 */
#define MV_SDIO_HOST_CR_PUSHPULL  (1 <<  0)
#define MV_SDIO_HOST_CR_MMC       (3 <<  1)
#define MV_SDIO_HOST_CR_BE        (1 <<  3)
#define MV_SDIO_HOST_CR_4BIT      (1 <<  9)
#define MV_SDIO_HOST_CR_HIGHSPEED (1 << 10)

#define MV_SDIO_HOST_CR_TMOVAL(x) ((x) << 11)
#define MV_SDIO_HOST_CR_TMO       ( 1 << 15 ) 

/*
 * NORmal status bits
 */


#define MV_SDIO_IRQ_ERR            (1<<15)
#define MV_SDIO_IRQ_UNEXPECTED_RSP (1<<14)
#define MV_SDIO_IRQ_AUTOCMD12      (1<<13)
#define MV_SDIO_IRQ_SUSPENSE_ON_IRQ_EN (1<<12)
#define MV_SDIO_IRQ_IMB_FIFO_WORD_AVAIL (1<<11)
#define MV_SDIO_IRQ_IMB_FIFO_WORD_FILLED (1<<10)
#define MV_SDIO_IRQ_READ_WAIT (1<<9)
#define MV_SDIO_IRQ_CARD_EVENT (1<<8)
#define MV_SDIO_IRQ_RX_FULL (1<<5)
#define MV_SDIO_IRQ_TX_EMPTY (1<<4)
#define MV_SDIO_IRQ_DMA (1<<3)
#define MV_SDIO_IRQ_BLOCK_GAP (1<<2)
#define MV_SDIO_IRQ_XFER (1<<1)
#define MV_SDIO_IRQ_CMD (1<<0)

#define MV_SDIO_IRQ_ALL (MV_SDIO_IRQ_CMD | MV_SDIO_IRQ_XFER | MV_SDIO_IRQ_BLOCK_GAP | MV_SDIO_IRQ_DMA | MV_SDIO_IRQ_RX_FULL | MV_SDIO_IRQ_TX_EMPTY | MV_SDIO_IRQ_CARD_EVENT | MV_SDIO_IRQ_READ_WAIT | MV_SDIO_IRQ_IMB_FIFO_WORD_FILLED | MV_SDIO_IRQ_IMB_FIFO_WORD_AVAIL | MV_SDIO_IRQ_SUSPENSE_ON_IRQ_EN | MV_SDIO_IRQ_AUTOCMD12 | MV_SDIO_IRQ_UNEXPECTED_RSP | MV_SDIO_IRQ_ERR )

//#define MV_SDIO_IRQ_SR 


/*
 * ERR status bits
 */
#define MV_SDIO_EIRQ_CRC_STAT     (1<<14)
#define MV_SDIO_EIRQ_CRC_STARTBIT (1<<13)
#define MV_SDIO_EIRQ_CRC_ENDBIT   (1<<12)
#define MV_SDIO_EIRQ_RSP_TBIT     (1<<11)
#define MV_SDIO_EIRQ_XFER_SIZE    (1<<10)
#define MV_SDIO_EIRQ_CMD_STARTBIT (1<<9)
#define MV_SDIO_EIRQ_AUTOCMD12    (1<<8)
#define MV_SDIO_EIRQ_DATA_ENDBIT  (1<<6)
#define MV_SDIO_EIRQ_DATA_CRC16   (1<<5)
#define MV_SDIO_EIRQ_DATA_TMO     (1<<4)
#define MV_SDIO_EIRQ_CMD_INDEX    (1<<3)
#define MV_SDIO_EIRQ_CMD_ENDBIT   (1<<2)
#define MV_SDIO_EIRQ_CMD_CRC7     (1<<1)
#define MV_SDIO_EIRQ_CMD_TMO      (1<<0)

#define MV_SDIO_EIRQ_ALL (MV_SDIO_EIRQ_CMD_TMO | \
                          MV_SDIO_EIRQ_CMD_CRC7 | \
                          MV_SDIO_EIRQ_CMD_ENDBIT | \
                          MV_SDIO_EIRQ_CMD_INDEX | \
                          MV_SDIO_EIRQ_DATA_TMO | \
                          MV_SDIO_EIRQ_DATA_CRC16 | \
                          MV_SDIO_EIRQ_DATA_ENDBIT | \
                          MV_SDIO_EIRQ_AUTOCMD12 | \
                          MV_SDIO_EIRQ_CMD_STARTBIT |\
                          MV_SDIO_EIRQ_XFER_SIZE |\
                          MV_SDIO_EIRQ_RSP_TBIT |\
                          MV_SDIO_EIRQ_CRC_ENDBIT |\
                          MV_SDIO_EIRQ_CRC_STARTBIT |\
                          MV_SDIO_EIRQ_CRC_STAT)

/* AUTOCMD12 register values */
#define MV_SDIO_AUTOCMD12_BUSY_CHECK (1<<0)
#define MV_SDIO_AUTOCMD12_INDEX_CHECK (1<<1)
#define MV_SDIO_AUTOCMD12_INDEX(x) (x<<8)

/* Software reset register */
#define MV_SDIO_SW_RESET_ALL (1<<8)

/* */
#define MV_SDIO_SIG_CD 1
#define MV_SDIO_SIG_WP 2

#endif /* _MVSDMMC_INCLUDE */

