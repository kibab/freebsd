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

#ifndef _SDIO_SUBR_H_
#define _SDIO_SUBR_H_

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <cam/cam.h>
#include <cam/cam_debug.h>
#include <cam/cam_ccb.h>
#include <cam/mmc/mmc_all.h>

/*
 * This file contains functions to work with SDIO cards.
 * All non-static functions should be usable BOTH from
 * the kernel and from the userland.
*/

struct cis_info {
	uint16_t man_id;
	uint16_t prod_id;
	uint16_t max_block_size;
};

struct card_info {
	uint8_t num_funcs;
	struct cis_info f[8];
};

#ifdef _KERNEL
uint8_t sdio_read_1(union ccb *ccb, uint8_t func_number, uint32_t addr, int *ret);
uint32_t sdio_get_common_cis_addr(union ccb *ccb);
int sdio_func_read_cis(union ccb *ccb, uint8_t func_number,
		       uint32_t cis_addr, struct cis_info *info);
int get_sdio_card_info(union ccb *ccb, struct card_info *ci);
#else /* _KERNEL */

#endif /* _KERNEL */
#endif /* _SDIO_SUBR_H_ */
