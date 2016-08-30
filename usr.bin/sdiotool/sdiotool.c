/*
 * Copyright (c) 2016 Ilya Bakulin
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
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/ioctl.h>
#include <sys/stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/endian.h>
#include <sys/sbuf.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <limits.h>
#include <fcntl.h>
#include <ctype.h>
#include <err.h>
#include <libutil.h>

#include <cam/cam.h>
#include <cam/cam_debug.h>
#include <cam/cam_ccb.h>
#include <cam/mmc/mmc_all.h>
#include <camlib.h>

static int sdio_rw_direct(struct cam_device *dev,
			  uint8_t func_number,
			  uint32_t addr,
			  uint8_t is_write,
			  uint8_t *data,
			  uint8_t *resp);

static int sdio_is_func_ready(struct cam_device *dev, uint8_t func_number, uint8_t *is_enab);
static int sdio_is_func_enabled(struct cam_device *dev, uint8_t func_number, uint8_t *is_enab);
static int sdio_func_enable(struct cam_device *dev, uint8_t func_number, int enable);

static int sdio_is_func_intr_enabled(struct cam_device *dev, uint8_t func_number, uint8_t *is_enab);
static int sdio_func_intr_enable(struct cam_device *dev, uint8_t func_number, int enable);


/* Use CMD52 to read or write a single byte */
int
sdio_rw_direct(struct cam_device *dev,
	       uint8_t func_number,
	       uint32_t addr,
	       uint8_t is_write,
	       uint8_t *data, uint8_t *resp) {
	union ccb *ccb;
	uint32_t flags;
	uint32_t arg;
	int retval = 0;

	ccb = cam_getccb(dev);
	if (ccb == NULL) {
		warnx("%s: error allocating CCB", __func__);
		return (1);
	}
	bzero(&(&ccb->ccb_h)[1],
	      sizeof(union ccb) - sizeof(struct ccb_hdr));

	flags = MMC_RSP_R5 | MMC_CMD_AC;
	arg = SD_IO_RW_FUNC(func_number) | SD_IO_RW_ADR(addr);
	if (is_write)
		arg |= SD_IO_RW_WR | SD_IO_RW_RAW | SD_IO_RW_DAT(*data);

	cam_fill_mmcio(&ccb->mmcio,
		       /*retries*/ 0,
		       /*cbfcnp*/ NULL,
		       /*flags*/ CAM_DIR_NONE,
		       /*mmc_opcode*/ SD_IO_RW_DIRECT,
		       /*mmc_arg*/ arg,
		       /*mmc_flags*/ flags,
		       /*mmc_data*/ 0,
		       /*timeout*/ 5000);

	if (((retval = cam_send_ccb(dev, ccb)) < 0)
	    || ((ccb->ccb_h.status & CAM_STATUS_MASK) != CAM_REQ_CMP)) {
		const char warnstr[] = "error sending command";

		if (retval < 0)
			warn(warnstr);
		else
			warnx(warnstr);
		return (-1);
	}

	*resp = ccb->mmcio.cmd.resp[0] & 0xFF;
	cam_freeccb(ccb);
	return (retval);
}

static int sdio_read_bool_for_func(struct cam_device *dev, uint32_t addr, uint8_t func_number, uint8_t *is_enab) {
	uint8_t resp;
	int ret;

	ret = sdio_rw_direct(dev, 0, addr, 0, NULL, &resp);
	if (ret < 0)
		return ret;

	*is_enab = (resp & (1 << func_number)) > 0 ? 1 : 0;

	return (0);
}

static int sdio_set_bool_for_func(struct cam_device *dev, uint32_t addr, uint8_t func_number, int enable) {
	uint8_t resp;
	int ret;
	uint8_t is_enabled;

	ret = sdio_rw_direct(dev, 0, addr, 0, NULL, &resp);
	if (ret != 0)
		return ret;

	is_enabled = resp & (1 << func_number);
	if ((is_enabled !=0 && enable == 1) || (is_enabled == 0 && enable == 0))
		return 0;

	if (enable)
		resp |= 1 << func_number;
	else
		resp &= ~ (1 << func_number);

	ret = sdio_rw_direct(dev, 0, addr, 1, &resp, &resp);

	return ret;
}

static int sdio_is_func_ready(struct cam_device *dev, uint8_t func_number, uint8_t *is_enab) {
	return sdio_read_bool_for_func(dev, SD_IO_CCCR_FN_READY, func_number, is_enab);
}

static int sdio_is_func_enabled(struct cam_device *dev, uint8_t func_number, uint8_t *is_enab) {
	return sdio_read_bool_for_func(dev, SD_IO_CCCR_FN_ENABLE, func_number, is_enab);
}

static int sdio_func_enable(struct cam_device *dev, uint8_t func_number, int enable) {
	return sdio_set_bool_for_func(dev, SD_IO_CCCR_FN_ENABLE, func_number, enable);
}

static int sdio_is_func_intr_enabled(struct cam_device *dev, uint8_t func_number, uint8_t *is_enab) {
	return sdio_read_bool_for_func(dev, SD_IO_CCCR_INT_ENABLE, func_number, is_enab);
}

static int sdio_func_intr_enable(struct cam_device *dev, uint8_t func_number, int enable) {
	return sdio_set_bool_for_func(dev, SD_IO_CCCR_INT_ENABLE, func_number, enable);
}

int
main(__unused int argc, __unused char **argv) {
	char device[] = "pass";
	int unit = 0;
	uint8_t resp;
	uint8_t is_enab;
	struct cam_device *cam_dev = NULL;

	if ((cam_dev = cam_open_spec_device(device, unit, O_RDWR, NULL)) == NULL)
		errx(1, "Cannot open device");

	/* Read Addr 7 of func 0 */
	int ret = sdio_rw_direct(cam_dev, 0, 7, 0, NULL, &resp);
	if (ret < 0)
		errx(1, "Error sending CAM command");
	printf("Result: %02x\n", resp);

	/* Check if func 1 is enabled */
	ret = sdio_is_func_enabled(cam_dev, 1, &is_enab);
	if (ret < 0)
		errx(1, "Cannot check if func is enabled");
	printf("F1 enabled: %d\n", is_enab);
	ret = sdio_func_enable(cam_dev, 1, 1 - is_enab);
	if (ret < 0)
		errx(1, "Cannot enable/disable func");
	printf("F1 en/dis result: %d\n", ret);

	/* Check if func 1 is ready */
	ret = sdio_is_func_ready(cam_dev, 1, &is_enab);
	if (ret < 0)
		errx(1, "Cannot check if func is ready");
	printf("F1 ready: %d\n", is_enab);

	/* Check if interrupts are enabled */
	ret = sdio_is_func_intr_enabled(cam_dev, 1, &is_enab);
	if (ret < 0)
		errx(1, "Cannot check if func intr is enabled");
	printf("F1 intr enabled: %d\n", is_enab);
	ret = sdio_func_intr_enable(cam_dev, 1, 1 - is_enab);
	if (ret < 0)
		errx(1, "Cannot enable/disable func intr");
	printf("F1 intr en/dis result: %d\n", ret);

	cam_close_spec_device(cam_dev);
}
