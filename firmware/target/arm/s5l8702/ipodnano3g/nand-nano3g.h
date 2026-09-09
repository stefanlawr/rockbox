/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2026 Rockbox contributors
 *
 * Low-level S5L8702 flash memory controller (FMC) driver for iPod nano 3G.
 * READ ONLY. Register sequences are transcribed from the S5L8702 boot ROM's
 * own NAND page reader (the routine used to boot an IMG1 from NAND).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#ifndef __NAND_NANO3G_H__
#define __NAND_NANO3G_H__

#include <stdint.h>
#include <stdbool.h>

#define NAND3G_PAGE_SIZE   2048
#define NAND3G_SPARE_WORDS 3
#define NAND3G_NUM_CTRL    2   /* FMC controllers, 0x400 apart */
#define NAND3G_NUM_CE      4   /* chip enables per controller probed */

/* Error codes (negative) returned by the functions below */
#define NAND3G_ETIMEOUT_RBB    -1  /* ready/busy never signalled */
#define NAND3G_ETIMEOUT_CMD    -2  /* command phase never completed */
#define NAND3G_ETIMEOUT_ADDR   -3  /* address phase never completed */
#define NAND3G_ETIMEOUT_DATA   -4  /* data transfer never completed */
#define NAND3G_ETIMEOUT_STATUS -5  /* status poll never completed */
#define NAND3G_ETIMEOUT_SPARE  -6  /* spare read never completed */
#define NAND3G_ETIMEOUT_ECC    -7  /* ECC engine never completed */
#define NAND3G_EBADARG         -8

/* Positive page-read results (from the boot ROM's own mapping) */
#define NAND3G_ECC_OK           0
#define NAND3G_ECC_CORRECTED    1   /* ROM code 1: error flagged, corrected */
#define NAND3G_ECC_UNCORRECTABLE 3  /* ROM code 3 */
#define NAND3G_ECC_OTHER        6   /* ROM code 6 */

/* Enable clocks and configure the NAND pin mux for controller 'ctrl'.
   Safe to call repeatedly. */
void nand3g_hw_init(int ctrl);

/* Reset the chip on (ctrl, ce) with CMD 0xFF and poll its status.
   Returns 0 or a negative error. */
int nand3g_reset(int ctrl, int ce);

/* READ ID (0x90). Fills id[0..7] (only the first bytes are meaningful).
   Returns 0 or a negative error. */
int nand3g_read_id(int ctrl, int ce, uint8_t id[8]);

/* Read one 2048-byte page. 'data' must be 32-byte aligned and is written
   by the controller's DMA; pass an uncached alias or discard the cache
   afterwards. 'spare' receives the 3 metadata words the ROM returns.
   Returns NAND3G_ECC_* (>= 0) or a negative error. 'raw_stat' (may be
   NULL) receives the OR of the per-chunk ECC status for diagnostics. */
int nand3g_read_page(int ctrl, int ce, uint32_t page,
                     void *data, uint32_t spare[NAND3G_SPARE_WORDS],
                     uint32_t *raw_stat);

/* Diagnostics from the last READ STATUS poll (see nand-nano3g.c). */
extern uint32_t nand3g_dbg_stat;
extern uint32_t nand3g_dbg_fifo;
extern uint32_t nand3g_dbg_status_timeouts;

/* Convenience: driver-owned 2048-byte page buffer (uncached alias). */
uint8_t *nand3g_page_buffer(void);

#endif /* __NAND_NANO3G_H__ */
