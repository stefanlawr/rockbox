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
 * iPod nano 3G NAND driver (S5L8702 FMC). READ ONLY so far.
 *
 * The register sequences below are a transcription of the S5L8702 boot
 * ROM's NAND routines (rom_nand_bank_reset @ 0x200096e4,
 * rom_nand_read_page @ 0x20009910 and helpers). The ROM uses them to boot
 * an IMG1 from block 0 of the NAND, so they are known-good for this SoC.
 * Every wait has a timeout so a hardware hang degrades to an error code.
 *
 * Nothing in this file erases or programs flash.
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

#include "config.h"
#include "system.h"
#include "s5l87xx.h"
#include "clocking-s5l8702.h"
#include "mv.h"
#include "storage.h"
#include "nand-nano3g.h"

/*
 * S5L8702 flash memory controller. Two controllers 0x400 apart share one
 * ECC engine at +0x800. Offsets and bit values come from the boot ROM.
 */
#define FMC3G_BASE          0x38A00000
#define FMC3G_CTRL_STRIDE   0x400
#define FMC3G_REG(c, off)   (*(REG32_PTR_T)(FMC3G_BASE + (c) * FMC3G_CTRL_STRIDE + (off)))

#define FMC3G_CTRL0(c)      FMC3G_REG(c, 0x00)  /* chip select / mode */
#define FMC3G_CTRL1(c)      FMC3G_REG(c, 0x04)  /* transfer trigger */
#define FMC3G_CMD(c)        FMC3G_REG(c, 0x08)
#define FMC3G_ADDR0(c)      FMC3G_REG(c, 0x0C)
#define FMC3G_ADDR1(c)      FMC3G_REG(c, 0x10)
#define FMC3G_ADDR2(c)      FMC3G_REG(c, 0x14)  /* ROM: chunk/transfer select */
#define FMC3G_UNK24(c)      FMC3G_REG(c, 0x24)  /* ROM: spare column select */
#define FMC3G_ANUM(c)       FMC3G_REG(c, 0x2C)  /* address cycle count */
#define FMC3G_DNUM(c)       FMC3G_REG(c, 0x30)  /* data byte count - 1 */
#define FMC3G_DMADST(c)     FMC3G_REG(c, 0x34)  /* ROM: data destination */
#define FMC3G_UNK38(c)      FMC3G_REG(c, 0x38)  /* ROM writes 7 before DMA */
#define FMC3G_STAT(c)       FMC3G_REG(c, 0x48)  /* write-1-to-clear */
#define FMC3G_SPARE0(c)     FMC3G_REG(c, 0x60)
#define FMC3G_SPARE1(c)     FMC3G_REG(c, 0x64)
#define FMC3G_SPARE2(c)     FMC3G_REG(c, 0x68)
#define FMC3G_SPARECTL(c)   FMC3G_REG(c, 0x78)
#define FMC3G_SPARETRIG(c)  FMC3G_REG(c, 0x7C)
#define FMC3G_FIFO(c)       FMC3G_REG(c, 0x80)

/* STAT bits */
#define STAT_RBB_DONE       0x00000002
#define STAT_CMD_DONE       0x00000004
#define STAT_ADDR_DONE      0x00000008
#define STAT_DATA_DONE      0x00100000
#define STAT_STATUS_DONE    0x00800000
#define STAT_ECC_ERROR      0x08000000

/* CTRL0 values */
#define CTRL0_BASE          0x00043801
#define CTRL0_CE(ce)        (1u << ((ce) + 1))
#define CTRL0_DESELECT      0x00043802
#define CTRL0_DMA_ON        0x01000001
#define CTRL0_DMA_MASK      0x00000400

/* CTRL1 values used by the ROM */
#define CTRL1_SETUP         0x0001F0E0
#define CTRL1_IDLE          0x000000E0
#define CTRL1_STATUS_SETUP  0x001000E0
#define CTRL1_XFER_ADDR     0x00000001
#define CTRL1_READ_STATUS   0x0000002A
#define CTRL1_READ_CHUNK    0x00000022
#define CTRL1_READ_CHUNK_HI 0x00000032
#define CTRL1_READ_ID       0x000001C2
#define CTRL1_FLUSH         0x00000340
#define CTRL1_DMA_START     0x000001A0

/* Shared ECC engine */
#define ECC3G_REG(off)      (*(REG32_PTR_T)(FMC3G_BASE + 0x800 + (off)))
#define ECC3G_START         ECC3G_REG(0x0C)
#define ECC3G_ERRSTAT       ECC3G_REG(0x10)
#define ECC3G_CONFIG        ECC3G_REG(0x14)
#define ECC3G_IRQSTAT       ECC3G_REG(0x40)
#define ECC3G_CONFIG_VAL    0x01000180
#define ECC3G_IRQ_ALL       0x7F
#define ECC3G_IRQ_DONE      0x04

/* Timeouts (microseconds). The ROM busy-loops 100000 iterations; be generous. */
#define NAND3G_TIMEOUT_US   200000

static uint8_t nand3g_buf[NAND3G_PAGE_SIZE] STORAGE_ALIGN_ATTR;
static bool nand3g_hw_ready[NAND3G_NUM_CTRL];

/* Wait until all bits in 'mask' are set in *reg. 0 on success, -1 on timeout. */
static int wait_set(volatile uint32_t *reg, uint32_t mask)
{
    unsigned stop = USEC_TIMER + NAND3G_TIMEOUT_US;
    while ((*reg & mask) != mask)
        if (TIME_AFTER(USEC_TIMER, stop))
            return -1;
    return 0;
}

/* Wait until all bits in 'mask' are clear in *reg. */
static int wait_clear(volatile uint32_t *reg, uint32_t mask)
{
    unsigned stop = USEC_TIMER + NAND3G_TIMEOUT_US;
    while (*reg & mask)
        if (TIME_AFTER(USEC_TIMER, stop))
            return -1;
    return 0;
}

/* Wait for a STAT bit and acknowledge it (write-1-to-clear). */
static int wait_stat(int c, uint32_t bit)
{
    int rc = wait_set(&FMC3G_STAT(c), bit);
    FMC3G_STAT(c) = bit;
    return rc;
}

uint8_t *nand3g_page_buffer(void)
{
    uint8_t *p = nand3g_buf;
    return S5L8702_UNCACHED_ADDR(p);
}

void nand3g_hw_init(int ctrl)
{
    /* Boot ROM: clock gates 8 (NAND) and 12 (NAND ECC). */
    clockgate_enable(CLOCKGATE_NAND, true);
    clockgate_enable(CLOCKGATE_NANDECC, true);

    /* Pin mux, transcribed from rom_nand_gpio_config(). Raw offsets from
       GPIO_BASE are used on purpose to match the ROM exactly. */
    volatile uint32_t *gpio = (volatile uint32_t *)GPIO_BASE;
    if (ctrl == 0) {
        gpio[0x140 / 4] = (gpio[0x140 / 4] & 0xFFFF0000) | 0x00002222;
        gpio[0x120 / 4] = (gpio[0x120 / 4] & 0xFFF0FFF0) | 0x00020002;
        gpio[0x100 / 4] = 0x22222222;
    } else {
        gpio[0x180 / 4] = (gpio[0x180 / 4] & 0xFFFF0000) | 0x00002222;
        gpio[0x0E0 / 4] = (gpio[0x0E0 / 4] & 0xFFF0FFFF) | 0x00020000;
        gpio[0x140 / 4] = (gpio[0x140 / 4] & 0x0000FFFF) | 0x22220000;
        gpio[0x160 / 4] = (gpio[0x160 / 4] & 0xFFF00000) | 0x00022222;
    }
    nand3g_hw_ready[ctrl] = true;
}

/* Address cycles programmed into ANUM for a page read. The ROM uses 4 and
   only ever reads block 0; a 16 Gbit die needs 2 column + 3 row bytes.
   Exposed so the dev menu can test whether high pages alias. */
uint32_t nand3g_anum = 4;

/* Diagnostics: STAT and FIFO seen during the last READ STATUS poll. */
uint32_t nand3g_dbg_stat;
uint32_t nand3g_dbg_fifo;
uint32_t nand3g_dbg_status_timeouts;

/* ROM FUN_20009614: issue READ STATUS (0x70) and wait for the ready bit.
   The ROM's own wait helper gives up silently after ~100000 loops and the
   ROM ignores its result, so on hardware the STATUS_DONE bit may never be
   raised. Mirror that: the second wait is short and non-fatal. */
static int nand3g_poll_status(int c)
{
    FMC3G_CTRL1(c) = CTRL1_STATUS_SETUP;
    FMC3G_ADDR2(c) = 1;
    FMC3G_CMD(c)   = 0x70;
    if (wait_stat(c, STAT_RBB_DONE))
        return NAND3G_ETIMEOUT_CMD;
    FMC3G_CTRL1(c) = CTRL1_READ_STATUS;
    unsigned stop = USEC_TIMER + 5000;
    while ((FMC3G_STAT(c) & STAT_STATUS_DONE) == 0) {
        if (TIME_AFTER(USEC_TIMER, stop)) {
            nand3g_dbg_status_timeouts++;
            break;
        }
    }
    nand3g_dbg_stat = FMC3G_STAT(c);
    nand3g_dbg_fifo = FMC3G_FIFO(c);
    FMC3G_STAT(c)  = STAT_STATUS_DONE;
    FMC3G_ADDR2(c) = 0;
    FMC3G_CTRL1(c) = CTRL1_IDLE;
    return 0;
}

int nand3g_reset(int c, int ce)
{
    if (c < 0 || c >= NAND3G_NUM_CTRL || ce < 0 || ce >= NAND3G_NUM_CE)
        return NAND3G_EBADARG;
    if (!nand3g_hw_ready[c])
        nand3g_hw_init(c);

    /* rom_nand_bank_reset(): select CE, CMD 0xFF, wait R/B, poll status */
    FMC3G_CTRL0(c) = CTRL0_CE(ce) | CTRL0_BASE;
    FMC3G_CMD(c)   = 0xFF;
    if (wait_stat(c, STAT_RBB_DONE))
        return NAND3G_ETIMEOUT_RBB;
    return nand3g_poll_status(c);
}

int nand3g_read_id(int c, int ce, uint8_t id[8])
{
    if (c < 0 || c >= NAND3G_NUM_CTRL || ce < 0 || ce >= NAND3G_NUM_CE)
        return NAND3G_EBADARG;
    if (!nand3g_hw_ready[c])
        nand3g_hw_init(c);

    FMC3G_CTRL0(c) = CTRL0_CE(ce) | CTRL0_BASE;

    /* Sequence used by wInd3x's identify payload on this controller. */
    FMC3G_CMD(c) = 0x90;
    if (wait_stat(c, STAT_RBB_DONE))
        return NAND3G_ETIMEOUT_RBB;

    FMC3G_ANUM(c)  = 0;
    FMC3G_ADDR0(c) = 0;
    FMC3G_CTRL1(c) = CTRL1_XFER_ADDR;
    if (wait_stat(c, STAT_CMD_DONE))
        return NAND3G_ETIMEOUT_ADDR;

    FMC3G_DNUM(c)  = 7;      /* 8 bytes */
    FMC3G_ADDR2(c) = 1;
    FMC3G_CTRL1(c) = CTRL1_READ_ID;
    if (wait_stat(c, STAT_ADDR_DONE))
        return NAND3G_ETIMEOUT_DATA;

    FMC3G_ADDR2(c) = 0x100;
    FMC3G_CTRL1(c) = CTRL1_FLUSH;

    uint32_t w0 = FMC3G_FIFO(c);
    uint32_t w1 = FMC3G_FIFO(c);
    for (int i = 0; i < 4; i++) {
        id[i]     = (w0 >> (8 * i)) & 0xFF;
        id[4 + i] = (w1 >> (8 * i)) & 0xFF;
    }
    FMC3G_CTRL1(c) = CTRL1_IDLE;
    return 0;
}

/* ROM FUN_2000ac1c: run the ECC engine for one chunk. */
static int nand3g_ecc_correct(uint32_t lo, uint32_t hi, uint32_t par)
{
    ECC3G_IRQSTAT = ECC3G_IRQ_ALL;
    ECC3G_CONFIG  = ECC3G_CONFIG_VAL;
    ECC3G_START   = (1u << (hi + 8)) | (par << 16) | (1u << (lo + 8)) | 1;
    int rc = wait_set(&ECC3G_IRQSTAT, ECC3G_IRQ_DONE);
    ECC3G_IRQSTAT = ECC3G_IRQ_DONE;
    return rc ? NAND3G_ETIMEOUT_ECC : 0;
}

int nand3g_read_page(int c, int ce, uint32_t page,
                     void *data, uint32_t spare[NAND3G_SPARE_WORDS],
                     uint32_t *raw_stat)
{
    if (c < 0 || c >= NAND3G_NUM_CTRL || ce < 0 || ce >= NAND3G_NUM_CE)
        return NAND3G_EBADARG;
    if (!nand3g_hw_ready[c])
        nand3g_hw_init(c);

    uint32_t ecc_flag = 0, errstat = 0, irqstat = 0;
    uintptr_t dst = (uintptr_t)data;
    int rc;

    /* --- command + address phase --- */
    FMC3G_CTRL1(c) = CTRL1_SETUP;
    FMC3G_CTRL0(c) = CTRL0_CE(ce) | CTRL0_BASE;
    FMC3G_CMD(c)   = 0x00;
    if (wait_stat(c, STAT_RBB_DONE))
        return NAND3G_ETIMEOUT_RBB;

    FMC3G_ANUM(c)  = nand3g_anum;
    FMC3G_ADDR0(c) = page << 16;    /* column 0, page low half */
    FMC3G_ADDR1(c) = page >> 16;
    FMC3G_CTRL1(c) = CTRL1_XFER_ADDR;
    if (wait_stat(c, STAT_CMD_DONE))
        return NAND3G_ETIMEOUT_ADDR;

    FMC3G_CMD(c) = 0x30;
    if (wait_stat(c, STAT_RBB_DONE))
        return NAND3G_ETIMEOUT_CMD;

    rc = nand3g_poll_status(c);
    if (rc)
        return rc;

    FMC3G_CMD(c) = 0x00;
    if (wait_stat(c, STAT_RBB_DONE))
        return NAND3G_ETIMEOUT_RBB;

    ECC3G_IRQSTAT = ECC3G_IRQ_ALL;

    /* --- four 512-byte chunks, each: 16 bytes spare/parity, 512 data --- */
    for (uint32_t chunk = 0; chunk < 4; chunk++) {
        uint32_t lo = chunk & 1;
        uint32_t hi = lo + 4;
        uint32_t col = (chunk < 2) ? 0 : 0x10;

        FMC3G_STAT(c)  = STAT_ECC_ERROR;

        /* parity/spare part */
        FMC3G_DNUM(c)  = 0x0F;
        FMC3G_ADDR2(c) = 1u << hi;
        FMC3G_UNK24(c) = col;
        FMC3G_CTRL1(c) = (hi < 4) ? CTRL1_READ_CHUNK : CTRL1_READ_CHUNK_HI;
        if (wait_stat(c, STAT_ADDR_DONE))
            return NAND3G_ETIMEOUT_DATA;

        /* data part */
        FMC3G_DNUM(c)  = 0x1FF;
        FMC3G_ADDR2(c) = 1u << lo;
        FMC3G_UNK24(c) = 0;
        FMC3G_CTRL1(c) = (lo < 4) ? CTRL1_READ_CHUNK : CTRL1_READ_CHUNK_HI;
        if (wait_stat(c, STAT_ADDR_DONE))
            return NAND3G_ETIMEOUT_DATA;

        uint32_t st = FMC3G_STAT(c);
        if (st & STAT_ECC_ERROR) {
            rc = nand3g_ecc_correct(lo, hi, (chunk >> 1) & 1);
            if (rc)
                return rc;
            errstat  |= ECC3G_ERRSTAT;
            ecc_flag |= st & STAT_ECC_ERROR;
            irqstat  |= ECC3G_IRQSTAT;
        }

        /* move chunk from FIFO to memory */
        FMC3G_DMADST(c) = dst + chunk * 0x200;
        FMC3G_UNK38(c)  = 7;
        FMC3G_CTRL0(c)  = (FMC3G_CTRL0(c) & ~CTRL0_DMA_MASK) | CTRL0_DMA_ON;
        FMC3G_ADDR2(c)  = 1u << (lo + 8);
        FMC3G_CTRL1(c)  = CTRL1_DMA_START;
        if (wait_stat(c, STAT_DATA_DONE))
            return NAND3G_ETIMEOUT_DATA;
    }

    /* --- spare/metadata words --- */
    FMC3G_SPARECTL(c)  = 0x5140;
    FMC3G_SPARETRIG(c) = 2;
    if (wait_clear(&FMC3G_SPARETRIG(c), 2))
        return NAND3G_ETIMEOUT_SPARE;
    if (spare) {
        spare[0] = FMC3G_SPARE0(c);
        spare[1] = FMC3G_SPARE1(c);
        spare[2] = FMC3G_SPARE2(c);
    }

    FMC3G_CTRL0(c) = CTRL0_DESELECT;

    int result = NAND3G_ECC_OK;
    if (ecc_flag) {
        if (errstat & 1)
            result = NAND3G_ECC_UNCORRECTABLE;
        else if (irqstat & 0x20)
            result = NAND3G_ECC_OTHER;
        else
            result = NAND3G_ECC_CORRECTED;
    }
    if (raw_stat)
        *raw_stat = (ecc_flag ? 0x80000000 : 0) | (errstat & 0xFFFF) | (irqstat << 16);

    FMC3G_STAT(c) = 0xFFFFFFFF;
    ECC3G_IRQSTAT = ECC3G_IRQ_ALL;
    return result;
}

/* ------------------------------------------------------------------------
 * nano 2G style NAND API (nand-target.h), consumed by ftl-nano3g.c, which
 * is the nano 2G Whimory FTL built read-only. Return value convention of
 * nand_read_page(): bit 0 = transfer error, bit 1 = page is empty (only
 * when checkempty), bits 4..7 = data ECC status (bit 4 = uncorrectable),
 * bits 8..11 = spare ECC status. The FTL treats (rc & 0x11F) != 0 as bad.
 * ---------------------------------------------------------------------- */

#include "kernel.h"
#include "nand-target.h"
#include "ftl-target.h"
#include <string.h>

/* Device table. Only the Intel/Micron parts seen so far; the geometry of
   0xA5D5D589 is the Intel twin of Micron 0xA5D5D52C in the nano 2G table.
   Timing fields are unused by this controller (the ROM's fixed values are
   used instead) and kept for structure compatibility. */
static const struct nand_device_info_type nand_deviceinfotable[] =
{
    {0xA5D5D589, 8192, 7744, 0x80, 7, 3, 2, 2, 1},
    {0xA5D5D52C, 8192, 7744, 0x80, 7, 3, 2, 2, 1},
};

static int nand_type[4] = { -1, -1, -1, -1 };
static long nand_last_activity_value = -1;
static struct mutex nand_mtx;
static bool nand_initialized;

/* Statistics for the dev menu / debugging */
uint32_t nand3g_stat_reads, nand3g_stat_ecc_flagged, nand3g_stat_ecc_bad,
         nand3g_stat_errors;

/* The FTL's banks are the chip-enables on controller 0 */
#define NAND3G_CTRL 0

static uint32_t nand_check_empty(const uint8_t* spare)
{
    uint32_t i, count = 0;
    for (i = 0; i < 0x40; i++) if (spare[i] != 0xFF) count++;
    return (count < 2) ? 1 : 0;
}

uint32_t nand_read_page(uint32_t bank, uint32_t page, void* databuffer,
                        void* sparebuffer, uint32_t doecc,
                        uint32_t checkempty)
{
    (void)doecc;    /* the controller always runs its ECC engine */
    uint32_t spare3[NAND3G_SPARE_WORDS];
    uint32_t raw, rc = 0;
    uint8_t *buf = nand3g_page_buffer();

    if (bank >= 4 || nand_type[bank] < 0)
        return 1;

    mutex_lock(&nand_mtx);
    nand_last_activity_value = current_tick;
    nand3g_stat_reads++;

    int r = nand3g_read_page(NAND3G_CTRL, bank, page, buf, spare3, &raw);
    if (r < 0) {
        nand3g_stat_errors++;
        mutex_unlock(&nand_mtx);
        return 1;
    }
    if (r != NAND3G_ECC_OK) {
        nand3g_stat_ecc_flagged++;
        if (r != NAND3G_ECC_CORRECTED) {
            nand3g_stat_ecc_bad++;
            rc |= 0x10;
        }
    }

    if (databuffer)
        memcpy(databuffer, buf, NAND3G_PAGE_SIZE);

    /* Build the 64-byte spare the FTL expects: the three metadata words
       from the controller are bytes 0..11 (lpn/usn, usn/idx, field_8,
       type, eccmark, field_B); the rest is ECC parity the controller
       already consumed, reported as 0xFF. An erased page reads 0xFF too. */
    uint8_t spare[0x40];
    memset(spare, 0xFF, sizeof(spare));
    memcpy(spare, spare3, 12);
    if (sparebuffer)
        memcpy(sparebuffer, spare, 0x40);
    if (checkempty)
        rc |= nand_check_empty(spare) << 1;

    mutex_unlock(&nand_mtx);
    return rc;
}

uint32_t nand_read_page_fast(uint32_t page, void* databuffer,
                             void* sparebuffer, uint32_t doecc,
                             uint32_t checkempty)
{
    uint32_t i, rc = 0;
    for (i = 0; i < 4; i++)
    {
        if (nand_type[i] < 0) continue;
        void* databuf = databuffer ? (void*)((uintptr_t)databuffer + 0x800 * i) : NULL;
        void* sparebuf = sparebuffer ? (void*)((uintptr_t)sparebuffer + 0x40 * i) : NULL;
        uint32_t ret = nand_read_page(i, page, databuf, sparebuf, doecc, checkempty);
        if (ret & 1) rc |= 1 << (i << 2);
        if (ret & 2) rc |= 2 << (i << 2);
        if (ret & 0x10) rc |= 4 << (i << 2);
        if (ret & 0x100) rc |= 8 << (i << 2);
    }
    return rc;
}

/* Read-only driver: every write/erase entry point fails loudly. */
uint32_t nand_write_page(uint32_t bank, uint32_t page, void* databuffer,
                         void* sparebuffer, uint32_t doecc)
{
    (void)bank; (void)page; (void)databuffer; (void)sparebuffer; (void)doecc;
    return 1;
}

uint32_t nand_write_page_start(uint32_t bank, uint32_t page, void* databuffer,
                               void* sparebuffer, uint32_t doecc)
{
    (void)bank; (void)page; (void)databuffer; (void)sparebuffer; (void)doecc;
    return 1;
}

uint32_t nand_write_page_collect(uint32_t bank)
{
    (void)bank;
    return 1;
}

uint32_t nand_block_erase(uint32_t bank, uint32_t page)
{
    (void)bank; (void)page;
    return 1;
}

uint32_t nand_reset(uint32_t bank)
{
    if (bank >= 4) return 1;
    return nand3g_reset(NAND3G_CTRL, bank) ? 1 : 0;
}

const struct nand_device_info_type* nand_get_device_type(uint32_t bank)
{
    if (bank >= 4 || nand_type[bank] < 0)
        return NULL;
    return &nand_deviceinfotable[nand_type[bank]];
}

void nand_set_active(void)
{
    nand_last_activity_value = current_tick;
}

long nand_last_activity(void)
{
    return nand_last_activity_value;
}

void nand_power_up(void)
{
    /* The NAND supply is PMU reg 0x10 bit 2, left on by pmu_preinit(). */
}

void nand_power_down(void)
{
}

int nand_device_init(void)
{
    if (nand_initialized) return 0;
    mutex_init(&nand_mtx);
    nand3g_hw_init(NAND3G_CTRL);

    for (uint32_t ce = 0; ce < 4; ce++)
    {
        uint8_t id[8];
        nand_type[ce] = -1;
        if (nand3g_reset(NAND3G_CTRL, ce)) continue;
        if (nand3g_read_id(NAND3G_CTRL, ce, id)) continue;
        uint32_t packed = id[0] | (id[1] << 8) | (id[2] << 16) | ((uint32_t)id[3] << 24);
        for (unsigned j = 0; j < ARRAYLEN(nand_deviceinfotable); j++)
            if (nand_deviceinfotable[j].id == packed) { nand_type[ce] = j; break; }
    }
    nand_last_activity_value = current_tick;
    nand_initialized = true;
    return (nand_type[0] < 0) ? -1 : 0;
}

/* ------------------------------------------------------------------------
 * Rockbox storage API, mirroring firmware/target/arm/s5l8700/ata-nand-s5l8700.c
 * ---------------------------------------------------------------------- */

int nand_init(void)
{
    if (ftl_init()) return 1;
    return 0;
}

void nand_spindown(int seconds)
{
    (void)seconds;
}

void nand_sleepnow(void)
{
    nand_power_down();
}

void nand_spin(void)
{
    nand_set_active();
}

void nand_enable(bool on)
{
    (void)on;
}

#ifdef HAVE_STORAGE_FLUSH
int nand_flush(void)
{
    return ftl_sync();
}
#endif

int nand_read_sectors(IF_MD(int drive,) sector_t start, int incount,
                     void* inbuf)
{
#ifdef HAVE_MULTIDRIVE
    (void) drive;
#endif
    return ftl_read(start, incount, inbuf);
}

/* Read-only phase: writes are accepted and discarded. Returning an error
   here makes Rockbox's disk cache panic on its first writeback at boot
   (dc_writeback_callback), which is not useful while there is no FTL
   write support. Nothing reaches the flash; changes simply do not
   persist across a reboot. The count is shown in the dev bootloader. */
uint32_t nand3g_stat_dropped_writes;

int nand_write_sectors(IF_MD(int drive,) sector_t start, int count,
                      const void* outbuf)
{
#ifdef HAVE_MULTIDRIVE
    (void) drive;
#endif
    (void) start;
    (void) outbuf;
    nand3g_stat_dropped_writes += count;
    return 0;
}

int nand_event(long id, intptr_t data)
{
    (void) id;
    (void) data;
    return 0;
}

#ifdef STORAGE_GET_INFO
void nand_get_info(IF_MD(int drive,) struct storage_info *info)
{
    IF_MD((void)drive);
    /* ftl_nand_type is the FTL's virtual geometry: 8-way interleaved
       superblocks of pagesperblock * 8 pages (see ftl-nano3g.c). */
    uint32_t ppb = 8 * ftl_nand_type->pagesperblock;
    info->sector_size = SECTOR_SIZE;
    info->num_sectors = ftl_nand_type->userblocks * ppb;
    info->vendor = "Apple";
    info->product = "iPod nano 3G";
    info->revision = "1.0";
}
#endif

long nand_last_disk_activity(void)
{
    return nand_last_activity();
}

#ifdef CONFIG_STORAGE_MULTI
int nand_num_drives(int first_drive)
{
    (void)first_drive;
    return 1;
}
#endif
