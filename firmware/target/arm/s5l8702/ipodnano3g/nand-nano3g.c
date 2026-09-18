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

/* The page buffer is followed by a guard zone. On the MB245 the FTL
   context in memory was overwritten in a way that matches a DMA overrun
   past this buffer (ftl_cxt sat 0x78 bytes behind it, the NAND mutex
   0x14 bytes). The guard absorbs such an overrun and every NAND operation
   checks it (through the uncached alias, DMA bypasses the cache) so the
   offending operation and the overrun length are recorded. */
#ifdef NANO3G_FTL_DIAG
#define NAND3G_GUARD_SZ 8192
#else
#define NAND3G_GUARD_SZ 0
#endif
static uint8_t nand3g_buf[NAND3G_PAGE_SIZE + NAND3G_GUARD_SZ] STORAGE_ALIGN_ATTR;
uint32_t nand3g_guard[8]; /* violations, op(1r 2w 3e), page, ce, first off, last off, first word, rc */
#ifndef NANO3G_FTL_DIAG
static inline void nand3g_guard_fill(void) {}
static inline void nand3g_guard_check(uint32_t op, uint32_t page, uint32_t ce, int rc)
{
    (void)op; (void)page; (void)ce; (void)rc;
}
#else
static void nand3g_guard_fill(void)
{
    uint8_t *base = nand3g_buf; volatile uint8_t *g = S5L8702_UNCACHED_ADDR(base) + NAND3G_PAGE_SIZE;
    for (unsigned i = 0; i < NAND3G_GUARD_SZ; i++) g[i] = (uint8_t)(0xA5 ^ i);
}
static void nand3g_guard_check(uint32_t op, uint32_t page, uint32_t ce, int rc)
{
    uint8_t *base = nand3g_buf; volatile uint8_t *g = S5L8702_UNCACHED_ADDR(base) + NAND3G_PAGE_SIZE;
    unsigned first = NAND3G_GUARD_SZ, last = 0;
    for (unsigned i = 0; i < NAND3G_GUARD_SZ; i++)
        if (g[i] != (uint8_t)(0xA5 ^ i)) { if (first == NAND3G_GUARD_SZ) first = i; last = i; }
    if (first == NAND3G_GUARD_SZ) return;
    if (nand3g_guard[0] == 0)
    {
        nand3g_guard[1] = op; nand3g_guard[2] = page; nand3g_guard[3] = ce;
        nand3g_guard[4] = first; nand3g_guard[5] = last;
        nand3g_guard[6] = g[first & ~3] | (g[(first & ~3) + 1] << 8) | (g[(first & ~3) + 2] << 16) | (g[(first & ~3) + 3] << 24);
        nand3g_guard[7] = (uint32_t)rc;
    }
    nand3g_guard[0]++;
    nand3g_guard_fill();
}
#endif /* NANO3G_FTL_DIAG */
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


/* Write-side registers/values, transcribed from the OF's FMSS sequencer
   programs (osos 1.1.3: erase = program at 0x8c30, single-page program =
   0x9e98, i.e. "m1fmssEraseSingleBlock"/"m1fmssWriteScatteredPages"). */
#define FMC3G_WCOL(c)       FMC3G_REG(c, 0x28)  /* parity column: chunk * 0x10 */
#define FMC3G_STATB(c)      FMC3G_REG(c, 0x4C)  /* status byte after CTRL1 0xCA */
#define CTRL1_W_SETUP       0x0FF3F8E0
#define STAT_W_CLEARALL     0x0FF00FFE
#define CTRL0_DMA_MODE      0x01000000
#define CTRL1_W_DMAIN0      0x000002E0  /* DMA chunk memory -> buffer 0 */
#define CTRL1_W_PARITY      0x00000034  /* 16 parity bytes buffer -> NAND */
#define CTRL1_W_DATA_DMA    0x000002E4  /* 512 data bytes -> NAND + DMA next */
#define CTRL1_W_DATA_LAST   0x000000E4  /* 512 data bytes -> NAND, no DMA */
#define CTRL1_W_STATUS      0x000000CA
#define CTRL1_W_STATUS_END  0x00000020
#define SPARECTL_WRITE      0x00003210
#define ECC3G_ENC(chunk)    (((chunk) << 16) | 0x1000 | (0x100 << ((chunk) & 1)) | 2)

unsigned nand3g_dbg_wstat;
unsigned nand3g_dbg_wstep, nand3g_dbg_wfail_stat, nand3g_dbg_wfail_ecc, nand3g_dbg_wfail_sp;

/* Read the NAND status byte the OF way; returns byte or -1 on timeout. */
static int nand3g_status_byte(int c)
{
    FMC3G_ADDR2(c) = 1;
    FMC3G_CMD(c)   = 0x70;
    if (wait_stat(c, STAT_RBB_DONE))
        return -1;
    FMC3G_CTRL1(c) = CTRL1_W_STATUS;
    unsigned stop = USEC_TIMER + NAND3G_TIMEOUT_US;
    while (!(FMC3G_STATB(c) & 0x40))
        if (TIME_AFTER(USEC_TIMER, stop))
            return -1;
    int st = FMC3G_STATB(c) & 0xFF;
    FMC3G_STAT(c)  = STAT_STATUS_DONE;
    FMC3G_CTRL1(c) = CTRL1_W_STATUS_END;
    nand3g_dbg_wstat = st;
    return st;
}

static void nand3g_w_begin(int c, int ce)
{
    if (!nand3g_hw_ready[c])
        nand3g_hw_init(c);
    FMC3G_CTRL1(c) = CTRL1_W_SETUP;
    FMC3G_STAT(c)  = STAT_W_CLEARALL;
    FMC3G_CTRL0(c) = CTRL0_BASE | CTRL0_CE(ce);
}

/* Erase one physical block. 0 ok, 1 NAND fail bit, <0 timeout. */
int nand3g_erase_block(int c, int ce, uint32_t block)
{
    if (c < 0 || c >= NAND3G_NUM_CTRL || ce < 0 || ce >= NAND3G_NUM_CE)
        return NAND3G_EBADARG;
    nand3g_w_begin(c, ce);

    int st = nand3g_status_byte(c);
    if (st < 0) { FMC3G_CTRL0(c) = CTRL0_DESELECT; return NAND3G_ETIMEOUT_STATUS; }

    FMC3G_CMD(c) = 0x60;
    if (wait_stat(c, STAT_RBB_DONE)) goto tmo;
    FMC3G_ANUM(c)  = 2;                 /* three row cycles */
    FMC3G_ADDR0(c) = block * NAND3G_PAGES_PER_BLOCK;
    FMC3G_CTRL1(c) = CTRL1_XFER_ADDR;
    if (wait_stat(c, STAT_CMD_DONE)) goto tmo;
    FMC3G_CMD(c) = 0xD0;
    if (wait_stat(c, STAT_RBB_DONE)) goto tmo;

    st = nand3g_status_byte(c);
    FMC3G_CTRL0(c) = CTRL0_DESELECT;
    if (st < 0) return NAND3G_ETIMEOUT_STATUS;
    return st & 1;
tmo:
    FMC3G_CTRL0(c) = CTRL0_DESELECT;
    return NAND3G_ETIMEOUT_CMD;
}

static int nand3g_ecc_encode(uint32_t chunk)
{
    ECC3G_IRQSTAT = 0x1FF;
    ECC3G_CONFIG  = 0x180;
    ECC3G_START   = ECC3G_ENC(chunk);
    /* encode completion is bit 0 (the OF program sequence clears bit 0 after
       its ECC wait; decode uses bit 2). Observed IRQSTAT 0x33 while waiting. */
    if (wait_set(&ECC3G_IRQSTAT, 0x1))
        return NAND3G_ETIMEOUT_ECC;
    ECC3G_IRQSTAT = 1;
    return 0;
}

static void nand3g_dma_in(int c, const void *src, uint32_t chunk)
{
    FMC3G_DMADST(c) = (uintptr_t)src + chunk * 0x200;
    FMC3G_UNK38(c)  = 7;
}

/* Program one page (2048 data bytes + 3 spare words), hardware ECC.
   0 ok, 1 NAND reported fail, <0 timeout. */
int nand3g_write_page(int c, int ce, uint32_t page,
                      const void *data, const uint32_t spare[NAND3G_SPARE_WORDS])
{
    if (c < 0 || c >= NAND3G_NUM_CTRL || ce < 0 || ce >= NAND3G_NUM_CE)
        return NAND3G_EBADARG;
    commit_dcache_range(data, NAND3G_PAGE_SIZE);
    nand3g_w_begin(c, ce);
    FMC3G_CTRL0(c) |= CTRL0_DMA_MODE;

    /* chunk 0 into buffer 0 */
    nand3g_dma_in(c, data, 0);
    FMC3G_DNUM(c)  = 0x1FF;
    FMC3G_ADDR2(c) = 1;
    FMC3G_CTRL1(c) = CTRL1_W_DMAIN0;

    /* spare words */
    FMC3G_SPARE0(c) = spare[0];
    FMC3G_SPARE1(c) = spare[1];
    FMC3G_SPARE2(c) = spare[2];
    FMC3G_SPARECTL(c)  = SPARECTL_WRITE;
    FMC3G_SPARETRIG(c) = 1;
    { nand3g_dbg_wstep = 1; if (wait_clear(&FMC3G_SPARETRIG(c), 1)) goto tmo; }

    { nand3g_dbg_wstep = 2; if (wait_stat(c, STAT_DATA_DONE)) goto tmo; }
    { nand3g_dbg_wstep = 3; if (nand3g_ecc_encode(0)) goto tmo; }

    FMC3G_CMD(c) = 0x80;
    { nand3g_dbg_wstep = 4; if (wait_stat(c, STAT_RBB_DONE)) goto tmo; }
    FMC3G_ANUM(c)  = 4;
    FMC3G_ADDR0(c) = page << 16;
    FMC3G_ADDR1(c) = page >> 16;
    FMC3G_CTRL1(c) = CTRL1_XFER_ADDR;
    { nand3g_dbg_wstep = 5; if (wait_stat(c, STAT_CMD_DONE)) goto tmo; }

    for (uint32_t chunk = 0; chunk < 4; chunk++) {
        /* 16 parity/meta bytes of this chunk */
        FMC3G_DNUM(c)  = 0x0F;
        FMC3G_ADDR2(c) = 0x1000;
        FMC3G_WCOL(c)  = chunk * 0x10;
        FMC3G_CTRL1(c) = CTRL1_W_PARITY;
        { nand3g_dbg_wstep = 6; if (wait_stat(c, STAT_ADDR_DONE)) goto tmo; }

        uint32_t bufbit = (chunk & 1) ? 0x200 : 0x100;
        if (chunk < 3) {
            /* data of this chunk -> NAND while DMA-ing the next chunk in */
            nand3g_dma_in(c, data, chunk + 1);
            FMC3G_ADDR2(c) = bufbit | ((chunk & 1) ? 0x1 : 0x2);
            FMC3G_DNUM(c)  = 0x1FF;
            FMC3G_CTRL1(c) = CTRL1_W_DATA_DMA;
            { nand3g_dbg_wstep = 7; if (wait_stat(c, STAT_DATA_DONE)) goto tmo; }
            { nand3g_dbg_wstep = 8; if (nand3g_ecc_encode(chunk + 1)) goto tmo; }
            { nand3g_dbg_wstep = 9; if (wait_stat(c, STAT_ADDR_DONE)) goto tmo; }
        } else {
            FMC3G_CTRL0(c) &= ~CTRL0_DMA_MODE;
            FMC3G_UNK38(c) = 7;
            FMC3G_ADDR2(c) = bufbit;
            FMC3G_DNUM(c)  = 0x1FF;
            FMC3G_CTRL1(c) = CTRL1_W_DATA_LAST;
            { nand3g_dbg_wstep = 10; if (wait_stat(c, STAT_ADDR_DONE)) goto tmo; }
        }
    }

    FMC3G_CMD(c) = 0x10;
    { nand3g_dbg_wstep = 11; if (wait_stat(c, STAT_RBB_DONE)) goto tmo; }

    int st = nand3g_status_byte(c);
    FMC3G_CTRL0(c) = CTRL0_DESELECT;
    if (st < 0) return NAND3G_ETIMEOUT_STATUS;
    return st & 1;
tmo:
    nand3g_dbg_wfail_stat = FMC3G_STAT(c);
    nand3g_dbg_wfail_ecc  = ECC3G_IRQSTAT;
    nand3g_dbg_wfail_sp   = FMC3G_SPARETRIG(c);
    FMC3G_CTRL0(c) = CTRL0_DESELECT;
    return NAND3G_ETIMEOUT_DATA;
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
    nand3g_guard_check(1, page, bank, r);
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

/* Write/erase glue for the FTL. Writes are gated by nand3g_write_enable so
   that a build with write support can still be run without touching the
   flash (restore-only mount tests). */
/* Writes: ON in the main firmware since 2026-09-11 (FTL write path fixed
   and verified offline in utils/nano3g-ftlsim, then on the device: NAND
   write tests 1-3 and FTL write tests 1-2 accepted by the OF). The
   bootloaders keep them OFF; the dev bootloader tests enable writes
   explicitly for their duration. */
/* 2026-09-11 evening: writes OFF again in the main firmware while the
   in-memory FTL context corruption seen at the first NOR-booted shutdown
   (control block list 25/32779/25 written into a clean context) is being
   hunted with the FTL debug page. */
#ifdef BOOTLOADER
int nand3g_write_enable = 0;
#else
int nand3g_write_enable = 1;   /* guarded DMA buffer + watchdog in place */
#endif
unsigned nand3g_stat_writes, nand3g_stat_erases, nand3g_stat_write_errors;

uint32_t nand_write_page(uint32_t bank, uint32_t page, void* databuffer,
                         void* sparebuffer, uint32_t doecc)
{
    (void)doecc;
    if (!nand3g_write_enable) return 1;
    if (bank >= 4 || nand_type[bank] < 0) return 1;
    uint32_t sp[NAND3G_SPARE_WORDS] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
    if (sparebuffer) memcpy(sp, sparebuffer, 12);
    uint8_t *buf = nand3g_page_buffer();
    if (databuffer) memcpy(buf, databuffer, NAND3G_PAGE_SIZE);
    else memset(buf, 0xFF, NAND3G_PAGE_SIZE);
    mutex_lock(&nand_mtx);
    nand_last_activity_value = current_tick;
    nand3g_stat_writes++;
    int r = nand3g_write_page(NAND3G_CTRL, bank, page, buf, sp);
    nand3g_guard_check(2, page, bank, r);
    mutex_unlock(&nand_mtx);
    if (r != 0) { nand3g_stat_write_errors++; return 1; }
    return 0;
}

uint32_t nand_write_page_start(uint32_t bank, uint32_t page, void* databuffer,
                               void* sparebuffer, uint32_t doecc)
{
    return nand_write_page(bank, page, databuffer, sparebuffer, doecc);
}

uint32_t nand_write_page_collect(uint32_t bank)
{
    (void)bank;
    return 0;
}

uint32_t nand_reset(uint32_t bank)
{
    if (bank >= 4) return 1;
    mutex_lock(&nand_mtx);
    int r = nand3g_reset(NAND3G_CTRL, bank);
    mutex_unlock(&nand_mtx);
    return r ? 1 : 0;
}

uint32_t nand_block_erase(uint32_t bank, uint32_t page)
{
    if (!nand3g_write_enable) return 1;
    if (bank >= 4 || nand_type[bank] < 0) return 1;
    mutex_lock(&nand_mtx);
    nand_last_activity_value = current_tick;
    nand3g_stat_erases++;
    int r = nand3g_erase_block(NAND3G_CTRL, bank, page / NAND3G_PAGES_PER_BLOCK);
    nand3g_guard_check(3, page, bank, r);
    mutex_unlock(&nand_mtx);
    if (r != 0) { nand3g_stat_write_errors++; return 1; }
    return 0;
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
    nand3g_guard_fill();
    return (nand_type[0] < 0) ? -1 : 0;
}

/* ------------------------------------------------------------------------
 * Rockbox storage API, mirroring firmware/target/arm/s5l8700/ata-nand-s5l8700.c
 * ---------------------------------------------------------------------- */

static bool ftl_mounted;

/* Export only the FAT partition as the storage device (a "superfloppy"),
   the way Apple's disk mode does. The MBR on the flash names the Apple
   firmware area (type 0x63, 160 MB) as a second partition, which Windows
   then offers to format whenever Rockbox is connected over USB. Rockbox
   has no use for that area (the bootloader loads rockbox.ipod from the
   FAT volume), so the window below hides it from the file system code and
   from USB hosts alike, and a host can never write outside the FAT
   partition. Set NAND3G_EXPORT_FAT_ONLY to 0 to export the whole flash
   (raw disk images through Rockbox's USB mode then include the MBR and the
   firmware partition again). */
#ifndef NAND3G_EXPORT_FAT_ONLY
#define NAND3G_EXPORT_FAT_ONLY 1
#endif
static sector_t nand3g_win_start;   /* SECTOR_SIZE units */
static sector_t nand3g_win_size;
uint32_t nand3g_win_info[3];        /* debug: MBR unit multiplier, entry start, entry size */
static uint8_t nand3g_win_buf[SECTOR_SIZE] STORAGE_ALIGN_ATTR;

static uint32_t nand3g_le32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void nand3g_find_window(void)
{
    uint32_t total = ftl_nand_type->userblocks * 8 * ftl_nand_type->pagesperblock;
    uint32_t start[4], size[4];
    uint8_t type[4];
    uint32_t i, m;

    nand3g_win_start = 0;
    nand3g_win_size = total;
    nand3g_win_info[0] = nand3g_win_info[1] = nand3g_win_info[2] = 0;
#if NAND3G_EXPORT_FAT_ONLY
    if (ftl_read(0, 1, nand3g_win_buf) != 0) return;
    if (nand3g_win_buf[510] != 0x55 || nand3g_win_buf[511] != 0xAA) return;
    for (i = 0; i < 4; i++)
    {
        const uint8_t *e = nand3g_win_buf + 0x1BE + 16 * i;
        type[i] = e[4];
        start[i] = nand3g_le32(e + 8);
        size[i] = nand3g_le32(e + 12);
    }
    for (i = 0; i < 4; i++)
    {
        if ((type[i] != 0x0B && type[i] != 0x0C) || start[i] == 0 || size[i] == 0)
            continue;
        /* The MBR entries are in the OF's units (4096-byte sectors on the
           MB245), the FTL's are SECTOR_SIZE: find the FAT boot sector at
           the candidate offsets and check its bytes-per-sector agrees. */
        for (m = MAX_VIRT_SECTOR_SIZE / SECTOR_SIZE; m >= 1; m >>= 1)
        {
            sector_t s = (sector_t)start[i] * m;
            const uint8_t *b = nand3g_win_buf;
            if (s >= total || ftl_read(s, 1, nand3g_win_buf) != 0) continue;
            if (b[510] != 0x55 || b[511] != 0xAA) continue;
            if (b[0] != 0xEB && b[0] != 0xE9) continue;
            if ((uint32_t)(b[11] | (b[12] << 8)) != SECTOR_SIZE * m) continue;
            nand3g_win_start = s;
            nand3g_win_size = (sector_t)size[i] * m;
            if (nand3g_win_start + nand3g_win_size > total)
                nand3g_win_size = total - nand3g_win_start;
            nand3g_win_info[0] = m; nand3g_win_info[1] = start[i]; nand3g_win_info[2] = size[i];
            return;
        }
    }
#else
    (void)start; (void)size; (void)type; (void)i; (void)m;
#endif
}

unsigned nand3g_stat_reinit, nand3g_stat_reinit_synced;
uint32_t nand3g_stat_reinit_rc;
int nand_init(void)
{
    /* Rockbox re-runs storage_init() when entering and leaving USB mode.
       The FTL is already mounted then and may hold unsynced log state, so
       sync it rather than mounting again from the (older) on-flash
       context. The counters show on the FTL debug page. */
    if (ftl_mounted)
    {
#if defined(BOOTLOADER)
        return 0;   /* the bootloaders re-init from their menus; never sync there */
#else
        nand3g_stat_reinit++;
        if (!nand3g_write_enable) return 0;
        nand3g_stat_reinit_rc = ftl_sync();
        if (nand3g_stat_reinit_rc == 0) nand3g_stat_reinit_synced++;
        return nand3g_stat_reinit_rc ? 1 : 0;
#endif
    }
    if (ftl_init()) return 1;
    ftl_mounted = true;
    nand3g_find_window();
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
unsigned nand3g_stat_flushes;
int nand_flush(void)
{
    nand3g_stat_flushes++;
    return ftl_sync();
}
#endif

int nand_read_sectors(IF_MD(int drive,) sector_t start, int incount,
                     void* inbuf)
{
#ifdef HAVE_MULTIDRIVE
    (void) drive;
#endif
    if (start + incount > nand3g_win_size || start + incount < start) return -1;
    return ftl_read(start + nand3g_win_start, incount, inbuf);
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
    if (!nand3g_write_enable)
    {
        nand3g_stat_dropped_writes += count;
        return 0;
    }
    if (start + count > nand3g_win_size || start + count < start) return -1;
    {
        uint32_t rc = ftl_write(start + nand3g_win_start, count, outbuf);
        if (rc == 0) return 0;
#if !defined(BOOTLOADER) && defined(NANO3G_FTL_DIAG)
        extern void ftl_nano3g_crumb_write(int code, uint32_t sector, uint32_t count);
        ftl_nano3g_crumb_write((int)rc, start, count);
#endif
        return -1;
    }
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
    info->sector_size = SECTOR_SIZE;
    info->num_sectors = nand3g_win_size;   /* the FAT partition, see nand3g_find_window() */
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
