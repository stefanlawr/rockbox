/*
 * Simulated raw NAND for the iPod nano 3G FTL host harness.
 *
 * Implements the nand-target.h API that ftl-nano3g.c consumes: two dies
 * ("banks") of 8192 blocks x 128 pages x 2048 bytes, each page carrying the
 * three 32-bit metadata words the S5L8702 controller stores (reported back
 * to the FTL as the first 12 bytes of a 64-byte spare, rest 0xFF).
 *
 * The point of the simulation is to catch layout and sequencing mistakes:
 *  - programming a page that is not erased is recorded as a double program;
 *    the page then reads back as ECC-uncorrectable garbage
 *  - programming or erasing a block marked bad in the sim BBT is counted
 *  - per-bank counters show whether both dies were used
 *  - a fault injector can make the N-th program report failure
 */
#include "config.h"
#include "nand-target.h"
#include "kernel.h"
#include "panic.h"
#include <stdarg.h>

long current_tick;
int sim_verbose = 0;

#define SIM_BANKS   2
#define SIM_BLOCKS  8192
#define SIM_PPB     128
#define SIM_PAGE    2048

enum { PG_ERASED = 0, PG_PROGRAMMED = 1, PG_DOUBLE = 2 };

struct sim_page {
    uint8_t data[SIM_PAGE];
    uint8_t spare[12];
    uint8_t state;
};

static struct sim_page *sim_blk[SIM_BANKS][SIM_BLOCKS];
static uint8_t sim_bad[SIM_BANKS][SIM_BLOCKS];   /* 1 = bad block */

struct sim_stats {
    unsigned long reads[SIM_BANKS], programs[SIM_BANKS], erases[SIM_BANKS];
    unsigned long double_programs, bad_block_programs, bad_block_erases;
    unsigned long injected_fails;
    uint32_t last_double_bank, last_double_page;
} sim_stats;

/* fault injection: when > 0, counts down on each program; the program that
   brings it to 0 reports failure and is not applied */
long sim_fail_program_in = 0;

static const struct nand_device_info_type sim_info =
    { 0xA5D5D589, 8192, 7744, 0x80, 7, 3, 2, 2, 1 };

void panicf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    printf("\n*** PANIC: ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
    fflush(stdout);
    exit(2);
}

static struct sim_page *sim_get(uint32_t bank, uint32_t page, int alloc)
{
    uint32_t blk = page / SIM_PPB;
    if (bank >= SIM_BANKS || blk >= SIM_BLOCKS)
        panicf("sim: page out of range bank %u page %u", bank, page);
    if (!sim_blk[bank][blk]) {
        if (!alloc) return NULL;
        sim_blk[bank][blk] = calloc(SIM_PPB, sizeof(struct sim_page));
        for (int i = 0; i < SIM_PPB; i++) {
            memset(sim_blk[bank][blk][i].data, 0xFF, SIM_PAGE);
            memset(sim_blk[bank][blk][i].spare, 0xFF, 12);
        }
    }
    return &sim_blk[bank][blk][page % SIM_PPB];
}

void sim_set_bad(uint32_t bank, uint32_t block, int bad) { sim_bad[bank][block] = bad; }
int sim_is_bad(uint32_t bank, uint32_t block) { return sim_bad[bank][block]; }

int sim_page_state(uint32_t bank, uint32_t page)
{
    struct sim_page *p = sim_get(bank, page, 0);
    return p ? p->state : PG_ERASED;
}

void sim_reset_stats(void) { memset(&sim_stats, 0, sizeof(sim_stats)); }

/* ---- nand-target.h API ---- */

int nand_device_init(void) { return 0; }
const struct nand_device_info_type* nand_get_device_type(uint32_t bank)
{
    return bank < SIM_BANKS ? &sim_info : NULL;
}
uint32_t nand_reset(uint32_t bank) { (void)bank; return 0; }
void nand_set_active(void) {}
long nand_last_activity(void) { return current_tick; }
void nand_power_up(void) {}
void nand_power_down(void) {}

uint32_t nand_read_page(uint32_t bank, uint32_t page, void* databuffer,
                        void* sparebuffer, uint32_t doecc, uint32_t checkempty)
{
    (void)doecc;
    if (bank >= SIM_BANKS) return 1;
    sim_stats.reads[bank]++;
    struct sim_page *p = sim_get(bank, page, 0);
    uint8_t spare[0x40];
    memset(spare, 0xFF, sizeof(spare));
    uint32_t rc = 0;
    if (!p || p->state == PG_ERASED) {
        if (databuffer) memset(databuffer, 0xFF, SIM_PAGE);
        if (checkempty) rc |= 2;
    } else if (p->state == PG_DOUBLE) {
        /* garbage: AND of both writes is what real flash would hold; the
           controller's ECC gives up */
        if (databuffer) memcpy(databuffer, p->data, SIM_PAGE);
        memcpy(spare, p->spare, 12);
        rc |= 0x10;
    } else {
        if (databuffer) memcpy(databuffer, p->data, SIM_PAGE);
        memcpy(spare, p->spare, 12);
    }
    if (sparebuffer) memcpy(sparebuffer, spare, 0x40);
    return rc;
}

uint32_t nand_read_page_fast(uint32_t page, void* databuffer, void* sparebuffer,
                             uint32_t doecc, uint32_t checkempty)
{
    uint32_t i, rc = 0;
    for (i = 0; i < SIM_BANKS; i++) {
        void* d = databuffer ? (uint8_t*)databuffer + 0x800 * i : NULL;
        void* s = sparebuffer ? (uint8_t*)sparebuffer + 0x40 * i : NULL;
        uint32_t ret = nand_read_page(i, page, d, s, doecc, checkempty);
        if (ret & 1) rc |= 1 << (i << 2);
        if (ret & 2) rc |= 2 << (i << 2);
        if (ret & 0x10) rc |= 4 << (i << 2);
        if (ret & 0x100) rc |= 8 << (i << 2);
    }
    return rc;
}

uint32_t nand_write_page(uint32_t bank, uint32_t page, void* databuffer,
                         void* sparebuffer, uint32_t doecc)
{
    (void)doecc;
    if (bank >= SIM_BANKS) return 1;
    sim_stats.programs[bank]++;
    if (sim_bad[bank][page / SIM_PPB]) sim_stats.bad_block_programs++;
    if (sim_fail_program_in > 0 && --sim_fail_program_in == 0) {
        sim_stats.injected_fails++;
        return 1;
    }
    struct sim_page *p = sim_get(bank, page, 1);
    if (p->state != PG_ERASED) {
        sim_stats.double_programs++;
        sim_stats.last_double_bank = bank;
        sim_stats.last_double_page = page;
        p->state = PG_DOUBLE;
        /* flash cells can only go 1 -> 0 */
        if (databuffer) for (int i = 0; i < SIM_PAGE; i++) p->data[i] &= ((uint8_t*)databuffer)[i];
        if (sparebuffer) for (int i = 0; i < 12; i++) p->spare[i] &= ((uint8_t*)sparebuffer)[i];
        return 0;   /* the chip does not complain */
    }
    if (databuffer) memcpy(p->data, databuffer, SIM_PAGE);
    else memset(p->data, 0xFF, SIM_PAGE);
    if (sparebuffer) memcpy(p->spare, sparebuffer, 12);
    else memset(p->spare, 0xFF, 12);
    p->state = PG_PROGRAMMED;
    return 0;
}

uint32_t nand_write_page_start(uint32_t bank, uint32_t page, void* databuffer,
                               void* sparebuffer, uint32_t doecc)
{
    return nand_write_page(bank, page, databuffer, sparebuffer, doecc);
}
uint32_t nand_write_page_collect(uint32_t bank) { (void)bank; return 0; }

uint32_t nand_block_erase(uint32_t bank, uint32_t page)
{
    if (bank >= SIM_BANKS) return 1;
    uint32_t blk = page / SIM_PPB;
    if (blk >= SIM_BLOCKS) return 1;
    sim_stats.erases[bank]++;
    if (sim_bad[bank][blk]) sim_stats.bad_block_erases++;
    if (sim_blk[bank][blk]) {
        free(sim_blk[bank][blk]);
        sim_blk[bank][blk] = NULL;
    }
    return 0;
}

/* Count programmed (non-erased) pages in a physical block. */
unsigned sim_block_used(uint32_t bank, uint32_t blk)
{
    if (!sim_blk[bank][blk]) return 0;
    unsigned n = 0;
    for (int i = 0; i < SIM_PPB; i++) if (sim_blk[bank][blk][i].state != PG_ERASED) n++;
    return n;
}
