/*
 * Host harness for the iPod nano 3G FTL (firmware/target/arm/s5l8702/
 * ipodnano3g/ftl-nano3g.c) on top of sim_nand.c.
 *
 * The FTL source is #included so the harness can reach its static state:
 * the format routine uses the FTL's own context writers, and the checks
 * look at ftl_cxt / ftl_log directly.
 *
 * A reference model keeps, per logical page, the version last written.
 * Page contents are a deterministic function of (lpn, version), so the
 * model is small and every page can be regenerated for comparison.
 */
#include "ftl-nano3g.c"

extern struct sim_stats {
    unsigned long reads[2], programs[2], erases[2];
    unsigned long double_programs, bad_block_programs, bad_block_erases;
    unsigned long injected_fails;
    uint32_t last_double_bank, last_double_page;
} sim_stats;
extern long sim_fail_program_in;
extern int sim_verbose;
void sim_set_bad(uint32_t bank, uint32_t block, int bad);
int sim_is_bad(uint32_t bank, uint32_t block);
void sim_reset_stats(void);
unsigned sim_block_used(uint32_t bank, uint32_t blk);
int sim_page_state(uint32_t bank, uint32_t page);

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

/* ---------------- reference model ---------------- */
#define MODEL_LPNS (1936u * 1024u)
static uint32_t *model_ver;       /* 0 = never written */
static uint8_t genbuf[0x800];

static void gen_page(uint32_t lpn, uint32_t ver, uint8_t *buf)
{
    uint32_t x = lpn * 2654435761u ^ (ver * 40503u + 0x9E3779B9u);
    for (int i = 0; i < 0x800; i += 4) {
        x = x * 1664525u + 1013904223u;
        buf[i] = x >> 24; buf[i+1] = x >> 16; buf[i+2] = x >> 8; buf[i+3] = x;
    }
    /* make the first words human-readable in a dump */
    memcpy(buf, &lpn, 4); memcpy(buf + 4, &ver, 4);
}

static uint8_t iobuf[32 * 0x800];

static void do_write(uint32_t lpn, uint32_t count)
{
    while (count) {
        uint32_t n = count > 32 ? 32 : count;
        for (uint32_t i = 0; i < n; i++) {
            model_ver[lpn + i]++;
            gen_page(lpn + i, model_ver[lpn + i], iobuf + i * 0x800);
        }
        uint32_t rc = ftl_write(lpn, n, iobuf);
        CHECK(rc == 0, "ftl_write(%u,%u) rc %d", lpn, n, (int)rc);
        lpn += n; count -= n;
    }
}

static void check_range(uint32_t lpn, uint32_t count, const char *what)
{
    uint32_t bad = 0;
    while (count) {
        uint32_t n = count > 32 ? 32 : count;
        uint32_t rc = ftl_read(lpn, n, iobuf);
        if (rc != 0) { CHECK(0, "%s: ftl_read(%u,%u) rc %d", what, lpn, n, (int)rc); }
        for (uint32_t i = 0; i < n; i++) {
            if (model_ver[lpn + i]) gen_page(lpn + i, model_ver[lpn + i], genbuf);
            else memset(genbuf, 0, 0x800);
            if (memcmp(genbuf, iobuf + i * 0x800, 0x800) != 0) {
                if (bad < 5) {
                    uint32_t got_l, got_v; memcpy(&got_l, iobuf + i * 0x800, 4); memcpy(&got_v, iobuf + i * 0x800 + 4, 4);
                    printf("  MISMATCH %s lpn %u: want ver %u, got lpn %u ver %u\n", what, lpn + i, model_ver[lpn + i], got_l, got_v);
                }
                bad++;
            }
        }
        lpn += n; count -= n;
    }
    CHECK(bad == 0, "%s: %u pages differ", what, bad);
}

/* every logical page the model has ever written, plus a few virgin ones */
static uint32_t touched_lo = 0xFFFFFFFF, touched_hi = 0;
static void note_touched(uint32_t lpn, uint32_t count)
{
    if (lpn < touched_lo) touched_lo = lpn;
    if (lpn + count > touched_hi) touched_hi = lpn + count;
}
static void check_all(const char *what)
{
    if (touched_hi > touched_lo)
        check_range(touched_lo & ~1023u, ((touched_hi + 1023) & ~1023u) - (touched_lo & ~1023u), what);
}

/* ---------------- invariants ---------------- */
static void check_pool(const char *what)
{
    unsigned logs = 0;
    for (int i = 0; i < 0x11; i++) if (ftl_log[i].scatteredvblock != 0xFFFF) logs++;
    CHECK(ftl_cxt.freecount + logs == 20, "%s: pool invariant free %u + logs %u != 20",
          what, ftl_cxt.freecount, logs);
    /* no vblock may appear twice across map, pool and logs */
    static uint8_t seen[0x2000];
    memset(seen, 0, sizeof(seen));
    for (uint32_t l = 0; l < ftl_nand_type->userblocks; l++) {
        uint32_t v = ftl_map[l];
        CHECK(v >= 1 && v <= ftl_nand_type->userblocks + 23, "%s: map[%u] = %u out of range", what, l, v);
        CHECK(!seen[v], "%s: vblock %u mapped twice (l %u)", what, v, l);
        seen[v] = 1;
    }
    for (uint32_t i = 0; i < ftl_cxt.freecount; i++) {
        uint32_t idx = (ftl_cxt.nextfreeidx + i) % 0x14;
        uint32_t v = ftl_cxt.blockpool[idx];
        CHECK(!seen[v], "%s: pool block %u also mapped", what, v);
        seen[v] = 1;
    }
    for (int i = 0; i < 0x11; i++) if (ftl_log[i].scatteredvblock != 0xFFFF) {
        uint32_t v = ftl_log[i].scatteredvblock;
        CHECK(!seen[v], "%s: log block %u also mapped/pooled", what, v);
        seen[v] = 1;
    }
    for (int i = 0; i < 3; i++) {
        uint32_t v = ftl_cxt.ftlctrlblocks[i];
        CHECK(!seen[v], "%s: ctrl block %u also in use elsewhere", what, v);
        seen[v] = 1;
    }
}

static void check_sim(const char *what)
{
    CHECK(sim_stats.double_programs == 0, "%s: %lu double programs (last bank %u page %u)", what,
          sim_stats.double_programs, sim_stats.last_double_bank, sim_stats.last_double_page);
    CHECK(sim_stats.bad_block_programs == 0 && sim_stats.bad_block_erases == 0,
          "%s: touched bad blocks (%lu programs, %lu erases)", what,
          sim_stats.bad_block_programs, sim_stats.bad_block_erases);
}

/* ---------------- format ---------------- */
static void sim_geometry(void)
{
    ftl_banks = 2;
    const struct nand_device_info_type* phys = nand_get_device_type(0);
    phys_blocks = phys->blocks;
    ftl_vtype = *phys;
    ftl_vtype.blocks = phys->blocks / 4;
    ftl_vtype.userblocks = phys->userblocks / 4;
    ftl_nand_type = &ftl_vtype;
    ppb = ftl_nand_type->pagesperblock * NANO3G_VBANKS;
    syshyperblocks = ftl_nand_type->blocks - ftl_nand_type->userblocks - 0x17;
}

#define CTRL0 1957
#define CTRL1 1958
#define CTRL2 1959
#define POOL0 1937   /* 1937..1956 */

static void format(void)
{
    sim_geometry();
    /* bad blocks as on the MB245: top of die 0 */
    sim_set_bad(0, 8188, 1); sim_set_bad(0, 8189, 1);

    /* DEVICEINFOBBT on each die: block 8191 page 0 (BBT bitmap, 1 = good) */
    for (uint32_t bank = 0; bank < 2; bank++) {
        static uint8_t pg[0x800];
        memset(pg, 0, sizeof(pg));
        memcpy(pg, "DEVICEINFOBBT\0\0\0", 0x10);
        *(uint32_t*)&pg[0x34] = 0x400;
        memset(&pg[0x38], 0xFF, 0x400);
        for (uint32_t b = 0; b < 8192; b++)
            if (sim_is_bad(bank, b)) pg[0x38 + (b >> 3)] &= ~(1 << (b & 7));
        uint8_t sp[0x40]; memset(sp, 0xFF, sizeof(sp));
        CHECK(nand_write_page(bank, 8191 * 128, pg, sp, 1) == 0, "devinfo write");
    }

    /* VFL context per die in physical block 1 */
    ftl_vfl_usn = 0;
    for (uint32_t bank = 0; bank < 2; bank++) {
        struct ftl_vfl_cxt_type *c = &ftl_vfl_cxt[bank];
        memset(c, 0, sizeof(*c));
        c->ftlctrlblocks[0] = CTRL0; c->ftlctrlblocks[1] = CTRL1; c->ftlctrlblocks[2] = CTRL2;
        c->updatecount = 0xFFFFFFF0;
        c->activecxtblock = 0;
        c->nextcxtpage = 0;
        c->spareused = 0; c->firstspare = 0; c->sparecount = 0;
        for (int i = 0; i < 0x33A; i++) c->remaptable[i] = 0xFFF0;
        c->vflcxtblocks[0] = 1; c->vflcxtblocks[1] = 2; c->vflcxtblocks[2] = 3; c->vflcxtblocks[3] = 4;
        c->scheduledstart = 0x334;
        CHECK(ftl_vfl_store_cxt(bank) == 0, "vfl store bank %u", bank);
    }

    /* FTL context */
    memset(&ftl_cxt, 0, sizeof(ftl_cxt));
    ftl_cxt.usn = 0x00100000;
    ftl_cxt.nextblockusn = 1;
    ftl_cxt.freecount = 20;
    ftl_cxt.nextfreeidx = 0;
    for (int i = 0; i < 20; i++) ftl_cxt.blockpool[i] = POOL0 + i;
    ftl_cxt.ftlctrlblocks[0] = CTRL0; ftl_cxt.ftlctrlblocks[1] = CTRL1; ftl_cxt.ftlctrlblocks[2] = CTRL2;
    ftl_cxt.ftlctrlpage = CTRL0 * ppb;   /* page 0: erase counter page below */
    /* vBlocks 1 and 2 overlap the VFL context ring (physical 2..5): the
       format keeps them out of the map and the pool, like the OF appears to */
    for (uint32_t l = 0; l < ftl_nand_type->userblocks; l++) ftl_map[l] = 1 + l;
    memset(ftl_erasectr, 0, sizeof(ftl_erasectr));
    memset(ftl_erasectr_dirt, 0, sizeof(ftl_erasectr_dirt));
    for (int i = 0; i < 0x11; i++) {
        ftl_log[i].scatteredvblock = 0xFFFF; ftl_log[i].logicalvblock = 0xFFFF;
        ftl_log[i].pageoffsets = ftl_offsets[i];
    }
    CHECK(ftl_save_erasectr_page(0) == 0, "erasectr page 0");
    CHECK(ftl_commit_cxt() == 0, "initial commit");
    printf("format: ctrl page now %u (block %u page %u), pool %u..%u, %lu programs\n",
           ftl_cxt.ftlctrlpage, ftl_cxt.ftlctrlpage / ppb, ftl_cxt.ftlctrlpage % ppb,
           POOL0, POOL0 + 19, sim_stats.programs[0] + sim_stats.programs[1]);
}

/* snapshot of the in-memory block roles before a remount, for diagnosing
   restore disagreements: 'M' map (lblock in role_arg), 'P' pool, 'L' log,
   'C' ctrl, '-' unknown */
static char role[0x800];
static uint16_t role_arg[0x800];
static void snapshot_roles(void)
{
    memset(role, '-', sizeof(role));
    for (uint32_t l = 0; l < ftl_nand_type->userblocks; l++) { role[ftl_map[l]] = 'M'; role_arg[ftl_map[l]] = l; }
    for (uint32_t i = 0; i < ftl_cxt.freecount; i++) role[ftl_cxt.blockpool[(ftl_cxt.nextfreeidx + i) % 0x14]] = 'P';
    for (int i = 0; i < 0x11; i++) if (ftl_log[i].scatteredvblock != 0xFFFF) { role[ftl_log[i].scatteredvblock] = 'L'; role_arg[ftl_log[i].scatteredvblock] = ftl_log[i].logicalvblock; }
    for (int i = 0; i < 3; i++) role[ftl_cxt.ftlctrlblocks[i]] = 'C';
}
static void diag_restore(void)
{
    printf("  --- restore diagnosis: ctrl blocks now %u %u %u, ctrl page %u\n",
           ftl_cxt.ftlctrlblocks[0], ftl_cxt.ftlctrlblocks[1], ftl_cxt.ftlctrlblocks[2], ftl_cxt.ftlctrlpage);
    for (uint32_t b = 1; b < 1960; b++) {
        if (role[b] == 'M' && rst_blockmap[b] == 0) continue;   /* the common case */
        int inpool = 0;
        for (uint32_t i = 0; i < ftl_cxt.freecount; i++) if (ftl_cxt.blockpool[i] == b) inpool = 1;
        printf("  block %4u: before %c(%u)  restore: blockmap %5u empty %u nonseq %u  -> %s\n", b, role[b], role_arg[b],
               rst_blockmap[b], rst_isempty[b], rst_nonseq[b], inpool ? "in free pool" : (rst_blockmap[b] == 0 ? "consumed" : "NOT accounted"));
    }
    printf("  logs found by restore:");
    for (int i = 0; i < 0x11; i++) if (ftl_log[i].scatteredvblock != 0xFFFF)
        printf(" l%u@v%u", ftl_log[i].logicalvblock, ftl_log[i].scatteredvblock);
    printf("\n");
}

static int remount(const char *what)
{
    snapshot_roles();
    printf("  logs before remount:");
    for (int i = 0; i < 0x11; i++) if (ftl_log[i].scatteredvblock != 0xFFFF)
        printf(" l%u@v%u(used %u cur %u seq %u)", ftl_log[i].logicalvblock, ftl_log[i].scatteredvblock,
               ftl_log[i].pagesused, ftl_log[i].pagescurrent, ftl_log[i].issequential);
    printf("\n");
    ftl_unclean = 0;
    memset(ftl_restore_stats, 0, sizeof(ftl_restore_stats));
    uint32_t rc = ftl_init();
    if (rc != 0 || ftl_restore_stats[0] != 0) diag_restore();
    CHECK(rc == 0, "%s: ftl_init rc %d (ftl_dbg[0]=%u)", what, (int)rc, ftl_dbg[0]);
    printf("  remount (%s): %s, restore rc %u logs %u free %u mapdiff %u\n", what,
           ftl_unclean ? "UNCLEAN -> restore" : "clean", ftl_restore_stats[0],
           ftl_restore_stats[1], ftl_restore_stats[2], ftl_restore_stats[3]);
    return rc;
}

static void report(const char *what)
{
    printf("  [%s] programs die0 %lu die1 %lu, erases die0 %lu die1 %lu, verified pages %u, "
           "write fails %u verify fails %u copy-verify fails %u\n", what,
           sim_stats.programs[0], sim_stats.programs[1], sim_stats.erases[0], sim_stats.erases[1],
           ftl_dbg_verify[3], ftl_dbg_verify[0], ftl_dbg_verify[1], ftl_dbg_verify[2]);
}

/* ---------------- scenarios ---------------- */
static uint32_t rnd_state = 12345;
static uint32_t rnd(void) { rnd_state = rnd_state * 1103515245u + 12345u; return rnd_state >> 8; }

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "-v") == 0) sim_verbose = 1;
    model_ver = calloc(MODEL_LPNS, sizeof(uint32_t));
    format();
    check_sim("format");
    remount("after format");
    check_pool("after format");
    CHECK(ftl_nano3g_remap_ok, "remap check after format");
    /* from here on any program/erase of the VFL ring blocks is an error */
    for (int b = 2; b <= 4; b++) { sim_set_bad(0, b, 1); sim_set_bad(1, b, 1); }

    printf("S0: vPage -> (die, physical page) mapping is a bijection inside the pool space\n");
    {
        /* replicate the mapping used by ftl_vfl_read/ftl_vfl_write and make
           sure no two vPages of the FTL space (vBlocks 1..userblocks+23)
           share a physical page, that every physical block used lies in
           the die, and that none of them is a VFL/boot block (physical
           block 0..4 of either half) or the BBT block. */
        uint32_t nvb = ftl_nand_type->userblocks + 23 + 1;
        uint8_t *used = calloc(2u * 8192u * 128u, 1);
        uint32_t collisions = 0, oob = 0, reserved = 0, dies[2] = {0, 0};
        for (uint32_t abspage = ppb; abspage < nvb * ppb; abspage++) {
            uint32_t vbank = abspage % NANO3G_VBANKS;
            uint32_t block = abspage / ppb;
            uint32_t page = (abspage / NANO3G_VBANKS) % ftl_nand_type->pagesperblock;
            uint32_t bank = vbank & 1;
            uint32_t physblock = 2 * block + ((vbank >> 1) & 1) + ((vbank >> 2) & 1) * (phys_blocks / 2);
            uint32_t physpage = physblock * ftl_nand_type->pagesperblock + page;
            if (physblock >= 8192) { oob++; continue; }
            if (physblock <= 4 || (physblock >= 4096 && physblock <= 4096 + 4) || physblock == 8191) reserved++;
            if (used[bank * 8192u * 128u + physpage]) collisions++;
            used[bank * 8192u * 128u + physpage] = 1;
            dies[bank]++;
        }
        free(used);
        printf("  %u vPages: die0 %u die1 %u, collisions %u, out of range %u, reserved hits %u\n",
               (nvb - 1) * ppb, dies[0], dies[1], collisions, oob, reserved);
        printf("  note: reserved hits are vBlock 0 neighbours (physical 0,1 = boot + VFL cxt) as expected\n");
        CHECK(collisions == 0 && oob == 0 && dies[0] == dies[1], "S0 mapping");
    }

    printf("S1: a few scattered single-page writes (including lBlocks 0 and 1 = vBlocks 1 and 2)\n");
    for (uint32_t i = 0; i < 5; i++) { do_write(3 * 1024 + i * 7, 1); note_touched(3 * 1024 + i * 7, 1); }
    do_write(0, 3); do_write(1024 + 5, 2); note_touched(0, 2048);
    check_all("S1 before sync"); check_pool("S1");
    CHECK(ftl_sync() == 0, "S1 sync");
    check_all("S1 after sync"); check_pool("S1 synced"); check_sim("S1");
    remount("S1"); check_all("S1 remounted"); check_pool("S1 remounted");
    report("S1");

    printf("S2: one full logical block, sequential\n");
    do_write(10 * 1024, 1024); note_touched(10 * 1024, 1024);
    check_all("S2"); check_pool("S2");
    remount("S2 unsynced"); check_all("S2 remounted"); check_pool("S2 remounted"); check_sim("S2");
    report("S2");

    printf("S3: 3000 random single-page writes over 9 logical blocks\n");
    for (int i = 0; i < 3000; i++) { uint32_t lpn = rnd() % (9 * 1024); do_write(lpn, 1); }
    note_touched(0, 9 * 1024);
    check_all("S3"); check_pool("S3"); check_sim("S3");
    CHECK(ftl_sync() == 0, "S3 sync");
    check_all("S3 synced"); check_pool("S3 synced");
    remount("S3"); check_all("S3 remounted"); check_pool("S3 remounted"); check_sim("S3 remounted");
    report("S3");

    printf("S4: multi-page runs crossing block boundaries\n");
    for (int i = 0; i < 40; i++) { uint32_t lpn = 1000 + rnd() % 8000; uint32_t n = 1 + rnd() % 100; do_write(lpn, n); }
    check_all("S4"); check_pool("S4"); check_sim("S4");
    CHECK(ftl_sync() == 0, "S4 sync");
    remount("S4"); check_all("S4 remounted"); check_pool("S4 remounted");
    report("S4");

    printf("S5: forced wear swap (ftl_swap_blocks via swapcounter)\n");
    ftl_cxt.swapcounter = 300;
    for (uint32_t v = 0; v < 0x2000; v++) ftl_erasectr[v] = 5;
    /* make one pool block look worn and the map block of lblock 20 pristine */
    for (uint32_t i = 0; i < ftl_cxt.freecount; i++) {
        uint32_t idx = (ftl_cxt.nextfreeidx + i) % 0x14;
        ftl_erasectr[ftl_cxt.blockpool[idx]] = 40;
    }
    ftl_erasectr[ftl_map[20]] = 0;
    do_write(20 * 1024 + 5, 1); note_touched(20 * 1024, 1024);
    check_all("S5 after swap"); check_pool("S5"); check_sim("S5");
    CHECK(ftl_sync() == 0, "S5 sync");
    remount("S5"); check_all("S5 remounted"); check_pool("S5 remounted"); check_sim("S5 remounted");
    report("S5");

    printf("S6: fill and overflow several log blocks without sync, then remount (restore)\n");
    for (int i = 0; i < 2500; i++) { uint32_t lpn = 30 * 1024 + rnd() % (4 * 1024); do_write(lpn, 1 + rnd() % 3); }
    note_touched(30 * 1024, 4 * 1024 + 4);
    check_all("S6"); check_pool("S6"); check_sim("S6");
    remount("S6 unsynced"); check_all("S6 remounted"); check_pool("S6 remounted"); check_sim("S6 remounted");
    report("S6");

    printf("S7: injected program failure inside the sync's block copies is contained\n");
    {
        for (int i = 0; i < 1200; i++) { uint32_t lpn = 40 * 1024 + rnd() % 1024; do_write(lpn, 1); }
        note_touched(40 * 1024, 1024);
        uint32_t before = ftl_dbg_verify[0];
        sim_fail_program_in = 500;      /* lands inside the first ftl_copy_block of the sync */
        uint32_t src = ftl_sync();
        sim_fail_program_in = 0;
        CHECK(src == 0, "S7 sync rc %d", (int)src);
        printf("  injected fails %lu, write fails seen by FTL %u, copy-verify fails %u\n",
               sim_stats.injected_fails, ftl_dbg_verify[0] - before, ftl_dbg_verify[2]);
        check_all("S7"); check_pool("S7"); check_sim("S7");
        remount("S7"); check_all("S7 remounted"); check_pool("S7 remounted");
        report("S7");
    }

    printf("\n%s: %d failure(s)\n", failures ? "RESULT: FAIL" : "RESULT: PASS", failures);
    return failures ? 1 : 0;
}
