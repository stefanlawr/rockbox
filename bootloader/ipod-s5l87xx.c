/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2005 by Dave Chapman
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
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "config.h"

#include "inttypes.h"
#include "cpu.h"
#include "system.h"
#include "lcd.h"
#include "../kernel-internal.h"
#include "file_internal.h"
#include "storage.h"
#include "disk.h"
#include "font.h"
#include "backlight.h"
#include "backlight-target.h"
#include "button.h"
#include "panic.h"
#include "power.h"
#include "file.h"
#include "common.h"
#include "rb-loader.h"
#include "loader_strerror.h"
#include "version.h"
#include "powermgmt.h"
#include "usb.h"
#ifdef HAVE_SERIAL
#include "serial.h"
#endif

#include "s5l87xx.h"
#include "clocking-s5l8702.h"
#include "spi-s5l8702.h"
#include "i2c-s5l8702.h"
#include "gpio-s5l8702.h"
#include "pmu-target.h"
#if defined(IPOD_6G) || defined(IPOD_NANO3G)
#include "norboot-target.h"
#endif
#ifdef IPOD_NANO3G
#include "nand-nano3g.h"
#include "nand-target.h"
#include "ftl-target.h"
#endif


#define ERR_RB      0
#define ERR_OF      1
#define ERR_STORAGE 2
#define ERR_LBA28   3

/* Safety measure - maximum allowed firmware image size.
   The largest known current (October 2009) firmware is about 6.2MB so
   we set this to 8MB.
*/
#define MAX_LOADSIZE (8*1024*1024)

#define LCD_RBYELLOW    LCD_RGBPACK(255,192,0)
#define LCD_REDORANGE   LCD_RGBPACK(255,70,0)
#define LCD_GREEN       LCD_RGBPACK(0,255,0)

extern void bss_init(void);
extern uint32_t _movestart;
extern uint32_t start_loc;

extern int line;

#ifndef S5L87XX_DEVELOPMENT_BOOTLOADER
#ifdef HAVE_BOOTLOADER_USB_MODE
static void usb_mode(void)
{
    int button;

    verbose = true;

    printf("Entering USB mode...");

    powermgmt_init();

    /* The code will ask for the maximum possible value */
    usb_charging_enable(USB_CHARGING_ENABLE);

    usb_init();
    usb_start_monitoring();

    /* Wait until USB is plugged */
    while (usb_detect() != USB_INSERTED)
    {
        printf("Plug USB cable");
        line--;
        sleep(HZ/10);
    }

    while(1)
    {
        button = button_get_w_tmo(HZ/10);

        if (button == SYS_USB_CONNECTED)
            break; /* Hit */

        if (usb_detect() == USB_EXTRACTED)
            break; /* Cable pulled */

        /* Wait for threads to connect or cable is pulled */
        printf("USB: Connecting...");
        line--;
    }

    if (button == SYS_USB_CONNECTED)
    {
        /* Got the message - wait for disconnect */
        printf("Bootloader USB mode");

        /* Ack the SYS_USB_CONNECTED polled from the button queue */
        usb_acknowledge(SYS_USB_CONNECTED_ACK, button_get_data());

        while(1)
        {
            button = button_get_w_tmo(HZ/2);
            if (button == SYS_USB_DISCONNECTED)
                break;
        }
    }

    /* We don't want the HDD to spin up if the USB is attached again */
    usb_close();
    printf("USB mode exit     ");
}
#endif /* HAVE_BOOTLOADER_USB_MODE */

void fatal_error(int err)
{
    verbose = true;

    /* System font is 6 pixels wide */
    line++;
    switch (err)
    {
        case ERR_RB:
#ifdef HAVE_BOOTLOADER_USB_MODE
            usb_mode();
            printf("Hold MENU+SELECT to reboot");
            break;
#endif
        case ERR_STORAGE:
            printf("Hold MENU+SELECT to reboot");
            printf("then SELECT+PLAY for disk mode");
            break;
        case ERR_OF:
            printf("Hold MENU+SELECT to reboot");
            printf("and enter Rockbox firmware");
            break;
        case ERR_LBA28:
            printf("Hold MENU+SELECT to reboot");
            printf("and LEFT if you are REALLY sure");
            break;
    }

#if (CONFIG_STORAGE & STORAGE_ATA)
    if (ide_powered())
        ata_sleepnow(); /* Immediately spindown the disk. */
#endif

    line++;
    lcd_set_foreground(LCD_REDORANGE);
    while (1) {
        lcd_puts(0, line, button_hold() ? "Hold switch on!"
                                        : "               ");
        lcd_update();
    }
}

#if (CONFIG_STORAGE & STORAGE_ATA)
extern unsigned short battery_level_disksafe;
static void battery_trap(void)
{
    int vbat, old_verb;
    int th = 50;

    old_verb = verbose;
    verbose = true;

    usb_charging_maxcurrent_change(100);

    while (1)
    {
        vbat = _battery_voltage();

        /*  Two reasons to use this threshold (may require adjustments):
         *  - when USB (or wall adaptor) is plugged/unplugged, Vbat readings
         *    differ as much as more than 200 mV when charge current is at
         *    maximum (~340 mA).
         *  - RB uses some sort of average/compensation for battery voltage
         *    measurements, battery icon blinks at battery_level_disksafe,
         *    when the HDD is used heavily (large database) the level drops
         *    to battery_level_shutoff quickly.
         */
        if (vbat >= battery_level_disksafe + th)
            break;
        th = 200;

        if (power_input_status() != POWER_INPUT_NONE) {
            lcd_set_foreground(LCD_RBYELLOW);
            printf("Low battery: %d mV, charging...     ", vbat);
            sleep(HZ*3);
        }
        else {
            /* Wait for the user to insert a charger */
            int tmo = 10;
            lcd_set_foreground(LCD_REDORANGE);
            while (1) {
                vbat = _battery_voltage();
                printf("Low battery: %d mV, power off in %d ", vbat, tmo);
                if (!tmo--) {
                    /* Raise Vsysok (hyst=0.02*Vsysok) to avoid PMU
                       standby<->active looping */
                    if (vbat < 3200)
                        pmu_write(PCF5063X_REG_SVMCTL, 0xA /*3200mV*/);
                    power_off();
                }
                sleep(HZ*1);
                if (power_input_status() != POWER_INPUT_NONE)
                    break;
                line--;
            }
        }
        line--;
    }

    verbose = old_verb;
    lcd_set_foreground(LCD_WHITE);
    printf("Battery status ok: %d mV            ", vbat);
}
#endif /* CONFIG_STORAGE & STORAGE_ATA */
#endif /* S5L87XX_DEVELOPMENT_BOOTLOADER */

static int launch_onb(int clkdiv)
{
#if defined(IPOD_6G) || defined(IPOD_NANO3G)
    /* SPI clock = PClk/(clkdiv+1) */
    spi_clkdiv(SPI_PORT, clkdiv);

    /* Actually IRAM1_ORIG contains current RB bootloader IM3 header,
       it will be replaced by ONB IM3 header, so this function must
       be called once!!! */
    struct Im3Info *hinfo = (struct Im3Info*)IRAM1_ORIG;
    uint32_t onb_off;

    {
        /* Derive the ONB location from the NOR itself, which works both
           when booted from NOR and when tethered (DFU leaves unrelated
           data in the IRAM1 header area). If the image at NORBOOT_OFF is
           the OF (known hash) launch it, otherwise it is an installed RB
           bootloader and the ONB sits right behind it. */
        struct Im3Info first;
        if (im3_read(NORBOOT_OFF, &first, NULL) != 0)
            return -1;
        onb_off = NORBOOT_OFF + im3_nor_sz(&first);
#ifdef IPOD_NANO3G
        unsigned char h[SIGN_SZ];
        memcpy(h, first.u.enc12.data_sign, SIGN_SZ);
        hwkeyaes(HWKEYAES_DECRYPT, HWKEYAES_UKEY, h, SIGN_SZ);
        static const unsigned char of113[SIGN_SZ] =
            { 0x60,0xAC,0x5A,0x12,0x38,0x65,0x0D,0x2B,0xC6,0x63,0x15,0x02,0xA0,0x44,0x84,0x39 };
        if (memcmp(h, of113, SIGN_SZ) == 0)
            onb_off = NORBOOT_OFF;
#endif
    }

    /* Loads ONB in IRAM0, exception vector table is destroyed !!! */
    int rc = im3_read(onb_off, hinfo, (void*)IRAM0_ORIG);

    if (rc != 0) {
        /* Restore exception vector table */
        memcpy((void*)IRAM0_ORIG, &_movestart, 4*(&start_loc-&_movestart));
        commit_discard_idcache();
        return rc;
    }

    /* Disable all external interrupts */
    eint_init();

    commit_discard_idcache();

    /* Branch to start of IRAM */
    asm volatile("mov pc, %0"::"r"(IRAM0_ORIG));
    while(1);
#elif defined(IPOD_NANO4G)
    (void) clkdiv;

    lcd_set_foreground(LCD_REDORANGE);
    printf("Not implemented");

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);

    return 0;
#endif
}

/* Launch OF when kernel mode is running */
static int kernel_launch_onb(void)
{
    disable_irq();
    int rc = launch_onb(3); /* 54/4 = 13.5 MHz. */
    enable_irq();
    return rc;
}

/*  The boot sequence is executed on power-on or reset. After power-up
 *  the device could come from a state of hibernation, OF hibernates
 *  the iPod after an inactive period of ~30 minutes, on this state the
 *  SDRAM is in self-refresh mode.
 *
 *  t0 = 0
 *     S5L8702 BOOTROM loads an IM3 image located at NOR:
 *     - IM3 header (first 0x800 bytes) is loaded at IRAM1_ORIG
 *     - IM3 body (decrypted RB bootloader) is loaded at IRAM0_ORIG
 *     The time needed to load the RB bootloader (~100 Kb) is estimated
 *     on 200~250 ms. Once executed, RB booloader moves itself from
 *     IRAM0_ORIG to IRAM1_ORIG+0x800, preserving current IM3 header
 *     that contains the NOR offset where the ONB (original NOR boot),
 *     is located (see dualboot.c for details).
 *
 *  t1 = ~250 ms.
 *     If the PMU is hibernated, decrypted ONB (size 128Kb) is loaded
 *       and executed, it takes ~120 ms. Then the ONB restores the
 *       iPod to the state prior to hibernation.
 *     If not, initialize system and RB kernel, wait for t2.
 *
 *  t2 = ~650 ms.
 *     Check user button selection.
 *     If OF, diagmode, or diskmode is selected then launch ONB.
 *     If not, wait for LCD initialization.
 *
 *  t3 = ~700,~900 ms. (lcd_type_01,lcd_type_23)
 *     LCD is initialized, baclight ON.
 *     Wait for HDD spin-up.
 *
 *  t4 = ~2600,~2800 ms.
 *     HDD is ready.
 *     If hold switch is locked, then load and launch ONB.
 *     If not, load rockbox.ipod file from HDD.
 *
 *  t5 = ~2800,~3000 ms.
 *     rockbox.ipod is executed.
 */

#ifdef S5L87XX_DEVELOPMENT_BOOTLOADER
#include "piezo.h"
#include "lcd-s5l8702.h"
extern int lcd_type;

static uint16_t alive[] = { 500,100,0, 0 };
static uint16_t alivelcd[] = { 2000,200,0, 0 };

#ifdef HAVE_LCD_SLEEP
static void sleep_test(void)
{
    int sleep_tmo = 5;
    int awake_tmo = 3;

    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;

    printf("Entering LCD sleep mode in %d seconds,", sleep_tmo);
    printf("during sleep mode you will see a white");
    printf("screen for about %d seconds.", awake_tmo);
    while (sleep_tmo--) {
        printf("Sleep in %d...", sleep_tmo);
        sleep(HZ*1);
    }
    lcd_sleep();
    sleep(HZ*awake_tmo);
    lcd_awake();

    line++;
    printf("Awake!");

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}
#endif

static void pmu_info(void)
{
    int loop = 0;

    lcd_clear_display();
    lcd_update();
    while (button_status() != BUTTON_NONE);

    while (1)
    {
        lcd_set_foreground(LCD_WHITE);
        lcd_clear_display();
        line = 0;
        printf("loop: %d", loop++);

        for (int i = 0; i < 128; i += 8)
        {
            unsigned char buf[8];

#if defined(IPOD_NANO3G)
            if (i == 0) {
                static int flip = 0;
                if (flip) {
                    pmu_write(6, 0xff);
                    pmu_write(7, 0xff);
                }
                else {
                    pmu_write(6, 0xe7);
                    pmu_write(7, 0xfe);
                }
                flip ^= 1;
            }
#elif defined(IPOD_NANO4G)
            if (i == 120)
                for (int j = 0; j < 8; j++)
                    pmu_write(i+j, j);
#endif
            for (int j = 0; j < 8; j++)
                buf[j] = pmu_read(i+j);

            printf(" %2x: %2x %2x %2x %2x %2x %2x %2x %2x", i,
                    buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
        }
        line++;
        printf("USB: %s    ", (usb_detect() == USB_INSERTED) ? "inserted" : "not inserted");
#if CONFIG_CHARGING
        printf("Firewire: %s    ", pmu_firewire_present() ? "inserted" : "not inserted");
#endif
#ifdef IPOD_ACCESSORY_PROTOCOL
        printf("Accessory: %s    ", pmu_accessory_present() ? "inserted" : "not inserted");
#endif
        printf("Hold Switch: %s  ", pmu_holdswitch_locked() ? "locked" : "unlocked");
        line++;
        lcd_set_foreground(LCD_RBYELLOW);
        printf("Press SELECT to continue");
        if (button_status() == BUTTON_SELECT)
            break;
        sleep(HZ/2);
    }
}

static void gpio_info(void)
{
    int loop = 0;

    lcd_clear_display();

    while (1)
    {
        lcd_set_foreground(LCD_WHITE);
        lcd_clear_display();
        line = 0;
        printf("loop: %d", loop++);
        for (int i = 0; i < GPIO_N_GROUPS; i ++)
        {
            printf(" %x: %8x %2x %4x %2x %2x", i,
                    PCON(i), PDAT(i), PUNA(i), PUNB(i), PUNC(i));
        }
        line++;
        lcd_set_foreground(LCD_RBYELLOW);
        printf("Press SELECT to continue");
        if (button_status() == BUTTON_SELECT)
            break;
        sleep(HZ/5);
    }
}

static void run_of(void)
{
    int tmo = 5;
    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;
    while (tmo--) {
        printf("Booting OF in %d...", tmo);
        sleep(HZ*1);
    }

    int rc = kernel_launch_onb();
    printf("Load OF error: %d", rc);
    sleep(HZ*10);
}

#if defined(IPOD_6G) || defined(IPOD_NANO3G)
static void print_syscfg(void)
{
    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;

    struct SysCfg syscfg;
    const ssize_t result = syscfg_read(&syscfg);

    if (result == -1) {
        printf("SCfg magic not found. NOR flash is corrupted.");
        goto end;
    }

    printf("Total size: %lu bytes, %lu entries", syscfg.header.size, syscfg.header.num_entries);

    if (result > 0) {
        printf("Wrong size: expected %ld, got %lu", result, syscfg.header.size);
    }

    if (syscfg.header.num_entries > SYSCFG_MAX_ENTRIES) {
        printf("Too many entries, showing only first %u", SYSCFG_MAX_ENTRIES);
    }

    const size_t syscfg_num_entries = MIN(syscfg.header.num_entries, SYSCFG_MAX_ENTRIES);

    for (size_t i = 0; i < syscfg_num_entries; i++) {
        const struct SysCfgEntry* entry = &syscfg.entries[i];
        const char* tag = (char *)&entry->tag;
        const uint32_t* data32 = (uint32_t *)entry->data;

        switch (entry->tag) {
        case SYSCFG_TAG_SRNM:
            printf("Serial number (SrNm): %s", entry->data);
            break;
        case SYSCFG_TAG_FWID:
            printf("Firmware ID (FwId): %07lX", data32[1] & 0x0FFFFFFF);
            break;
        case SYSCFG_TAG_HWID:
            printf("Hardware ID (HwId): %08lX", data32[0]);
            break;
        case SYSCFG_TAG_HWVR:
            printf("Hardware version (HwVr): %06lX", data32[1]);
            break;
        case SYSCFG_TAG_CODC:
            printf("Codec (Codc): %s", entry->data);
            break;
        case SYSCFG_TAG_SWVR:
            printf("Software version (SwVr): %s", entry->data);
            break;
        case SYSCFG_TAG_MLBN:
            printf("Logic board serial number (MLBN): %s", entry->data);
            break;
        case SYSCFG_TAG_MODN:
            printf("Model number (Mod#): %s", entry->data);
            break;
        case SYSCFG_TAG_REGN:
            printf("Sales region (Regn): %08lX %08lX", data32[0], data32[1]);
            break;
        default:
            printf("%c%c%c%c: %08lX %08lX %08lX %08lX",
                tag[3], tag[2], tag[1], tag[0],
                data32[0], data32[1], data32[2], data32[3]
            );
            break;
        }
    }

end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}

static void print_bootloader_hash(void)
{
    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;

    struct Im3Info hinfo;
    int rc = im3_read(NORBOOT_OFF, &hinfo, NULL);

    if (rc != 0) {
        printf("Error loading the primary bootloader: %d", rc);
        goto end;
    }

    unsigned char primary_hash[SIGN_SZ];

    memcpy(primary_hash, hinfo.u.enc12.data_sign, SIGN_SZ);
    hwkeyaes(HWKEYAES_DECRYPT, HWKEYAES_UKEY, primary_hash, SIGN_SZ);

    unsigned bl_nor_sz = im3_nor_sz(&hinfo);
    rc = im3_read(NORBOOT_OFF + bl_nor_sz, &hinfo, NULL);

    if (rc == 0) {
        // Rockbox bootloader is installed as primary
        // Stock bootloader is backed up
        unsigned char backup_hash[SIGN_SZ];
        memcpy(backup_hash, hinfo.u.enc12.data_sign, SIGN_SZ);
        hwkeyaes(HWKEYAES_DECRYPT, HWKEYAES_UKEY, backup_hash, SIGN_SZ);

        printf("Rockbox bootloader hash:");

        for (int i = 0; i < SIGN_SZ; i++) {
            lcd_putsf(i * 2, line, "%02X", primary_hash[i]);
        }

        line += 2;
        lcd_update();

        printf("Stock bootloader hash:");

        for (int i = 0; i < SIGN_SZ; i++) {
            lcd_putsf(i * 2, line, "%02X", backup_hash[i]);
        }

        line++;
        lcd_update();
    }
    else {
        // Stock bootloader is installed as primary
        // No backup bootloader
        printf("Rockbox bootloader is not installed!");
        line++;

        printf("Stock bootloader hash:");

        for (int i = 0; i < SIGN_SZ; i++) {
            lcd_putsf(i * 2, line, "%02X", primary_hash[i]);
        }

        line++;
        lcd_update();
    }

end:
    line++;
    while (button_status() != BUTTON_NONE)
        sleep(HZ/100);
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}

#ifdef HAVE_SERIAL

#define FLASH_PAGES (FLASH_SIZE >> 12)
#define FLASH_PAGE_SIZE (FLASH_SIZE >> 8)

static void dump_bootflash(void)
{
    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;

    uint8_t page[FLASH_PAGE_SIZE];
    printf("Total pages: %d", FLASH_PAGES);

    bootflash_init(SPI_PORT);

    for (int i = 0; i < FLASH_PAGES; i++) {
        printf("Reading flash... %d", i + 1);
        bootflash_read(SPI_PORT, i << 12, FLASH_PAGE_SIZE, page);

        printf("Sending over UART... %d", i + 1);
        serial_tx_raw(page, FLASH_PAGE_SIZE);
        line -= 2;
    }

    bootflash_close(SPI_PORT);

    line += 2;
    printf("Done!");
    piezo_seq(alive);

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}
#endif /* HAVE_SERIAL */
#endif /* IPOD_6G || IPOD_NANO3G */

#ifdef IPOD_NANO3G
/* Read-only NAND probe: chip IDs on one controller, then a page from
   block 1 (where the Whimory VFL context lives) and page 0. Everything
   is polled with timeouts so a hang shows the last line reached. */
static void nand_probe_ctrl(int ctrl)
{
    uint8_t id[8];
    uint32_t spare[NAND3G_SPARE_WORDS];
    uint32_t raw;
    int rc;

    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;
    printf("NAND probe, controller %d", ctrl);

    nand3g_hw_init(ctrl);
    printf("clocks+gpio ok");
    /* GPIO groups 8..10 carry the NAND bus; show config, data and pulls
       so they can be compared with the boot ROM / DFU state. */
    for (int g = 8; g <= 10; g++)
        printf("g%d con %08lx dat %02lx un %02lx %02lx %02lx", g,
               (unsigned long)PCON(g), (unsigned long)(PDAT(g) & 0xff),
               (unsigned long)(PUNA(g) & 0xff), (unsigned long)(PUNB(g) & 0xff),
               (unsigned long)(PUNC(g) & 0xff));
    printf("pmu 10:%02x 15:%02x 16:%02x 18:%02x 1b:%02x",
           pmu_read(0x10), pmu_read(0x15), pmu_read(0x16),
           pmu_read(0x18), pmu_read(0x1b));
    printf("pmu 2e:%02x 30:%02x 32:%02x 34:%02x 36:%02x 38:%02x 3a:%02x",
           pmu_read(0x2e), pmu_read(0x30), pmu_read(0x32), pmu_read(0x34),
           pmu_read(0x36), pmu_read(0x38), pmu_read(0x3a));

    for (int ce = 0; ce < NAND3G_NUM_CE; ce++) {
        rc = nand3g_reset(ctrl, ce);
        if (rc) {
            printf("ce%d reset err %d", ce, rc);
            continue;
        }
        rc = nand3g_read_id(ctrl, ce, id);
        if (rc) {
            printf("ce%d id err %d", ce, rc);
            continue;
        }
        printf("ce%d id %02x %02x %02x %02x %02x %02x", ce,
               id[0], id[1], id[2], id[3], id[4], id[5]);
    }
    printf("stat %08lx fifo %08lx sto %lu", (unsigned long)nand3g_dbg_stat,
           (unsigned long)nand3g_dbg_fifo,
           (unsigned long)nand3g_dbg_status_timeouts);

    uint8_t *buf = nand3g_page_buffer();
    const uint32_t pages[] = { 128, 152, 0 };
    for (unsigned i = 0; i < ARRAYLEN(pages); i++) {
        memset(buf, 0xAA, 32);
        rc = nand3g_read_page(ctrl, 0, pages[i], buf, spare, &raw);
        if (rc < 0) {
            printf("pg %lu: err %d", (unsigned long)pages[i], rc);
            continue;
        }
        printf("pg %lu: ecc %d raw %08lx", (unsigned long)pages[i], rc,
               (unsigned long)raw);
        printf(" sp %08lx %08lx %08lx", (unsigned long)spare[0],
               (unsigned long)spare[1], (unsigned long)spare[2]);
        printf(" %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
               buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
    }

    /* Second pass: bisect the GPIO table. In DFU (where the NAND works and
       its data bus idles at 0xFF) every GPIO group except 8..10 is at its
       reset state (all inputs). Rockbox's gpio_preinit drives many pins.
       Restore one group at a time to the DFU state and watch the NAND
       data bus (PDAT(8)) come back up. Values captured with wInd3x dump. */
    if (ctrl == 0) {
        static const uint32_t dfu_con[16] = {
            0x00002221, 0, 0, 0, 0, 0, 0, 0,
            0x22222222, 0x00020002, 0x00002222, 0, 0, 0, 0, 0 };
        static const uint8_t dfu_punc[16] = {
            0x00, 0xfc, 0x0f, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0xf0, 0xff, 0x03, 0x30 };
        printf("gpio bisect, dat8 now %02lx:", (unsigned long)(PDAT(8) & 0xff));
        PCON(9)  = dfu_con[9];
        PCON(10) = dfu_con[10];
        /* Per-pin bisect (GPIO ruled out on 2026-09-09: no single pin
           brings the bus up). Kept behind NAND3G_GPIO_BISECT. */
#ifdef NAND3G_GPIO_BISECT
        int found_g = -1, found_p = -1;
        uint8_t upmask[16];
        for (int g = 0; g < 16; g++) {
            upmask[g] = 0;
            if (g >= 8 && g <= 10)
                continue;
            for (int p = 0; p < 8; p++) {
                uint32_t mask = 0xFu << (4 * p);
                uint32_t save_con = PCON(g);
                if ((save_con & mask) == (dfu_con[g] & mask))
                    continue;
                PCON(g) = (save_con & ~mask) | (dfu_con[g] & mask);
                udelay(50000);
                uint8_t d8 = PDAT(8) & 0xff;
                if (d8 == 0xff) {
                    upmask[g] |= 1 << p;
                    if (found_g < 0) { found_g = g; found_p = p; }
                }
                PCON(g) = save_con;
                udelay(20000);
            }
        }
        /* Piezo report, independent of the LCD:
           found: (g+1) short beeps, pause, (p+1) short beeps.
           none:  three long beeps. */
        static uint16_t beep[] = { 1500, 120, 0, 0 };
        static uint16_t longbeep[] = { 800, 700, 0, 0 };
        if (found_g >= 0) {
            for (int i = 0; i <= found_g; i++) { piezo_seq(beep); sleep(HZ/4); }
            sleep(HZ);
            for (int i = 0; i <= found_p; i++) { piezo_seq(beep); sleep(HZ/4); }
        } else {
            for (int i = 0; i < 3; i++) { piezo_seq(longbeep); sleep(HZ/2); }
        }
        /* Bring the LCD back in case a tested pin was its reset line. */
        lcd_init();
        lcd_set_foreground(LCD_WHITE);
        lcd_set_background(LCD_BLACK);
        lcd_clear_display();
        lcd_setfont(FONT_SYSFIXED);
        line = 0;
        printf("pin bisect (mask of pins that raise dat8):");
        for (int g = 0; g < 16; g += 4)
            printf(" g%d-%d: %02x %02x %02x %02x", g, g + 3, upmask[g],
                   upmask[g+1], upmask[g+2], upmask[g+3]);
        printf("first hit: group %d pin %d", found_g, found_p);
        if (found_g >= 0) {
            uint32_t mask = 0xFu << (4 * found_p);
            PCON(found_g) = (PCON(found_g) & ~mask) | (dfu_con[found_g] & mask);
            udelay(50000);
            printf("applied; dat8 now %02lx", (unsigned long)(PDAT(8) & 0xff));
        }
#else
        (void)dfu_punc;
        printf("dat8 now %02lx, retry:", (unsigned long)(PDAT(8) & 0xff));
#endif
        rc = nand3g_reset(ctrl, 0);
        rc = nand3g_read_id(ctrl, 0, id);
        printf("ce0 id %02x %02x %02x %02x %02x %02x rc %d",
               id[0], id[1], id[2], id[3], id[4], id[5], rc);
        memset(buf, 0xAA, 32);
        rc = nand3g_read_page(ctrl, 0, 128, buf, spare, &raw);
        printf("pg 128: ecc %d raw %08lx sp %08lx", rc, (unsigned long)raw,
               (unsigned long)spare[0]);
        printf(" %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
               buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
    }

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE)
        sleep(HZ/100);
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}

static void nand_probe_ctrl0(void) { nand_probe_ctrl(0); }
static void nand_probe_ctrl1(void) { nand_probe_ctrl(1); }

/* I2S transmit test: put the WM8975 into I2S master mode over I2C, route
   12 MHz to the codec MCLK the way the Classic does, then for each of the
   three I2S instances enable its clock, configure TX like pcm-s5l8702.c,
   push samples into the TX FIFO and watch the status register. An instance
   whose status changes as data is pushed and time passes is being clocked;
   one that stays constant is not. RAM/registers only. */
static void wm_write(int reg, int data)
{
    unsigned char d = data & 0xff;
    i2c_write(0, 0x34, (reg << 1) | ((data & 0x100) >> 8), 1, &d);
}

static void i2s_tx_test(void)
{
    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;
    printf("I2S TX test (codec = WM8975 master)");

    /* MCLK: OSC0 12 MHz to the codec (CLKCON3 low half = 0, then on) */
    CLKCON3 = (CLKCON3 & ~0xffff) | 0x8000;
    udelay(100);
    CLKCON3 &= ~0x8000;
    printf("CLKCON3 %08lx CLKCON5 %08lx", (unsigned long)CLKCON3, (unsigned long)CLKCON5);
    printf("CG16 AUD0 %04x AUD1 %04x AUD2 %04x", CG16_AUD0, CG16_AUD1, CG16_AUD2);

    /* WM8975 as I2S master, 12 MHz USB mode, 44.1 kHz, outputs on */
    wm_write(0x0F, 0x000);          /* reset */
    sleep(HZ/50);
    wm_write(0x19, 0x0C0);          /* PWRMGMT1: VMID 50k, VREF */
    sleep(HZ/50);
    wm_write(0x1A, 0x1E0);          /* PWRMGMT2: DACL DACR LOUT1 ROUT1 */
    wm_write(0x07, 0x042);          /* AINTFCE: master, I2S, 16 bit */
    wm_write(0x08, 0x123);          /* SAMPCTRL: BCM=MCLK/8, SR=44.1k USB, USB */
    wm_write(0x05, 0x000);          /* DAPCTRL: unmute */
    wm_write(0x22, 0x150);          /* LOUTMIX1: DAC to left out */
    wm_write(0x25, 0x150);          /* ROUTMIX2: DAC to right out */
    wm_write(0x02, 0x179);          /* LOUT1VOL 0 dB, update */
    wm_write(0x03, 0x179);
    wm_write(0x24, 0x003);          /* OF: ROUTMIX1 */
    wm_write(0x25, 0x120);          /* OF: ROUTMIX2 */
    wm_write(0x43, 0x008);          /* OF: R67 |= 8 */
    wm_write(0x18, 0x100);          /* OF: R24 = 0x100 */
    wm_write(0x18, 0x104);          /* OF: R24 |= 4 */
    wm_write(0x1b, 0x040);          /* OF: R27 |= 0x40 */
    printf("codec configured (OF sequence)");

    /* Per the S5L8700 datasheet: I2SSTATUS bit1 = TXDBFUL (tx buffer full),
       bit0 = TXLRIDX (toggles per channel while transmitting). The 8700's
       I2S Tx is master only; the 8702 adds undocumented TXCON bits 20/24/25/27
       (Classic uses 0xb100019 with the codec as master, openiboot's iPhone
       uses 0x1100301 with the codec as slave). Try candidates on I2S0 and
       I2S1: count words until the buffer reports full, count TXLRIDX toggles,
       then stream a square wave with full-flag pacing so a working config is
       audible. */
    struct { uint32_t txcon; int codec_master; const char *name; } cand[] = {
        { 0x0b100001, 1, "OF: b100001, codec master" },
        { 0x0b100001, 0, "b100001, codec slave" },
        { 0x0b100019, 1, "classic, codec master" },
    };
    for (unsigned c = 0; c < ARRAYLEN(cand); c++) {
        /* codec interface mode for this candidate */
        wm_write(0x07, cand[c].codec_master ? 0x042 : 0x002);
        wm_write(0x08, cand[c].codec_master ? 0x123 : 0x023);
        for (int i = 0; i < 2; i++) {
            uintptr_t base = I2S_BASE + (i == 1 ? I2S_INTERFACE1_OFFSET : 0);
            volatile uint32_t *clkcon = (volatile uint32_t *)(base + 0x00);
            volatile uint32_t *txcon  = (volatile uint32_t *)(base + 0x04);
            volatile uint32_t *txcom  = (volatile uint32_t *)(base + 0x08);
            volatile uint32_t *txdb   = (volatile uint32_t *)(base + 0x10);
            volatile uint32_t *status = (volatile uint32_t *)(base + 0x3C);
            volatile uint32_t *clkdiv = (volatile uint32_t *)(base + 0x40);

            clockgate_enable(I2SCLKGATE(i), true);
            if (i == 1) cg16_config(&CG16_AUD1, true, CG16_SEL_OSC, 1, 1);
            *txcom = 0xa;
            *clkcon = 0;
            udelay(100);
            *txcon = cand[c].txcon;
            *(volatile uint32_t *)(base + 0x30) = 0x1000;   /* RXCON as the OF */
            *clkdiv = 12000000 / 44100;
            *clkcon = 1;
            *txcom = 0xe;

            int full_at = -1;
            for (int n = 0; n < 512; n++) {
                *txdb = (n & 16) ? 0x40004000 : 0xC000C000;
                if (*status & 2) { full_at = n; break; }
            }
            int toggles = 0;
            uint32_t prev = *status & 1;
            for (int n = 0; n < 4000; n++) {
                uint32_t b = *status & 1;
                if (b != prev) { toggles++; prev = b; }
            }
            /* stream ~250 ms of 1.4 kHz square wave, paced by the full flag */
            long stop = current_tick + HZ/4;
            uint32_t words = 0, n = 0;
            while (TIME_BEFORE(current_tick, stop)) {
                if (*status & 2) continue;
                *txdb = ((n++ >> 4) & 1) ? 0x40004000 : 0xC000C000;
                words++;
            }
            uint32_t st = *status;
            *txcom = 0xa;
            printf("%s i2s%d: full@%d tog %d w %lu st %lx", cand[c].name, i,
                   full_at, toggles, (unsigned long)words, (unsigned long)st);
        }
    }
    printf("(full@: words until TXDBFUL, -1 never; tog: TXLRIDX toggles/4000;");
    printf(" w: words streamed in 250 ms)");

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE)
        sleep(HZ/100);
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}

/* DMA playback test: same DMA channel setup as pcm-s5l8702.c, feeding a
   square wave to I2S0 with the OF's I2S/codec configuration. Reports how
   many DMA completion callbacks arrive in one second (expect ~20 for
   8 KiB chunks at 44.1 kHz stereo 16-bit) and the I2S status. */
#include "dma-s5l8702.h"
/* ---- NAND write test: uses one fully erased block in the VFL reserved
   range at the top of die 0 (physical 7968..8190). Round trip: erase,
   program page 0, read back, erase, verify blank. Net effect on the NAND:
   one extra erase cycle on a block that was already erased. ---- */
#define NAND3G_CTRL 0   /* FMC controller 0 (the one the NAND is on) */

/* ---- FTL write test 1: mount (with restore), overwrite the first 4 KiB
   of /testtone.wav through the file API with a pattern, read it back.
   User pages only: a pool block gets erased and a few pages programmed
   in it; no FTL context / control block is written (no ftl_sync). ---- */
#include "file.h"
#include "disk.h"
#include "storage.h"
static uint8_t wt_orig[4096] STORAGE_ALIGN_ATTR;
static uint8_t wt_pat[4096] STORAGE_ALIGN_ATTR;
static uint8_t wt_back[4096] STORAGE_ALIGN_ATTR;
/* ---- FTL write test 2: like test 1, then ftl_sync() (commits map, erase
   counters and context: control block rotation + VFL context commit),
   then a second small write so the FTL leaves its dirty marker (type
   0x47) above the new context and the OF runs its restore on top of it
   rather than trusting the context blindly. ---- */
static void mark_unclean_inplace(void)
{
    extern uint32_t ftl_unclean, ftl_dbg_ctrl[6];
    extern uint32_t ftl_nano3g_mark_unclean(void);
    extern void ftl_nano3g_vfl_dump(uint32_t, uint32_t*);
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("Mark unclean in current ctrl block");
    int rc = storage_init();
    uint32_t o[16]; ftl_nano3g_vfl_dump(0, o);
    printf("storage_init %d unclean %lu; VFL ctrl list %lx %lx %lx", rc, (unsigned long)ftl_unclean,
           (unsigned long)(o[13] & 0xffff), (unsigned long)(o[13] >> 16), (unsigned long)o[14]);
    if (rc) goto end;
    if (ftl_unclean) { printf("already unclean, nothing to do"); goto end; }
    nand3g_write_enable = 1;
    rc = ftl_nano3g_mark_unclean();
    nand3g_write_enable = 0;
    printf("mark 0x4F rc %d at vpage %lx = vblock %lu page %lu; writes %u", rc,
           (unsigned long)ftl_dbg_ctrl[3], (unsigned long)(ftl_dbg_ctrl[3] / 1024),
           (unsigned long)(ftl_dbg_ctrl[3] % 1024), nand3g_stat_writes);
end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}

static void vfl_dump(void)
{
    extern void ftl_nano3g_vfl_dump(uint32_t, uint32_t*);
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("VFL state dump");
    int rc = storage_init();
    printf("storage_init %d", rc);
    for (uint32_t b = 0; b < 2; b++)
    {
        uint32_t o[16];
        ftl_nano3g_vfl_dump(b, o);
        printf("bank %lu: usn %lx upd %lx act %lu nextpg %lu cksum %lu", (unsigned long)b,
               (unsigned long)o[0], (unsigned long)o[1], (unsigned long)o[2], (unsigned long)o[3], (unsigned long)o[12]);
        printf(" cxtblocks %lu %lu %lu %lu spare first %lu cnt %lu used %lu sched %lu",
               (unsigned long)o[4], (unsigned long)o[5], (unsigned long)o[6], (unsigned long)o[7],
               (unsigned long)o[8], (unsigned long)o[9], (unsigned long)o[10], (unsigned long)o[11]);
        printf(" ftlctrl %lx %lx %lx", (unsigned long)(o[13] & 0xffff), (unsigned long)(o[13] >> 16), (unsigned long)o[14]);
        /* raw spare of page 0 and of the latest cxt page in the active block */
        uint8_t *buf = nand3g_page_buffer(); uint32_t sp[3]; uint32_t raw;
        uint32_t blk = o[4 + (o[2] & 3)];
        for (uint32_t pg = 0; pg < 2; pg++)
        {
            uint32_t page = blk * 128 + (pg ? (o[3] ? o[3] - 1 : 0) : 0);
            int r = nand3g_read_page(NAND3G_CTRL, b, page, buf, sp, &raw);
            printf(" pblk %lu pg %lu: rc %d sp %08lx %08lx %08lx %02x%02x%02x%02x", (unsigned long)blk,
                   (unsigned long)(page % 128), r, (unsigned long)sp[0], (unsigned long)sp[1], (unsigned long)sp[2],
                   buf[0], buf[1], buf[2], buf[3]);
        }
        int r = nand3g_read_page(NAND3G_CTRL, b, 8191 * 128, buf, sp, &raw);
        printf(" devinfo 8191/0: rc %d %c%c%c%c%c%c%c%c%c%c%c%c%c", r, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
               buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12]);
    }
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}

static void erase_631(void)
{
    extern uint32_t ftl_nano3g_erase_vblock(uint32_t);
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("Erase vblock 631");
    int rc = storage_init();
    printf("storage_init %d", rc);
    if (rc) goto end;
    nand3g_write_enable = 1;
    rc = ftl_nano3g_erase_vblock(631);
    nand3g_write_enable = 0;
    printf("erase vblock 631 rc %d; erases %u errors %u", rc, nand3g_stat_erases, nand3g_stat_write_errors);
    {
        uint8_t *buf = nand3g_page_buffer(); uint32_t sp[3]; uint32_t raw;
        int r = nand3g_read_page(NAND3G_CTRL, 0, (2 * 631) * 128, buf, sp, &raw);
        printf("pblock %d page 0 after: rc %d sp %08lx data %02x%02x", 2 * 631, r, (unsigned long)sp[0], buf[0], buf[1]);
    }
end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}

static void ftl_recover(void)
{
    extern uint32_t ftl_unclean, ftl_restore_stats[8], ftl_restore_mapdiff[3], ftl_dbg_ctrl[6];
    extern uint32_t ftl_nano3g_force_restore(void);
    extern uint32_t ftl_nano3g_mark_unclean(void);
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("FTL recover");
    int rc = storage_init();
    printf("storage_init %d unclean %lu ctrl %lx %lx %lx page %lx", rc, (unsigned long)ftl_unclean,
           (unsigned long)ftl_dbg_ctrl[0], (unsigned long)ftl_dbg_ctrl[1], (unsigned long)ftl_dbg_ctrl[2],
           (unsigned long)ftl_dbg_ctrl[3]);
    if (rc) goto end;
    rc = ftl_nano3g_force_restore();
    printf("force restore rc %d logs %lu free %lu mapdiff %lu", rc, (unsigned long)ftl_restore_stats[1],
           (unsigned long)ftl_restore_stats[2], (unsigned long)ftl_restore_stats[3]);
    for (int i = 0; i < 3; i++)
        printf(" diff %d: lblock %lu committed %lu rebuilt %lu", i,
               (unsigned long)(ftl_restore_mapdiff[i] & 0xfff), (unsigned long)((ftl_restore_mapdiff[i] >> 12) & 0x3ff),
               (unsigned long)(ftl_restore_mapdiff[i] >> 22));
    if (rc) goto end;
    uint8_t *buf = nand3g_page_buffer();
    rc = ftl_read(0, 1, buf);
    printf("sector 0 via rebuilt map: rc %d sig %02x%02x", rc, buf[510], buf[511]);
    if (rc || buf[510] != 0x55 || buf[511] != 0xAA)
    {
        extern uint32_t ftl_nano3g_peek(uint32_t, uint32_t*, uint32_t*, uint32_t*, uint8_t*);
        uint32_t blk[2] = { ftl_restore_mapdiff[0] >> 22, (ftl_restore_mapdiff[0] >> 12) & 0x3ff };
        for (int b = 0; b < 2; b++)
            for (uint32_t pg = 0; pg < 4; pg++)
            {
                uint32_t lpn, usn, type; uint8_t f[4];
                uint32_t r = ftl_nano3g_peek(blk[b] * 1024 + pg, &lpn, &usn, &type, f);
                printf(" vb %lu pg %lu: ret %lx lpn %lx usn %lx type %lx %02x%02x%02x%02x",
                       (unsigned long)blk[b], (unsigned long)pg, (unsigned long)r, (unsigned long)lpn,
                       (unsigned long)usn, (unsigned long)type, f[0], f[1], f[2], f[3]);
            }
        printf("sector 0 unreadable: marking unclean anyway so the OF rebuilds");
    }
    nand3g_write_enable = 1;
    rc = ftl_nano3g_mark_unclean();
    nand3g_write_enable = 0;
    printf("mark unclean (0x4F) rc %d at page %lx; writes %u", rc, (unsigned long)ftl_dbg_ctrl[3], nand3g_stat_writes);
end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}

static void ftl_wtest2(void)
{
    extern uint32_t ftl_unclean, ftl_restore_stats[8];
    extern uint32_t ftl_dbg_ctrl[6];
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("FTL write test 2 (sync)");
    int rc = storage_init();
    printf("storage_init %d, unclean %lu restore rc %lu logs %lu free %lu", rc,
           (unsigned long)ftl_unclean, (unsigned long)ftl_restore_stats[0],
           (unsigned long)ftl_restore_stats[1], (unsigned long)ftl_restore_stats[2]);
    if (rc) goto end;
    filesystem_init();
    rc = disk_mount_all();
    if (rc <= 0) { printf("disk_mount_all %d", rc); goto end; }

    for (int i = 0; i < 4096; i++) wt_pat[i] = "NANO3G-WRITE-TEST-2 "[i % 20];
    nand3g_write_enable = 1;
    int fd = open("/testtone.wav", O_WRONLY);
    if (fd < 0) { printf("open for write failed %d", fd); nand3g_write_enable = 0; goto end; }
    int n = write(fd, wt_pat, 4096);
    rc = close(fd);
    printf("write %d close %d; writes %u erases %u err %u", n, rc,
           nand3g_stat_writes, nand3g_stat_erases, nand3g_stat_write_errors);
    printf("ctrl blocks before: %lx %lx %lx page %lx usn %lx",
           (unsigned long)ftl_dbg_ctrl[0], (unsigned long)ftl_dbg_ctrl[1],
           (unsigned long)ftl_dbg_ctrl[2], (unsigned long)ftl_dbg_ctrl[3], (unsigned long)ftl_dbg_ctrl[4]);
    unsigned w0 = nand3g_stat_writes, e0 = nand3g_stat_erases;
    rc = ftl_sync();
    printf("ftl_sync rc %d: +%u writes +%u erases err %u", rc,
           nand3g_stat_writes - w0, nand3g_stat_erases - e0, nand3g_stat_write_errors);
    printf("ctrl blocks after: %lx %lx %lx page %lx usn %lx vflcommits %lu",
           (unsigned long)ftl_dbg_ctrl[0], (unsigned long)ftl_dbg_ctrl[1],
           (unsigned long)ftl_dbg_ctrl[2], (unsigned long)ftl_dbg_ctrl[3],
           (unsigned long)ftl_dbg_ctrl[4], (unsigned long)ftl_dbg_ctrl[5]);
    if (rc == 0)
    {
        for (int i = 0; i < 4096; i++) wt_pat[i] = "NANO3G-WRITE-TEST-3 "[i % 20];
        fd = open("/testtone.wav", O_WRONLY);
        n = write(fd, wt_pat, 4096);
        rc = close(fd);
        printf("2nd write %d close %d (dirty mark): writes %u erases %u err %u", n, rc,
               nand3g_stat_writes, nand3g_stat_erases, nand3g_stat_write_errors);
    }
    nand3g_write_enable = 0;
    fd = open("/testtone.wav", O_RDONLY);
    n = read(fd, wt_back, 4096);
    close(fd);
    printf("readback: %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", wt_back[0], wt_back[1], wt_back[2],
           wt_back[3], wt_back[4], wt_back[5], wt_back[6], wt_back[7], wt_back[8], wt_back[9],
           wt_back[10], wt_back[11], wt_back[12], wt_back[13], wt_back[14], wt_back[15],
           wt_back[16], wt_back[17], wt_back[18], wt_back[19]);
    {
        extern uint32_t ftl_dbg_verify[6], ftl_dbg_guard[2], ftl_nano3g_remap_ok;
        printf("verify: pages %lu, write fails %lu, verify fails %lu, copy fails %lu, guard %lu, remap %s",
               (unsigned long)ftl_dbg_verify[3], (unsigned long)ftl_dbg_verify[0],
               (unsigned long)ftl_dbg_verify[1], (unsigned long)ftl_dbg_verify[2],
               (unsigned long)ftl_dbg_guard[0], ftl_nano3g_remap_ok ? "ok" : "NOT OK");
        if (ftl_dbg_verify[0] || ftl_dbg_verify[1] || ftl_dbg_verify[2])
            printf(" last bad vpage %lx bank/ppage %lx", (unsigned long)ftl_dbg_verify[4], (unsigned long)ftl_dbg_verify[5]);
    }
end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}

static void ftl_wtest1(void)
{
    extern uint32_t ftl_unclean, ftl_restore_stats[8];
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("FTL write test 1");
    int rc = storage_init();
    printf("storage_init %d, unclean %lu restore rc %lu logs %lu free %lu", rc,
           (unsigned long)ftl_unclean, (unsigned long)ftl_restore_stats[0],
           (unsigned long)ftl_restore_stats[1], (unsigned long)ftl_restore_stats[2]);
    if (rc) goto end;
    filesystem_init();
    rc = disk_mount_all();
    printf("disk_mount_all %d", rc);
    if (rc <= 0) goto end;

    int fd = open("/testtone.wav", O_RDONLY);
    if (fd < 0) { printf("open testtone.wav failed %d", fd); goto end; }
    int n = read(fd, wt_orig, 4096);
    close(fd);
    printf("orig: read %d, head %02x %02x %02x %02x (%c%c%c%c)", n,
           wt_orig[0], wt_orig[1], wt_orig[2], wt_orig[3],
           wt_orig[0], wt_orig[1], wt_orig[2], wt_orig[3]);
    if (n != 4096) goto end;

    for (int i = 0; i < 4096; i++) wt_pat[i] = "NANO3G-WRITE-TEST-1 "[i % 20];
    nand3g_write_enable = 1;
    fd = open("/testtone.wav", O_WRONLY);
    if (fd < 0) { printf("open for write failed %d", fd); nand3g_write_enable = 0; goto end; }
    n = write(fd, wt_pat, 4096);
    rc = close(fd);
    printf("write %d close %d; nand writes %u erases %u errors %u", n, rc,
           nand3g_stat_writes, nand3g_stat_erases, nand3g_stat_write_errors);
    nand3g_write_enable = 0;

    fd = open("/testtone.wav", O_RDONLY);
    n = read(fd, wt_back, 4096);
    close(fd);
    int bad = 0; for (int i = 0; i < 4096; i++) if (wt_back[i] != wt_pat[i]) bad++;
    printf("readback %d bytes, %d differ from pattern: %c%c%c%c%c%c%c%c", n, bad,
           wt_back[0], wt_back[1], wt_back[2], wt_back[3], wt_back[4], wt_back[5], wt_back[6], wt_back[7]);
    {
        extern uint32_t ftl_dbg_verify[6], ftl_dbg_guard[2], ftl_nano3g_remap_ok;
        printf("verify: pages %lu, write fails %lu, verify fails %lu, copy fails %lu, guard %lu, remap %s",
               (unsigned long)ftl_dbg_verify[3], (unsigned long)ftl_dbg_verify[0],
               (unsigned long)ftl_dbg_verify[1], (unsigned long)ftl_dbg_verify[2],
               (unsigned long)ftl_dbg_guard[0], ftl_nano3g_remap_ok ? "ok" : "NOT OK");
        if (ftl_dbg_verify[0] || ftl_dbg_verify[1] || ftl_dbg_verify[2])
            printf(" last bad vpage %lx bank/ppage %lx", (unsigned long)ftl_dbg_verify[4], (unsigned long)ftl_dbg_verify[5]);
    }
    printf("no ftl_sync: FTL context untouched, OF restores on next boot");
end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}

static int wtest_block = -1;
static int wtest_page_empty(int ce, uint32_t page)
{
    uint8_t *buf = nand3g_page_buffer();
    uint32_t sp[3]; uint32_t raw;
    int r = nand3g_read_page(NAND3G_CTRL, ce, page, buf, sp, &raw);
    if (r < 0) return 0;
    if (sp[0] != 0xFFFFFFFF || sp[1] != 0xFFFFFFFF || sp[2] != 0xFFFFFFFF) return 0;
    for (int i = 0; i < 2048; i++) if (buf[i] != 0xFF) return 0;
    return 1;
}
static void nand_wtest_find(void)
{
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("NAND write test 1: scan die 0 (ce 0)");
    wtest_block = -1;
    nand3g_hw_init(0);
    nand3g_reset(0, 0);     /* cold reads misbehave without a chip reset */
    /* DEVICEINFOBBT: block 8191 page 127 on CE0, bitmap at 0x38, 1 = good */
    static uint8_t bbt[0x400];
    memset(bbt, 0xFF, sizeof(bbt));
    {
        uint8_t *buf = nand3g_page_buffer(); uint32_t sp[3]; uint32_t raw;
        int found = -1;
        /* Bank 6 = die 0, plane 1, upper half: physical odd blocks
           4096 + 2*v + 1. Its DEVICEINFOBBT lives in the last 10% of its
           virtual blocks (v >= 1843), any page. Scan from the top. */
        for (int b = 8191; b >= 7373 && found < 0; b--) {   /* last 10%, as the FTL */
            int bad = 0;
            for (int pg = 0; pg < 128 && bad <= 2; pg++) {
                int r = nand3g_read_page(NAND3G_CTRL, 0, b * 128 + pg, buf, sp, &raw);
                if (r < 0 || r == NAND3G_ECC_UNCORRECTABLE) { bad++; continue; }
                if (memcmp(buf, "DEVICEINFOBBT\0\0\0", 0x10) == 0) { found = b * 128 + pg; break; }
            }
        }
        if (found >= 0) {
            uint32_t len = *(uint32_t *)&buf[0x34];
            if (len > sizeof(bbt)) len = sizeof(bbt);
            memcpy(bbt, &buf[0x38], len);
            /* bitmap is indexed by the bank's virtual block: v = (b-4096)/2 */
            int v = 8189;   /* bitmap indexed by physical block on this die */
            printf("BBT at page %d (%lu bytes), 8189 (v%d) marked %s", found, (unsigned long)len, v,
                   (bbt[v >> 3] >> (v & 7)) & 1 ? "good" : "BAD");
        } else
            printf("BBT not found in blocks 7373..8191, assuming all good");
    }
    int nbad = 0;
    for (int b = 8190; b >= 7968; b--) {
        int v = b;
        if (b == 8189) continue;   /* erase failed there (status E1): likely a bad block */
        if (!((bbt[v >> 3] >> (v & 7)) & 1)) { nbad++; continue; }
        uint32_t base = b * 128;
        if (wtest_page_empty(0, base) && wtest_page_empty(0, base + 1) &&
            wtest_page_empty(0, base + 64) && wtest_page_empty(0, base + 127)) {
            wtest_block = b; break;
        }
    }
    printf("bad blocks skipped in range: %d", nbad);
    if (wtest_block < 0) printf("no fully erased good block in 7968..8190");
    else printf("erased good block found: %d (pages %d..%d)", wtest_block, wtest_block*128, wtest_block*128+127);
    printf("nothing was written. Test 2 uses this block.");
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}
static uint8_t wtest_data[2048] STORAGE_ALIGN_ATTR;
static void nand_wtest_run(void)
{
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("NAND write test 2");
    if (wtest_block < 0) { printf("run test 1 first"); goto end; }
    uint32_t page = wtest_block * 128;
    int r;
    nand3g_hw_init(0);
    nand3g_reset(0, 0);
    r = nand3g_erase_block(NAND3G_CTRL, 0, wtest_block);
    printf("erase block %d: rc %d (status %02x)", wtest_block, r, nand3g_dbg_wstat);
    if (r) goto end;
    for (int i = 0; i < 2048; i++) wtest_data[i] = (uint8_t)(i * 7 + 3) ^ 0xA5;
    uint32_t sp[3] = { 0x12345678, 0x9ABCDEF0, 0x0F0F00FF };
    r = nand3g_write_page(NAND3G_CTRL, 0, page, wtest_data, sp);
    printf("program page %lu: rc %d (status %02x)", (unsigned long)page, r, nand3g_dbg_wstat);
    if (r < 0)
        printf(" timeout at wait %u: STAT %08x ECC %08x SPTRIG %x", nand3g_dbg_wstep,
               nand3g_dbg_wfail_stat, nand3g_dbg_wfail_ecc, nand3g_dbg_wfail_sp);
    {
        uint8_t *buf = nand3g_page_buffer(); uint32_t rs[3]; uint32_t raw;
        int rr = nand3g_read_page(NAND3G_CTRL, 0, page, buf, rs, &raw);
        int bad = 0; for (int i = 0; i < 2048; i++) if (buf[i] != wtest_data[i]) bad++;
        printf("readback: ecc rc %d raw %08lx, %d data bytes differ", rr, (unsigned long)raw, bad);
        printf("spare %08lx %08lx %08lx", (unsigned long)rs[0], (unsigned long)rs[1], (unsigned long)rs[2]);
        printf("first bytes %02x %02x %02x %02x (want %02x %02x %02x %02x)",
               buf[0], buf[1], buf[2], buf[3], wtest_data[0], wtest_data[1], wtest_data[2], wtest_data[3]);
    }
    r = nand3g_erase_block(NAND3G_CTRL, 0, wtest_block);
    printf("erase again: rc %d (status %02x)", r, nand3g_dbg_wstat);
    printf("page 0 blank again: %s", wtest_page_empty(0, page) ? "yes" : "NO");
end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}


/* NAND write test 3: the second die (chip-enable 1) was never exercised by
   test 2. Find an erased good block near the top of die 1, erase it,
   program pages 0 and 1 with different patterns, read both back, check
   that the same page numbers on die 0 are untouched, then erase again. */
static void nand_wtest_die1(void)
{
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("NAND write test 3: die 1 (ce 1)");
    uint8_t *buf = nand3g_page_buffer(); uint32_t sp[3]; uint32_t raw;
    nand3g_hw_init(0);
    nand3g_reset(0, 0);
    nand3g_reset(0, 1);
    static uint8_t bbt1[0x400];
    memset(bbt1, 0xFF, sizeof(bbt1));
    int found = -1;
    for (int b = 8191; b >= 7373 && found < 0; b--) {
        int bad = 0;
        for (int pg = 0; pg < 128 && bad <= 2; pg++) {
            int r = nand3g_read_page(NAND3G_CTRL, 1, b * 128 + pg, buf, sp, &raw);
            if (r < 0 || r == NAND3G_ECC_UNCORRECTABLE) { bad++; continue; }
            if (memcmp(buf, "DEVICEINFOBBT\0\0\0", 0x10) == 0) { found = b * 128 + pg; break; }
        }
    }
    if (found >= 0) {
        uint32_t len = *(uint32_t *)&buf[0x34];
        if (len > sizeof(bbt1)) len = sizeof(bbt1);
        memcpy(bbt1, &buf[0x38], len);
        printf("die 1 BBT at page %d (%lu bytes)", found, (unsigned long)len);
    } else printf("die 1 BBT not found, assuming all good");
    int blk = -1, nbad = 0;
    for (int b = 8190; b >= 7968; b--) {
        if (!((bbt1[b >> 3] >> (b & 7)) & 1)) { nbad++; continue; }
        uint32_t base = b * 128;
        if (wtest_page_empty(1, base) && wtest_page_empty(1, base + 1) &&
            wtest_page_empty(1, base + 64) && wtest_page_empty(1, base + 127)) { blk = b; break; }
    }
    printf("bad blocks skipped: %d, erased good block: %d", nbad, blk);
    if (blk < 0) goto end;
    uint32_t page = blk * 128;
    /* snapshot die 0 at the same pages */
    uint32_t d0sp[2][3]; uint8_t d0head[2][16]; int d0rc[2];
    for (int p = 0; p < 2; p++) {
        d0rc[p] = nand3g_read_page(NAND3G_CTRL, 0, page + p, buf, d0sp[p], &raw);
        memcpy(d0head[p], buf, 16);
    }
    int r = nand3g_erase_block(NAND3G_CTRL, 1, blk);
    printf("erase die1 block %d: rc %d (status %02x)", blk, r, nand3g_dbg_wstat);
    if (r) goto end;
    for (int p = 0; p < 2; p++) {
        for (int i = 0; i < 2048; i++) wtest_data[i] = (uint8_t)((i * 7 + 3 + p * 0x33) ^ (p ? 0x5A : 0xA5));
        uint32_t wsp[3] = { 0x11110000u + p, 0x22220000u + p, 0x0F0F00FFu };
        r = nand3g_write_page(NAND3G_CTRL, 1, page + p, wtest_data, wsp);
        printf("program die1 page %lu: rc %d (status %02x)", (unsigned long)(page + p), r, nand3g_dbg_wstat);
        if (r < 0) printf(" timeout at wait %u STAT %08x", nand3g_dbg_wstep, nand3g_dbg_wfail_stat);
        uint32_t rs[3];
        int rr = nand3g_read_page(NAND3G_CTRL, 1, page + p, buf, rs, &raw);
        int bad = 0; for (int i = 0; i < 2048; i++) if (buf[i] != wtest_data[i]) bad++;
        printf(" readback ecc %d, %d bytes differ, sp %08lx %08lx", rr, bad,
               (unsigned long)rs[0], (unsigned long)rs[1]);
    }
    /* die 0 must be untouched */
    for (int p = 0; p < 2; p++) {
        uint32_t nsp[3];
        int rr = nand3g_read_page(NAND3G_CTRL, 0, page + p, buf, nsp, &raw);
        int same = (rr == d0rc[p]) && memcmp(nsp, d0sp[p], 12) == 0 && memcmp(buf, d0head[p], 16) == 0;
        printf("die0 page %lu unchanged: %s", (unsigned long)(page + p), same ? "yes" : "NO!");
    }
    r = nand3g_erase_block(NAND3G_CTRL, 1, blk);
    printf("erase again: rc %d, pages blank: %s %s", r,
           wtest_page_empty(1, page) ? "yes" : "NO", wtest_page_empty(1, page + 1) ? "yes" : "NO");
end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}


/* Read-only: what is in the lowest physical blocks of each die (boot image,
   VFL context, and the FTL's vBlocks 1..2 which the OF maps to lBlocks
   0x2AF / 0xF4) and in the same blocks of the upper half. Prints page 0 and
   page 127: spare word 0 (lpn or usn), word 2 low byte (type). */
static void low_blocks_probe(void)
{
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("Low blocks probe (read-only): ce/pblk: pg0 lpn/type, pg127 lpn/type");
    uint8_t *buf = nand3g_page_buffer(); uint32_t sp[3]; uint32_t raw;
    nand3g_hw_init(0);
    nand3g_reset(0, 0);
    nand3g_reset(0, 1);
    const uint32_t blocks[] = { 0, 1, 2, 3, 4, 5, 6, 4096, 4097, 4098, 4099 };
    for (int ce = 0; ce < 2; ce++) {
        for (unsigned i = 0; i < ARRAYLEN(blocks); i++) {
            uint32_t b = blocks[i];
            int r0 = nand3g_read_page(NAND3G_CTRL, ce, b * 128, buf, sp, &raw);
            uint32_t l0 = sp[0], t0 = (sp[2] >> 8) & 0xFF;
            int r1 = nand3g_read_page(NAND3G_CTRL, ce, b * 128 + 127, buf, sp, &raw);
            printf("%d/%4lu: %d %08lx %02lx | %d %08lx %02lx", ce, (unsigned long)b,
                   r0, (unsigned long)l0, (unsigned long)t0, r1, (unsigned long)sp[0],
                   (unsigned long)((sp[2] >> 8) & 0xFF));
        }
    }
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}


/* Read-only: physical blocks 2..4 of each die are erased although the FTL
   layout puts the lower halves of vBlocks 1 and 2 there. Scan page 0 of
   every block on both dies for the logical pages that should live there
   (lBlock 0x2AF pages 0/2 -> vBlock 1 planes 0/1, lBlock 0xF4 page 0 ->
   vBlock 2 plane 0; die 1 holds the odd pages) and print where they are. */
static void find_displaced_probe(void)
{
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("Find displaced pages (read-only), scanning page 0 of all blocks");
    uint8_t *buf = nand3g_page_buffer(); uint32_t sp[3]; uint32_t raw;
    nand3g_hw_init(0);
    nand3g_reset(0, 0);
    nand3g_reset(0, 1);
    const uint32_t want[2][3] = { { 0xABC00, 0xABC02, 0x3D000 }, { 0xABC01, 0xABC03, 0x3D001 } };
    for (int ce = 0; ce < 2; ce++) {
        int hits = 0;
        for (uint32_t b = 0; b < 8192 && hits < 6; b++) {
            int r = nand3g_read_page(NAND3G_CTRL, ce, b * 128, buf, sp, &raw);
            if (r < 0 || r == NAND3G_ECC_UNCORRECTABLE) continue;
            for (int k = 0; k < 3; k++)
                if (sp[0] == want[ce][k]) {
                    uint32_t sp2[3];
                    nand3g_read_page(NAND3G_CTRL, ce, b * 128 + 127, buf, sp2, &raw);
                    printf("ce%d lpn %05lx at pblk %lu (page127 lpn %08lx type %02lx usn %08lx)", ce,
                           (unsigned long)want[ce][k], (unsigned long)b, (unsigned long)sp2[0],
                           (unsigned long)((sp2[2] >> 8) & 0xFF), (unsigned long)sp[1]);
                    hits++;
                }
        }
        if (!hits) printf("ce%d: none of the three found", ce);
    }
    /* and what the VFL context says in its 0x20.. region: first 16 halfwords */
    {
        int r = nand3g_read_page(NAND3G_CTRL, 0, 1 * 128 + 63, buf, sp, &raw);
        uint16_t *h = (uint16_t *)(buf + 0x20);
        printf("VFL cxt (die0 blk1 pg63 rc %d) +0x20: %04x %04x %04x %04x %04x %04x %04x %04x", r,
               h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
        printf(" +0x30: %04x %04x %04x %04x %04x %04x %04x %04x", h[8], h[9], h[10], h[11], h[12], h[13], h[14], h[15]);
        h = (uint16_t *)(buf + 0x10);
        printf(" +0x10: %04x %04x %04x %04x %04x %04x %04x %04x", h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
        h = (uint16_t *)(buf + 0x694);
        printf(" +0x694: %04x %04x %04x %04x %04x %04x %04x %04x", h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
    }
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}


/* Read-only: decode the VFL remapping. Dumps every halfword of the newest
   VFL context page (block 1, last written copy) that is neither 0xFFFF nor
   0xFFF0 in the 0x000..0x6A0 region, locates lBlock 0x722 (whose upper
   half sits in physical 4096/4097 = "vBlock 0"), and lists the bad blocks
   from each die's DEVICEINFOBBT. */
static void vfl_remap_probe(void)
{
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("VFL remap probe (read-only)");
    uint8_t *buf = nand3g_page_buffer(); uint32_t sp[3]; uint32_t raw;
    nand3g_hw_init(0);
    nand3g_reset(0, 0);
    nand3g_reset(0, 1);
    for (int ce = 0; ce < 2; ce++) {
        /* newest cxt copy: highest page in block 1 with type 0x80 */
        int found = -1;
        for (int pg = 127; pg >= 0 && found < 0; pg--) {
            int r = nand3g_read_page(NAND3G_CTRL, ce, 128 + pg, buf, sp, &raw);
            if (r >= 0 && r != NAND3G_ECC_UNCORRECTABLE && ((sp[2] >> 8) & 0xFF) == 0x80) found = pg;
        }
        if (found < 0) { printf("ce%d: no VFL cxt page in block 1", ce); continue; }
        char s[70]; int n = 0, shown = 0;
        n = snprintf(s, sizeof(s), "ce%d pg%d:", ce, found);
        uint16_t *h = (uint16_t *)buf;
        for (int i = 0; i < 0x6A0 / 2 && shown < 40; i++) {
            if (h[i] == 0xFFFF || h[i] == 0xFFF0) continue;
            int k = snprintf(s + n, sizeof(s) - n, " %x=%x", i * 2, h[i]);
            if (n + k >= 60) { printf("%s", s); n = 0; s[0] = 0; k = snprintf(s, sizeof(s), " %x=%x", i * 2, h[i]); }
            n += k; shown++;
        }
        if (n) printf("%s", s);
        printf(" +0x694: %04x %04x %04x %04x sched %04x", h[0x694/2], h[0x696/2], h[0x698/2], h[0x69a/2], h[0x69c/2]);
        /* bad blocks per die */
        int bbt = -1;
        for (int b = 8191; b >= 7373 && bbt < 0; b--)
            for (int pg = 0; pg < 128; pg++) {
                int r = nand3g_read_page(NAND3G_CTRL, ce, b * 128 + pg, buf, sp, &raw);
                if (r < 0 || r == NAND3G_ECC_UNCORRECTABLE) { if (pg > 2) break; continue; }
                if (memcmp(buf, "DEVICEINFOBBT\0\0\0", 0x10) == 0) { bbt = b * 128 + pg; break; }
            }
        if (bbt >= 0) {
            uint32_t len = *(uint32_t *)&buf[0x34]; if (len > 0x400) len = 0x400;
            n = snprintf(s, sizeof(s), " bad blocks (BBT pg %d):", bbt);
            int nb = 0;
            for (uint32_t b = 0; b < len * 8 && b < 8192; b++)
                if (!((buf[0x38 + (b >> 3)] >> (b & 7)) & 1)) {
                    if (nb < 10) n += snprintf(s + n, sizeof(s) - n, " %lu", (unsigned long)b);
                    nb++;
                }
            printf("%s (total %d)", s, nb);
        } else printf(" BBT not found");
    }
    /* where is lBlock 0x722's lower half? lpn 0x1C8800 (die 0 plane 0) */
    for (int ce = 0; ce < 2; ce++) {
        uint32_t want = 0x1C8800 + ce;
        int hits = 0;
        for (uint32_t b = 0; b < 8192 && hits < 2; b++) {
            int r = nand3g_read_page(NAND3G_CTRL, ce, b * 128, buf, sp, &raw);
            if (r < 0 || r == NAND3G_ECC_UNCORRECTABLE) continue;
            if (sp[0] == want || sp[0] == want + 2) {
                printf("ce%d lpn %05lx at pblk %lu (usn %08lx)", ce, (unsigned long)sp[0], (unsigned long)b, (unsigned long)sp[1]);
                hits++;
            }
        }
        if (!hits) printf("ce%d: lpn %05lx/+2 not found in any page 0", ce, (unsigned long)want);
    }
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}


/* Read-only: identify the SPI NOR chip. The NOR driver uses the SST25
   command set (AAI word program 0xAD, 4 KB sector erase 0x20, EWSR/WRSR
   unlock); SST25VF080B answers JEDEC BF 25 8E. Anything else needs a
   look before the dual-boot installer may write. Also reads the IM3
   header at 32 KB and the free space of the flsh area. */
static void nor_chip_id(void)
{
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("NOR chip ID (read-only)");
    uint8_t id[3], st;
    bootflash_init(SPI_PORT);
    bootflash_read_id(SPI_PORT, id, &st);
    bootflash_close(SPI_PORT);
    printf("JEDEC %02X %02X %02X  status %02X  (SST25VF080B = BF 25 8E)", id[0], id[1], id[2], st);
    struct Im3Info hinfo;
    int rc = im3_read(NORBOOT_OFF, &hinfo, NULL);
    printf("IM3 @32K: rc %d enc %u data_sz %lu nor_sz %lu", rc, hinfo.enc_type,
           (unsigned long)(hinfo.data_sz[0] | (hinfo.data_sz[1] << 8) | (hinfo.data_sz[2] << 16) | (hinfo.data_sz[3] << 24)),
           (unsigned long)im3_nor_sz(&hinfo));
    printf("flsh unused: %lu bytes", (unsigned long)flsh_get_unused());
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}


#ifdef NANO3G_FTL_DIAG
/* Read-only: print the write-failure breadcrumb the main firmware leaves at
   the top of DRAM before a dc_writeback panic (see ftl_nano3g_crumb_write). */
static void show_crumb(void)
{
    lcd_clear_display(); lcd_set_foreground(LCD_WHITE); line = 0;
    printf("Write-failure crumb (read-only)");
    uint32_t *c = (uint32_t*)(DRAM_ORIG + DRAM_SIZE - 256);
    if (c[0] != 0x4E33474B) { printf("no crumb (magic %08lx)", (unsigned long)c[0]); goto end; }
    printf("code %ld sector %lu cnt %lu werr %ld alloc %lx", (long)c[1], (unsigned long)c[2],
           (unsigned long)c[3], (long)c[4], (unsigned long)c[5]);
    printf("ctrl %lu %lu %lu free %lu page %lx", (unsigned long)(c[6] & 0xffff), (unsigned long)(c[6] >> 16),
           (unsigned long)(c[7] & 0xffff), (unsigned long)(c[7] >> 16), (unsigned long)c[8]);
    printf("guard hits %lu op %lu pg %lx ce %lu off %lu..%lu word %08lx rc %ld", (unsigned long)c[9],
           (unsigned long)c[10], (unsigned long)c[11], (unsigned long)c[12], (unsigned long)c[13],
           (unsigned long)c[14], (unsigned long)c[15], (long)c[16]);
    printf("watch armed %lu changes %lu first tick %lu region %lu off %lx %08lx->%08lx",
           (unsigned long)c[17], (unsigned long)c[18], (unsigned long)c[19], (unsigned long)c[20],
           (unsigned long)c[21], (unsigned long)c[22], (unsigned long)c[23]);
end:
    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE) sleep(HZ/100);
    while (button_status() != BUTTON_SELECT) sleep(HZ/100);
}
#endif /* NANO3G_FTL_DIAG */

static struct dmac_tsk dma_test_tskbuf[4];
static struct dmac_lli volatile dma_test_llibuf[4] CACHEALIGN_ATTR;
static volatile uint32_t dma_test_cbs;
static int16_t dma_test_tone[4096] STORAGE_ALIGN_ATTR;  /* 8 KiB */
static void dma_test_callback(void *data);
static struct dmac_ch dma_test_ch = {
    .dmac = &s5l8702_dmac0,
    .prio = DMAC_CH_PRIO(2),
    .cb_fn = dma_test_callback,
    .tskbuf = dma_test_tskbuf,
    .tskbuf_mask = 3,
    .queue_mode = QUEUE_LINK,
    .llibuf = dma_test_llibuf,
    .llibuf_mask = 3,
    .llibuf_bus = DMAC_MASTER_AHB1,
};
static struct dmac_ch_cfg dma_test_ch_cfg = {
    .srcperi = S5L8702_DMAC0_PERI_MEM,
    .dstperi = S5L8702_DMAC0_PERI_IIS0_TX,
    .sbsize  = DMACCxCONTROL_BSIZE_8,
    .dbsize  = DMACCxCONTROL_BSIZE_4,
    .swidth  = DMACCxCONTROL_WIDTH_16,
    .dwidth  = DMACCxCONTROL_WIDTH_16,
    .sbus    = DMAC_MASTER_AHB1,
    .dbus    = DMAC_MASTER_AHB1,
    .sinc    = DMACCxCONTROL_INC_ENABLE,
    .dinc    = DMACCxCONTROL_INC_DISABLE,
    .prot    = DMAC_PROT_CACH | DMAC_PROT_BUFF | DMAC_PROT_PRIV,
    .lli_xfer_max_count = DMAC_LLI_MAX_COUNT & ~1,
};
static volatile int dma_test_running;
static void dma_test_callback(void *data)
{
    (void)data;
    dma_test_cbs++;
    if (dma_test_running)
        dmac_ch_queue(&dma_test_ch, dma_test_tone,
                      (void*)S5L8702_DADDR_PERI_IIS0_TX, sizeof(dma_test_tone), NULL);
}

static void dma_play_test(void)
{
    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;
    printf("DMA playback test (I2S0, OF config)");

    /* 1.4 kHz square wave, stereo 16-bit */
    for (int i = 0; i < 4096; i++)
        dma_test_tone[i] = ((i / 2) & 16) ? 0x3000 : -0x3000;
    commit_dcache_range(dma_test_tone, sizeof(dma_test_tone));

    /* MCLK + codec exactly as the OF */
    CLKCON3 = (CLKCON3 & ~0xffff) | 0x8000;
    udelay(100);
    CLKCON3 &= ~0x8000;
    wm_write(0x0F, 0x000);
    sleep(HZ/50);
    wm_write(0x19, 0x0C0);
    sleep(HZ/50);
    wm_write(0x07, 0x042);
    wm_write(0x1A, 0x1E0);
    wm_write(0x22, 0x150);
    wm_write(0x25, 0x120);
    wm_write(0x24, 0x003);
    wm_write(0x43, 0x008);
    wm_write(0x18, 0x100);
    wm_write(0x18, 0x104);
    wm_write(0x1b, 0x040);
    wm_write(0x08, 0x123);
    wm_write(0x05, 0x000);
    wm_write(0x02, 0x179);
    wm_write(0x03, 0x179);

    /* I2S0 as pcm-s5l8702.c sink_dma_init does */
    PWRCON(1) &= ~(1 << 7);
    dmac_ch_init(&dma_test_ch, &dma_test_ch_cfg);
    I2STXCON = 0x0b100001;
    I2SRXCON = 0x1000;
    I2SCLKCON = 1;
    I2SCLKDIV = 12000000 / 44100;
    printf("DMAC0 cfg: INTSTATUS %08lx  I2S st %08lx",
           (unsigned long)*(volatile uint32_t*)0x38200000, (unsigned long)I2SSTATUS);

    dma_test_cbs = 0;
    dma_test_running = 1;
    I2STXCOM = 0xe;
    dmac_ch_queue(&dma_test_ch, dma_test_tone, (void*)S5L8702_DADDR_PERI_IIS0_TX,
                  sizeof(dma_test_tone), NULL);
    dmac_ch_queue(&dma_test_ch, dma_test_tone, (void*)S5L8702_DADDR_PERI_IIS0_TX,
                  sizeof(dma_test_tone), NULL);
    uint32_t s0 = I2SSTATUS;
    sleep(HZ/2);
    uint32_t c1 = dma_test_cbs, s1 = I2SSTATUS;
    sleep(HZ/2);
    uint32_t c2 = dma_test_cbs, s2 = I2SSTATUS;
    dma_test_running = 0;
    sleep(HZ/10);
    dmac_ch_stop(&dma_test_ch);
    I2STXCOM = 0xa;
    printf("callbacks after 0.5 s: %lu, after 1 s: %lu", (unsigned long)c1, (unsigned long)c2);
    printf("I2S status: start %lx, 0.5 s %lx, 1 s %lx", (unsigned long)s0,
           (unsigned long)s1, (unsigned long)s2);
    printf("DMAC0 INTSTATUS %08lx RAWINTTC %08lx ENBLD %08lx",
           (unsigned long)*(volatile uint32_t*)0x38200000,
           (unsigned long)*(volatile uint32_t*)0x38200014,
           (unsigned long)*(volatile uint32_t*)0x3820001c);
    printf("(expect ~20 callbacks/s and an audible tone)");

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE)
        sleep(HZ/100);
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}

/* I2C bus 0 scan: which 7-bit addresses ACK. Then identify the audio
   codec: a Cirrus CS42L55 (0x4A) has a readable chip ID at register 1;
   Wolfson parts (0x1A) are write-only, so an ACK there with no readable
   ID is itself the answer. Read-only apart from the address probes. */
static void i2c_scan(void)
{
    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;
    printf("I2C bus 0 scan (7-bit addresses that ACK):");
    char s[64];
    int n = 0, found = 0;
    for (int a = 0x08; a < 0x78; a++) {
        if (i2c_write(0, a << 1, -1, 0, NULL) == 0) {
            n += snprintf(s + n, sizeof(s) - n, "%02x ", a);
            found++;
            if (n > 40) { printf("%s", s); n = 0; }
        }
    }
    if (n) printf("%s", s);
    printf("%d device(s)", found);

    unsigned char d[8];
    if (i2c_write(0, 0x94, -1, 0, NULL) == 0) {
        int rc = i2c_read(0, 0x94, 0x01, 1, d);
        printf("0x4A: CS42L55 id reg1 = %02x (rc %d)", d[0], rc);
        rc = i2c_read(0, 0x94, 0x02, 4, d);
        printf("0x4A: regs 2..5 = %02x %02x %02x %02x (rc %d)", d[0], d[1], d[2], d[3], rc);
    } else {
        printf("0x4A (CS42L55): no ACK");
    }
    if (i2c_write(0, 0x34, -1, 0, NULL) == 0) {
        int rc = i2c_read(0, 0x34, 0x00, 2, d);
        printf("0x1A: ACK (Wolfson?) read reg0 -> %02x %02x (rc %d)", d[0], d[1], rc);
    } else {
        printf("0x1A (Wolfson): no ACK");
    }
    printf("PMU 0x73: %s", i2c_write(0, 0xe6, -1, 0, NULL) == 0 ? "ACK" : "no ACK");

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE)
        sleep(HZ/100);
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}

extern uint32_t nand3g_stat_reads, nand3g_stat_ecc_flagged,
                nand3g_stat_ecc_bad, nand3g_stat_errors;

/* Hex viewer for one NAND page: 4 screens of 512 bytes, 24 bytes per
   line, SELECT advances. Used to capture the DEVICEINFOBBT page layout. */
static void nand_hexdump_page(int ce, uint32_t page)
{
    uint8_t *buf = nand3g_page_buffer();
    uint32_t spare[NAND3G_SPARE_WORDS], raw;
    nand3g_hw_init(0);
    nand3g_reset(0, ce);
    int rc = nand3g_read_page(0, ce, page, buf, spare, &raw);
    for (int screen = 0; screen < 4; screen++) {
        lcd_clear_display();
        lcd_set_foreground(LCD_WHITE);
        line = 0;
        printf("ce%d pg %lu ecc %d sp %08lx %08lx %08lx  %d/4", ce,
               (unsigned long)page, rc, (unsigned long)spare[0],
               (unsigned long)spare[1], (unsigned long)spare[2], screen + 1);
        for (int l = 0; l < 22; l++) {
            int off = screen * 512 + l * 24;
            if (off >= 2048) break;
            char s[64];
            int n = 0;
            for (int i = 0; i < 24 && off + i < 2048; i++)
                n += snprintf(s + n, sizeof(s) - n, "%02x", buf[off + i]);
            printf("%s", s);
        }
        lcd_set_foreground(LCD_RBYELLOW);
        printf("SELECT: next");
        while (button_status() != BUTTON_NONE)
            sleep(HZ/100);
        while (button_status() != BUTTON_SELECT)
            sleep(HZ/100);
    }
}

static void nand_hexdump_devinfo(void)
{
    /* block 8191, page 127 on CE0: the newest DEVICEINFOBBT copy */
    nand_hexdump_page(0, 8191 * 128 + 127);
}

static void nand_hexdump_ce1_vfl(void)
{
    /* CE0 page 152: newest VFL context copy in block 1 (all 2048 bytes) */
    nand_hexdump_page(0, 152);
}

/* Survey of the FTL control blocks (0x3EF, 0x520, 0x660 on this unit):
   spare words and first bytes of a few pages on both dies, to learn the
   nano 3G's page type numbering (nano 2G: 0x43 cxt, 0x44 map, 0x46 erase
   counters, 0x47 dirty marker). */
static void nand_ftlblock_survey(void)
{
    uint8_t *buf = nand3g_page_buffer();
    uint32_t spare[NAND3G_SPARE_WORDS], raw;

    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;
    printf("Block map: page 0 spare type of every block, both dies");
    nand3g_hw_init(0);
    nand3g_reset(0, 0);
    nand3g_reset(0, 1);
    /* tally per type: 0x40, 0x41, 0x43, 0x44, 0x45, 0x46, 0x47, 0x80,
       other, erased, read error */
    uint32_t tally[11] = {0};
    char list[6][64];
    int nlist = 0, listlen = 0;
    memset(list, 0, sizeof(list));
    for (int ce = 0; ce < 2; ce++)
        for (uint32_t blk = 0; blk < 8192; blk++) {
            int rc = nand3g_read_page(0, ce, blk * 128, buf, spare, &raw);
            uint8_t type = (spare[2] >> 8) & 0xFF;
            if (rc < 0) { tally[10]++; continue; }
            if (spare[0] == 0xFFFFFFFF && spare[2] == 0xFFFFFFFF) { tally[9]++; continue; }
            switch (type) {
                case 0x40: tally[0]++; break;
                case 0x41: tally[1]++; break;
                case 0x43: tally[2]++; break;
                case 0x44: tally[3]++; break;
                case 0x45: tally[4]++; break;
                case 0x46: tally[5]++; break;
                case 0x47: tally[6]++; break;
                case 0x80: tally[7]++; break;
                default:   tally[8]++; break;
            }
            /* record control-type blocks (not 0x40/0x41/0x80) */
            if (type != 0x40 && type != 0x41 && type != 0x80 && nlist < 6) {
                int n = snprintf(list[nlist] + listlen, 64 - listlen, "%d/%lx:%02x ",
                                 ce, (unsigned long)blk, type);
                listlen += n;
                if (listlen > 44) { nlist++; listlen = 0; }
            }
        }
    printf("40:%lu 41:%lu 43:%lu 44:%lu 45:%lu 46:%lu", (unsigned long)tally[0],
           (unsigned long)tally[1], (unsigned long)tally[2], (unsigned long)tally[3],
           (unsigned long)tally[4], (unsigned long)tally[5]);
    printf("47:%lu 80:%lu other:%lu erased:%lu err:%lu", (unsigned long)tally[6],
           (unsigned long)tally[7], (unsigned long)tally[8], (unsigned long)tally[9],
           (unsigned long)tally[10]);
    printf("control-type blocks (ce/blk:type):");
    for (int i = 0; i < 6; i++)
        if (list[i][0]) printf("%s", list[i]);

    /* Bank order probe: vblock 0x330 maps to physical 0x660/0x661 and
       0x1660/0x1661 on each die. The logical page number (spare word 0)
       of page 0 in each tells the interleave order. */
    printf("lpn of page 0/1 in sibling blocks (ce/blk: lpn0 lpn1):");
    const uint32_t sib[] = { 0x660, 0x661, 0x1660, 0x1661 };
    for (int ce = 0; ce < 2; ce++) {
        char s[64]; int n = 0;
        for (unsigned i = 0; i < ARRAYLEN(sib); i++) {
            uint32_t l0, l1;
            nand3g_read_page(0, ce, sib[i] * 128, buf, spare, &raw); l0 = spare[0];
            nand3g_read_page(0, ce, sib[i] * 128 + 1, buf, spare, &raw); l1 = spare[0];
            n += snprintf(s + n, sizeof(s) - n, "%lx:%lx/%lx ", (unsigned long)sib[i],
                          (unsigned long)l0, (unsigned long)l1);
        }
        printf("ce%d %s", ce, s);
    }
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE)
        sleep(HZ/100);
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}

/* Address-cycle / aliasing test and DEVICEINFOSIGN scan, read-only.
   Reads page 128 and page 128+65536 with 4 and 5 address cycles: if the
   two pages are identical with 4 cycles, high pages alias. Then scans for
   the device-info signature with 5 cycles. */
static void nand_addr_test(void)
{
    uint8_t *buf = nand3g_page_buffer();
    uint32_t spare[NAND3G_SPARE_WORDS], raw;
    int rc;

    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;
    printf("Address cycle test");
    nand3g_hw_init(0);
    nand3g_reset(0, 0);

    const uint32_t pages[] = { 128, 128 + 65536, 128 + 2 * 65536 };
    for (uint32_t anum = 4; anum <= 5; anum++) {
        nand3g_anum = anum;
        for (unsigned i = 0; i < ARRAYLEN(pages); i++) {
            memset(buf, 0xAA, 16);
            rc = nand3g_read_page(0, 0, pages[i], buf, spare, &raw);
            printf("a%lu pg %lu: ecc %d sp %08lx %02x%02x%02x%02x %02x%02x%02x%02x",
                   (unsigned long)anum, (unsigned long)pages[i], rc,
                   (unsigned long)spare[0], buf[0], buf[1], buf[2], buf[3],
                   buf[4], buf[5], buf[6], buf[7]);
        }
    }

    /* Signature scan: any page starting with "DEVICEINFO" (nano 2G uses
       "DEVICEINFOSIGN", the iPhone-era Whimory uses "DEVICEINFOBBT"),
       every page of the last 10% of blocks, chip-enables 0 and 1. Stops
       after the first few hits per CE. Also counts erased pages so we
       learn how the tail of the flash is used. */
    uint32_t found = 0, hits = 0, scanned = 0;
    for (int ce = 0; ce < 2; ce++) {
        uint32_t cehits = 0, erased = 0;
        for (uint32_t blk = 8191; blk >= 8192 - 819 && cehits < 2; blk--) {
            for (uint32_t p = 0; p < 128; p++) {
                uint32_t page = blk * 128 + p;
                scanned++;
                rc = nand3g_read_page(0, ce, page, buf, spare, &raw);
                if (rc < 0) continue;
                if (spare[0] == 0xFFFFFFFF && buf[0] == 0xFF && buf[1] == 0xFF) {
                    erased++;
                    continue;
                }
                if (memcmp(buf, "DEVICEINFO", 10) == 0) {
                    printf("ce%d blk %lu pg %lu: %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
                           ce, (unsigned long)blk, (unsigned long)p,
                           buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
                           buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
                    printf("  sp %08lx %08lx %08lx", (unsigned long)spare[0],
                           (unsigned long)spare[1], (unsigned long)spare[2]);
                    if (!found) found = page;
                    hits++; cehits++;
                }
            }
        }
        printf("ce%d: %lu hits, %lu erased pages in scan", ce,
               (unsigned long)cehits, (unsigned long)erased);
    }
    printf("scanned %lu pages, %lu hits, first %lu", (unsigned long)scanned,
           (unsigned long)hits, (unsigned long)found);

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE)
        sleep(HZ/100);
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}

/* Read-only FTL mount test: nand_init() runs ftl_init() (device info scan,
   VFL and FTL context load), then sector 0 (MBR) and the first sector of
   partition 1 are read through ftl_read(). Nothing is written. */
static void ftl_mount_test(void)
{
    static uint8_t sec[SECTOR_SIZE] STORAGE_ALIGN_ATTR;

    lcd_clear_display();
    lcd_set_foreground(LCD_WHITE);
    line = 0;
    printf("FTL mount test (read-only)");

    extern uint32_t ftl_dbg[12];
    long t0 = current_tick;
    int rc = nand_init();
    long dt = current_tick - t0;
    printf("nand_init/ftl_init: %d  (%ld ms)", rc, dt * (1000 / HZ));
    printf("dbg step %lu: %lx %lx %lx %lx %lx", (unsigned long)ftl_dbg[0],
           (unsigned long)ftl_dbg[1], (unsigned long)ftl_dbg[2],
           (unsigned long)ftl_dbg[3], (unsigned long)ftl_dbg[4],
           (unsigned long)ftl_dbg[5]);
    {
        extern uint32_t ftl_unclean, ftl_restore_stats[8];
        printf("unclean %lu restore rc %lu logs %lu free %lu mapdiff %lu",
               (unsigned long)ftl_unclean, (unsigned long)ftl_restore_stats[0],
               (unsigned long)ftl_restore_stats[1], (unsigned long)ftl_restore_stats[2],
               (unsigned long)ftl_restore_stats[3]);
        {
            extern uint32_t ftl_restore_dbg[8];
            printf(" OF pool: free %lu idx %lu [%lx %lx %lx %lx .. %lx %lx]",
                   (unsigned long)(ftl_restore_dbg[0] & 0xffff), (unsigned long)(ftl_restore_dbg[0] >> 16),
                   (unsigned long)(ftl_restore_dbg[1] & 0xffff), (unsigned long)(ftl_restore_dbg[1] >> 16),
                   (unsigned long)(ftl_restore_dbg[2] & 0xffff), (unsigned long)(ftl_restore_dbg[2] >> 16),
                   (unsigned long)(ftl_restore_dbg[3] & 0xffff), (unsigned long)(ftl_restore_dbg[3] >> 16));
            printf(" found free: %lx %lx %lx %lx .. overflow at %lx",
                   (unsigned long)ftl_restore_dbg[4], (unsigned long)ftl_restore_dbg[5],
                   (unsigned long)ftl_restore_dbg[6], (unsigned long)ftl_restore_dbg[7],
                   (unsigned long)ftl_restore_stats[7]);
        }
        {
            extern uint32_t ftl_restore_diff[8];
            printf(" extra free (blk|map<<16|e<<12): %lx %lx %lx", (unsigned long)ftl_restore_diff[1],
                   (unsigned long)ftl_restore_diff[2], (unsigned long)ftl_restore_diff[3]);
            printf(" OF-pool not free: %lx %lx %lx", (unsigned long)ftl_restore_diff[4],
                   (unsigned long)ftl_restore_diff[5], (unsigned long)ftl_restore_diff[6]);
        }
        printf(" highest usn %lu blocks %lu empty %lu writes %u erases %u",
               (unsigned long)ftl_restore_stats[4], (unsigned long)ftl_restore_stats[5],
               (unsigned long)ftl_restore_stats[6], nand3g_stat_writes, nand3g_stat_erases);
    }
    printf(" %lx %lx %lx %lx %lx %lx", (unsigned long)ftl_dbg[6],
           (unsigned long)ftl_dbg[7], (unsigned long)ftl_dbg[8],
           (unsigned long)ftl_dbg[9], (unsigned long)ftl_dbg[10],
           (unsigned long)ftl_dbg[11]);
    printf("reads %lu eccflag %lu eccbad %lu err %lu",
           (unsigned long)nand3g_stat_reads, (unsigned long)nand3g_stat_ecc_flagged,
           (unsigned long)nand3g_stat_ecc_bad, (unsigned long)nand3g_stat_errors);
    if (rc == 0) {
        printf("banks %lu blocks %u user %u ppb %u",
               (unsigned long)ftl_banks, ftl_nand_type->blocks,
               ftl_nand_type->userblocks, ftl_nand_type->pagesperblock);
        rc = ftl_read(0, 1, sec);
        {
            extern void ftl_nano3g_space_probe(uint32_t *out);
            uint32_t o[18]; ftl_nano3g_space_probe(o);
            printf("remap check %s: 3920/1/2 pg0 lpn %lx %lx %lx", o[14] ? "OK" : "FAILED",
                   (unsigned long)o[15], (unsigned long)o[16], (unsigned long)o[17]);
            printf("space: map %lu..%lu pool %lu..%lu ctrl %lu %lu %lu hi %lu",
                   (unsigned long)o[0], (unsigned long)o[1], (unsigned long)o[4], (unsigned long)o[5],
                   (unsigned long)o[6], (unsigned long)o[7], (unsigned long)o[8], (unsigned long)o[13]);
            printf(" vb1 -> %lx vb2 -> %lx vb1959 -> %lx above %lu (FFFF none, FFFx pool, FFCx ctrl)",
                   (unsigned long)o[2], (unsigned long)o[3], (unsigned long)o[12], (unsigned long)o[9]);
        }
        printf("sector 0: rc %d sig %02x%02x", rc, sec[510], sec[511]);
        printf(" %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
               sec[0], sec[1], sec[2], sec[3], sec[4], sec[5], sec[6], sec[7],
               sec[8], sec[9], sec[10], sec[11], sec[12], sec[13], sec[14], sec[15]);
        /* partition table entries at 0x1BE: type at +4, LBA start at +8 */
        for (int p = 0; p < 2; p++) {
            const uint8_t *e = &sec[0x1BE + 16 * p];
            uint32_t lba = e[8] | (e[9] << 8) | (e[10] << 16) | ((uint32_t)e[11] << 24);
            uint32_t len = e[12] | (e[13] << 8) | (e[14] << 16) | ((uint32_t)e[15] << 24);
            printf("part%d type %02x lba %lu len %lu", p, e[4],
                   (unsigned long)lba, (unsigned long)len);
            if ((e[4] == 0x0B || e[4] == 0x0C) && lba != 0) {
                /* The table's unit is not known yet (2048 or 4096 bytes):
                   try the LBA as a 2048-byte FTL sector and doubled. */
                for (int mult = 1; mult <= 2; mult++) {
                    rc = ftl_read(lba * mult, 1, sec);
                    printf(" x%d: rc %d %02x%02x%02x oem %c%c%c%c%c%c%c%c fs %c%c%c%c%c",
                           mult, rc, sec[0], sec[1], sec[2], sec[3], sec[4], sec[5],
                           sec[6], sec[7], sec[8], sec[9], sec[10],
                           sec[82], sec[83], sec[84], sec[85], sec[86]);
                    printf("   bps %u spc %u sig %02x%02x",
                           sec[11] | (sec[12] << 8), sec[13], sec[510], sec[511]);
                }
            }
        }
    }
    printf("reads %lu eccflag %lu eccbad %lu err %lu",
           (unsigned long)nand3g_stat_reads, (unsigned long)nand3g_stat_ecc_flagged,
           (unsigned long)nand3g_stat_ecc_bad, (unsigned long)nand3g_stat_errors);

    line++;
    lcd_set_foreground(LCD_RBYELLOW);
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_NONE)
        sleep(HZ/100);
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);
}
#endif /* IPOD_NANO3G */

static void devel_menu(void)
{
    const char *items[] = {
#ifdef IPOD_NANO3G
#ifdef NANO3G_FTL_DIAG
        "Show write-failure crumb (read-only)",
#endif
        "Mark unclean in current ctrl block (no restore)",
        "VFL state dump (read-only)",
        "Low blocks probe (read-only)",
        "Find displaced pages (read-only)",
        "VFL remap probe (read-only)",
        "NOR chip ID (read-only)",
        "Erase vblock 631 (garbage copy of lblock 0)",
        "FTL recover: force restore + mark unclean (0x4F)",
        "FTL write test 2: write + ftl_sync + write (dirty)",
        "FTL write test 1: rewrite testtone.wav head (no sync)",
        "NAND write test 1: find erased block (no write)",
        "NAND write test 2: ERASE+PROGRAM that block",
        "NAND write test 3: die 1 ERASE+PROGRAM+verify",
        "DMA playback test (tone via DMA)",
        "I2S TX test (3 instances)",
        "I2C scan / codec identify",
        "Block type map (page 0 of every block)",
        "Hexdump DEVICEINFOBBT page",
        "Hexdump VFL cxt page 152",
        "NAND address cycle test",
        "FTL mount test (read-only)",
        "NAND probe (ctrl 0)",
        "NAND probe (ctrl 1)",
#endif
#ifdef HAVE_LCD_SLEEP
        "LCD sleep/awake test",
#endif
        "PMU info",
        "GPIO info",
#if defined(IPOD_6G) || defined(IPOD_NANO3G)
        "Show SysCfg",
        "Show bootloader hash",
#ifdef HAVE_SERIAL
        "Dump bootflash to UART",
#endif
#endif
        "Launch OF",
        //"Launch Rockbox",
        "Restart",
        "Power off",
    };
    void (*handlers[])(void) = {
#ifdef IPOD_NANO3G
#ifdef NANO3G_FTL_DIAG
        show_crumb,
#endif
        mark_unclean_inplace,
        vfl_dump,
        low_blocks_probe,
        find_displaced_probe,
        vfl_remap_probe,
        nor_chip_id,
        erase_631,
        ftl_recover,
        ftl_wtest2,
        ftl_wtest1,
        nand_wtest_find,
        nand_wtest_run,
        nand_wtest_die1,
        dma_play_test,
        i2s_tx_test,
        i2c_scan,
        nand_ftlblock_survey,
        nand_hexdump_devinfo,
        nand_hexdump_ce1_vfl,
        nand_addr_test,
        ftl_mount_test,
        nand_probe_ctrl0,
        nand_probe_ctrl1,
#endif
#ifdef HAVE_LCD_SLEEP
        sleep_test,
#endif
        pmu_info,
        gpio_info,
#if defined(IPOD_6G) || defined(IPOD_NANO3G)
        print_syscfg,
        print_bootloader_hash,
#ifdef HAVE_SERIAL
        dump_bootflash,
#endif
#endif
        run_of,
        //run_rockbox,
        system_reboot,
        power_off,
    };
    const size_t items_count = sizeof(items) / sizeof(items[0]);
    unsigned char selected_item = 0;

    while (1)
    {
        lcd_clear_display();
        lcd_set_foreground(LCD_RBYELLOW);
        line = 0;
        printf("Development menu");

        for (size_t i = 0; i < items_count; i++) {
            lcd_set_foreground(i == selected_item ? LCD_GREEN : LCD_WHITE);
            printf(items[i]);
        }

        while (button_status() != BUTTON_NONE);

        bool done = false;
        while (!done)
        {
            switch (button_status())
            {
                case BUTTON_MENU:
                case BUTTON_LEFT:
                    if (selected_item > 0) {
                        selected_item--;
                        done = true;
                    }
                    else {
                        sleep(HZ/100);
                    }
                    break;
                case BUTTON_PLAY:
                case BUTTON_RIGHT:
                    if (selected_item < items_count - 1) {
                        selected_item++;
                        done = true;
                    }
                    else {
                        sleep(HZ/100);
                    }
                    break;
                case BUTTON_SELECT:
                    handlers[selected_item]();
                    done = true;
                    break;
                default:
                    sleep(HZ/100);
                    break;
            }
        }
    }
}
#endif /* S5L87XX_DEVELOPMENT_BOOTLOADER */

void main(void)
{
    int rc = 0;

    usec_timer_init();

#ifdef S5L87XX_DEVELOPMENT_BOOTLOADER
    piezo_seq(alive);
#endif

    /* Configure I2C0 */
    i2c_preinit(0);

    if (pmu_is_hibernated()) {
        rc = launch_onb(1); /* 27/2 = 13.5 MHz. */
    }

    system_preinit();
    memory_init();
    /*
     * XXX: BSS is initialized here, do not use .bss before this line
     */
    bss_init();

    system_init();
    kernel_init();
    i2c_init();
    power_init();

    enable_irq();

#ifdef HAVE_SERIAL
    serial_setup();
#endif

    button_init();
    if (rc == 0) {
        /* User button selection timeout */
        while (USEC_TIMER < 400000);
        int btn = button_read_device();
        /* This prevents HDD spin-up when the user enters DFU */
        if (btn == (BUTTON_SELECT|BUTTON_MENU)) {
            while (button_read_device() == (BUTTON_SELECT|BUTTON_MENU))
                sleep(HZ/10);
            sleep(HZ);
            btn = button_read_device();
        }
        /* Enter OF, diagmode and diskmode using ONB */
        if ((btn == BUTTON_MENU)
                || (btn == (BUTTON_SELECT|BUTTON_LEFT))
                || (btn == (BUTTON_SELECT|BUTTON_PLAY))) {
            rc = kernel_launch_onb();
        }
    }

    lcd_init();
    lcd_set_foreground(LCD_WHITE);
    lcd_set_background(LCD_BLACK);
    lcd_clear_display();
    font_init();
    lcd_setfont(FONT_SYSFIXED);

    // TODO: see if removing this causes the nano3g LCD to initialize properly
#ifdef S5L87XX_DEVELOPMENT_BOOTLOADER
    sleep(HZ);
    for (int i = 0; i < lcd_type+1; i++) {
        sleep(HZ/2);
        piezo_seq(alivelcd);
    }
#endif

    lcd_update();
    sleep(HZ/40);  /* wait for lcd update */

    verbose = true;

    printf("Rockbox boot loader");
    printf("Version: %s", rbversion);

    backlight_init(); /* Turns on the backlight */

#ifdef S5L87XX_DEVELOPMENT_BOOTLOADER
    line++;
    printf("lcd type: %d", lcd_type);
#ifdef S5L_LCD_WITH_READID
    extern unsigned char lcd_id[4];
    uint32_t* lcd_id_32 = (uint32_t *)lcd_id;
    printf("lcd id: 0x%x", *lcd_id_32);
#endif
#ifdef IPOD_NANO4G
    printf("boot cfg: 0x%x", pmu_read(0x7f));
#endif
    line++;
    printf("Press SELECT to continue");
    while (button_status() != BUTTON_SELECT)
        sleep(HZ/100);

    devel_menu();
#endif /* S5L87XX_DEVELOPMENT_BOOTLOADER */

#ifndef S5L87XX_DEVELOPMENT_BOOTLOADER
    if (rc == 0) {
#if (CONFIG_STORAGE & STORAGE_ATA)
        /* Wait until there is enought power to spin-up HDD */
        battery_trap();
#endif

        rc = storage_init();
        if (rc != 0) {
            printf("Storage error: %d", rc);
            fatal_error(ERR_STORAGE);
        }

        filesystem_init();

        /* We wait until HDD spins up to check for hold button */
#ifdef IPOD_NANO3G
        /* the hold switch state was sampled at power_init(), long before
           the user could move it after the reset: sample it again now, and
           also accept a held MENU as the request to boot the OF */
        pmu_refresh_inputs();
        if (pmu_holdswitch_locked() || (button_read_device() & BUTTON_MENU)) {
#else
        if (button_hold()) {
#endif
#ifdef SYSCFG_MAX_ENTRIES
            bool lba48 = false;
            struct SysCfg syscfg;
            const ssize_t result = syscfg_read(&syscfg);
            if (result != -1) {
                const size_t syscfg_num_entries = MIN(syscfg.header.num_entries, SYSCFG_MAX_ENTRIES);
                for (size_t i = 0; i < syscfg_num_entries; i++) {
                    const struct SysCfgEntry* entry = &syscfg.entries[i];
                    const uint32_t* data32 = (uint32_t *)entry->data;
                    if (entry->tag == SYSCFG_TAG_HWVR) {
                        lba48 = (data32[1] >= 0x130200);
                        break;
                    }
                }

                int btn = button_read_device();

                struct storage_info sinfo;
                storage_get_info(0, &sinfo);
                if (sinfo.num_sectors < (1 << 28) || lba48 || btn & BUTTON_LEFT) {
                    printf("Executing OF...");
#if (CONFIG_STORAGE & STORAGE_ATA)
                    ata_sleepnow();
#endif
                    rc = kernel_launch_onb();
                } else {
                    printf("OF does not support LBA48");
                    fatal_error(ERR_LBA28);
                }
            }
#else
            printf("Executing OF...");
#if (CONFIG_STORAGE & STORAGE_ATA)
            ata_sleepnow();
#endif
            rc = kernel_launch_onb();
#endif /* SYSCFG_MAX_ENTRIES */
        }
    }

    if (rc != 0) {
        printf("Load OF error: %d", rc);
        fatal_error(ERR_OF);
    }

#ifdef HAVE_BOOTLOADER_USB_MODE
    /* Enter USB mode if SELECT+RIGHT are pressed */
    if (button_read_device() == (BUTTON_SELECT|BUTTON_RIGHT)) {
#if defined(MAX_VIRT_SECTOR_SIZE) && defined(DEFAULT_VIRT_SECTOR_SIZE)
#ifdef HAVE_MULTIDRIVE
            for (int i = 0 ; i < NUM_DRIVES ; i++)
#endif
                disk_set_sector_multiplier(IF_MD(i,) DEFAULT_VIRT_SECTOR_SIZE/SECTOR_SIZE);
#endif
        usb_mode();
    }
#endif

    rc = disk_mount_all();
    if (rc <= 0) {
#ifdef STORAGE_GET_INFO
        struct storage_info sinfo;
        storage_get_info(0, &sinfo);
#ifdef MAX_PHYS_SECTOR_SIZE
        printf("id: '%s' s:%u*%u", sinfo.product, sinfo.sector_size, sinfo.phys_sector_mult);
#else
        printf("id: '%s' s:%u", sinfo.product, sinfo.sector_size);
#endif
#endif
        struct partinfo pinfo;
        printf("No partition found");
        for (int i = 0 ; i < NUM_VOLUMES ; i++) {
            disk_partinfo(i, &pinfo);
            if (pinfo.type)
                printf("P%d T%02x S%llx",
                       i, pinfo.type, (unsigned long long)pinfo.size);
        }
        fatal_error(ERR_RB);
    }

    printf("Loading Rockbox...");
    unsigned char *loadbuffer = (unsigned char *)DRAM_ORIG;
    rc = load_firmware(loadbuffer, BOOTFILE, MAX_LOADSIZE);

    if (rc <= EFILE_EMPTY) {
        printf("Error!");
        printf("Can't load " BOOTFILE ": ");
        printf(loader_strerror(rc));
        fatal_error(ERR_RB);
    }

    printf("Rockbox loaded.");

    /* If we get here, we have a new firmware image at 0x08000000, run it */
    disable_irq();

    int (*kernel_entry)(void) = (void*)loadbuffer;
    commit_discard_idcache();
    rc = kernel_entry();

    /* End stop - should not get here */
    enable_irq();
    printf("ERR: Failed to boot");
    while(1);
#endif
}
