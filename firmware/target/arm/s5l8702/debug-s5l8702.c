/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright © 2008 Rafaël Carré
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

#include <stdio.h>
#include <stdbool.h>
#include "system.h"
#include "config.h"
#include "usb-designware.h"
#include "kernel.h"
#include "button.h"
#include "lcd.h"
#include "font.h"
#include "storage.h"
#include "power.h"
#include "adc.h"
#include "pmu-target.h"
#include "pcm-target.h"
#ifdef HAVE_SERIAL
#include "uart-target.h"
#include "uc87xx.h"
#endif
#ifdef HAVE_MIKEY_REMOTE
#include "mikey-target.h"
#endif
#include "clocking-s5l8702.h"

#define DEBUG_CANCEL BUTTON_MENU

/*  Skeleton for adding target specific debug info to the debug menu
 */

#define _DEBUG_PRINTF(a, varargs...) lcd_putsf(0, line++, (a), ##varargs);

extern int lcd_type;
extern int rec_hw_ver;

bool dbg_hw_info(void)
{
    int line;
    int i;
    unsigned int state = 0;
#ifdef UC87XX_DEBUG
    const unsigned int max_states=3;
#elif defined(IPOD_NANO3G)
    const unsigned int max_states=5;
#else
    const unsigned int max_states=2;
#endif

    lcd_clear_display();
    lcd_setfont(FONT_SYSFIXED);

    state=0;
    while(1)
    {
        lcd_clear_display();
        line = 0;

        if(state == 0)
        {
            unsigned cpu_hz;
            get_system_freqs(&cpu_hz, NULL, NULL);
            _DEBUG_PRINTF("CPU:");
            _DEBUG_PRINTF("speed: %d MHz", cpu_hz / 1000000);
            _DEBUG_PRINTF("current_tick: %d", (unsigned int)current_tick);
            line++;
            _DEBUG_PRINTF("LCD type: %d", lcd_type);
            line++;
            _DEBUG_PRINTF("capture HW type: %d", rec_hw_ver);
            line++;
#ifdef CLOCKING_DEBUG
            /* show all clocks */
            unsigned f_clk, c_clk, h_clk, p_clk, l_clk, s_clk;

            f_clk = get_system_freqs(&c_clk, &h_clk, &p_clk);
            s_clk = h_clk / soc_get_hsdiv();
            l_clk = h_clk >> ((LCD_CON & 7) + 1); /* div = 2^(val+1) */

            #define MHZ 1000000
            #define TMHZ 100000
            _DEBUG_PRINTF("Clocks (MHz):");
            _DEBUG_PRINTF("FClk: %d.%d",  f_clk / MHZ, (f_clk % MHZ) / TMHZ);
            _DEBUG_PRINTF(" CPU: %d.%d",  c_clk / MHZ, (c_clk % MHZ) / TMHZ);
            _DEBUG_PRINTF(" AHB: %d.%d",  h_clk / MHZ, (h_clk % MHZ) / TMHZ);
            _DEBUG_PRINTF("  SM1: %d.%d", s_clk / MHZ, (s_clk % MHZ) / TMHZ);
            _DEBUG_PRINTF("  LCD: %d.%d", l_clk / MHZ, (l_clk % MHZ) / TMHZ);
            _DEBUG_PRINTF(" APB: %d.%d",  p_clk / MHZ, (p_clk % MHZ) / TMHZ);
#endif
        }
        else if(state==1)
        {
            _DEBUG_PRINTF("PMU:");
            for(i=0;i<7;i++)
            {
                static const char *const device[] = {"unknown",
                                  "unknown",
                                  "LCD",
                                  "AUDIO",
                                  "unknown",
                                  "CLICKWHEEL",
                                  "ACCESSORY"};
                _DEBUG_PRINTF("ldo%d %s: %dmV (%s)",i,
                    pmu_read(0x2e + (i << 1))?" on":"off",
                    900 + pmu_read(0x2d + (i << 1))*100,
                    device[i]);
            }
            _DEBUG_PRINTF("cpu voltage: %dmV",625 + pmu_read(0x1e)*25);
            _DEBUG_PRINTF("memory voltage: %dmV",625 + pmu_read(0x22)*25);
            line++;
            _DEBUG_PRINTF("charging: %s", charging_state() ? "true" : "false");
            _DEBUG_PRINTF("backlight: %s", pmu_read(0x29) ? "on" : "off");
            _DEBUG_PRINTF("brightness value: %d", pmu_read(0x28));
            line++;
            _DEBUG_PRINTF("USB present: %s",
                    (power_input_status() & POWER_INPUT_USB) ? "true" : "false");
#if CONFIG_CHARGING
            _DEBUG_PRINTF("FW present: %s",
                            pmu_firewire_present() ? "true" : "false");
#endif
            _DEBUG_PRINTF("holdswitch locked: %s",
                            pmu_holdswitch_locked() ? "true" : "false");
#ifdef IPOD_ACCESSORY_PROTOCOL
            _DEBUG_PRINTF("accessory present: %s",
                            pmu_accessory_present() ? "true" : "false");
#endif
#ifdef HAVE_MIKEY_REMOTE
            /* r4/r5 are live register reads to help characterize the
             * remote-ID behavior across units (it varies; see
             * mikey-6g.c). r4 bit6 = ID bit, r5 = event register. */
            _DEBUG_PRINTF("mikey remote ctrl: %s r4=%02x r5=%02x",
                            mikey_present() ? "ok" : "--",
                            mikey_read(4), mikey_read(5));
#endif
            line++;
            _DEBUG_PRINTF("ADC:");
            _DEBUG_PRINTF("%s: %d mV", adc_name(ADC_BATTERY),
                            adc_read_battery_voltage());
            _DEBUG_PRINTF("%s: %d Ohms", adc_name(ADC_ACCESSORY),
                            adc_read_accessory_resistor());
            _DEBUG_PRINTF("USB D+: %d mV", adc_read_usbdata_voltage(true));
            _DEBUG_PRINTF("USB D-: %d mV", adc_read_usbdata_voltage(false));
#ifdef IPOD_NANO3G
            {
                extern unsigned short pmu_read_adc_raw(int mux, int bits8);
                _DEBUG_PRINTF("ADC raw m0 %d m1 %d m2 %d m3 %d",
                    pmu_read_adc_raw(0,0), pmu_read_adc_raw(1,0),
                    pmu_read_adc_raw(2,0), pmu_read_adc_raw(3,0));
                _DEBUG_PRINTF("m4 %d m5 %d m6 %d m7 %d m13/8b %d",
                    pmu_read_adc_raw(4,0), pmu_read_adc_raw(5,0),
                    pmu_read_adc_raw(6,0), pmu_read_adc_raw(7,0),
                    pmu_read_adc_raw(0x13,1));
                {
                    extern void rtc_nano3g_debug(int*, long*, long*, long*, long*);
                    int ld; long od, os, cd, cs;
                    unsigned char rr[6];
                    rtc_nano3g_debug(&ld, &od, &os, &cd, &cs);
                    pmu_read_multiple(0x40, 6, rr);
                    _DEBUG_PRINTF("RTC regs %02x %02x %02x %02x %02x %02x cnt %ldd %lds",
                        rr[0], rr[1], rr[2], rr[3], rr[4], rr[5], cd, cs);
                    _DEBUG_PRINTF("prefs offset loaded %d: %ldd %lds", ld, od, os);
                }
                _DEBUG_PRINTF("PMU r04 %02x r10 %02x r21 %02x r30 %02x r4b %02x r4c %02x r4d %02x",
                    pmu_read(0x04), pmu_read(0x10), pmu_read(0x21), pmu_read(0x30),
                    pmu_read(0x4b), pmu_read(0x4c), pmu_read(0x4d));
            }
#endif
            line++;
        }
#ifdef IPOD_NANO3G
        else if(state==2)
        {
            extern int pcm_nano3g_debug(int line);
            line = pcm_nano3g_debug(line);
        }
#endif
#ifdef IPOD_NANO3G
        else if(state==3)
        {
            _DEBUG_PRINTF("USB debug:");
            extern unsigned usb_dbg_enable_clocks, usb_dbg_usb_enable;
            extern int usb_detect(void);
            _DEBUG_PRINTF("USB: detect %d usb_enable calls %u phy inits %u",
                          usb_detect(), usb_dbg_usb_enable, usb_dbg_enable_clocks);
            extern uint32_t usb_dbg_hw[5]; extern int usb_dbg_clocks_on;
            extern unsigned usb_dbg_irqs, usb_dbg_rst, usb_dbg_enumdne, usb_dbg_setups, usb_dbg_oep, usb_dbg_iep;
            extern uint32_t usb_dbg_dsts, usb_dbg_gintsts_last, usb_dbg_doepint0;
            _DEBUG_PRINTF("DWC id %08lx cfg1 %08lx cfg2 %08lx", usb_dbg_hw[0], usb_dbg_hw[1], usb_dbg_hw[2]);
            _DEBUG_PRINTF("DWC cfg3 %08lx cfg4 %08lx clk %d", usb_dbg_hw[3], usb_dbg_hw[4], usb_dbg_clocks_on);
            _DEBUG_PRINTF("irq %u rst %u enum %u setup %u oep %u iep %u",
                          usb_dbg_irqs, usb_dbg_rst, usb_dbg_enumdne, usb_dbg_setups, usb_dbg_oep, usb_dbg_iep);
            _DEBUG_PRINTF("dsts %08lx gint %08lx doep0 %08lx", usb_dbg_dsts, usb_dbg_gintsts_last, usb_dbg_doepint0);
            if (usb_dbg_clocks_on)
                _DEBUG_PRINTF("live gintsts %08lx dsts %08lx dctl %08lx gusbcfg %08lx",
                              DWC_GINTSTS, DWC_DSTS, DWC_DCTL, DWC_GUSBCFG);
            if (usb_dbg_clocks_on)
                _DEBUG_PRINTF("doepctl0 %08lx doeptsiz0 %08lx gahbcfg %08lx daint %08lx",
                              DWC_DOEPCTL(0), DWC_DOEPTSIZ(0), DWC_GAHBCFG, DWC_DAINT);
        }
#endif
#ifdef IPOD_NANO3G
        else if(state==4)
        {
            extern void ftl_nano3g_state(uint32_t *out);
            extern uint32_t ftl_dbg_verify[6], ftl_dbg_guard[2], ftl_dbg_badrelease[2], ftl_dbg_ctrlrot[2];
            extern unsigned nand3g_stat_writes, nand3g_stat_erases, nand3g_stat_write_errors;
            uint32_t o[16]; ftl_nano3g_state(o);
            _DEBUG_PRINTF("FTL debug:");
            _DEBUG_PRINTF("ctrl %lu %lu %lu page %lx (blk %lu pg %lu) usn %lx",
                          o[0], o[1], o[2], o[3], o[3] / 1024, o[3] % 1024, o[4]);
            _DEBUG_PRINTF("pool free %lu idx %lu logs %lu clean %lu unclean-at-mount %lu",
                          o[5], o[6], o[15], o[7], o[8]);
            _DEBUG_PRINTF("VFL0 ctrl %lx %lx %lx usn %lx  VFL1 ctrl %lx %lx %lx usn %lx",
                          o[9] & 0xffff, o[9] >> 16, o[10] & 0xffff, o[10] >> 16,
                          o[11] & 0xffff, o[11] >> 16, o[12] & 0xffff, o[12] >> 16);
            _DEBUG_PRINTF("VFL nextpg %lu / %lu   ctrl ok %lu remap ok %lu",
                          o[13] & 0xffff, o[13] >> 16, o[14] & 1, (o[14] >> 1) & 1);
            _DEBUG_PRINTF("writes %u erases %u errs %u verified %lu vfail %lu cfail %lu",
                          nand3g_stat_writes, nand3g_stat_erases, nand3g_stat_write_errors,
                          ftl_dbg_verify[3], ftl_dbg_verify[1], ftl_dbg_verify[2]);
            {
                extern uint32_t ftl_watch[8];
                extern void ftl_nano3g_watch_poll(void);
                ftl_nano3g_watch_poll();
                static const char *reg[3] = { "cxt", "map", "vfl" };
                _DEBUG_PRINTF("watch armed %lu changes %lu first tick %lu (now %ld) last %lu",
                              ftl_watch[0], ftl_watch[1], ftl_watch[2], current_tick, ftl_watch[7]);
                if (ftl_watch[1])
                    _DEBUG_PRINTF("first change: %s +0x%lx  %08lx -> %08lx", reg[ftl_watch[3] % 3],
                                  ftl_watch[4], ftl_watch[5], ftl_watch[6]);
            }
            {
                extern uint32_t nand3g_guard[8];
                _DEBUG_PRINTF("nand guard hits %lu: op %lu page %lx ce %lu off %lu..%lu word %08lx rc %ld",
                              nand3g_guard[0], nand3g_guard[1], nand3g_guard[2], nand3g_guard[3],
                              nand3g_guard[4], nand3g_guard[5], nand3g_guard[6], (long)nand3g_guard[7]);
            }
            {
                extern uint32_t ftl_crumb[24], ftl_dbg_werr[4];
                _DEBUG_PRINTF("last write err %ld sector %lu cnt %lu alloc %lx", (long)ftl_dbg_werr[0],
                              ftl_dbg_werr[1], ftl_dbg_werr[2], ftl_dbg_werr[3]);
                if (ftl_crumb[0])
                {
                    _DEBUG_PRINTF("CRUMB from last panic: code %ld sector %lu cnt %lu werr %ld alloc %lx",
                                  (long)ftl_crumb[1], ftl_crumb[2], ftl_crumb[3], (long)ftl_crumb[4], ftl_crumb[5]);
                    _DEBUG_PRINTF(" ctrl %lu %lu %lu free %lu page %lx guard hits %lu op %lu pg %lx off %lu..%lu",
                                  ftl_crumb[6] & 0xffff, ftl_crumb[6] >> 16, ftl_crumb[7] & 0xffff, ftl_crumb[7] >> 16,
                                  ftl_crumb[8], ftl_crumb[9], ftl_crumb[10], ftl_crumb[11], ftl_crumb[13], ftl_crumb[14]);
                    _DEBUG_PRINTF(" watch changes %lu first tick %lu region %lu off %lx %08lx->%08lx",
                                  ftl_crumb[18], ftl_crumb[19], ftl_crumb[20], ftl_crumb[21], ftl_crumb[22], ftl_crumb[23]);
                }
            }
            _DEBUG_PRINTF("guard %lu (%lx) badrelease %lu (%lu) rotations %lu (last %lu->%lu)",
                          ftl_dbg_guard[0], ftl_dbg_guard[1], ftl_dbg_badrelease[0], ftl_dbg_badrelease[1],
                          ftl_dbg_ctrlrot[0], ftl_dbg_ctrlrot[1] & 0xffff, ftl_dbg_ctrlrot[1] >> 16);
        }
#endif
#ifdef UC87XX_DEBUG
        else if(state==(max_states-1))
        {
            extern struct uartc_port ser_port;
            bool opened = !!ser_port.uartc->port_l[ser_port.id];
            _DEBUG_PRINTF("UART %d: %s", ser_port.id, opened ? "opened":"closed");
            if (opened)
            {
                int tx_stat, rx_stat, tx_speed, rx_speed;
                char line_cfg[4];
                int abr_stat;
                uint32_t abr_cnt;
                static const char * const abrstatus[] = {"Idle", "Launched", "Counting", "Abnormal"};

                uartc_port_get_line_info(&ser_port,
                            &tx_stat, &rx_stat, &tx_speed, &rx_speed, line_cfg);
                abr_stat = uartc_port_get_abr_info(&ser_port, &abr_cnt);

                line++;
                _DEBUG_PRINTF("line: %s", line_cfg);
                _DEBUG_PRINTF("Tx: %s, speed: %d", tx_stat ? "On":"Off", tx_speed);
                _DEBUG_PRINTF("Rx: %s, speed: %d", rx_stat ? "On":"Off", rx_speed);
                _DEBUG_PRINTF("ABR: %s, cnt: %u", abrstatus[abr_stat], abr_cnt);
            }
            line++;
            _DEBUG_PRINTF("n_tx_bytes: %u", ser_port.n_tx_bytes);
            _DEBUG_PRINTF("n_rx_bytes: %u", ser_port.n_rx_bytes);
            _DEBUG_PRINTF("n_ovr_err: %u", ser_port.n_ovr_err);
            _DEBUG_PRINTF("n_parity_err: %u", ser_port.n_parity_err);
            _DEBUG_PRINTF("n_frame_err: %u", ser_port.n_frame_err);
            _DEBUG_PRINTF("n_break_detect: %u", ser_port.n_break_detect);
            _DEBUG_PRINTF("ABR n_abnormal: %u %u",
                            ser_port.n_abnormal0, ser_port.n_abnormal1);
        }
#endif
        else
        {
            state=0;
        }


        lcd_update(); 
        switch(button_get_w_tmo(HZ/20))
        {
            case BUTTON_SCROLL_BACK:
                if(state!=0) state--;
                break;

            case BUTTON_SCROLL_FWD:
                if(state!=max_states-1)
                {
                    state++;
                }
                break;

            case DEBUG_CANCEL:
            case BUTTON_REL:
                lcd_setfont(FONT_UI);
                return false;
        }
    }

    lcd_setfont(FONT_UI);
    return false;
}

bool dbg_ports(void)
{
    int line;

    lcd_setfont(FONT_SYSFIXED);

    while(1)
    {
        lcd_clear_display();
        line = 0;
        
        _DEBUG_PRINTF("GPIO  0: %08x",(unsigned int)PDAT(0));
        _DEBUG_PRINTF("GPIO  1: %08x",(unsigned int)PDAT(1));
        _DEBUG_PRINTF("GPIO  2: %08x",(unsigned int)PDAT(2));
        _DEBUG_PRINTF("GPIO  3: %08x",(unsigned int)PDAT(3));
        _DEBUG_PRINTF("GPIO  4: %08x",(unsigned int)PDAT(4));
        _DEBUG_PRINTF("GPIO  5: %08x",(unsigned int)PDAT(5));
        _DEBUG_PRINTF("GPIO  6: %08x",(unsigned int)PDAT(6));
        _DEBUG_PRINTF("GPIO  7: %08x",(unsigned int)PDAT(7));
        _DEBUG_PRINTF("GPIO  8: %08x",(unsigned int)PDAT(8));
        _DEBUG_PRINTF("GPIO  9: %08x",(unsigned int)PDAT(9));
        _DEBUG_PRINTF("GPIO 10: %08x",(unsigned int)PDAT(10));
        _DEBUG_PRINTF("GPIO 11: %08x",(unsigned int)PDAT(11));
        _DEBUG_PRINTF("GPIO 12: %08x",(unsigned int)PDAT(12));
        _DEBUG_PRINTF("GPIO 13: %08x",(unsigned int)PDAT(13));
        _DEBUG_PRINTF("GPIO 14: %08x",(unsigned int)PDAT(14));
        _DEBUG_PRINTF("GPIO 15: %08x",(unsigned int)PDAT(15));
        _DEBUG_PRINTF("USEC   : %08x",(unsigned int)USEC_TIMER);

        lcd_update();
        if (button_get_w_tmo(HZ/10) == (DEBUG_CANCEL|BUTTON_REL))
            break;
    }
    lcd_setfont(FONT_UI);
    return false;
}

