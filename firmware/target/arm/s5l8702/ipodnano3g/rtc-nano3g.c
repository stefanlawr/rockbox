/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2026 by the Rockbox nano 3G port contributors
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

/* iPod nano 3G real-time clock.
 *
 * The D1671 PMU keeps a binary calendar in registers 0x40..0x45, but it is
 * only a counter that restarts at 2000-01-01 00:00:00 whenever the PMU
 * loses power (e.g. a DFU/reset cycle). Apple's firmware turns it into
 * wall time by adding an offset it keeps in its settings file
 * /iPod_Control/Device/Preferences, at file offset 0xb78: two little
 * endian 32-bit words, days and seconds-of-day (recovered from the 1.1.3
 * OS image, FUN_080539f0/FUN_08060e38, and verified on an MB245).
 *
 * Reading: wall = counter + offset. Setting the time does what the OF
 * does: counter = wall - offset, so both firmwares keep agreeing. The
 * offset file is only read, never written.
 */

#include "config.h"
#include "rtc.h"
#include "kernel.h"
#include "system.h"
#include "file.h"
#include "pmu-target.h"
#include "timefuncs.h"
#include "time.h"

#define PREFS_FILE      "/iPod_Control/Device/Preferences"
#define PREFS_OFF_DAYS  0xb78
#define PREFS_MIN_VER   0x30    /* offset field exists in the 0x3c layout */
#define SECS_PER_DAY    86400
#define EPOCH_2000      946684800L  /* 2000-01-01 00:00:00 UTC as time_t */

static long off_days = 0, off_secs = 0;
static bool off_loaded = false;
static long off_next_try = 0;

static void load_offset(void)
{
    if (off_loaded || TIME_BEFORE(current_tick, off_next_try))
        return;
    off_next_try = current_tick + HZ * 5;

    int fd = open(PREFS_FILE, O_RDONLY);
    if (fd < 0)
        return;
    uint32_t ver = 0, v[2] = { 0, 0 };
    bool ok = (read(fd, &ver, 4) == 4) && (ver >= PREFS_MIN_VER)
           && (lseek(fd, PREFS_OFF_DAYS, SEEK_SET) == PREFS_OFF_DAYS)
           && (read(fd, v, 8) == 8) && (v[1] < SECS_PER_DAY);
    close(fd);
    if (ok) {
        off_days = v[0];
        off_secs = v[1];
    }
    off_loaded = true;  /* file present: use it (or 0 if malformed) */
}

/* counter (from 2000-01-01) as days + seconds */
static void counter_read(long *days, long *secs)
{
    unsigned char buf[7];
    struct tm tm;
    pmu_read_rtc(buf);  /* binary */
    tm.tm_sec = buf[0]; tm.tm_min = buf[1]; tm.tm_hour = buf[2];
    tm.tm_mday = buf[4]; tm.tm_mon = buf[5] - 1; tm.tm_year = buf[6] + 100;
    time_t t = mktime(&tm) - EPOCH_2000;
    if (t < 0) t = 0;
    *days = t / SECS_PER_DAY;
    *secs = t % SECS_PER_DAY;
}

static void counter_write(long days, long secs)
{
    time_t t = EPOCH_2000 + days * SECS_PER_DAY + secs;
    struct tm tm;
    unsigned char buf[7];
    gmtime_r(&t, &tm);
    buf[0] = tm.tm_sec; buf[1] = tm.tm_min; buf[2] = tm.tm_hour;
    buf[3] = tm.tm_wday; buf[4] = tm.tm_mday; buf[5] = tm.tm_mon + 1;
    buf[6] = tm.tm_year - 100;
    pmu_write_rtc(buf);  /* binary */
}

void rtc_init(void)
{
}

/* for the PMU debug page */
void rtc_nano3g_debug(int *loaded, long *days, long *secs, long *cdays, long *csecs)
{
    load_offset();
    *loaded = off_loaded; *days = off_days; *secs = off_secs;
    counter_read(cdays, csecs);
}

int rtc_read_datetime(struct tm *tm)
{
    long days, secs;
    load_offset();
    counter_read(&days, &secs);
    days += off_days;
    secs += off_secs;
    if (secs >= SECS_PER_DAY) {
        secs -= SECS_PER_DAY;
        days++;
    }
    time_t t = EPOCH_2000 + days * SECS_PER_DAY + secs;
    gmtime_r(&t, tm);
    return 0;
}

int rtc_write_datetime(const struct tm *tm)
{
    struct tm tmp = *tm;
    load_offset();
    time_t t = mktime(&tmp) - EPOCH_2000;
    long days = t / SECS_PER_DAY - off_days;
    long secs = t % SECS_PER_DAY - off_secs;
    if (secs < 0) {
        secs += SECS_PER_DAY;
        days--;
    }
    if (days < 0) {
        days = 0;
        secs = 0;
    }
    counter_write(days, secs);
    return 0;
}
