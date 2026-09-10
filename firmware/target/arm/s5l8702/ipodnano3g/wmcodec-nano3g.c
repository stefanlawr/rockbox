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
 * iPod nano 3G audio codec glue. The codec is an Apple-marked Wolfson
 * ("WM1870", 338S0462) that behaves as a WM8975: write-only 9-bit
 * registers at I2C address 0x1A (0x34 write) on bus 0. The OF's volume
 * writes (registers 2 and 3 with the update bit) match the WM8975 map.
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

#include "system.h"
#include "audiohw.h"
#include "i2c-s5l8702.h"
#include "s5l87xx.h"
#include "wmcodec.h"

void audiohw_init(void)
{
    /* audiohw_preinit() is called from the PCM driver once the I2S
       controller and MCLK are configured (pcm-s5l8702.c). */
}

void wmcodec_write(int reg, int data)
{
    unsigned char d = data & 0xff;
    /* 9-bit Wolfson protocol: first byte = (reg << 1) | data bit 8 */
    i2c_write(0, 0x34, (reg << 1) | ((data & 0x100) >> 8), 1, &d);
}
