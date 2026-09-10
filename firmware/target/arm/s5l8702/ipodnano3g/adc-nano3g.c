/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2009 by Bertrik Sikken
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

#include "inttypes.h"
#include "s5l87xx.h"
#include "adc.h"
#include "adc-target.h"
#include "pmu-target.h"
#include "kernel.h"

/* Channel table. Mux 0 is VBAT: on an MB245 it read 829 unplugged after
   a long charge and 929 on USB. Scale follows the Dialog convention used
   by the DA905x drivers, mV = 2500 + raw * 2000 / 1023 (10 bit, 2.5-4.5 V),
   which gives 4121 mV / 4316 mV for those readings; a 5 V full scale would
   put the charging value at an impossible 4.54 V. The OF's wrapper maps:
   1 -> mux 0x13 8-bit averaged (thermistor, bias on), 2 -> mux 4 (USB
   D+/D- via ADCIN), 3 -> mux 1. Muxes 2, 5 and 6 also move with USB. */
static const struct pmu_adc_channel adc_channels[] =
{
    [ADC_BATTERY] =
    {
        .name = "Battery",
        .mux = 0, .bits8 = 0,
        .mv_num = 2000, .mv_den = 1023, .mv_off = 2500,
    },
    [ADC_USBDATA] =
    {
        .name = "USB D+/D-",
        .mux = 4, .bits8 = 0,
        .mv_num = 5000, .mv_den = 1023, .mv_off = 0,   /* TBC */
    },
    [ADC_ACCESSORY] =
    {
        .name = "Accessory",
        .mux = 1, .bits8 = 0,
        .mv_num = 0, .mv_den = 0, .mv_off = 0,          /* raw */
    },
};

const char *adc_name(int channel)
{
    return adc_channels[channel].name;
}

unsigned short adc_read_millivolts(int channel)
{
    const struct pmu_adc_channel *ch = &adc_channels[channel];
    return pmu_adc_raw2mv(ch, pmu_read_adc(ch));
}

/* Returns battery voltage [millivolts] */
unsigned int adc_read_battery_voltage(void)
{
    return adc_read_millivolts(ADC_BATTERY);
}

/* API functions */
unsigned short adc_read(int channel)
{
    return pmu_read_adc(&adc_channels[channel]);
}

int adc_read_accessory_resistor(void)
{
    return adc_read(ADC_ACCESSORY);
}

unsigned int adc_read_usbdata_voltage(bool dp)
{
    (void)dp;
    return adc_read_millivolts(ADC_USBDATA);
}

void adc_init(void)
{
}
