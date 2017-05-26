/*
 * Copyright 2017 Jason Kölker
 */

/*
 * This library is derived and follows the conventions of the arduino
 * DallasTemperature from Miles Burton, et. al.
 * https://github.com/milesburton/Arduino-Temperature-Control-Library
 */


/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "dallastemp.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "fw/src/mgos_hal.h"
#include "fw/src/mgos_onewire.h"
#include "fw/src/mgos_timers.h"


#define CONVERT_T 0x44
#define WRITE_SCRATCHPAD 0x4E
#define READ_POWER_SUPPLY 0xB4
#define READ_SCRATCHPAD 0xBE

#define constrain(v, l, h) ((v) < (l) ? (l) : ((v) > (h) ? (h) : (v)))
#define max(a,b) ((a)>(b)?(a):(b))

struct dallastemp_temp_info {
    temp_callback cb;
    struct dallastemp *dt;
    const unsigned char *rom;
};

struct dallastemp *dallastemp_init(struct mgos_onewire *ow) {
    struct dallastemp *dt = calloc(1, sizeof(*dt));

    if (dt == NULL) {
        return NULL;
    }

    dt->ow = ow;
    dt->count = 0;
    dt->resolution = 9;
    SLIST_INIT(&dt->devices);

    return dt;
}

unsigned int dallastemp_begin(struct dallastemp *dt) {
    unsigned char rom[8] = {0};
    struct dallastemp_device *device = NULL;

    if (dt == NULL) {
        return 0;
    }

    while (!SLIST_EMPTY(&dt->devices)) {
        device = SLIST_FIRST(&dt->devices);
        SLIST_REMOVE_HEAD(&dt->devices, devices);
        free(device);
    }

    dt->count = 0;
    device = NULL;
    mgos_onewire_search_clean(dt->ow);

    while (mgos_onewire_next(dt->ow, rom, 0)) {
        if (dallastemp_valid_address(rom)) {
            dt->resolution = max(dt->resolution,
                                 dallastemp_get_resolution(dt, rom));
            device = calloc(1, sizeof(*device));
            memcpy(device->rom, rom, sizeof(rom));
            SLIST_INSERT_HEAD(&dt->devices, device, devices);
            dt->count++;
        }
    }

    return dt->count;
}

void dallastemp_end(struct dallastemp *dt) {
    struct dallastemp_device *device = NULL;

    if (dt == NULL) {
        return;
    }

    while (!SLIST_EMPTY(&dt->devices)) {
        device = SLIST_FIRST(&dt->devices);
        SLIST_REMOVE_HEAD(&dt->devices, devices);
        free(device);
    }

    free(dt);
}

static void dallastemp_invoke_temp_cb(void *param) {
    struct dallastemp_temp_info *ti = (struct dallastemp_temp_info *) param;
    int raw_temp = dallastemp_raw_temp(ti->dt, ti->rom);

    ti->cb(ti->rom, raw_temp);
    free(ti);
}

mgos_timer_id dallastemp_temp(struct dallastemp *dt,
                              const unsigned char *rom,
                              temp_callback cb) {
    unsigned int conversion_time;
    struct dallastemp_temp_info *ti = calloc(1, sizeof(*ti));

    ti->cb = cb;
    ti->dt = dt;
    ti->rom = rom;

    conversion_time = dallastemp_convert_temp(dt, rom);
    return mgos_set_timer(conversion_time,
                          false /* repeat */,
                          dallastemp_invoke_temp_cb, ti);
}

static void dallastemp_invoke_temp_all_cb(void *param) {
    struct dallastemp_temp_info *ti = (struct dallastemp_temp_info *) param;
    struct dallastemp_device *device = NULL;

    SLIST_FOREACH(device, &ti->dt->devices, devices) {
        ti->cb(device->rom, dallastemp_raw_temp(ti->dt, device->rom));
    }

    free(ti);
}

mgos_timer_id dallastemp_temp_all(struct dallastemp *dt,
                                  temp_callback cb) {
    unsigned int conversion_time;
    struct dallastemp_temp_info *ti = calloc(1, sizeof(*ti));

    ti->cb = cb;
    ti->dt = dt;
    ti->rom = NULL;

    conversion_time = dallastemp_convert_temp_all(dt);
    return mgos_set_timer(conversion_time,
                          false /* repeat */,
                          dallastemp_invoke_temp_all_cb, ti);
}

int dallastemp_raw_temp(struct dallastemp *dt,
                        const unsigned char *rom) {
    unsigned char scratchpad[SCRATCHPAD_SIZE] = {0};

    if (dallastemp_connected(dt, rom, scratchpad, SCRATCHPAD_SIZE)) {
        return dallastemp_calculate_temp(rom, scratchpad);
    }

    return DEVICE_DISCONNECTED_RAW;
}

float dallastemp_c_temp(struct dallastemp *dt,
                        const unsigned char *rom) {
    return dallastemp_raw_to_c(dallastemp_raw_temp(dt, rom));
}

float dallastemp_f_temp(struct dallastemp *dt,
                        const unsigned char *rom) {
    return dallastemp_raw_to_f(dallastemp_raw_temp(dt, rom));
}

float dallastemp_raw_to_c(int raw) {
    if (raw <= DEVICE_DISCONNECTED_RAW) {
        return DEVICE_DISCONNECTED_C;
    }

    /* C = RAW/128 */
    return raw * 0.0078125;
}


float dallastemp_raw_to_f(int raw) {
    if (raw <= DEVICE_DISCONNECTED_RAW) {
        return DEVICE_DISCONNECTED_F;
    }

    /* C = RAW/128 */
    /* F = (C*1.8)+32 = (RAW/128*1.8)+32 = (RAW*0.0140625)+32 */
    return raw * 0.0140625 + 32;
}

unsigned int dallastemp_calculate_temp(const unsigned char *rom,
                                       const unsigned char *scratchpad) {
    unsigned int temp =((scratchpad[TEMP_MSB]) << 11) |
                        ((scratchpad[TEMP_LSB]) << 3);

    /*
    DS1820 and DS18S20 have a 9-bit temperature register.
    Resolutions greater than 9-bit can be calculated using the data from
    the temperature, and COUNT REMAIN and COUNT PER °C registers in the
    scratchpad.  The resolution of the calculation depends on the model.
    While the COUNT PER °C register is hard-wired to 16 (10h) in a
    DS18S20, it changes with temperature in DS1820.
    After reading the scratchpad, the TEMP_READ value is obtained by
    truncating the 0.5°C bit (bit 0) from the temperature data. The
    extended resolution temperature can then be calculated using the
    following equation:
                                    COUNT_PER_C - COUNT_REMAIN
    TEMPERATURE = TEMP_READ - 0.25 + --------------------------
                                            COUNT_PER_C
    Hagai Shatz simplified this to integer arithmetic for a 12 bits
    value for a DS18S20, and James Cameron added legacy DS1820 support.
    See:
    http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
    */

    if (rom[0] == DS18S20MODEL){
        temp = ((temp & 0xfff0) << 3) - 16 +
            (
                ((scratchpad[COUNT_PER_C] - scratchpad[COUNT_REMAIN]) << 7) /
                  scratchpad[COUNT_PER_C]
            );
    }

    return temp;
}

unsigned int dallastemp_conversion_time(unsigned int resolution) {
    switch (resolution) {
        case 9:
            return 94;
        case 10:
            return 188;
        case 11:
            return 375;
        default:
            return 750;
    }
}

unsigned int dallastemp_convert_temp(struct dallastemp *dt,
                                     const unsigned char *rom) {

    unsigned int resolution = dallastemp_get_resolution(dt, rom);

    if (resolution == 0 || mgos_onewire_reset(dt->ow) == 0) {
        return 0;
    }

    mgos_onewire_select(dt->ow, rom);
    mgos_onewire_write(dt->ow, CONVERT_T);
    return dallastemp_conversion_time(resolution);
}

unsigned int dallastemp_convert_temp_all(struct dallastemp *dt) {
    if (mgos_onewire_reset(dt->ow) == 0) {
        return 0;
    }

    mgos_onewire_skip(dt->ow);
    mgos_onewire_write(dt->ow, CONVERT_T);
    return dallastemp_conversion_time(dt->resolution);
}

bool dallastemp_connected(struct dallastemp *dt,
                          const unsigned char *rom,
                          unsigned char *scratchpad,
                          size_t count) {
    return (dallastemp_read_scratchpad(dt, rom, scratchpad, count) &&
            mgos_onewire_crc8(scratchpad, 8) == scratchpad[SCRATCHPAD_CRC]);
}

bool dallastemp_read_powersupply(struct dallastemp *dt,
                                 const unsigned char *rom) {
    bool parasitic = false;

    if (mgos_onewire_reset(dt->ow) == 0) {
        return false;
    }

    mgos_onewire_select(dt->ow, rom);
    mgos_onewire_write(dt->ow, READ_POWER_SUPPLY);

    if (mgos_onewire_read_bit(dt->ow) == 0) {
        parasitic = true;
    }

    mgos_onewire_reset(dt->ow);

    return parasitic;
}

bool dallastemp_read_scratchpad(struct dallastemp *dt,
                                const unsigned char *rom,
                                unsigned char *scratchpad,
                                size_t len) {
    if (len < SCRATCHPAD_SIZE || mgos_onewire_reset(dt->ow) == 0) {
        return false;
    }

    mgos_onewire_select(dt->ow, rom);
    mgos_onewire_write(dt->ow, READ_SCRATCHPAD);
    mgos_onewire_read_bytes(dt->ow, scratchpad, SCRATCHPAD_SIZE);

    return (mgos_onewire_reset(dt->ow) == 1);
}

bool dallastemp_write_scratchpad(struct dallastemp *dt,
                                 const unsigned char *rom,
                                 unsigned char *scratchpad,
                                 size_t len) {
    if (rom[0] == DS18S20MODEL && len < 4) {
        return false;
    }

    if (len < 5 || mgos_onewire_reset(dt->ow) == 0) {
        return false;
    }

    mgos_onewire_select(dt->ow, rom);
    mgos_onewire_write(dt->ow, WRITE_SCRATCHPAD);
    mgos_onewire_write(dt->ow, scratchpad[HIGH_ALARM_TEMP]);
    mgos_onewire_write(dt->ow, scratchpad[LOW_ALARM_TEMP]);

    if (rom[0] != DS18S20MODEL) {
        mgos_onewire_write(dt->ow, scratchpad[CONFIGURATION]);
    }

    return (mgos_onewire_reset(dt->ow) == 1);
}

unsigned int dallastemp_get_resolution(struct dallastemp *dt,
                                       const unsigned char *rom) {
    if (rom[0] == DS18S20MODEL) {
        return 12;
    }

    unsigned char scratchpad[SCRATCHPAD_SIZE] = {0};

    if (dallastemp_connected(dt, rom, scratchpad, SCRATCHPAD_SIZE)) {
        switch (scratchpad[CONFIGURATION]) {
            case TEMP_12_BIT:
                return 12;

            case TEMP_11_BIT:
                return 11;

            case TEMP_10_BIT:
                return 10;

            case TEMP_9_BIT:
                return 9;
        }
    }
    return 0;
}

bool dallastemp_set_resolution(struct dallastemp *dt,
                               const unsigned char *rom,
                               unsigned int resolution) {
    if (rom [0] == DS18S20MODEL) {
        return true;
    }

    resolution = constrain(resolution, 9, 12);

    if (dallastemp_get_resolution(dt, rom) == resolution) {
        return true;
    }

    unsigned char scratchpad[SCRATCHPAD_SIZE] = {0};

    if (dallastemp_connected(dt, rom, scratchpad, SCRATCHPAD_SIZE)) {
        switch (resolution) {
            case 12:
                scratchpad[CONFIGURATION] = TEMP_12_BIT;
                break;
            case 11:
                scratchpad[CONFIGURATION] = TEMP_11_BIT;
                break;
            case 10:
                scratchpad[CONFIGURATION] = TEMP_10_BIT;
                break;
            case 9:
            default:
                scratchpad[CONFIGURATION] = TEMP_9_BIT;
                break;
        }
    }

    return dallastemp_write_scratchpad(dt, rom, scratchpad, SCRATCHPAD_SIZE);
}

void dallastemp_set_resolution_all(struct dallastemp *dt,
                                   unsigned int resolution) {
    struct dallastemp_device *device = NULL;

    if (dt == NULL) {
        return;
    }

    dt->resolution = constrain(resolution, 9, 12);

    SLIST_FOREACH(device, &dt->devices, devices) {
        dallastemp_set_resolution(dt, device->rom, dt->resolution);
    }
}

int dallastemp_get_user_data(struct dallastemp *dt,
                             const unsigned char *rom) {
    int user_data = 0;
    unsigned char scratchpad[SCRATCHPAD_SIZE] = {0};

    if (dallastemp_connected(dt, rom, scratchpad, SCRATCHPAD_SIZE)) {
        user_data = scratchpad[HIGH_ALARM_TEMP] << 8;
        user_data += scratchpad[LOW_ALARM_TEMP];
    }

    return user_data;
}

bool dallastemp_set_user_data(struct dallastemp *dt,
                              const unsigned char *rom,
                              int user_data) {
    if (dallastemp_get_user_data(dt, rom) == user_data) {
        return true;
    }

    unsigned char scratchpad[SCRATCHPAD_SIZE] = {0};

    if (!dallastemp_connected(dt, rom, scratchpad, SCRATCHPAD_SIZE)) {
        return false;
    }

    scratchpad[HIGH_ALARM_TEMP] = user_data >> 8;
    scratchpad[LOW_ALARM_TEMP] = user_data & 255;
    return dallastemp_write_scratchpad(dt, rom, scratchpad, SCRATCHPAD_SIZE);
}

bool dallastemp_valid_address(const unsigned char *rom) {
    return mgos_onewire_crc8(rom, 7) == rom[7];
}

bool dallastemp_valid_family(const unsigned char *rom) {
    switch (rom[0]) {
        case DS18S20MODEL:
        case DS18B20MODEL:
        case DS1822MODEL:
        case DS1825MODEL:
            return true;
        default:
        return false;
    }
}
