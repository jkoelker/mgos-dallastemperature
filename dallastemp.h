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

#ifndef SRC_DALLASTEMP_H_
#define SRC_DALLASTEMP_H_

/* Model IDs */
#define DS18S20MODEL 0x10 /* also DS1820 */
#define DS18B20MODEL 0x28
#define DS1822MODEL  0x22
#define DS1825MODEL  0x3B

/* Scratchpad size */
#define SCRATCHPAD_SIZE 9

/* Scratchpad locations */
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

/* Device resolution */
#define TEMP_9_BIT  0x1F /* 9 bit */
#define TEMP_10_BIT 0x3F /* 10 bit */
#define TEMP_11_BIT 0x5F /* 11 bit */
#define TEMP_12_BIT 0x7F /* 12 bit */

/* Error Codes */
#define DEVICE_DISCONNECTED_C (-127)
#define DEVICE_DISCONNECTED_F (-196.6)
#define DEVICE_DISCONNECTED_RAW (-7040)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stddef.h>

#include "common/queue.h"
#include "fw/src/mgos_onewire.h"
#include "fw/src/mgos_timers.h"

typedef void (*temp_callback)(const unsigned char *rom, int raw_temp);

struct dallastemp_device {
    unsigned char rom[8];
    SLIST_ENTRY(dallastemp_device) devices;
};

struct dallastemp {
    struct mgos_onewire *ow;
    unsigned int count;
    unsigned int resolution;
    SLIST_HEAD(devices, dallastemp_device) devices;
};

struct dallastemp *dallastemp_init(struct mgos_onewire *ow);

unsigned int dallastemp_begin(struct dallastemp *dt);

void dallastemp_end(struct dallastemp *dt);

mgos_timer_id dallastemp_temp(struct dallastemp *dt,
                              const unsigned char *rom,
                              temp_callback cb);

mgos_timer_id dallastemp_temp_all(struct dallastemp *dt,
                                  temp_callback cb);

int dallastemp_raw_temp(struct dallastemp *dt,
                        const unsigned char *rom);

float dallastemp_f_temp(struct dallastemp *dt,
                        const unsigned char *rom);

float dallastemp_c_temp(struct dallastemp *dt,
                        const unsigned char *rom);

float dallastemp_raw_to_c(int raw);

float dallastemp_raw_to_f(int raw);

unsigned int dallastemp_calculate_temp(const unsigned char *rom,
                                       const unsigned char *scratchpad);

unsigned int dallastemp_conversion_time(unsigned int resolution);

unsigned int dallastemp_convert_temp(struct dallastemp *dt,
                                     const unsigned char *rom);

unsigned int dallastemp_convert_temp_all(struct dallastemp *dt);

bool dallastemp_connected(struct dallastemp *dt,
                          const unsigned char *rom,
                          unsigned char *scratchpad,
                          size_t count);

bool dallastemp_read_powersupply(struct dallastemp *dt,
                                 const unsigned char *rom);

bool dallastemp_read_scratchpad(struct dallastemp *dt,
                                const unsigned char *rom,
                                unsigned char *scratchpad,
                                size_t len);

bool dallastemp_write_scratchpad(struct dallastemp *dt,
                                 const unsigned char *rom,
                                 unsigned char *scratchpad,
                                 size_t len);

unsigned int dallastemp_get_resolution(struct dallastemp *dt,
                                       const unsigned char *rom);

bool dallastemp_set_resolution(struct dallastemp *dt,
                               const unsigned char *rom,
                               unsigned int resolution);

void dallastemp_set_resolution_all(struct dallastemp *dt,
                                   unsigned int resolution);

int dallastemp_get_user_data(struct dallastemp *dt,
                             const unsigned char *rom);

bool dallastemp_set_user_data(struct dallastemp *dt,
                              const unsigned char *rom,
                              int user_data);


bool dallastemp_valid_address(const unsigned char *rom);

bool dallastemp_valid_family(const unsigned char *rom);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SRC_DALLASTEMP_H_ */
