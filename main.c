#include <stdbool.h>
#include <stdio.h>

#include "common/platform.h"
#include "common/queue.h"
#include "fw/src/mgos_app.h"
#include "fw/src/mgos_gpio.h"
#include "fw/src/mgos_onewire.h"
#include "fw/src/mgos_sys_config.h"
#include "fw/src/mgos_timers.h"

#include "dallastemp.h"


#if CS_PLATFORM == CS_P_ESP8266 /* ESP8266 - main target */
    #define LED_GPIO 2
    #define ONEWIRE_GPIO 0
#else
    #error Unsupported platform
#endif

#ifndef TEMP_UPDATE_INTERVAL
    #define TEMP_UPDATE_INTERVAL 5000
#endif

#ifndef TEMP_EMA_SPAN
    #define TEMP_EMA_SPAN 12
#endif

#define _EMA(c, p, s) ((p) + (2.0 / ((s) + 1)) * ((c) - (p)))
#if TEMP_EMA_SPAN >= 1
    #define TEMP_EMA(c, p, s) _EMA(c, p, s)
#else
    #define TEMP_EMA(c, p, s) (c)
#endif

#define SMOOTH_TEMP(c, p) TEMP_EMA(c, p, TEMP_EMA_SPAN)


struct mgos_onewire *ow = NULL;
struct dallastemp *dt = NULL;

static void dallastemp_temp_cb(const unsigned char *rom,
                               int raw_temp) {
    mgos_gpio_write(LED_GPIO, 0);
    LOG(LL_INFO, ("Temp of %x:%x:%x:%x:%x:%x:%x:%x %d (%04f)",
                  rom[0], rom[1], rom[2], rom[3],
                  rom[4], rom[5], rom[6], rom[7],
                  raw_temp, dallastemp_raw_to_c(raw_temp)));
    mgos_gpio_write(LED_GPIO, 1);
}

static void temp_timer_cb(void *arg) {
    struct dallastemp_device *device;

    if (dt == NULL) {
        return;
    }

    SLIST_FOREACH(device, &dt->devices, devices) {
        dallastemp_temp(dt, device->rom, dallastemp_temp_cb);
    }

    (void) arg;
}

enum mgos_app_init_result mgos_app_init(void) {

    mgos_gpio_set_mode(LED_GPIO, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(LED_GPIO, 1);

    ow = mgos_onewire_init(ONEWIRE_GPIO);
    if (ow == NULL) {
        return MGOS_APP_INIT_ERROR;
    }

    dt = dallastemp_init(ow);
    if (dt == NULL) {
        return MGOS_APP_INIT_ERROR;
    }

    dallastemp_begin(dt);

    mgos_set_timer(5000 /* ms */,
                   true /* repeat */,
                   temp_timer_cb, NULL);

  return MGOS_APP_INIT_SUCCESS;
}
