/**
 * @file ep_bsp.c
 * @author Pawel Radecki
 * @date 12 Nov 2017
 *
 */

#include "ep_bsp.h"
#include "utils.h"

#include "nrf_log.h"
#include "boards.h"
#include "bsp_thread.h"

static otInstance *ot_main_instance = NULL;

static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_3:
            NRF_LOG_INFO("Thread: FACTORY RESET\r\n");
            otInstanceFactoryReset(ot_main_instance);
            break;

        default:
            return;
    }
}

static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}

static void thread_bsp_init(otInstance *ot_instance)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(ot_instance);
    APP_ERROR_CHECK(err_code);
}

void ep_bsp_init(otInstance *ot_instance)
{
    ot_main_instance = ot_instance;

    leds_init();
    thread_bsp_init(ot_instance);
}
