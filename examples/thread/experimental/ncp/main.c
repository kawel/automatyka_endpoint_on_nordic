/* Copyright (c) 2016-2017 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ncp_example_main main.c
 * @{
 * @ingroup ncp_example
 * @brief An example presenting OpenThread NCP.
 *
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bsp_thread.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <openthread/openthread.h>
#include <openthread/thread_ftd.h>
#include <openthread/ncp.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/settings.h>

#define ROUTER_SELECTION_JITTER  5       /**< A value of router selection jitter. */

typedef struct
{
    otInstance * p_ot_instance;
} application_t;

static application_t m_app =
{
    .p_ot_instance = NULL
};

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

static void thread_init(void)
{
    otInstance *p_instance;

    PlatformInit(0, NULL);

    otPlatSettingsWipe(NULL);

    p_instance = otInstanceInit();
    assert(p_instance);

    otNcpInit(p_instance);

    otThreadSetRouterSelectionJitter(p_instance, ROUTER_SELECTION_JITTER);

    m_app.p_ot_instance = p_instance;
}

static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}

static void thread_bsp_init(void)
{
    uint32_t err_code;
    err_code = bsp_init(BSP_INIT_LED, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
    NRF_LOG_INIT(NULL);

    thread_init();

    timer_init();
    thread_bsp_init();
    leds_init();

    while (true)
    {
        otTaskletsProcess(m_app.p_ot_instance);
        PlatformProcessDrivers(m_app.p_ot_instance);
    }
}

/**
 *@}
 **/
