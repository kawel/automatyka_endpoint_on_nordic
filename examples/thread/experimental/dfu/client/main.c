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
 * @defgroup thread_secure_dfu_example_main main.c
 * @{
 * @ingroup thread_secure_dfu_example
 * @brief Thread Secure DFU Example Application main file.
 *
 */
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "app_util.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp_thread.h"
#include "nrf_delay.h"
#include "mem_manager.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "coap_dfu.h"
#include "sdk_config.h"
#include "nrf_dfu_utils.h"

#include <openthread/openthread.h>
#include <openthread/diag.h>
#include <openthread/cli.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/alarm.h>
#include <openthread/platform/random.h>

typedef struct
{
    otInstance     * p_ot_instance;
    otNetifAddress   addresses[10];
    bool             trigger_dfu;
} application_t;

static application_t m_app =
{
    .p_ot_instance = NULL,
    .trigger_dfu   = false,
};

uint16_t m_coap_message_id = 0;

static void coap_error_handler(uint32_t error_code, coap_message_t * p_message)
{
    // If any response fill the p_response with a appropriate response message.
}

static void address_print(const otIp6Address *addr)
{
    char ipstr[40];
    snprintf(ipstr, sizeof(ipstr), "%x:%x:%x:%x:%x:%x:%x:%x",
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 0)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 1)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 2)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 3)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 4)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 5)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 6)),
             uint16_big_decode((uint8_t *)(addr->mFields.m16 + 7)));

    NRF_LOG_INFO("%s\r\n", (uint32_t)ipstr);
}

static void addresses_print(otInstance * aInstance)
{
    for (const otNetifAddress *addr = otIp6GetUnicastAddresses(aInstance); addr; addr = addr->mNext)
    {
        address_print(&addr->mAddress);
    }
}

static void state_changed_callback(uint32_t aFlags, void *aContext)
{
    if (aFlags & OT_CHANGED_THREAD_NETDATA)
    {
        otIp6SlaacUpdate(m_app.p_ot_instance,
                         m_app.addresses,
                         sizeof(m_app.addresses) / sizeof(m_app.addresses[0]),
                         otIp6CreateRandomIid,
                         NULL);

        addresses_print(m_app.p_ot_instance);
    }

    otDeviceRole role = otThreadGetDeviceRole(m_app.p_ot_instance);
    NRF_LOG_INFO("New role: %d\r\n", role);

    if (aFlags & OT_CHANGED_THREAD_ROLE)
    {
        switch(role)
        {
            case OT_DEVICE_ROLE_CHILD:
            case OT_DEVICE_ROLE_ROUTER:
            case OT_DEVICE_ROLE_LEADER:
                m_app.trigger_dfu = true;
                break;

            case OT_DEVICE_ROLE_DISABLED:
            case OT_DEVICE_ROLE_DETACHED:
            default:
                break;
        }
    }
}

static void thread_bsp_init(void)
{
    uint32_t err_code;
    err_code = bsp_init(BSP_INIT_LED, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}

static void thread_init(void)
{
    otInstance * p_instance;

    PlatformInit(0, NULL);

    p_instance = otInstanceInit();
    assert(p_instance != NULL);

    otCliUartInit(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    otSetStateChangedCallback(p_instance, state_changed_callback, NULL);

    if (!otDatasetIsCommissioned(p_instance))
    {
        assert(otLinkSetChannel(p_instance, THREAD_CHANNEL) == OT_ERROR_NONE);
        assert(otLinkSetPanId(p_instance, THREAD_PANID) == OT_ERROR_NONE);
    }

    assert(otIp6SetEnabled(p_instance, true) == OT_ERROR_NONE);
    assert(otThreadSetEnabled(p_instance, true) == OT_ERROR_NONE);

    m_app.p_ot_instance = p_instance;
}

static void coap_main_init()
{
    NRF_LOG_DEBUG("Init coap\r\n");
    coap_port_t local_port_list[COAP_PORT_COUNT] =
    {
        {
            .port_number = DFU_UDP_PORT
        }
    };

    coap_transport_init_t transport_params;
    transport_params.p_port_table = &local_port_list[0];
    transport_params.p_arg        = m_app.p_ot_instance;

    m_coap_message_id = otPlatRandomGet();
    coap_init(otPlatRandomGet(), &transport_params);
    coap_error_handler_register(coap_error_handler);
}

// Reset handler used after bootloader update, required by nrf_dfu_utils module.
static void reset_delay_timer_handler(void * p_context)
{
    NRF_LOG_DEBUG("Reset delay timer expired, resetting.\r\n");
#ifdef NRF_DFU_DEBUG_VERSION
    nrf_delay_ms(100);
#endif
    NVIC_SystemReset();
}

static void timers_init(void)
{
    APP_ERROR_CHECK(app_timer_init());
    APP_ERROR_CHECK(app_timer_create(&nrf_dfu_utils_reset_delay_timer, APP_TIMER_MODE_SINGLE_SHOT, reset_delay_timer_handler));
}

int main(int argc, char *argv[])
{
    NRF_LOG_INIT(NULL);
    nrf_mem_init();

    timers_init();

    thread_init();
    coap_main_init();

    thread_bsp_init();

    coap_dfu_init();

    uint32_t now, before = otPlatAlarmGetNow();
    while (1)
    {
        otTaskletsProcess(m_app.p_ot_instance);
        PlatformProcessDrivers(m_app.p_ot_instance);

        now = otPlatAlarmGetNow();
        if (now - before > 1000)
        {
            coap_time_tick();
            before = now;
        }

        if (m_app.trigger_dfu)
        {
            m_app.trigger_dfu = false;
            coap_dfu_trigger(NULL);
        }
    }
}

/** @} */
