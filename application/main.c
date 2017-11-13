/**
 * @file main.c
 * @author Pawel Radecki
 * @date 10 Nov 2017
 * @brief Main file for the KIWI End Point.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ep_cfg.h"
#include "ep_udp.h"
#include "ep_bsp.h"
#include "ep_res.h"

#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"

#include <openthread/openthread.h>
#include <openthread/cli.h>
#include <openthread/platform/platform.h>


static otInstance *ot_instance = NULL;

void otTaskletsSignalPending (otInstance *aInstance)
{
    (void)aInstance;
}

static void state_changed_callback(uint32_t flags, void * p_context)
{
    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n",
                 flags, otThreadGetDeviceRole(p_context));
}

static void thread_init(void)
{
    otInstance * p_instance;

    PlatformInit(0, NULL);

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance,
                                     &state_changed_callback,
                                     p_instance) == OT_ERROR_NONE);
    assert(otIp6SetEnabled(p_instance, true) == OT_ERROR_NONE);

    if (otDatasetIsCommissioned(p_instance))
    {
        NRF_LOG_INFO("Active Operational Dataset: Valid network present\r\n");
        otThreadSetEnabled(p_instance, true);
    }
    else
    {
        NRF_LOG_INFO("Active Operational Dataset: No Valid network present\r\n");
    }

    ot_instance = p_instance;
}

static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    NRF_LOG_INIT(NULL);

    thread_init();
    timer_init();

    ep_bsp_init(ot_instance);
    ep_cfg_init();
    ep_res_init(ot_instance);
    ep_udp_start(ot_instance);

    while (true)
    {
        otTaskletsProcess(ot_instance);
        PlatformProcessDrivers(ot_instance);

        ep_cfg_check_and_apply(ot_instance);
    }
}

/**
 *@}
 **/
