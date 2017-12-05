/**
 * @file ep_thread.c
 * @author Pawel Radecki
 * @date 04 Dec 2017
 *
 */

#include <assert.h>

#include "ep_thread.h"
#include "utils.h"

#include "nrf_log.h"
#include "ep_bsp.h"
#include "ex_light.h"

#include <openthread/platform/platform.h>
#include <openthread/cli.h>


#ifdef EP_THREAD_LOGGING
    #define EP_THREAD_LOG_INFO(...)   NRF_LOG_INFO("EP THREAD: " __VA_ARGS__)
    #define EP_THREAD_LOG_ERROR(...)  NRF_LOG_ERROR("EP THREAD: " __VA_ARGS__)
#else
    #define EP_THREAD_LOG_INFO(...)
    #define EP_THREAD_LOG_ERROR(...)
#endif


static void handle_role_change(void * p_context, otDeviceRole role)
{
    switch(role)
    {
        case OT_DEVICE_ROLE_CHILD:
        case OT_DEVICE_ROLE_ROUTER:
        case OT_DEVICE_ROLE_LEADER:
            ep_bsp_indication_thread_set(EP_BSP_INDICATE_THREAD_CONNECTED);
            break;

        case OT_DEVICE_ROLE_DISABLED:
            ep_bsp_indication_thread_set(EP_BSP_INDICATE_THREAD_DISABLED);
            break;

        case OT_DEVICE_ROLE_DETACHED:
        default:
            ep_bsp_indication_thread_set(EP_BSP_INDICATE_THREAD_DETACHED);
            ex_light_set_peer_address_unspecified();
            break;
    }
}

void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        handle_role_change(p_context, otThreadGetDeviceRole(p_context));
    }

    if (flags & OT_CHANGED_THREAD_PARTITION_ID)
    {
        ex_light_set_peer_address_unspecified();
    }

    EP_THREAD_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}


otInstance * ep_thread_init(void)
{
    otInstance *p_instance;

    PlatformInit(0, NULL);

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    EP_THREAD_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    EP_THREAD_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance) == OT_ERROR_NONE);
    assert(otIp6SetEnabled(p_instance, true) == OT_ERROR_NONE);

    if (otDatasetIsCommissioned(p_instance))
    {
        EP_THREAD_LOG_INFO("Active Operational Dataset: Valid network present\r\n");
        assert(otThreadSetEnabled(p_instance, true) == OT_ERROR_NONE);
    }
    else
    {
        EP_THREAD_LOG_INFO("Active Operational Dataset: No Valid network present. Load default values.\r\n");
        ep_bsp_indication_thread_set(EP_BSP_INDICATE_THREAD_DISABLED);
    }

    EP_THREAD_LOG_INFO("Thread channel: %d\r\n", otLinkGetChannel(p_instance));
    EP_THREAD_LOG_INFO("Thread PANID: 0x%X\r\n", otLinkGetPanId(p_instance));

    return p_instance;
}
