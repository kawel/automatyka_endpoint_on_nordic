/**
 * Copyright (c) 2017 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_and_thread_coap_client_main main.c
 * @{
 * @ingroup  ble_sdk_uart_over_ble_and_thread_coap_client
 * @brief    UART over BLE application and Thread CoAP Client main file.
 *
 * This application shows dynamic multiprotocol support for Thread and Bluetooth Low Energy.
 * Thread operates on 802.15.4 radio during Bluetooth Low Energy radio's inactive time.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_drv_clock.h"

#include <openthread/openthread.h>
#include <openthread/cli.h>
#include <openthread/platform/platform.h>
#include <openthread/platform/platform-softdevice.h>

#include "ep_cfg.h"
#include "ep_udp.h"
#include "ep_bsp.h"
#include "ep_coap.h"
#include "ex_light.h"
#include "ble/ep_ble_main.h"


static otInstance *ot_instance = NULL;


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
	appble_sys_evt_dispatch(sys_evt);

    // Dispatch nRF driver clock events.
    nrf_drv_clock_on_soc_event(sys_evt);
    
    // Dispatch the system event to the OpenThread platform.
    PlatformSoftdeviceSocEvtHandler(sys_evt);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



/***************************************************************************************************
 * @section State change handling.
 **************************************************************************************************/

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

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{    
    switch (event)
    {
        case BSP_EVENT_KEY_0:
			NRF_LOG_INFO("Button 0 pushed.\r\n");
			appble_adv_triggered();
            ex_light_unicast();
            break;

        case BSP_EVENT_KEY_1:
            ex_light_multicast();
            break;

        case BSP_EVENT_KEY_2:
            break;

        case BSP_EVENT_KEY_3:
            ex_light_provisioning();
            break;

        case BSP_EVENT_DISCONNECT:
        	appble_disconn_triggered();
            break;

        case BSP_EVENT_WHITELIST_OFF:
        	appble_wl_off_triggered();
            break;

        default:
            break;
    }
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    ep_bsp_init();

    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

static void thread_init(void)
{
    otInstance *p_instance;

    PlatformInit(0, NULL);

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance) == OT_ERROR_NONE);
    assert(otIp6SetEnabled(p_instance, true) == OT_ERROR_NONE);

    if (otDatasetIsCommissioned(p_instance))
    {
        NRF_LOG_INFO("Active Operational Dataset: Valid network present\r\n");
        assert(otThreadSetEnabled(p_instance, true) == OT_ERROR_NONE);
    }
    else
    {
        NRF_LOG_INFO("Active Operational Dataset: No Valid network present. Load default values.\r\n");
        ep_bsp_indication_thread_set(EP_BSP_INDICATE_THREAD_DISABLED);
    }

    NRF_LOG_INFO("Thread channel: %d\r\n", otLinkGetChannel(p_instance));
    NRF_LOG_INFO("Thread PANID: 0x%X\r\n", otLinkGetPanId(p_instance));

    ot_instance = p_instance;
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool     erase_bonds;

    // Initialize.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    log_init();
    buttons_leds_init(&erase_bonds);
    appble_init(sys_evt_dispatch);
    
    // Thread initialization
    thread_init();
    ex_light_init(ot_instance);
    ep_cfg_init();
    ep_coap_init(ot_instance);
    ep_udp_start(ot_instance);
      
    appble_start(erase_bonds);
    NRF_LOG_INFO("Blinky Started!\r\n");
    
    // Enter main loop.
    for (;;)
    {
        otTaskletsProcess(ot_instance);
        PlatformProcessDrivers(ot_instance);
        ep_cfg_check_and_apply(ot_instance);

        if (NRF_LOG_PROCESS() == false && !otTaskletsArePending(ot_instance))
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
