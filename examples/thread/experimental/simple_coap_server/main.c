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
 * @defgroup simple_coap_server_example_main main.c
 * @{
 * @ingroup simple_coap_server_example_example
 * @brief Simple CoAP Server Example Application main file.
 *
 * @details This example demonstrates a CoAP server application that provides resources to control BSP_LED_3
 *          via CoAP messages. It can be controlled by a board with related Simple CoAP Server application.
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
#include <openthread/diag.h>
#include <openthread/coap.h>
#include <openthread/cli.h>
#include <openthread/thread_ftd.h>
#include <openthread/platform/alarm.h>
#include <openthread/platform/platform.h>

#define LED_INTERVAL             100
#define PROVISIONING_EXPIRY_TIME 5000

APP_TIMER_DEF(m_provisioning_timer);
APP_TIMER_DEF(m_led_timer);

static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage           * p_message,
                                  const otMessageInfo * p_message_info);

static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage           * p_message,
                                         const otMessageInfo * p_message_info);

typedef enum
{
    DEVICE_TYPE_REMOTE_CONTROL,
    DEVICE_TYPE_LIGHT
} device_type_t;

typedef enum
{
    LIGHT_OFF = 0,
    LIGHT_ON,
    LIGHT_TOGGLE
} light_command_t;

typedef struct
{
    otInstance     * p_ot_instance;         /**< A pointer to the OpenThread instance. */
    bool             enable_provisioning;   /**< Information if provisioning is enabled. */
    uint32_t         provisioning_expiry;   /**< Provisioning timeout time. */
    otCoapResource   provisioning_resource; /**< CoAP provisioning resource. */
    otCoapResource   light_resource;        /**< CoAP light resource. */
} application_t;

application_t m_app =
{
    .p_ot_instance         = NULL,
    .enable_provisioning   = false,
    .provisioning_expiry   = 0,
    .provisioning_resource = {"provisioning", provisioning_request_handler, NULL, NULL},
    .light_resource        = {"light", light_request_handler, NULL, NULL},
};

/***************************************************************************************************
 * @section CoAP
 **************************************************************************************************/

static void light_on(void)
{
    LEDS_ON(BSP_LED_3_MASK);
}

static void light_off(void)
{
    LEDS_OFF(BSP_LED_3_MASK);
}

static void light_toggle(void)
{
    LEDS_INVERT(BSP_LED_3_MASK);
}

static void provisioning_disable(otInstance * p_instance)
{
    m_app.enable_provisioning = false;
    m_app.provisioning_expiry = 0;
    app_timer_stop(m_provisioning_timer);
}

static void provisioning_enable(otInstance * p_instance)
{
    m_app.enable_provisioning = true;
    m_app.provisioning_expiry = otPlatAlarmGetNow() + PROVISIONING_EXPIRY_TIME;
    app_timer_start(m_provisioning_timer,
                    APP_TIMER_TICKS(PROVISIONING_EXPIRY_TIME),
                    p_instance);
}

static void light_response_send(void                * p_context,
                                otCoapHeader        * p_request_header,
                                const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NONE;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
        otCoapHeaderSetMessageId(&header, otCoapHeaderGetMessageId(p_request_header));
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));

        p_response = otCoapNewMessage(p_context, &header);
        if (p_response == NULL)
        {
            break;
        }

        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if (error != OT_ERROR_NONE && p_response != NULL)
    {
        otMessageFree(p_response);
    }
}

static void light_request_handler(void                * p_context,
                                  otCoapHeader        * p_header,
                                  otMessage           * p_message,
                                  const otMessageInfo * p_message_info)
{
    (void)p_message;
    uint8_t command;

    do
    {
        if (otCoapHeaderGetType(p_header) != OT_COAP_TYPE_CONFIRMABLE &&
            otCoapHeaderGetType(p_header) != OT_COAP_TYPE_NON_CONFIRMABLE)
        {
            break;
        }

        if (otCoapHeaderGetCode(p_header) != OT_COAP_CODE_PUT)
        {
            break;
        }

        if (otMessageRead(p_message, otMessageGetOffset(p_message), &command, 1) != 1)
        {
            NRF_LOG_INFO("light handler - missing command\r\n");
        }

        switch (command)
        {
            case LIGHT_ON:
                light_on();
                break;

            case LIGHT_OFF:
                light_off();
                break;

            case LIGHT_TOGGLE:
                light_toggle();
                break;

            default:
                break;
        }

        if (otCoapHeaderGetType(p_header) == OT_COAP_TYPE_CONFIRMABLE)
        {
            light_response_send(p_context, p_header, p_message_info);
        }

    } while (false);
}

static otError provisioning_response_send(void                * p_context,
                                          otCoapHeader        * p_request_header,
                                          uint8_t               device_type,
                                          const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NO_BUFS;
    otCoapHeader header;
    otMessage  * p_response;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));
        otCoapHeaderSetPayloadMarker(&header);

        p_response = otCoapNewMessage(p_context, &header);
        if (p_response == NULL)
        {
            break;
        }

        error = otMessageAppend(p_response, &device_type, 1);
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otMessageAppend(p_response, otThreadGetMeshLocalEid(p_context), sizeof(otIp6Address));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if (error != OT_ERROR_NONE && p_response != NULL)
    {
        otMessageFree(p_response);
    }

    return error;
}

static void provisioning_request_handler(void                * p_context,
                                         otCoapHeader        * p_header,
                                         otMessage           * p_message,
                                         const otMessageInfo * p_message_info)
{
    (void)p_message;
    otMessageInfo message_info;

    if (!m_app.enable_provisioning)
    {
        return;
    }

    if (otCoapHeaderGetType(p_header) == OT_COAP_TYPE_NON_CONFIRMABLE &&
        otCoapHeaderGetCode(p_header) == OT_COAP_CODE_GET)
    {
        message_info = *p_message_info;
        memset(&message_info.mSockAddr, 0, sizeof(message_info.mSockAddr));
        if (provisioning_response_send(p_context, p_header, DEVICE_TYPE_LIGHT, &message_info) ==
            OT_ERROR_NONE)
        {
            provisioning_disable(p_context);
        }
    }
}

/***************************************************************************************************
 * @section State change handling
 **************************************************************************************************/

static void role_change_handler(void * p_context, otDeviceRole role)
{
    switch(role)
    {
        case OT_DEVICE_ROLE_CHILD:
        case OT_DEVICE_ROLE_ROUTER:
        case OT_DEVICE_ROLE_LEADER:
            break;

        case OT_DEVICE_ROLE_DISABLED:
        case OT_DEVICE_ROLE_DETACHED:
        default:
            provisioning_disable(p_context);
            break;
    }
}

static void state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        role_change_handler(p_context, otThreadGetDeviceRole(p_context));
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}

/***************************************************************************************************
 * @section Buttons
 **************************************************************************************************/

static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            break;

        case BSP_EVENT_KEY_1:
            break;

        case BSP_EVENT_KEY_2:
            break;

        case BSP_EVENT_KEY_3:
            provisioning_enable(m_app.p_ot_instance);
            break;

        default:
            return;
    }
}

/***************************************************************************************************
 * @section Timers
 **************************************************************************************************/

static void provisioning_timer_handler(void * p_context)
{
    provisioning_disable(p_context);
}

static void led_timer_handler(void * p_context)
{
    (void)p_context;

    if (m_app.enable_provisioning)
    {
        LEDS_INVERT(BSP_LED_2_MASK);
    }
    else
    {
        LEDS_OFF(BSP_LED_2_MASK);
    }
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

static void thread_init(void)
{
    otInstance * p_instance;

    PlatformInit(0, NULL);

    p_instance = otInstanceInit();
    assert(p_instance);

    otCliUartInit(p_instance);

    NRF_LOG_INFO("Thread version: %s\r\n", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s\r\n", (uint32_t)otThreadGetNetworkName(p_instance));

    assert(otSetStateChangedCallback(p_instance, &state_changed_callback, p_instance) == OT_ERROR_NONE);

    if (!otDatasetIsCommissioned(p_instance))
    {
        assert(otLinkSetChannel(p_instance, THREAD_CHANNEL) == OT_ERROR_NONE);
        assert(otLinkSetPanId(p_instance, THREAD_PANID) == OT_ERROR_NONE);
    }

    assert(otIp6SetEnabled(p_instance, true) == OT_ERROR_NONE);
    assert(otThreadSetEnabled(p_instance, true) == OT_ERROR_NONE);

    m_app.p_ot_instance = p_instance;
}

static void coap_init()
{
    m_app.light_resource.mContext = m_app.p_ot_instance;
    m_app.provisioning_resource.mContext = m_app.p_ot_instance;

    assert(otCoapStart(m_app.p_ot_instance, OT_DEFAULT_COAP_PORT) == OT_ERROR_NONE);
    assert(otCoapAddResource(m_app.p_ot_instance, &m_app.light_resource) == OT_ERROR_NONE);
    assert(otCoapAddResource(m_app.p_ot_instance, &m_app.provisioning_resource) == OT_ERROR_NONE);
}

static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    app_timer_create(&m_provisioning_timer, APP_TIMER_MODE_SINGLE_SHOT, provisioning_timer_handler);
    app_timer_create(&m_led_timer, APP_TIMER_MODE_REPEATED, led_timer_handler);
}

static void thread_bsp_init(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(m_app.p_ot_instance);
    APP_ERROR_CHECK(err_code);
}

static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    app_timer_start(m_led_timer, APP_TIMER_TICKS(LED_INTERVAL), NULL);
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
    NRF_LOG_INIT(NULL);

    thread_init();
    coap_init();

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
